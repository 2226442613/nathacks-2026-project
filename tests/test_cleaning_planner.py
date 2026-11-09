"""Module 05 contact geometry only; every plan must remain non-actionable."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from vision import GroundCalibration,DualPlaneCalibration,CleaningHardware,CleaningPlanner
from vision.cleaning_geometry import footprint,contained,area,covered_area,rectangle_set,validate_rings
from vision.localization import utc_timestamp


@pytest.fixture
def cal():
    floor=GroundCalibration([[20,200],[620,200],[620,500],[20,500]],120,60,800,600)
    return DualPlaneCalibration.from_floor_and_wall_top(floor,[[20,20],[620,20]],120,36,'top')


@pytest.fixture
def hardware(): return json.loads(Path('config/cleaning_demo.json').read_text())


@pytest.fixture
def planner(cal,hardware): return CleaningPlanner(cal,hardware,mode='SIMULATION')


def rect(x0,y0,x1,y1,kind='outer'):
    return {'kind':kind,'points':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]}


def target(planner,surface='FLOOR',rings=None,**kwargs):
    record={'target_id':7,'surface':surface,'calibration_id':planner.calibration_id,
        'coordinate_frame':{'frame_id':planner.calibration_id},'active':True,'timestamp':utc_timestamp(1000),
        'valid_until':utc_timestamp(1002),'localization_status':'UNVERIFIED','geometry_status':'UNVERIFIED',
        'geometry_cm':{'contours':rings or [rect(40,25,50,30)],'trust_state':'UNVERIFIED'}}
    record.update(kwargs);return record


def plan_floor(planner,rings=None,**kwargs): return planner.plan(target(planner,rings=rings,**kwargs),timestamp=1001)


def plan_wall(planner,low,high,x0=40,x1=60,**kwargs):
    return planner.plan(target(planner,'WALL',[rect(x0,low,x1,high)],**kwargs),timestamp=1001)


def contact_segments(plan): return [p for p in plan.coverage_segments if p['segment_type']=='CONTACT_CLEAN']


def test_small_floor_target_has_actual_contact_stroke(planner):
    p=plan_floor(planner,[rect(40,25,42,27)])
    assert p.reachability=='FULLY_REACHABLE' and p.metrics['estimated_cleanable_fraction']==pytest.approx(1)
    assert len(contact_segments(p))==1 and p.approach_pose
    stroke=contact_segments(p)[0]
    assert stroke['start_pose']!=stroke['end_pose']
    assert p.approach_pose['desired_contact_geometry']['contact_stroke_start_pose']==stroke['start_pose']
    assert p.approach_pose['x_cm']!=41 or p.approach_pose['y_cm']!=26


def test_wide_target_requires_multiple_passes(planner):
    p=plan_floor(planner,[rect(30,15,60,45)])
    assert p.reachability=='FULLY_REACHABLE' and len(contact_segments(p))>1
    assert all(s['waypoints'] for s in contact_segments(p))


@pytest.mark.parametrize('rings',[ [rect(0,20,5,30)],[rect(115,20,120,30)],
    [rect(40,0,50,5)],[rect(40,55,50,60)] ])
def test_targets_near_every_boundary_have_contained_chassis(planner,rings):
    p=plan_floor(planner,rings)
    assert p.reachability in ('PARTIALLY_REACHABLE','UNREACHABLE','FULLY_REACHABLE')
    for segment in p.coverage_segments:
        for pose in segment['waypoints']:
            assert planner.safe_pose(pose)
    if p.approach_pose: assert planner.safe_pose(p.approach_pose)


def test_front_sponge_offset_used_in_swept_region(planner):
    p=plan_floor(planner,[rect(40,25,42,27)])
    segment=contact_segments(p)[0];start=segment['start_pose'];end=segment['end_pose']
    assert start['heading_deg']==end['heading_deg']
    model=segment['coverage_metadata']['contact_model']
    assert model['offset_forward_cm']==4
    actual=segment['coverage_metadata']['swept_region']
    assert covered_area([rect(40,25,42,27)],[actual])==pytest.approx(4)


@pytest.mark.parametrize('heading',[0,45,90,135,180,270,359])
def test_oriented_chassis_matches_module04_axes_and_containment(heading):
    pose={'x_cm':40,'y_cm':30,'heading_deg':heading}
    polygon=footprint(pose,12,8)
    a,b=polygon[1]-polygon[0],polygon[2]-polygon[1]
    assert abs(a[0]*b[1]-a[1]*b[0])==pytest.approx(96)
    assert contained(polygon,120,60)


@pytest.mark.parametrize('kind',['irregular','hole','disconnected'])
def test_complex_target_geometry_not_replaced_by_bounds(planner,kind):
    if kind=='irregular': rings=[{'kind':'outer','points':[[30,20],[60,20],[60,25],[40,25],[40,40],[30,40]]}];expected=300
    elif kind=='hole': rings=[rect(30,20,60,40),rect(40,25,50,35,'hole')];expected=500
    else:rings=[rect(25,20,35,30),rect(65,35,75,45)];expected=200
    p=plan_floor(planner,rings)
    assert p.metrics['total_target_area_cm2']==pytest.approx(expected)
    assert p.metrics['combined_cleanable_area_cm2']==pytest.approx(expected)
    assert p.target_geometry==rings and p.reachability=='FULLY_REACHABLE'


def test_incomplete_floor_spray_is_not_full_cleaning(cal,hardware):
    hardware['floor_nozzle'].update(offset_forward_cm=100,offset_lateral_cm=0,coverage_length_cm=2,coverage_width_cm=2)
    planner=CleaningPlanner(cal,hardware,mode='SIMULATION')
    p=plan_floor(planner,[rect(40,25,50,30)])
    assert p.reachability!='FULLY_REACHABLE'
    assert p.metrics['contact_reachable_area_cm2']>p.metrics['spray_reachable_area_cm2']


@pytest.mark.parametrize('section,key',[('floor_sponge','contact_width_cm'),('floor_sponge','offset_forward_cm'),
    ('floor_nozzle','coverage_width_cm'),('floor_nozzle','fixed_orientation_deg'),('body','position_uncertainty_cm')])
def test_missing_hardware_is_unknown_not_unreachable(cal,hardware,section,key):
    hardware[section][key]=None;planner=CleaningPlanner(cal,hardware,mode='SIMULATION')
    p=plan_floor(planner)
    assert p.reachability=='UNKNOWN' and not p.coverage_segments and any(key in reason for reason in p.reasons)


@pytest.mark.parametrize('lo,hi,state,fraction',[(1,3,'UNREACHABLE',0),(3,8,'PARTIALLY_REACHABLE',.6),
    (6,11,'FULLY_REACHABLE',1),(11,16,'PARTIALLY_REACHABLE',.4),(17,20,'UNREACHABLE',0),
    (3,16,'PARTIALLY_REACHABLE',8/13)])
def test_wall_whole_height_geometry_and_both_blind_zones(planner,lo,hi,state,fraction):
    p=plan_wall(planner,lo,hi)
    assert p.reachability==state
    assert p.metrics['estimated_cleanable_fraction']==pytest.approx(fraction)
    assert p.metrics['contact_reachable_area_cm2']==pytest.approx(20*max(0,min(hi,13)-max(lo,5)))
    assert p.metrics['lower_unreachable_area_cm2']==pytest.approx(20*max(0,min(hi,5)-lo))
    assert p.metrics['upper_unreachable_area_cm2']==pytest.approx(20*max(0,hi-max(lo,13)))


def test_irregular_wall_area_clipping_is_not_height_fraction(planner):
    # Wide low region, narrow tall region: total140, contact40, fraction2/7.
    rings=[{'kind':'outer','points':[[40,1],[60,1],[60,5],[45,5],[45,17],[40,17]]}]
    p=planner.plan(target(planner,'WALL',rings),timestamp=1001)
    assert p.metrics['total_target_area_cm2']==pytest.approx(140)
    assert p.metrics['contact_reachable_area_cm2']==pytest.approx(40)
    assert p.metrics['estimated_cleanable_fraction']==pytest.approx(2/7)


def test_spray_only_wall_is_not_cleanable(planner):
    p=plan_wall(planner,17,20)
    assert p.metrics['spray_reachable_area_cm2']>0 and p.metrics['contact_reachable_area_cm2']==0
    assert p.reachability=='UNREACHABLE'


def test_sponge_only_wall_is_not_cleanable(cal,hardware):
    hardware['wall_nozzle'].update(impact_min_height_cm=16,impact_max_height_cm=20)
    planner=CleaningPlanner(cal,hardware,mode='SIMULATION');p=plan_wall(planner,6,11)
    assert p.metrics['contact_reachable_area_cm2']>0 and p.metrics['spray_reachable_area_cm2']==0
    assert p.reachability=='UNREACHABLE'


@pytest.mark.parametrize('section,key',[('wall_sponge','min_contact_height_cm'),('wall_sponge','max_contact_height_cm'),
    ('wall_nozzle','lift_min_cm'),('wall_nozzle','lift_max_cm'),('wall_nozzle','impact_max_height_cm')])
def test_wall_unknown_parameters_are_unknown(cal,hardware,section,key):
    hardware[section][key]=None;planner=CleaningPlanner(cal,hardware,mode='SIMULATION')
    p=plan_wall(planner,6,11);assert p.reachability=='UNKNOWN'


@pytest.mark.parametrize('side,heading',[('LEFT',180),('RIGHT',0)])
def test_side_orientation_and_horizontal_stroke(cal,hardware,side,heading):
    hardware['wall_sponge']['side']=side
    hardware['wall_nozzle']['mount_offset_lateral_cm']=2 if side=='LEFT' else -2
    planner=CleaningPlanner(cal,hardware,mode='SIMULATION');p=plan_wall(planner,6,11)
    assert p.reachability=='FULLY_REACHABLE' and p.approach_pose['heading_deg']==heading
    assert p.wall_contact_model['chassis_wall_gap_cm']==pytest.approx(2.8)
    for segment in contact_segments(p):
        a,b=segment['waypoints'];assert a['y_cm']==b['y_cm']
        assert a['heading_deg']==b['heading_deg']==heading and a['x_cm']!=b['x_cm']
        assert planner.safe_pose(a) and planner.safe_pose(b)
        assert segment['coverage_metadata']['vertical_sponge_motion'] is False


@pytest.mark.parametrize('edge',[(0,5),(115,120)])
def test_wall_edges_respect_chassis_containment(planner,edge):
    p=plan_wall(planner,6,11,*edge)
    assert p.reachability in ('PARTIALLY_REACHABLE','UNREACHABLE')
    assert all(planner.safe_pose(w) for s in p.coverage_segments for w in s['waypoints'])


def test_insufficient_protrusion_does_not_put_chassis_in_wall(cal,hardware):
    hardware['wall_sponge']['lateral_protrusion_cm']=.5
    planner=CleaningPlanner(cal,hardware,mode='SIMULATION');p=plan_wall(planner,6,11)
    assert p.reachability=='UNREACHABLE' and not p.approach_pose


def test_nozzle_lift_not_assumed_equal_impact_height(planner):
    p=plan_wall(planner,6,11)
    model=p.wall_contact_model['nozzle']
    assert model['impact_height_band_cm']==[0,20] and model['lift_limits_cm']==[0,15]
    assert model['specific_lift_angle_solution'] is None


@pytest.mark.parametrize('surface',['BOUNDARY','UNKNOWN'])
def test_ambiguous_surface_no_arbitrary_plan(planner,surface):
    p=planner.plan(target(planner,surface),timestamp=1001)
    assert p.reachability=='UNKNOWN' and not p.coverage_segments


def test_inactive_target_requires_explicit_override(planner):
    t=target(planner,active=False)
    assert planner.plan(t,timestamp=1001).reachability=='UNKNOWN'
    p=planner.plan(t,timestamp=1001,allow_inactive=True)
    assert p.reachability=='FULLY_REACHABLE' and p.target_validity['inactive_override']


def test_missing_physical_measurements_do_not_use_demo(cal):
    hardware=CleaningHardware(json.loads(Path('config/cleaning_hardware.json').read_text()))
    planner=CleaningPlanner(cal,hardware,mode='PHYSICAL');p=plan_floor(planner)
    assert p.reachability=='UNKNOWN' and not p.approach_pose and planner.margin is None
    assert hardware.data['body']['chassis_length_cm']==26
    assert hardware.data['body']['chassis_width_cm']==15.2
    assert p.configuration_status=='CONFIGURATION_REQUIRED'


def test_simulation_config_cannot_be_silently_physical(cal,hardware):
    with pytest.raises(ValueError): CleaningPlanner(cal,hardware,mode='PHYSICAL')


def test_complete_physical_configuration_still_never_authorizes(cal,hardware):
    hardware.update(geometry_source='PHYSICAL',measurements_validated=True)
    planner=CleaningPlanner(cal,hardware,mode='PHYSICAL');p=plan_floor(planner)
    assert p.reachability=='FULLY_REACHABLE' and p.execution_authorization=='NONE' and not p.actionable
    assert p.localization_status=='UNVERIFIED'


def test_existing_module03_event_prefers_full_geometry(planner):
    raw=target(planner)
    full=[rect(40,25,50,30),rect(43,26,47,29,'hole')]
    event={'record_key':'run:0:7','active':True,'target_message':raw,
           'localization':{'full_physical_contours_cm':full,'geometry_status':'UNVERIFIED'}}
    p=planner.plan(event,timestamp=1001)
    assert p.target_id==7 and p.metrics['total_target_area_cm2']==pytest.approx(38)


def test_robot_stale_missing_and_fresh_distinguished(planner):
    from vision import RobotTracker,RobotConfig
    from test_robot_tracking import marker_scene
    # Missing pixel/calibration pose remains explicitly unusable; use the actual v1
    # data contract for a current estimate in the same floor reference.
    pose=RobotTracker().update(marker_scene(),timestamp=1000).to_dict()
    pose.update(calibration_id=planner.calibration_id,position_cm={'x':40.,'y':30.},heading_deg=0.,localization_status='UNVERIFIED')
    for now,expected in [(1000.1,True),(1002,False)]:
        p=planner.plan(target(planner),pose,timestamp=now)
        assert p.robot_pose_validity['fresh']==expected and p.reachability=='FULLY_REACHABLE'
        assert not p.robot_pose_validity['navigation_to_approach_authorized']
    p=plan_floor(planner);assert not p.robot_pose_validity['provided']


def test_json_has_no_executable_actuator_fields(planner):
    p=plan_wall(planner,3,16);data=json.loads(json.dumps(p.to_dict(),allow_nan=False))
    assert data['schema_version']==1 and data['message_type']=='CLEANING_PLAN'
    assert data['execution_authorization']==data['physical_execution_authorization']=='NONE'
    assert not data['actionable'] and not data['route_to_approach_validated']
    assert all(s['declarative_only'] for s in data['coverage_segments'])
    types={s['segment_type'] for s in data['coverage_segments']}
    assert types=={'APPROACH','ALIGN','SPRAY','CONTACT_CLEAN','RETREAT','REQUEST_VERIFICATION'}
    assert all('command' not in s and 'speed' not in s and 'duration' not in s for s in data['coverage_segments'])


@pytest.mark.parametrize('damage',['nan','self_crossing','missing','wrong_calibration','invalid_status'])
def test_bad_geometry_or_provenance_is_not_planned(planner,damage):
    t=target(planner)
    if damage=='nan':t['geometry_cm']['contours'][0]['points'][0][0]=float('nan')
    elif damage=='self_crossing':t['geometry_cm']['contours']=[{'kind':'outer','points':[[30,20],[50,40],[30,40],[50,20]]}]
    elif damage=='missing':t['geometry_cm']={'bounds':{'min_x':40,'max_x':50}}
    elif damage=='wrong_calibration':t['calibration_id']='other'
    else:t['localization_status']='INVALID'
    p=planner.plan(t,timestamp=1001)
    assert p.reachability=='UNKNOWN' and not p.coverage_segments and p.reasons
    json.dumps(p.to_dict(),allow_nan=False)


@pytest.mark.parametrize('section,key,value',[('body','chassis_length_cm',0),('body','chassis_width_cm',-1),
    ('body','safety_margin_cm',-1),('floor_nozzle','fixed_orientation_deg',360),('wall_sponge','side','FRONT'),
    ('wall_sponge','min_contact_height_cm',20),('wall_nozzle','lift_min_cm',20),('wall_nozzle','rotation_max_deg',181),
    ('floor_sponge','contact_width_cm',float('nan')),('wall_sponge','installed',1)])
def test_hardware_invalid_values_rejected(hardware,section,key,value):
    hardware[section][key]=value
    with pytest.raises(ValueError): CleaningHardware(hardware)


def test_polygon_area_boolean_clipping_holes_and_overlapping_sweeps():
    rings=[rect(0,0,10,10),rect(2,2,8,8,'hole')]
    union=rectangle_set([[0,0,6,10],[4,0,10,10]])
    assert covered_area(rings,union)==pytest.approx(64)
    clipped=rectangle_set(union,[[0,0,5,10]],'intersection')
    assert covered_area(rings,clipped)==pytest.approx(32)
    remaining=rectangle_set([[0,0,10,10]],clipped,'difference')
    assert covered_area(rings,remaining)==pytest.approx(32)


def test_wall_diameter_does_not_define_effective_height_band(cal,hardware):
    hardware['wall_sponge']['diameter_cm']=30
    p=CleaningPlanner(cal,hardware,mode='SIMULATION')
    result=plan_wall(p,6,11)
    assert result.reachability=='FULLY_REACHABLE'
    assert result.wall_contact_model['min_contact_height_cm']==5 and result.wall_contact_model['max_contact_height_cm']==13


def test_module01_rejects_inconsistent_top_wall_x_definition(cal):
    # Respect Module 01's orientation invariant; never fabricate a flipped mapping.
    with pytest.raises(ValueError):
        DualPlaneCalibration.from_floor_and_wall_top(cal.floor,[[620,20],[20,20]],120,36,'top',True)


def test_non_top_wall_is_explicitly_unknown(hardware):
    floor=GroundCalibration([[200,200],[400,200],[400,400],[200,400]],120,80,600,600)
    from vision import WallCalibration
    wall=WallCalibration([[400,500],[200,500],[200,400],[400,400]],120,60,600,600)
    calibration=DualPlaneCalibration(floor,wall,'bottom',True)
    p=CleaningPlanner(calibration,hardware,mode='SIMULATION');result=plan_wall(p,6,11)
    assert result.reachability=='UNKNOWN' and any('top shared' in r for r in result.reasons)


def test_sponge_present_during_spray_and_retreat(cal,hardware):
    hardware['floor_sponge'].update(offset_forward_cm=14)
    p=CleaningPlanner(cal,hardware,mode='SIMULATION');result=plan_floor(p,[rect(40,25,50,30)])
    assert result.coverage_segments
    for segment in result.coverage_segments:
        for pose in segment['waypoints']: assert p.safe_floor_tools(pose)
        if segment['segment_type']=='RETREAT' and segment['end_pose']: assert p.safe_floor_tools(segment['end_pose'])


def test_stroke_limit_reports_unknown_not_a_fake_full_plan(cal,hardware):
    hardware['planning']['max_strokes']=1
    p=CleaningPlanner(cal,hardware,mode='SIMULATION');result=plan_floor(p)
    assert result.reachability=='UNKNOWN' and not result.coverage_segments
    assert any('stroke budget' in r for r in result.reasons)


def test_unsupported_nozzle_orientation_explicit(cal,hardware):
    hardware['floor_nozzle']['fixed_orientation_deg']=37
    p=CleaningPlanner(cal,hardware,mode='SIMULATION');result=plan_floor(p)
    assert result.reachability=='UNKNOWN' and any('Non-cardinal' in r for r in result.reasons)


def test_unknown_nozzle_coverage_not_assumed_calibrated(cal,hardware):
    hardware['wall_nozzle']['coverage_calibrated']=False
    p=CleaningPlanner(cal,hardware,mode='SIMULATION');result=plan_wall(p,3,16)
    assert result.reachability=='UNKNOWN' and result.metrics['estimated_cleanable_fraction'] is None
    assert result.metrics['lower_unreachable_area_cm2']==pytest.approx(40)


def test_uninstalled_tool_has_known_zero_combined_fraction(cal,hardware):
    hardware['floor_sponge']['installed']=False
    p=CleaningPlanner(cal,hardware,mode='SIMULATION');result=plan_floor(p)
    assert result.reachability=='UNREACHABLE' and result.metrics['estimated_cleanable_fraction']==0


def test_uncovered_geometry_area_matches_partial_fraction(planner):
    p=plan_wall(planner,3,16)
    assert area(p.unreachable_target_regions)==pytest.approx(p.metrics['total_target_area_cm2']-p.metrics['combined_cleanable_area_cm2'])


def test_concave_clip_with_disconnected_intersection_keeps_area():
    ring={'kind':'outer','points':[[0,0],[10,0],[10,10],[7,10],[7,3],[3,3],[3,10],[0,10]]}
    assert covered_area([ring],[[0,5,10,9]])==pytest.approx(24)


@pytest.mark.parametrize('change',[{'units':'mm'},{'target_id':None},{'target_id':True}])
def test_inconsistent_units_or_missing_id_rejected(planner,change):
    p=planner.plan(target(planner,**change),timestamp=1001)
    assert p.reachability=='UNKNOWN' and not p.coverage_segments


def test_nonfinite_robot_pose_never_enters_plan(planner):
    from vision import RobotTracker
    from test_robot_tracking import marker_scene
    robot=RobotTracker().update(marker_scene(),timestamp=1000).to_dict()
    robot.update(calibration_id=planner.calibration_id,localization_status='UNVERIFIED',position_cm={'x':float('nan'),'y':30},heading_deg=0)
    p=planner.plan(target(planner),robot,timestamp=1000.1)
    assert not p.robot_pose_validity['current_pose_usable_for_offline_reference']
    json.dumps(p.to_dict(),allow_nan=False)


def test_even_verified_target_does_not_verify_missing_robot(planner):
    p=planner.plan(target(planner,localization_status='VERIFIED',geometry_status='VERIFIED'),timestamp=1001)
    assert p.localization_trust['target']=='VERIFIED' and p.localization_status=='UNVERIFIED'
    assert not p.actionable


def test_future_target_timestamp_not_current(planner):
    p=planner.plan(target(planner),timestamp=999)
    assert p.target_validity['fresh'] is False


def test_custom_tool_polygon_never_silently_replaced(hardware):
    hardware['floor_sponge']['footprint_polygon']=[[0,0],[1,0],[0,1]]
    with pytest.raises(ValueError,match='unsupported'):CleaningHardware(hardware)


def test_plan_schema_contract_and_config_snapshot(planner):
    schema=json.loads(Path('config/cleaning_plan_v1.schema.json').read_text())
    hardware_schema=json.loads(Path('config/cleaning_hardware.schema.json').read_text())
    plan=plan_floor(planner).to_dict()
    assert all(key in plan for key in schema['required'])
    assert all(key in plan['hardware_configuration'] for key in hardware_schema['required'])
    assert CleaningHardware(plan['hardware_configuration']).identity==plan['configuration_id']
    for segment in plan['coverage_segments']:
        assert segment['segment_type'] in schema['properties']['coverage_segments']['items']['properties']['segment_type']['enum']
        assert segment['declarative_only'] is True


def test_module03_registration_fallback_does_not_block_geometric_demo_plan(planner):
    t=target(planner,camera_registration={'status':'MOVED','current_to_calibration':None,'fallback_active':True,
        'registration_warning':'Registration exceeds safe small-motion limit'},quality_warnings=['UNVERIFIED current-frame fallback'])
    p=planner.plan(t,timestamp=1001)
    assert p.reachability=='FULLY_REACHABLE' and p.localization_status=='UNVERIFIED'
    assert p.source_provenance['camera_registration']['fallback_active']
    assert not p.actionable


def test_wall_target_height_is_not_used_as_floor_goal_y(planner):
    a=plan_wall(planner,6,7);b=plan_wall(planner,11,12)
    assert a.approach_pose['y_cm']==b.approach_pose['y_cm']
    assert a.wall_contact_model['fixed_height'] and b.wall_contact_model['fixed_height']
