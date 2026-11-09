"""Operator-reported physical measurements; geometry tests never operate hardware."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

import main
from navigation.geometry import Arena
from navigation.timing import load_timing
from vision.cleaning_config import CleaningHardware
from vision.cleaning_geometry import footprint
from vision.cleaning_planner import CleaningPlanner
from test_cleaning_planner import cal,hardware,rect,target
from test_navigation import rig


def measured_hardware():return json.loads(Path('config/cleaning_hardware.json').read_text())


@pytest.mark.parametrize('direction,factor,amount,duration',[
    ('F',62.5,16,1000),('B',1000/18.5,18.5,1000),('L',4000/90,90,4000),('R',3000/90,90,3000)])
def test_direction_specific_physical_time_estimates(direction,factor,amount,duration):
    timing=load_timing('config/movement_timing.json')
    assert timing.data['mode']=='PHYSICAL'
    assert timing.magnitude_rate(direction)==pytest.approx(factor)
    assert round(amount*timing.magnitude_rate(direction))==duration
    row=next(s for s in timing.data['operator_estimates'] if s['direction']==direction)
    assert row['verified'] is True and row['approximate'] is True and 'Operator real-world' in row['source']


def test_approximate_rates_keep_observed_turn_signs_without_invented_stopping_bounds():
    timing=load_timing('config/movement_timing.json')
    assert timing.rate('F')==pytest.approx((62.5,1))
    assert timing.rate('B')==pytest.approx((1000/18.5,-1))
    assert timing.data.get('stopping_samples',[])==[]
    with pytest.raises(ValueError,match='stopping'):timing.stopping('F',.15)
    assert timing.rate('L')==pytest.approx((4000/90,-1))
    assert timing.rate('R')==pytest.approx((3000/90,1))
    assert [(r['direction'],r['duration_ms']) for r in timing.data['references']]==[('R',1500),('R',3000)]
    # Contradictory future data must block rather than silently replace observation.
    timing.add(direction='R',duration_ms=3000,observed_change=-90,verified=True,source='synthetic signed-pose regression')
    with pytest.raises(ValueError,match='Conflicting'):timing.rate('R')
    assert timing.rate('L')==pytest.approx((4000/90,-1))


def test_measured_straight_rates_remain_closed_loop_and_bounded():
    nav,plant,transport,plan=rig()
    nav.timing=load_timing('config/movement_timing.json')
    plant.actual_rates.update(F=62.5,B=1000/18.5)
    nav.axis('X',70)
    assert abs(plant.x-70)<=2.5
    assert all(int(line.split(',')[2])<=500 for line in transport.lines if line.startswith('MOVE,'))
    assert any(e.get('fresh_pose') for e in nav.events)
    assert not any(e.get('movement_mode')=='LONG_MONITORED' for e in nav.events)


def test_reported_side_geometry_keeps_effective_contact_and_axes_unknown():
    data=measured_hardware();s=data['wall_sponge'];m=s['measurements']
    assert (data['body']['chassis_length_cm'],data['body']['chassis_width_cm'])==(26,15.2)
    assert s['side']=='LEFT' and s['installed'] and s['lateral_protrusion_cm']==5
    assert m['dimensions_cm']==[8,13.5] and m['front_edge_setback_cm']==6
    assert data['body']['chassis_length_cm']/2-m['front_edge_setback_cm']==7
    assert m['lowest_edge_height_cm']==s['min_contact_height_cm']==5 and m['rotation_about_center'] is True
    assert m['rotation_axis'] is None and m['dimension_axes'] is None
    assert s['offset_forward_cm'] is None and s['contact_width_cm'] is None and s['max_contact_height_cm'] is None
    assert s['diameter_cm'] is None and not m['rotation_envelope_verified'] and not m['effective_contact_verified']


@pytest.mark.parametrize('side,heading',[('LEFT',180),('RIGHT',0)])
def test_wall_parallel_reference_and_overhang_side(cal,side,heading):
    data=measured_hardware();data['wall_sponge']['side']=side
    model=CleaningHardware(data);planner=CleaningPlanner(cal,model,mode='PHYSICAL')
    plan=planner.plan(target(planner,'WALL',[rect(40,6,42,8)]),timestamp=1001)
    assert plan.wall_contact_model['required_parallel_heading_deg']==heading
    assert plan.wall_contact_model['uncompressed_centerline_reference_cm']==pytest.approx(12.6)
    assert plan.reachability=='UNKNOWN' and plan.configuration_status=='CONFIGURATION_REQUIRED'
    assert not plan.coverage_segments and plan.approach_pose is None
    assert model.side_collision_envelope()['offset_left_cm']==(2.5 if side=='LEFT' else -2.5)


def test_left_overhang_collision_is_not_just_a_chassis_point_or_floor_contact():
    box=CleaningHardware(measured_hardware()).side_collision_envelope()
    assert box['scope']=='STATIC_OBSERVED_PANEL_ONLY'
    assert box['length_cm']==26 and box['width_cm']==pytest.approx(20.2)
    body=Arena(margin=.1);with_side=Arena(margin=.1,side_tool=box)
    wall_pose={'x_cm':40,'y_cm':10,'heading_deg':180}
    assert body.safe(wall_pose) and not with_side.safe(wall_pose)
    assert with_side.safe({**wall_pose,'y_cm':13})
    assert with_side.safe({**wall_pose,'heading_deg':0})  # LEFT is now away from top wall.
    corners=with_side.corners({**wall_pose,'y_cm':13})[-1]
    assert min(p[1] for p in corners)==pytest.approx(.4)


def test_body_rotation_checks_static_carried_overhang_without_inventing_sponge_rotation():
    box=CleaningHardware(measured_hardware()).side_collision_envelope()
    start={'x_cm':40,'y_cm':16,'heading_deg':180};end={**start,'heading_deg':90}
    Arena(margin=.1).check_segment(start,end,turn_change=-90)
    with pytest.raises(ValueError,match='swept area'):
        Arena(margin=.1,side_tool=box).check_segment(start,end,turn_change=-90)


def test_static_overhang_sweep_detects_obstacles_above_the_floor_tool():
    box=CleaningHardware(measured_hardware()).side_collision_envelope()
    kwargs=dict(margin=.1,obstacles=[[55,40,56,41]])
    start={'x_cm':40,'y_cm':30,'heading_deg':0};end={**start,'x_cm':70}
    Arena(**kwargs).check_segment(start,end)
    arena=Arena(**kwargs,side_tool=box)
    assert arena.safe(start) and arena.safe(end)
    with pytest.raises(ValueError,match='swept area'):arena.check_segment(start,end)


def test_unknown_floor_tool_stays_configuration_required(cal):
    data=measured_hardware();original=deepcopy(data['floor_sponge'])
    assert all(v is None for v in original.values())
    planner=CleaningPlanner(cal,data,mode='PHYSICAL')
    plan=planner.plan(target(planner,'FLOOR'),timestamp=1001)
    assert plan.reachability=='UNKNOWN' and plan.configuration_status=='CONFIGURATION_REQUIRED'
    assert any('floor_sponge.contact' in reason for reason in plan.reasons)
    assert not plan.coverage_segments and not plan.actionable and plan.approach_pose is None
    assert data['floor_sponge']==original  # Elevated side sponge never substitutes for underside contact.


def test_separately_configured_underside_tool_is_preserved_and_independent(cal,hardware):
    # Explicit synthetic effective floor fixture; never saved as physical measurements.
    data=deepcopy(hardware);data['geometry_source']='PHYSICAL';data['measurements_validated']=True
    data['body'].update(chassis_length_cm=26,chassis_width_cm=15.2)
    data['wall_sponge']=measured_hardware()['wall_sponge']
    original=deepcopy(data['floor_sponge'])
    planner=CleaningPlanner(cal,data,mode='PHYSICAL')
    floor=planner.plan(target(planner,'FLOOR',[rect(40,25,42,27)]),timestamp=1001)
    assert floor.reachability=='FULLY_REACHABLE' and floor.configuration_status=='CONFIGURED'
    assert data['floor_sponge']==original
    contacts=[s for s in floor.coverage_segments if s['segment_type']=='CONTACT_CLEAN']
    assert contacts and all(s['required_tool']=='FRONT_FLOOR_SPONGE' for s in contacts)
    assert all(s['coverage_metadata']['contact_model']['length']==original['contact_length_cm'] for s in contacts)
    wall=planner.plan(target(planner,'WALL',[rect(40,6,42,8)]),timestamp=1001)
    assert wall.configuration_status=='CONFIGURATION_REQUIRED' and not wall.coverage_segments


def test_wall_lower_blind_zone_is_retained_even_when_full_contact_is_unknown(cal):
    data=measured_hardware();planner=CleaningPlanner(cal,data,mode='PHYSICAL')
    plan=planner.plan(target(planner,'WALL',[rect(40,1,42,4)]),timestamp=1001)
    assert plan.metrics['lower_unreachable_area_cm2']==pytest.approx(6)
    assert plan.metrics['contact_reachable_area_cm2'] is None and not plan.coverage_segments
    data['wall_sponge']['min_contact_height_cm']=4
    with pytest.raises(ValueError,match='below the measured'):CleaningHardware(data)


def test_nozzle_measurements_do_not_become_exact_offsets_or_coverage():
    data=measured_hardware();obs=data['nozzle_observation']
    assert obs['mount_location']=='NEAR_FRONT_LEFT' and obs['height_cm']==16.5
    assert obs['landing_ahead_cm']==10 and obs['approximate'] is True
    assert obs['landing_reference']=='UNSPECIFIED_POINT_ON_ROBOT'
    assert data['floor_nozzle']['mount_height_cm']==16.5
    for name in ('floor_nozzle','wall_nozzle'):
        assert data[name]['mount_offset_forward_cm'] is None and data[name]['mount_offset_lateral_cm'] is None
        assert data[name]['coverage_calibrated'] is not True
    assert data['floor_nozzle']['offset_forward_cm'] is None and data['floor_nozzle']['offset_lateral_cm'] is None
    assert data['wall_nozzle']['impact_offset_forward_cm'] is None


def test_explicit_calibrated_nozzle_offsets_remain_authoritative(cal,hardware):
    data=deepcopy(hardware);data['nozzle_observation']=measured_hardware()['nozzle_observation']
    one=CleaningPlanner(cal,data,mode='SIMULATION')
    first=one.plan(target(one,'FLOOR'),timestamp=1001)
    data['nozzle_observation'].update(landing_ahead_cm=20,height_cm=30)
    two=CleaningPlanner(cal,data,mode='SIMULATION')
    second=two.plan(target(two,'FLOOR'),timestamp=1001)
    assert first.approach_pose==second.approach_pose and first.metrics==second.metrics
    assert data['floor_nozzle']==hardware['floor_nozzle']


def test_physical_navigation_configuration_failure_opens_no_camera_or_robot(cal,tmp_path,monkeypatch,caplog):
    import navigation.cli as cli
    calibration=tmp_path/'cal.json';cal.save(calibration)
    monkeypatch.setattr(cli,'selected_target',lambda *a:({},dict(surface='FLOOR')))
    def forbidden(*a,**kw):pytest.fail('Incomplete geometry must not open hardware')
    monkeypatch.setattr(cli,'CameraManager',forbidden);monkeypatch.setattr(cli,'RobotClient',forbidden)
    assert main.main(['navigate-task','--host','192.0.2.1','--targets','unused.json','--target-id','1',
        '--calibration',str(calibration),'--hardware','config/cleaning_hardware.json','--timing','config/movement_timing.json',
        '--width','800','--height','600','--output-dir',str(tmp_path/'run')])==1
    assert 'CONFIGURATION_REQUIRED' in caplog.text
