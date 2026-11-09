"""Autonomous queue regressions; all motion feedback and transport are simulated."""
from dataclasses import replace
import io
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from navigation.automatic import SceneScanner,run_queue
from navigation.controller import Navigator,NavigationConfig,NavigationFailure
from navigation.geometry import Arena,front_bumper_metrics,front_parking_candidates
from navigation.simulation import SimClock,SimPlant,SimTransport
from robot_comm import WireLogger
from vision import BackgroundReference,load_calibration
from vision.live_test import LiveVision
from vision.robot_cli import build_robot_tracker
from test_automatic_cleaning import geometry_scanner
from test_live_vision import live_inputs
from test_mvp_association import physical_rig
from test_navigation import rig


def test_actual_triangle_edge_is_not_its_bounding_box_and_parking_solves_true_gap():
    target=dict(surface='FLOOR',geometry_cm=dict(bounds=dict(min_x=80,min_y=50,max_x=100,max_y=90),
        contours=[dict(kind='outer',points=[[80,50],[100,50],[100,90]])]))
    pose=dict(x_cm=62.,y_cm=70.,heading_deg=0.)
    expected=(2*75-62.4-110)/math.sqrt(5)
    assert front_bumper_metrics(target,pose,26,15.2)['clearance_cm']==pytest.approx(abs(expected))
    assert abs(expected)>10  # Old bbox method falsely reported 5 cm here.
    goal=front_parking_candidates(target,Arena(width=200,depth=150))[0]
    assert goal['x_cm']==pytest.approx(62+(22.4-5*math.sqrt(5))/2)
    assert front_bumper_metrics(target,goal,26,15.2)['clearance_cm']==pytest.approx(5,abs=1e-6)


def test_bumper_edge_intersection_and_actual_body_overlap_are_zero():
    target=dict(geometry_cm=dict(contours=[dict(kind='outer',points=[[80,50],[100,50],[100,90]])]))
    assert front_bumper_metrics(target,dict(x_cm=77,y_cm=60,heading_deg=0),26,15.2)['clearance_cm']==0.


def test_confirmation_waits_past_transient_startup_misses_without_changing_thresholds():
    scanner,frame,_=geometry_scanner();blank=scanner.detector._reference.copy()
    frames=iter([blank,frame,blank,frame,frame,frame])
    scanner.frame_source=lambda:(next(frames),None)
    assert any(r['category']=='RED_SPILL' for r in scanner.scan())
    assert scanner.last_scan['frames']==6 and scanner.tracker.config.confirmation_frames==3


def test_empty_confirmation_window_is_bounded_and_does_not_emit_stale_targets():
    scanner,frame,_=geometry_scanner();assert scanner.scan()
    blank=scanner.detector._reference.copy();scanner.frame_source=lambda:(blank,None)
    assert scanner.scan()==[] and scanner.last_scan['frames']==12 and scanner.last_scan['confirmed']==0


@pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')
def test_auto_and_test_vision_share_mask_confirmation_and_current_frame_coordinates(live_inputs):
    frame,args=live_inputs;options=main.build_parser().parse_args(args)
    view=LiveVision(options)
    robot=build_robot_tracker(options.robot_settings,options.calibration,marker_plane_path=options.marker_plane)
    def source():
        pose=robot.update(frame)
        return frame,robot.exclusion_mask(pose,frame.shape[1::-1])
    scanner=SceneScanner(BackgroundReference.load(options.background,confirmed_clean=True),view.calibration,
        view.localizer.calibration_id,options.settings,source,logger=WireLogger(stream=io.StringIO()))
    actual=scanner.scan()
    for _ in range(3):_,summary=view.process(frame)
    expected=[r.target_message() for r in view.records]
    assert {t['category'] for t in actual}=={'RED_SPILL','NON_RED_SPILL'}
    assert [(r['category'],r['position_cm'],r['geometry_cm']) for r in actual]==[(r['category'],r['position_cm'],r['geometry_cm']) for r in expected]
    assert scanner.registration.to_dict()==view.registration.to_dict()
    assert all(s['confirmed'] for s in summary['spills'])


def nav_at(start):
    calibration,clock,plant,transport,_,timing,hardware,arena,adapter=physical_rig()
    plant.x,plant.y,plant.heading=start;plant.move_gain=plant.turn_gain=1.
    nav=Navigator(plant,transport,timing,arena,NavigationConfig(),clock=clock,sleep=clock.sleep)
    nav.simple_navigation=True
    return nav,plant,transport


@pytest.mark.parametrize('start',[(16.,18.,0.),(20.,17.,0.)])
def test_safe_escape_direct_or_one_corner_route_instead_of_false_y_first_rejection(start):
    nav,plant,transport=nav_at(start)
    goal=dict(x_cm=48.23578934628188,y_cm=38.46276029011315,heading_deg=0.)
    with pytest.raises(ValueError):nav._preflight_route(plant.observe().geometry(),goal,('Y','X'))
    route=nav.plan_simple_route(plant.observe().geometry(),goal)
    assert len(route['waypoints'])<=2
    nav.route(goal)
    assert math.hypot(plant.x-goal['x_cm'],plant.y-goal['y_cm'])<=2.5
    assert abs((plant.heading+180)%360-180)<=7
    assert all(int(line.split(',')[2])<=500 for line in transport.lines if line.startswith('MOVE,'))


def test_straight_ahead_and_reverse_avoid_unnecessary_rotations():
    nav,plant,transport=nav_at((40.,30.,0.))
    for x,direction in [(55.,'F'),(35.,'B')]:
        goal=dict(x_cm=x,y_cm=30.,heading_deg=0.)
        plan=nav.plan_simple_route(plant.observe().geometry(),goal)
        assert plan['turn_degrees']==0 and len(plan['waypoints'])==1
        before=len(transport.lines);nav.route(goal)
        moves=[line for line in transport.lines[before:] if line.startswith('MOVE,')]
        assert moves and all(line.startswith('MOVE,'+direction+',') for line in moves)


@pytest.mark.parametrize('gain',[.65,1.35])
def test_simple_route_under_and_overshoot_correct_using_fresh_feedback(gain):
    nav,plant,transport=nav_at((32.,29.,0.));plant.move_gain=gain
    goal=dict(x_cm=70.,y_cm=40.,heading_deg=0.)
    nav.route(goal)
    assert math.hypot(plant.x-70,plant.y-40)<=2.5
    assert all(e['fresh_pose']!=e['previous_pose'] for e in nav.events if 'fresh_pose' in e)


def test_simple_route_obstacle_wall_and_unverified_pose_still_stop_without_cleaning():
    nav,plant,transport=nav_at((32.,29.,0.))
    nav.arena=Arena(obstacles=[(55,0,56,68.4)],side_tool=nav.arena.side_tool)
    with pytest.raises(ValueError,match='No safe simple'):nav.route(dict(x_cm=85,y_cm=29,heading_deg=0))
    assert not transport.lines
    nav,plant,transport=nav_at((32.,29.,0.));observe=plant.observe
    plant.observe=lambda:replace(observe(),verified=False)
    with pytest.raises(NavigationFailure,match='VERIFIED'):nav.route(dict(x_cm=60,y_cm=30,heading_deg=0))
    assert not transport.lines


def test_default_auto_processes_red_then_nonred_without_forced_home_and_finishes_empty(tmp_path):
    out=tmp_path/'direct'
    assert main.main(['auto-clean','--output-dir',str(out)])==0
    result=json.loads((out/'summary.json').read_text())
    assert result['state']=='READY' and not result['return_home']
    assert [r['target']['category'] for r in result['completed']]==['RED_SPILL','NON_RED_SPILL']
    assert all(r['result']['cleaning_done_received'] and not r['result']['home_verified'] for r in result['completed'])
    events=[json.loads(line) for line in (out/'progress.jsonl').read_text().splitlines()]
    assert not any(e.get('state','').startswith('RETURNING') for e in events)
    assert not any(e['home_verified'] for e in events if e['event_type']=='NAVIGATION_PROGRESS')
    log=(out/'wire.log').read_text()
    assert log.count('[TX] ARRIVED')==2 and log.count('[RX] DONE,CLEANING')==2
    first=log.index('[TX] TARGET,GROUND,BLOOD');second=log.index('[TX] TARGET,GROUND,OTHER')
    assert first<log.index('[RX] DONE,CLEANING')<second
    assert result['last_scan']['candidates']==0


def test_explicit_batch_cap_is_not_reported_as_all_targets_finished(tmp_path):
    out=tmp_path/'limited'
    assert main.main(['auto-clean','--max-tasks','1','--output-dir',str(out)])==0
    result=json.loads((out/'summary.json').read_text())
    assert result['state']=='BATCH_LIMIT' and result['batch_limit_reached'] and len(result['completed'])==1


@pytest.mark.parametrize('fault',['lost','stale','done_missing','stop_ack_missing','cleaning_fault'])
def test_full_simple_task_faults_never_clean_early_replay_or_start_next_task(fault):
    nav,plant,transport,plan=rig();nav.timing.data['stopping_samples']=[]
    if fault=='lost':plant.lost=True
    elif fault=='stale':plant.stale=True
    elif fault=='done_missing':transport.no_done=True
    elif fault=='stop_ack_missing':
        transport.missing_stop_ack=True;plant.lost=True
    else:transport.clean_fault=True
    result=nav.run(plan,'RED_SPILL',return_home=False,simple_navigation=True)
    assert result['state']!='READY' and not result['home_verified'] and nav.faulted
    assert transport.stopped and transport.lines.count('ARRIVED')<=(1 if fault=='cleaning_fault' else 0)
    assert not any(e['state'].startswith('RETURNING') for e in nav.events)


def test_actual_recorded_spill_geometry_reachable_with_simulated_feedback_only():
    path=Path('data/iphone_debug/frame_000200_input.png')
    if not path.exists() or not Path('config/camera_config.json').exists():pytest.skip('Optional recorded pixels are not distributed')
    calibration,clock,plant,transport,_,timing,_,arena,adapter=physical_rig()
    recorded=load_calibration('config/camera_config.json');image=cv2.imread(str(path))
    scanner=SceneScanner(BackgroundReference.load('data/iphone_reference.png',confirmed_clean=True),recorded,
        'recorded-diagnostic-not-physical-validation','config/spill_config.json',lambda:(image,None),
        logger=WireLogger(stream=io.StringIO()))
    events=[]
    result=run_queue(scanner,Mock(),adapter,plant,transport,timing,arena,NavigationConfig(),clock,clock.sleep,
        events.append,max_tasks=1,calibration=recorded,return_home=False)
    assert result['state']=='READY' and len(result['completed'])==1 and transport.cleaned
    arrival=next(e for e in events if e['event_type']=='BUMPER_CLEARANCE_VERIFIED')
    assert abs(arrival['clearance_cm']-5)<=2.5 and arrival['pose_verified']
    # 'verified' here is the simulated pose provider, never evidence of actual robot motion.


def test_zero_motion_has_no_fictitious_unsampled_sweep_but_real_motion_keeps_margin():
    arena=Arena();start=dict(x_cm=14.6,y_cm=30.,heading_deg=0.)
    assert arena.safe(start)
    arena.check_segment(start,start);arena.check_segment(start,start,turn_change=0.)
    with pytest.raises(ValueError,match='swept area'):
        arena.check_segment(start,{**start,'x_cm':14.4})


def test_failed_target_is_reconsidered_from_next_pose_and_nearby_distinct_targets_not_retired(monkeypatch):
    calibration,clock,plant,transport,_,timing,_,arena,adapter=physical_rig()
    def target(i,x,category='RED_SPILL'):
        return dict(target_id=i,category=category,surface='FLOOR',position_cm=dict(x=x,y=35.),
                    geometry_cm=dict(bounds=dict(min_x=x-.2,max_x=x+.2,min_y=34.8,max_y=35.2),
                                     contours=[dict(kind='outer',points=[[x-.2,34.8],[x+.2,34.8],[x+.2,35.2],[x-.2,35.2]])]))
    targets=[target(1,45.),target(2,95.,'NON_RED_SPILL'),target(3,47.)]
    original=Navigator.plan_simple_route
    def temporarily_blocked(self,start,goal):
        if not transport.cleaned and start['x_cm']<40 and goal['x_cm']<70:raise ValueError('Synthetic route blocked from initial pose')
        return original(self,start,goal)
    monkeypatch.setattr(Navigator,'plan_simple_route',temporarily_blocked)
    events=[]
    result=run_queue(SimpleNamespace(scan=lambda **kwargs:targets),Mock(),adapter,plant,transport,timing,arena,
        NavigationConfig(),clock,clock.sleep,events.append,max_tasks=4,calibration=calibration,return_home=False)
    assert result['state']=='READY' and not result['skipped']
    assert [t['target']['target_id'] for t in result['completed']]==[2,1,3]
    assert len(result['completed'])==3  # Two nearby red spills are distinct measured polygons.


def test_partial_failure_is_not_hidden_by_one_completed_task():
    calibration,clock,plant,transport,_,timing,_,arena,adapter=physical_rig()
    def target(i,x,y):
        return dict(target_id=i,surface='FLOOR',category='RED_SPILL' if i==1 else 'NON_RED_SPILL',position_cm=dict(x=x,y=y),
                    geometry_cm=dict(bounds=dict(min_x=x-.5,max_x=x+.5,min_y=y-.5,max_y=y+.5),
                        contours=[dict(kind='outer',points=[[x-.5,y-.5],[x+.5,y-.5],[x+.5,y+.5],[x-.5,y+.5]])]))
    targets=[target(1,3,3),target(2,90,35)]
    result=run_queue(SimpleNamespace(scan=lambda **kwargs:targets),Mock(),adapter,plant,transport,timing,arena,
        NavigationConfig(),clock,clock.sleep,lambda e:None,max_tasks=3,calibration=calibration,return_home=False)
    assert result['state']=='PARTIAL' and len(result['completed'])==len(result['skipped'])==1


def test_already_parked_robot_does_not_turn_around_to_correct_subtolerance_position():
    nav,plant,transport=nav_at((40,30,0))
    goal=dict(x_cm=39.,y_cm=30.,heading_deg=0.)
    assert nav.plan_simple_route(plant.observe().geometry(),goal)['waypoints']==[]
    nav.route(goal)
    assert not transport.lines


def test_post_settling_wait_skips_buffered_frames_instead_of_false_stale_pose_fault():
    nav,plant,transport,_=rig()
    class BufferedFrames:
        old_reads=0;fresh_reads=0
        def observe(self):
            self.old_reads+=1
            return replace(plant.observe(),observed_at=nav.clock()-1.)
        def observe_after(self,after):
            self.fresh_reads+=1;nav.sleep(.02);return plant.observe()
    source=BufferedFrames();nav.poses=source
    boundary=nav.clock();pose=nav.observe(after=boundary)
    assert pose.observed_at>boundary and source.old_reads==0 and source.fresh_reads==1
    assert not transport.lines


def test_color_change_after_done_does_not_automatically_respray_same_geometry():
    calibration,clock,plant,transport,_,timing,_,arena,adapter=physical_rig()
    t=dict(target_id=1,category='RED_SPILL',surface='FLOOR',position_cm=dict(x=80.,y=35.),
        geometry_cm=dict(bounds=dict(min_x=79.,min_y=34.,max_x=81.,max_y=36.),
            contours=[dict(kind='outer',points=[[79.,34.],[81.,34.],[81.,36.],[79.,36.]])]))
    def scan(**kwargs):return [{**t,'category':'NON_RED_SPILL','target_id':2}] if transport.cleaned else [t]
    result=run_queue(SimpleNamespace(scan=scan),Mock(),adapter,plant,transport,timing,arena,NavigationConfig(),
        clock,clock.sleep,lambda e:None,max_tasks=3,calibration=calibration,return_home=False)
    assert result['state']=='READY' and len(result['completed'])==1 and transport.lines.count('ARRIVED')==1


def test_complete_queue_uses_real_aruco_decoder_from_synthetic_camera_feedback(monkeypatch):
    """No pose estimates from timing: every controller pose is decoded from pixels."""
    from dataclasses import asdict
    from navigation.adapters import CameraPoses
    from vision import GroundCalibration,RobotConfig,RobotTracker
    from vision.robot_plane import MarkerPlaneMapping
    from vision.robot_tracking import generate_marker
    calibration,clock,plant,transport,_,timing,_,arena,adapter=physical_rig()
    floor=GroundCalibration([[0,0],[1279,0],[1279,719],[0,719]],121.6,68.4,1280,720)
    config=RobotConfig(marker_size_cm=9.,mounting_height_cm=16.6,marker_offset_forward_cm=2.,
                       marker_offset_left_cm=1.,body_length_cm=26.,body_width_cm=15.2)
    training=[dict(pixel=list(floor.world_to_pixel(x,y)),ground_cm=[x,y]) for x,y in [(10,10),(110,10),(110,60),(10,60)]]
    validation=[dict(pixel=list(floor.world_to_pixel(x,y)),ground_cm=[x,y]) for x,y in [(40,30),(70,40),(85,25)]]
    # Synthetic ground measurements establish only this generated camera model.
    mapping=MarkerPlaneMapping.fit(training,validation,calibration_id='synthetic-camera',frame_size=(1280,720),
        floor_size_cm=(121.6,68.4),height_cm=16.6,attested=True)
    tracker=RobotTracker(floor,config,calibration_id='synthetic-camera',marker_plane=mapping)
    tracker.verify_context(dict(training=training,validation=validation,metadata=dict(attested_measured=True,
        measurement_reference='MARKER_GROUND_PROJECTION',calibration_id='synthetic-camera',frame_size=[1280,720],
        mounting_height_cm=16.6,robot_configuration=asdict(config))),fixed_camera=True,ground_reference_confirmed=True)
    monkeypatch.setattr('navigation.adapters.build_robot_tracker',lambda *a,**k:tracker)
    monkeypatch.setattr('navigation.adapters.time.monotonic',clock)
    clean=np.full((720,1280,3),150,np.uint8);scene=clean.copy();locations={}
    for category,x,y,color in [('RED_SPILL',78,36,(20,20,220)),('NON_RED_SPILL',95,40,(20,210,210))]:
        pixel=tuple(round(v) for v in floor.world_to_pixel(x,y));locations[category]=pixel
        cv2.circle(scene,pixel,8,color,-1)
    marker=cv2.cvtColor(generate_marker(config,90,15),cv2.COLOR_GRAY2BGR)
    def read():
        clock.sleep(.002);a=math.radians(plant.heading);r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        center=np.array([plant.x,plant.y])+r @ np.array([2.,1.])
        world=np.array([[-6,-6],[6,-6],[6,6],[-6,6]]) @ r.T+center
        pixels=np.float32([floor.world_to_pixel(*p) for p in world])
        transform=cv2.getPerspectiveTransform(np.float32([[0,0],[119,0],[119,119],[0,119]]),pixels)
        patch=cv2.warpPerspective(marker,transform,(1280,720));mask=np.zeros((720,1280),np.uint8)
        cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),255)
        frame=scene.copy();frame[mask!=0]=patch[mask!=0];return frame
    source=CameraPoses(SimpleNamespace(read=read),'synthetic','settings','mapping',log=lambda _:None)
    def frames():
        source.observe();frame,robot=source.latest_observation
        return frame,tracker.exclusion_mask(robot,frame.shape[1::-1])
    scanner=SceneScanner(BackgroundReference(clean,confirmed_clean=True),floor,'synthetic-camera','config/spill_config.json',frames,
                         logger=WireLogger(stream=io.StringIO()))
    def remove(target):
        x,y=locations[target['category']];scene[y-12:y+13,x-12:x+13]=clean[y-12:y+13,x-12:x+13]
    events=[]
    result=run_queue(scanner,Mock(),adapter,source,transport,timing,arena,NavigationConfig(),clock,clock.sleep,
                     events.append,max_tasks=3,after_task=remove,calibration=floor,return_home=False)
    assert result['state']=='READY' and len(result['completed'])==2 and not result['skipped']
    assert [t['target']['category'] for t in result['completed']]==['RED_SPILL','NON_RED_SPILL']
    arrivals=[e for e in events if e['event_type']=='BUMPER_CLEARANCE_VERIFIED']
    assert len(arrivals)==2 and all(e['pose_verified'] and abs(e['clearance_cm']-5)<=2.5 for e in arrivals)
    assert source.frame>20 and transport.lines.count('ARRIVED')==2
