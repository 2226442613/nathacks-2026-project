"""Offline geometry from run 073050; its unrecorded robot pose is never invented."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from navigation.automatic import run_queue
from navigation.controller import Navigator,NavigationConfig,Pose
from navigation.geometry import Arena,front_parking_candidates,front_bumper_metrics
from test_mvp_association import physical_rig


@pytest.fixture
def recorded_target():
    x,y,X,Y=66.23578934628188,35.61300244707212,71.88277104859377,41.31251813315419
    return dict(target_id=1,surface='FLOOR',category='RED_SPILL',
        position_cm=dict(x=68.99999112493391,y=38.49447436576853),
        geometry_cm=dict(bounds=dict(min_x=x,min_y=y,max_x=X,max_y=Y),
                         contours=[dict(kind='outer',points=[[x,y],[X,y],[X,Y],[x,Y]])]))


def test_recorded_spill_parking_has_five_cm_bumper_gap_and_keeps_boundary(recorded_target):
    *_,arena,adapter=physical_rig()
    goals=front_parking_candidates(recorded_target,arena,include_unsafe=True)
    expected=[
        (48.23578934628188,38.46276029011315,0.),(89.88277104859377,38.46276029011315,180.),
        (69.05928019743783,17.61300244707212,90.),(69.05928019743783,59.31251813315419,270.)]
    for goal,values in zip(goals,expected):
        assert (goal['x_cm'],goal['y_cm'],goal['heading_deg'])==pytest.approx(values)
    assert [arena.safe(g) for g in goals]==[True,True,True,False]
    assert front_parking_candidates(recorded_target,arena)==goals[:3]
    for goal in goals:
        assert front_bumper_metrics(recorded_target,goal,26.,15.2)==dict(clearance_cm=5.,facing_error_deg=0.)
    assert arena.corners(goals[3])[0][:,1].max()==pytest.approx(73.8125181331542)
    assert arena.depth==68.4  # 5.4125 cm beyond the floor, not a detection-quality gate.


def test_recorded_wall_side_parking_fails_actual_cleaning_sweep_but_west_east_pass(recorded_target):
    *_,arena,adapter=physical_rig()
    goals=front_parking_candidates(recorded_target,arena)
    def check(goal):
        adapter.check(dict(surface='FLOOR',approach_pose=goal,front_clearance_cm=5.,
                           target_geometry=recorded_target['geometry_cm']['contours']),
                      Pose(**goal,observed_at=0,frame_id=0),verify_tool_coverage=False)
    check(goals[0]);check(goals[1])
    with pytest.raises(ValueError,match='swept area') as error:check(goals[2])
    assert "'min_y': -0.17033088626121184" in str(error.value)
    assert 'extra_margin_cm=0.25' in str(error.value)
    # Worst bound of segment two: F350 at 16 cm/s, then B600 at 18.5 cm/s.
    assert goals[2]['y_cm']+16*.350-18.5*.600-(13+1.5+.25)==pytest.approx(-2.6369975529278786)


def test_safe_reverse_heading_is_tried_when_nearest_turn_sweeps_tool_outside_floor():
    _,clock,plant,transport,_,timing,_,arena,_=physical_rig()
    start=dict(x_cm=18.,y_cm=18.,heading_deg=0.)  # Synthetic counterexample, NOT recorded pose.
    nav=Navigator(plant,transport,timing,arena,NavigationConfig(),clock=clock,sleep=clock.sleep)
    with pytest.raises(ValueError,match='swept area'):
        arena.check_segment(start,{**start,'heading_deg':90.},turn_change=90.)
    arena.check_segment(start,{**start,'heading_deg':270.},turn_change=-90.)
    assert nav._axis_heading(start,'Y',38.)==270
    nav._preflight_route(start,dict(x_cm=18.,y_cm=38.,heading_deg=270.),('Y','X'))
    plant.x=plant.y=18.;plant.move_gain=plant.turn_gain=1.
    nav.axis('Y',38.)
    assert abs(plant.y-38.)<=nav.config.axis_tolerance_cm
    moves=[line for line in transport.lines if line.startswith('MOVE,')]
    assert moves[0].startswith('MOVE,L,') and any(line.startswith('MOVE,B,') for line in moves)
    assert all(int(line.split(',')[2])<=nav.config.maximum_segment_ms for line in moves)
    assert arena.safe(plant.observe().geometry())


def test_alternative_heading_does_not_bypass_obstacle_or_send_motion():
    _,clock,plant,transport,_,timing,hardware,_,_=physical_rig()
    arena=Arena(obstacles=[(60.,0.,61.,68.4)],side_tool=hardware.side_collision_envelope())
    nav=Navigator(plant,transport,timing,arena,NavigationConfig(),clock=clock,sleep=clock.sleep)
    assert arena.safe(plant.observe().geometry())
    with pytest.raises(ValueError,match='No collision-safe/calibrated X heading'):
        nav.axis('X',90.)
    assert not transport.lines


def test_no_reachable_target_records_pose_all_candidates_and_exact_failure_stages(recorded_target):
    calibration,clock,plant,transport,_,timing,hardware,_,adapter=physical_rig()
    arena=Arena(obstacles=[(60.,0.,61.,68.4)],side_tool=hardware.side_collision_envelope())
    adapter.arena=arena
    events=[]
    result=run_queue(SimpleNamespace(scan=lambda:[recorded_target]),Mock(),adapter,plant,transport,timing,
                     arena,NavigationConfig(),clock,clock.sleep,events.append,max_tasks=1,calibration=calibration)
    assert result['state']=='NO_REACHABLE_TARGET'
    skipped=result['skipped'][0]
    assert skipped['robot_pose']==dict(x_cm=32.,y_cm=29.,heading_deg=0.)
    attempts=skipped['parking_attempts']
    assert len(attempts)==4 and all(not a['accepted'] for a in attempts)
    assert {a['stage'] for a in attempts}=={'PARKING_BOUNDARY','OUTBOUND_Y_X'}
    assert all(a['clearance_cm']==5. for a in attempts)
    assert all('floor_cm=[0, 0, 121.6, 68.4]' in a['reason'] for a in attempts)
    assert events[0]['event_type']=='AUTO_TARGET_EVALUATION'
    assert len([e for e in events if e['event_type']=='AUTO_PARKING_CANDIDATE'])==4
    assert not transport.lines  # Rejected before MOVE, TARGET or ARRIVED.
