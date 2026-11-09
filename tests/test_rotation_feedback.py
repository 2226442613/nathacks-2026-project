"""Rotation recovery uses only fresh simulated feedback; no devices or sockets."""
from dataclasses import replace
import pytest

from navigation.adapters import PoseUnavailable
from navigation.controller import NavigationFailure
from navigation.timing import angle_error
from test_navigation import rig as navigation_rig


def rig():
    result = navigation_rig()
    # Current physical measurements have no validated stopping-latency samples.
    result[0].timing.data['stopping_samples'] = []
    return result


class InterruptedFeedback:
    def __init__(self, plant, transport, *, trigger=None, recovered_heading=None,
                 permanent=False, stale=False):
        self.plant, self.transport = plant, transport
        self.trigger, self.recovered_heading = trigger, recovered_heading
        self.permanent, self.stale = permanent, stale
        self.missing = False
        self.triggered = False
        self.recoveries = []

    def observe(self):
        if not self.triggered and self.trigger and self.trigger():
            self.triggered = self.missing = True
        if self.missing:
            raise PoseUnavailable('MARKER_NOT_VISIBLE', 'synthetic rotation blur', retry_startup=True)
        return self.plant.observe()

    def reacquire(self, *, after=None, timeout):
        # Recovery may wait only once motion is positively completed or stopped.
        assert self.transport.pending is None
        self.recoveries.append((list(self.transport.lines), after, timeout))
        if self.permanent:
            raise PoseUnavailable('MARKER_NOT_VISIBLE', 'synthetic permanent occlusion', retry_startup=True)
        self.missing = False
        if self.recovered_heading is not None:
            self.plant.heading = self.recovered_heading
        pose = self.plant.observe()
        return replace(pose, observed_at=pose.observed_at - 10) if self.stale else pose


def moves(transport):
    return [line for line in transport.lines if line.startswith('MOVE,')]


def test_done_without_observed_rotation_still_stops_with_measured_diagnostics():
    nav,plant,transport,plan=navigation_rig(turn_gain=0.)
    nav.timing.data['stopping_samples']=[]
    result=nav.run(plan,'RED_SPILL')
    assert result['state']=='BLOCKED' and not nav.ready and transport.stopped
    assert '3 consecutive attempts below 0.5 deg' in result['reason']
    assert 'does not prove chassis rotation' in result['reason']
    assert len(moves(transport))==3 and 'ARRIVED' not in transport.lines
    completed=[e for e in nav.events if e.get('segment_outcome')=='completed']
    assert len(completed)==3 and all(e['movement_completed'] for e in completed)
    assert all(e['observed_heading_change_deg']==0 and e['expected_change']>0 for e in completed)
    progress=[e for e in nav.events if 'heading_progress_deg' in e]
    assert [e['no_progress_attempts'] for e in progress]==[1,2,3]
    assert all(e['heading_progress_deg']==0 and e['no_progress_limit']==3 for e in progress)


def test_turn_diagnostics_are_signed_measured_deltas_across_wraparound():
    nav,plant,transport,_=rig()
    plant.heading=10.
    nav.align(340.,'ROTATING')
    completed=[e for e in nav.events if 'segment_outcome' in e]
    assert completed and all(e['command'].startswith('MOVE,L,') for e in completed)
    for event in completed:
        assert event['change_units']=='deg' and event['expected_change']<0
        assert event['observed_heading_change_deg']==pytest.approx(angle_error(
            event['fresh_pose']['heading_deg'],event['previous_pose']['heading_deg']))


def test_done_arriving_during_camera_check_uses_stationary_reacquisition(monkeypatch):
    nav,plant,transport,_=rig()
    original=nav.observe;interrupted=False
    monkeypatch.setattr(type(transport),'get_move_result',lambda self,result:self.move,raising=False)
    def observe(*args,**kwargs):
        nonlocal interrupted
        if kwargs.get('stationary') is False and not interrupted:
            interrupted=True
            # Model DONE arriving while the camera monitor is awaiting a frame.
            nav.clock.sleep(transport.model.move_deadline-nav.clock())
            transport._advance()
            raise NavigationFailure('Synthetic camera read delayed past DONE','PAUSED',recoverable=True)
        return original(*args,**kwargs)
    monkeypatch.setattr(nav,'observe',observe)
    nav.align(90.,'ROTATING')
    assert interrupted and 'STOP' not in transport.lines
    assert abs(angle_error(90.,nav.pose.heading_deg))<=nav.config.heading_tolerance_deg
    assert all(e['segment_outcome']=='completed' for e in nav.events if 'segment_outcome' in e)


def test_active_rotation_loss_stops_before_stationary_recovery_and_new_move():
    nav, plant, transport, _ = rig()
    source = InterruptedFeedback(plant, transport,
        trigger=lambda: transport.move is not None and transport.move.execution_state == 'accepted',
        recovered_heading=40.)
    nav.poses = source
    nav.align(90., 'ROTATING')

    assert len(source.recoveries) == 1
    prior_lines, _, timeout = source.recoveries[0]
    assert prior_lines == ['MOVE,R,500', 'STOP', 'PING']
    assert timeout == nav.config.reacquisition_seconds
    assert len(moves(transport)) > 1
    assert abs(plant.heading - 90) <= nav.config.heading_tolerance_deg
    recovered = next(e for e in nav.events if e.get('segment_outcome') == 'early_stopped')
    assert recovered['fresh_pose']['heading_deg'] == 40.
    later_alignment = [e for e in nav.events if e['sequence'] > recovered['sequence'] and 'heading_error_deg' in e]
    assert later_alignment[0]['heading_error_deg'] == 50.


def test_completed_rotation_reacquires_after_settling_from_new_heading(monkeypatch):
    nav, plant, transport, _ = rig()
    completed = False
    original = transport.wait_for_move
    def wait(*args, **kwargs):
        nonlocal completed
        result = original(*args, **kwargs)
        completed = True
        return result
    monkeypatch.setattr(transport, 'wait_for_move', wait)
    source = InterruptedFeedback(plant, transport, trigger=lambda: completed, recovered_heading=88.)
    nav.poses = source
    nav.align(90., 'ROTATING')

    assert source.recoveries and source.recoveries[0][1] is not None
    assert moves(transport) == ['MOVE,R,500']
    assert 'STOP' not in transport.lines
    assert nav.pose.heading_deg == 88.
    assert nav.pose.observed_at > source.recoveries[0][1]


def test_permanent_marker_loss_exits_without_next_move_cleaning_or_home_return():
    nav, plant, transport, plan = rig()
    source = InterruptedFeedback(plant, transport,
        trigger=lambda: transport.move is not None and transport.move.execution_state == 'accepted',
        permanent=True)
    nav.poses = source
    result = nav.run(plan, 'RED_SPILL')

    assert result['state'] == 'PAUSED' and not nav.ready
    assert len(moves(transport)) == 1
    assert transport.lines[-2:] == ['STOP', 'PING']
    assert len(source.recoveries) == 1
    assert not any(line.startswith(('TARGET,', 'ARRIVED')) for line in transport.lines)
    assert not any(e['state'].startswith('RETURNING') for e in nav.events)


def test_missing_stop_ack_locks_without_attempting_reacquisition():
    nav, plant, transport, plan = rig()
    transport.missing_stop_ack = True
    source = InterruptedFeedback(plant, transport,
        trigger=lambda: transport.move is not None and transport.move.execution_state == 'accepted')
    nav.poses = source
    result = nav.run(plan, 'RED_SPILL')

    assert result['state'] == 'ERROR' and nav.faulted
    assert len(moves(transport)) == 1
    assert source.recoveries == []
    assert 'STOP acknowledgment' in result['reason']


def test_stale_reacquired_pose_is_rejected_without_another_move():
    nav, plant, transport, plan = rig()
    source = InterruptedFeedback(plant, transport,
        trigger=lambda: transport.move is not None and transport.move.execution_state == 'accepted',
        stale=True)
    nav.poses = source
    result = nav.run(plan, 'RED_SPILL')

    assert result['state'] == 'PAUSED' and 'Stale' in result['reason']
    assert len(moves(transport)) == 1
    assert len(source.recoveries) == 1


def test_pre_send_recovery_does_not_transmit_command_computed_from_old_heading():
    nav, plant, transport, _ = rig()
    observations = 0
    def second_observation():
        nonlocal observations
        observations += 1
        # align observes once to compute the turn; _pulse observes again before TX.
        return observations == 2
    source = InterruptedFeedback(plant, transport, trigger=second_observation, recovered_heading=88.)
    nav.poses = source
    nav.align(90., 'ROTATING')

    assert source.recoveries and nav.pose.heading_deg == 88.
    assert moves(transport) == []
    assert any(e.get('recompute_command') for e in nav.events)
