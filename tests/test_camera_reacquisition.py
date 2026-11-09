"""Existing Module04 tracker, synthetic detections and injected camera reads only."""
from dataclasses import replace
import json
import threading
import time
from types import SimpleNamespace

import pytest

from navigation.adapters import CameraPoses,PoseUnavailable
from test_navigation_only import verified_tracker
from test_robot_tracking import floor,observed,marker_scene


def source_for_test(floor,monkeypatch):
    tracker,evidence=verified_tracker(floor)
    tracker.verify_context(evidence,fixed_camera=True,ground_reference_confirmed=True)
    state=dict(visible=True,angle=0.,read_failure=False,block=None)
    image=marker_scene();logs=[]
    def detect(frame):
        tracker.detector.last_quality=dict(decoded_marker_count=int(state['visible']),
            configured_marker_count=int(state['visible']),rejected_candidates=2)
        if not state['visible']:return None
        return replace(observed(center=(420,250),angle=state['angle']),quality=dict(
            marker_area_pixels=3600.,minimum_edge_pixels=60.,**tracker.detector.last_quality))
    monkeypatch.setattr(tracker.detector,'detect',detect)
    monkeypatch.setattr('navigation.adapters.build_robot_tracker',lambda *a,**k:tracker)
    def read():
        time.sleep(.005)
        if state['block'] is not None:state['block'].wait(2)
        if state['read_failure']:raise ValueError('Synthetic camera read failed')
        return image
    source=CameraPoses(SimpleNamespace(read=read),'cal','robot','map',log=logs.append)
    source.start(.03,startup_wait_seconds=.3)
    return source,state,logs


def wait_for(predicate):
    deadline=time.monotonic()+1
    while not predicate() and time.monotonic()<deadline:time.sleep(.002)
    assert predicate()


def test_runtime_marker_loss_keeps_worker_alive_invalidates_cache_and_recovers_new_heading(floor,monkeypatch):
    source,state,logs=source_for_test(floor,monkeypatch)
    try:
        first=source.observe();state['visible']=False
        wait_for(lambda:source.capture_error is not None)
        assert source.worker.is_alive() and source.latest is None
        with pytest.raises(PoseUnavailable):source.observe()
        state['angle']=40.
        timer=threading.Timer(.04,lambda:state.update(visible=True));timer.start()
        start=time.monotonic()
        fresh=source.reacquire(timeout=.25)
        timer.join(1)
        assert fresh.frame_id>first.frame_id and fresh.observed_at>start and fresh.verified
        assert fresh.heading_deg==pytest.approx(40.,abs=.01)
        assert source.last_unavailable is None and source.capture_error is None
        assert any('REACQUIRE_VERIFIED' in s for s in logs)
        reports=[json.loads(s.split('POSE_DIAGNOSTIC ',1)[1]) for s in logs if s.startswith('POSE_DIAGNOSTIC ')]
        visible=next(r for r in reports if r['marker_visible'] is True)
        assert visible['position_cm'] and visible['marker_id']==0
        assert visible['heading_deg'] is not None and len(visible['marker_corners_pixels'])==4
        assert visible['age_basis']=='host frame receipt; source capture latency unknown'
    finally:source.close()


def test_stationary_reacquisition_is_bounded_and_never_returns_last_good_pose(floor,monkeypatch):
    source,state,logs=source_for_test(floor,monkeypatch)
    try:
        old=source.observe();state['visible']=False
        wait_for(lambda:source.capture_error is not None)
        start=time.monotonic()
        with pytest.raises(PoseUnavailable,match='STALE_POSE'):source.reacquire(timeout=.08)
        assert .07<=time.monotonic()-start<.4
        assert source.latest is None and source.consumed==old.frame_id
        reports=[json.loads(s.split('POSE_DIAGNOSTIC ',1)[1]) for s in logs if s.startswith('POSE_DIAGNOSTIC ')]
        missing=next(r for r in reports if r['marker_visible'] is False)
        assert missing['frame_read']=='OK' and missing['detection_confidence']=='NOT_DECODED'
        assert missing['decoded_marker_count']==0 and missing['pose_age_seconds']>=0
        assert missing['confidence_probability'] is None
    finally:source.close()


def test_blocked_camera_read_reports_read_status_and_cannot_block_recovery_forever(floor,monkeypatch):
    source,state,logs=source_for_test(floor,monkeypatch)
    release=threading.Event()
    try:
        source.observe();state['block']=release
        wait_for(lambda:source.frame_read_status=='READING')
        # Consume any final sample completed just before the injected stall.
        with source.condition:source.latest=None
        start=time.monotonic()
        with pytest.raises(PoseUnavailable):source.reacquire(timeout=.07)
        assert time.monotonic()-start<.3
        report=json.loads(next(s.split('POSE_DIAGNOSTIC ',1)[1] for s in reversed(logs) if s.startswith('POSE_DIAGNOSTIC ')))
        assert report['frame_read']=='READING' and report['read_in_progress_seconds']>0
        assert report['marker_visible'] is None and report['detection_confidence']=='UNKNOWN'
    finally:release.set();source.close()


def test_camera_read_failure_is_fatal_without_extended_reacquisition(floor,monkeypatch):
    source,state,logs=source_for_test(floor,monkeypatch)
    try:
        source.observe();state['read_failure']=True
        wait_for(lambda:source.capture_error is not None)
        start=time.monotonic()
        with pytest.raises(ValueError,match='Synthetic camera read failed'):source.reacquire(timeout=.5)
        assert time.monotonic()-start<.1
        report=json.loads(next(s.split('POSE_DIAGNOSTIC ',1)[1] for s in reversed(logs) if s.startswith('POSE_DIAGNOSTIC ')))
        assert report['frame_read']=='FAILED' and report['confidence_probability'] is None
    finally:source.close()


def test_real_aruco_decoder_reports_visibility_quality_without_inventing_probability():
    import numpy as np
    from vision.robot_tracking import RobotMarkerDetector
    detector=RobotMarkerDetector()
    assert detector.detect(marker_scene()) is not None
    assert detector.last_quality['configured_marker_count']==1
    assert detector.last_quality['marker_area_pixels']>0 and detector.last_quality['minimum_edge_pixels']>0
    assert detector.detect(np.full((600,800,3),255,np.uint8)) is None
    assert detector.last_quality['decoded_marker_count']==0 and 'marker_area_pixels' not in detector.last_quality
