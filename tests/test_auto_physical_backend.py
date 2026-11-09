"""Physical branch selection and pose safety with injected camera/socket boundaries."""
from contextlib import contextmanager
import json
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import main
from navigation.controller import Pose
from navigation.timing import load_timing


def physical_args(output,*,navigation_only=False):
    flags=(['--navigation-only','--confirm-navigation-test'] if navigation_only else
           ['--authorize-motion','--confirm-clean','--confirm-fixed-camera','--confirm-ground-reference','--confirm-static-tools'])
    return ['auto-clean','--mode','physical','--host','172.20.10.12','--port','3333',
            '--background','data/iphone_reference.png','--max-tasks','1','--output-dir',str(output)]+flags


def test_full_physical_cleaning_still_requires_operator_authorization(tmp_path,monkeypatch,caplog):
    import navigation.automatic as module
    if not Path('config/camera_config.json').exists():pytest.skip('Private calibration not distributed')
    def forbidden(*a,**k):raise AssertionError('Camera/socket or simulation backend accessed')
    monkeypatch.setattr(module,'CameraManager',forbidden);monkeypatch.setattr(module,'RobotClient',forbidden)
    monkeypatch.setattr('navigation.cli.make_demo',forbidden)
    args=physical_args(tmp_path/'blocked');args.remove('--authorize-motion')
    assert main.main(args)==1
    assert 'motion authorization' in caplog.text


@pytest.mark.parametrize('navigation_only',[False,True])
def test_physical_navigation_uses_camera_source_realtime_and_explicit_tcp_client(tmp_path,monkeypatch,navigation_only):
    import navigation.automatic as module
    if not Path('config/camera_config.json').exists():pytest.skip('Private calibration not distributed')
    frame=np.zeros((720,1280,3),np.uint8)
    observation=SimpleNamespace(position_cm={'x':40.,'y':30.},heading_deg=0.,marker_corners_pixels=None)
    poses=Mock();poses.observe.return_value=Pose(40.,30.,0.,time.monotonic(),1,verified=True)
    poses.consumed_observation=(frame,observation)
    poses.tracker.exclusion_mask.return_value=np.zeros(frame.shape[:2],np.uint8)
    source=Mock(return_value=poses);monkeypatch.setattr(module,'CameraPoses',source)
    camera=Mock()
    @contextmanager
    def manager(*a,**k):yield camera
    monkeypatch.setattr(module,'CameraManager',manager)
    transport=Mock()
    @contextmanager
    def client(*a,**k):
        assert a==('172.20.10.12',3333) and k['mode']=='physical' and k['firmware_profile']=='simple'
        yield transport
    monkeypatch.setattr(module,'RobotClient',client)
    def forbidden(*a,**k):raise AssertionError('Physical mode chose simulated backend')
    monkeypatch.setattr('navigation.cli.make_demo',forbidden)
    called=[]
    def queue(scanner,planner,adapter,p,t,timing,arena,config,clock,sleep,emit,**kwargs):
        called.append(True)
        assert p is poses and t is transport and poses.camera is camera
        assert clock is time.monotonic and sleep is time.sleep
        assert timing.data['mode']=='PHYSICAL' and timing.rate('F')[0]==62.5
        assert scanner.physical and scanner.navigation_only==navigation_only and kwargs['navigation_only']==navigation_only
        assert adapter.hardware.mode=='PHYSICAL' and adapter.h['floor_sponge']['offset_forward_cm'] is None
        assert (arena.length,arena.body_width)==(26.,15.2)
        current,mask,hard=scanner.frame_source()
        assert current is frame and np.count_nonzero(hard)>0
        assert source.call_args.kwargs['verification_points_path']=='data/robot_plane_points_marker9.json'
        emit(dict(event_type='BUMPER_CLEARANCE_VERIFIED',clearance_cm=5.,pose_frame_id=7))
        return dict(state='NO_CONFIRMED_SPILL',completed=[],skipped=[])
    monkeypatch.setattr(module,'run_queue',queue)
    out=tmp_path/'physical'
    assert main.main(physical_args(out,navigation_only=navigation_only))==0 and called==[True]
    log=(out/'wire.log').read_text()
    assert 'PHYSICAL live camera 0' in log and 'TCP 172.20.10.12:3333' in log and '[ARRIVAL]' in log
    assert json.loads((out/'summary.json').read_text())['mode']=='physical'


def test_physical_feedback_never_uses_predicted_motion_as_arrival():
    from test_navigation import rig
    nav,plant,transport,_=rig(move_gain=0.)
    nav.timing=load_timing('config/movement_timing.json')
    result=nav.run_navigation_only(dict(x_cm=plant.x+12,y_cm=plant.y,heading_deg=plant.heading))
    assert result['state']=='BLOCKED' and 'no measurable progress' in result['reason']
    assert not result['arrival_verified'] and not result['home_verified'] and transport.stopped
    assert any('predicted_pose' in e and e['predicted_pose']['x_cm']!=plant.x for e in nav.events)
    assert plant.x==32. and not any(line.startswith('TARGET,') or line=='ARRIVED' for line in transport.lines)
