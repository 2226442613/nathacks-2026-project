"""Shared real vision algorithms on synthetic frames, mocked camera/HighGUI only."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import socket
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import BackgroundReference,GroundCalibration,RobotConfig
from vision.robot_plane import MarkerPlaneMapping
from vision.robot_tracking import RobotTracker,generate_marker
from vision.spill_detection import SpillDetector
from vision.live_test import ground_coordinates
from test_robot_tracking import training_data
from test_localization import dual

pytestmark=pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')


@pytest.fixture
def live_inputs(tmp_path):
    cal=tmp_path/'camera.json'
    GroundCalibration([[0,0],[799,0],[799,599],[0,599]],79.9,59.9,800,600).save(cal)
    robot=tmp_path/'robot.json';robot.write_bytes(Path('config/robot_config.json').read_bytes())
    train,validation=training_data()
    mapping=MarkerPlaneMapping.fit(train,validation,calibration_id='sha256:'+hashlib.sha256(cal.read_bytes()).hexdigest(),
        frame_size=(800,600),floor_size_cm=(79.9,59.9),height_cm=16.6,attested=True)
    plane=tmp_path/'plane.json';plane.write_text(json.dumps(mapping.to_dict()))
    clean=np.full((600,800,3),180,np.uint8);frame=clean.copy()
    marker=generate_marker(RobotConfig(marker_size_cm=9),90,15)
    frame[240:360,340:460]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    cv2.circle(frame,(660,250),12,(20,20,220),-1)
    cv2.circle(frame,(660,450),12,(20,210,210),-1)
    baseline=tmp_path/'background.png';BackgroundReference(clean,confirmed_clean=True).save(baseline)
    return frame,['test-vision','--camera','0','--calibration',str(cal),'--robot-settings',str(robot),
        '--marker-plane',str(plane),'--background',str(baseline)]


def mock_live(monkeypatch,frame):
    import vision.live_test as module
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=lambda:frame.copy()
    manager=Mock(return_value=camera);monkeypatch.setattr(module,'CameraManager',manager)
    for name in ('namedWindow','imshow','destroyAllWindows'):monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[-1,-1,-1,-1,ord('q')]))
    monkeypatch.setattr(module.time,'monotonic',Mock(side_effect=[0.,.4,.8,1.2,1.6]))
    def forbidden(*a,**k):raise AssertionError('Vision command attempted network access')
    monkeypatch.setattr(socket,'create_connection',forbidden)
    texts=[];put=cv2.putText
    def capture(image,text,*a,**k):texts.append(text);return put(image,text,*a,**k)
    monkeypatch.setattr(cv2,'putText',capture)
    return manager,camera,texts


def test_one_camera_shared_frames_boxes_coordinates_one_second_print_and_no_writes(live_inputs,tmp_path,monkeypatch,capsys):
    frame,args=live_inputs;manager,camera,texts=mock_live(monkeypatch,frame)
    files={p:p.read_bytes() for p in tmp_path.iterdir()};robot_frames=[];spill_frames=[]
    robot_update=RobotTracker.update;detect=SpillDetector.detect
    def robot_spy(self,frame,*a,**k):robot_frames.append(id(frame));return robot_update(self,frame,*a,**k)
    def spill_spy(self,frame,*a,**k):spill_frames.append(id(frame));return detect(self,frame,*a,**k)
    monkeypatch.setattr(RobotTracker,'update',robot_spy);monkeypatch.setattr(SpillDetector,'detect',spill_spy)
    assert main.main(args)==0
    manager.assert_called_once_with(0,800,600,30)
    assert camera.read.call_count==5 and robot_frames==spill_frames and len(robot_frames)==5
    assert cv2.imshow.call_count==5;camera.__exit__.assert_called_once();cv2.destroyAllWindows.assert_called_once()
    records=[json.loads(s) for s in capsys.readouterr().out.splitlines()]
    assert len(records)==2  # At 0 and 1.2 seconds, rather than every frame.
    assert records[-1]['robot']['position_cm']==pytest.approx({'x':35.95,'y':33.95},abs=.1)
    spills=records[-1]['spills']
    assert {s['category'] for s in spills}=={'RED_SPILL','NON_RED_SPILL'}
    assert all(s['confirmed'] and s['position_cm'] is not None for s in spills)
    assert any('theta=' in s for s in texts) and any('RED_SPILL' in s for s in texts) and any('NON_RED_SPILL' in s for s in texts)
    assert any('X=' in s and 'Y=' in s for s in texts)
    assert set(tmp_path.iterdir())==set(files) and all(p.read_bytes()==data for p,data in files.items())


@pytest.mark.parametrize('component',['robot','spill','camera','background'])
def test_specific_failure_visible_and_window_stays_open(live_inputs,monkeypatch,capsys,component):
    frame,args=live_inputs;_,camera,texts=mock_live(monkeypatch,frame)
    if component=='robot':monkeypatch.setattr(RobotTracker,'update',Mock(side_effect=ValueError('marker geometry test failure')))
    elif component=='spill':monkeypatch.setattr(SpillDetector,'detect',Mock(side_effect=ValueError('spill resolution test failure')))
    elif component=='camera':camera.read.side_effect=[frame,None,frame,frame,frame]
    else:args[args.index('--background')+1]+='.missing'
    assert main.main(args)==0
    assert camera.read.call_count==5 and cv2.imshow.call_count==5  # Only Q ends the test.
    expected={'robot':'marker geometry test failure','spill':'spill resolution test failure',
              'camera':'Camera returned no frame','background':'.missing'}[component]
    assert any(expected in s for s in texts)
    if component in ('robot','spill','background'):assert expected in capsys.readouterr().out


def test_wall_height_is_not_mislabeled_as_floor_y(dual):
    record=SimpleNamespace(surface='WALL',world_coordinates_cm={'x':50.,'h':25.})
    result=ground_coordinates(record,dual)
    x,y=dual.wall_to_floor_base(50,0)
    assert result['x']==x and result['y']==y and result['height_cm']==25
