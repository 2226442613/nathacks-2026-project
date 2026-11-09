"""Read-only command planning; camera frames and transport are never real."""
from contextlib import contextmanager
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest

import main
from navigation.geometry import Arena
from navigation.quick_plan import coordinate_plan,timed_route
from navigation.timing import load_timing,angle_error
from vision import GroundCalibration,BackgroundReference,RobotConfig
from vision.robot_plane import MarkerPlaneMapping
from vision.robot_tracking import generate_marker
from test_robot_tracking import training_data
from test_localization import dual


def timing():return load_timing('config/movement_timing.json')


@pytest.mark.parametrize('goal,direction,duration',[
    ({'x_cm':56.,'y_cm':30.,'heading_deg':0.},'F',1000),
    ({'x_cm':21.5,'y_cm':30.,'heading_deg':0.},'B',1000),
    ({'x_cm':40.,'y_cm':30.,'heading_deg':90.},'R',3000),
    ({'x_cm':40.,'y_cm':30.,'heading_deg':270.},'L',4000)])
def test_direction_specific_move_preview_and_estimated_pose(goal,direction,duration):
    route=timed_route({'x_cm':40.,'y_cm':30.,'heading_deg':0.},goal,timing(),Arena(),1000)
    assert all(c.startswith('MOVE,'+direction+',') for c in route['commands'])
    assert sum(int(c.split(',')[2]) for c in route['commands'])==duration
    assert all(0<int(c.split(',')[2])<=1000 for c in route['commands'])
    assert route['estimated_final_pose']==pytest.approx(goal)


def test_shorter_reverse_route_and_floor_sweep_rejection():
    start={'x_cm':55.,'y_cm':30.,'heading_deg':0.}
    goal={**start,'x_cm':30.}
    route=timed_route(start,goal,timing(),Arena(),1000)
    assert all(c.startswith('MOVE,B,') for c in route['commands'])
    assert route['turn_degrees']==0
    with pytest.raises(ValueError,match='No simple'):
        timed_route({'x_cm':30.,'y_cm':10.,'heading_deg':0.},
                    {'x_cm':30.,'y_cm':10.,'heading_deg':90.},timing(),Arena(),1000)


@pytest.mark.parametrize('surface',['FLOOR','WALL'])
def test_beside_spill_parking_and_wall_parallel(dual,surface):
    arena=Arena(width=120,depth=80)
    target={'surface':surface,'category':'RED_SPILL','position_cm':{'x':70,'y':40} if surface=='FLOOR' else {'x':70,'h':15}}
    start={'x_cm':35.,'y_cm':35.,'heading_deg':0.}
    plan=coordinate_plan(start,[target],timing(),arena,dual,1.5,1000)
    assert arena.safe(plan['parking'])
    if surface=='WALL':assert plan['parking']['heading_deg'] in (0,180)
    else:assert math.dist([70,40],[plan['parking']['x_cm'],plan['parking']['y_cm']])>=arena.body_width/2
    assert math.dist([plan['parking']['x_cm'],plan['parking']['y_cm']],
                     [plan['estimated_final_pose']['x_cm'],plan['estimated_final_pose']['y_cm']])<.1
    assert abs(angle_error(plan['parking']['heading_deg'],plan['estimated_final_pose']['heading_deg']))<.1


def test_missing_coordinates_and_impossible_parking(dual):
    start={'x_cm':40.,'y_cm':30.,'heading_deg':0.}
    with pytest.raises(ValueError,match='No confirmed'):coordinate_plan(start,[],timing(),Arena(),dual,1.5,1000)
    target={'surface':'FLOOR','category':'RED_SPILL','position_cm':None}
    with pytest.raises(ValueError,match='coordinates missing'):coordinate_plan(start,[target],timing(),Arena(),dual,1.5,1000)
    target['position_cm']={'x':10,'y':10}
    with pytest.raises(ValueError,match='outside the floor|No beside-spill'):
        coordinate_plan({'x_cm':15.,'y_cm':15.,'heading_deg':0.},[target],timing(),Arena(width=30,depth=30,margin=4),dual,4,1000)


def test_cli_uses_real_image_detection_aruco_and_localization_without_network_or_writes(tmp_path,monkeypatch,capsys):
    import navigation.quick_plan as module
    import robot_comm
    def forbidden(*a,**k):raise AssertionError('Planner attempted transport')
    monkeypatch.setattr(robot_comm.RobotClient,'connect',forbidden)
    cal=tmp_path/'camera.json'
    GroundCalibration([[0,0],[799,0],[799,599],[0,599]],79.9,59.9,800,600).save(cal)
    config=tmp_path/'robot.json';config.write_bytes(Path('config/robot_config.json').read_bytes())
    train,validation=training_data()
    plane=MarkerPlaneMapping.fit(train,validation,calibration_id='sha256:'+hashlib.sha256(cal.read_bytes()).hexdigest(),
        frame_size=(800,600),floor_size_cm=(79.9,59.9),height_cm=16.6,attested=True)
    mapping=tmp_path/'plane.json';mapping.write_text(json.dumps(plane.to_dict()))
    clean=np.full((600,800,3),180,np.uint8);scene=clean.copy()
    marker=generate_marker(RobotConfig(marker_size_cm=9),90,15)
    scene[240:360,340:460]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    cv2.circle(scene,(670,300),12,(20,20,220),-1)
    background=tmp_path/'baseline.png';BackgroundReference(clean,confirmed_clean=True).save(background)
    files=[cal,config,mapping,background,Path(str(background)+'.json')]
    before={p:p.read_bytes() for p in files};all_before=set(tmp_path.iterdir())
    @contextmanager
    def camera(*a,**k):
        from types import SimpleNamespace
        yield SimpleNamespace(read=lambda:scene.copy())
    monkeypatch.setattr(module,'CameraManager',camera)
    assert main.main(['quick-plan','--camera','0','--calibration',str(cal),'--robot-settings',str(config),
        '--marker-plane',str(mapping),'--background',str(background)])==0
    text=capsys.readouterr().out
    assert 'PLAN ONLY' in text and 'Robot:' in text and 'Spill: FLOOR RED_SPILL' in text
    assert 'Parking:' in text and 'MOVE,' in text and 'Estimated final pose:' in text
    assert 'TARGET,' not in text and 'ARRIVED' not in text
    assert all(p.read_bytes()==before[p] for p in files) and set(tmp_path.iterdir())==all_before
