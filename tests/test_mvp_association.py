"""Physical MVP configuration and handoff exercised with synthetic frames/transport."""
import io
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from navigation.automatic import SceneScanner,run_queue
from navigation.adapters import ToolApproach
from navigation.cli import synthetic_calibration
from navigation.controller import NavigationConfig,Pose,NavigationFailure
from navigation.geometry import Arena
from navigation.simulation import SimClock,SimPlant,SimTransport
from navigation.timing import load_timing
from robot_comm import WireLogger
from vision import BackgroundReference
from vision.cleaning_config import CleaningHardware
from test_auto_physical_backend import physical_args


@pytest.mark.parametrize('field,value,message',[
    ('calibration_id','sha256:wrong','different calibration'),('source',2,'camera source mismatch')])
def test_conflicting_background_association_fails_before_camera_or_tcp(tmp_path,monkeypatch,caplog,field,value,message):
    import navigation.automatic as module
    if not Path('config/camera_config.json').exists():pytest.skip('Private calibration not distributed')
    background=BackgroundReference.load('data/iphone_reference.png',confirmed_clean=True)
    background._metadata[field]=value
    path=tmp_path/'background.png';background.save(path)
    def forbidden(*a,**k):raise AssertionError('Hardware accessed on conflicting association')
    monkeypatch.setattr(module,'CameraManager',forbidden);monkeypatch.setattr(module,'RobotClient',forbidden)
    args=physical_args(tmp_path/'run');args[args.index('--background')+1]=str(path)
    assert main.main(args)==1 and message in caplog.text


def physical_rig():
    calibration=synthetic_calibration();clock=SimClock();plant=SimPlant(clock)
    transport=SimTransport(plant);logger=WireLogger(stream=io.StringIO());transport.logger=logger
    timing=load_timing('config/movement_timing.json')
    plant.actual_rates.update(F=62.5,B=1000/18.5,L=4000/90,R=3000/90)
    hardware=CleaningHardware(json.loads(Path('config/cleaning_hardware.json').read_text()))
    arena=Arena(side_tool=hardware.side_collision_envelope())
    adapter=ToolApproach(hardware,calibration,arena,timing)
    return calibration,clock,plant,transport,logger,timing,hardware,arena,adapter


def test_partial_physical_contact_profile_can_handoff_ground_preset_but_keeps_collision_checks():
    calibration,clock,plant,transport,logger,timing,hardware,arena,adapter=physical_rig()
    assert hardware.missing('FLOOR')  # No invented contact or spray dimensions.
    clean=np.full((720,1280,3),150,np.uint8);scene=clean.copy();patches={}
    for category,x,y,color in [('RED_SPILL',78,36,(20,20,220)),('NON_RED_SPILL',90,40,(20,210,210))]:
        pixel=tuple(round(v) for v in calibration.floor.world_to_pixel(x,y));patches[category]=pixel
        cv2.circle(scene,pixel,7,color,-1)
    scanner=SceneScanner(BackgroundReference(clean,confirmed_clean=True),calibration,'test-calibration',
        'config/spill_config.json',lambda:(scene.copy(),None),physical=True,logger=logger)
    planner=Mock();planner.plan.side_effect=AssertionError('Floor preset must not demand unmeasured tool coverage')
    def removed(target):
        x,y=patches[target['category']];scene[y-10:y+11,x-10:x+11]=clean[y-10:y+11,x-10:x+11]
    events=[]
    result=run_queue(scanner,planner,adapter,plant,transport,timing,arena,NavigationConfig(),clock,clock.sleep,events.append,
        max_tasks=2,after_task=removed,calibration=calibration)
    assert result['state']=='READY' and len(result['completed'])==2 and not result['skipped']
    assert [r['target']['category'] for r in result['completed']]==['RED_SPILL','NON_RED_SPILL']
    assert all(r['result']['home_verified'] for r in result['completed'])
    assert len([e for e in events if e['event_type']=='BUMPER_CLEARANCE_VERIFIED'])==2
    log=logger.stream.getvalue()
    assert log.count('[TX] TARGET,')==2 and log.count('[TX] ARRIVED')==2 and log.count('[RX] DONE,CLEANING')==2
    assert not any(line.startswith(('PUMP,','SERVO,','SPONGE,')) for line in transport.lines)


def test_ground_preset_collision_sweep_remains_required_without_contact_coverage():
    _,_,_,_,_,_,_,arena,adapter=physical_rig()
    goal=dict(x_cm=100.,y_cm=35.,heading_deg=0.)
    assert arena.safe(goal)  # Parked chassis fits, but its firmware forward stroke does not.
    plan=dict(surface='FLOOR',target_geometry=[],approach_pose=goal)
    with pytest.raises(ValueError,match='swept area'):
        adapter.check(plan,Pose(**goal,observed_at=0,frame_id=0),verify_tool_coverage=False)


def test_unknown_wall_contact_geometry_is_still_not_invented():
    _,_,_,_,_,_,_,_,adapter=physical_rig()
    with pytest.raises(NavigationFailure,match='Unconfigured cleaning geometry'):
        adapter.prepare(dict(surface='WALL',reachability='FULLY_REACHABLE',approach_pose={},target_geometry=[]))
