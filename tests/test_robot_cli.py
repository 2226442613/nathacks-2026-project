"""Real synthetic image/video CLI exercises and mocked live camera; no hardware claims."""
import hashlib
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import GroundCalibration, BackgroundReference
from test_robot_tracking import marker_scene,training_data

# These fixtures deliberately exercise supported legacy floor-only calibration.
pytestmark = pytest.mark.filterwarnings("ignore:Legacy floor-only calibration:UserWarning")


@pytest.fixture
def scene_files(tmp_path):
    cal=GroundCalibration([[0,0],[799,0],[799,599],[0,599]],79.9,59.9,800,600)
    cal.save(tmp_path/'floor.json')
    assert cv2.imwrite(str(tmp_path/'scene.png'),marker_scene())
    video=cv2.VideoWriter(str(tmp_path/'scene.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(800,600))
    assert video.isOpened()
    for scene in (marker_scene(),np.full((600,800,3),255,np.uint8),marker_scene(angle=90)): video.write(scene)
    video.release()
    return tmp_path


def read(path): return [json.loads(line) for line in path.read_text().splitlines()]


def test_generate_printable_marker_and_refuse_overwrite(tmp_path):
    output=tmp_path/'marker.png'
    args=['generate-aruco','--output',str(output),'--side-pixels','240','--quiet-zone-pixels','40','--marker-size-cm','6']
    assert main.main(args)==0
    info=json.loads(Path(str(output)+'.json').read_text())
    assert info['dictionary']=='DICT_4X4_50' and info['marker_id']==0
    assert info['required_total_image_width_cm']==8 and not info['physical_size_verified']
    before=output.read_bytes()
    assert main.main(args)==1 and output.read_bytes()==before


@pytest.mark.parametrize('source',['image','video'])
def test_offline_cli_json_and_no_headless_gui(scene_files,monkeypatch,source):
    for name in ('imshow','waitKey','destroyAllWindows'): monkeypatch.setattr(cv2,name,Mock(side_effect=AssertionError('Headless')))
    output=scene_files/f'{source}.jsonl'
    source_path=scene_files/('scene.png' if source=='image' else 'scene.avi')
    assert main.main(['track-robot',f'--{source}',str(source_path),'--calibration',str(scene_files/'floor.json'),
        '--headless','--output',str(output),'--annotated-dir',str(scene_files/f'{source}_preview'),'--mounting-height-cm','10'])==0
    values=read(output)
    assert values[0]['tracking_status']=='VISIBLE' and values[0]['position_cm'] is not None
    assert values[0]['localization_status']=='UNVERIFIED'
    assert values[0]['marker_configuration']['mounting_height_cm']==10
    assert values[0]['message_type']=='ROBOT_POSE' and values[0]['schema_version']==1
    assert not values[0]['motion_authorized']
    assert (scene_files/f'{source}_preview/frame_000000.png').exists()
    if source=='video':
        assert len(values)==3
        assert values[1]['tracking_event']=='TRACKING_LOST' and values[1]['position_cm'] is None
        assert values[1]['last_known_pose']['position_cm']
        assert values[2]['tracking_event']=='TRACKING_GAINED'


def test_live_tracking_releases_camera_and_preserves_inputs(scene_files,monkeypatch):
    import vision.robot_cli as cli
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=[marker_scene(),np.full((600,800,3),255,np.uint8)]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    show=Mock();close=Mock()
    monkeypatch.setattr(cv2,'imshow',show);monkeypatch.setattr(cv2,'destroyAllWindows',close)
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[-1,ord('q')]))
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    cal=scene_files/'floor.json';before=cal.read_bytes()
    assert main.main(['track-robot','--camera','0','--calibration',str(cal),'--output',str(scene_files/'live.jsonl')])==0
    assert show.call_count==2 and cal.read_bytes()==before
    camera.__exit__.assert_called_once();close.assert_called_once()
    assert read(scene_files/'live.jsonl')[-1]['position_cm'] is None


def test_empirical_plane_cli_and_tracker_height_binding(scene_files):
    training,validation=training_data()
    points=scene_files/'points.json';points.write_text(json.dumps({'training':training,'validation':validation}))
    output=scene_files/'marker_plane.json'
    assert main.main(['calibrate-robot-plane','--calibration',str(scene_files/'floor.json'),'--points',str(points),
        '--mounting-height-cm','10','--confirm-measured','--output',str(output)])==0
    data=json.loads(output.read_text())
    assert data['calibration_id']=='sha256:'+hashlib.sha256((scene_files/'floor.json').read_bytes()).hexdigest()
    assert data['validation_report']['maximum_error_cm']<1e-5
    assert main.main(['track-robot','--image',str(scene_files/'scene.png'),'--calibration',str(scene_files/'floor.json'),
        '--marker-plane',str(output),'--mounting-height-cm','10','--headless','--output',str(scene_files/'empirical.jsonl')])==0
    assert read(scene_files/'empirical.jsonl')[0]['pose_quality']['mapping_method']=='EMPIRICAL_FIXED_HEIGHT_PLANE'


def test_spill_pipeline_dynamic_body_mask_leaves_rest_of_floor_active(scene_files):
    background=scene_files/'baseline.png'
    BackgroundReference(np.full((600,800,3),255,np.uint8),confirmed_clean=True).save(background)
    scene=marker_scene();cv2.circle(scene,(80,500),18,(10,10,230),-1)
    image=scene_files/'robot_and_spill.png';cv2.imwrite(str(image),scene)
    protected=[background,Path(str(background)+'.json'),scene_files/'floor.json',Path('config/target_message_v1.schema.json')]
    before=[p.read_bytes() for p in protected]
    output=scene_files/'spills.jsonl'
    assert main.main(['detect-spills','--image',str(image),'--background',str(background),'--confirm-clean',
        '--calibration',str(scene_files/'floor.json'),'--confirmation-frames','1','--localize','--track-robot',
        '--headless','--output',str(output)])==0
    record=read(output)[0]
    assert record['robot_pose']['tracking_status']=='VISIBLE' and record['robot_exclusion']['pixel_count']>0
    assert len(record['detections'])==1 and record['detections'][0]['category']=='RED_SPILL'
    assert record['localizations'][0]['surface']=='FLOOR'
    assert record['target_events'][0]['target_message']['schema_version']==1
    assert [p.read_bytes() for p in protected]==before


def test_pose_output_cannot_overwrite_calibration(scene_files):
    cal=scene_files/'floor.json';before=cal.read_bytes()
    assert main.main(['track-robot','--image',str(scene_files/'scene.png'),'--calibration',str(cal),
        '--headless','--output',str(cal)])==1
    assert cal.read_bytes()==before
