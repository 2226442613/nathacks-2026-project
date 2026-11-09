"""Module 03 offline/live CLI integration via synthetic files and mocked camera."""
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from tools.generate_localization_demo import generate_demo
from vision import load_calibration,TargetEventStore,GroundCalibration,DualPlaneCalibration


@pytest.fixture
def demo(tmp_path): return generate_demo(tmp_path/'module03')


def common(demo,output):
    return ['detect-spills','--background',str(demo/'baseline.png'),'--calibration',str(demo/'calibration.json'),
            '--confirm-clean','--output',str(output),'--localize']


def records(path): return [json.loads(line) for line in path.read_text().splitlines()]


def test_image_additive_localization_and_persistent_safe_messages(demo):
    output=demo/'image.jsonl'
    protected=[demo/name for name in ('calibration.json','baseline.png','baseline.png.json')]
    before=[path.read_bytes() for path in protected]
    assert main.main(common(demo,output)+['--image',str(demo/'targets.png'),'--headless','--confirmation-frames','1'])==0
    value=records(output)[0]
    assert len(value['detections'])==len(value['localizations'])==3
    by_surface={record['surface']:record for record in value['localizations']}
    assert set(by_surface)=={'FLOOR','WALL','BOUNDARY'}
    assert by_surface['FLOOR']['world_coordinates_cm']==pytest.approx({'x':32.1429,'y':44},abs=.3)
    assert by_surface['WALL']['world_coordinates_cm']==pytest.approx({'x':73.9286,'h':33},abs=.3)
    assert by_surface['BOUNDARY']['world_coordinates_cm'] is None
    assert all(record['localization_status']=='UNVERIFIED' for record in value['localizations'])
    state=TargetEventStore.read(Path(str(output)+'.targets.jsonl'))
    assert len(state)==3 and not any(item['active'] for item in state.values())  # exit retires current observations
    assert all(not item['target_message']['actionable'] for item in state.values())
    assert [path.read_bytes() for path in protected]==before
    # Existing invocation still has the original Module 02 shape.
    assert main.main([arg for arg in common(demo,output) if arg!='--localize']+
                     ['--image',str(demo/'targets.png'),'--headless','--confirmation-frames','1'])==0
    assert 'localizations' not in records(output)[0]


def test_recorded_video_confirmation_ids_and_inactive_events(demo):
    output=demo/'video.jsonl'
    assert main.main(common(demo,output)+['--video',str(demo/'targets.avi'),'--headless'])==0
    frames=records(output)
    assert len(frames)==16 and all(not value['localizations'] for value in frames[:5])
    assert len(frames[5]['localizations'])==3
    assert all(not value['localizations'] for value in frames[13:])
    observed=[value for frame in frames for value in frame['localizations'] if value['surface']=='WALL']
    assert observed and len({value['target_id'] for value in observed})==1
    events=records(Path(str(output)+'.targets.jsonl'))
    assert any(value['event_type']=='INACTIVE' for value in events)
    assert all(value['target_message']['schema_version']==1 for value in events)


@pytest.mark.parametrize('mode',['missing','invalid','resolution'])
def test_unavailable_calibration_reports_invalid_but_visual_pipeline_runs(demo,mode):
    output=demo/'bad.jsonl'
    args=common(demo,output)
    if mode=='missing':
        index=args.index('--calibration'); del args[index:index+2]
    elif mode=='invalid':
        broken=demo/'broken.json'; broken.write_text('{"calibration_version":9}')
        args[args.index('--calibration')+1]=str(broken)
    else:
        floor=GroundCalibration([[20,220],[580,220],[580,420],[20,420]],120,80,800,600)
        cal=DualPlaneCalibration.from_floor_and_wall_top(floor,[[20,20],[580,20]],120,60,'top')
        changed=demo/'other_dimensions.json';cal.save(changed)
        args[args.index('--calibration')+1]=str(changed)
    assert main.main(args+['--image',str(demo/'targets.png'),'--headless','--confirmation-frames','1'])==0
    value=records(output)[0]
    assert value['detections'] and value['localizations']
    assert all(item['localization_status']=='INVALID' and item['world_coordinates_cm'] is None for item in value['localizations'])


def test_live_small_motion_continues_without_operator_prompt(demo,monkeypatch):
    import vision.spill_cli as cli
    scene=cv2.imread(str(demo/'targets.png'))
    shifted=cv2.warpAffine(scene,np.float32([[1,0,1],[0,1,0]]),(600,440))
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=[scene,shifted,shifted]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    monkeypatch.setattr(cv2,'imshow',Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[-1,-1,ord('q')]))
    close=Mock();monkeypatch.setattr(cv2,'destroyAllWindows',close)
    output=demo/'live.jsonl'
    assert main.main(common(demo,output)+['--camera','0','--confirmation-frames','1',
        '--localization-reference',str(demo/'baseline.png')])==0
    frames=records(output)
    assert all(value['diagnostics']['framing']['state']=='NORMAL' for value in frames)
    initial={record['surface']:record for record in frames[0]['localizations']}
    later={record['surface']:record for record in frames[-1]['localizations']}
    for surface in ('FLOOR','WALL'):
        assert later[surface]['world_coordinates_cm']==pytest.approx(initial[surface]['world_coordinates_cm'],abs=.4)
        assert later[surface]['camera_registration']['compensated']
        assert later[surface]['localization_status']=='UNVERIFIED'
    camera.__exit__.assert_called_once();close.assert_called_once()


def test_validation_cli_exports_independent_errors_and_binds_reference(demo):
    cal=load_calibration(demo/'calibration.json')
    points=[]
    for surface,xy in [('FLOOR',(20,10)),('FLOOR',(100,10)),('FLOOR',(60,65)),
                       ('WALL',(20,10)),('WALL',(100,10)),('WALL',(60,50))]:
        point=cal.floor_to_pixel(*xy) if surface=='FLOOR' else cal.wall_to_pixel(*xy)
        points.append({'surface':surface,'pixel':list(point),'expected_cm':list(xy)})
    measured=demo/'measured.json';measured.write_text(json.dumps(points))
    output=demo/'validation.json'
    args=['validate-localization','--calibration',str(demo/'calibration.json'),'--reference',str(demo/'baseline.png'),
          '--points',str(measured),'--confirm-measured','--output',str(output)]
    before=(demo/'calibration.json').read_bytes()
    assert main.main(args)==0
    report=json.loads(output.read_text())
    assert report['physical_validation_passed'] and report['max_error_cm']<1e-5
    assert len(report['points'])==6 and report['reference_id'].startswith('sha256:')
    assert (demo/'calibration.json').read_bytes()==before
    assert main.main(args)==1  # previously measured report is never silently overwritten
    localized=demo/'validated.jsonl'
    assert main.main(common(demo,localized)+['--image',str(demo/'targets.png'),'--headless','--confirmation-frames','1',
        '--localization-reference',str(demo/'baseline.png'),'--localization-validation',str(output)])==0
    for record in records(localized)[0]['localizations']:
        assert not record['actionable']


def test_target_event_path_protects_inputs(demo):
    before=(demo/'baseline.png').read_bytes()
    output=demo/'safe.jsonl'
    assert main.main(common(demo,output)+['--image',str(demo/'targets.png'),'--headless',
        '--target-events',str(demo/'baseline.png')])==1
    assert (demo/'baseline.png').read_bytes()==before


@pytest.mark.parametrize('choice',['continue','reregister'])
def test_large_motion_override_and_new_baseline_do_not_validate_physical_coordinates(demo,monkeypatch,choice):
    import vision.spill_cli as cli
    scene=cv2.imread(str(demo/'targets.png'))
    moved=cv2.warpAffine(scene,np.float32([[1,0,18],[0,1,0]]),(600,440))
    if choice=='continue':
        frames=[moved,moved];keys=[ord('c'),ord('q')]
    else:
        new=moved.copy();cv2.circle(new,(500,330),15,(220,30,20),-1)
        frames=[moved]*5+[new];keys=[ord('r'),ord('y'),-1,-1,-1,ord('q')]
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=frames
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    for name in ('imshow','destroyAllWindows'): monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=keys))
    output=demo/'motion.jsonl'
    before=(demo/'calibration.json').read_bytes()
    assert main.main(common(demo,output)+['--camera','0','--reregister-frames','3','--confirmation-frames','1',
        '--localization-reference',str(demo/'baseline.png')])==0
    values=records(output)
    assert values[0]['localizations'] and values[-1]['localizations']
    estimates=[record for record in values[-1]['localizations'] if record['world_coordinates_cm'] is not None]
    assert estimates
    assert all(record['localization_status']=='UNVERIFIED' and record['geometry_cm'] is not None
               and record['camera_registration']['fallback_active'] for record in estimates)
    assert all(record['camera_registration']['status']=='MOVED' for record in values[-1]['localizations'])
    assert (demo/'calibration.json').read_bytes()==before
    assert all(not item['target_message']['actionable'] for item in records(Path(str(output)+'.targets.jsonl')))


def test_optional_boundary_overlay_and_geometry_persistence(demo):
    output=demo/'geometry.jsonl'
    debug=demo/'geometry_debug'
    assert main.main(common(demo,output)+['--image',str(demo/'targets.png'),'--headless',
        '--confirmation-frames','1','--localization-boundaries','--debug-dir',str(debug)])==0
    value=records(output)[0]
    assert (debug/'frame_000000_annotated.png').exists()
    for r in value['localizations']:
        assert r['image_geometry']['contours']
        if r['surface'] in ('FLOOR','WALL'):
            assert r['geometry_cm']['area_cm2']>0 and r['full_physical_contours_cm']
        else: assert r['geometry_cm'] is None
    state=TargetEventStore.read(Path(str(output)+'.targets.jsonl'))
    for e in state.values():
        assert not e['active'] and e['target_message']['geometry_cm'] is None
        if e['localization']['surface'] in ('FLOOR','WALL'):
            assert e['target_message']['last_known_geometry_cm'] is not None


@pytest.mark.parametrize('enabled',[False,True])
def test_existing_geometry_flag_controls_only_visualization(demo,monkeypatch,enabled):
    import vision.localization_runtime as runtime
    original=runtime.draw_localizations
    calls=[]
    def draw(image,items,**kwargs):
        calls.append((kwargs['boundaries'],[r.to_dict() for r in items]))
        return original(image,items,**kwargs)
    monkeypatch.setattr(runtime,'draw_localizations',draw)
    protected=[demo/name for name in ('calibration.json','baseline.png','baseline.png.json')]
    protected.append(Path('config/target_message_v1.schema.json'))
    before=[p.read_bytes() for p in protected]
    output=demo/'overlay.jsonl'
    args=common(demo,output)+['--image',str(demo/'targets.png'),'--headless',
        '--confirmation-frames','1','--debug-dir',str(demo/'overlay_debug')]
    if enabled: args.append('--localization-boundaries')
    assert main.main(args)==0
    assert calls and calls[0][0] is enabled
    assert records(output)[0]['localizations']==calls[0][1]
    assert all(e['target_message']['schema_version']==1 for e in records(output)[0]['target_events'])
    assert [p.read_bytes() for p in protected]==before


def test_boundary_display_flag_requires_localization(demo,caplog):
    args=common(demo,demo/'bad.jsonl')
    args.remove('--localize')
    assert main.main(args+['--image',str(demo/'targets.png'),'--headless','--localization-boundaries'])==1
    assert '--localization-boundaries requires --localize' in caplog.text
