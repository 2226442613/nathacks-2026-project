"""Non-blocking framing is an explicit demo policy, never physical verification."""
from dataclasses import replace
import json
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import SurfaceLocalizer, CameraRegistration, SpillCategory, TargetEventStore, DetectorConfig
from vision.frame_geometry import GeometryCheck
from vision.framing_session import FramingSession
from vision.localization_runtime import LocalizationRegistration
from test_localization import dual, observation, identity, measured_report
from test_localization_cli import demo, common, records


@pytest.mark.parametrize('surface',['FLOOR','WALL'])
@pytest.mark.parametrize('warning',['over_limit','missing','uncertain','association'])
def test_exact_non_red_target_uses_current_pixel_fallback(dual,surface,warning):
    pixel=dual.floor_to_pixel(50,30) if surface=='FLOOR' else dual.wall_to_pixel(50,30)
    target,mask=observation(pixel)
    target=replace(target,category=SpillCategory.NON_RED_SPILL)
    if warning=='over_limit':
        reg=CameraRegistration.from_reference_to_current([[1,0,18],[0,1,0]],(800,600),override_active=True)
        assert reg.reason=='Registration exceeds safe small-motion limit'
    else:
        reg=CameraRegistration(reason={'missing':'No calibration view declared','uncertain':'No consistent fit',
                                       'association':'Baseline/calibration association unknown'}[warning])
    model=SurfaceLocalizer(dual,physical_evidence=measured_report(dual))
    result=model.localize(target,mask,frame_size=(800,600),registration=reg)
    nominal=model.localize(target,mask,frame_size=(800,600),registration=identity())
    assert result.category=='NON_RED_SPILL' and result.surface==surface
    assert result.localization_status==result.geometry_status=='UNVERIFIED'
    assert result.world_coordinates_cm==pytest.approx(nominal.world_coordinates_cm)
    assert result.geometry_cm['bounds']==pytest.approx(nominal.geometry_cm['bounds'])
    assert result.geometry_cm['area_cm2']==pytest.approx(nominal.geometry_cm['area_cm2'])
    assert result.calibration_pixel_anchor==result.pixel_anchor
    assert result.camera_registration['fallback_active']
    assert result.camera_registration['mapping_mode']=='CURRENT_FRAME_FALLBACK'
    assert result.camera_registration['registration_warning']==reg.reason
    assert result.overlap_coordinate_frame=='current_frame_loaded_calibration_polygons'
    assert any(reg.reason in warning for warning in result.warnings)
    assert reg.reason in result.geometry_reason
    message=result.target_message()
    assert message['schema_version']==1 and message['geometry_cm']['trust_state']=='UNVERIFIED'
    assert not message['physically_verified'] and not message['actionable'] and not message['motion_authorized']
    json.dumps(result.to_dict(),allow_nan=False)


def test_fallback_does_not_verify_even_with_bound_measured_report(dual):
    target,mask=observation(dual.floor_to_pixel(60,30))
    model=SurfaceLocalizer(dual,physical_evidence=measured_report(dual))
    reg=replace(identity('sha256:measured-view'),current_to_calibration=None)
    r=model.localize(target,mask,frame_size=(800,600),registration=reg)
    assert r.surface=='FLOOR' and r.geometry_cm
    assert r.localization_status==r.geometry_status=='UNVERIFIED'


@pytest.mark.parametrize('kind',['boundary','outside'])
def test_fallback_does_not_hide_genuine_surface_ambiguity(dual,kind):
    pixel=dual.wall_to_pixel(50,0) if kind=='boundary' else (700,500)
    target,mask=observation(pixel,radius=8)
    r=SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=CameraRegistration())
    assert r.surface==('BOUNDARY' if kind=='boundary' else 'UNKNOWN')
    assert r.world_coordinates_cm is None and r.geometry_cm is None and r.image_geometry
    assert r.reason


def test_unknown_baseline_association_is_not_a_coordinate_gate(dual):
    image=np.full((600,800,3),100,np.uint8)
    bridge=LocalizationRegistration(image,image,baseline_unverified=True,config=DetectorConfig())
    reg=bridge.for_frame(image,Mock())
    assert reg.current_to_calibration is None
    target,mask=observation(dual.wall_to_pixel(50,30))
    r=SurfaceLocalizer(dual).localize(target,mask,frame_size=(800,600),registration=reg)
    assert r.surface=='WALL' and r.geometry_cm and r.geometry_status=='UNVERIFIED'


def test_reported_live_scenario_no_reference_or_c_required(demo,monkeypatch):
    import vision.spill_cli as cli
    import vision.localization_runtime as runtime
    baseline=demo/'baseline.png'
    scene=cv2.imread(str(baseline))
    cv2.circle(scene,(170,330),17,(20,210,210),-1)  # NON_RED_SPILL wholly inside FLOOR.
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=[scene]*3
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    shown=Mock();monkeypatch.setattr(cv2,'imshow',shown)
    monkeypatch.setattr(cv2,'destroyAllWindows',Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[-1,-1,ord('q')]))
    real=runtime.LocalizationRegistration
    def independent_registration(*args,**kwargs):
        bridge=real(*args,**kwargs)
        bridge.reuse_detection=False
        # Exercise the real bridge -> CameraRegistration limit -> localizer path.
        def check(frame):
            bridge.guard.transform=np.float64([[1,0,18],[0,1,0]])
            return GeometryCheck('MATCHED','Synthetic independent fit',30,18.,0.,0.)
        bridge.guard.check=check
        return bridge
    monkeypatch.setattr(runtime,'LocalizationRegistration',independent_registration)
    texts=[];put=cv2.putText
    def text(image,value,*args,**kwargs):
        texts.append(value);return put(image,value,*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',text)
    protected=[baseline,Path(str(baseline)+'.json'),demo/'calibration.json',Path('config/target_message_v1.schema.json')]
    before=[p.read_bytes() for p in protected]
    output=demo/'reported_scenario.jsonl'
    assert main.main(common(demo,output)+['--camera','0','--confirmation-frames','1','--localization-boundaries'])==0
    values=records(output)
    assert len(values)==shown.call_count==3
    ids=[]
    for value in values:
        assert not value['diagnostics']['detection_paused']
        r=value['localizations'][0];ids.append(r['target_id'])
        assert r['category']=='NON_RED_SPILL' and r['surface']=='FLOOR'
        assert r['world_coordinates_cm']==pytest.approx({'x':32.142857,'y':44},abs=.3)
        assert r['localization_status']==r['geometry_status']=='UNVERIFIED'
        assert r['camera_registration']['status']=='MOVED'
        assert r['camera_registration']['registration_warning']=='Registration exceeds safe small-motion limit'
        assert r['geometry_cm']['contours'] and r['geometry_cm']['area_cm2']>0
    assert len(set(ids))==1
    assert not any(any(banner in t for banner in ('OVERRIDE_ACTIVE','WAITING_FOR_OPERATOR','CALIBRATION MAY BE INVALID')) for t in texts)
    assert any('UNVERIFIED' in t for t in texts) and any('W=' in t for t in texts)
    assert [p.read_bytes() for p in protected]==before
    events=records(Path(str(output)+'.targets.jsonl'))
    assert any(e['event_type']=='UPSERT' and e['target_message']['geometry_cm'] for e in events)
    assert events[-1]['event_type']=='INACTIVE' and events[-1]['target_message']['last_known_geometry_cm']
    assert not TargetEventStore.read_current_messages(Path(str(output)+'.targets.jsonl'))
    camera.__exit__.assert_called_once()


def test_detailed_legacy_banners_are_debug_only(monkeypatch):
    session=FramingSession(state='OVERRIDE_ACTIVE',override_active=True,calibration_may_be_invalid=True,reason='Moved')
    texts=[]
    monkeypatch.setattr(cv2,'putText',lambda image,text,*args,**kwargs:texts.append(text))
    session.draw(np.zeros((720,1280,3),np.uint8))
    assert not texts
    session.draw(np.zeros((720,1280,3),np.uint8),debug=True)
    assert any('OVERRIDE_ACTIVE' in t for t in texts)
    assert any('CALIBRATION MAY BE INVALID' in t for t in texts)


def test_source_index_warning_also_keeps_estimates(demo,monkeypatch):
    import vision.spill_cli as cli
    scene=cv2.imread(str(demo/'targets.png'))
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=[scene,None]
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    output=demo/'other_index.jsonl'
    assert main.main(common(demo,output)+['--camera','1','--headless','--confirmation-frames','1'])==0
    values=records(output)
    estimates=[r for r in values[0]['localizations'] if r['surface'] in ('FLOOR','WALL')]
    assert estimates and all(r['world_coordinates_cm'] and r['geometry_cm'] for r in estimates)
    assert all(r['localization_status']=='UNVERIFIED' and r['camera_registration']['fallback_active'] for r in estimates)
    assert all(any('source index differs' in w for w in r['warnings']) for r in estimates)


def test_camera_disconnection_is_still_an_explicit_failure(demo,monkeypatch,caplog):
    import vision.spill_cli as cli
    from vision import CameraError
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False)
    camera.read.side_effect=CameraError('Camera disconnected')
    monkeypatch.setattr(cli,'CameraManager',Mock(return_value=camera))
    assert main.main(common(demo,demo/'disconnected.jsonl')+['--camera','0','--headless'])==1
    assert 'Camera disconnected' in caplog.text
    camera.__exit__.assert_called_once()
