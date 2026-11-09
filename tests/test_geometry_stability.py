"""MVP background-fit warnings and genuine transform hazards; no hardware."""
from dataclasses import replace
import io
import json

import cv2
import numpy as np
import pytest

import main
from navigation.automatic import SceneScanner
from robot_comm import WireLogger
from vision import BackgroundReference,GroundCalibration
from vision.frame_geometry import GeometryCheck


def corner_fit(*,shift=2.6,rotation=.09,scale=1.01):
    matrix=cv2.getRotationMatrix2D((640,360),rotation,scale)
    matrix[0,2]+=shift
    return np.vstack((matrix,[0.,0.,1.]))


def scanner_with_fits(fits,*,navigation_only=True):
    gray=np.random.default_rng(42).integers(60,180,(720,1280),dtype=np.uint8)
    clean=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);scene=clean.copy()
    cv2.circle(scene,(720,430),15,(20,20,220),-1)
    calibration=GroundCalibration([[100,100],[1179,100],[1179,619],[100,619]],120,60,1280,720)
    source=[]
    def frames():
        frame=scene.copy();source.append(frame);return frame,None
    logger=WireLogger(stream=io.StringIO())
    scanner=SceneScanner(BackgroundReference(clean,confirmed_clean=True),calibration,'synthetic-calibration',
        'config/spill_config.json',frames,physical=True,navigation_only=navigation_only,logger=logger)
    original=scanner.detector.detect
    sequence=iter(fits)
    def detect(*args,**kwargs):
        result=original(*args,**kwargs);fit=next(sequence,None)
        if fit is None:return result
        center=np.array([640,360,1.]);shift=float(np.linalg.norm(fit@center-center))
        angle=float(np.degrees(np.arctan2(fit[1,0],fit[0,0])))
        scale=abs(float(np.hypot(fit[0,0],fit[1,0]))-1)
        return replace(result,geometry=GeometryCheck('MOVED','Camera framing changed beyond safe small-shift alignment',100,shift,angle,scale),
                       reference_to_current=None,background_alignment_transform=fit)
    scanner.detector.detect=detect
    return scanner,source,logger


@pytest.mark.parametrize('fits',[
    [corner_fit()]*3,
    [corner_fit(scale=1.03258235)]*3,
    [corner_fit(shift=7,rotation=0,scale=1),corner_fit(shift=-7,rotation=0,scale=1),None]])
def test_background_fit_limits_are_diagnostic_and_confirmation_still_uses_three_new_frames(fits):
    scanner,frames,log=scanner_with_fits(fits)
    records=scanner.scan()
    assert records and len(frames)==3 and len({id(f) for f in frames})==3
    assert all(r['localization_status']=='UNVERIFIED' and not r['motion_authorized'] for r in records)
    assert '[WARN]' in log.stream.getvalue()
    assert 'GEOMETRY_STABILITY' not in log.stream.getvalue()


def test_background_fit_is_never_used_as_a_current_to_calibration_transform():
    plain,_,_=scanner_with_fits([])
    moved,_,_=scanner_with_fits([corner_fit(scale=1.03258235)]*3,navigation_only=False)
    first=plain.scan()[0];second=moved.scan()[0]
    assert first['position_cm']==pytest.approx(second['position_cm'])
    assert first['geometry_cm']['bounds']==pytest.approx(second['geometry_cm']['bounds'])
    assert second['camera_registration']['reference_id']=='synthetic-calibration'
    assert second['camera_registration']['reference_association']=='OPERATOR_CONFIRMED_CURRENT_CALIBRATION_FRAME'
    assert np.allclose(second['camera_registration']['current_to_calibration'],np.eye(3))
    assert moved.background_id!=second['camera_registration']['reference_id']


@pytest.mark.parametrize('bad',[
    np.zeros((3,3)),np.full((3,3),float('nan')),np.diag([-1.,1.,1.]),
    np.array([[1.,0.,0.],[0.,1.,0.],[-.002,0.,1.]])])
def test_invalid_coordinate_or_background_transform_still_fails_immediately(bad):
    scanner,frames,_=scanner_with_fits([bad])
    with pytest.raises(RuntimeError,match='Detection geometry unsafe'):scanner.scan()
    assert len(frames)==1


def test_invalid_calibration_is_never_downgraded_to_warning():
    scanner,frames,_=scanner_with_fits([corner_fit()])
    scanner.localizer.error='Invalid calibration: test mismatch'
    with pytest.raises(RuntimeError,match='Invalid calibration'):scanner.scan()
    assert len(frames)==1


def test_invalid_transform_stops_auto_before_any_move(tmp_path,monkeypatch):
    initialize=SceneScanner.__init__
    def init(self,*a,**k):
        initialize(self,*a,**k);detect=self.detector.detect
        def moved(*args,**kwargs):
            return replace(detect(*args,**kwargs),background_alignment_transform=np.zeros((3,3)))
        self.detector.detect=moved
    monkeypatch.setattr(SceneScanner,'__init__',init)
    out=tmp_path/'fault'
    assert main.main(['auto-clean','--navigation-only','--output-dir',str(out)])==1
    log=(out/'wire.log').read_text()
    assert '[TX] STOP' in log and '[TX] MOVE,' not in log and '[TX] TARGET,' not in log
    assert json.loads((out/'summary.json').read_text())['state']=='ERROR'
