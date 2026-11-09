"""Optional geometry presentation; no changes to physical calculations."""
from copy import deepcopy
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from vision.localization_runtime import draw_localizations, _geometry_overlay_lines, _wrap_overlay_line


def record(surface='FLOOR',**changes):
    wall=surface=='WALL'
    result=SimpleNamespace(target_id=7,category='RED_SPILL',surface=surface,
        localization_status='UNVERIFIED',world_coordinates_cm={'x':12.3,'h' if wall else 'y':23.4},
        geometry_cm={'width_cm':10.2,'height_cm' if wall else 'depth_cm':4.5,'area_cm2':37.8},
        geometry_status='UNVERIFIED',geometry_reason='No measured physical validation',reason='test reason',
        pixel_anchor=None,image_geometry={'contours':[{'kind':'outer','points':[[500.5,350.5],[520.5,350.5],[520.5,360.5],[510.5,360.5],[510.5,370.5],[500.5,370.5]]}]})
    result.__dict__.update(changes)
    return result


@pytest.mark.parametrize('surface,axis,dimension',[('FLOOR','Y','D'),('WALL','H','H')])
@pytest.mark.parametrize('category',['RED_SPILL','NON_RED_SPILL'])
def test_floor_wall_required_fields(surface,axis,dimension,category):
    lines=' '.join(_geometry_overlay_lines(record(surface,category=category)))
    for text in ('#7',category,surface,'X=12.3',axis+'=23.4','W=10.2 cm',dimension+'=4.5 cm','area ~37.8 cm^2','UNVERIFIED'):
        assert text in lines
    assert ('depth' not in lines) if surface=='WALL' else ('H=' not in lines)


@pytest.mark.parametrize('surface,status',[('BOUNDARY','UNVERIFIED'),('UNKNOWN','UNVERIFIED'),('FLOOR','INVALID')])
def test_unavailable_reason_no_invented_dimensions(surface,status):
    # Even a stale geometry field must not override invalid/ambiguous ownership.
    r=record(surface,localization_status=status,geometry_reason='Registration unresolved: no reliable transform')
    lines=' '.join(_geometry_overlay_lines(r))
    assert 'Geometry unavailable: Registration unresolved: no reliable transform' in lines
    assert all(text not in lines for text in ('W=','D=','H=','area ~','X='))


def test_geometry_trust_is_distinct_from_verified_center():
    lines=' '.join(_geometry_overlay_lines(record(localization_status='VERIFIED')))
    assert '| VERIFIED' in lines and 'geometry UNVERIFIED' in lines


def test_draws_actual_irregular_contour_with_subpixel_coordinates(monkeypatch):
    r=record();image=np.zeros((440,600,3),np.uint8)
    calls=[];original=cv2.polylines
    def draw(*args,**kwargs):
        calls.append((args,kwargs));return original(*args,**kwargs)
    monkeypatch.setattr(cv2,'polylines',draw)
    before=deepcopy(r.__dict__)
    draw_localizations(image,[r],top=30,boundaries=True)
    args,kwargs=calls[0]
    assert len(args[1][0])==6 and kwargs['shift']==1
    assert args[1][0].tolist()==(np.asarray(r.image_geometry['contours'][0]['points'])*2).astype(int).tolist()
    assert image[371,505].any()  # true bottom L edge
    assert not image[371,520].any()  # empty bounding-box corner
    assert r.__dict__==before


def test_wrapping_and_rows_fit_without_text_overlap(monkeypatch):
    image=np.zeros((440,600,3),np.uint8)
    drawn=[];original=cv2.putText
    def draw(*args,**kwargs):
        drawn.append((args[1],args[2],args[4]));return original(*args,**kwargs)
    monkeypatch.setattr(cv2,'putText',draw)
    targets=[record(),record('WALL',target_id=8),record('BOUNDARY',target_id=9,geometry_cm=None,
        world_coordinates_cm=None,geometry_reason='Significant overlap with both planes or shared-edge uncertainty band')]
    draw_localizations(image,targets,top=100,boundaries=True)
    ys=[xy[1] for _,xy,_ in drawn]
    assert all(b-a>=16 for a,b in zip(ys,ys[1:]))
    assert all(cv2.getTextSize(text,0,scale,1)[0][0]+xy[0]<600 and xy[1]<440 for text,xy,scale in drawn)
    assert all(any(f'#{target.target_id}' in text for text,_,_ in drawn) for target in targets)
    assert any('shared-edge' in text for text,_,_ in drawn)
    assert all(cv2.getTextSize(text,0,.4,1)[0][0]<=100 for text in _wrap_overlay_line('x'*150,100))


def test_default_overlay_does_not_draw_geometry(monkeypatch):
    drawn=[]
    monkeypatch.setattr(cv2,'polylines',lambda *args,**kwargs: pytest.fail('default must not draw contour'))
    monkeypatch.setattr(cv2,'putText',lambda image,text,*args,**kwargs: drawn.append(text))
    draw_localizations(np.zeros((440,600,3),np.uint8),[record()],top=100)
    assert drawn[0]=='#7 RED_SPILL | FLOOR | X=12.3 Y=23.4 cm | UNVERIFIED'
    assert all('area' not in text for text in drawn)


def test_many_targets_explicit_overflow_instead_of_clipped_rows(monkeypatch):
    drawn=[]
    monkeypatch.setattr(cv2,'putText',lambda image,text,*args,**kwargs: drawn.append(text))
    draw_localizations(np.zeros((100,600,3),np.uint8),[record(target_id=i) for i in range(20)],top=20,boundaries=True)
    assert any('overlay space limited' in text for text in drawn)
