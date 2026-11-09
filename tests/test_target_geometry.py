"""Raster-footprint mathematics, not claims of real-camera accuracy."""
from dataclasses import replace
import json

import cv2
import numpy as np
import pytest

from vision import GroundCalibration, WallCalibration, SurfaceLocalizer, CameraRegistration, LocalizationConfig, TargetEventStore
from test_localization import observation, dual, identity
from vision.target_geometry import image_geometry, signed_area


@pytest.fixture
def floor():
    return GroundCalibration([[0,0],[799,0],[799,599],[0,599]],799,599,800,600)


def locate(cal,mask,**kwargs):
    target,mask=observation(mask=mask)
    return SurfaceLocalizer(cal,kwargs.pop('config',None)).localize(target,mask,frame_size=(800,600),registration=kwargs.pop('registration',identity()),**kwargs)


def rectangle():
    mask=np.zeros((600,800),np.uint8)
    mask[100:110,200:220]=255
    return mask


def test_rectangle_exact_pixel_cells_extents_and_area(floor):
    r=locate(floor,rectangle());g=r.geometry_cm
    assert g['bounds']==pytest.approx(dict(min_x=199.5,max_x=219.5,min_y=99.5,max_y=109.5))
    assert g['width_cm']==pytest.approx(20) and g['depth_cm']==pytest.approx(10)
    assert g['area_cm2']==pytest.approx(200)
    assert len(g['contour'])==4
    assert r.geometry_status=='UNVERIFIED'
    assert len(r.full_physical_contours_cm[0]['points'])==60


@pytest.mark.parametrize('wall',[False,True])
@pytest.mark.parametrize('irregular',[False,True])
def test_surface_and_irregular_extents(dual,wall,irregular):
    surface='WALL' if wall else 'FLOOR'
    pixel=dual.wall_to_pixel(50,30) if wall else dual.floor_to_pixel(50,40)
    x,y=map(round,pixel)
    mask=np.zeros((600,800),np.uint8);mask[y:y+6,x:x+8]=255
    if irregular: mask[y+3:y+6,x+4:x+8]=0
    r=locate(dual,mask);g=r.geometry_cm
    assert r.surface==surface and g is not None
    assert ('height_cm' if wall else 'depth_cm') in g
    assert ('min_h' if wall else 'min_y') in g['bounds']
    assert len(g['contour'])>= (6 if irregular else 4)
    assert g['area_cm2']>0


@pytest.mark.parametrize('shape',['hole','islands','diagonal','thin','single'])
def test_mask_topology_and_exact_raster_area(floor,shape):
    mask=np.zeros((600,800),np.uint8)
    if shape=='hole':
        mask[100:120,200:230]=255;mask[105:115,210:220]=0
    elif shape=='islands': mask[100:105,200:205]=255;mask[110:113,210:213]=255
    elif shape=='diagonal': mask[100,200]=255;mask[101,201]=255
    elif shape=='thin': mask[100,200:240]=255
    else: mask[100,200]=255
    r=locate(floor,mask);g=r.geometry_cm
    assert g['area_cm2']==pytest.approx(np.count_nonzero(mask))
    if shape=='hole': assert sorted(r['kind'] for r in g['contours'])==['hole','outer']
    if shape in ('islands','diagonal'): assert len(g['contours'])==2 and g['contour'] is None
    restored=np.zeros_like(mask)
    for y,a,b in r.image_geometry['mask_runs_y_xstart_xend_exclusive']: restored[y,a:b]=255
    assert np.array_equal(mask,restored)
    assert sum(signed_area(ring['points']) for ring in r.image_geometry['contours'])==np.count_nonzero(mask)


def test_perspective_uses_actual_vertices(dual):
    pixel=dual.floor_to_pixel(50,40);x,y=map(round,pixel)
    mask=np.zeros((600,800),np.uint8);mask[y:y+12,x:x+18]=255
    r=locate(dual,mask)
    expected=[dual.pixel_to_floor(*p) for p in r.image_geometry['contours'][0]['points']]
    assert np.asarray(r.full_physical_contours_cm[0]['points'])==pytest.approx(np.asarray(expected))
    assert r.geometry_cm['area_cm2']==pytest.approx(abs(signed_area(expected)))
    # Independent transformed four cell-edge corners give the same polygon area.
    corners=[dual.pixel_to_floor(u,v) for u,v in [(x-.5,y-.5),(x+17.5,y-.5),(x+17.5,y+11.5),(x-.5,y+11.5)]]
    assert r.geometry_cm['area_cm2']==pytest.approx(abs(signed_area(corners)))


def test_registration_inverse_once(floor):
    reference=locate(floor,rectangle())
    shifted=np.roll(rectangle(),1,axis=1)
    registration=CameraRegistration.from_reference_to_current([[1,0,1],[0,1,0]],(800,600))
    current=locate(floor,shifted,registration=registration)
    assert current.geometry_cm['bounds']==pytest.approx(reference.geometry_cm['bounds'])
    assert current.geometry_cm['area_cm2']==pytest.approx(200)
    assert current.geometry_status=='UNVERIFIED'


@pytest.mark.parametrize('tolerance',[0,.02,.2,5])
def test_simplification_protects_narrow_regions(floor,tolerance):
    mask=rectangle();mask[90:100,209]=255
    r=locate(floor,mask,config=LocalizationConfig(contour_tolerance_cm=tolerance))
    assert r.geometry_cm['area_cm2']==pytest.approx(210)
    assert len(r.geometry_cm['contour'])>=8
    assert r.geometry_cm['effective_tolerance_cm']<=.25


def test_payload_limit_omits_instead_of_truncating(floor):
    mask=rectangle();mask[90:100,209]=255
    r=locate(floor,mask,config=LocalizationConfig(contour_max_points=4))
    assert r.geometry_cm['payload_status']=='CONTOURS_OMITTED_LIMIT'
    assert r.geometry_cm['contours'] is None
    assert r.full_physical_contours_cm and r.image_geometry


@pytest.mark.parametrize('mode',['boundary','unknown','invalid'])
def test_ambiguous_or_invalid_never_publishes_geometry(dual,mode):
    if mode=='boundary':
        target,mask=observation(dual.wall_to_pixel(50,0),radius=8)
    else: target,mask=observation((700,500) if mode=='unknown' else dual.floor_to_pixel(50,40))
    localizer=SurfaceLocalizer(None if mode=='invalid' else dual)
    r=localizer.localize(target,mask,frame_size=(800,600),registration=CameraRegistration() if mode=='unresolved' else identity())
    assert r.image_geometry and r.geometry_cm is None and r.geometry_reason
    assert r.target_message()['geometry_cm'] is None


def test_geometry_message_additive_and_events(floor,tmp_path):
    r=locate(floor,rectangle());msg=r.target_message()
    old=json.loads(open('config/target_message_v1.schema.json').read())
    assert all(key in msg for key in old['required'])
    assert msg['schema_version']==1 and msg['geometry_cm']['area_cm2']==pytest.approx(200)
    assert 'image_geometry' not in msg and 'full_physical_contours_cm' not in msg
    json.dumps(msg,allow_nan=False)
    store=TargetEventStore(tmp_path/'events.jsonl',session_id='geometry')
    store.update([r,replace(r,target_id=2)])
    changed=locate(floor,np.roll(rectangle(),2,axis=1))
    events=store.update([changed]);assert any(e['event_type']=='INACTIVE' for e in events)
    assert store.states['geometry:0:1']['target_message']['geometry_cm']['bounds']['min_x']==pytest.approx(201.5)
    store.update([])
    states=TargetEventStore.read(tmp_path/'events.jsonl')
    assert all(not e['active'] and e['target_message']['geometry_cm'] is None for e in states.values())
    assert states['geometry:0:1']['target_message']['last_known_geometry_cm']['area_cm2']==pytest.approx(200)
    assert states['geometry:0:1']['localization']['full_physical_contours_cm']


def test_nonfinite_physical_boundary_rejected(floor):
    from vision.target_geometry import physical_geometry
    with pytest.raises(ValueError,match='Non-finite'):
        physical_geometry(image_geometry(rectangle()),np.eye(3),lambda u,v:(float('nan'),v),surface='FLOOR',width=799,height=599,boundary_tolerance=.5)


def test_full_boundary_verification_scoped_to_measured_hull(dual):
    from test_localization import measured_report
    model=SurfaceLocalizer(dual,physical_evidence=measured_report(dual))
    target,mask=observation(dual.floor_to_pixel(60,30),radius=2)
    r=model.localize(target,mask,frame_size=(800,600),registration=identity('sha256:measured-view'))
    assert r.geometry_status=='VERIFIED'
    # Centroid is inside the triangle, but part of a broad component is outside.
    target,mask=observation(dual.floor_to_pixel(60,15),radius=38)
    r=model.localize(target,mask,frame_size=(800,600),registration=identity('sha256:measured-view'))
    assert r.geometry_status!='VERIFIED'


def test_horizon_crossing_is_rejected():
    from vision.target_geometry import physical_geometry
    backward=np.array([[1,0,0],[0,1,0],[1,0,-210.]])
    with pytest.raises(ValueError,match='horizon'):
        physical_geometry(image_geometry(rectangle()),backward,lambda u,v:(u,v),surface='FLOOR',width=799,height=599,boundary_tolerance=.5)


@pytest.mark.parametrize('settings',[{'contour_tolerance_cm':float('nan')},{'contour_tolerance_cm':-1},
    {'contour_max_points':3},{'contour_max_points':True},{'contour_max_bytes':100}])
def test_geometry_configuration_validation(settings):
    with pytest.raises(ValueError): LocalizationConfig(**settings)


def test_byte_budget_and_inactive_message(floor):
    mask=rectangle()
    r=locate(floor,mask,config=LocalizationConfig(contour_max_bytes=1024))
    assert len(json.dumps(r.geometry_cm).encode())<=1024
    message=r.target_message(active=False)
    assert message['geometry_cm'] is None and message['last_known_geometry_cm'] is not None
    assert replace(r,surface='BOUNDARY').target_message()['geometry_cm'] is None
