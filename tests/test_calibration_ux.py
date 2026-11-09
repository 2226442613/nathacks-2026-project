"""Six-click selection, exact endpoint inheritance, retry/save UX, and v2 regression."""
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import CalibrationError, DualPlaneCalibration, GroundCalibration


@pytest.fixture
def floor():
    return GroundCalibration([[200,200],[400,200],[400,400],[200,400]], 120,80,600,600,source=2)


@pytest.mark.parametrize('boundary,indices', [('top',(0,1)),('right',(1,2)),
                                             ('bottom',(3,2)),('left',(0,3))])
@pytest.mark.parametrize('reverse', [False,True])
def test_exact_inherited_endpoints_all_boundaries_and_directions(floor, boundary, indices, reverse):
    expected_bl, expected_br = indices[::-1] if reverse else indices
    base = DualPlaneCalibration.wall_base_corners(floor,boundary,reverse)
    assert np.array_equal(base[0],floor.corners_pixels[expected_bl])
    assert np.array_equal(base[1],floor.corners_pixels[expected_br])
    base[:] = 0
    assert floor.corners_pixels.min() == 200  # Returned endpoints do not alias mutable floor storage.


@pytest.mark.parametrize('boundary,reverse,top', [
    ('top',False,[[200,100],[400,100]]),
    ('right',False,[[500,200],[500,400]]),
    ('bottom',True,[[400,500],[200,500]]),
    ('left',True,[[100,400],[100,200]]),
])
def test_six_point_wall_homography_all_boundaries(floor, boundary, reverse, top, tmp_path):
    width = 120 if boundary in ('top','bottom') else 80
    calibration = DualPlaneCalibration.from_floor_and_wall_top(floor,top,width,60,boundary,reverse)
    base = DualPlaneCalibration.wall_base_corners(floor,boundary,reverse)
    assert np.array_equal(calibration.wall.corners_pixels[3],base[0])
    assert np.array_equal(calibration.wall.corners_pixels[2],base[1])
    assert len(set(map(tuple,np.vstack((floor.corners_pixels,top))))) == 6
    for pixel,world in zip(calibration.wall.corners_pixels,[(0,60),(width,60),(width,0),(0,0)]):
        assert calibration.pixel_to_wall(*pixel) == pytest.approx(world,abs=1e-5)
        assert calibration.wall_to_pixel(*world) == pytest.approx(pixel,abs=1e-5)
    path = tmp_path / f'{boundary}.json'
    calibration.save(path)
    loaded = DualPlaneCalibration.load(path)
    assert loaded.to_dict() == calibration.to_dict()
    assert set(loaded.to_dict()) == {'calibration_version','camera','floor','wall','shared_geometry'}


def test_subpixel_floor_coordinates_are_copied_without_rounding():
    floor = GroundCalibration([[100.125,200.375],[400.625,200.375],
                               [400.625,400.875],[100.125,400.875]],120,80,600,600)
    calibration = DualPlaneCalibration.from_floor_and_wall_top(floor,
        [[100.125,100.25],[400.625,100.25]],120,60,'top')
    assert np.array_equal(calibration.wall.corners_pixels[3],floor.corners_pixels[0])
    assert np.array_equal(calibration.wall.corners_pixels[2],floor.corners_pixels[1])


@pytest.mark.parametrize('top', [[], [[200,100]], [[200,100],[400,100],[400,200]],
    [[200,100],[200,100]], [[200,200],[400,200]],
    [[200,100],[float('nan'),100]], [[400,100],[200,100]], [[200,300],[400,300]]])
def test_invalid_wall_top_selection_rejected(floor, top):
    with pytest.raises(CalibrationError):
        DualPlaneCalibration.from_floor_and_wall_top(floor,top,120,60,'top')


def test_factory_preserves_dimension_and_shared_geometry_validation(floor):
    with pytest.raises(CalibrationError):
        DualPlaneCalibration.from_floor_and_wall_top(floor,[[200,100],[400,100]],120,0,'top')
    with pytest.raises(CalibrationError,match='Wall width'):
        DualPlaneCalibration.from_floor_and_wall_top(floor,[[200,100],[400,100]],110,60,'top')
    approximate = DualPlaneCalibration.from_floor_and_wall_top(floor,[[180,80],[400,130]],120,60,'top')
    assert approximate.quality_report['shared_boundary']['max_error_px'] > 3
    assert approximate.quality_report['warnings']
    with pytest.raises(CalibrationError,match='boundary'):
        DualPlaneCalibration.wall_base_corners(floor,'ceiling')
    with pytest.raises(CalibrationError,match='boolean'):
        DualPlaneCalibration.wall_base_corners(floor,'top','false')


@pytest.fixture
def gui(monkeypatch):
    for name in ('namedWindow','imshow','destroyWindow','destroyAllWindows'):
        monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',lambda *_: 1)


def arguments(tmp_path):
    return main.build_parser().parse_args(['calibrate','--dual-plane','--image','frame.png',
        '--arena-width-cm','120','--arena-depth-cm','80','--wall-width-cm','120','--wall-height-cm','60',
        '--config',str(tmp_path/'camera.json')])


def test_wall_selector_two_click_limit_inherited_points_and_reset(floor, monkeypatch, gui):
    frame = np.zeros((600,600,3),np.uint8)
    base = DualPlaneCalibration.wall_base_corners(floor,'top')
    callback = None
    events = iter(['reset','select','confirm'])
    captured = []
    real_text = cv2.putText

    def text(image,value,*args,**kwargs):
        captured.append(value)
        return real_text(image,value,*args,**kwargs)

    def install(_window,fn):
        nonlocal callback
        callback = fn

    def key(_delay):
        event = next(events)
        if event == 'reset':
            callback(cv2.EVENT_LBUTTONDOWN,300,300,0,None)
            return ord('r')
        if event == 'select':
            for point in [(200,100),(400,100),(10,10),(20,20)]:
                callback(cv2.EVENT_LBUTTONDOWN,*point,0,None)
            return 0
        return 13

    monkeypatch.setattr(cv2,'putText',text)
    monkeypatch.setattr(cv2,'setMouseCallback',install)
    monkeypatch.setattr(cv2,'waitKey',key)
    selected = main.select_corners(frame,'Wall',floor,wall_base=base)
    assert selected == [(200,100),(400,100)]
    assert any('W3 inherited' in value for value in captured)
    assert any('W4 inherited' in value for value in captured)
    assert any('locked' in value for value in captured)


def test_invalid_wall_geometry_can_be_corrected_in_same_window(floor, monkeypatch, gui):
    callback = None
    steps = iter(['bad','reset','valid'])

    def install(_window,fn):
        nonlocal callback
        callback = fn

    def key(_delay):
        step = next(steps)
        if step == 'reset':
            return ord('r')
        for x,y in ([(200,300),(400,300)] if step == 'bad' else [(200,100),(400,100)]):
            callback(cv2.EVENT_LBUTTONDOWN,x,y,0,None)
        return 13

    monkeypatch.setattr(cv2,'setMouseCallback',install)
    monkeypatch.setattr(cv2,'waitKey',key)
    validator = lambda top: DualPlaneCalibration.from_floor_and_wall_top(floor,top,120,60,'top')
    selected = main.select_corners(np.zeros((600,600,3),np.uint8),'Wall',floor,
        wall_base=DualPlaneCalibration.wall_base_corners(floor,'top'),validator=validator)
    assert selected == [(200,100),(400,100)]
    cv2.destroyWindow.assert_called_once()  # The invalid attempt did not tear down the window.


@pytest.mark.parametrize('action', ['floor','wall','reset','dimensions'])
def test_review_corrections_do_not_save_until_final_confirmation(floor, monkeypatch, tmp_path, action):
    args = arguments(tmp_path)
    path = tmp_path/'camera.json'
    path.write_text('previous valid file')
    selections = Mock(side_effect=[floor.corners_pixels.tolist(),[[200,100],[400,100]],
        *([floor.corners_pixels.tolist()] if action in ('floor','reset','dimensions') else []),
        [[200,100],[400,100]]])
    monkeypatch.setattr(main,'select_corners',selections)
    prompts = Mock(side_effect=[120,80,120,60,180,90,180,75])
    monkeypatch.setattr(main,'ask_dimension',prompts if action == 'dimensions' else main.ask_dimension)
    reviews = []

    def review(_frame,calibration,_args):
        assert path.read_text() == 'previous valid file'
        reviews.append(calibration)
        return action if len(reviews) == 1 else 'save'

    monkeypatch.setattr(main,'review_dual_calibration',review)
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),args,2)
    assert len(reviews) == 2
    assert DualPlaneCalibration.load(path).source == 2
    if action == 'reset':
        assert selections.call_args_list[2].kwargs['initial_points'] is None
    if action == 'dimensions':
        assert prompts.call_count == 8
        assert all(call.args[1] is None for call in prompts.call_args_list[4:])
        saved = DualPlaneCalibration.load(path)
        assert saved.floor.width_cm == 180 and saved.wall.height_cm == 75


def test_review_cancel_does_not_touch_existing_file(floor, monkeypatch, tmp_path):
    path = tmp_path/'camera.json'
    path.write_text('previous calibration')
    monkeypatch.setattr(main,'select_corners',Mock(side_effect=[floor.corners_pixels.tolist(),[[200,100],[400,100]]]))
    monkeypatch.setattr(main,'review_dual_calibration',lambda *_: 'cancel')
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),arguments(tmp_path),2)
    assert path.read_text() == 'previous calibration'


def test_floor_edit_from_wall_stage_reinherits_updated_base(floor, monkeypatch, tmp_path):
    changed = floor.corners_pixels + (1,0)
    selections = Mock(side_effect=[floor.corners_pixels.tolist(),'floor',changed.tolist(),[[201,100],[401,100]]])
    monkeypatch.setattr(main,'select_corners',selections)
    monkeypatch.setattr(main,'review_dual_calibration',lambda *_: 'save')
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),arguments(tmp_path),2)
    first_base = selections.call_args_list[1].kwargs['wall_base']
    second_base = selections.call_args_list[3].kwargs['wall_base']
    assert np.array_equal(second_base,first_base + (1,0))
    saved = DualPlaneCalibration.load(tmp_path/'camera.json')
    assert np.array_equal(saved.wall.corners_pixels[3],changed[0])


@pytest.mark.parametrize('key,expected', [(13,'save'),(ord('F'),'floor'),(ord('w'),'wall'),
    (ord('r'),'reset'),(ord('d'),'dimensions'),(ord('q'),'cancel')])
def test_actual_review_controls_and_no_implicit_save(floor, monkeypatch, gui, tmp_path, key, expected):
    calibration = DualPlaneCalibration.from_floor_and_wall_top(floor,[[200,100],[400,100]],120,60,'top')
    monkeypatch.setattr(cv2,'waitKey',lambda *_: key)
    path = tmp_path/'camera.json'
    assert main.review_dual_calibration(np.zeros((600,600,3),np.uint8),calibration,arguments(tmp_path)) == expected
    assert not path.exists()
    assert cv2.imshow.call_count == 3


def test_existing_valid_v2_with_small_base_click_error_still_loads(floor, tmp_path):
    from vision import WallCalibration
    wall = WallCalibration([[201,100],[401,100],[401,200],[201,200]],120,60,600,600,source=2)
    old = DualPlaneCalibration(floor,wall,'top')
    path = tmp_path/'existing_v2.json'
    old.save(path)
    loaded = DualPlaneCalibration.load(path)
    assert loaded.to_dict() == old.to_dict()
    assert np.array_equal(loaded.wall.corners_pixels[3],floor.corners_pixels[0] + (1,0))


def test_invalid_dimensions_can_be_corrected_before_selection(floor, monkeypatch, tmp_path):
    args = arguments(tmp_path)
    args.arena_width_cm = 0
    args.wall_width_cm = 110
    args.wall_height_cm = float('nan')
    answers = iter(['120','120','60'])
    monkeypatch.setattr('builtins.input',lambda *_: next(answers))
    monkeypatch.setattr(main,'select_corners',Mock(side_effect=[floor.corners_pixels.tolist(),[[200,100],[400,100]]]))
    monkeypatch.setattr(main,'review_dual_calibration',lambda *_: 'save')
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),args,2)
    loaded = DualPlaneCalibration.load(tmp_path/'camera.json')
    assert loaded.floor.width_cm == 120
    assert loaded.wall.width_cm == 120
    assert loaded.wall.height_cm == 60
