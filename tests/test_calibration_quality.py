"""Quality is not matrix validity: pixel diagnostics, ruler-based cm checks, explicit consent."""
import copy
import json
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from vision import (CalibrationError, CalibrationQualityThresholds, DualPlaneCalibration,
                    GroundCalibration, PhysicalValidationPoint, WallCalibration, load_validation_points)


@pytest.fixture
def floor():
    return GroundCalibration([[200,200],[400,200],[400,400],[200,400]],108.5,43,600,600)


def pair(floor, top=None, **kwargs):
    return DualPlaneCalibration.from_floor_and_wall_top(floor,
        [[196,96],[400,104]] if top is None else top,108.5,29,'top',**kwargs)


def test_small_manual_error_is_valid_and_not_a_physical_accuracy_claim(floor):
    calibration = pair(floor,[[199,99],[400,101]])
    report = calibration.quality_report
    assert 0 < report['shared_boundary']['max_error_px'] < 3
    assert report['warnings'] == []
    assert report['physical_validation']['status'] == 'NOT_ASSESSED'
    assert report['physical_validation']['max_error_cm'] is None


def test_reported_dimensions_and_warning_level_residual_do_not_reject(floor):
    calibration = pair(floor)
    report = calibration.quality_report
    assert report['shared_boundary']['max_error_px'] == pytest.approx(3.92464678,abs=1e-6)
    assert report['shared_boundary']['rms_error_px'] > 0
    assert 'px' in report['warnings'][0]
    assert 'cm' not in report['warnings'][0]
    assert not report['accepted_warnings']
    assert np.array_equal(calibration.wall.corners_pixels[3],floor.corners_pixels[0])
    assert np.array_equal(calibration.wall.corners_pixels[2],floor.corners_pixels[1])
    assert np.linalg.matrix_rank(calibration.wall.homography) == 3


def test_warning_requires_acceptance_before_save_and_is_recorded(floor,tmp_path):
    calibration = pair(floor)
    path = tmp_path/'camera.json'
    path.write_text('existing')
    with pytest.raises(CalibrationError,match='explicit acceptance'):
        calibration.save(path)
    assert path.read_text() == 'existing'
    calibration.accept_quality_warnings()
    calibration.save(path)
    data = json.loads(path.read_text())
    assert data['calibration_version'] == 2
    assert data['quality']['accepted_warnings'] is True
    assert data['quality']['warnings']
    assert data['quality']['shared_boundary']['max_error_px'] > 3
    with pytest.warns(UserWarning,match='acceptance recorded'):
        loaded = DualPlaneCalibration.load(path)
    assert loaded.to_dict() == data


def test_pixel_warning_threshold_is_configurable_not_cm_based(floor):
    calibration = pair(floor,quality_thresholds=CalibrationQualityThresholds(shared_edge_warning_px=4))
    assert calibration.quality_report['warnings'] == []
    assert calibration.quality_report['physical_validation']['status'] == 'NOT_ASSESSED'
    assert calibration.to_dict()['quality']['thresholds']['shared_edge_warning_px'] == 4


@pytest.mark.parametrize('error,status', [(0,'GOOD'),(2,'GOOD'),(2.1,'WARNING'),(5,'WARNING'),(5.1,'POOR')])
def test_independent_physical_thresholds_are_in_centimeters(floor,error,status):
    point = PhysicalValidationPoint('FLOOR',(300,300),(54.25+error,21.5))
    calibration = pair(floor,[[200,100],[400,100]],validation_points=(point,))
    physical = calibration.quality_report['physical_validation']
    assert physical['status'] == status
    assert physical['max_error_cm'] == pytest.approx(error,abs=1e-10)
    assert physical['points'][0]['predicted_cm'] == pytest.approx((54.25,21.5))
    assert bool(calibration.quality_report['warnings']) == (status != 'GOOD')
    assert calibration.quality_report['shared_boundary']['max_error_px'] < 1e-8


def test_known_wall_measurement_uses_upward_height(floor):
    point = PhysicalValidationPoint('WALL',(300,150),(54.25,17.5))  # Matrix predicts H=14.5 cm.
    calibration = pair(floor,[[200,100],[400,100]],validation_points=(point,))
    physical = calibration.quality_report['physical_validation']
    assert physical['max_error_cm'] == pytest.approx(3)
    assert physical['status'] == 'WARNING'


def test_poor_physical_quality_can_only_be_saved_after_explicit_acceptance(floor,tmp_path):
    point = PhysicalValidationPoint('FLOOR',(300,300),(60.25,21.5))
    calibration = pair(floor,[[200,100],[400,100]],validation_points=(point,))
    path = tmp_path/'poor.json'
    with pytest.raises(CalibrationError):
        calibration.save(path)
    calibration.accept_quality_warnings()
    calibration.save(path)
    with pytest.warns(UserWarning,match='POOR'):
        loaded = DualPlaneCalibration.load(path)
    assert loaded.quality_report['physical_validation']['status'] == 'POOR'
    assert loaded.quality_report['physical_validation']['max_error_cm'] == pytest.approx(6)


def test_new_evidence_invalidates_previous_acceptance(floor):
    calibration = pair(floor)
    calibration.accept_quality_warnings()
    calibration.set_quality_validation(CalibrationQualityThresholds())
    assert calibration.quality_report['warnings']
    assert not calibration.quality_report['accepted_warnings']
    detached = calibration.quality_report
    detached['accepted_warnings'] = True
    assert not calibration.quality_report['accepted_warnings']


def test_exact_geometry_and_old_v2_still_load_without_quality_block(floor,tmp_path):
    calibration = pair(floor,[[200,100],[400,100]])
    assert 'quality' not in calibration.to_dict()
    path = tmp_path/'old.json'
    calibration.save(path)
    assert DualPlaneCalibration.load(path).to_dict() == calibration.to_dict()


@pytest.mark.parametrize('top', [[[200,100],[200,100]],[[200,200],[400,200]],
    [[400,100],[200,100]],[[200,300],[400,300]],[[200,100],[float('nan'),100]]])
def test_quality_thresholds_do_not_override_invalid_wall_geometry(floor,top):
    with pytest.raises(CalibrationError):
        pair(floor,top,quality_thresholds=CalibrationQualityThresholds(shared_edge_warning_px=1e9))


def test_wrong_endpoints_and_dimensions_still_reject(floor):
    wall = WallCalibration([[210,100],[410,100],[410,200],[210,200]],108.5,29,600,600)
    with pytest.raises(CalibrationError,match='base corners'):
        DualPlaneCalibration(floor,wall,'top',quality_thresholds=CalibrationQualityThresholds(1e9))
    for width,height in [(0,29),(108.5,-1),(108.5,float('inf'))]:
        with pytest.raises(CalibrationError):
            DualPlaneCalibration.from_floor_and_wall_top(floor,[[200,100],[400,100]],width,height,'top')


def test_accepted_metadata_cannot_override_invalid_matrix(floor):
    calibration = pair(floor)
    calibration.accept_quality_warnings()
    data = calibration.to_dict()
    data['wall']['homography'] = [[0,0,0]]*3
    with pytest.raises(CalibrationError,match='homography'):
        DualPlaneCalibration.from_dict(data)


@pytest.mark.parametrize('values', [{'shared_edge_warning_px':-1},{'physical_good_cm':float('nan')},
    {'physical_warning_cm':1},{'shared_edge_warning_px':float('inf')}])
def test_invalid_quality_settings_rejected(values):
    with pytest.raises(CalibrationError):
        CalibrationQualityThresholds(**values)


def test_independent_point_file_and_malformed_data(floor,tmp_path):
    path = tmp_path/'validation.json'
    records = [{'surface':'FLOOR','pixel':[300,300],'expected_cm':[54.25,21.5]}]
    path.write_text(json.dumps(records))
    loaded = load_validation_points(path)
    assert loaded[0].surface.value == 'FLOOR'
    assert pair(floor,validation_points=loaded).validation_points == loaded
    for damaged in [[], {'bad':'data'}, [{'surface':'FLOOR','pixel':[float('nan'),0],'expected_cm':[0,0]}]]:
        path.write_text(json.dumps(damaged))
        with pytest.raises(CalibrationError):
            load_validation_points(path)
    with pytest.raises(CalibrationError,match='outside'):
        pair(floor,validation_points=(PhysicalValidationPoint('FLOOR',(300,300),(200,21)),))


@pytest.mark.parametrize('accept', [True,False])
def test_warning_preview_blocks_enter_requires_a_and_displays_units(floor,monkeypatch,tmp_path,accept):
    calibration = pair(floor)
    for name in ('imshow','destroyWindow'):
        monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',lambda *_: 1)
    keys = iter([13,ord('A') if accept else ord('q')])
    monkeypatch.setattr(cv2,'waitKey',lambda *_: next(keys))
    messages = []
    original = main.calibration_message
    def capture(image,text,**kwargs):
        messages.append(text)
        return original(image,text,**kwargs)
    monkeypatch.setattr(main,'calibration_message',capture)
    args = main.build_parser().parse_args(['calibrate','--dual-plane','--arena-width-cm','108.5',
        '--arena-depth-cm','43','--wall-width-cm','108.5','--wall-height-cm','29',
        '--config',str(tmp_path/'camera.json')])
    result = main.review_dual_calibration(np.zeros((600,600,3),np.uint8),calibration,args)
    assert result == ('save' if accept else 'cancel')
    assert calibration.quality_report['accepted_warnings'] == accept
    assert any('3.925 px' in text and 'NOT ASSESSED' in text for text in messages)
    assert cv2.imshow.call_count == 4  # Geometry, two rectified planes, quality report.
    assert not (tmp_path/'camera.json').exists()  # Review alone does not write.


def test_full_six_point_workflow_accepts_and_records_warning(floor,monkeypatch,tmp_path):
    args = main.build_parser().parse_args(['calibrate','--dual-plane','--arena-width-cm','108.5',
        '--arena-depth-cm','43','--wall-width-cm','108.5','--wall-height-cm','29',
        '--config',str(tmp_path/'camera.json')])
    monkeypatch.setattr(main,'select_corners',Mock(side_effect=[floor.corners_pixels.tolist(),[[196,96],[400,104]]]))
    for name in ('imshow','destroyWindow'):
        monkeypatch.setattr(cv2,name,Mock())
    monkeypatch.setattr(cv2,'getWindowProperty',lambda *_: 1)
    keys = iter([13,ord('a')])
    monkeypatch.setattr(cv2,'waitKey',lambda *_: next(keys))
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),args,0)
    with pytest.warns(UserWarning,match='acceptance recorded'):
        loaded = DualPlaneCalibration.load(tmp_path/'camera.json')
    assert loaded.quality_report['accepted_warnings']
    assert loaded.floor.width_cm == 108.5 and loaded.wall.height_cm == 29


def test_cli_quality_thresholds_and_independent_points_are_applied(floor,monkeypatch,tmp_path):
    points = tmp_path/'measured.json'
    points.write_text(json.dumps([{'surface':'FLOOR','pixel':[300,300],'expected_cm':[57.25,21.5]}]))
    output = tmp_path/'camera.json'
    args = main.build_parser().parse_args(['calibrate','--dual-plane','--arena-width-cm','108.5',
        '--arena-depth-cm','43','--wall-width-cm','108.5','--wall-height-cm','29',
        '--config',str(output),'--validation-points',str(points),
        '--shared-edge-warning-px','6','--physical-good-cm','1','--physical-warning-cm','4'])
    monkeypatch.setattr(main,'select_corners',Mock(side_effect=[floor.corners_pixels.tolist(),[[200,100],[400,100]]]))
    def review(_frame,calibration,_args):
        assert calibration.quality_thresholds == CalibrationQualityThresholds(6,1,4)
        assert calibration.quality_report['physical_validation']['status'] == 'WARNING'
        assert calibration.quality_report['physical_validation']['max_error_cm'] == pytest.approx(3)
        calibration.accept_quality_warnings()
        return 'save'
    monkeypatch.setattr(main,'review_dual_calibration',review)
    main.run_dual_calibration(np.zeros((600,600,3),np.uint8),args,0)
    with pytest.warns(UserWarning,match='WARNING independent physical'):
        loaded = DualPlaneCalibration.load(output)
    assert loaded.quality_thresholds == CalibrationQualityThresholds(6,1,4)


@pytest.mark.parametrize('damage',['accepted_type','nonfinite_summary','nonfinite_point'])
def test_invalid_saved_quality_metadata_is_rejected(floor,damage):
    point = PhysicalValidationPoint('FLOOR',(300,300),(57.25,21.5))
    calibration = pair(floor,validation_points=(point,))
    calibration.accept_quality_warnings()
    data = calibration.to_dict()
    if damage == 'accepted_type':
        data['quality']['accepted_warnings'] = 'yes'
    elif damage == 'nonfinite_summary':
        data['quality']['shared_boundary']['max_error_px'] = float('nan')
    else:
        data['quality']['physical_validation']['points'][0]['pixel'][0] = float('inf')
    with pytest.raises(CalibrationError):
        DualPlaneCalibration.from_dict(data)
