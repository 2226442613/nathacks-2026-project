"""Hardware-free dual-plane geometry, persistence, and compatibility checks."""
import copy
import json

import numpy as np
import pytest

from vision import (CalibrationError, GroundCalibration, WallCalibration, DualPlaneCalibration,
                    SurfaceType, RobotReachGeometry, load_calibration)


def project(x, y, z):
    """Synthetic oblique projective camera, with floor Z=0 and wall Y=0."""
    return ((2*x + .3*y + .1*z + 100) / (1 + .002*y + .001*z),
            (.1*x + 2*y - 2*z + 300) / (1 + .002*y + .001*z))


@pytest.fixture
def dual():
    floor = GroundCalibration([project(0, 0, 0), project(120, 0, 0),
                               project(120, 80, 0), project(0, 80, 0)], 120, 80, 800, 600)
    wall = WallCalibration([project(0, 0, 60), project(120, 0, 60),
                            project(120, 0, 0), project(0, 0, 0)], 120, 60, 800, 600)
    return DualPlaneCalibration(floor, wall, 'top')


def test_oblique_floor_transform_and_inverse(dual):
    for x, y in [(0, 0), (120, 80), (47, 33)]:
        pixel = project(x, y, 0)
        assert dual.pixel_to_floor(*pixel) == pytest.approx((x, y), abs=1e-4)
        assert dual.floor_to_pixel(x, y) == pytest.approx(pixel, abs=1e-4)


def test_oblique_wall_height_and_inverse(dual):
    for x, h in [(0, 0), (120, 60), (47, 33), (60, 0)]:
        pixel = project(x, 0, h)
        assert dual.pixel_to_wall(*pixel) == pytest.approx((x, h), abs=1e-4)
        assert dual.wall_to_pixel(x, h) == pytest.approx(pixel, abs=1e-4)
        assert dual.wall_to_pixel(*dual.pixel_to_wall(*pixel)) == pytest.approx(pixel, abs=1e-7)
    assert dual.pixel_to_wall(*project(60, 0, 55))[1] > dual.pixel_to_wall(*project(60, 0, 10))[1]
    assert not np.allclose(dual.floor.homography, dual.wall.homography)


@pytest.mark.parametrize('boundary,reversed_x,corners,expected', [
    ('top', False, [[200,100],[400,100],[400,200],[200,200]], (30,0)),
    ('bottom', True, [[400,500],[200,500],[200,400],[400,400]], (90,80)),
    ('left', True, [[100,400],[100,200],[200,200],[200,400]], (0,60)),
    ('right', False, [[500,200],[500,400],[400,400],[400,200]], (120,20)),
])
def test_all_boundaries_and_reversed_direction(boundary, reversed_x, corners, expected):
    floor = GroundCalibration([[200,200],[400,200],[400,400],[200,400]], 120, 80, 600, 600)
    length = 120 if boundary in ('top', 'bottom') else 80
    wall = WallCalibration(corners, length, 60, 600, 600)
    cal = DualPlaneCalibration(floor, wall, boundary, reversed_x)
    for h in (0, 30, 60):
        assert cal.wall_to_floor_base(length / 4, h) == pytest.approx(expected)
    for x in (0, length/2, length):
        assert cal.floor_to_pixel(*cal.wall_to_floor_base(x, 0)) == pytest.approx(cal.wall_to_pixel(x, 0))


def test_surface_dispatch_and_robot_geometry_inputs(dual):
    floor = dual.localize(SurfaceType.FLOOR, *project(40, 30, 0))
    assert floor.surface == SurfaceType.FLOOR
    assert floor.floor_xy_cm == pytest.approx((40, 30), abs=1e-4)
    assert floor.wall_xh_cm is None
    assert floor.height_above_floor_cm == 0
    wall = dual.localize('WALL', *project(40, 0, 30))
    assert wall.wall_xh_cm == pytest.approx((40, 30), abs=1e-4)
    assert wall.floor_xy_cm == pytest.approx((40, 0), abs=1e-4)
    assert wall.height_above_floor_cm == pytest.approx(30, abs=1e-4)
    limits = RobotReachGeometry(max_reachable_height_cm=40, required_wall_distance_cm=12)
    assert limits.required_wall_distance_cm == 12  # Data only, no reachability decision.
    with pytest.raises(CalibrationError, match='FLOOR or WALL'):
        dual.localize('UNKNOWN', 1, 2)


@pytest.mark.parametrize('height,distance', [(-1,10),(10,-1),(float('nan'),10),(10,float('inf'))])
def test_invalid_robot_geometry(height, distance):
    with pytest.raises(CalibrationError):
        RobotReachGeometry(height, distance)


@pytest.mark.parametrize('x,h', [(-1,0),(121,0),(30,-1),(30,61),(float('nan'),0),(0,float('inf'))])
def test_invalid_wall_projection(dual, x, h):
    with pytest.raises(CalibrationError):
        dual.wall_to_floor_base(x, h)


def test_version_two_persistence(dual, tmp_path):
    path = tmp_path / 'camera.json'
    dual.save(path)
    result = DualPlaneCalibration.load(path)
    assert result.to_dict() == dual.to_dict()
    assert isinstance(load_calibration(path), DualPlaneCalibration)
    assert result.pixel_to_wall(*project(47,0,33)) == pytest.approx((47,33), abs=1e-4)
    assert result.to_dict()['floor']['depth_cm'] == 80


@pytest.mark.parametrize('damage', ['missing_wall','invalid_wall','wall_matrix','boundary',
                                  'width','direction','moved_base','missing_geometry','version'])
def test_invalid_dual_configuration(dual, tmp_path, damage):
    data = copy.deepcopy(dual.to_dict())
    if damage == 'missing_wall':
        del data['wall']
    elif damage == 'invalid_wall':
        data['wall']['corners_pixels'] = [[1,1]] * 4
    elif damage == 'wall_matrix':
        data['wall']['homography'] = [[0,0,0]] * 3
    elif damage == 'boundary':
        data['shared_geometry']['floor_boundary'] = 'ceiling'
    elif damage == 'width':
        data['wall']['width_cm'] = 999
    elif damage == 'direction':
        data['shared_geometry']['wall_x_reversed'] = 'false'
    elif damage == 'moved_base':
        data['wall']['corners_pixels'][2][0] += 20
    elif damage == 'missing_geometry':
        del data['shared_geometry']
    else:
        data['calibration_version'] = 5
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps(data))
    with pytest.raises(CalibrationError):
        load_calibration(path)


def test_legacy_floor_is_not_a_dual_calibration(dual, tmp_path):
    path = tmp_path / 'old.json'
    data = dual.floor.to_dict()
    del data['surface']  # Exactly the pre-extension v1 format.
    path.write_text(json.dumps(data))
    with pytest.warns(UserWarning, match='floor-only'):
        loaded = load_calibration(path)
    assert isinstance(loaded, GroundCalibration)
    assert not isinstance(loaded, DualPlaneCalibration)
    with pytest.raises(CalibrationError, match='floor-only'):
        DualPlaneCalibration.load(path)


def test_both_planes_require_matching_source_and_resolution(dual):
    with pytest.raises(CalibrationError, match='WALL'):
        DualPlaneCalibration(dual.floor, None, 'top')
    for changed in ('source', 'frame_width'):
        wall = copy.deepcopy(dual.wall)
        setattr(wall, changed, 2 if changed == 'source' else 1000)
        with pytest.raises(CalibrationError, match='same camera source and resolution'):
            DualPlaneCalibration(dual.floor, wall, 'top')
    with pytest.warns(UserWarning, match='recalibration'), pytest.raises(CalibrationError):
        dual.validate_resolution(640, 480)


def test_incorrect_shared_base_and_projective_parameterization():
    floor = GroundCalibration([[200,200],[400,200],[400,400],[200,400]], 120, 80, 600, 600)
    wall = WallCalibration([[180,80],[400,130],[400,200],[200,200]], 120, 60, 600, 600)
    # Matching endpoints alone cannot establish consistent physical X along the edge.
    approximate = DualPlaneCalibration(floor, wall, 'top')
    assert approximate.quality_report['shared_boundary']['max_error_px'] > 3
    assert any('Shared-boundary inconsistency' in value for value in approximate.quality_report['warnings'])
    wall = WallCalibration([[200,100],[400,100],[400,200],[200,200]], 120, 60, 600, 600)
    with pytest.raises(CalibrationError, match='base corners'):
        DualPlaneCalibration(floor, wall, 'bottom')


def test_wall_rectification_upright_and_bounded_with_grid():
    wall = WallCalibration([[100,50],[700,50],[700,350],[100,350]], 120, 60, 800, 600)
    image = np.zeros((600,800,3), np.uint8)
    image[120:130,395:406] = (255,0,0)
    image[270:280,395:406] = (0,0,255)
    rectified = wall.birdseye(image, 5)
    assert rectified.shape == (301,601,3)
    assert rectified[75,300,0] == 255  # High physical point stays near image top.
    assert rectified[225,300,2] == 255
    grid = wall.birdseye(image, 1e10, grid=True)
    assert max(grid.shape[:2]) <= 2048
    assert wall.pixel_to_wall(400,350) == pytest.approx((60,0), abs=1e-6)
