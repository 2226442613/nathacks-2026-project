"""Operator-attested independent physical point report, never a calibration rewrite."""
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .calibration import DualPlaneCalibration, PhysicalValidationPoint, load_calibration, load_validation_points
from .localization import utc_timestamp
from .localization_runtime import image_identity
from .serialization import json_line


def evaluate_measurements(calibration, points, *, calibration_id, reference_id, tolerance_cm=2.,attested_measured=False):
    if isinstance(tolerance_cm,bool) or not isinstance(tolerance_cm,(float,int)) or not math.isfinite(tolerance_cm) or not 0<tolerance_cm<=10:
        raise ValueError('Physical tolerance must be finite, >0 and <=10 cm')
    planes = {'FLOOR':calibration.floor,'WALL':calibration.wall} if isinstance(calibration,DualPlaneCalibration) else {calibration.surface.value:calibration}
    samples = []
    expected_by_surface = {key:[] for key in planes}
    for point in points:
        if not isinstance(point,PhysicalValidationPoint) or point.surface.value not in planes:
            raise ValueError('Measured point surface is not calibrated')
        plane = planes[point.surface.value]
        if not (0<=point.expected_cm[0]<=plane.width_cm and 0<=point.expected_cm[1]<=plane.height_cm):
            raise ValueError('Measured point lies outside physical surface dimensions')
        if cv2.pointPolygonTest(plane.corners_pixels.astype(np.float32),point.pixel,True)<-1:
            raise ValueError('Measured image point outside calibrated polygon')
        predicted = (calibration.pixel_to_floor(*point.pixel) if point.surface.value=='FLOOR' else calibration.pixel_to_wall(*point.pixel)) if isinstance(calibration,DualPlaneCalibration) else plane.pixel_to_world(*point.pixel)
        errors = [abs(predicted[i]-point.expected_cm[i]) for i in range(2)]
        distance = float(math.hypot(*errors))
        if not math.isfinite(distance):
            raise ValueError('Non-finite measured physical residual')
        expected_by_surface[point.surface.value].append(point.expected_cm)
        samples.append({'surface':point.surface.value,'pixel':list(point.pixel),'expected_cm':list(point.expected_cm),
                        'predicted_cm':list(predicted),'absolute_axis_error_cm':errors,'euclidean_error_cm':distance})
    coverage = {}
    for name,plane in planes.items():
        xy = np.float32(expected_by_surface[name]).reshape(-1,2)
        area = float(cv2.contourArea(cv2.convexHull(xy))) if len(xy)>=3 else 0.
        coverage[name] = {'count':len(xy),'physical_hull_area_fraction':area/(plane.width_cm*plane.height_cm),
                          'physical_hull_cm':cv2.convexHull(xy).reshape(-1,2).tolist() if len(xy)>=3 else []}
    passed = (attested_measured is True and bool(samples) and
              all(item['count']>=3 and item['physical_hull_area_fraction']>=.05 for item in coverage.values()) and
              all(item['euclidean_error_cm']<=tolerance_cm for item in samples))
    return {'report_version':1,'calibration_id':calibration_id,'reference_id':reference_id,
            'timestamp':utc_timestamp(),'attested_measured':attested_measured is True,'tolerance_cm':tolerance_cm,
            'points':samples,'coverage':coverage,'physical_validation_passed':bool(passed),
            'max_error_cm':max((item['euclidean_error_cm'] for item in samples),default=None),
            'rms_error_cm':float(np.sqrt(np.mean([item['euclidean_error_cm']**2 for item in samples]))) if samples else None,
            'scope':'Only measured reference-view evidence at the stated tolerance; not automatic robot motion authorization'}


def revalidate_report(report, calibration, calibration_id):
    if not isinstance(report,dict) or report.get('report_version')!=1 or report.get('calibration_id')!=calibration_id:
        raise ValueError('Measured report calibration/version mismatch')
    if not isinstance(report.get('reference_id'),str) or not report['reference_id'].startswith('sha256:'):
        raise ValueError('Measured report lacks reference image identity')
    points = [PhysicalValidationPoint(item['surface'],item['pixel'],item['expected_cm']) for item in report['points']]
    computed = evaluate_measurements(calibration,points,calibration_id=calibration_id,
        reference_id=report['reference_id'],tolerance_cm=report['tolerance_cm'],attested_measured=report.get('attested_measured') is True)
    computed['report_timestamp'] = report['timestamp']
    computed['report_id'] = 'sha256:'+hashlib.sha256(json_line(report).encode()).hexdigest()
    return computed


def run_validation(args):
    calibration = load_calibration(args.calibration)
    image = cv2.imread(args.reference,cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Cannot load independently verified calibration-view image')
    calibration.validate_resolution(image.shape[1],image.shape[0])
    report = evaluate_measurements(calibration,load_validation_points(args.points),
        calibration_id='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest(),
        reference_id=image_identity(image),tolerance_cm=args.tolerance_cm,attested_measured=args.confirm_measured)
    output = Path(args.output)
    inputs = [args.calibration,args.reference,args.reference+'.json',args.points]
    if any(output.resolve()==Path(item).resolve() for item in inputs):
        raise ValueError('Validation output must not overwrite calibration/reference/measurements')
    if output.exists():
        raise ValueError('Validation report already exists; choose a new report path')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json_line(report)+'\n',encoding='utf-8')
    print(json_line(report))
