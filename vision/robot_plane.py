"""Independent empirical map for a fixed elevated marker plane; never edits Module 01."""
import math
import cv2
import numpy as np

from .robot_tracking import project_points
from .serialization import json_native


class MarkerPlaneValidationError(ValueError):
    """A failed hold-out check, with diagnostics from the existing fitted map."""

    def __init__(self, report, errors_cm):
        super().__init__('Independent marker-plane validation exceeds tolerance')
        self.report = report
        self.errors_cm = errors_cm


class MarkerPlaneMapping:
    def __init__(self,data):
        if data.get('schema_version')!=1 or data.get('mapping_type')!='FIXED_HEIGHT_MARKER_PLANE':
            raise ValueError('Unsupported marker-plane mapping')
        # Re-fit from raw measured points; serialized pass flags/matrices alone
        # are not independent evidence or a usable mapping.
        rebuilt=self.fit(data['training'],data['validation'],calibration_id=data['calibration_id'],
            frame_size=data['frame_size'],floor_size_cm=data['floor_size_cm'],height_cm=data['height_cm'],
            tolerance_cm=data['tolerance_cm'],attested=data['attested_measured'])
        self.__dict__.update(rebuilt.__dict__)

    @classmethod
    def fit(cls,training,validation,*,calibration_id,frame_size,floor_size_cm,height_cm,tolerance_cm=2.,attested=False):
        if not isinstance(calibration_id,str) or not calibration_id: raise ValueError('Calibration identity required')
        if len(frame_size)!=2 or any(isinstance(v,bool) or not isinstance(v,int) or v<=0 for v in frame_size): raise ValueError('Invalid marker-plane frame size')
        if len(floor_size_cm)!=2 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in floor_size_cm): raise ValueError('Invalid physical floor size')
        if isinstance(height_cm,bool) or not isinstance(height_cm,(float,int)) or not math.isfinite(height_cm) or height_cm<0: raise ValueError('Measured mounting height required')
        if isinstance(tolerance_cm,bool) or not isinstance(tolerance_cm,(int,float)) or not math.isfinite(tolerance_cm) or tolerance_cm<=0: raise ValueError('Positive validation tolerance required')
        if attested is not True: raise ValueError('Explicit independent-measurement attestation required')
        arrays=[]
        for points,minimum in ((training,4),(validation,3)):
            if not isinstance(points,list) or len(points)<minimum: raise ValueError('Need >=4 training and >=3 independent validation points')
            pixel=np.asarray([p['pixel'] for p in points],float);world=np.asarray([p['ground_cm'] for p in points],float)
            if pixel.shape!=(len(points),2) or world.shape!=pixel.shape or not np.isfinite(pixel).all() or not np.isfinite(world).all(): raise ValueError('Invalid measured points')
            if (pixel<0).any() or (pixel>=np.array(frame_size)).any() or (world<0).any() or (world>np.array(floor_size_cm)).any(): raise ValueError('Measured points outside frame/floor')
            if cv2.contourArea(cv2.convexHull(world.astype(np.float32)))<.01*np.prod(floor_size_cm): raise ValueError('Insufficient non-collinear measured coverage')
            arrays.append((pixel,world))
        if any(np.min(np.linalg.norm(arrays[0][0]-p,axis=1))<1e-3 for p in arrays[1][0]): raise ValueError('Validation pixels must be independent of training points')
        if any(np.min(np.linalg.norm(arrays[0][1]-p,axis=1))<1e-3 for p in arrays[1][1]): raise ValueError('Validation ground points must be independent')
        matrix,_=cv2.findHomography(arrays[0][0],arrays[0][1],method=0)
        if matrix is None: raise ValueError('Singular empirical marker-plane mapping')
        project_points([[0,0],[frame_size[0]-1,0],[frame_size[0]-1,frame_size[1]-1],[0,frame_size[1]-1]],matrix)
        errors=project_points(arrays[1][0],matrix)-arrays[1][1]
        residual=np.linalg.norm(errors,axis=1)
        obj=object.__new__(cls);obj.matrix=matrix
        obj.report={'residuals_cm':residual.tolist(),'maximum_error_cm':float(residual.max()),
            'rms_error_cm':float(np.sqrt(np.mean(residual**2))),'tolerance_cm':tolerance_cm,
            'attested_measured':True,'physical_validation_scope':'fixed height, measured points, frozen camera pose only'}
        if float(residual.max())>tolerance_cm:
            raise MarkerPlaneValidationError(obj.report, errors.tolist())
        obj.data={'schema_version':1,'mapping_type':'FIXED_HEIGHT_MARKER_PLANE','calibration_id':calibration_id,
            'frame_size':list(frame_size),'floor_size_cm':list(floor_size_cm),'height_cm':height_cm,
            'tolerance_cm':tolerance_cm,'attested_measured':True,'training':training,'validation':validation,
            'homography':matrix.tolist(),'validation_report':obj.report}
        return obj

    def validate_context(self,calibration_id,config,frame_size):
        if self.data['calibration_id']!=calibration_id or tuple(self.data['frame_size'])!=tuple(frame_size): raise ValueError('Marker-plane calibration/frame identity mismatch')
        if config.mounting_height_cm is None or not math.isclose(config.mounting_height_cm,self.data['height_cm'],abs_tol=1e-6): raise ValueError('Marker-plane mapping mounting height mismatch')

    def map(self,points):
        return project_points(points,self.matrix)

    def to_dict(self):
        return json_native(self.data)
