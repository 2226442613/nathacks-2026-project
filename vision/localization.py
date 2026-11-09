"""Module 03: mask ownership and representative points, never robot approach poses."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone, timedelta
import hashlib
import math

import cv2
import numpy as np

from .calibration import DualPlaneCalibration, GroundCalibration, WallCalibration
from .serialization import json_native, json_line


def utc_timestamp(value: float | None = None) -> str:
    return datetime.fromtimestamp(value,timezone.utc).isoformat().replace('+00:00','Z') if value is not None else datetime.now(timezone.utc).isoformat().replace('+00:00','Z')


@dataclass(frozen=True)
class LocalizationConfig:
    ownership_fraction: float = .85
    significant_other_fraction: float = .10
    max_outside_fraction: float = .15
    boundary_margin_pixels: int = 2
    boundary_fraction: float = .10
    world_boundary_tolerance_cm: float = .5
    contour_tolerance_cm: float = .2
    contour_max_points: int = 512
    contour_max_bytes: int = 32768

    def __post_init__(self):
        for key,value in asdict(self).items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
        for key in ('ownership_fraction','significant_other_fraction','max_outside_fraction','boundary_fraction'):
            if not 0 < getattr(self,key) <= 1:
                raise ValueError(f'{key} must be in (0,1]')
        if not isinstance(self.boundary_margin_pixels,int) or not 0 <= self.boundary_margin_pixels <= 20:
            raise ValueError('boundary_margin_pixels must be 0..20')
        if not 0 <= self.contour_tolerance_cm <= 5:
            raise ValueError('contour_tolerance_cm must be 0..5')
        for key,low,high in (('contour_max_points',4,10000),('contour_max_bytes',1024,1048576)):
            if not isinstance(getattr(self,key),int) or not low <= getattr(self,key) <= high:
                raise ValueError(f'{key} outside supported limits')
        if not 0 <= self.world_boundary_tolerance_cm <= 5:
            raise ValueError('world_boundary_tolerance_cm must be 0..5')


@dataclass
class CameraRegistration:
    status: str = 'UNVERIFIED'
    current_to_calibration: np.ndarray | None = None
    compensated: bool = False
    reason: str = 'No current-to-calibration registration available'
    max_displacement_pixels: float | None = None
    reference_association: str = 'UNKNOWN'
    override_active: bool = False
    reference_id: str | None = None

    @classmethod
    def from_reference_to_current(cls, matrix, frame_size, *, safe_limit_pixels=2.,
                                  reference_association='OPERATOR_DECLARED_CALIBRATION_VIEW', override_active=False,reference_id=None):
        """Invert ONCE: detector warps baseline forward; anchors must map backward."""
        try:
            if isinstance(safe_limit_pixels,bool) or not math.isfinite(safe_limit_pixels) or safe_limit_pixels<=0:
                raise ValueError('Safe motion limit must be positive and finite')
            if len(frame_size)!=2 or any(isinstance(v,bool) or not isinstance(v,(int,np.integer)) or v<1 for v in frame_size):
                raise ValueError('Registration frame dimensions must be positive integers')
            forward = np.asarray(matrix,dtype=float)
            if forward.shape == (2,3):
                forward = np.vstack((forward,[0.,0.,1.]))
            if forward.shape != (3,3) or not np.isfinite(forward).all() or np.linalg.matrix_rank(forward) != 3:
                raise ValueError('Registration matrix must be finite and invertible 3x3')
            width,height = frame_size
            corners = np.float64([[0,0],[width-1,0],[width-1,height-1],[0,height-1]])
            homogeneous = np.column_stack((corners,np.ones(4))) @ forward.T
            if not ((homogeneous[:,2] > 1e-9).all() or (homogeneous[:,2] < -1e-9).all()):
                raise ValueError('Registration crosses a projective horizon')
            moved = cv2.perspectiveTransform(corners.reshape(1,-1,2),forward)[0]
            if cv2.contourArea(moved.astype(np.float32),oriented=True)<=0:
                raise ValueError('Registration must preserve image orientation')
            displacement = float(np.linalg.norm(moved-corners,axis=1).max())
            if not math.isfinite(displacement) or displacement > safe_limit_pixels:
                return cls('MOVED',None,False,'Registration exceeds safe small-motion limit',displacement,reference_association,override_active,reference_id)
            return cls('MATCHED',np.linalg.inv(forward),displacement>.1,'Reliable bounded image registration',
                       displacement,reference_association,override_active,reference_id)
        except (ValueError,TypeError,np.linalg.LinAlgError,cv2.error) as exc:
            return cls(reason=str(exc),reference_association=reference_association,override_active=override_active,reference_id=reference_id)

    def to_dict(self):
        try:
            return json_native(asdict(self))
        except (ValueError,TypeError):
            return {'status':'INVALID','current_to_calibration':None,'compensated':False,
                    'reason':'Non-finite or unsupported camera registration data','override_active':bool(self.override_active)}


@dataclass(frozen=True)
class LocalizationResult:
    target_id: int | None
    category: str
    surface: str
    pixel_anchor: tuple[float,float] | None
    calibration_pixel_anchor: tuple[float,float] | None
    world_coordinates_cm: dict | None
    localization_status: str
    reason: str
    timestamp: str
    calibration_id: str | None
    calibration_version: int | None
    coordinate_frame: dict
    camera_registration: dict
    surface_confidence: float = 0.
    floor_pixel_count: int = 0
    wall_pixel_count: int = 0
    outside_pixel_count: int = 0
    component_pixel_count: int = 0
    floor_overlap_ratio: float = 0.
    wall_overlap_ratio: float = 0.
    outside_overlap_ratio: float = 0.
    warnings: tuple[str,...] = ()
    calibration_quality: dict = field(default_factory=dict)
    first_observed_timestamp: str | None = None
    visual_detection_usable: bool = True
    overlap_coordinate_frame: str = 'current_registered_calibration_rois'
    validation_evidence: dict = field(default_factory=dict)
    image_geometry: dict | None = None
    geometry_cm: dict | None = None
    full_physical_contours_cm: list | None = None
    geometry_status: str = 'UNAVAILABLE'
    geometry_reason: str = 'Physical geometry unavailable'
    actionable: bool = False  # Navigation authorization is outside Module 03.

    def to_dict(self):
        return json_native(asdict(self))

    def target_message(self, *, active=True, timestamp=None, record_key=None):
        position = self.world_coordinates_cm if self.surface in ('FLOOR','WALL') and self.localization_status!='INVALID' else None
        if position is not None and set(position)!=({'x','y'} if self.surface=='FLOOR' else {'x','h'}):
            raise ValueError('Target message position keys disagree with surface coordinate convention')
        localized = position is not None
        geometry = self.geometry_cm if localized else None
        state = ('INACTIVE' if not active else 'DETECTED' if not localized else
                 'LOCALIZED_VERIFIED' if self.localization_status=='VERIFIED' else 'LOCALIZED_ESTIMATE')
        valid_until = (datetime.fromisoformat(self.timestamp.replace('Z','+00:00'))+timedelta(seconds=2)).isoformat().replace('+00:00','Z')
        return json_native({'schema_version':1,'message_type':'TARGET_DETECTION','record_key':record_key,
            'target_id':self.target_id,'category':self.category,'surface':self.surface,
            'position_cm':position if active else None,
            'geometry_cm':geometry if active else None,
            'last_known_geometry_cm':geometry,
            'geometry_status':self.geometry_status,'geometry_reason':self.geometry_reason,
            'last_known_position_cm':position,'units':'cm','coordinate_frame':self.coordinate_frame,
            'localization_status':self.localization_status,'detection_state':state,'active':bool(active),
            'actionable':False,'motion_authorized':False,
            'physically_verified':bool(active and localized and self.localization_status=='VERIFIED'),
            'timestamp':timestamp or self.timestamp,'last_observed_timestamp':self.timestamp,
            'valid_until':valid_until,'valid_for_seconds':2.,
            'first_observed_timestamp':self.first_observed_timestamp,
            'calibration_id':self.calibration_id,'calibration_version':self.calibration_version,
            'camera_registration':self.camera_registration,'visual_detection_usable':self.visual_detection_usable,
            'quality_warnings':self.warnings,'localization_reason':self.reason,
            'surface_confidence_heuristic':self.surface_confidence})


class SurfaceLocalizer:
    def __init__(self, calibration, config=None, *, calibration_id=None, physical_evidence=None, calibration_error=None):
        self.calibration = calibration
        self.config = config or LocalizationConfig()
        self.error = calibration_error
        self.evidence = None
        self.masks = {}
        self.quality_warnings = []
        self.quality = {}
        self.calibration_id = calibration_id
        self.version = None
        self.coordinate_frame = {'frame_id':calibration_id,'units':'cm',
            'image_coordinates':'current original pixels; calibration anchor mapped once',
            'floor':{'origin':'Module 01 floor (0,0)','axes':{'x':'floor +X','y':'floor +Y'}},
            'wall':{'origin':'Module 01 wall X=0 at shared base','axes':{'x':'wall +X','h':'up from shared base'}}}
        if calibration is None:
            self.error = self.error or 'Calibration missing'
            return
        try:
            data = calibration.to_dict()
            self.version = data['calibration_version']
            # Reuse Module 01 validation (including stored matrix/corner agreement).
            type(calibration).from_dict(data)
            self.calibration_id = calibration_id or 'sha256:'+hashlib.sha256(json_line(data).encode()).hexdigest()
            self.coordinate_frame['frame_id'] = self.calibration_id
            planes = {'FLOOR':calibration.floor,'WALL':calibration.wall} if isinstance(calibration,DualPlaneCalibration) else {calibration.surface.value:calibration}
            self.coordinate_frame['available_surfaces'] = list(planes)
            plane = next(iter(planes.values()))
            self.frame_size = (plane.frame_width,plane.frame_height)
            for name,item in planes.items():
                mask = np.zeros((plane.frame_height,plane.frame_width),np.uint8)
                cv2.fillPoly(mask,[np.rint(item.corners_pixels).astype(np.int32)],255)
                self.masks[name] = mask
                self.coordinate_frame[name.lower()].update(width_cm=item.width_cm,
                    **{('depth_cm' if name=='FLOOR' else 'height_cm'):item.height_cm})
            self.shared_band = np.zeros_like(next(iter(self.masks.values())))
            if isinstance(calibration,DualPlaneCalibration):
                self.quality = calibration.quality_report
                self.quality_warnings = list(self.quality['warnings'])
                self.coordinate_frame.update(shared_floor_boundary=calibration.floor_boundary,wall_x_reversed=calibration.wall_x_reversed)
                self.coordinate_frame['wall_origin_in_floor_cm'] = dict(zip(('x','y'),calibration.wall_to_floor_base(0,0)))
                self.coordinate_frame['wall_x_endpoint_in_floor_cm'] = dict(zip(('x','y'),calibration.wall_to_floor_base(calibration.wall.width_cm,0)))
                cv2.line(self.shared_band,tuple(np.rint(calibration.wall.corners_pixels[3]).astype(int)),
                         tuple(np.rint(calibration.wall.corners_pixels[2]).astype(int)),255,
                         max(1,2*self.config.boundary_margin_pixels+1))
                if calibration.floor_boundary=='top':
                    self.coordinate_frame['floor'].update(origin='Module 01 floor top-left at shared edge',
                        axes={'x':'horizontal along shared edge in floor +X direction','y':'away from wall; shared edge Y=0'})
                else:
                    self.quality_warnings += ['Non-top shared boundary: floor uses native Module 01 X/Y; Y is not rebased to wall distance']
            self._cached_transform = None
            if physical_evidence is not None:
                from .localization_validation import revalidate_report
                try:
                    self.evidence = revalidate_report(physical_evidence,calibration,self.calibration_id)
                    if not self.evidence['physical_validation_passed']:
                        self.quality_warnings.append('Measured validation report did not pass attestation/coverage/residual requirements')
                except (ValueError,KeyError,TypeError) as exc:
                    self.quality_warnings.append(f'Measured validation evidence rejected: {exc}')
        except (ValueError,TypeError,KeyError,AttributeError,np.linalg.LinAlgError,cv2.error) as exc:
            self.error = f'Invalid calibration: {exc}'

    def _current_masks(self, registration):
        backward = registration.current_to_calibration
        if backward is None:
            return self.masks,self.shared_band
        backward = np.asarray(backward,float)
        if backward.shape!=(3,3) or not np.isfinite(backward).all() or np.linalg.matrix_rank(backward)!=3:
            raise ValueError('Invalid current-to-calibration transform')
        forward = np.linalg.inv(backward)
        if np.allclose(forward,np.eye(3),atol=1e-7):
            return self.masks,self.shared_band
        if self._cached_transform is None or not np.allclose(forward,self._cached_transform,atol=1e-7):
            self._cached_masks = {name:cv2.warpPerspective(mask,forward,self.frame_size,flags=cv2.INTER_NEAREST) for name,mask in self.masks.items()}
            self._cached_band = cv2.warpPerspective(self.shared_band,forward,self.frame_size,flags=cv2.INTER_NEAREST)
            self._cached_transform = forward.copy()
        return self._cached_masks,self._cached_band

    def localize(self, target, mask, *, frame_size, registration=None, timestamp=None, origin=(0,0)):
        registration = registration or CameraRegistration()
        time_error = None
        try:
            timestamp = timestamp or utc_timestamp(target.last_seen)
            if not isinstance(timestamp,str) or datetime.fromisoformat(timestamp.replace('Z','+00:00')).tzinfo is None:
                raise ValueError('Observation timestamp must be timezone-aware ISO format')
            first_timestamp = utc_timestamp(target.first_seen) if target.first_seen is not None else timestamp
        except (ValueError,TypeError,OverflowError,OSError):
            timestamp = first_timestamp = utc_timestamp()
            time_error = 'Invalid observation timestamp'
        fields = dict(target_id=target.target_id,category=target.category.value,surface='UNKNOWN',pixel_anchor=None,
            calibration_pixel_anchor=None,world_coordinates_cm=None,localization_status='UNVERIFIED',reason='',
            visual_detection_usable=False,
            timestamp=timestamp,calibration_id=self.calibration_id,calibration_version=self.version,calibration_quality=self.quality,
            first_observed_timestamp=first_timestamp,
            coordinate_frame=self.coordinate_frame,camera_registration=registration.to_dict(),warnings=tuple(self.quality_warnings))
        if self.evidence:
            fields['validation_evidence'] = {name:self.evidence[name] for name in
                ('report_id','report_timestamp','tolerance_cm','physical_validation_passed','reference_id')}
        try:
            if time_error:
                raise ValueError(time_error)
            if not target.confirmed:
                raise ValueError('Only confirmed targets are localized')
            mask = np.asarray(mask)
            if mask.ndim!=2 or mask.dtype not in (np.dtype('bool'),np.dtype('uint8')):
                raise ValueError('Exact component mask must be bool/uint8')
            left,top = origin
            if any(isinstance(v,bool) or not isinstance(v,(int,np.integer)) for v in (left,top)):
                raise ValueError('Component mask origin must contain finite integers')
            height,width = mask.shape
            if left<0 or top<0 or left+width>frame_size[0] or top+height>frame_size[1]:
                raise ValueError('Component mask coordinates outside frame')
            component = mask!=0
            count = int(component.sum())
            if count==0 or count!=target.area_pixels:
                raise ValueError('Missing, empty or inconsistent exact component mask')
            ys,xs = np.nonzero(component)
            # Nearest actual member to mask centroid: guaranteed inside even for holes/concavity.
            index = int(np.argmin((xs-xs.mean())**2+(ys-ys.mean())**2))
            fields['pixel_anchor'] = (float(xs[index]+left),float(ys[index]+top))
            fields.update(component_pixel_count=count,visual_detection_usable=True)
            from .target_geometry import image_geometry, physical_geometry
            fields['image_geometry'] = image_geometry(component,origin)
            if self.error:
                raise ValueError(self.error)
            if tuple(frame_size) != self.frame_size:
                raise ValueError('Frame dimensions differ from calibration')
            fallback = registration.current_to_calibration is None or registration.status!='MATCHED'
            effective = replace(registration,current_to_calibration=np.eye(3),compensated=False) if fallback else registration
            fields['camera_registration'].update(
                fallback_active=fallback,
                mapping_mode='CURRENT_FRAME_FALLBACK' if fallback else 'REGISTERED_CURRENT_TO_CALIBRATION',
                registration_warning=registration.reason if fallback else None)
            if fallback:
                fields['warnings'] += ('Registration warning: '+registration.reason+'; current-frame homography estimates are UNVERIFIED',)
                fields['overlap_coordinate_frame']='current_frame_loaded_calibration_polygons'
            masks,band = self._current_masks(effective)
            roi = np.s_[top:top+height,left:left+width]
            floor = masks['FLOOR'][roi]!=0 if 'FLOOR' in masks else np.zeros(component.shape,bool)
            wall = masks['WALL'][roi]!=0 if 'WALL' in masks else np.zeros(component.shape,bool)
            f,w = int(np.count_nonzero(component&floor)),int(np.count_nonzero(component&wall))
            outside = int(np.count_nonzero(component&~(floor|wall)))
            fields.update(floor_pixel_count=f,wall_pixel_count=w,outside_pixel_count=outside,component_pixel_count=count,
                          floor_overlap_ratio=f/count,wall_overlap_ratio=w/count,outside_overlap_ratio=outside/count,
                          surface_confidence=max(f,w)/count)
            cfg = self.config
            near = np.count_nonzero(component & (band[roi]!=0))/count if cfg.boundary_margin_pixels else 0
            if outside/count > cfg.max_outside_fraction:
                fields['reason'] = 'Too much component area outside calibrated polygons'
            elif ((np.count_nonzero(component&floor&~wall)/count >= cfg.significant_other_fraction and
                   np.count_nonzero(component&wall&~floor)/count >= cfg.significant_other_fraction) or near>=cfg.boundary_fraction):
                fields.update(surface='BOUNDARY',reason='Significant overlap with both planes or shared-edge uncertainty band')
            elif f/count >= cfg.ownership_fraction and np.count_nonzero(component&wall&~floor)/count < cfg.significant_other_fraction:
                fields['surface'] = 'FLOOR'
            elif w/count >= cfg.ownership_fraction and np.count_nonzero(component&floor&~wall)/count < cfg.significant_other_fraction:
                fields['surface'] = 'WALL'
            else:
                fields['reason'] = 'Insufficient clear surface ownership'
            if fields['surface'] not in ('FLOOR','WALL'):
                return LocalizationResult(**dict(fields,geometry_reason=fields['reason'] if fields.get('geometry_cm') is None else fields['geometry_reason']))
            chosen = component & (floor if fields['surface']=='FLOOR' else wall)
            ys,xs = np.nonzero(chosen)
            index = int(np.argmin((xs-xs.mean())**2+(ys-ys.mean())**2))
            anchor = (float(xs[index]+left),float(ys[index]+top))
            fields['pixel_anchor'] = anchor
            back = np.asarray(effective.current_to_calibration,float)
            if back.shape!=(3,3) or not np.isfinite(back).all() or np.linalg.matrix_rank(back)!=3:
                raise ValueError('Invalid current-to-calibration transform')
            raw = back @ np.array([*anchor,1.])
            if abs(raw[2])<1e-9:
                raise ValueError('Pixel anchor lies on registration horizon')
            reference_anchor = cv2.perspectiveTransform(np.float64([[anchor]]),back)[0,0]
            if not np.isfinite(reference_anchor).all():
                raise ValueError('Non-finite calibration pixel anchor')
            fields['calibration_pixel_anchor'] = tuple(float(v) for v in reference_anchor)
            surface = fields['surface']
            plane = (self.calibration.floor if surface=='FLOOR' else self.calibration.wall) if isinstance(self.calibration,DualPlaneCalibration) else self.calibration
            if isinstance(self.calibration,DualPlaneCalibration):
                mapped = self.calibration.pixel_to_floor(*reference_anchor) if surface=='FLOOR' else self.calibration.pixel_to_wall(*reference_anchor)
            else:
                mapped = plane.pixel_to_world(*reference_anchor)
            if not all(math.isfinite(v) for v in mapped):
                raise ValueError('Non-finite physical coordinates')
            if not (-cfg.world_boundary_tolerance_cm<=mapped[0]<=plane.width_cm+cfg.world_boundary_tolerance_cm and
                    -cfg.world_boundary_tolerance_cm<=mapped[1]<=plane.height_cm+cfg.world_boundary_tolerance_cm):
                raise ValueError('Physical anchor extrapolates outside calibrated surface')
            fields['world_coordinates_cm'] = {'x':mapped[0],('y' if surface=='FLOOR' else 'h'):mapped[1]}
            fields['reason'] = ('Current-frame fallback estimate: '+registration.reason+'; physical accuracy UNVERIFIED' if fallback else 'Mask representative mapped via existing Module 01 API; physical accuracy unverified')
            warnings = list(fields['warnings'])
            if registration.compensated:
                warnings.append('Small-motion compensation is image registration, not measured centimeter validation')
            if registration.override_active:
                warnings.append('Framing override active; physical coordinates not verified')
            if (not fallback and isinstance(self.evidence,dict) and self.evidence.get('physical_validation_passed') is True and
                    self.evidence.get('attested_measured') is True and self.evidence.get('calibration_id')==self.calibration_id and
                    self.evidence.get('reference_id')==registration.reference_id and
                    registration.status=='MATCHED' and not registration.compensated and not registration.override_active):
                hull = np.float32(self.evidence['coverage'][surface]['physical_hull_cm'])
                if cv2.pointPolygonTest(hull,tuple(mapped),False)>=0:
                    fields['localization_status'] = 'VERIFIED'
                    fields['reason'] = 'Within explicitly attested measured-point hull at reported tolerance; no motion authorization'
                else:
                    warnings.append('Outside measured validation hull; coordinates remain UNVERIFIED')
            fields['warnings'] = tuple(warnings)
            def contour_mapper(u,v):
                den = plane.homography @ np.array([u,v,1.])
                if abs(den[2])<1e-9:
                    raise ValueError('Contour lies on physical homography horizon')
                return (self.calibration.pixel_to_floor(u,v) if surface=='FLOOR' else self.calibration.pixel_to_wall(u,v)) if isinstance(self.calibration,DualPlaneCalibration) else plane.pixel_to_world(u,v)
            # Reject a horizon crossing, even if all vertices map to finite values.
            for ring in fields['image_geometry']['contours']:
                hp=np.column_stack((ring['points'],np.ones(len(ring['points'])))) @ back.T @ plane.homography.T
                if not ((hp[:,2]>1e-9).all() or (hp[:,2]<-1e-9).all()):
                    raise ValueError('Contour crosses physical homography horizon')
            geometry,full = physical_geometry(fields['image_geometry'],back,contour_mapper,
                surface=surface,width=plane.width_cm,height=plane.height_cm,
                boundary_tolerance=cfg.world_boundary_tolerance_cm,tolerance_cm=cfg.contour_tolerance_cm,
                max_points=cfg.contour_max_points,max_bytes=cfg.contour_max_bytes)
            status=fields['localization_status']
            if status=='VERIFIED':
                hull=np.float32(self.evidence['coverage'][surface]['physical_hull_cm'])
                if any(cv2.pointPolygonTest(hull,tuple(p),False)<0 for ring in full for p in ring['points']):
                    status='UNVERIFIED'
            fields.update(geometry_cm=geometry,full_physical_contours_cm=full,geometry_status=status,
                geometry_reason=('Current-frame fallback estimate: '+registration.reason+'; all physical geometry UNVERIFIED' if fallback else 'Raster footprint estimate; inherits calibration/alignment quality; full boundary must lie in measured hull to be VERIFIED'))
            geometry['trust_state']=status
            return LocalizationResult(**dict(fields,geometry_reason=fields['reason'] if fields.get('geometry_cm') is None else fields['geometry_reason']))
        except (ValueError,TypeError,KeyError,IndexError,AttributeError,np.linalg.LinAlgError,cv2.error) as exc:
            fields.update(localization_status='INVALID',world_coordinates_cm=None,geometry_cm=None,full_physical_contours_cm=None,geometry_status='INVALID',geometry_reason=str(exc),reason=str(exc))
            return LocalizationResult(**dict(fields,geometry_reason=fields['reason'] if fields.get('geometry_cm') is None else fields['geometry_reason']))

    def localize_detections(self, result, targets, *, frame_size, registration, timestamp=None):
        records = []
        for target in targets:
            if not target.confirmed:
                continue
            x,y,w,h = target.bounding_box
            if result.component_labels is None:
                mask = np.zeros((1,1),np.uint8)
                origin = (0,0)
            else:
                mask = result.component_labels[y:y+h,x:x+w] == target.component_id
                origin = (x,y)
            records.append(self.localize(target,mask,origin=origin,frame_size=frame_size,
                                         registration=registration,timestamp=timestamp))
        return records
