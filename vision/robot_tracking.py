"""Module 04: one ArUco robot, explicit elevated-plane estimates, no control."""
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime, timezone, timedelta
import hashlib
import math

import cv2
import numpy as np

from .calibration import DualPlaneCalibration, GroundCalibration
from .localization import CameraRegistration, utc_timestamp
from .serialization import json_native, json_line
from .spill_detection import validate_image


def dictionary(name):
    if not isinstance(name,str) or not name.startswith('DICT_') or not hasattr(cv2.aruco,name):
        raise ValueError('Unknown predefined ArUco dictionary')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,name))


@dataclass(frozen=True)
class RobotConfig:
    dictionary: str = 'DICT_4X4_50'
    marker_id: int = 0
    marker_size_cm: float = 6.
    mounting_height_cm: float | None = None
    marker_offset_forward_cm: float = 0.
    marker_offset_left_cm: float = 0.
    marker_to_robot_heading_deg: float = 0.
    robot_id: int = 0
    missed_frame_expiration: int = 5
    max_observation_age_seconds: float = .5
    smoothing_alpha: float = 1.
    exclusion_padding_pixels: int = 80
    body_length_cm: float | None = None
    body_width_cm: float | None = None

    def __post_init__(self):
        d=dictionary(self.dictionary)
        for name in ('marker_id','robot_id','missed_frame_expiration','exclusion_padding_pixels'):
            v=getattr(self,name)
            if isinstance(v,bool) or not isinstance(v,int) or v<0: raise ValueError(f'{name} must be a nonnegative integer')
        if self.marker_id>=len(d.bytesList): raise ValueError('Marker ID outside dictionary')
        for name,v in asdict(self).items():
            if name=='dictionary': continue
            if v is None:
                if name not in ('mounting_height_cm','body_length_cm','body_width_cm'):
                    raise ValueError(f'{name} cannot be null')
                continue
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v): raise ValueError(f'{name} must be finite')
        for name in ('marker_size_cm','max_observation_age_seconds','body_length_cm','body_width_cm'):
            v=getattr(self,name)
            if v is not None and v<=0: raise ValueError(f'{name} must be positive')
        if self.mounting_height_cm is not None and self.mounting_height_cm<0: raise ValueError('Mounting height cannot be negative')
        if not 0<self.smoothing_alpha<=1: raise ValueError('smoothing_alpha must be in (0,1]')
        if (self.body_length_cm is None)!=(self.body_width_cm is None): raise ValueError('Provide both body dimensions')
        if self.exclusion_padding_pixels<1: raise ValueError('A positive body exclusion margin is required')


def generate_marker(config, side_pixels=1200, quiet_zone_pixels=200):
    if any(isinstance(v,bool) or not isinstance(v,int) for v in (side_pixels,quiet_zone_pixels)):
        raise ValueError('Marker and quiet-zone pixel sizes must be integers')
    modules=dictionary(config.dictionary).markerSize+2
    if not modules*8<=side_pixels<=8000 or not math.ceil(side_pixels/modules)<=quiet_zone_pixels<=4000:
        raise ValueError('Use sufficient resolution and a white quiet zone of at least one module')
    marker=cv2.aruco.generateImageMarker(dictionary(config.dictionary),config.marker_id,side_pixels)
    return cv2.copyMakeBorder(marker,quiet_zone_pixels,quiet_zone_pixels,quiet_zone_pixels,quiet_zone_pixels,cv2.BORDER_CONSTANT,value=255)


def project_points(points, matrix):
    points=np.asarray(points,float)
    h=np.asarray(matrix,float)
    if points.ndim!=2 or points.shape[1]!=2 or h.shape!=(3,3) or not np.isfinite(h).all() or np.linalg.matrix_rank(h)!=3:
        raise ValueError('Invalid point transformation')
    homogeneous=np.column_stack((points,np.ones(len(points)))) @ h.T
    den=homogeneous[:,2]
    if not np.isfinite(homogeneous).all() or not ((den>1e-9).all() or (den < -1e-9).all()):
        raise ValueError('Non-finite coordinates or projective horizon')
    return homogeneous[:,:2]/den[:,None]


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    corners_pixels: tuple
    center_pixels: tuple
    axis_pixels: tuple
    quality: dict
    timestamp: str = field(default_factory=utc_timestamp)

    def to_dict(self):
        return json_native(asdict(self))


class RobotMarkerDetector:
    def __init__(self, config=None):
        self.config=config or RobotConfig()
        params=cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(dictionary(self.config.dictionary),params)
        self.last_reason='Marker not visible'
        self.last_quality={}

    def detect(self,frame):
        validate_image(frame)
        corners,ids,rejected=self.detector.detectMarkers(frame)
        selected=[] if ids is None else [c for c,i in zip(corners,ids.ravel()) if int(i)==self.config.marker_id]
        self.last_quality={'decoded_marker_count':0 if ids is None else len(ids),
                           'configured_marker_count':len(selected),'rejected_candidates':len(rejected)}
        self.last_reason='Configured marker not visible' if not selected else 'Duplicate configured marker IDs; robot identity ambiguous'
        if len(selected)!=1: return None
        p=np.asarray(selected[0],float).reshape(4,2)
        if not np.isfinite(p).all() or not cv2.isContourConvex(p.astype(np.float32)) or abs(cv2.contourArea(p.astype(np.float32)))<16:
            self.last_reason='Invalid or collapsed marker corners';return None
        canonical=np.float32([[0,0],[1,0],[1,1],[0,1]])
        h=cv2.getPerspectiveTransform(canonical,p.astype(np.float32))
        # Perspective center is the diagonal intersection, not mean image corners.
        points=project_points([[.5,.5],[.25,.5],[.75,.5]],h)
        lengths=np.linalg.norm(p-np.roll(p,1,axis=0),axis=1)
        self.last_reason='Configured marker decoded'
        self.last_quality.update(marker_area_pixels=abs(cv2.contourArea(p.astype(np.float32))),
             minimum_edge_pixels=float(lengths.min()),corner_order='canonical TL,TR,BR,BL; rotates with code',
             center_method='projective_diagonal_intersection')
        return MarkerObservation(self.config.marker_id,tuple(map(tuple,p)),tuple(points[0]),tuple(map(tuple,points[1:])),self.last_quality.copy())


@dataclass(frozen=True)
class RobotPose:
    robot_id: int
    marker_id: int
    timestamp: str
    tracking_status: str
    localization_status: str
    position_cm: dict | None
    heading_deg: float | None
    marker_corners_pixels: tuple | None
    marker_center_pixels: tuple | None
    coordinate_frame: dict
    calibration_id: str | None
    marker_configuration: dict
    camera_registration: dict
    pose_quality: dict
    last_known_pose: dict | None = None
    last_observed_timestamp: str | None = None
    observation_age_seconds: float | None = None
    missed_frames: int = 0
    tracking_event: str | None = None
    schema_version: int = 1
    message_type: str = 'ROBOT_POSE'
    surface: str = 'FLOOR'
    actionable: bool = False
    motion_authorized: bool = False
    suitable_for_navigation: bool = False

    def to_dict(self):
        return json_native(asdict(self))


def robot_pose_is_current(message, *, timestamp=None):
    """Freshness predicate for serialized JSONL too; historical VISIBLE is not current."""
    now=datetime.now(timezone.utc).timestamp() if timestamp is None else float(timestamp)
    if message.get('tracking_status')!='VISIBLE' or not math.isfinite(now): return False
    try:
        seen=datetime.fromisoformat(message['timestamp'].replace('Z','+00:00')).timestamp()
        expiry=datetime.fromisoformat(message['pose_quality']['valid_until'].replace('Z','+00:00')).timestamp()
        return seen<=now<=expiry
    except (KeyError,TypeError,ValueError): return False


class RobotTracker:
    def __init__(self, calibration=None, config=None, *, calibration_id=None, marker_plane=None):
        self.config=config or RobotConfig()
        self.detector=RobotMarkerDetector(self.config)
        self.calibration=calibration
        self.floor=calibration.floor if isinstance(calibration,DualPlaneCalibration) else calibration
        self.calibration_id=calibration_id
        self.error=None
        self.marker_plane=marker_plane
        try:
            if not isinstance(self.floor,GroundCalibration): raise ValueError('Floor calibration unavailable')
            data=calibration.to_dict();type(calibration).from_dict(data)
            self.calibration_id=calibration_id or 'sha256:'+hashlib.sha256(json_line(data).encode()).hexdigest()
            if marker_plane is not None: marker_plane.validate_context(self.calibration_id,self.config,(self.floor.frame_width,self.floor.frame_height))
        except (ValueError,TypeError,AttributeError) as exc: self.error=str(exc)
        self.last=None;self.missed=0;self.last_time=None;self.visible=False;self.smooth=None;self.smooth_mode=None
        self.verified_context=None

    def verify_context(self,evidence,*,fixed_camera=False,ground_reference_confirmed=False):
        """Read-only evidence check plus explicit CURRENT operator attestations.

        Plane validation establishes marker projection, not robot offset/heading
        alignment or whether the camera is still in its calibration pose.
        """
        self.verified_context=None
        if self.error or self.marker_plane is None:raise ValueError(self.error or 'Measured marker-plane mapping required')
        m=evidence['metadata'];plane=self.marker_plane.data
        if m.get('attested_measured') is not True or m.get('measurement_reference')!='MARKER_GROUND_PROJECTION':raise ValueError('Measured projection evidence required')
        if m.get('calibration_id')!=self.calibration_id or tuple(m.get('frame_size',()))!=tuple(plane['frame_size']):raise ValueError('Verification evidence camera context mismatch')
        if m.get('mounting_height_cm')!=plane['height_cm']:raise ValueError('Verification evidence mounting height mismatch')
        for kind in ('training','validation'):
            rows=[{k:p[k] for k in ('pixel','ground_cm')} for p in evidence[kind]]
            if rows!=plane[kind]:raise ValueError('Verification points do not match the saved validated mapping')
        recorded=m['robot_configuration'];current=asdict(self.config)
        keys=('dictionary','marker_id','marker_size_cm','mounting_height_cm','marker_offset_forward_cm','marker_offset_left_cm','marker_to_robot_heading_deg','body_length_cm','body_width_cm')
        if any(recorded.get(k)!=current[k] for k in keys):raise ValueError('Verification evidence marker/robot settings mismatch')
        if not fixed_camera:raise ValueError('Confirm camera remains in its measured calibration pose')
        if not ground_reference_confirmed and not (m.get('heading_alignment_verified') is True and m.get('chassis_reference_conversion_supported') is True):
            raise ValueError('Confirm actual chassis ground-center offsets and marker-to-forward heading alignment')
        self.verified_context={'scope':'fixed-height mapping, matching robot settings, operator-confirmed fixed camera and chassis reference',
            'fixed_camera_confirmed':True,'ground_reference_confirmed':True}

    def update(self,frame,*,timestamp=None,registration=None):
        if isinstance(timestamp,bool): raise ValueError('Robot timestamps cannot be boolean')
        moment=datetime.now(timezone.utc).timestamp() if timestamp is None else float(timestamp)
        if not math.isfinite(moment) or moment<0 or (self.last_time is not None and moment<self.last_time): raise ValueError('Robot timestamps must be finite and monotonic')
        self.last_time=moment
        stamp=utc_timestamp(moment)
        observation=self.detector.detect(frame)
        visible=observation is not None
        event='TRACKING_GAINED' if visible and not self.visible else 'TRACKING_LOST' if not visible and self.visible else None
        self.visible=visible
        reg=registration or CameraRegistration(reason='No camera alignment supplied; current-frame estimate')
        registration_metadata=reg.to_dict()
        frame_metadata={'frame_id':self.calibration_id,'units':'cm','axes':{'x':'Module 01 floor +X','y':'Module 01 floor +Y'},
            'heading':'degrees [0,360); 0=+X; positive toward +Y',
            'reference_point':'robot ground-center; approximate ray intersection in fallback mode',
            'shared_floor_boundary':getattr(self.calibration,'floor_boundary',None)}
        position=heading=None
        quality={'reason':self.detector.last_reason,'mapping_method':'UNAVAILABLE','warnings':[]}
        status='UNAVAILABLE'
        if visible:
            self.missed=0
            quality.update(observation.quality)
            try:
                if self.error: raise ValueError(self.error)
                if frame.shape[1::-1]!=(self.floor.frame_width,self.floor.frame_height): raise ValueError('Frame dimensions differ from calibration')
                fallback=reg.current_to_calibration is None or reg.status!='MATCHED'
                back=np.eye(3) if fallback else reg.current_to_calibration
                registration_metadata.update(fallback_active=fallback,registration_warning=reg.reason if fallback else None)
                # A planar image fit at floor height is not a guaranteed correction
                # at marker height. Compensation remains explicitly UNVERIFIED.
                reference=project_points([observation.center_pixels,*observation.axis_pixels,*observation.corners_pixels],back)
                if self.marker_plane is not None:
                    mapped=self.marker_plane.map(reference)
                    quality['mapping_method']='EMPIRICAL_FIXED_HEIGHT_PLANE'
                    quality['independent_validation']=self.marker_plane.report
                    quality['mapping_validation_status']='PASS'
                    quality['warnings'].append('Empirical mapping only valid at measured mounting height and reference camera pose')
                else:
                    mapped=np.asarray([self.floor.pixel_to_world(*p) for p in reference])
                    quality['mapping_method']='APPROXIMATE_FLOOR_RAY_INTERSECTION'
                    quality['warnings'].append('Elevated marker height parallax is NOT compensated; this is not the true robot ground position')
                if not np.isfinite(mapped).all(): raise ValueError('Non-finite robot mapping')
                vector=mapped[2]-mapped[1]
                if np.linalg.norm(vector)<1e-6: raise ValueError('Collapsed marker heading vector')
                angle=(math.degrees(math.atan2(vector[1],vector[0]))+self.config.marker_to_robot_heading_deg)%360
                rad=math.radians(angle);f,l=self.config.marker_offset_forward_cm,self.config.marker_offset_left_cm
                center=mapped[0]-[f*math.cos(rad)-l*math.sin(rad),f*math.sin(rad)+l*math.cos(rad)]
                if not (0<=center[0]<=self.floor.width_cm and 0<=center[1]<=self.floor.height_cm): raise ValueError('Estimated robot center outside calibrated floor')
                mode=(quality['mapping_method'],fallback,reg.reference_id)
                if self.smooth is not None and self.smooth_mode==mode:
                    alpha=self.config.smoothing_alpha
                    center=alpha*center+(1-alpha)*self.smooth[0]
                    delta=(angle-self.smooth[1]+180)%360-180
                    angle=(self.smooth[1]+alpha*delta)%360
                self.smooth=(center.copy(),angle);self.smooth_mode=mode
                position={'x':float(center[0]),'y':float(center[1])};heading=float(angle)
                quality['reason']='Diagnostic robot estimate; physical robot validation pending'
                quality['mapped_marker_mean_edge_cm']=float(np.linalg.norm(mapped[3:]-np.roll(mapped[3:],1,axis=0),axis=1).mean())
                quality['configured_marker_size_cm']=self.config.marker_size_cm
                quality['marker_scale_discrepancy_fraction']=abs(quality['mapped_marker_mean_edge_cm']/self.config.marker_size_cm-1)
                if quality['marker_scale_discrepancy_fraction']>.3:
                    quality['warnings'].append('Mapped marker size differs from configured black-square size; inspect print scale, height/parallax and calibration')
                if fallback: quality['warnings'].append('Registration fallback: '+reg.reason)
                if reg.compensated: quality['warnings'].append('Image compensation across height planes can retain parallax')
                if self.config.mounting_height_cm is None: quality['warnings'].append('Marker mounting height not measured/configured')
                if getattr(self.calibration,'floor_boundary','top')!='top': quality['warnings'].append('Native Module 01 axes retained; Y is not necessarily away from wall')
                status='UNVERIFIED'
                registration_valid=(registration is None or (reg.status=='MATCHED' and not reg.compensated and not reg.override_active and
                    reg.current_to_calibration is not None and np.allclose(reg.current_to_calibration,np.eye(3),atol=1e-6)))
                if (self.marker_plane is not None and self.verified_context is not None and registration_valid and
                    quality['marker_scale_discrepancy_fraction']<=.3):
                    status='VERIFIED';quality['verification_context']=self.verified_context
                    quality['reason']='Measured fixed-height mapping and current camera/robot context verified'
                elif self.marker_plane is not None:
                    quality['reason']='Mapping PASS; current camera/chassis reference or observed marker scale not verified'
            except (ValueError,TypeError,np.linalg.LinAlgError,cv2.error) as exc:
                status='INVALID';quality['reason']=str(exc);self.smooth=None;self.smooth_mode=None
        else:
            self.missed+=1;self.smooth=None;self.smooth_mode=None
        age=None if self.last is None else max(0,moment-self.last['observed_at_seconds'])
        expired=self.missed>self.config.missed_frame_expiration or (age is not None and age>self.config.max_observation_age_seconds)
        tracking='VISIBLE' if visible else 'NOT_SEEN' if self.last is None else 'EXPIRED' if expired else 'LOST'
        if visible:
            self.last={'position_cm':position,'heading_deg':heading,'marker_center_pixels':observation.center_pixels,
                       'marker_corners_pixels':observation.corners_pixels,'timestamp':stamp,'observed_at_seconds':moment,
                       'localization_status':status,'pose_quality':quality}
            age=0.
        fresh_until=(datetime.fromtimestamp(moment,timezone.utc)+timedelta(seconds=self.config.max_observation_age_seconds)).isoformat().replace('+00:00','Z') if visible else None
        quality.update(observation_current=visible,last_known_expired=bool(not visible and expired),
            valid_until=fresh_until,quality_is_not_navigation_authorization=True)
        last=None if self.last is None else {k:v for k,v in self.last.items() if k!='observed_at_seconds'}
        return RobotPose(self.config.robot_id,self.config.marker_id,stamp,tracking,status,position,heading,
            observation.corners_pixels if visible else None,observation.center_pixels if visible else None,
            frame_metadata,self.calibration_id,asdict(self.config),registration_metadata,quality,last,
            None if self.last is None else self.last['timestamp'],age,self.missed,event,suitable_for_navigation=status=='VERIFIED')

    def exclusion_mask(self,pose,frame_size,*,timestamp=None):
        """Approximate BODY exclusion; visible/fresh only, never persist a stale mask."""
        mask=np.zeros(frame_size[::-1],np.uint8)
        now=datetime.now(timezone.utc).timestamp() if timestamp is None else float(timestamp)
        seen=datetime.fromisoformat(pose.timestamp.replace('Z','+00:00')).timestamp()
        if pose.tracking_status!='VISIBLE' or now is None or not 0<=now-seen<=self.config.max_observation_age_seconds: return mask
        if pose.marker_corners_pixels is None: return mask
        corners=np.asarray(pose.marker_corners_pixels,float)
        cv2.fillConvexPoly(mask,np.rint(corners).astype(np.int32),255)
        radius=self.config.exclusion_padding_pixels
        # Conservative marker bbox expansion covers body above the floor too;
        # the operator must size this margin for the actual robot in the image.
        lo=np.floor(corners.min(axis=0)-radius).astype(int);hi=np.ceil(corners.max(axis=0)+radius).astype(int)
        cv2.rectangle(mask,tuple(lo),tuple(hi),255,-1)
        if self.config.body_length_cm is not None and pose.position_cm is not None:
            theta=math.radians(pose.heading_deg);forward=np.array([math.cos(theta),math.sin(theta)]);left=np.array([-forward[1],forward[0]])
            center=np.array([pose.position_cm['x'],pose.position_cm['y']])
            world=np.array([center+a*self.config.body_length_cm/2*forward+b*self.config.body_width_cm/2*left for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
            pixels=np.array([self.floor.world_to_pixel(*p) for p in world])
            reg=pose.camera_registration
            if not reg.get('fallback_active',True) and reg.get('current_to_calibration') is not None:
                pixels=project_points(pixels,np.linalg.inv(reg['current_to_calibration']))
            cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),255)
        return mask


def draw_robot_pose(image,pose,*,fps=0.,top=18):
    if pose.marker_corners_pixels is not None:
        p=np.rint(pose.marker_corners_pixels).astype(np.int32)
        cv2.polylines(image,[p],True,(0,255,0),2)
        center=tuple(round(v) for v in pose.marker_center_pixels)
        cv2.circle(image,center,4,(0,255,255),-1)
        # Deliberately show decoded marker +u; numeric theta is plane-aware robot heading.
        transform=cv2.getPerspectiveTransform(np.float32([[0,0],[1,0],[1,1],[0,1]]),np.asarray(p,np.float32))
        axis=project_points([[.25,.5],[.75,.5]],transform)
        v=axis[1]-axis[0];v=v/max(1,np.linalg.norm(v))*45
        cv2.arrowedLine(image,center,tuple(np.rint(np.array(center)+v).astype(int)),(255,180,0),2)
    position='position unavailable' if pose.position_cm is None else f"X={pose.position_cm['x']:.1f} Y={pose.position_cm['y']:.1f} cm | theta={pose.heading_deg:.1f} deg"
    mapping=('APPROX: elevated marker height/parallax NOT compensated' if pose.pose_quality['mapping_method']=='APPROXIMATE_FLOOR_RAY_INTERSECTION' else 'Fixed-height empirical plane estimate' if pose.pose_quality['mapping_method']=='EMPIRICAL_FIXED_HEIGHT_PLANE' else 'Physical mapping unavailable')
    if pose.pose_quality['mapping_method']=='EMPIRICAL_FIXED_HEIGHT_PLANE':mapping+=' | mapping PASS; '+('current context VERIFIED' if pose.localization_status=='VERIFIED' else 'current context UNVERIFIED')
    lines=[f'Robot {pose.robot_id} / marker {pose.marker_id} | {pose.tracking_status} | FPS {fps:.1f}',
           position+' | '+pose.localization_status,mapping,'Arrow: decoded marker +u in image (not a navigation arrow)']
    if pose.position_cm is None or pose.localization_status=='UNVERIFIED': lines.append(pose.pose_quality['reason'])
    for i,line in enumerate(lines):
        y=top+18*i
        cv2.putText(image,line,(8,y),0,.43,(0,0,0),3)
        cv2.putText(image,line,(8,y),0,.43,(0,255,255),1)
