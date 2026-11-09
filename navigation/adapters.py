"""Adapters reuse Module04 ground-center and Module05 measured tool geometry."""
from copy import deepcopy
import json
import math
import time
import threading
import logging
import cv2
import numpy as np
from vision.cleaning_geometry import footprint,rotation
from vision.cleaning_config import CleaningHardware
from vision.robot_cli import build_robot_tracker
from .controller import Pose,NavigationFailure
from .geometry import spill_bounds,front_bumper_metrics,front_parking_candidates


class PoseUnavailable(ValueError):
    def __init__(self,code,reason,*,retry_startup=False):
        super().__init__(f'{code}: Module04 ground-center unavailable: {reason}')
        self.retry_startup=retry_startup;self.recoverable=retry_startup


class CameraPoses:
    def __init__(self,camera,calibration_path,settings_path,mapping_path,*,verification_points_path=None,fixed_camera=False,ground_reference_confirmed=False,log=None):
        self.camera=camera
        self.log=log or logging.getLogger(__name__).warning
        try:
            self.tracker=build_robot_tracker(settings_path,calibration_path,marker_plane_path=mapping_path,
                verification_points_path=verification_points_path,fixed_camera=fixed_camera,ground_reference_confirmed=ground_reference_confirmed)
        except Exception as exc:
            self.log(f'MAPPING_CONTEXT_ERROR: {exc}')
            raise
        if self.tracker.error:
            self.log('MAPPING_CONTEXT_ERROR: '+self.tracker.error)
            raise ValueError('MAPPING_CONTEXT_ERROR: '+self.tracker.error)
        if self.tracker.marker_plane is None:
            self.log('MAPPING_ERROR: Validated fixed-height Module04 mapping required')
            raise ValueError('MAPPING_ERROR: Validated fixed-height Module04 mapping required')
        if self.tracker.config.smoothing_alpha!=1:
            raise ValueError('Navigation needs unfiltered fresh Module04 poses; supply separate settings rather than overwrite geometry')
        self.frame=0;self.worker=None;self.condition=threading.Condition();self.closed=threading.Event()
        self.latest=None;self.capture_error=None;self.consumed=-1;self.last_unavailable=None
        self.frame_read_status='NOT_STARTED';self.read_started=None;self.read_seconds=None;self.frame_received_at=None
        self.last_verified_at=None;self.diagnostic_at=0.;self.diagnostic_status=None

    def start(self,maximum_wait_seconds,*,startup_wait_seconds=None):
        """One capture worker; a blocked camera read cannot block STOP monitoring."""
        if startup_wait_seconds is None:startup_wait_seconds=maximum_wait_seconds
        if not math.isfinite(startup_wait_seconds) or not 0<startup_wait_seconds<=5:
            raise ValueError('Startup wait must be finite and within (0, 5] seconds')
        self.maximum_wait_seconds=maximum_wait_seconds
        self.startup_deadline=time.monotonic()+startup_wait_seconds
        self.worker=threading.Thread(target=self._capture,daemon=True);self.worker.start()
        return self

    def _capture(self):
        acquired=False;last_reason=None
        while not self.closed.is_set():
            try:pose=self._read_pose()
            except Exception as exc:
                reason=str(exc) if isinstance(exc,PoseUnavailable) else f'CAMERA_LOCALIZATION_ERROR: {type(exc).__name__}: {exc}'
                self.last_unavailable=reason
                if reason!=last_reason:self.log(reason);last_reason=reason
                with self.condition:
                    self.latest=None;self.latest_sample=None;self.capture_error=exc;self.condition.notify_all()
                if (isinstance(exc,PoseUnavailable) and exc.recoverable and
                        (acquired or time.monotonic()<self.startup_deadline)):
                    self.closed.wait(.01)
                    continue
                return
            if not acquired:self.log('POSE_VERIFIED: current empirical Module04 ground-center acquired')
            acquired=True
            last_reason=None;self.last_unavailable=None
            with self.condition:
                self.capture_error=None;self.last_verified_at=pose.observed_at
                self.latest=pose;self.latest_sample=getattr(self,'latest_observation',None);self.condition.notify_all()

    def close(self):
        self.closed.set()
        with self.condition:self.condition.notify_all()
        if self.worker:self.worker.join(.2)

    def observe(self,*,after=None,wait_seconds=None):
        if self.worker is None:return self._read_pose()
        startup=self.consumed<0
        deadline=(self.startup_deadline if startup and wait_seconds is None else time.monotonic()+
                  (self.maximum_wait_seconds if wait_seconds is None else wait_seconds))
        with self.condition:
            while not self.closed.is_set():
                if self.capture_error and not ((startup or wait_seconds is not None) and getattr(self.capture_error,'recoverable',False)):
                    raise self.capture_error
                if (self.latest is not None and self.latest.frame_id>self.consumed and
                    (after is None or self.latest.observed_at>after)):
                    self.consumed=self.latest.frame_id;self.consumed_observation=self.latest_sample;return self.latest
                left=deadline-time.monotonic()
                if left<=0:
                    reason='STARTUP_TIMEOUT' if self.consumed<0 else 'STALE_POSE'
                    message='fresh verified camera pose timed out; frame_read='+self.frame_read_status+'; '+(self.last_unavailable or 'No completed camera observation')
                    self._diagnostic(force=True)
                    self.log(reason+': '+message)
                    raise PoseUnavailable(reason,message,retry_startup=not startup)
                self.condition.wait(left)
        raise ValueError('Camera pose source closed')

    def reacquire(self,*,after=None,timeout=1.):
        """Caller must first establish DONE,MOVE or acknowledged STOP; no motion here."""
        if self.worker is None:raise ValueError('Stationary reacquisition requires background camera capture')
        if not math.isfinite(timeout) or not 0<timeout<=2:raise ValueError('Reacquisition window must be within (0, 2] seconds')
        after=max(time.monotonic(),after or 0.)
        self.log(f'REACQUIRE_BEGIN: stationary; timeout={timeout:.2f}s; require capture after {after:.6f}')
        try:pose=self.observe(after=after,wait_seconds=timeout)
        except Exception as exc:
            self.log('REACQUIRE_FAILED: '+str(exc));raise
        self.log(f'REACQUIRE_VERIFIED: frame={pose.frame_id}; heading={pose.heading_deg:.2f}; pose_age={time.monotonic()-pose.observed_at:.3f}s')
        return pose

    def observe_after(self,after):
        """Wait for capture after settling, skipping any queued older frames."""
        return self.observe(after=after)

    def _diagnostic(self,pose=None,*,force=False):
        now=time.monotonic()
        if pose is None:
            sample=getattr(self,'latest_observation',None);pose=sample[1] if sample else None
        status=(self.frame_read_status,getattr(pose,'tracking_status',None),getattr(pose,'localization_status',None))
        if not force and status==self.diagnostic_status and now-self.diagnostic_at<.2:return
        self.diagnostic_at=now;self.diagnostic_status=status
        quality=getattr(pose,'pose_quality',{})
        visible=(getattr(pose,'tracking_status',None)=='VISIBLE') if self.frame_read_status=='OK' else None
        detector=getattr(getattr(self.tracker,'detector',None),'last_quality',{})
        if not isinstance(detector,dict):detector={}
        evidence={k:quality.get(k,detector.get(k)) for k in
                  ('marker_area_pixels','minimum_edge_pixels','decoded_marker_count','configured_marker_count','rejected_candidates')}
        if visible is None:evidence={k:None for k in evidence}
        self.log('POSE_DIAGNOSTIC '+json.dumps(dict(frame_id=self.frame,frame_read=self.frame_read_status,
            marker_id=getattr(pose,'marker_id',None),position_cm=getattr(pose,'position_cm',None),
            heading_deg=getattr(pose,'heading_deg',None),marker_corners_pixels=getattr(pose,'marker_corners_pixels',None),
            age_basis='host frame receipt; source capture latency unknown',
            read_seconds=self.read_seconds,read_in_progress_seconds=now-self.read_started if self.frame_read_status=='READING' else None,
            marker_visible=visible,tracking_status=status[1],localization_status=status[2],
            observation_current=bool(visible and quality.get('observation_current')),
            pose_age_seconds=(max(0,now-self.frame_received_at) if visible and self.frame_received_at is not None else
                              None if self.last_verified_at is None else max(0,now-self.last_verified_at)),
            detection_confidence='UNKNOWN' if visible is None else 'DECODED' if visible else 'NOT_DECODED',confidence_probability=None,
            reason=quality.get('reason',self.last_unavailable),**evidence),allow_nan=False))

    def _read_pose(self):
        self.read_started=time.monotonic();self.frame_read_status='READING'
        try:frame=self.camera.read()
        except Exception:
            self.read_seconds=time.monotonic()-self.read_started
            self.frame_read_status='FAILED';self._diagnostic(force=True);raise
        received=time.monotonic();self.frame_received_at=received;self.read_seconds=received-self.read_started
        self.frame_read_status='OK' if frame is not None else 'EMPTY'
        if frame is None:
            self._diagnostic(force=True);raise ValueError('CAMERA_ERROR: Camera feed ended')
        self.frame+=1
        pose=self.tracker.update(frame)
        self.latest_observation=(frame,pose)
        self._diagnostic(pose)
        quality=pose.pose_quality;reason=quality.get('reason','No tracker detail')
        if pose.tracking_status!='VISIBLE':
            code='STALE_POSE' if pose.tracking_status in ('LOST','EXPIRED') else 'MARKER_NOT_VISIBLE'
            raise PoseUnavailable(code,reason,retry_startup=True)
        if not quality.get('observation_current'):
            raise PoseUnavailable('STALE_POSE',reason,retry_startup=True)
        if quality.get('mapping_method')!='EMPIRICAL_FIXED_HEIGHT_PLANE':
            raise PoseUnavailable('MAPPING_ERROR',reason)
        if pose.localization_status!='VERIFIED' or not pose.suitable_for_navigation:
            raise PoseUnavailable('LOCALIZATION_'+pose.localization_status,reason)
        if not pose.position_cm or pose.heading_deg is None:
            raise PoseUnavailable('INVALID_POSE',reason)
        # This is a host read timestamp, not a claim about sensor exposure timing.
        return Pose(pose.position_cm['x'],pose.position_cm['y'],pose.heading_deg,received,self.frame,verified=True)


class ToolApproach:
    def __init__(self,hardware,calibration,arena,timing):
        self.hardware=hardware if isinstance(hardware,CleaningHardware) else CleaningHardware(hardware)
        self.h=self.hardware.data;self.calibration=calibration;self.arena=arena;self.timing=timing

    def floor_tools(self):
        sponge=self.h['floor_sponge'];nozzle=self.h['floor_nozzle']
        length,width=nozzle['coverage_length_cm'],nozzle['coverage_width_cm']
        if abs(nozzle['fixed_orientation_deg']/90-round(nozzle['fixed_orientation_deg']/90))>1e-8:
            raise NavigationFailure('Unsupported non-cardinal effective nozzle rectangle')
        if round(nozzle['fixed_orientation_deg']/90)%2:length,width=width,length
        return [(sponge['contact_length_cm'],sponge['contact_width_cm'],sponge['offset_forward_cm'],sponge['offset_lateral_cm']),
                (length,width,nozzle['offset_forward_cm'],nozzle['offset_lateral_cm'])]

    def prepare(self,plan,*,front_clearance_cm=None,heading_deg=None):
        plan=deepcopy(plan.to_dict() if hasattr(plan,'to_dict') else plan)
        surface=plan['surface']
        missing=self.hardware.missing(surface)
        if missing:raise NavigationFailure('Unconfigured cleaning geometry: '+', '.join(missing))
        if plan.get('reachability')!='FULLY_REACHABLE' or not plan.get('approach_pose'):
            raise NavigationFailure('Module05 cannot provide a fully reachable approach')
        points=np.array([p for ring in plan['target_geometry'] if ring['kind']=='outer' for p in ring['points']],float)
        pose={k:plan['approach_pose'][k] for k in ('x_cm','y_cm','heading_deg')}
        if surface=='FLOOR':
            if heading_deg is not None:pose['heading_deg']=heading_deg
            r=rotation(pose['heading_deg']);local=points @ r
            lower=np.full(2,-np.inf);upper=np.full(2,np.inf)
            for length,width,forward,lateral in self.floor_tools():
                offset=np.array([forward,lateral]);half=np.array([length,width])/2
                lower=np.maximum(lower,local.max(axis=0)-offset-half)
                upper=np.minimum(upper,local.min(axis=0)-offset+half)
            if np.any(lower>upper):
                raise NavigationFailure('SIMPLE fixed cleaning cannot execute this multi-pass Module05 target; no invented coverage')
            center=(lower+upper)/2 @ r.T
            if front_clearance_cm is not None:
                goals=front_parking_candidates(plan,self.arena,front_clearance_cm,include_unsafe=True,from_pose=pose)
                matching=[g for g in goals if abs((g['heading_deg']-pose['heading_deg']+180)%360-180)<1e-6]
                if not matching:raise NavigationFailure('No actual-contour front-clearance parking position')
                center=np.array([matching[0]['x_cm'],matching[0]['y_cm']])
                plan['front_clearance_cm']=front_clearance_cm
            pose.update(x_cm=float(center[0]),y_cm=float(center[1]))
        elif surface=='WALL':
            x=np.array([self.calibration.wall_to_floor_base(float(v),0)[0] for v in points[:,0]])
            sign=1 if abs((pose['heading_deg']+180)%360-180)<1 else -1
            s,n=self.h['wall_sponge'],self.h['wall_nozzle']
            lower=max(x.max()-sign*s['offset_forward_cm']-s['contact_width_cm']/2,
                      x.max()-sign*(n['mount_offset_forward_cm']+n['impact_offset_forward_cm'])-n['coverage_width_cm']/2)
            upper=min(x.min()-sign*s['offset_forward_cm']+s['contact_width_cm']/2,
                      x.min()-sign*(n['mount_offset_forward_cm']+n['impact_offset_forward_cm'])+n['coverage_width_cm']/2)
            if lower>upper:raise NavigationFailure('Wall target wider than stationary SIMPLE cleaning contact/spray')
            pose['x_cm']=float((lower+upper)/2)
        else:raise NavigationFailure('Unknown surface')
        plan['module05_approach_pose']=deepcopy(plan['approach_pose'])
        plan['approach_pose'].update(pose)
        plan['execution_scope']='single fixed SIMPLE cleaning preset; not arbitrary Module05 coverage strokes'
        self.check(plan,Pose(**pose,observed_at=0,frame_id=0))
        return plan

    def check(self,plan,observed,*,clearance_tolerance_cm=2.5,heading_tolerance_deg=7.,verify_tool_coverage=True):
        pose=observed.geometry();surface=plan['surface']
        if not self.arena.safe(pose):raise NavigationFailure('Actual tool/chassis approach is unsafe')
        points=[p for ring in plan['target_geometry'] if ring['kind']=='outer' for p in ring['points']]
        if surface=='FLOOR':
            if 'front_clearance_cm' in plan:
                metrics=front_bumper_metrics(plan,pose,self.arena.length,self.arena.body_width)
                if (abs(metrics['clearance_cm']-plan['front_clearance_cm'])>clearance_tolerance_cm or
                    metrics['facing_error_deg']>heading_tolerance_deg):
                    raise NavigationFailure('Front bumper clearance/facing verification failed: '+str(metrics))
            for length,width,forward,lateral in (self.floor_tools() if verify_tool_coverage else []):
                polygon=np.float32(footprint(pose,length,width,(forward,lateral)))
                if any(cv2.pointPolygonTest(polygon,tuple(map(float,p)),False)<0 for p in points):
                    raise NavigationFailure('Actual pose does not place target inside both tool contact and spray')
            current=pose.copy()
            for direction in ('F','B')*3:
                rate,sign=self.timing.rate(direction)
                # A failed cleaner may continue a segment to its independent 600ms bound.
                limit=current.copy();a=math.radians(current['heading_deg'])
                for key,projection in (('x_cm',math.cos(a)),('y_cm',math.sin(a))):limit[key]+=sign*600/rate*projection
                self.arena.check_segment(current,limit)
                current={**current,'x_cm':current['x_cm']+sign*350/rate*math.cos(a),
                         'y_cm':current['y_cm']+sign*350/rate*math.sin(a)}
        else:
            s,n=self.h['wall_sponge'],self.h['wall_nozzle']
            for wallx,height in points:
                x=self.calibration.wall_to_floor_base(wallx,0)[0]
                sign=1 if math.cos(math.radians(pose['heading_deg']))>0 else -1
                contact=pose['x_cm']+sign*s['offset_forward_cm']
                spray=pose['x_cm']+sign*(n['mount_offset_forward_cm']+n['impact_offset_forward_cm'])
                if (abs(x-contact)>s['contact_width_cm']/2 or abs(x-spray)>n['coverage_width_cm']/2 or
                    not s['min_contact_height_cm']<=height<=s['max_contact_height_cm'] or
                    not n['impact_min_height_cm']<=height<=n['impact_max_height_cm']):
                    raise NavigationFailure('Actual stationary wall tool cannot cover target')
            expected=self.h['body']['chassis_width_cm']/2+s['lateral_protrusion_cm']-s['contact_compression_cm']
            # Wall contact has a tighter geometric requirement than a loose navigation tolerance.
            if abs(pose['y_cm']-expected)>.5:raise NavigationFailure('Wall contact gap differs from measured tool geometry')
            expected_heading=0 if s['side']=='RIGHT' else 180
            if abs((pose['heading_deg']-expected_heading+180)%360-180)>2:
                raise NavigationFailure('Wall contact requires tighter measured parallel alignment')
