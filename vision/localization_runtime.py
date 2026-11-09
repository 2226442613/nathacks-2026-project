"""Registration bridge and persistent target events; no network or robot transport."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from uuid import uuid4
from datetime import datetime,timezone

import cv2
import numpy as np

from .frame_geometry import FrameGeometryGuard
from .localization import CameraRegistration, utc_timestamp
from .serialization import json_line


def image_identity(image):
    return 'sha256:'+hashlib.sha256(str(image.shape).encode()+image.tobytes()).hexdigest()


class LocalizationRegistration:
    """Freeze the calibration view independently of later Module 02 baseline resets."""
    def __init__(self, calibration_view, initial_baseline, *, declared=False, baseline_unverified=False, config):
        self.reference_id = image_identity(calibration_view)
        self.association = 'OPERATOR_DECLARED_CALIBRATION_VIEW' if declared else 'ASSUMED_INITIAL_BASELINE_VIEW'
        self.association_unknown = baseline_unverified and not declared
        self.reuse_detection = np.array_equal(calibration_view,initial_baseline)
        self.guard = FrameGeometryGuard(calibration_view,max_shift_pixels=config.geometry_max_shift_pixels,
            max_rotation_degrees=config.geometry_max_rotation_degrees,max_scale_change=config.geometry_max_scale_change,
            min_matches=config.geometry_min_matches,min_inlier_ratio=config.geometry_min_inlier_ratio)
        self.safe_limit = config.alignment_max_displacement_pixels
        self.shape = calibration_view.shape

    def for_frame(self, frame, result, *, baseline_changed=False, override_active=False):
        association = self.association
        if frame.shape != self.shape:
            return CameraRegistration(reason='Calibration view dimensions differ from current frame',reference_association=association,
                                      reference_id=self.reference_id,override_active=override_active)
        if self.association_unknown:
            return CameraRegistration(reason='Baseline/calibration association unknown after movement; using diagnostic current-frame estimates',
                                      reference_association=association,reference_id=self.reference_id,override_active=override_active)
        if self.reuse_detection and not baseline_changed and not override_active:
            geometry,matrix = result.geometry,result.reference_to_current
        else:
            geometry = self.guard.check(frame)
            matrix = self.guard.transform
        if geometry.status=='MATCHED' and matrix is not None:
            return CameraRegistration.from_reference_to_current(matrix,(frame.shape[1],frame.shape[0]),
                safe_limit_pixels=self.safe_limit,reference_association=association,
                reference_id=self.reference_id,override_active=override_active)
        if (geometry.status in ('UNVERIFIED','DISABLED') and not override_active and
                not result.detection_paused and not baseline_changed):
            return CameraRegistration('UNVERIFIED',np.eye(3),False,
                'Nominal identity assumed for sparse low-texture scene; framing is NOT verified',None,association,override_active,self.reference_id)
        return CameraRegistration(geometry.status,None,False,geometry.reason,None,association,override_active,self.reference_id)


class TargetEventStore:
    """Append-only event snapshots; key includes run/epoch so reset IDs cannot collide."""
    def __init__(self, path=None, *, session_id=None):
        self.path = Path(path) if path else None
        self.session_id = session_id or uuid4().hex
        self.states = self.read(self.path) if self.path and self.path.exists() else {}
        # A previous process's last observation is historical, never a current target.
        for key,value in list(self.states.items()):
            if value['active']:
                self._inactive(key,utc_timestamp(),'new_session_previous_observation_is_stale')

    @staticmethod
    def read(path):
        states = {}
        for index,line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(),1):
            try:
                value = json.loads(line)
                if value['event_schema_version']!=1 or value['event_type'] not in ('UPSERT','INACTIVE'):
                    raise ValueError('Unsupported target-event schema')
                json_line(value)  # Reject NaN/Infinity in externally read histories too.
                if not isinstance(value['active'],bool) or not isinstance(value['session_id'],str):
                    raise ValueError('Invalid active/session metadata')
                for required in ('target_message','localization','last_seen','first_seen'):
                    if required not in value:
                        raise ValueError(f'Missing event field {required}')
                states[value['record_key']] = value
            except (KeyError,ValueError,TypeError) as exc:
                raise ValueError(f'Invalid target-event line {index}: {exc}') from exc
        return states

    def _append(self,event):
        line = json_line(event)
        if self.path:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            with self.path.open('a',encoding='utf-8') as stream:
                stream.write(line+'\n')
        self.states[event['record_key']] = event

    def _inactive(self,key,timestamp,reason):
        previous = self.states[key]
        message = dict(previous['target_message'],active=False,position_cm=None,actionable=False,
                       motion_authorized=False,physically_verified=False,geometry_cm=None,detection_state='INACTIVE',timestamp=timestamp)
        event = dict(previous,event_type='INACTIVE',active=False,timestamp=timestamp,reason=reason,target_message=message)
        self._append(event)
        return event

    def update(self, records, *, epoch=0, timestamp=None):
        timestamp = timestamp or utc_timestamp()
        active_keys = set()
        events = []
        for record in records:
            if record.target_id is None:
                raise ValueError('Persistent localization requires a tracked target ID')
            key = f'{self.session_id}:{epoch}:{record.target_id}'
            active_keys.add(key)
            previous = self.states.get(key)
            event = {'event_schema_version':1,'event_type':'UPSERT','record_key':key,
                'session_id':self.session_id,'tracking_epoch':epoch,'target_id':record.target_id,'active':True,
                'timestamp':timestamp,'first_seen':previous['first_seen'] if previous else (record.first_observed_timestamp or record.timestamp),
                'last_seen':record.timestamp,'localization':record.to_dict(),
                'target_message':record.target_message(record_key=key)}
            self._append(event)
            events.append(event)
        for key,value in list(self.states.items()):
            if value['active'] and value['session_id']==self.session_id and key not in active_keys:
                events.append(self._inactive(key,timestamp,'not_observed_in_current_frame'))
        return events

    def deactivate_all(self, reason='tracking_reset'):
        for key,value in list(self.states.items()):
            if value['active'] and value['session_id']==self.session_id:
                self._inactive(key,utc_timestamp(),reason)

    def current_messages(self, *, now=None):
        """Fresh active observations only; raw read() snapshots remain historical."""
        return self._fresh_messages(self.states,now)

    @classmethod
    def read_current_messages(cls,path,*,now=None):
        """Read-only integration API; never construct a new producer to consume data."""
        return cls._fresh_messages(cls.read(path),now)

    @staticmethod
    def _fresh_messages(states,now):
        moment = datetime.fromisoformat(now.replace('Z','+00:00')) if now else datetime.now(timezone.utc)
        messages=[]
        for item in states.values():
            message=item['target_message']
            if not item['active'] or 'valid_until' not in message:
                continue
            seen=datetime.fromisoformat(message['last_observed_timestamp'].replace('Z','+00:00'))
            expiry=datetime.fromisoformat(message['valid_until'].replace('Z','+00:00'))
            if seen<=moment<=expiry:
                messages.append(message)
        return messages


def _geometry_overlay_lines(record):
    """Presentation only: trust and availability come from the existing record."""
    identity=f'#{record.target_id} {record.category} | {record.surface} | {record.localization_status}'
    valid=(record.surface in ('FLOOR','WALL') and record.localization_status!='INVALID')
    if valid and record.world_coordinates_cm is not None:
        axes=('x','y') if record.surface=='FLOOR' else ('x','h')
        identity+=' | '+' '.join(f'{axis.upper()}={record.world_coordinates_cm[axis]:.1f}' for axis in axes)+' cm'
    lines=[identity]
    if valid and record.geometry_cm is not None and record.geometry_status in ('UNVERIFIED','VERIFIED'):
        g=record.geometry_cm
        axis,key=('D','depth_cm') if record.surface=='FLOOR' else ('H','height_cm')
        lines.append(f"W={g['width_cm']:.1f} cm | {axis}={g[key]:.1f} cm | area ~{g['area_cm2']:.1f} cm^2 | geometry {record.geometry_status}")
    else:
        lines.append('Geometry unavailable: '+(record.geometry_reason or record.reason or 'No physical geometry available'))
    return lines


def _wrap_overlay_line(text, width, scale=.40):
    """Fit by rendered pixel width, including unusually long reason tokens."""
    lines=[]
    current=''
    for word in text.split():
        candidate=(current+' '+word).strip()
        if cv2.getTextSize(candidate,0,scale,1)[0][0]<=width:
            current=candidate
            continue
        if current:
            lines.append(current)
            current=''
        for char in word:
            if current and cv2.getTextSize(current+char,0,scale,1)[0][0]>width:
                lines.append(current)
                current=''
            current+=char
    if current: lines.append(current)
    return lines


def draw_localizations(image, records, *, top=160, boundaries=False):
    """Keep the default overlay; optional geometry uses stacked, width-fitted rows."""
    if boundaries:
        # Preserve half-pixel cell edges instead of rounding away thin outlines.
        for record in records:
            if record.image_geometry:
                for ring in record.image_geometry['contours']:
                    cv2.polylines(image,[np.rint(np.asarray(ring['points'])*2).astype(np.int32)],True,
                                  (255,0,255) if ring['kind']=='hole' else (0,255,255),1,shift=1)
            if record.pixel_anchor is not None:
                cv2.circle(image,tuple(round(v) for v in record.pixel_anchor),3,(255,255,0),-1)
        y=max(14,top)
        for index,record in enumerate(records):
            lines=[line for text in _geometry_overlay_lines(record)
                   for line in _wrap_overlay_line(text,max(1,image.shape[1]-16))]
            # Reserve room for an honest overflow notice rather than overlapping
            # or clipping a partially rendered target's geometry/reason.
            reserve=16 if index<len(records)-1 else 0
            if y+(len(lines)-1)*16+reserve>=image.shape[0]-3:
                notice=f'+{len(records)-index} targets: overlay space limited'
                if y<image.shape[0]-3:
                    for line in _wrap_overlay_line(notice,max(1,image.shape[1]-16)):
                        if y>=image.shape[0]-3: break
                        _draw_geometry_text(image,line,y)
                        y+=16
                break
            for line in lines:
                _draw_geometry_text(image,line,y)
                y+=16
            y+=5
        return
    for index,record in enumerate(records):
        if record.pixel_anchor is not None:
            cv2.circle(image,tuple(round(v) for v in record.pixel_anchor),3,(255,255,0),-1)
        if record.world_coordinates_cm is None:
            position = 'position ambiguous' if record.surface in ('BOUNDARY','UNKNOWN') else 'position unavailable'
        else:
            position = ' '.join(f'{key.upper()}={value:.1f}' for key,value in record.world_coordinates_cm.items())+' cm'
        line = f'#{record.target_id} {record.category} | {record.surface} | {position} | {record.localization_status}'
        y = top+index*18
        if y < image.shape[0]-3:
            cv2.putText(image,line,(8,y),0,.40,(0,0,0),3)
            cv2.putText(image,line,(8,y),0,.40,(255,255,0),1)


def _draw_geometry_text(image, text, y):
    width,height=cv2.getTextSize(text,0,.40,1)[0]
    # A backing prevents underlying detection-box labels from obscuring text.
    cv2.rectangle(image,(6,y-height-2),(min(image.shape[1]-1,width+10),y+3),(20,20,20),-1)
    cv2.putText(image,text,(8,y),0,.40,(255,255,0),1,cv2.LINE_AA)
