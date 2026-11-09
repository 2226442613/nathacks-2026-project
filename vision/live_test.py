"""Shared-frame live vision diagnostics only; no navigation, transport or writes."""
import json
from pathlib import Path
import textwrap
import time

import cv2
import numpy as np

from .camera import CameraManager
from .calibration import load_calibration
from .localization import CameraRegistration,LocalizationConfig,SurfaceLocalizer
from .robot_cli import build_robot_tracker
from .robot_tracking import draw_robot_pose
from .spill_detection import BackgroundReference,SpillDetector,render_detections
from .spill_tracking import SpillTracker,load_spill_settings


def add_test_vision_command(commands):
    p=commands.add_parser('test-vision',help='Live shared-camera robot/spill coordinates; display only, Q exits')
    p.add_argument('--camera',type=int,default=0)
    p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720)
    for name,path in [('calibration','config/camera_config.json'),('background','data/iphone_reference.png'),
        ('robot-settings','config/robot_config.json'),('marker-plane','config/robot_marker_plane.json'),
        ('settings','config/spill_config.json'),('localization-settings','config/localization_config.json')]:
        p.add_argument('--'+name,default=path)
    p.set_defaults(headless=False)


def ground_coordinates(record,calibration):
    position=record.world_coordinates_cm
    if position is None:return None
    if record.surface=='FLOOR':return dict(position)
    if record.surface=='WALL':
        x,y=calibration.wall_to_floor_base(position['x'],0)
        return {'x':float(x),'y':float(y),'height_cm':position['h'],'reference':'wall floor-base projection'}
    return None


def message_overlay(image,messages):
    lines=[line for message in messages for line in textwrap.wrap(message,width=max(20,image.shape[1]//7))]
    top=max(115,image.shape[0]-18*len(lines)-6)
    for i,line in enumerate(lines):
        point=(8,top+18*i)
        cv2.putText(image,line,point,0,.43,(0,0,0),3)
        cv2.putText(image,line,point,0,.43,(0,190,255),1)


def current_frame_registration(identity):
    # The registered legal background is not necessarily the calibration image.
    # Robot and spill coordinates must both use current pixels through Module01.
    return CameraRegistration('UNVERIFIED',np.eye(3),False,
        'Current-frame Module01 estimate; background alignment is image-only',
        reference_association='OPERATOR_CONFIRMED_CURRENT_CALIBRATION_FRAME',reference_id=identity)


class LiveVision:
    """The test-vision processor/renderer, shared with the display-only Dashboard."""
    def __init__(self,args,*,calibration_id=None):
        self.startup=[];self.calibration=self.robot_tracker=self.detector=self.tracker=self.localizer=self.registration=None
        try:self.calibration=load_calibration(args.calibration)
        except Exception as exc:self.startup.append('Calibration: '+str(exc))
        try:self.robot_tracker=build_robot_tracker(args.robot_settings,args.calibration,marker_plane_path=args.marker_plane)
        except Exception as exc:self.startup.append('Robot tracker: '+str(exc))
        try:
            detector_config,tracker_config=load_spill_settings(args.settings)
            reference=BackgroundReference.load(args.background,confirmed_clean=True)
            self.detector=SpillDetector(reference,detector_config,self.calibration);self.tracker=SpillTracker(tracker_config)
            if self.calibration is not None:
                self.localizer=SurfaceLocalizer(self.calibration,LocalizationConfig(**json.loads(Path(args.localization_settings).read_text())),calibration_id=calibration_id)
                self.registration=current_frame_registration(self.localizer.calibration_id)
        except Exception as exc:self.startup.append('Spill detector/localizer: '+str(exc))
        floor=getattr(self.calibration,'floor',self.calibration)
        self.width=getattr(floor,'frame_width',args.width);self.height=getattr(floor,'frame_height',args.height)
        self.last_frame=np.zeros((self.height,self.width,3),np.uint8);self.previous=None
        self.records=[];self.paused=False

    def process(self,frame,*,camera_error=None):
        calibration,robot_tracker,detector,tracker,localizer,registration=(
            self.calibration,self.robot_tracker,self.detector,self.tracker,self.localizer,self.registration)
        messages=list(self.startup);robot=mask=result=None;detections=[];records=[]
        if frame is None:
            messages.append('Camera: '+str(camera_error or 'Camera returned no frame'))
            if tracker is not None:tracker.reset()
        else:
            self.last_frame=frame.copy()
            if robot_tracker is not None:
                try:
                    robot=robot_tracker.update(frame)
                    mask=robot_tracker.exclusion_mask(robot,frame.shape[1::-1])
                    if robot.position_cm is None:messages.append('Robot: '+robot.pose_quality['reason'])
                except Exception as exc:messages.append('Robot: '+str(exc))
            if detector is not None and tracker is not None:
                try:
                    result=detector.detect(frame,exclusion_mask=mask)
                    detections=tracker.update(result.detections)
                    if result.geometry.status!='MATCHED':messages.append('Spill geometry '+result.geometry.status+': '+result.geometry.reason)
                    if not detections:messages.append('Spills: no baseline-relative candidates detected')
                except Exception as exc:
                    tracker.reset();messages.append('Spill detection: '+str(exc))
                if result is not None and localizer is not None and registration is not None:
                    try:
                        records=localizer.localize_detections(result,detections,frame_size=frame.shape[1::-1],
                            registration=registration)
                    except Exception as exc:messages.append('Spill localization: '+str(exc))
        image=render_detections(self.last_frame,detections,debug=True)
        now=time.monotonic();fps=0. if self.previous is None else 1/max(now-self.previous,1e-6);self.previous=now
        if robot is not None:draw_robot_pose(image,robot,fps=fps)
        by_id={r.target_id:r for r in records};spill_values=[]
        for detection in detections:
            record=by_id.get(detection.target_id)
            position=ground_coordinates(record,calibration) if record is not None else None
            reason=(record.reason if record is not None else 'Awaiting confirmation' if not detection.confirmed else 'Localization unavailable; see diagnostic')
            if position is None and detection.confirmed:messages.append(f'Spill #{detection.target_id}: '+reason)
            label=(f"X={position['x']:.1f} Y={position['y']:.1f} cm" if position else 'Coordinates unavailable: '+reason)
            if position and 'height_cm' in position:label=f"Wall base {label} H={position['height_cm']:.1f} cm"
            x,y,w,h=detection.bounding_box
            message_overlay_at=(max(5,x),min(image.shape[0]-5,y+h+16))
            cv2.putText(image,label,message_overlay_at,0,.43,(0,0,0),3)
            cv2.putText(image,label,message_overlay_at,0,.43,(0,255,255),1)
            spill_values.append({'id':detection.target_id,'category':detection.category.value,
                'confirmed':detection.confirmed,'surface':record.surface if record else None,
                'position_cm':position,'reason':reason if position is None else None})
        message_overlay(image,messages+['DISPLAY ONLY | Q to exit'])
        self.records=records;self.paused=frame is None or bool(messages) or (result is not None and result.detection_paused)
        summary={'robot':None if robot is None else {'position_cm':robot.position_cm,
            'heading_deg':robot.heading_deg,'tracking':robot.tracking_status,'reason':robot.pose_quality['reason']},
            'spills':spill_values,'diagnostics':messages}
        return image,summary


def run_test_vision(args):
    view=LiveVision(args);last_print=float('-inf')
    window='Hospital Guardian - live robot + spills | Q exit'
    with CameraManager(args.camera,view.width,view.height,30) as camera:
        cv2.namedWindow(window,cv2.WINDOW_AUTOSIZE)
        while True:
            try:frame=camera.read();error=None
            except Exception as exc:frame=None;error=exc
            image,summary=view.process(frame,camera_error=error)
            cv2.imshow(window,image)
            if view.previous-last_print>=1.:
                print(json.dumps(summary,allow_nan=False),flush=True);last_print=view.previous
            if cv2.waitKey(30)&0xff in (ord('q'),ord('Q'),27):break
            try:
                if cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
            except cv2.error:break
