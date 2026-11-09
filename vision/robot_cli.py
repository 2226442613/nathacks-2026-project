"""Module 04 standalone tools, using the existing CameraManager on the main thread."""
from contextlib import ExitStack
from dataclasses import asdict
import hashlib
import json
import time
from pathlib import Path

import cv2

from .camera import CameraManager
from .calibration import load_calibration, DualPlaneCalibration
from .frame_geometry import FrameGeometryGuard
from .localization import CameraRegistration
from .robot_tracking import RobotConfig, RobotTracker, generate_marker, draw_robot_pose
from .robot_plane import MarkerPlaneMapping
from .serialization import json_line


def load_robot_config(path,**overrides):
    values=json.loads(Path(path).read_text())
    values.update({k:v for k,v in overrides.items() if v is not None})
    return RobotConfig(**values)


def add_robot_commands(commands):
    marker=commands.add_parser('generate-aruco',help='Generate a printable marker PNG with white quiet zone')
    marker.add_argument('--output',required=True)
    marker.add_argument('--side-pixels',type=int,default=1200,help='BLACK square size; excludes quiet zone')
    marker.add_argument('--quiet-zone-pixels',type=int,default=200)
    marker.add_argument('--dictionary',default='DICT_4X4_50')
    marker.add_argument('--marker-id',type=int,default=0)
    marker.add_argument('--marker-size-cm',type=float,default=6.,help='Intended BLACK-square print size; must be measured with ruler')
    marker.set_defaults(headless=True)
    track=commands.add_parser('track-robot',help='Standalone ArUco robot pixels, position estimates and heading')
    sources=track.add_mutually_exclusive_group(required=True)
    sources.add_argument('--camera',type=int);sources.add_argument('--video');sources.add_argument('--image')
    track.add_argument('--width',type=int,default=1280);track.add_argument('--height',type=int,default=720);track.add_argument('--fps',type=float,default=30.)
    track.add_argument('--calibration',default='config/camera_config.json')
    track.add_argument('--robot-settings',default='config/robot_config.json')
    track.add_argument('--marker-size-cm',type=float);track.add_argument('--mounting-height-cm',type=float)
    track.add_argument('--marker-plane',help='Independently measured fixed-height mapping JSON')
    track.add_argument('--verification-points',help='Matching measured session JSON; read only')
    track.add_argument('--confirm-fixed-camera',action='store_true',help='Operator confirms current camera has not moved since calibration')
    track.add_argument('--confirm-ground-reference',action='store_true',help='Operator has checked chassis ground-center offsets and marker-to-forward heading')
    track.add_argument('--alignment-reference',help='Optional frozen calibration-pose image; warnings remain non-blocking')
    track.add_argument('--headless',action='store_true');track.add_argument('--output',help='Optional robot-pose JSONL; otherwise stdout')
    track.add_argument('--annotated-dir',help='Optional saved annotated frames')
    track.add_argument('--max-frames',type=int)
    plane=commands.add_parser('calibrate-robot-plane',help='Fit elevated marker-plane mapping from independent measured points')
    plane.add_argument('--points',required=True);plane.add_argument('--calibration',default='config/camera_config.json')
    plane.add_argument('--mounting-height-cm',type=float,required=True);plane.add_argument('--tolerance-cm',type=float,default=2.)
    plane.add_argument('--confirm-measured',action='store_true',required=True);plane.add_argument('--output',required=True)
    plane.set_defaults(headless=True)
    wizard=commands.add_parser('calibrate-robot-interactive',help='Live measured marker-ground-projection calibration wizard')
    wizard.add_argument('--camera',type=int,default=0)
    wizard.add_argument('--width',type=int,default=1280)
    wizard.add_argument('--height',type=int,default=720)
    wizard.add_argument('--fps',type=float,default=30.)
    wizard.add_argument('--calibration',default='config/camera_config.json')
    wizard.add_argument('--robot-settings',default='config/robot_config.json')
    wizard.add_argument('--mounting-height-cm',type=float,help='Measured height; must match settings when configured')
    wizard.add_argument('--points-output',default='data/robot_plane_points.json')
    wizard.add_argument('--resume',action='store_true',help='Load existing --points-output; preserve it and save numbered snapshots')
    wizard.add_argument('--output',default='config/robot_marker_plane.json')
    wizard.add_argument('--tolerance-cm',type=float,default=2.)
    wizard.set_defaults(headless=False)


def build_robot_tracker(settings,calibration_path,*,marker_plane_path=None,verification_points_path=None,fixed_camera=False,ground_reference_confirmed=False,**overrides):
    config=load_robot_config(settings,**overrides)
    calibration=load_calibration(calibration_path) if calibration_path else None
    identity='sha256:'+hashlib.sha256(Path(calibration_path).read_bytes()).hexdigest() if calibration_path else None
    plane=MarkerPlaneMapping(json.loads(Path(marker_plane_path).read_text())) if marker_plane_path else None
    tracker=RobotTracker(calibration,config,calibration_id=identity,marker_plane=plane)
    if verification_points_path:
        tracker.verify_context(json.loads(Path(verification_points_path).read_text()),fixed_camera=fixed_camera,ground_reference_confirmed=ground_reference_confirmed)
    return tracker


def new_output(path,protected=()):
    path=Path(path)
    if path.exists() or any(value and path.resolve()==Path(value).resolve() for value in protected): raise ValueError('Output already exists or would overwrite an input; choose a new path')
    path.parent.mkdir(parents=True,exist_ok=True)
    return path


def run_robot_command(args):
    if args.command=='calibrate-robot-interactive':
        from .robot_calibration_wizard import run_wizard
        run_wizard(args)
        return
    if args.command=='generate-aruco':
        config=RobotConfig(dictionary=args.dictionary,marker_id=args.marker_id,marker_size_cm=args.marker_size_cm)
        output=new_output(args.output)
        info=Path(str(output)+'.json')
        if info.exists(): raise ValueError('Print metadata already exists; choose a new output')
        if output.suffix.lower()!='.png': raise ValueError('Generate marker as PNG')
        image=generate_marker(config,args.side_pixels,args.quiet_zone_pixels)
        if not cv2.imwrite(str(output),image): raise OSError('Cannot save marker')
        total=config.marker_size_cm*image.shape[0]/args.side_pixels
        metadata={'dictionary':config.dictionary,'marker_id':config.marker_id,'black_square_pixels':args.side_pixels,
            'quiet_zone_each_side_pixels':args.quiet_zone_pixels,'intended_black_square_cm':config.marker_size_cm,
            'required_total_image_width_cm':total,'physical_size_verified':False,
            'print_instruction':'Print square image without crop/distortion at required total width; disable fit-to-page. Measure BLACK square with a ruler in both axes; adjust until intended size.'}
        info.write_text(json_line(metadata)+'\n');print(json_line(metadata));return
    if args.command=='calibrate-robot-plane':
        output=new_output(args.output,[args.points,args.calibration])
        calibration=load_calibration(args.calibration);floor=calibration.floor if isinstance(calibration,DualPlaneCalibration) else calibration
        data=json.loads(Path(args.points).read_text())
        mapping=MarkerPlaneMapping.fit(data['training'],data['validation'],
            calibration_id='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest(),
            frame_size=(floor.frame_width,floor.frame_height),floor_size_cm=(floor.width_cm,floor.height_cm),
            height_cm=args.mounting_height_cm,tolerance_cm=args.tolerance_cm,attested=args.confirm_measured)
        output.write_text(json_line(mapping.to_dict())+'\n');return
    if args.max_frames is not None and args.max_frames<1: raise ValueError('max-frames must be positive')
    tracker=build_robot_tracker(args.robot_settings,args.calibration,marker_plane_path=args.marker_plane,
        marker_size_cm=args.marker_size_cm,mounting_height_cm=args.mounting_height_cm,
        verification_points_path=args.verification_points,fixed_camera=args.confirm_fixed_camera,ground_reference_confirmed=args.confirm_ground_reference)
    protected=[args.image,args.video,args.calibration,args.robot_settings,args.marker_plane,args.alignment_reference,args.verification_points]
    output=new_output(args.output,protected) if args.output else None
    directory=Path(args.annotated_dir) if args.annotated_dir else None
    if directory:
        if directory.exists(): raise ValueError('Annotated directory already exists; choose a new path')
        directory.mkdir(parents=True)
    guard=None
    if args.alignment_reference:
        reference=cv2.imread(args.alignment_reference)
        if reference is None: raise ValueError('Cannot read alignment reference')
        guard=FrameGeometryGuard(reference)
    with ExitStack() as stack:
        stream=stack.enter_context(output.open('w')) if output else None
        camera=None if args.image else stack.enter_context(CameraManager(args.video if args.video else args.camera,args.width,args.height,args.fps))
        index=0
        while args.max_frames is None or index<args.max_frames:
            start=time.perf_counter()
            frame=cv2.imread(args.image) if args.image else camera.read()
            if frame is None:
                if args.image: raise ValueError('Cannot read robot image')
                break
            reg=None
            if guard:
                geometry=guard.check(frame)
                reg=CameraRegistration.from_reference_to_current(guard.transform,frame.shape[1::-1]) if geometry.status=='MATCHED' else CameraRegistration(status=geometry.status,reason=geometry.reason)
            pose=tracker.update(frame,registration=reg)
            line=json_line(pose.to_dict())
            if stream: stream.write(line+'\n')
            else: print(line)
            fps=1/max(time.perf_counter()-start,1e-9)
            if not args.headless or directory:
                annotated=frame.copy();draw_robot_pose(annotated,pose,fps=fps)
                if directory and not cv2.imwrite(str(directory/f'frame_{index:06d}.png'),annotated): raise OSError('Cannot save annotated frame')
                if not args.headless:
                    title='Hospital Guardian - robot tracker | Q exit';cv2.imshow(title,annotated)
                    key=cv2.waitKey(1)&255
                    if key in (27,ord('q'),ord('Q')) or cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1: break
            index+=1
            if args.image: break
