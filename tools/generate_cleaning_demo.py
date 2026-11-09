"""NEW Module 05 fixtures from Module 03/04 APIs. No physical hardware claims."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import time

import cv2
import numpy as np

from vision import (load_calibration,DualPlaneCalibration,SpillDetection,SpillCategory,
                    SurfaceLocalizer,CameraRegistration,TargetEventStore,RobotTracker,RobotConfig)
from vision.robot_tracking import generate_marker
from vision.serialization import json_line


def generate_demo(directory,calibration_path='config/camera_config.json'):
    directory=Path(directory)
    if directory.exists(): raise ValueError('Choose a new demo directory; existing experiments are preserved')
    calibration=load_calibration(calibration_path)
    if not isinstance(calibration,DualPlaneCalibration): raise ValueError('Dual-plane calibration required for this demo')
    directory.mkdir(parents=True)
    shutil.copyfile(calibration_path,directory/'calibration.json')
    identity='sha256:'+hashlib.sha256((directory/'calibration.json').read_bytes()).hexdigest()
    floor=calibration.floor;w,d=floor.width_cm,floor.height_cm;ww=calibration.wall.width_cm;wh=calibration.wall.height_cm
    size=(floor.frame_width,floor.frame_height)
    localizer=SurfaceLocalizer(calibration,calibration_id=identity)
    def polygon(mask,points,surface,value=255):
        project=calibration.floor_to_pixel if surface=='FLOOR' else calibration.wall_to_pixel
        cv2.fillPoly(mask,[np.rint([project(*p) for p in points]).astype(np.int32)],value)
    masks=[]
    mask=np.zeros(size[::-1],np.uint8)
    polygon(mask,[[.20*w,.25*d],[.55*w,.25*d],[.55*w,.45*d],[.40*w,.45*d],[.40*w,.70*d],[.20*w,.70*d]],'FLOOR')
    polygon(mask,[[.25*w,.35*d],[.30*w,.35*d],[.30*w,.45*d],[.25*w,.45*d]],'FLOOR',0)
    masks.append(mask)
    for x0,x1,h0,h1 in ((.60*ww,.80*ww,.10*wh,.55*wh),(.15*ww,.25*ww,.03*wh,.09*wh)):
        mask=np.zeros(size[::-1],np.uint8);polygon(mask,[[x0,h0],[x1,h0],[x1,h1],[x0,h1]],'WALL');masks.append(mask)
    records=[];now=time.time();image=np.full((*size[::-1],3),235,np.uint8)
    for index,mask in enumerate(masks,1):
        ys,xs=np.nonzero(mask);x,y=int(xs.min()),int(ys.min());bw,bh=int(xs.max()-x+1),int(ys.max()-y+1)
        contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]
        contour=max(contours,key=cv2.contourArea)[:,0]
        target=SpillDetection(SpillCategory.RED_SPILL if index==1 else SpillCategory.NON_RED_SPILL,
            (float(xs.mean()),float(ys.mean())),(x,y,bw,bh),tuple(map(tuple,contour)),int(len(xs)),1.,'synthetic Module 05 fixture',
            target_id=index,confirmed=True,first_seen=now,last_seen=now)
        reg=CameraRegistration.from_reference_to_current(np.eye(3),size)
        records.append(localizer.localize(target,mask,frame_size=size,registration=reg))
        image[mask!=0]=(20,20,220) if index==1 else (20,200,200)
    TargetEventStore(directory/'targets.jsonl',session_id='module05_synthetic').update(records)
    if not cv2.imwrite(str(directory/'synthetic_targets.png'),image): raise OSError('Cannot save target fixture')
    robot_image=np.full_like(image,255)
    marker=cv2.cvtColor(generate_marker(RobotConfig(),96,16),cv2.COLOR_GRAY2BGR)
    center=calibration.floor_to_pixel(.50*w,.65*d)
    x=int(np.clip(center[0]-64,0,size[0]-128));y=int(np.clip(center[1]-64,0,size[1]-128))
    robot_image[y:y+128,x:x+128]=marker
    robot=RobotTracker(calibration,calibration_id=identity).update(robot_image)
    (directory/'robot_pose.jsonl').write_text(json_line(robot.to_dict())+'\n')
    (directory/'source_metadata.json').write_text(json_line({'synthetic_fixture':True,'real_robot_or_cleaning_validation':False,
        'camera_calibration_copied_read_only_from':str(calibration_path),'hardware_geometry':'config/cleaning_demo.json illustrative only'})+'\n')
    return directory


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',default='data/module05_demo')
    parser.add_argument('--calibration',default='config/camera_config.json')
    args=parser.parse_args();print(generate_demo(args.output_dir,args.calibration))
