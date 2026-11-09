"""Synthetic Module 03 engineering fixtures; no physical camera or accuracy claims."""
from pathlib import Path
import argparse

import cv2
import numpy as np

from vision import GroundCalibration,DualPlaneCalibration,BackgroundReference


def generate_demo(directory):
    directory=Path(directory)
    if (directory/'baseline.png').exists() or (directory/'calibration.json').exists():
        raise ValueError('Synthetic fixture directory already contains a baseline/calibration; choose a fresh --output-dir')
    directory.mkdir(parents=True,exist_ok=True)
    floor=GroundCalibration([[20,220],[580,220],[580,420],[20,420]],120,80,600,440)
    calibration=DualPlaneCalibration.from_floor_and_wall_top(floor,[[20,20],[580,20]],120,60,'top')
    calibration.save(directory/'calibration.json')
    clean=np.full((440,600,3),(95,145,185),np.uint8)
    for y in (70,160,280,390):
        cv2.putText(clean,'LEGAL BACKGROUND',(35,y),0,.6,(30,30,30),1)
    cv2.rectangle(clean,(455,35),(530,80),(20,20,220),-1)
    BackgroundReference(clean,confirmed_clean=True).save(directory/'baseline.png')
    scene=clean.copy()
    for center,color in [((170,330),(20,20,220)),((365,110),(20,210,210)),((280,220),(30,30,220))]:
        cv2.circle(scene,center,17,color,-1)
    if not cv2.imwrite(str(directory/'targets.png'),scene):
        raise OSError('Cannot save synthetic target image')
    writer=cv2.VideoWriter(str(directory/'targets.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(600,440))
    if not writer.isOpened(): raise OSError('MJPG encoder unavailable')
    try:
        for i in range(16): writer.write(clean if i<3 or i>=13 else scene)
    finally:
        writer.release()
    return directory


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',default='data/module03_demo')
    args=parser.parse_args()
    print(generate_demo(args.output_dir))
