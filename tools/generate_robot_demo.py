"""Create NEW synthetic Module 04 camera fixtures, never real calibration/baselines."""
from pathlib import Path
import argparse
import cv2
import numpy as np
from vision import GroundCalibration,RobotConfig
from vision.robot_tracking import generate_marker


def generate_demo(directory):
    path=Path(directory)
    if path.exists(): raise ValueError('Choose a new demo directory')
    path.mkdir(parents=True)
    GroundCalibration([[0,0],[799,0],[799,599],[0,599]],19.975,14.975,800,600).save(path/'calibration.json')
    marker=generate_marker(RobotConfig(),240,40)
    blank=np.full((600,800,3),255,np.uint8)
    image=blank.copy();image[140:460,240:560]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(path/'robot.png'),image)
    writer=cv2.VideoWriter(str(path/'robot.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(800,600))
    if not writer.isOpened(): raise OSError('MJPG writer unavailable')
    try:
        for i in range(20):
            frame=blank.copy()
            if 3<=i<16:
                x=240+5*(i-3)
                tile=np.rot90(marker,3 if i>=10 else 0).copy()
                frame[140:460,x:x+320]=cv2.cvtColor(tile,cv2.COLOR_GRAY2BGR)
            writer.write(frame)
    finally: writer.release()
    return path


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',default='data/module04_demo')
    print(generate_demo(parser.parse_args().output_dir))
