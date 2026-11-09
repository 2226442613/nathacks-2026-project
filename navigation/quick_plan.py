"""Read-only camera -> parking -> timed SIMPLE command preview. No transport."""
import json
import math
from pathlib import Path

import numpy as np

from .geometry import Arena
from .timing import angle_error,finite,load_timing
from vision import BackgroundReference,CameraManager,SpillDetector,SpillTracker,SurfaceLocalizer,load_calibration
from vision.cleaning_config import CleaningHardware
from vision.localization_runtime import LocalizationRegistration
from vision.robot_cli import build_robot_tracker
from vision.spill_tracking import load_spill_settings


def add_quick_plan_command(commands):
    p=commands.add_parser('quick-plan',help='Camera coordinates to MOVE preview only; never connects or moves')
    p.add_argument('--camera',type=int,default=0)
    for name,path in [('background','data/iphone_reference.png'),('calibration','config/camera_config.json'),
        ('robot-settings','config/robot_config.json'),('marker-plane','config/robot_marker_plane.json'),
        ('settings','config/spill_config.json'),('timing','config/movement_timing.json'),
        ('hardware','config/cleaning_hardware.json'),('navigation-settings','config/navigation_config.json')]:
        p.add_argument('--'+name,default=path)
    p.set_defaults(headless=True)


def timed_route(start,goal,timing,arena,cap_ms):
    """Shortest direct-line alternatives: turn, F/B, optional final heading turn."""
    if type(cap_ms) is not int or not 1<=cap_ms<=10000:raise ValueError('Invalid MOVE duration cap')
    routes=[]
    for drive in ('F','B'):
        pose={k:finite(start[k],k) for k in ('x_cm','y_cm','heading_deg')}
        commands=[];turns=0.
        if not arena.safe(pose) or not arena.safe(goal):continue
        def append(direction,amount):
            nonlocal turns
            rate,sign=timing.rate(direction);duration=round(abs(amount)*rate)
            if not duration:return
            end=dict(pose);change=sign*duration/rate
            if direction in 'LR':
                end['heading_deg']=(pose['heading_deg']+change)%360
                arena.check_segment(pose,end,turn_change=change);turns+=abs(change)
            else:
                theta=math.radians(pose['heading_deg'])
                end['x_cm']+=change*math.cos(theta);end['y_cm']+=change*math.sin(theta)
                arena.check_segment(pose,end)
            commands.extend(f'MOVE,{direction},{min(cap_ms,duration-i)}' for i in range(0,duration,cap_ms))
            pose.update(end)
        def turn(heading):
            error=angle_error(heading,pose['heading_deg'])
            if abs(abs(error)-180)<1e-8:
                direction=min(('L','R'),key=lambda d:timing.rate(d)[0])
            else:direction=timing.turn_direction(error) if abs(error)>1e-8 else None
            if direction:append(direction,abs(error))
        try:
            dx,dy=goal['x_cm']-pose['x_cm'],goal['y_cm']-pose['y_cm'];distance=math.hypot(dx,dy)
            if distance>1e-8:
                turn((math.degrees(math.atan2(dy,dx))+(180 if drive=='B' else 0))%360)
                append(drive,distance)
            turn(goal['heading_deg'])
            routes.append({'commands':commands,'estimated_final_pose':pose,'distance_cm':distance,
                'turn_degrees':turns,'duration_ms':sum(int(c.split(',')[2]) for c in commands)})
        except ValueError:continue  # Try the other direct-line direction; never route outside the floor.
    if not routes:raise ValueError('No simple turn-and-drive route fits the floor/chassis sweep')
    return min(routes,key=lambda r:(r['distance_cm'],r['turn_degrees'],r['duration_ms']))


def parking_candidates(target,start,arena,calibration,gap):
    pos=target.get('position_cm')
    if not pos:raise ValueError('Spill coordinates missing')
    surface=target.get('surface');candidates=[]
    if surface=='FLOOR':
        x,y=(finite(pos[k],k) for k in ('x','y'))
        b=(target.get('geometry_cm') or {}).get('bounds',{})
        lowx,highx=b.get('min_x',x),b.get('max_x',x);lowy,highy=b.get('min_y',y),b.get('max_y',y)
        headings={0.,90.,180.,270.,start['heading_deg']%360}
    elif surface=='WALL':
        x,y=calibration.wall_to_floor_base(finite(pos['x'],'wall x'),0)
        a=np.asarray(calibration.wall_to_floor_base(0,0));b=np.asarray(calibration.wall_to_floor_base(calibration.wall.width_cm,0))
        tangent=b-a;heading=math.degrees(math.atan2(tangent[1],tangent[0]))%360
        headings={heading,(heading+180)%360}
        normal=np.array([-tangent[1],tangent[0]],float);normal/=np.linalg.norm(normal)
        if np.dot(normal,np.array([arena.width/2,arena.depth/2])-a)<0:normal=-normal
    else:raise ValueError('Spill has no floor/wall coordinates')
    for h in sorted(headings):
        corners=np.concatenate(arena.corners({'x_cm':0.,'y_cm':0.,'heading_deg':h}))
        low,high=corners.min(axis=0),corners.max(axis=0)
        if np.any(high-low+2*gap>np.array([arena.width,arena.depth])):continue
        clamp=lambda value,axis:min(max(value,-low[axis]+gap),(arena.width,arena.depth)[axis]-high[axis]-gap)
        if surface=='WALL':
            radius=float(np.linalg.norm(corners,axis=1).max())
            center=np.array([x,y])+normal*max(gap-float((corners @ normal).min()),radius+gap)
            points=[(clamp(center[0],0),clamp(center[1],1))]
        else:
            points=[(lowx-high[0]-gap,clamp(y,1)),(highx-low[0]+gap,clamp(y,1)),
                    (clamp(x,0),lowy-high[1]-gap),(clamp(x,0),highy-low[1]+gap)]
        for X,Y in points:
            goal={'x_cm':float(X),'y_cm':float(Y),'heading_deg':float(h)}
            if arena.safe(goal):candidates.append(goal)
    return candidates


def coordinate_plan(robot,targets,timing,arena,calibration,gap,cap_ms):
    start={k:finite(robot[k],k) for k in ('x_cm','y_cm','heading_deg')}
    if not arena.safe(start):raise ValueError('Robot chassis/tool footprint is outside the floor')
    if not targets:raise ValueError('No confirmed spill coordinates available')
    plans=[]
    for target in targets:
        for goal in parking_candidates(target,start,arena,calibration,gap):
            try:route=timed_route(start,goal,timing,arena,cap_ms)
            except ValueError:continue
            plans.append({**route,'robot':start,'spill':target,'parking':goal})
    if not plans:raise ValueError('No beside-spill parking position with a floor-contained simple route')
    return min(plans,key=lambda p:(p['spill']['category']!='RED_SPILL',p['distance_cm'],p['turn_degrees'],p['duration_ms']))


def run_quick_plan(args):
    calibration=load_calibration(args.calibration)
    floor=getattr(calibration,'floor',calibration)
    robot_tracker=build_robot_tracker(args.robot_settings,args.calibration,marker_plane_path=args.marker_plane)
    detector_config,tracker_config=load_spill_settings(args.settings)
    reference=BackgroundReference.load(args.background,confirmed_clean=True)
    detector=SpillDetector(reference,detector_config,calibration);spills=SpillTracker(tracker_config)
    localizer=SurfaceLocalizer(calibration,calibration_id=robot_tracker.calibration_id)
    registration=LocalizationRegistration(reference.image,reference.image,declared=True,config=detector_config)
    records=[];robot=None
    with CameraManager(args.camera,floor.frame_width,floor.frame_height,30) as camera:
        # Existing multi-frame confirmation; no new startup or physical-motion gates.
        for _ in range(tracker_config.confirmation_frames):
            frame=camera.read()
            if frame is None:raise ValueError('No live camera frame')
            robot=robot_tracker.update(frame)
            mask=robot_tracker.exclusion_mask(robot,frame.shape[1::-1])
            result=detector.detect(frame,exclusion_mask=mask)
            reg=registration.for_frame(frame,result)
            records=localizer.localize_detections(result,spills.update(result.detections),frame_size=frame.shape[1::-1],registration=reg)
    if robot.tracking_status!='VISIBLE' or robot.position_cm is None or robot.heading_deg is None:
        raise ValueError('Current robot coordinates missing: '+robot.pose_quality['reason'])
    settings=json.loads(Path(args.navigation_settings).read_text());config=robot_tracker.config
    hardware=CleaningHardware(json.loads(Path(args.hardware).read_text()))
    arena=Arena(floor.width_cm,floor.height_cm,config.body_length_cm,config.body_width_cm,
        settings['clearance_margin_cm'],side_tool=hardware.side_collision_envelope())
    targets=[r.target_message() for r in records if r.surface in {'FLOOR','WALL'} and r.world_coordinates_cm is not None and r.localization_status!='INVALID']
    plan=coordinate_plan({'x_cm':robot.position_cm['x'],'y_cm':robot.position_cm['y'],'heading_deg':robot.heading_deg},
        targets,load_timing(args.timing),arena,calibration,settings['clearance_margin_cm'],settings['maximum_segment_ms'])
    print('PLAN ONLY: no ESP32 connection, movement or cleaning; timing estimates are approximate.')
    print('Robot:',json.dumps(plan['robot']),'localization:',robot.localization_status)
    print('Spill:',plan['spill']['surface'],plan['spill']['category'],json.dumps(plan['spill']['position_cm']))
    print('Detection geometry:',result.geometry.status,result.geometry.reason,'| spill localization:',plan['spill']['localization_status'])
    print('Parking:',json.dumps(plan['parking']))
    print('MOVE commands (ordered preview):')
    for command in plan['commands']:print(command)
    print('Estimated final pose:',json.dumps(plan['estimated_final_pose']))
    print('Route: shortest direct-line candidate, then least rotation/time; chassis and known static side-tool envelope only.')
