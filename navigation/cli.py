"""Module06 CLI: deterministic demo, measured timing revisions, explicit physical run."""
from contextlib import ExitStack
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import time
from uuid import uuid4

from .controller import Navigator,NavigationConfig,NavigationFailure
from .geometry import Arena
from .timing import MovementTiming,load_timing,save_timing,angle_error
from .adapters import CameraPoses,ToolApproach
from .simulation import SimClock,SimPlant,SimTransport,demo_timing,demo_hardware
from vision.calibration import GroundCalibration,DualPlaneCalibration,load_calibration
from vision.cleaning_planner import CleaningPlanner
from vision.cleaning_config import CleaningHardware
from vision.cleaning_cli import load_targets
from vision.camera import CameraManager
from vision.robot_calibration_wizard import write_new_json
from robot_comm import RobotClient,WireLogger


def add_navigation_commands(commands):
    timing=commands.add_parser('movement-calibration',help='Record direction-specific measured timing; never sends movement')
    timing.add_argument('action',choices=['show','add','add-stop'])
    timing.add_argument('--file',default='config/movement_timing.json')
    timing.add_argument('--direction',choices=['F','B','L','R'])
    timing.add_argument('--duration-ms',type=int)
    timing.add_argument('--angle-deg',type=float,help='SIGNED Module04 heading change, not unsigned physical rotation')
    timing.add_argument('--distance-cm',type=float,help='Body-forward signed displacement: F positive, B negative')
    timing.add_argument('--before-pose');timing.add_argument('--after-pose')
    timing.add_argument('--confirm-measured',action='store_true')
    timing.add_argument('--pose-latency-ms',type=float)
    timing.add_argument('--stop-latency-ms',type=float)
    timing.add_argument('--maximum-speed',type=float,help='Measured upper speed bound: cm/s for F/B, deg/s for L/R')
    timing.add_argument('--coast-units',type=float,help='Measured residual coast after output shutdown: cm or deg')
    timing.set_defaults(headless=True)
    demo=commands.add_parser('simulate-navigation',help='Deterministic SIMPLE feedback task and return; no real hardware')
    demo.add_argument('--output-dir',default='data/module06_demo')
    demo.add_argument('--surface',choices=['FLOOR','WALL'],default='FLOOR')
    demo.add_argument('--move-gain',type=float,default=.85);demo.add_argument('--turn-gain',type=float,default=.85)
    demo.add_argument('--task-key',default='demo:0:1')
    demo.add_argument('--navigation-settings',default='config/navigation_config.json')
    demo.set_defaults(headless=True)
    nav=commands.add_parser('navigate-task',help='Explicit physical SIMPLE navigation; measured timing/tool geometry required')
    nav.add_argument('--host',required=True);nav.add_argument('--port',type=int,default=3333)
    nav.add_argument('--targets',required=True);nav.add_argument('--target-id',type=int,required=True)
    nav.add_argument('--hardware',required=True);nav.add_argument('--timing',required=True)
    nav.add_argument('--output-dir',required=True)
    nav.add_argument('--camera',type=int,default=0);nav.add_argument('--width',type=int,default=1280);nav.add_argument('--height',type=int,default=720)
    nav.add_argument('--calibration',default='config/camera_config.json')
    nav.add_argument('--robot-settings',default='config/robot_config.json')
    nav.add_argument('--marker-plane',default='config/robot_marker_plane.json')
    nav.add_argument('--navigation-settings',default='config/navigation_config.json')
    add_verification_options(nav)
    nav.set_defaults(headless=True)
    test=commands.add_parser('navigate-test',help='Navigation-only supervised round trip; simulation by default, no cleaning commands')
    test.add_argument('--mode',choices=['simulation','physical'],default='simulation')
    test.add_argument('--host');test.add_argument('--port',type=int,default=3333)
    test.add_argument('--x-cm',type=float,required=True);test.add_argument('--y-cm',type=float,required=True)
    test.add_argument('--heading-deg',type=float,help='Destination heading; defaults to initial observed heading')
    test.add_argument('--authorize-motion',action='store_true',help='Explicit operator initiation of this physical round trip')
    test.add_argument('--confirm-static-tools',action='store_true',help='Operator checked ALL parked tools and STOP homing fit the conservative body/side collision outline')
    test.add_argument('--camera',type=int,default=0);test.add_argument('--width',type=int,default=1280);test.add_argument('--height',type=int,default=720)
    test.add_argument('--calibration',default='config/camera_config.json');test.add_argument('--robot-settings',default='config/robot_config.json')
    test.add_argument('--marker-plane',default='config/robot_marker_plane.json')
    test.add_argument('--hardware',default='config/cleaning_hardware.json');test.add_argument('--timing',default='config/movement_timing.json')
    test.add_argument('--navigation-settings',default='config/navigation_config.json');test.add_argument('--output-dir',required=True)
    add_verification_options(test);test.set_defaults(headless=True)


def add_verification_options(parser):
    parser.add_argument('--verification-points',default='data/robot_plane_points_marker9.json')
    parser.add_argument('--confirm-fixed-camera',action='store_true')
    parser.add_argument('--confirm-ground-reference',action='store_true',help='Operator checked actual ground-center offsets and marker-to-forward heading, independently of plane fit')


def synthetic_calibration():
    floor=GroundCalibration([[20,220],[1260,220],[1260,700],[20,700]],121.6,68.4,1280,720)
    return DualPlaneCalibration.from_floor_and_wall_top(floor,[[20,20],[1260,20]],121.6,31.5,'top')


def synthetic_target(identity,surface):
    x,y=(90,40) if surface=='FLOOR' else (80,9)
    stamp=datetime.now(timezone.utc).isoformat()
    return {'target_id':1,'record_key':'demo:0:1','surface':surface,'category':'NON_RED_SPILL','active':True,
            'calibration_id':identity,'coordinate_frame':{'frame_id':identity},'timestamp':stamp,
            'localization_status':'UNVERIFIED','geometry_status':'UNVERIFIED',
            'geometry_cm':{'contours':[{'kind':'outer','points':[[x-1,y-1],[x+1,y-1],[x+1,y+1],[x-1,y+1]]}]}}


def make_demo(surface='FLOOR',config=None,**plant_options):
    config=config or NavigationConfig();calibration=synthetic_calibration();hardware=CleaningHardware(demo_hardware())
    clock=SimClock();plant=SimPlant(clock,home=(32,13,0) if surface=='WALL' else (32,29,0),**plant_options)
    transport=SimTransport(plant);timing=demo_timing(plant.move_gain,plant.turn_gain)
    planner=CleaningPlanner(calibration,hardware,mode='SIMULATION',calibration_id='SIMULATED_MODULE06_ARENA')
    target=synthetic_target(planner.calibration_id,surface)
    plan=planner.plan(target)
    arena=Arena(planner.width,planner.depth,26,15.2,config.clearance_margin_cm,tool=hardware.data['floor_sponge'])
    adapter=ToolApproach(hardware,calibration,arena,timing)
    prepared=adapter.prepare(plan)
    return clock,plant,transport,timing,arena,adapter,prepared


def measured_sample(direction,before,after):
    for value in (before,after):
        if (value.get('message_type')!='ROBOT_POSE' or value.get('tracking_status')!='VISIBLE' or
            value.get('localization_status') in {'INVALID','UNAVAILABLE'} or not value.get('position_cm') or
            value.get('pose_quality',{}).get('mapping_method')!='EMPIRICAL_FIXED_HEIGHT_PLANE'):
            raise ValueError('Timing samples require actual current Module04 ground-center poses')
    if before.get('calibration_id')!=after.get('calibration_id') or before.get('marker_configuration')!=after.get('marker_configuration'):
        raise ValueError('Before/after pose context mismatch')
    if datetime.fromisoformat(after['timestamp'].replace('Z','+00:00'))<=datetime.fromisoformat(before['timestamp'].replace('Z','+00:00')):
        raise ValueError('After pose must be newer')
    delta=[after['position_cm'][k]-before['position_cm'][k] for k in ('x','y')]
    change=angle_error(after['heading_deg'],before['heading_deg'])
    if direction in 'LR':
        if math.hypot(*delta)>2.5:raise ValueError('Rotation sample translated too far; inspect calibration')
        return change
    if abs(change)>7:raise ValueError('Straight sample heading changed too far')
    a=math.radians(before['heading_deg']);along=delta[0]*math.cos(a)+delta[1]*math.sin(a)
    across=-delta[0]*math.sin(a)+delta[1]*math.cos(a)
    if abs(across)>max(1.,.2*abs(along)):raise ValueError('Straight sample has excessive cross drift')
    return along


def run_timing(args):
    timing=load_timing(args.file)
    if args.action=='show':print(json.dumps(timing.summary(),indent=2));return
    if args.action=='add-stop':
        timing.add_stopping(direction=args.direction,pose_latency_ms=args.pose_latency_ms,stop_latency_ms=args.stop_latency_ms,
            maximum_speed_per_second=args.maximum_speed,coast_units=args.coast_units,
            verified=args.confirm_measured,source='operator measured stopping upper bounds')
        print('Saved immutable stopping revision:',save_timing(args.file,timing));return
    if not args.direction or args.duration_ms is None:raise ValueError('Add requires direction and duration-ms')
    if bool(args.before_pose)!=bool(args.after_pose):raise ValueError('Provide both before/after poses')
    if args.before_pose:
        before=json.loads(Path(args.before_pose).read_text());after=json.loads(Path(args.after_pose).read_text())
        amount=measured_sample(args.direction,before,after);source='Module04 before/after ground-center observations'
    else:
        amount=args.angle_deg if args.direction in 'LR' else args.distance_cm
        source='operator measured signed change'
    timing.add(direction=args.direction,duration_ms=args.duration_ms,observed_change=amount,
               verified=args.confirm_measured,source=source)
    output=save_timing(args.file,timing)
    print('Saved immutable timing revision:',output)
    print(json.dumps(timing.summary(),indent=2))


def selected_target(path,target_id,identity):
    targets=[r for r in load_targets(path) if r.get('target_message',r).get('target_id')==target_id]
    if len(targets)!=1:raise ValueError('Select one unambiguous current target record')
    value=targets[0];body=value.get('target_message',value)
    stamp=body.get('last_observed_timestamp',body.get('timestamp'))
    age=datetime.now(timezone.utc).timestamp()-datetime.fromisoformat(stamp.replace('Z','+00:00')).timestamp()
    if value.get('active',body.get('active',True)) is not True or not 0<=age<=2:
        raise ValueError('Target is inactive/stale; provide a refreshed Module03 snapshot')
    if body.get('calibration_id')!=identity:raise ValueError('Target calibration mismatch')
    return value,body


def run_navigation(args):
    if args.command=='movement-calibration':return run_timing(args)
    config=NavigationConfig(**json.loads(Path(args.navigation_settings).read_text()))
    out=Path(args.output_dir)
    if out.exists():raise ValueError('Use a new output directory; logs never overwrite evidence')
    out.mkdir(parents=True,exist_ok=False)
    if args.command=='navigate-test':return run_navigation_test(args,config,out)
    mode='SIMULATION' if args.command=='simulate-navigation' else 'PHYSICAL'
    with ExitStack() as stack:
        if mode=='SIMULATION':
            clock,poses,transport,timing,arena,adapter,plan=make_demo(args.surface,config,
                move_gain=args.move_gain,turn_gain=args.turn_gain)
            sleep=clock.sleep;category='NON_RED_SPILL';task_key=args.task_key;identity='SIMULATED_MODULE06_ARENA'
        else:
            from vision.robot_cli import load_robot_config
            calibration=load_calibration(args.calibration)
            if not isinstance(calibration,DualPlaneCalibration) or calibration.floor_boundary!='top':
                raise ValueError('Navigation requires native X along top wall and Y away from it')
            floor=calibration.floor;robot=load_robot_config(args.robot_settings)
            if (args.width,args.height)!=(floor.frame_width,floor.frame_height):raise ValueError('Camera resolution mismatch')
            if (robot.body_length_cm,robot.body_width_cm,robot.mounting_height_cm)!=(26.,15.2,16.6):
                raise ValueError('Navigation profile does not match measured robot geometry')
            identity='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest()
            hardware=CleaningHardware(json.loads(Path(args.hardware).read_text()))
            timing=load_timing(args.timing)
            if hardware.mode!='PHYSICAL' or timing.data['mode']!='PHYSICAL':raise ValueError('Simulation measurements cannot authorize physical execution')
            if (hardware.data['body']['chassis_length_cm'],hardware.data['body']['chassis_width_cm'])!=(robot.body_length_cm,robot.body_width_cm):
                raise ValueError('Cleaning/chassis geometry mismatch')
            value,body=selected_target(args.targets,args.target_id,identity)
            missing=hardware.missing(body['surface'])
            if missing:raise NavigationFailure('CONFIGURATION_REQUIRED: '+', '.join(missing))
            arena=Arena(floor.width_cm,floor.height_cm,robot.body_length_cm,robot.body_width_cm,
                        max(config.clearance_margin_cm,hardware.data['body']['safety_margin_cm']+hardware.data['body']['position_uncertainty_cm']),
                        tool=hardware.data['floor_sponge'],side_tool=hardware.side_collision_envelope())
            planner=CleaningPlanner(calibration,hardware,mode='PHYSICAL',calibration_id=identity)
            adapter=ToolApproach(hardware,calibration,arena,timing);plan=adapter.prepare(planner.plan(value))
            # Validate files/configuration BEFORE opening camera or network.
            poses=CameraPoses(None,args.calibration,args.robot_settings,args.marker_plane,verification_points_path=args.verification_points,
                fixed_camera=args.confirm_fixed_camera,ground_reference_confirmed=args.confirm_ground_reference)
            camera=stack.enter_context(CameraManager(args.camera,args.width,args.height,30))
            poses.camera=camera
            poses.start(config.observation_gap_seconds,startup_wait_seconds=2.);stack.callback(poses.close)
            clock=time.monotonic;sleep=time.sleep;category=body['category'];task_key=body.get('record_key') or f'physical:{args.target_id}'
            logger=WireLogger(out/'wire.log');stack.callback(logger.close)
            transport=stack.enter_context(RobotClient(args.host,args.port,mode='physical',firmware_profile='simple',logger=logger))
        stream=stack.enter_context((out/'progress.jsonl').open('x'))
        session=uuid4().hex
        def emit(event):
            record={**event,'mode':mode,'session_id':session,'calibration_id':identity,
                    'timestamp':datetime.now(timezone.utc).isoformat()}
            stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush()
        def verify(plan,pose):
            if mode=='PHYSICAL':
                current,current_body=selected_target(args.targets,args.target_id,identity)
                if current_body['category']!=category:raise NavigationFailure('Target classification changed; replan required')
                updated=planner.plan(current)
                if updated.target_geometry!=plan['target_geometry']:raise NavigationFailure('Target geometry changed; replan required')
            adapter.check(plan,pose)
        controller=Navigator(poses,transport,timing,arena,config,clock=clock,sleep=sleep,on_event=emit,check_approach=verify)
        try:result=controller.run(plan,category,task_key=task_key)
        except BaseException:
            transport.send_stop();raise
        write_new_json(out/'summary.json',{**result,'mode':mode,'session_id':session,'calibration_id':identity,
            'physical_accuracy_claim':False,'cleaning_effectiveness_verified':False})
        write_new_json(out/'plan.json',plan)
        if mode=='SIMULATION':write_new_json(out/'commands.json',{'mode':mode,'commands':transport.lines})
        print(json.dumps(result,indent=2))
        print('Evidence:',out)
        if result['state']!='READY':raise NavigationFailure(result.get('reason','Navigation did not complete'))


def run_navigation_test(args,config,out):
    """Separate entry point: no target ingestion, cleaning planner or cleaning API."""
    mode=args.mode.upper()
    with ExitStack() as stack:
        logger=WireLogger(out/'wire.log');stack.callback(logger.close)
        if mode=='SIMULATION':
            clock=SimClock();poses=SimPlant(clock);sleep=clock.sleep
            timing=demo_timing(poses.move_gain,poses.turn_gain)
            arena=Arena(margin=config.clearance_margin_cm,tool=demo_hardware()['floor_sponge'])
            transport=SimTransport(poses);transport.logger=logger
            logger.emit('INFO','SIMULATION ONLY; synthetic pose, timing and stopping bounds')
            identity='SIMULATED_MODULE06_ARENA'
        else:
            if not (args.authorize_motion and args.confirm_fixed_camera and args.confirm_ground_reference and args.confirm_static_tools):
                raise ValueError('Physical test requires explicit motion initiation and checked fixed camera, ground reference and static tool clearance')
            if not args.host:raise ValueError('Physical test requires an explicit host')
            from vision.robot_cli import load_robot_config
            calibration=load_calibration(args.calibration);robot=load_robot_config(args.robot_settings)
            if not isinstance(calibration,DualPlaneCalibration) or calibration.floor_boundary!='top':raise ValueError('Native top-wall coordinate frame required')
            floor=calibration.floor
            if (args.width,args.height)!=(floor.frame_width,floor.frame_height):raise ValueError('Camera resolution mismatch')
            hardware=CleaningHardware(json.loads(Path(args.hardware).read_text()));timing=load_timing(args.timing)
            if hardware.mode!='PHYSICAL' or timing.data['mode']!='PHYSICAL':raise ValueError('Physical test cannot use simulation profiles')
            if (hardware.data['body']['chassis_length_cm'],hardware.data['body']['chassis_width_cm'])!=(robot.body_length_cm,robot.body_width_cm):raise ValueError('Chassis geometry mismatch')
            for direction in ('F','B','L','R'):timing.rate(direction)
            sponge=hardware.data['floor_sponge']
            known_floor=(sponge['installed'] is True and all(sponge[k] is not None for k in ('contact_length_cm','contact_width_cm','offset_forward_cm','offset_lateral_cm')))
            # Unknown tools must be physically confirmed contained in this outline;
            # no cleaning contact dimensions or spray coverage are inferred.
            arena=Arena(floor.width_cm,floor.height_cm,robot.body_length_cm,robot.body_width_cm,config.clearance_margin_cm,
                tool=sponge if known_floor else None,side_tool=hardware.side_collision_envelope())
            poses=CameraPoses(None,args.calibration,args.robot_settings,args.marker_plane,verification_points_path=args.verification_points,
                fixed_camera=args.confirm_fixed_camera,ground_reference_confirmed=args.confirm_ground_reference,
                log=lambda message:logger.emit('LOCALIZATION',message))
            camera=stack.enter_context(CameraManager(args.camera,args.width,args.height,30));poses.camera=camera
            poses.start(config.observation_gap_seconds,startup_wait_seconds=2.);stack.callback(poses.close)
            poses.observe()  # Reject unverified/currently lost localization before connecting TCP.
            clock=time.monotonic;sleep=time.sleep
            identity='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest()
            transport=stack.enter_context(RobotClient(args.host,args.port,mode='physical',firmware_profile='simple',logger=logger))
        initial=poses.observe()
        goal={'x_cm':args.x_cm,'y_cm':args.y_cm,'heading_deg':initial.heading_deg if args.heading_deg is None else args.heading_deg}
        stream=stack.enter_context((out/'progress.jsonl').open('x'));session=uuid4().hex
        def emit(event):
            stream.write(json.dumps({**event,'mode':mode,'calibration_id':identity,'session_id':session},allow_nan=False)+'\n');stream.flush()
        controller=Navigator(poses,transport,timing,arena,config,clock=clock,sleep=sleep,on_event=emit)
        try:result=controller.run_navigation_only(goal,task_key='navigation-test:'+session)
        except BaseException:transport.send_stop();raise
        write_new_json(out/'summary.json',{**result,'mode':mode,'destination':goal,'physical_accuracy_claim':False})
        if mode=='SIMULATION':write_new_json(out/'commands.json',{'mode':mode,'commands':transport.lines})
        print(json.dumps(result,indent=2));print('Navigation-only evidence:',out)
        if result['state']!='READY':raise NavigationFailure(result.get('reason','Navigation test failed'))
