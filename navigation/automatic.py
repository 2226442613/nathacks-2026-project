"""Small sequential MVP coordinator; ESP32 exclusively owns physical cleaning."""
from contextlib import ExitStack
import hashlib
import json
import logging
import math
from pathlib import Path
import time
from uuid import uuid4

import cv2
import numpy as np

from .cli import (add_verification_options,synthetic_calibration,NavigationConfig,Arena,
    CameraPoses,ToolApproach,Navigator,NavigationFailure,SimClock,SimPlant,SimTransport,
    demo_timing,demo_hardware,CleaningHardware,CleaningPlanner,CameraManager,RobotClient,WireLogger)
from .controller import Pose
from .timing import load_timing
from .geometry import front_parking_candidates,front_bumper_metrics,spill_bounds
from vision import BackgroundReference,SpillDetector,SpillTracker,SurfaceLocalizer,load_calibration
from vision.localization import CameraRegistration,LocalizationConfig
from vision.localization_runtime import image_identity
from vision.spill_tracking import load_spill_settings
from vision.robot_cli import load_robot_config
from vision.live_test import current_frame_registration


def add_auto_command(commands):
    p=commands.add_parser('auto-clean',help='Detect -> prioritize -> observed arrival -> ESP32 clean -> next target; simulation by default')
    p.add_argument('--mode',choices=['simulation','physical'],default='simulation')
    p.add_argument('--navigation-only',action='store_true',help='Auto-select a spill, navigate and return; never invoke cleaning')
    p.add_argument('--return-home',action='store_true',help='Optional legacy Y/X round trip after each clean; default continues directly to the next target')
    p.add_argument('--confirm-navigation-test',action='store_true',help='Explicitly authorize navigation; confirm current legal background/camera, chassis reference, clear route, parked tool outline and cleaning-actuator power isolated')
    p.add_argument('--host');p.add_argument('--port',type=int,default=3333)
    p.add_argument('--connect-timeout',type=float,default=5.,help='Initial TCP connection timeout in seconds (0 < value <= 10); command/STOP deadlines unchanged')
    p.add_argument('--background');p.add_argument('--confirm-clean',action='store_true')
    p.add_argument('--authorize-motion',action='store_true');p.add_argument('--confirm-static-tools',action='store_true')
    p.add_argument('--camera',type=int,default=0);p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=720)
    p.add_argument('--calibration',default='config/camera_config.json');p.add_argument('--robot-settings',default='config/robot_config.json')
    p.add_argument('--marker-plane',default='config/robot_marker_plane.json');p.add_argument('--hardware',default='config/cleaning_hardware.json')
    p.add_argument('--timing',default='config/movement_timing.json');p.add_argument('--settings',default='config/spill_config.json')
    p.add_argument('--navigation-settings',default='config/navigation_config.json')
    p.add_argument('--localization-settings',default='config/localization_config.json')
    p.add_argument('--output-dir');p.add_argument('--max-tasks',type=int)
    add_verification_options(p);p.set_defaults(headless=True)


class SceneScanner:
    def __init__(self,reference,calibration,identity,settings,frame_source,*,physical=False,navigation_only=False,logger=None,localization_settings=None):
        detector_config,tracker_config=load_spill_settings(settings)
        self.detector=SpillDetector(reference,detector_config,calibration)
        self.tracker=SpillTracker(tracker_config);self.frames=tracker_config.confirmation_frames
        self.localizer=SurfaceLocalizer(calibration,LocalizationConfig(**(localization_settings or {})),calibration_id=identity)
        # A newly captured legal background is NOT the original calibration view.
        # Keep robot and spill geometry in current pixels, using Module01 once.
        self.background_id=image_identity(reference.image)
        self.registration=current_frame_registration(identity)
        self.frame_source=frame_source;self.physical=physical
        self.navigation_only=navigation_only;self.logger=logger

    def _check_geometry(self,result,frame,registration):
        """Background fit quality warns; invalid coordinate/image transforms stop."""
        config=self.detector.config;values=result.geometry.to_dict();problem=None;warnings=[]
        limits={'center_shift_pixels':config.geometry_max_shift_pixels,
                'rotation_degrees':config.geometry_max_rotation_degrees,'scale_change':config.geometry_max_scale_change,
                'corner_displacement_pixels':config.geometry_max_shift_pixels}
        values['corner_displacement_pixels']=None
        values['calibrated_region_displacement_pixels']=None
        h,w=frame.shape[:2];corners=np.array([[0,0],[w-1,0],[w-1,h-1],[0,h-1]],float)
        matrices=[result.background_alignment_transform,result.reference_to_current,registration.current_to_calibration]
        for index,matrix in enumerate(matrices):
            if matrix is None:continue
            matrix=np.asarray(matrix,float)
            if matrix.shape!=(3,3) or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix)!=3:
                problem='Invalid image registration transform';break
            raw=np.column_stack((corners,np.ones(4))) @ matrix.T
            if not ((raw[:,2]>1e-9).all() or (raw[:,2]<-1e-9).all()):
                problem='Image registration crosses projective horizon';break
            mapped=raw[:,:2]/raw[:,2:]
            if not np.isfinite(mapped).all() or cv2.contourArea(mapped.astype(np.float32),oriented=True)<=0:
                problem='Invalid image registration orientation/projection';break
            if index<2 and values['corner_displacement_pixels'] is None:
                values['corner_displacement_pixels']=float(np.linalg.norm(mapped-corners,axis=1).max())
                if self.detector.polygons:
                    points=np.concatenate(list(self.detector.polygons.values())).astype(float)
                    projected=cv2.perspectiveTransform(points[None],matrix)[0]
                    values['calibrated_region_displacement_pixels']=float(np.linalg.norm(projected-points,axis=1).max())
        for name,limit in limits.items():
            value=values.get(name)
            if value is not None:
                if not math.isfinite(value):problem='Non-finite registration diagnostics';values[name]=str(value)
                elif abs(value)>limit:warnings.append(name+' exceeds background framing diagnostic limit')
        if tuple(frame.shape[1::-1])!=getattr(self.localizer,'frame_size',None):problem='Frame dimensions differ from calibration'
        if self.localizer.error:problem=self.localizer.error
        if result.detection_paused:problem='Detection paused; no current target observations'
        if result.geometry.status not in {'MATCHED','MOVED','UNVERIFIED','DISABLED'} or registration.status=='INVALID':
            problem='Invalid detection/registration status'
        if result.geometry.status!='MATCHED':warnings.append(result.geometry.reason)
        if registration.status!='MATCHED':warnings.append(registration.reason)
        report={'detection':values,'limits':limits,'small_alignment_limit_pixels':config.alignment_max_displacement_pixels,
                'frame_size':list(frame.shape[1::-1]),'background_size':list(self.detector.shape[::-1]),
                'calibration_size':getattr(self.localizer,'frame_size',None),'calibration_id':self.localizer.calibration_id,
                'background_reference_id':self.background_id,'registration':registration.to_dict(),
                'hazard':problem,'warnings':warnings}
        text=json.dumps(report,allow_nan=False,default=str)
        if self.logger:self.logger.emit('GEOMETRY',text)
        if problem:raise NavigationFailure('Detection geometry unsafe: '+problem+'; '+text)
        if warnings:
            message='Background/detection quality warning; current-frame coordinates remain estimates; fresh VERIFIED robot poses and collision/STOP checks remain required. '+text
            if self.logger:self.logger.emit('WARN',message)
            else:logging.getLogger(__name__).warning(message)
        return report

    def scan(self,verify_target=None):
        records=[]
        # The same detector/tracker thresholds as test-vision, but allow a short
        # bounded observation window after a transient miss/category/shape reset.
        for index in range(self.frames*4):
            frame,mask,*hard=self.frame_source()
            if verify_target is not None and verify_target['surface']=='FLOOR' and mask is not None and hard:
                low,high=spill_bounds(verify_target);floor=getattr(self.localizer.calibration,'floor',self.localizer.calibration)
                # Arrival-only ROI: remove broad padding, never the actual robot/tool mask.
                corners=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
                pixels=np.rint([floor.world_to_pixel(*p) for p in corners]).astype(np.int32)
                roi=np.zeros(frame.shape[:2],np.uint8);cv2.fillConvexPoly(roi,pixels,255)
                padding=self.detector.config.roi_margin_pixels+self.detector.config.exclusion_padding_pixels
                roi=cv2.dilate(roi,np.ones((2*padding+1,2*padding+1),np.uint8))
                mask=mask.copy();mask[roi!=0]=0;mask[hard[0]!=0]=255
            result=self.detector.detect(frame,exclusion_mask=mask)
            registration=self.registration
            self._check_geometry(result,frame,registration)
            tracked=self.tracker.update(result.detections)
            records=self.localizer.localize_detections(result,tracked,frame_size=frame.shape[1::-1],registration=registration)
            if index+1>=self.frames and records and all(d.confirmed for d in tracked):break
        self.last_scan={'frames':index+1,'candidates':len(tracked),'confirmed':sum(d.confirmed for d in tracked),
                        'localizations':[{'id':r.target_id,'surface':r.surface,'reason':r.reason} for r in records]}
        if self.logger:self.logger.emit('SCAN',json.dumps(self.last_scan,allow_nan=False))
        return [r.target_message() for r in records if r.surface in {'FLOOR','WALL'} and
                r.localization_status!='INVALID' and r.world_coordinates_cm is not None]


def navigation_destination(target,home,arena,calibration,margin):
    """Floor bumper approach or separate parallel wall observation; no cleaning claim."""
    if target['surface']=='FLOOR':
        goals=front_parking_candidates(target,arena)
        if not goals:raise NavigationFailure('No safe 5 cm front-clearance parking position')
        return min(goals,key=lambda p:math.hypot(p['x_cm']-home['x_cm'],p['y_cm']-home['y_cm']))
    elif target['surface']=='WALL':
        x,y=calibration.wall_to_floor_base(target['position_cm']['x'],0)
        a=calibration.wall_to_floor_base(0,0);b=calibration.wall_to_floor_base(calibration.wall.width_cm,0)
        tangent=math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))%360
        heading=min((tangent,(tangent+180)%360),key=lambda h:abs((h-home['heading_deg']+180)%360-180))
    else:raise NavigationFailure('No floor navigation destination for this surface')
    polygons=arena.corners({'x_cm':0.,'y_cm':0.,'heading_deg':heading})
    radius=max(math.hypot(*point) for polygon in polygons for point in polygon)+margin
    if 2*radius>=min(arena.width,arena.depth):raise NavigationFailure('Chassis/tool rotation clearance does not fit arena')
    goal={'x_cm':min(max(x,radius),arena.width-radius),
          'y_cm':min(max(y,radius),arena.depth-radius),'heading_deg':heading}
    if not arena.safe(goal):raise NavigationFailure('Detected destination chassis/tool clearance unsafe')
    return goal


def run_queue(scanner,planner,adapter,poses,transport,timing,arena,config,clock,sleep,emit,*,max_tasks=10,after_task=None,navigation_only=False,calibration=None,return_home=True):
    """Bounded live queue; re-detect between tasks and before ARRIVED, never retry faults."""
    completed=[];skipped=[];retired=[];home=None;limit_reached=False
    def same(a,b):
        if a['surface']!=b['surface'] or a['category']!=b['category']:return False
        if sum((a['position_cm'][k]-b['position_cm'][k])**2 for k in a['position_cm'])>2.5**2:return False
        if a.get('geometry_cm',{}).get('bounds') and b.get('geometry_cm',{}).get('bounds'):
            low,high=spill_bounds(a);L,H=spill_bounds(b);intersection=float(np.maximum(0,np.minimum(high,H)-np.maximum(low,L)).prod())
            return intersection>=.5*min(float((high-low).prod()),float((H-L).prod()))
        return True
    def already_done(record):
        # A color fluctuation after spraying must not re-run the same preset.
        return any(same({**record,'category':t['category']},t) for t in retired)
    for _ in range(max_tasks):
        candidates=sorted(scanner.scan(),key=lambda r:(r['category']!='RED_SPILL',r['target_id']))
        nav=Navigator(poses,transport,timing,arena,config,clock=clock,sleep=sleep,on_event=emit)
        if candidates:
            current=nav.observe().geometry()
            if home is None:home=current
        selected=None
        for target in candidates:
            if already_done(target):continue
            attempts=[]
            emit({'event_type':'AUTO_TARGET_EVALUATION','target_id':target['target_id'],
                  'robot_pose':current,'spill_position_cm':target['position_cm'],
                  'surface':target['surface'],'category':target['category'],
                  'spill_bounds_cm':target.get('geometry_cm',{}).get('bounds')})
            try:
                preset=(timing.data['mode']=='PHYSICAL' and target['surface']=='FLOOR' and not navigation_only)
                raw=None if navigation_only or preset else planner.plan(target)
                if target['surface']=='FLOOR':
                    options=sorted(front_parking_candidates(target,arena,include_unsafe=True,
                        from_pose=current if not return_home and not navigation_only else None),
                        key=lambda p:math.hypot(p['x_cm']-current['x_cm'],p['y_cm']-current['y_cm']))
                else:options=[navigation_destination(target,home,arena,calibration,config.clearance_margin_cm) if navigation_only else None]
                last_error='No safe 5 cm front-clearance parking position'
                viable=[]
                for option in options:
                    attempt={'event_type':'AUTO_PARKING_CANDIDATE','target_id':target['target_id'],
                             'parking_pose':option,'stage':'PARKING_BOUNDARY'}
                    try:
                        if option is not None:
                            if target['surface']=='FLOOR':attempt.update(front_bumper_metrics(target,option,arena.length,arena.body_width))
                            if not arena.safe(option):arena.check_segment(option,option)
                        attempt['stage']='CLEANING_PRESET_SWEEP' if preset else 'TOOL_PLAN'
                        if navigation_only:plan=option
                        elif preset:
                            plan={'surface':'FLOOR','reachability':'FULLY_REACHABLE','reachability_scope':'NAVIGATION_PARKING_ONLY',
                                  'approach_pose':option,'target_geometry':target['geometry_cm']['contours'],
                                  'front_clearance_cm':5.,'execution_scope':'ESP32 preset; tool coverage/effectiveness unverified'}
                            adapter.check(plan,Pose(**option,observed_at=0,frame_id=0),verify_tool_coverage=False)
                        elif target['surface']=='FLOOR':plan=adapter.prepare(raw,front_clearance_cm=5.,heading_deg=option['heading_deg'])
                        else:plan=adapter.prepare(raw)
                        goal=plan if navigation_only else plan['approach_pose']
                        if not return_home and not navigation_only:
                            attempt['stage']='OUTBOUND_SIMPLE'
                            route=nav.plan_simple_route(current,goal);cost=route['duration_ms']
                            attempt['route']=route
                        else:
                            attempt['stage']='OUTBOUND_Y_X';nav._preflight_route(current,goal,('Y','X'))
                            attempt['stage']='RETURN_X_Y';nav._preflight_route(goal,home,('X','Y'))
                            cost=math.hypot(goal['x_cm']-current['x_cm'],goal['y_cm']-current['y_cm'])
                        attempt.update(stage='ACCEPTED',accepted=True);attempts.append(attempt);emit(attempt)
                        viable.append((cost,plan))
                    except (ValueError,NavigationFailure) as exc:
                        last_error=str(exc);attempt.update(accepted=False,reason=last_error)
                        attempts.append(attempt);emit(attempt)
                if not viable:raise NavigationFailure(last_error)
                plan=min(viable,key=lambda p:p[0])[1]
            except (ValueError,NavigationFailure) as exc:
                skipped=[s for s in skipped if not same(s['target'],target)]
                skipped.append({'target':target,'reason':str(exc),'robot_pose':current,'parking_attempts':attempts});continue
            selected=(target,plan);break
        if selected is None:break
        target,plan=selected
        emit({'event_type':'AUTO_TASK_SELECTED','category':target['category'],'priority':'HIGH' if target['category']=='RED_SPILL' else 'LOW','target_id':target['target_id']})
        def verify(p,pose):
            fresh=next((r for r in scanner.scan(verify_target=target) if same(r,target)),None)
            if fresh is None:raise NavigationFailure('Spill changed/disappeared before arrival; stop task')
            new=None
            if not navigation_only:
                new=({**p,'target_geometry':fresh['geometry_cm']['contours']} if preset else
                     adapter.prepare(planner.plan(fresh),front_clearance_cm=5.,heading_deg=p['approach_pose']['heading_deg'])
                     if fresh['surface']=='FLOOR' else adapter.prepare(planner.plan(fresh)))
            pose=nav.observe()  # Take the robot pose AFTER the spill refresh and replanning.
            if fresh['surface']=='FLOOR':
                metrics=front_bumper_metrics(fresh,pose.geometry(),arena.length,arena.body_width)
                goal=p if navigation_only else p['approach_pose']
                if (not return_home and not navigation_only and metrics['facing_error_deg']>config.heading_tolerance_deg and
                    math.hypot(pose.x_cm-goal['x_cm'],pose.y_cm-goal['y_cm'])<=config.axis_tolerance_cm and
                    abs((pose.heading_deg-goal['heading_deg']+180)%360-180)<=config.heading_tolerance_deg):
                    # Position tolerance changes the observed bearing. Face the
                    # actual spill rather than requiring extra X/Y micro-pulses.
                    low,high=spill_bounds(fresh);center=(low+high)/2
                    bearing=math.degrees(math.atan2(center[1]-pose.y_cm,center[0]-pose.x_cm))%360
                    nav.align(bearing,'ALIGNING_SPILL_HEADING',config.heading_tolerance_deg/2)
                    fresh=next((r for r in scanner.scan(verify_target=target) if same(r,target)),None)
                    if fresh is None:raise NavigationFailure('Spill changed/disappeared after facing correction')
                    pose=nav.observe();new['target_geometry']=fresh['geometry_cm']['contours']
                    metrics=front_bumper_metrics(fresh,pose.geometry(),arena.length,arena.body_width)
                if abs(metrics['clearance_cm']-5.)>config.axis_tolerance_cm or metrics['facing_error_deg']>config.heading_tolerance_deg:
                    raise NavigationFailure('Front bumper clearance/facing verification failed: '+str(metrics))
                if not navigation_only:
                    adapter.check(new,pose,clearance_tolerance_cm=config.axis_tolerance_cm,heading_tolerance_deg=config.heading_tolerance_deg,
                                  verify_tool_coverage=not preset)
                low,high=spill_bounds(fresh)
                emit({'event_type':'BUMPER_CLEARANCE_VERIFIED','target_id':fresh['target_id'],**metrics,
                      'observed_pose':pose.geometry(),'pose_frame_id':pose.frame_id,'pose_observed_at':pose.observed_at,
                      'pose_verified':pose.verified,'spill_bounds_cm':[low.tolist(),high.tolist()]})
            elif not navigation_only:
                if new['target_geometry']!=p['target_geometry']:raise NavigationFailure('Spill geometry changed; no stale cleaning')
                adapter.check(new,pose)
        if navigation_only:
            emit({'event_type':'AUTO_NAVIGATION_DESTINATION','target_id':target['target_id'],'destination':plan})
            result=nav.run_navigation_only(plan,task_key=f"navigation-test:auto:{target['target_id']}",home_pose=home,verify_arrival=lambda pose:verify(plan,pose))
        else:
            nav.check_approach=verify
            result=nav.run(plan,target['category'],task_key=f"auto:{target['target_id']}",home_pose=home,
                           return_home=return_home,simple_navigation=not return_home)
        if result['state']!='READY':raise NavigationFailure(result['reason'],'ERROR')
        home=nav.home
        skipped=[s for s in skipped if not same(s['target'],target)]
        completed.append({'target':target,'result':result});retired.append(target)
        if after_task and not navigation_only:after_task(target)
    else:
        if not return_home and not navigation_only:
            candidates=scanner.scan();limit_reached=any(not already_done(r) for r in candidates)
    skipped=[s for s in skipped if any(same(s['target'],r) for r in candidates)]
    return {'state':'BATCH_LIMIT' if limit_reached else 'PARTIAL' if completed and skipped else 'READY' if completed else 'NO_REACHABLE_TARGET' if skipped else 'NO_CONFIRMED_SPILL',
            'completed':completed,'skipped':skipped,'batch_limit':max_tasks,'navigation_only':navigation_only,
            'return_home':return_home,'batch_limit_reached':limit_reached,'last_scan':getattr(scanner,'last_scan',None),
            'scope':'Navigation arrival and HOME only; no cleaning' if navigation_only else
                    'DONE,CLEANING'+(' and verified HOME' if return_home else '; continue from fresh measured pose')+'; not visual cleanliness certification'}


def run_auto(args):
    if not 0<args.connect_timeout<=10:raise ValueError('connect-timeout must be > 0 and <= 10 seconds')
    if args.max_tasks is None:args.max_tasks=1 if args.navigation_only else 100
    if not 1<=args.max_tasks<=100:raise ValueError('max-tasks must be 1..100')
    physical=args.mode=='physical'
    if args.confirm_navigation_test:
        if not args.navigation_only:raise ValueError('Navigation-test confirmation cannot authorize cleaning')
        args.authorize_motion=args.confirm_clean=args.confirm_fixed_camera=args.confirm_ground_reference=args.confirm_static_tools=True
    if args.navigation_only and physical and not args.background:args.background='data/iphone_reference.png'
    if physical and not all((args.host,args.background,args.confirm_clean,args.authorize_motion,
        args.confirm_fixed_camera,args.confirm_ground_reference,args.confirm_static_tools)):
        raise ValueError('Physical auto-clean requires host/background, clean reference, motion authorization and fixed camera/ground reference/static tool confirmations')
    out=Path(args.output_dir or f'data/auto_navigation_{time.strftime("%Y%m%d_%H%M%S")}_{uuid4().hex[:6]}');out.mkdir(parents=True,exist_ok=False)
    config=NavigationConfig(**json.loads(Path(args.navigation_settings).read_text()))
    with ExitStack() as stack:
        logger=WireLogger(out/'wire.log');stack.callback(logger.close)
        if args.confirm_navigation_test:logger.emit('AUTH','Operator explicitly authorized navigation only and attested current reference/camera/ground/tool checks; cleaning-actuator power isolated')
        after_task=None
        if not physical:
            from .cli import make_demo
            clock,poses,transport,timing,arena,adapter,_=make_demo(config=config)
            calibration=synthetic_calibration();hardware=CleaningHardware(demo_hardware());identity='SIMULATED_MODULE06_ARENA'
            planner=CleaningPlanner(calibration,hardware,mode='SIMULATION',calibration_id=identity);sleep=clock.sleep
            clean=np.full((720,1280,3),150,np.uint8)
            for y in range(50,700,90):cv2.putText(clean,'LEGAL BACKGROUND',(40,y),0,.6,(40,40,40),1)
            scene=clean.copy();patches={}
            for category,x,y,color in [('NON_RED_SPILL',90,40,(20,210,210)),('RED_SPILL',78,36,(20,20,220))]:
                pixel=tuple(round(v) for v in calibration.floor.world_to_pixel(x,y));patches[category]=pixel
                cv2.circle(scene,pixel,7,color,-1)
            reference=BackgroundReference(clean,confirmed_clean=True)
            def frames():return scene.copy(),None
            def after_task(target):
                px,py=patches[target['category']];scene[py-10:py+11,px-10:px+11]=clean[py-10:py+11,px-10:px+11]
            transport.logger=logger
        else:
            calibration=load_calibration(args.calibration);robot=load_robot_config(args.robot_settings)
            if getattr(calibration,'floor_boundary',None)!='top':raise ValueError('Native dual-plane top-wall frame required')
            floor=calibration.floor
            if (args.width,args.height)!=(floor.frame_width,floor.frame_height):raise ValueError('Resolution mismatch')
            hardware=CleaningHardware(json.loads(Path(args.hardware).read_text()));timing=load_timing(args.timing)
            if hardware.mode!='PHYSICAL' or timing.data['mode']!='PHYSICAL':raise ValueError('Physical profiles required')
            if (robot.body_length_cm,robot.body_width_cm)!=(hardware.data['body']['chassis_length_cm'],hardware.data['body']['chassis_width_cm']):raise ValueError('Chassis mismatch')
            reference=BackgroundReference.load(args.background,confirmed_clean=True)
            identity='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest()
            if reference.resolution!=(floor.frame_width,floor.frame_height):raise ValueError('Background/calibration resolution mismatch')
            if args.camera!=calibration.source:raise ValueError('Camera source differs from saved calibration')
            declared=reference.metadata.get('calibration_id')
            if declared is not None and declared!=identity:raise ValueError('Background metadata declares a different calibration')
            if isinstance(reference.metadata.get('source'),int) and reference.metadata['source']!=args.camera:
                raise ValueError('Background camera source mismatch')
            if reference.metadata.get('calibration_may_be_invalid'):
                logger.emit('WARN','Background contains historical framing warning; fixed-camera association is operator-confirmed, not established by background registration')
            for d in 'FBLR':timing.rate(d)
            if not args.navigation_only:
                logger.emit('INFO','MVP GROUND: PC verifies parking and preset chassis/tool collision sweep; ESP32 owns cleaning; contact/spray coverage is not certified')
            planner=CleaningPlanner(calibration,hardware,mode='PHYSICAL',calibration_id=identity)
            body=hardware.data['body'];measured_margin=(body['safety_margin_cm'],body['position_uncertainty_cm'])
            margin=max(config.clearance_margin_cm,sum(measured_margin)) if all(v is not None for v in measured_margin) else config.clearance_margin_cm
            sponge=hardware.data['floor_sponge'];tool=sponge if all(sponge[k] is not None for k in ('contact_length_cm','contact_width_cm','offset_forward_cm','offset_lateral_cm')) else None
            arena=Arena(floor.width_cm,floor.height_cm,robot.body_length_cm,robot.body_width_cm,margin,tool=tool,side_tool=hardware.side_collision_envelope())
            if hardware.data['wall_sponge']['installed'] is True and arena.side_tool is None:
                raise NavigationFailure('CONFIGURATION_REQUIRED: installed side-tool collision geometry missing')
            adapter=ToolApproach(hardware,calibration,arena,timing)
            poses=CameraPoses(None,args.calibration,args.robot_settings,args.marker_plane,verification_points_path=args.verification_points,
                fixed_camera=True,ground_reference_confirmed=True,log=lambda message:logger.emit('LOCALIZATION',message))
            logger.emit('ASSOCIATION',json.dumps(dict(calibration_id=identity,
                mapping_calibration_id=json.loads(Path(args.marker_plane).read_text())['calibration_id'],
                background_reference_id=image_identity(reference.image),background_metadata=reference.metadata,
                frame_size=[floor.frame_width,floor.frame_height],floor_cm=[floor.width_cm,floor.height_cm],
                current_camera_association='OPERATOR_CONFIRMED_FIXED_CAMERA; Module04 context validated',
                coordinate_policy='CURRENT_FRAME_MODULE01; background fit is image-only'),allow_nan=False))
            camera=stack.enter_context(CameraManager(args.camera,args.width,args.height,30));poses.camera=camera
            poses.start(config.observation_gap_seconds,startup_wait_seconds=2.);stack.callback(poses.close)
            def frames():
                poses.observe();frame,pose=poses.consumed_observation
                mask=poses.tracker.exclusion_mask(pose,frame.shape[1::-1])
                hard=np.zeros(frame.shape[:2],np.uint8)
                for polygon in arena.corners({'x_cm':pose.position_cm['x'],'y_cm':pose.position_cm['y'],'heading_deg':pose.heading_deg}):
                    pixels=np.rint([floor.world_to_pixel(*p) for p in polygon]).astype(np.int32)
                    cv2.fillConvexPoly(hard,pixels,255)
                if pose.marker_corners_pixels is not None:cv2.fillConvexPoly(hard,np.rint(pose.marker_corners_pixels).astype(np.int32),255)
                return frame,mask,hard
            poses.observe();clock=time.monotonic;sleep=time.sleep
            logger.emit('INFO',f'PHYSICAL live camera {args.camera}; verified Module04 feedback; TCP {args.host}:{args.port}; MOVE <= {config.maximum_segment_ms} ms; DONE,MOVE required')
            transport=stack.enter_context(RobotClient(args.host,args.port,mode='physical',firmware_profile='simple',logger=logger,connect_timeout=args.connect_timeout))
        scanner=SceneScanner(reference,calibration,identity,args.settings,frames,physical=physical,navigation_only=args.navigation_only,logger=logger,
            localization_settings=json.loads(Path(args.localization_settings).read_text()))
        stream=stack.enter_context((out/'progress.jsonl').open('x'))
        def emit(event):
            stream.write(json.dumps(event,allow_nan=False)+'\n');stream.flush()
            if physical and event['event_type']=='BUMPER_CLEARANCE_VERIFIED':logger.emit('ARRIVAL',json.dumps(event,allow_nan=False))
            if physical and event['event_type'] in {'AUTO_TARGET_EVALUATION','AUTO_PARKING_CANDIDATE'}:
                logger.emit('PLAN',json.dumps(event,allow_nan=False))
            if physical and event.get('event_type')=='NAVIGATION_PROGRESS' and ('segment_outcome' in event or 'heading_progress_deg' in event):
                logger.emit('MOTION',json.dumps(event,allow_nan=False))
        try:
            result=run_queue(scanner,planner,adapter,poses,transport,timing,arena,config,clock,sleep,emit,
                max_tasks=args.max_tasks,after_task=after_task,navigation_only=args.navigation_only,calibration=calibration,
                return_home=args.return_home or args.navigation_only)
        except BaseException as exc:
            failure={'state':'ERROR','reason':str(exc)}
            try:transport.send_stop()
            except Exception as stop_error:
                failure['stop_error']=str(stop_error)
                logger.emit('ERROR','Shutdown STOP unavailable; original task fault retained: '+str(stop_error))
            (out/'summary.json').write_text(json.dumps(failure)+'\n');raise
        (out/'summary.json').write_text(json.dumps({**result,'mode':args.mode},indent=2)+'\n')
        print(json.dumps({'state':result['state'],'completed':len(result['completed']),'skipped':len(result['skipped']),'evidence':str(out)}))
