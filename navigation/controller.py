"""Bounded Y→X / X→Y controller; every action closes on a fresh measured pose."""
from dataclasses import dataclass,asdict
import math
from itertools import product
from .timing import angle_error,finite


class NavigationFailure(RuntimeError):
    def __init__(self,reason,state='BLOCKED',*,recoverable=False):
        super().__init__(reason);self.state=state;self.recoverable=recoverable


class ReplanNeeded(RuntimeError):pass


@dataclass(frozen=True)
class Pose:
    x_cm:float
    y_cm:float
    heading_deg:float
    observed_at:float
    frame_id:int
    valid:bool=True
    verified:bool=False

    def geometry(self):
        return {key:getattr(self,key) for key in ('x_cm','y_cm','heading_deg')}


@dataclass
class NavigationConfig:
    axis_tolerance_cm:float=2.5
    heading_tolerance_deg:float=7.
    home_position_tolerance_cm:float=3.
    home_heading_tolerance_deg:float=10.
    wall_reference_y_cm:float=13.
    wall_y_tolerance_cm:float=2.5
    settle_seconds:float=.5
    pose_max_age_seconds:float=.5
    maximum_pulse_ms:int=500
    maximum_segment_ms:int=1000
    observation_gap_seconds:float=.1
    reacquisition_seconds:float=1.
    minimum_turn_ms:int=100
    minimum_move_ms:int=40
    correction_gain:float=.8
    maximum_attempts:int=24
    no_progress_attempts:int=3
    minimum_position_progress_cm:float=.2
    minimum_heading_progress_deg:float=.5
    maximum_cross_drift_cm:float=3.
    maximum_replans:int=2
    clearance_margin_cm:float=1.5
    cleaning_timeout_seconds:float=30.

    def __post_init__(self):
        for name,value in asdict(self).items():
            if finite(value,name)<=0:raise ValueError(name+' must be positive')
        for name in ('maximum_pulse_ms','maximum_segment_ms','minimum_turn_ms','minimum_move_ms','maximum_attempts','no_progress_attempts','maximum_replans'):
            if type(getattr(self,name)) is not int:raise ValueError(name+' must be integer')
        if not 1<=self.minimum_turn_ms<=self.maximum_pulse_ms<=10000 or not 1<=self.minimum_move_ms<=self.maximum_pulse_ms:
            raise ValueError('Invalid duration limits')
        if not self.maximum_pulse_ms<=self.maximum_segment_ms<10000:raise ValueError('Segment cap must cover short pulses and stay below firmware maximum')
        if self.observation_gap_seconds>self.pose_max_age_seconds:raise ValueError('Observation wait exceeds pose freshness limit')
        if self.reacquisition_seconds>2:raise ValueError('Stationary reacquisition must stay within 2 seconds')
        if not 0<self.correction_gain<=1 or self.heading_tolerance_deg>=45 or self.home_heading_tolerance_deg>=45:
            raise ValueError('Invalid gain/heading tolerance')


class Navigator:
    def __init__(self,poses,transport,timing,arena,config=None,*,clock,sleep,on_event=None,check_approach=None):
        self.poses,self.transport,self.timing,self.arena=poses,transport,timing,arena
        self.config=config or NavigationConfig();self.clock,self.sleep=clock,sleep
        self.on_event=on_event or (lambda event:None);self.check_approach=check_approach
        self.state='IDLE';self.home=None;self.pose=None;self.last_frame=-1;self.motion_count=0
        self.events=[];self.ready=False;self.task_key=None
        self.cleaning_done=False
        self.faulted=False;self.stop_confirmed=False;self.stop_attempted=False
        self.navigation_only=False
        self.reacquisitions=0
        self.simple_navigation=False

    def event(self,state,**details):
        self.state=state
        phase=('Ready' if state in {'COMPLETE','READY'} else 'Returning' if state.startswith(('RETURN','CORRECTING_RETURN','RESTORING_HOME','VERIFYING_HOME'))
               else 'Cleaning' if state in {'CLEANING','WAITING_FOR_CLEANING_DONE'} else 'Failed' if state in {'ERROR','BLOCKED','PAUSED'}
               else 'Aligning' if state.startswith(('ALIGN','ROTAT','CORRECT')) else 'Navigating')
        value=dict(event_type='NAVIGATION_PROGRESS',state=state,phase=phase,sequence=len(self.events)+1,
                   task_key=self.task_key,home_verified=self.ready,observed_at=self.clock(),navigation_only=self.navigation_only,**details)
        value.setdefault('cleaning_done_received',self.cleaning_done)
        if self.pose:value['pose']=self.pose.geometry()
        self.events.append(value);self.on_event(value)

    def observe(self,after=None,*,stationary=True):
        try:
            pose=(self.poses.observe_after(after) if after is not None and
                  callable(getattr(type(self.poses),'observe_after',None)) else self.poses.observe())
            # A background camera may still hold its last pre-settling frame.
            # Consume it, then wait once (bounded by the adapter) for a new frame.
            if after is not None and isinstance(pose,Pose) and pose.observed_at<=after:
                pose=self.poses.observe()
        except Exception as exc:
            if stationary and getattr(exc,'recoverable',False) and hasattr(self.poses,'reacquire'):
                try:
                    pose=self.poses.reacquire(after=after,timeout=self.config.reacquisition_seconds)
                    self.reacquisitions+=1
                except Exception as failure:
                    raise NavigationFailure('Stationary camera reacquisition failed: '+str(failure),'PAUSED') from failure
            else:raise NavigationFailure('Camera/localization unavailable: '+str(exc),'PAUSED',recoverable=getattr(exc,'recoverable',False)) from exc
        now=self.clock()
        if (not isinstance(pose,Pose) or not pose.valid or any(not math.isfinite(v) for v in
             (pose.x_cm,pose.y_cm,pose.heading_deg,pose.observed_at))):
            raise NavigationFailure('No valid current Module04 pose','PAUSED')
        if self.timing.data['mode']=='PHYSICAL' and not pose.verified:
            raise NavigationFailure('Physical navigation requires a VERIFIED current Module04 pose','PAUSED')
        if not 0<=now-pose.observed_at<=self.config.pose_max_age_seconds or (after is not None and pose.observed_at<=after):
            raise NavigationFailure('Stale or pre-movement pose','PAUSED')
        if type(pose.frame_id) is not int or pose.frame_id<=self.last_frame:
            raise NavigationFailure('Reused camera frame','PAUSED')
        if not self.arena.safe(pose.geometry()):raise NavigationFailure('Observed chassis/tool outside safe arena')
        self.pose=pose;self.last_frame=pose.frame_id
        return pose

    def _monitor(self):
        self.observe(stationary=False)

    def _halt(self,result=None):
        self.stop_attempted=True
        try:
            value=self.transport.stop_and_wait(result)
            self.stop_confirmed=True
            return value
        except Exception as exc:
            self.faulted=True
            raise NavigationFailure('STOP acknowledgment/outcome uncertain; motion locked: '+str(exc),'ERROR') from exc

    def _pulse(self,direction,amount,*,remaining=None,tolerance=None,heading=None,cross_reference=None):
        if self.faulted:raise NavigationFailure('Navigation fault locked; create a new explicitly authorized session','ERROR')
        revision=self.reacquisitions;before=self.observe()
        if self.reacquisitions!=revision:
            # Let the existing axis/heading loop recompute direction and duration
            # from this recovered measurement before transmitting any command.
            self.event(self.state,pose_reacquired=True,recompute_command=True)
            return before,before
        rate,sign=self.timing.rate(direction)
        try:stopping=self.timing.stopping(direction,self.config.observation_gap_seconds+.05)
        except ValueError:stopping=None
        if self.timing.data['mode']=='PHYSICAL' and self.config.maximum_segment_ms>1000 and stopping is None:
            raise NavigationFailure('Segments above 1000 ms require measured stopping data')
        margin=stopping['margin'] if stopping else None
        long=bool(remaining and stopping and abs(amount)>margin+2*(tolerance or 0))
        minimum=self.config.minimum_turn_ms if direction in 'LR' else self.config.minimum_move_ms
        requested=round(abs(amount)*rate*(1 if long else self.config.correction_gain))
        if requested<minimum:raise NavigationFailure('Required pulse below calibrated/effective minimum; stopping instead of oscillating')
        duration=min(self.config.maximum_segment_ms if long else self.config.maximum_pulse_ms,requested)
        end=before.geometry()
        predicted=duration/rate*sign
        if stopping:predicted=sign*(duration/1000*stopping['maximum_speed_per_second']+stopping['coast_units'])
        if direction in 'LR':
            end['heading_deg']=(end['heading_deg']+predicted)%360
            self.arena.check_segment(before.geometry(),end,turn_change=predicted)
        else:
            a=math.radians(before.heading_deg)
            end['x_cm']+=predicted*math.cos(a);end['y_cm']+=predicted*math.sin(a)
            self.arena.check_segment(before.geometry(),end)
        self.event(self.state,command=f'MOVE,{direction},{duration}',predicted_pose=end,
                   movement_mode='LONG_MONITORED' if long else 'SHORT_CORRECTION',stopping_margin=margin)
        intentional=False;previous=before
        def monitor():
            nonlocal intentional,previous
            if intentional:return
            try:current=self.observe(stationary=False)
            except NavigationFailure as exc:
                if not exc.recoverable:raise
                # DONE can arrive while the camera check itself is waiting.
                # Confirmed completion uses the stationary fresh-pose path below.
                getter=getattr(type(self.transport),'get_move_result',None)
                if callable(getter) and getter(self.transport,result).execution_state=='completed':return
                # Localization loss cannot extend MOVE. Confirm STOP first;
                # only the stationary post-segment path may reacquire a pose.
                stopped=self._halt(result);intentional=True
                self.event(self.state,early_stop=stopped.execution_state=='aborted',stop_requested=True,
                           stop_confirmed=True,stop_reason='localization loss: '+str(exc))
                return
            midpoint={**current.geometry(),'heading_deg':previous.heading_deg}
            self.arena.check_segment(previous.geometry(),midpoint)
            change=angle_error(current.heading_deg,previous.heading_deg)
            if abs(change)>1e-8:self.arena.check_segment(midpoint,current.geometry(),turn_change=change)
            previous=current
            reason=None
            if heading is not None and abs(angle_error(heading,current.heading_deg))>self.config.heading_tolerance_deg:reason='heading drift'
            cross=(cross_reference(current) if callable(cross_reference) else
                   abs(getattr(current,cross_reference[0])-cross_reference[1]) if cross_reference is not None else 0.)
            if cross>self.config.maximum_cross_drift_cm:reason='cross-axis drift'
            live_margin=margin
            if stopping:live_margin+=max(0,self.clock()-current.observed_at)*stopping['maximum_speed_per_second']
            if long and remaining(current)<=live_margin:reason='stopping margin'
            if self.simple_navigation and remaining and remaining(current)<=(tolerance or 0):reason='target tolerance'
            if reason:
                stopped=self._halt(result);intentional=True
                self.event(self.state,early_stop=stopped.execution_state=='aborted',stop_requested=True,stop_confirmed=True,
                    timer_completed_before_stop=stopped.execution_state=='completed',stop_reason=reason,remaining=remaining(current) if remaining else None)
        self.stop_confirmed=False;self.stop_attempted=False
        try:
            result=self.transport.send_command(f'MOVE,{direction},{duration}')
            if result.firmware_accepted is not True:raise NavigationFailure('MOVE rejected: '+str(result.response),'ERROR')
            result=self.transport.wait_for_move(result,check=monitor)
            if not ((result.execution_completed and result.execution_state=='completed') or
                    (intentional and self.stop_confirmed and result.execution_state=='aborted')):
                self.faulted=True
                raise NavigationFailure('MOVE '+result.execution_state+'; motion locked; no replay','ERROR')
        except Exception:
            if not self.stop_attempted:self._halt()
            raise
        done=self.clock();self.motion_count+=1
        self.sleep(self.config.settle_seconds)
        after=self.observe(after=done+self.config.settle_seconds-1e-9)
        self.event(self.state,movement_completed=result.execution_state=='completed',segment_outcome='early_stopped' if result.execution_state=='aborted' else 'completed',
                   previous_pose=before.geometry(),fresh_pose=after.geometry(),command=f'MOVE,{direction},{duration}',
                   expected_change=duration/rate*sign,change_units='deg' if direction in 'LR' else 'cm',
                   observed_heading_change_deg=angle_error(after.heading_deg,before.heading_deg))
        return before,after

    def align(self,heading,state,tolerance=None):
        tolerance=tolerance or self.config.heading_tolerance_deg
        no_progress=0
        for attempt in range(self.config.maximum_attempts+1):
            pose=self.observe();error=angle_error(heading,pose.heading_deg)
            self.event(state,heading_error_deg=error,attempt=attempt)
            if abs(error)<=tolerance:return
            if attempt==self.config.maximum_attempts:break
            if abs(abs(error)-180)<1e-6:error=self._checked_turn(pose.geometry(),heading)[1]
            direction=self.timing.turn_direction(error)
            end=pose.geometry();end['heading_deg']=heading
            self.arena.check_segment(pose.geometry(),end,turn_change=error)
            sign=self.timing.rate(direction)[1]
            before,after=self._pulse(direction,abs(error),remaining=lambda p:angle_error(heading,p.heading_deg)*sign,tolerance=tolerance)
            progress=abs(angle_error(heading,before.heading_deg))-abs(angle_error(heading,after.heading_deg))
            no_progress=no_progress+1 if progress<self.config.minimum_heading_progress_deg else 0
            self.event(state,heading_progress_deg=progress,observed_heading_change_deg=angle_error(after.heading_deg,before.heading_deg),
                       minimum_progress_deg=self.config.minimum_heading_progress_deg,no_progress_attempts=no_progress,
                       no_progress_limit=self.config.no_progress_attempts)
            if no_progress>=self.config.no_progress_attempts:
                raise NavigationFailure(f'Heading corrections made no measurable progress: {no_progress} consecutive attempts below '
                    f'{self.config.minimum_heading_progress_deg:g} deg; last measured heading {before.heading_deg:.2f} -> '
                    f'{after.heading_deg:.2f} deg, progress {progress:.3f} deg toward {heading:.2f} deg; '
                    'MOVE acknowledgment/completion does not prove chassis rotation; inspect actuator motion and live marker feedback')
        raise NavigationFailure('Maximum heading correction attempts reached')

    def axis(self,axis,target,*,returning=False):
        pose=self.observe();key=axis.lower()+'_cm';other='y_cm' if axis=='X' else 'x_cm'
        tolerance=min(self.config.axis_tolerance_cm,self.config.home_position_tolerance_cm/math.sqrt(2)) if returning else self.config.axis_tolerance_cm
        error=target-getattr(pose,key)
        if abs(error)<=tolerance:
            self.event(('RETURNING_' if returning else 'MOVING_')+axis,axis_aligned=True,error_cm=error);return
        heading=self._axis_heading(pose.geometry(),axis,target)
        self.align(heading,('RETURNING_' if returning else 'ALIGNING_')+axis+'_HEADING')
        cross_reference=getattr(self.pose,other);no_progress=0
        for attempt in range(self.config.maximum_attempts+1):
            pose=self.observe();error=target-getattr(pose,key)
            state=('CORRECTING_RETURN_' if returning else 'CORRECTING_')+axis if attempt else ('RETURNING_' if returning else 'MOVING_')+axis
            self.event(state,axis=axis,error_cm=error,attempt=attempt,cross_axis_drift_cm=getattr(pose,other)-cross_reference)
            if abs(error)<=tolerance:return
            if attempt==self.config.maximum_attempts:break
            if abs(angle_error(heading,pose.heading_deg))>self.config.heading_tolerance_deg:
                self.align(heading,('RETURNING_' if returning else 'ALIGNING_')+axis+'_HEADING');pose=self.pose
                error=target-getattr(pose,key)
                if abs(error)<=tolerance:return
            projection=math.cos(math.radians(pose.heading_deg)) if axis=='X' else math.sin(math.radians(pose.heading_deg))
            if abs(projection)<.8:raise NavigationFailure('Heading unsuitable for axis movement')
            direction='F' if error*projection>0 else 'B'
            error_sign=1 if error>0 else -1
            before,after=self._pulse(direction,abs(error/projection),remaining=lambda p:(target-getattr(p,key))*error_sign,
                tolerance=tolerance,heading=heading,cross_reference=(other,cross_reference))
            drift=getattr(after,other)-cross_reference
            self.event(state,cross_axis_drift_cm=drift)
            if abs(drift)>self.config.maximum_cross_drift_cm:raise ReplanNeeded('Cross-axis drift requires new ordered route')
            progress=abs(target-getattr(before,key))-abs(target-getattr(after,key))
            no_progress=no_progress+1 if progress<self.config.minimum_position_progress_cm else 0
            if no_progress>=self.config.no_progress_attempts:raise NavigationFailure('Axis corrections made no measurable progress')
        raise NavigationFailure('Maximum axis correction attempts reached')

    def route(self,goal,*,returning=False):
        if self.simple_navigation:return self._simple_route(goal)
        order=('X','Y') if returning else ('Y','X')
        for revision in range(self.config.maximum_replans+1):
            self.observe();self.event('RETURNING_X' if returning else 'PLANNING',axis_order=list(order),route_revision=revision)
            try:
                for axis in order:self.axis(axis,goal[axis.lower()+'_cm'],returning=returning)
                self.align(goal['heading_deg'],'RESTORING_HOME_HEADING' if returning else 'ALIGNING_FINAL_HEADING',
                           self.config.home_heading_tolerance_deg if returning else self.config.heading_tolerance_deg)
                pose=self.observe()
                tolerance=self.config.home_position_tolerance_cm if returning else self.config.axis_tolerance_cm
                if max(abs(pose.x_cm-goal['x_cm']),abs(pose.y_cm-goal['y_cm']))<=tolerance:return
                raise ReplanNeeded('Final cross-axis drift outside accepted range')
            except ReplanNeeded as exc:
                self.event('PLANNING',reason=str(exc),returning=returning)
        raise NavigationFailure('Ordered replanning budget exhausted')

    def _checked_turn(self,start,heading):
        error=angle_error(heading,start['heading_deg']);choices=[];failure=None
        for change in ([error,-error] if abs(abs(error)-180)<1e-6 else [error]):
            try:
                cost=0.
                if abs(change)>1e-8:cost=abs(change)*self.timing.rate(self.timing.turn_direction(change))[0]
                self.arena.check_segment(start,{**start,'heading_deg':heading},turn_change=change)
                choices.append((cost,change))
            except ValueError as exc:failure=exc
        if not choices:raise failure
        return min(choices)

    def plan_simple_route(self,start,goal):
        """Only direct and two one-corner F/B routes; no grid/search framework."""
        if math.hypot(goal['x_cm']-start['x_cm'],goal['y_cm']-start['y_cm'])<=self.config.axis_tolerance_cm:
            cost,change=self._checked_turn(start,goal['heading_deg'])
            return dict(waypoints=[],duration_ms=cost,distance_cm=0.,turn_degrees=abs(change))
        options=[];failures=[]
        paths=[[goal],
               [{**start,'x_cm':goal['x_cm']},goal],
               [{**start,'y_cm':goal['y_cm']},goal]]
        for path in paths:
            for drives in product(('F','B'),repeat=len(path)):
                p=dict(start);legs=[];cost=distance=turns=0.
                try:
                    for point,drive in zip(path,drives):
                        dx,dy=point['x_cm']-p['x_cm'],point['y_cm']-p['y_cm'];travel=math.hypot(dx,dy)
                        if travel<1e-8:continue
                        heading=(math.degrees(math.atan2(dy,dx))+(180 if drive=='B' else 0))%360
                        turn_cost,change=self._checked_turn(p,heading)
                        aligned={**p,'heading_deg':heading};end={**aligned,'x_cm':point['x_cm'],'y_cm':point['y_cm']}
                        self.arena.check_segment(aligned,end)
                        cost+=turn_cost+travel*self.timing.rate(drive)[0];distance+=travel;turns+=abs(change)
                        legs.append({**end,'direction':drive});p=end
                    turn_cost,change=self._checked_turn(p,goal['heading_deg']);cost+=turn_cost;turns+=abs(change)
                    options.append(dict(waypoints=legs,duration_ms=cost,distance_cm=distance,turn_degrees=turns))
                except ValueError as exc:failures.append(str(exc))
        if not options:raise ValueError('No safe simple turn-and-drive route; '+(failures[-1] if failures else 'No calibrated direction'))
        return min(options,key=lambda p:(p['duration_ms'],p['distance_cm'],p['turn_degrees']))

    def _drive_to(self,goal):
        heading=goal['heading_deg'];a=math.radians(heading);v=(math.cos(a),math.sin(a));normal=(-v[1],v[0]);no_progress=0
        error=lambda p:math.hypot(goal['x_cm']-p.x_cm,goal['y_cm']-p.y_cm)
        along=lambda p:(goal['x_cm']-p.x_cm)*v[0]+(goal['y_cm']-p.y_cm)*v[1]
        cross=lambda p:abs((goal['x_cm']-p.x_cm)*normal[0]+(goal['y_cm']-p.y_cm)*normal[1])
        for attempt in range(self.config.maximum_attempts+1):
            pose=self.observe();self.event('NAVIGATING',attempt=attempt,error_cm=error(pose))
            if error(pose)<=self.config.axis_tolerance_cm:return
            if attempt==self.config.maximum_attempts:break
            if cross(pose)>self.config.axis_tolerance_cm or abs(along(pose))<self.config.minimum_position_progress_cm:
                raise ReplanNeeded('Fresh lateral position requires a new simple route')
            if abs(angle_error(heading,pose.heading_deg))>self.config.heading_tolerance_deg:
                self.align(heading,'ALIGNING_ROUTE_HEADING');continue
            direction='F' if along(pose)>0 else 'B';sign=1 if direction=='F' else -1
            before,after=self._pulse(direction,abs(along(pose)),remaining=lambda p:along(p)*sign,
                tolerance=self.config.axis_tolerance_cm,heading=heading,cross_reference=cross)
            progress=error(before)-error(after);no_progress=no_progress+1 if progress<self.config.minimum_position_progress_cm else 0
            if no_progress>=self.config.no_progress_attempts:raise NavigationFailure('Simple route corrections made no measurable progress')
        raise NavigationFailure('Maximum simple route correction attempts reached')

    def _simple_route(self,goal):
        for revision in range(self.config.maximum_replans+1):
            pose=self.observe();route=self.plan_simple_route(pose.geometry(),goal)
            self.event('PLANNING',simple_route=route,route_revision=revision)
            try:
                for waypoint in route['waypoints']:
                    self.align(waypoint['heading_deg'],'ALIGNING_ROUTE_HEADING');self._drive_to(waypoint)
                self.align(goal['heading_deg'],'ALIGNING_FINAL_HEADING');pose=self.observe()
                if math.hypot(goal['x_cm']-pose.x_cm,goal['y_cm']-pose.y_cm)<=self.config.axis_tolerance_cm:return
                raise ReplanNeeded('Final measured position needs correction')
            except ReplanNeeded as exc:self.event('PLANNING',reason=str(exc))
        raise NavigationFailure('Simple route replanning budget exhausted')

    def _return_home(self):
        self.route(self.home,returning=True)
        self.event('VERIFYING_HOME');pose=self.observe()
        if (math.hypot(pose.x_cm-self.home['x_cm'],pose.y_cm-self.home['y_cm'])>self.config.home_position_tolerance_cm or
            abs(angle_error(self.home['heading_deg'],pose.heading_deg))>self.config.home_heading_tolerance_deg):
            raise NavigationFailure('Final HOME verification failed')
        self.ready=True;self.event('COMPLETE');self.event('READY')
        return {'state':'READY','home_verified':True,'home':self.home,'final_pose':pose.geometry(),'movements':self.motion_count}

    def _failed(self,exc):
        self.faulted=True
        if not self.stop_attempted:
            try:self._halt()
            except NavigationFailure as stop_error:exc=stop_error
        state=exc.state if isinstance(exc,NavigationFailure) else 'ERROR'
        self.ready=False;self.event(state,reason=str(exc))
        return {'state':state,'home_verified':False,'reason':str(exc),'movements':self.motion_count}

    def run_navigation_only(self,goal,*,task_key='navigation-test',home_pose=None,verify_arrival=None):
        """No cleaning capability is invoked in this supervised round trip."""
        self.task_key=task_key;self.ready=False;self.cleaning_done=False;self.navigation_only=True
        arrival=False
        try:
            if self.faulted:raise NavigationFailure('Navigation fault locked; no automatic recovery','ERROR')
            goal={k:finite(goal[k],k) for k in ('x_cm','y_cm','heading_deg')}
            if not self.arena.safe(goal):raise NavigationFailure('Test destination chassis/tool clearance is unsafe')
            current=self.observe().geometry()
            self.home=dict(home_pose) if home_pose is not None else current;self.event('PLANNING',home=self.home)
            self._preflight_route(current,goal,('Y','X'))
            self._preflight_route(goal,self.home,('X','Y'))
            self.route(goal);pose=self.observe()
            if (max(abs(pose.x_cm-goal['x_cm']),abs(pose.y_cm-goal['y_cm']))>self.config.axis_tolerance_cm or
                abs(angle_error(goal['heading_deg'],pose.heading_deg))>self.config.heading_tolerance_deg):
                raise NavigationFailure('Test destination verification failed')
            if verify_arrival:verify_arrival(pose)
            arrival=True;self.event('VERIFYING_ARRIVAL',arrival_verified=True)
            result=self._return_home()
        except Exception as exc:result=self._failed(exc)
        return {**result,'navigation_only':True,'arrival_verified':arrival}

    def _axis_heading(self,start,axis,target):
        """Choose a collision-safe F/B heading before preferring the shorter turn."""
        candidates=[];rejections=[];collision_failure=None;key=axis.lower()+'_cm';error=target-start[key]
        for heading in ((0,180) if axis=='X' else (90,270)):
            try:
                projection=(math.cos if axis=='X' else math.sin)(math.radians(heading))
                self.timing.rate('F' if error*projection>0 else 'B')
                turn=angle_error(heading,start['heading_deg'])
                if abs(turn)>self.config.heading_tolerance_deg:self.timing.turn_direction(turn)
            except ValueError as exc:
                rejections.append(f'heading={heading}: {exc}');continue
            try:
                aligned={**start,'heading_deg':heading}
                self.arena.check_segment(start,aligned,turn_change=turn)
                self.arena.check_segment(aligned,{**aligned,key:target})
                candidates.append((abs(turn),heading))
            except ValueError as exc:
                collision_failure=exc;rejections.append(f'heading={heading}: {exc}')
        if not candidates:
            message='No collision-safe/calibrated '+axis+' heading; '+'; '.join(rejections)
            if collision_failure is not None:raise ValueError(message) from collision_failure
            raise NavigationFailure(message)
        return min(candidates)[1]

    def _preflight_route(self,start,goal,order):
        """Nominal full round-trip clearance before test motion; live checks remain."""
        p=dict(start)
        def turn(heading):
            error=angle_error(heading,p['heading_deg'])
            if abs(error)>self.config.heading_tolerance_deg:self.timing.turn_direction(error)
            end={**p,'heading_deg':heading}
            self.arena.check_segment(p,end,turn_change=error);p.update(end)
        for axis in order:
            key=axis.lower()+'_cm';error=goal[key]-p[key]
            if abs(error)<=self.config.axis_tolerance_cm:continue
            turn(self._axis_heading(p,axis,goal[key]));end={**p,key:goal[key]}
            self.arena.check_segment(p,end);p.update(end)
        turn(goal['heading_deg'])

    def run(self,plan,category,*,task_key='module06:1',home_pose=None,return_home=True,simple_navigation=False):
        self.task_key=task_key;self.ready=False;self.cleaning_done=False
        self.navigation_only=False
        self.simple_navigation=simple_navigation
        try:
            if self.faulted:raise NavigationFailure('Navigation fault locked; no automatic recovery','ERROR')
            if category not in {'RED_SPILL','NON_RED_SPILL'}:raise NavigationFailure('Unsupported spill category')
            if plan.get('surface') not in {'FLOOR','WALL'} or plan.get('reachability')!='FULLY_REACHABLE' or not plan.get('approach_pose'):
                raise NavigationFailure('Module05 plan has no fully reachable tool approach')
            goal={k:finite(plan['approach_pose'][k],k) for k in ('x_cm','y_cm','heading_deg')}
            if not self.arena.safe(goal):raise NavigationFailure('Module05 approach chassis/tool is unsafe')
            if plan['surface']=='WALL':
                if abs(goal['y_cm']-self.config.wall_reference_y_cm)>self.config.wall_y_tolerance_cm:
                    raise NavigationFailure('Wall tool approach disagrees with measured Y≈13 cm reference')
                if min(abs(angle_error(h,goal['heading_deg'])) for h in (0,180))>self.config.heading_tolerance_deg:
                    raise NavigationFailure('Wall cleaning requires a parallel chassis')
            self.home=self.observe().geometry()
            if home_pose is not None:self.home=dict(home_pose)
            self.event('PLANNING',home=self.home)
            self.route(goal)
            self.event('VERIFYING_ARRIVAL')
            current=self.observe()
            if self.check_approach:self.check_approach(plan,current)
            else:raise NavigationFailure('Actual tool reachability verifier required')
            self.event('CLEANING')
            self.stop_confirmed=False;self.stop_attempted=False
            self.transport.begin_cleaning('GROUND' if plan['surface']=='FLOOR' else 'WALL',
                                          'BLOOD' if category=='RED_SPILL' else 'OTHER')
            self.event('WAITING_FOR_CLEANING_DONE')
            if self.transport.wait_for_cleaning(timeout=self.config.cleaning_timeout_seconds,check=self._monitor)!='DONE,CLEANING':
                raise NavigationFailure('Cleaning outcome unknown; no retry','ERROR')
            self.cleaning_done=True
            done=self.clock();self.sleep(self.config.settle_seconds);self.observe(after=done+self.config.settle_seconds-1e-9)
            if not return_home:
                self.event('COMPLETE',returned_home=False)
                return {'state':'READY','home_verified':False,'cleaning_done_received':True,
                        'final_pose':self.pose.geometry(),'movements':self.motion_count,'returned_home':False}
            return self._return_home()
        except Exception as exc:return self._failed(exc)
