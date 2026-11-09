"""Deterministic pose-feedback plant; deliberately distinct from physical calibration."""
from copy import deepcopy
import math
from types import SimpleNamespace
from .controller import Pose
from .timing import MovementTiming
from robot_comm.simulator import SimpleFirmwareModel


class SimClock:
    def __init__(self):self.now=1.
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds


class SimPlant:
    def __init__(self,clock,home=(32.,29.,0.),*,move_gain=.85,turn_gain=.85,cross_drift=0.,turn_drift=0.):
        self.clock=clock;self.x,self.y,self.heading=home;self.frame=0
        self.move_gain,self.turn_gain=move_gain,turn_gain
        self.cross_drift,self.turn_drift=cross_drift,turn_drift
        self.valid=True;self.stale=False;self.lost=False;self.commands=[]
        self.actual_rates={'F':40.,'B':45.,'L':3000/90,'R':3000/90}

    def observe(self):
        if self.lost:raise RuntimeError('camera disconnected')
        self.clock.sleep(.002);self.frame+=1
        return Pose(self.x,self.y,self.heading,self.clock()-10 if self.stale else self.clock(),self.frame,self.valid,verified=True)

    def apply(self,direction,duration):
        amount=duration/self.actual_rates[direction]
        if direction in 'LR':
            self.heading=(self.heading+amount*self.turn_gain*(-1 if direction=='L' else 1))%360
            self.x+=self.turn_drift*duration/500
        else:
            amount*=self.move_gain*(1 if direction=='F' else -1)
            a=math.radians(self.heading)
            drift=self.cross_drift*duration/500
            self.x+=amount*math.cos(a)-drift*math.sin(a)
            self.y+=amount*math.sin(a)+drift*math.cos(a)


class SimTransport:
    def __init__(self,plant):
        self.plant=plant;self.clock=plant.clock;self.pending=None;self.lines=[]
        self.model=SimpleFirmwareModel(self.clock);self.model.connected();self.stopped=False
        self.busy=False;self.disconnected=False;self.no_done=False;self.clean_fault=False
        self.cleaning=False;self.cleaned=False
        self.move=None;self.last_tick=None;self.missing_stop_ack=False;self.faulted=False
        self.stop_latency_seconds=.04
        self.logger=None

    def log(self,kind,line):
        if self.logger:self.logger.emit(kind,line)

    def send_command(self,line):
        if self.faulted:raise RuntimeError('motion fault locked')
        if self.disconnected:raise RuntimeError('connection lost')
        self.lines.append(line)
        self.log('TX',line)
        if line.startswith('MOVE,'):
            if self.busy:
                self.log('RX','ERR,BUSY')
                return SimpleNamespace(firmware_accepted=False,response='ERR,BUSY')
            reply=self.model.command(line)
            for response in reply:self.log('RX',response)
            if reply!=['OK,MOVE']:return SimpleNamespace(firmware_accepted=False,response=reply[-1])
            self.pending=line
            self.last_tick=self.clock()
        self.move=SimpleNamespace(firmware_accepted=True,response='OK,MOVE',execution_completed=False,execution_state='accepted')
        return self.move

    def _advance(self):
        if not self.pending or self.move.execution_state!='accepted':return
        if self.model.move_deadline is None:return
        _,direction,duration=self.pending.split(',')
        now=min(self.clock(),self.model.move_deadline)
        if now>self.last_tick:self.plant.apply(direction,(now-self.last_tick)*1000)
        self.last_tick=now
        events=self.model.tick()
        for event in events:
            if not (self.no_done and event=='DONE,MOVE'):self.log('RX',event)
        if 'DONE,MOVE' in events and not self.no_done:
            self.move.execution_completed=True;self.move.execution_state='completed';self.pending=None

    def wait_for_move(self,result,check=None):
        if self.disconnected:raise RuntimeError('connection lost')
        deadline=self.model.move_deadline
        while result.execution_state=='accepted':
            self.clock.sleep(min(.02,max(0,deadline-self.clock())))
            self._advance()
            if check and result.execution_state=='accepted':check()
            if self.no_done and self.clock()>=deadline:
                result.execution_state='unknown';break
        return result

    def send_stop(self):
        self.lines.append('STOP');self.stopped=True
        self.log('TX','STOP')
        if self.pending and self.move.execution_state=='accepted':
            self.clock.sleep(self.stop_latency_seconds);self._advance()
        replies=self.model.command('STOP');self.pending=None
        if not self.missing_stop_ack and not self.disconnected:
            for reply in replies:self.log('RX',reply)
        if self.move and self.move.execution_state=='accepted':self.move.execution_state='aborted'

    def stop_and_wait(self,result=None):
        self.send_stop()
        if self.missing_stop_ack or self.disconnected:
            self.faulted=True;raise RuntimeError('STOP acknowledgment missing')
        self.lines.append('PING');self.log('TX','PING')
        assert self.model.command('PING')==['PONG'];self.log('RX','PONG')
        return result

    def begin_cleaning(self,surface,fluid):
        for line in (f'TARGET,{surface},{fluid}','ARRIVED'):
            self.lines.append(line);replies=self.model.command(line)
            self.log('TX',line)
            for reply in replies:self.log('RX',reply)
            if any(r.startswith('ERR,') for r in replies):raise RuntimeError(replies[-1])
        self.cleaning=True

    def wait_for_cleaning(self,timeout=30,check=None):
        if self.clean_fault:raise RuntimeError('EVENT,PUMP_TIMEOUT')
        deadline=self.clock()+timeout
        while self.clock()<deadline:
            if check:check()
            self.clock.sleep(.02)
            if self.model.direction:self.plant.apply(self.model.direction,20)
            events=self.model.tick()
            for event in events:self.log('RX',event)
            if any(event in {'EVENT,CLEAN_MOTOR_TIMEOUT','EVENT,PUMP_TIMEOUT'} for event in events):
                raise RuntimeError(events[-1])
            if 'DONE,CLEANING' in events:
                self.cleaned=True;self.cleaning=False;return 'DONE,CLEANING'
        raise RuntimeError('cleaning completion unknown')


def demo_timing(move_gain=1.,turn_gain=1.):
    timing=MovementTiming({'schema_version':1,'mode':'SIMULATION','samples':[],'references':[]})
    for direction,change,duration in [('F',10,400),('B',-10,450),('L',-90,3000),('R',90,3000)]:
        timing.add(direction=direction,duration_ms=duration,observed_change=change,verified=True,source='SYNTHETIC PLANT ONLY')
        speed=abs(change)/duration*1000*(turn_gain if direction in 'LR' else move_gain)
        timing.add_stopping(direction=direction,pose_latency_ms=30.,stop_latency_ms=40.,
            maximum_speed_per_second=max(.1,speed*1.1),coast_units=.3 if direction in 'LR' else .05,
            verified=True,source='SYNTHETIC PLANT STOP BOUNDS ONLY')
    return timing


def demo_hardware():
    import json
    from pathlib import Path
    data=deepcopy(json.loads(Path('config/cleaning_demo.json').read_text()))
    data['body'].update(chassis_length_cm=26.,chassis_width_cm=15.2)
    data['floor_sponge'].update(offset_forward_cm=16.,contact_length_cm=12.,contact_width_cm=16.)
    data['floor_nozzle'].update(offset_forward_cm=16.,coverage_length_cm=14.,coverage_width_cm=18.)
    data['wall_sponge'].update(lateral_protrusion_cm=5.6,contact_compression_cm=.2,contact_width_cm=16.)
    data['wall_nozzle'].update(coverage_width_cm=18.,safe_distance_max_cm=20.)
    return data
