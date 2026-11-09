"""Bounded, in-memory dashboard orchestration; zero physical control interfaces."""
import hashlib
import json
from pathlib import Path
import threading
import time
from datetime import datetime

import cv2
from vision.calibration import load_calibration,DualPlaneCalibration
from vision.cleaning_config import CleaningHardware
from vision.cleaning_planner import CleaningPlanner

from .model import TaskStore,iso
from .demo import STEPS,BASE,demo_event
from .live import LiveMonitor


class DashboardService:
    def __init__(self,calibration='config/camera_config.json',*,hardware='config/cleaning_hardware.json',
                 demo_hardware='config/cleaning_demo.json',live_options=None,live_factory=LiveMonitor,navigation_events=None):
        self.calibration_path=str(calibration);self.calibration=load_calibration(calibration)
        self.calibration_id='sha256:'+hashlib.sha256(Path(calibration).read_bytes()).hexdigest()
        floor=self.calibration.floor if isinstance(self.calibration,DualPlaneCalibration) else self.calibration
        self.arena={'floor':{'width_cm':floor.width_cm,'depth_cm':floor.height_cm},'wall':None,
                    'frame_size':[floor.frame_width,floor.frame_height]}
        if isinstance(self.calibration,DualPlaneCalibration):self.arena['wall']={'width_cm':self.calibration.wall.width_cm,'height_cm':self.calibration.wall.height_cm}
        self.planners={}
        self.plan_errors={}
        for mode,path in [('LIVE',hardware),('DEMO',demo_hardware)]:
            try:
                model=CleaningHardware(json.loads(Path(path).read_text()))
                self.planners[mode]=CleaningPlanner(self.calibration,model,mode='PHYSICAL' if mode=='LIVE' else 'SIMULATION',calibration_id=self.calibration_id)
            except (OSError,ValueError) as exc:self.plan_errors[mode]=str(exc)
        self.lock=threading.RLock();self.frame_condition=threading.Condition(self.lock)
        self.mode_lock=threading.Lock();self.mode='DEMO'
        self.stores={'DEMO':TaskStore('DEMO'),'LIVE':TaskStore('LIVE')}
        self.step=0;self.demo_message=None;self.playing=False;self.next_step=0;self.demo_interval=3.
        self.monitor={'status':'SIMULATED','message':'Illustrative monitoring scene · no live camera','frame_revision':0,'last_frame_at':None}
        self.jpeg=None;self.last_frame_clock=None;self.frame_revision=0
        self.live_factory=live_factory;self.live_options=live_options or {};self.live=None
        self.plan_clocks={}
        self.stores['DEMO'].event('Demo ready · monitoring area is clear',at=iso(BASE))
        self.stopped=threading.Event();self.ticker=None
        self.navigation_events=Path(navigation_events) if navigation_events else None
        self.navigation_offset=0;self.navigation_latest={}

    def read_navigation_progress(self):
        if self.navigation_events is None or not self.navigation_events.exists():return
        with self.navigation_events.open() as stream:
            stream.seek(self.navigation_offset)
            while True:
                offset=stream.tell();line=stream.readline()
                if not line or not line.endswith('\n'):
                    self.navigation_offset=offset;break
                self.navigation_offset=stream.tell()
                try:
                    event=json.loads(line)
                    if event.get('event_type')!='NAVIGATION_PROGRESS':continue
                    mode={'SIMULATION':'DEMO','PHYSICAL':'LIVE'}.get(event.get('mode'))
                    key=event.get('task_key')
                    if mode is None or not isinstance(key,str):continue
                    if mode=='LIVE' and event.get('calibration_id')!=self.calibration_id:continue
                    self.navigation_latest[(mode,key)]={k:event.get(k) for k in
                        ('phase','home_verified','cleaning_done_received')}
                    if len(self.navigation_latest)>200:self.navigation_latest.pop(next(iter(self.navigation_latest)))
                except (ValueError,TypeError):continue
        for (mode,key),event in self.navigation_latest.items():
            try:self.stores[mode].navigation_progress(key,event)
            except ValueError:continue

    def start(self):
        if self.ticker and self.ticker.is_alive():return
        self.stopped.clear()
        self.ticker=threading.Thread(target=self._tick,name='dashboard-demo',daemon=True);self.ticker.start()

    def _tick(self):
        while not self.stopped.wait(.2):
            with self.lock:
                if self.mode=='DEMO' and self.playing and time.monotonic()>=self.next_step:
                    self._advance();self.next_step=time.monotonic()+self.demo_interval
                if self.mode=='LIVE' and self.last_frame_clock is not None and time.monotonic()-self.last_frame_clock>3:
                    self.monitor['status']='UNAVAILABLE';self.monitor['message']='Camera feed interrupted · last observations retained for review'
                    self.jpeg=None
                    self.frame_condition.notify_all()
                    for t in self.stores['LIVE'].tasks.values():t['active']=False

    def close(self):
        self.stopped.set()
        with self.frame_condition:self.frame_condition.notify_all()
        if self.live:self.live.stop()
        if self.ticker:self.ticker.join(1)

    def set_mode(self,mode):
        if mode not in {'DEMO','LIVE'}:raise ValueError('Select DEMO or LIVE')
        with self.mode_lock:
            if mode==self.mode:return
            if self.live:self.live.stop();self.live=None
            with self.lock:
                self.playing=False;self.mode=mode;self.jpeg=None;self.last_frame_clock=None
                self.monitor={'status':'SIMULATED' if mode=='DEMO' else 'CONNECTING',
                              'message':'Illustrative monitoring scene · no live camera' if mode=='DEMO' else 'Opening live monitoring',
                              'frame_revision':self.frame_revision,'last_frame_at':None}
                self.frame_condition.notify_all()
            if mode=='LIVE':
                self.live=self.live_factory(self,**self.live_options);self.live.start()

    def retry_live(self):
        with self.mode_lock:
            if self.mode!='LIVE':raise ValueError('Select LIVE mode first')
            if self.live:self.live.stop()
            self.live=self.live_factory(self,**self.live_options);self.live.start()

    def demo_control(self,action):
        with self.lock:
            if self.mode!='DEMO':raise ValueError('Demo controls are only available in DEMO mode')
            if action=='play':self.playing=self.step<len(STEPS)-1;self.next_step=time.monotonic()
            elif action=='pause':self.playing=False
            elif action=='step':self.playing=False;self._advance()
            elif action=='reset':
                self.playing=False;self.step=0;self.demo_message=None;self.stores['DEMO']=TaskStore('DEMO')
                self.plan_clocks={k:v for k,v in self.plan_clocks.items() if k[0]!='DEMO'}
                self.stores['DEMO'].event('Demo restarted · monitoring area is clear',at=iso(BASE))
            else:raise ValueError('Unknown demo action')

    def transition(self,key,status):
        with self.lock:
            task=self.stores[self.mode].transition(key,status)
            if self.mode=='DEMO':
                self.playing=False
                self.demo_message='Scenario adjusted manually · restart to replay the scripted sequence'
            return task

    def _plan(self,event,mode):
        if mode not in self.planners:
            return {'reachability':'UNKNOWN','cleaning_mode':None,'coverage_segments':[],
                    'reasons':['CONFIGURATION REQUIRED: '+self.plan_errors.get(mode,'Planning configuration unavailable')],
                    'geometry_source':'SIMULATION' if mode=='DEMO' else 'PHYSICAL'}
        body=event.get('target_message',event)
        timestamp=datetime.fromisoformat(body['timestamp'].replace('Z','+00:00')).timestamp()
        try:return self.planners[mode].plan(event,timestamp=timestamp).to_dict()
        except (ValueError,KeyError,TypeError) as exc:return {'reachability':'UNKNOWN','cleaning_mode':None,'coverage_segments':[],'reasons':[str(exc)]}

    def ingest(self,event,mode='LIVE'):
        with self.lock:
            store=self.stores[mode];key=event.get('record_key') or event.get('target_message',event).get('record_key')
            previous=store.tasks.get(key)
            body=event.get('target_message',event)
            changed=previous is None or previous['geometry']!=body.get('geometry_cm') or previous['category']!=body.get('category') or previous['surface']!=body.get('surface')
            due=previous is None or time.monotonic()-self.plan_clocks.get((mode,key),0)>=2
            plan=self._plan(event,mode) if event.get('active',body.get('active',True)) and changed and due else None
            if plan is not None:self.plan_clocks[(mode,key)]=time.monotonic()
            task=store.ingest(event,plan)
            if len(self.plan_clocks)>400:
                self.plan_clocks={k:v for k,v in self.plan_clocks.items() if k[1] in self.stores[k[0]].tasks}
            return task

    def _advance(self):
        if self.step>=len(STEPS)-1:self.playing=False;return
        self.step+=1;self.demo_message=None
        store=self.stores['DEMO'];at=iso(BASE+self.step*4)
        if self.step in {1,2,5}:
            index={1:1,2:2,5:3}[self.step]
            if index==3 and not self.arena['wall']:
                store.event('No wall calibration available · wall example skipped',at=at);return
            event=demo_event(index,'RED_SPILL' if index==2 else 'NON_RED_SPILL','WALL' if index==3 else 'FLOOR',self.arena,self.calibration_id,self.step)
            self.ingest(event,'DEMO')
        elif self.step in {3,4,6,7,8,9}:
            index={3:2,4:2,6:1,7:1,8:3,9:3}[self.step]
            key=f'demo:0:{index}'
            if key in store.tasks:
                status='IN_PROGRESS' if self.step in {3,6,8} else 'COMPLETED'
                try:store.transition(key,status,at=at)
                except ValueError:
                    self.demo_message='Scripted step skipped after operator action · restart for a repeatable scenario'
                    store.event('Scripted transition skipped after operator action',store.tasks[key],at=at)
        if self.step==len(STEPS)-1:self.playing=False

    def live_frame(self,frame,events,paused=False,diagnostics=None):
        success,data=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,82])
        if not success:raise ValueError('Camera image encoding unavailable')
        with self.lock:
            if self.mode!='LIVE':return
            for event in events:self.ingest(event,'LIVE')
            self.jpeg=data.tobytes();self.frame_revision+=1;self.last_frame_clock=time.monotonic()
            self.monitor={'status':'ATTENTION' if paused else 'AVAILABLE','message':(' · '.join(diagnostics or []) or 'Check camera framing and registered scene') if paused else 'Live monitoring available',
                          'frame_revision':self.frame_revision,'last_frame_at':iso()}
            self.frame_condition.notify_all()

    def frame_stream(self):
        """Latest shared JPEG only; browser clients never own or open the camera."""
        revision=-1
        while True:
            with self.frame_condition:
                self.frame_condition.wait_for(lambda:self.stopped.is_set() or self.mode!='LIVE' or
                    self.monitor['status']=='UNAVAILABLE' or (self.jpeg is not None and self.frame_revision!=revision),timeout=1.)
                if self.stopped.is_set() or self.mode!='LIVE' or self.monitor['status']=='UNAVAILABLE':return
                if self.jpeg is None or self.frame_revision==revision:continue
                jpeg=self.jpeg;revision=self.frame_revision
            yield (b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode('ascii')+
                   b'\r\n\r\n'+jpeg+b'\r\n')

    def live_unavailable(self,reason):
        with self.lock:
            if self.mode!='LIVE':return
            changed=self.monitor['status']!='UNAVAILABLE'
            self.jpeg=None;self.last_frame_clock=None
            self.monitor={'status':'UNAVAILABLE','message':reason,'frame_revision':self.frame_revision,'last_frame_at':None}
            self.frame_condition.notify_all()
            for task in self.stores['LIVE'].tasks.values():task['active']=False
            if changed:self.stores['LIVE'].event('Live monitoring unavailable · check camera and legal scene reference',kind='warning')

    def snapshot(self,status=None,priority=None):
        with self.lock:
            self.read_navigation_progress()
            data=self.stores[self.mode].snapshot(status,priority)
            # All targets are supplied separately for scene/map overlays; filters affect only the task list.
            all_tasks=self.stores[self.mode].snapshot()['tasks']
            return {**data,'scene_targets':all_tasks,'mode':self.mode,'arena':self.arena,'monitoring':dict(self.monitor),
                    'demo':{'playing':self.playing,'step':self.step,'total_steps':len(STEPS)-1,'description':self.demo_message or STEPS[self.step]},
                    'non_actuating':True,'priority_note':'Demo priority is based on visual color classification; it does not identify a medical substance.'}
