"""Hospital-facing task state; no actuator or communication imports."""
from collections import deque
from copy import deepcopy
from datetime import datetime,timezone
import math

# The only priority/category presentation rule in the application.
PRIORITY_RULES={
    'RED_SPILL':{'priority':'HIGH','rank':0,'label':'Red Spill','color':'#ff667c'},
    'NON_RED_SPILL':{'priority':'LOW','rank':1,'label':'Blue / Non-Red Spill','color':'#55a6ff'},
}
STATUSES=('PENDING','IN_PROGRESS','COMPLETED','FAILED','UNREACHABLE')
TRANSITIONS={'PENDING':{'IN_PROGRESS','FAILED','UNREACHABLE'},
             'IN_PROGRESS':{'COMPLETED','FAILED','UNREACHABLE'},
             'FAILED':{'PENDING'},'UNREACHABLE':{'PENDING'},'COMPLETED':set()}


def iso(moment=None):
    return datetime.fromtimestamp(moment,timezone.utc).isoformat().replace('+00:00','Z') if moment is not None else datetime.now(timezone.utc).isoformat().replace('+00:00','Z')


def valid_position(position,surface):
    keys={'x','y'} if surface=='FLOOR' else {'x','h'} if surface=='WALL' else set()
    if not keys or not isinstance(position,dict) or set(position)!=keys:return None
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in position.values()):return None
    return dict(position)


class TaskStore:
    def __init__(self,mode='DEMO',max_tasks=200):
        self.mode=mode;self.max_tasks=max_tasks
        self.tasks={};self.events=deque(maxlen=160);self.sequence=0

    def event(self,title,task=None,*,kind='info',at=None,source=None):
        self.sequence+=1
        self.events.appendleft({'id':self.sequence,'title':title,'task_id':None if task is None else task['task_id'],
                               'target_id':None if task is None else task['target_id'],
                               'kind':kind,'timestamp':at or iso(),'source':source or ('SIMULATED' if self.mode=='DEMO' else 'OBSERVED')})

    def ingest(self,event,plan=None):
        message=event.get('target_message',event)
        category=message.get('category')
        if category not in PRIORITY_RULES:raise ValueError('Unsupported detection category')
        target_id=message.get('target_id')
        if isinstance(target_id,bool) or not isinstance(target_id,int) or target_id<0:raise ValueError('Stable target ID required')
        key=event.get('record_key') or message.get('record_key')
        if not isinstance(key,str) or not key:raise ValueError('Producer record_key required to preserve target identity')
        active=event.get('active',message.get('active',True))
        if not isinstance(active,bool):raise ValueError('Observation active state must be boolean')
        task=self.tasks.get(key)
        if task is not None and task['target_id']!=target_id:raise ValueError('Stable record key cannot change target ID')
        if task is None and not active:return None
        timestamp=message.get('timestamp') or event.get('timestamp') or iso()
        if task is None:
            if len(self.tasks)>=self.max_tasks:
                retired=next((k for k,t in self.tasks.items() if not t['active'] and t['task_status'] in {'COMPLETED','FAILED','UNREACHABLE'}),None)
                if retired is None:raise ValueError('Task capacity reached; review existing tasks before adding more')
                del self.tasks[retired]
            self.sequence+=1
            task={'task_id':key,'target_id':target_id,'category':category,'task_status':'PENDING',
                  'detected_at':event.get('first_seen') or message.get('first_observed_timestamp') or timestamp,
                  'queue_sequence':self.sequence,'completion_source':None,'completed_at':None,'plan':None,
                  'reachability':'UNKNOWN','cleaning_mode':None,'location':None,'geometry':None,'image_geometry':None,
                  'active':active,'surface':'UNKNOWN','localization_status':'UNAVAILABLE','last_observed_at':timestamp}
            self.tasks[key]=task
            rule=PRIORITY_RULES[category]
            self.event(f'Target #{target_id:02d} detected',task,kind='detected',at=timestamp)
            self.event(f'{rule["priority"]} priority assigned · added to queue',task,kind=rule['priority'].lower(),at=timestamp)
        previous_category=task['category'];previous_active=task['active']
        task.update(category=category,active=active,updated_at=timestamp)
        if previous_category!=category:self.event(f'Priority updated to {PRIORITY_RULES[category]["priority"]}',task,at=timestamp)
        if active:
            task.update(surface=message.get('surface','UNKNOWN'),last_observed_at=timestamp,
                        localization_status=message.get('localization_status','UNVERIFIED'))
            task['location']=valid_position(message.get('position_cm'),task['surface']) if task['localization_status'] not in {'INVALID','UNAVAILABLE'} else None
            task['geometry']=deepcopy(message.get('geometry_cm')) if task['location'] else None
            task['image_geometry']=deepcopy(event.get('localization',{}).get('image_geometry') or message.get('image_geometry'))
            if plan is not None:
                task['plan']=deepcopy(plan)
                task['reachability']=plan.get('reachability','UNKNOWN')
                task['cleaning_mode']=plan.get('cleaning_mode')
        elif previous_active:
            self.event('Target no longer visible · awaiting review',task,kind='warning',at=timestamp)
        return task

    def transition(self,key,status,*,at=None):
        if status not in STATUSES:raise ValueError('Unknown task status')
        if key not in self.tasks:raise KeyError('Task not found')
        task=self.tasks[key]
        if status not in TRANSITIONS[task['task_status']]:raise ValueError('Invalid task state transition')
        if status=='IN_PROGRESS' and any(t['task_status']=='IN_PROGRESS' for t in self.tasks.values()):
            raise ValueError('Another task is already in progress; it will not be interrupted')
        if status=='COMPLETED' and task.get('navigation_progress',{}).get('label') not in (None,'Ready'):
            raise ValueError('Navigation task completes only after measured return home')
        task['task_status']=status;task['updated_at']=at or iso()
        source='SIMULATED' if self.mode=='DEMO' else 'MANUAL'
        if status=='COMPLETED':
            task['completion_source']='SIMULATED' if self.mode=='DEMO' else 'MANUAL'
            task['completed_at']=task['updated_at']
            self.event('Simulated cleaning completed' if self.mode=='DEMO' else 'Completion recorded manually · physical verification pending',task,kind='completed',at=at,source=source)
            self.event('Verification requested',task,kind='verification',at=at,source=source)
        elif status=='IN_PROGRESS':self.event('Simulated cleaning started' if self.mode=='DEMO' else 'Task marked in progress manually',task,kind='started',at=at,source=source)
        else:self.event('Task '+status.lower().replace('_',' '),task,kind='warning',at=at,source=source)
        return task

    def navigation_progress(self,key,event):
        """High-level progress only. A cleaning DONE cannot complete a return task."""
        phase=event.get('phase')
        if phase not in {'Navigating','Aligning','Cleaning','Returning','Ready','Failed'}:
            raise ValueError('Unknown navigation progress')
        if key not in self.tasks:return
        if phase=='Ready' and not (event.get('home_verified') is True and event.get('cleaning_done_received') is True):
            raise ValueError('Ready requires measured HOME and cleaning completion')
        task=self.tasks[key]
        task['navigation_progress']={'label':phase,'mode':'SIMULATED' if self.mode=='DEMO' else 'OBSERVED'}
        if phase=='Ready' and task['task_status']=='PENDING':self.transition(key,'IN_PROGRESS')
        if phase not in {'Ready','Failed'} and task['task_status']=='PENDING':
            self.transition(key,'IN_PROGRESS')
        if phase=='Failed' and task['task_status'] in {'PENDING','IN_PROGRESS'}:self.transition(key,'FAILED')
        if phase=='Ready' and task['task_status']=='IN_PROGRESS':
            self.transition(key,'COMPLETED')
            task['completion_source']='SIMULATED' if self.mode=='DEMO' else 'RETURN_VERIFIED'
            self.event('Returned home · ready for next task',task,kind='completed')

    def snapshot(self,status=None,priority=None):
        if status and status not in STATUSES:raise ValueError('Unknown status filter')
        if priority and priority not in {'HIGH','LOW'}:raise ValueError('Unknown priority filter')
        def decorate(task):
            value=deepcopy(task);value.update(PRIORITY_RULES[task['category']]);value.pop('rank',None)
            value['location_current']=bool(task['active'])
            return value
        values=[decorate(t) for t in self.tasks.values()]
        age=lambda t:datetime.fromisoformat(t['detected_at'].replace('Z','+00:00')).timestamp()
        pending=sorted((t for t in values if t['task_status']=='PENDING'),
                       key=lambda t:(PRIORITY_RULES[t['category']]['rank'],age(t),t['queue_sequence']))
        active=next((t for t in values if t['task_status']=='IN_PROGRESS'),None)
        matches=lambda t:(not status or t['task_status']==status) and (not priority or t['priority']==priority)
        tasks=sorted(values,key=lambda t:(STATUSES.index(t['task_status']),PRIORITY_RULES[t['category']]['rank'],age(t),t['queue_sequence']))
        return {'tasks':[t for t in tasks if matches(t)],'pending_queue':pending,'active_task':active,
                'metrics':{'total':len(values),'open':sum(t['task_status'] in {'PENDING','IN_PROGRESS'} for t in values),
                           'high':sum(t['priority']=='HIGH' and t['task_status'] in {'PENDING','IN_PROGRESS'} for t in values),
                           'low':sum(t['priority']=='LOW' and t['task_status'] in {'PENDING','IN_PROGRESS'} for t in values),
                           'completed':sum(t['task_status']=='COMPLETED' for t in values)},
                'events':list(self.events),'priority_rules':deepcopy(PRIORITY_RULES)}
