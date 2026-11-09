"""Hospital-facing state and read-only Module 02/03/05 integration; no hardware."""
from copy import deepcopy
from datetime import datetime,timezone,timedelta
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from dashboard.app import create_app
from dashboard.model import TaskStore,PRIORITY_RULES,iso
from dashboard.service import DashboardService
from dashboard.live import LiveMonitor
from vision.calibration import GroundCalibration
from vision.spill_detection import BackgroundReference


class FakeLive:
    instances=[]
    def __init__(self,service,**kwargs):self.service=service;self.started=False;self.stopped=False;self.instances.append(self)
    def start(self):self.started=True
    def stop(self):self.stopped=True


@pytest.fixture
def service():
    value=DashboardService(live_factory=FakeLive)
    yield value
    value.close()


@pytest.fixture
def client(service):return create_app(service).test_client()


def event(id=1,category='NON_RED_SPILL',surface='FLOOR',time='2026-11-08T10:00:00Z',key=None,active=True):
    position={'x':20,'y' if surface=='FLOOR' else 'h':10} if surface in {'FLOOR','WALL'} else None
    return {'record_key':key or f'live:0:{id}','active':active,'first_seen':time,
            'target_message':{'target_id':id,'category':category,'surface':surface,'position_cm':position,
              'timestamp':time,'localization_status':'UNVERIFIED','active':active,'geometry_cm':None}}


def step(client,n=1):
    for _ in range(n):
        response=client.post('/api/demo/step',json={})
        assert response.status_code==200,response.get_json()
    return response.get_json()


def test_startup_demo_without_camera_or_esp32(client,monkeypatch):
    import dashboard.live as live
    monkeypatch.setattr(live,'CameraManager',Mock(side_effect=AssertionError('No camera in DEMO')))
    count=len(FakeLive.instances)
    response=client.get('/');assert response.status_code==200

    assert b'HOSPITAL CLEANING MANAGEMENT SYSTEM' in response.data
    state=client.get('/api/state').get_json()
    assert state['mode']=='DEMO' and state['monitoring']['status']=='SIMULATED'
    assert state['metrics']=={'total':0,'open':0,'high':0,'low':0,'completed':0}
    assert state['non_actuating'] and len(FakeLive.instances)==count
    assert client.get('/health').get_json()['status']=='ok'
    assert client.get('/api/frame.jpg').status_code==503


@pytest.mark.parametrize('category,priority,label',[('RED_SPILL','HIGH','Red Spill'),('NON_RED_SPILL','LOW','Blue / Non-Red Spill')])
def test_priority_is_centralized_and_backend_category_preserved(category,priority,label):
    store=TaskStore();store.ingest(event(category=category))
    task=store.snapshot()['tasks'][0]
    assert task['category']==category and task['priority']==priority and task['label']==label
    assert 'BLUE_SPILL' not in PRIORITY_RULES


def test_new_high_preempts_pending_order_not_active_task():
    store=TaskStore('LIVE');store.ingest(event())
    store.transition('live:0:1','IN_PROGRESS')
    store.ingest(event(2,'RED_SPILL',time='2026-11-08T10:01:00Z'))
    state=store.snapshot()
    assert state['active_task']['target_id']==1
    assert [t['target_id'] for t in state['pending_queue']]==[2]
    with pytest.raises(ValueError,match='already in progress'):store.transition('live:0:2','IN_PROGRESS')


def test_high_before_low_and_fifo_in_real_time_across_timezone_offsets():
    store=TaskStore()
    store.ingest(event(1,time='2026-11-08T10:00:00Z'))
    store.ingest(event(2,'RED_SPILL',time='2026-11-08T09:59:00Z'))
    store.ingest(event(3,'RED_SPILL',time='2026-11-08T12:00:00+02:00'))
    store.ingest(event(4,time='2026-11-08T10:02:00Z'))
    assert [t['target_id'] for t in store.snapshot()['pending_queue']]==[2,3,1,4]


def test_stable_target_updates_no_duplicate_or_duplicate_events():
    store=TaskStore('LIVE');first=event()
    store.ingest(first);count=len(store.events)
    update=deepcopy(first);update['target_message']['position_cm']['x']=22
    for _ in range(10):store.ingest(update)
    assert len(store.tasks)==1 and len(store.events)==count
    assert store.snapshot()['tasks'][0]['location']['x']==22
    other=event(key='new-session:0:1');store.ingest(other)
    assert len(store.tasks)==2  # Producer scopes prevent reset ID collisions.


@pytest.mark.parametrize('surface,keys',[('FLOOR',{'x','y'}),('WALL',{'x','h'}),('BOUNDARY',None),('UNKNOWN',None)])
def test_surface_and_missing_coordinates(surface,keys):
    store=TaskStore();store.ingest(event(surface=surface))
    task=store.snapshot()['tasks'][0]
    assert task['surface']==surface
    assert (set(task['location']) if task['location'] else None)==keys
    assert task['reachability']=='UNKNOWN' and task['geometry'] is None


def test_invalid_nonfinite_or_wrong_plane_coordinates_are_not_invented():
    store=TaskStore()
    for i,position in enumerate([{'x':1,'h':2},{'x':float('nan'),'y':2},None],1):
        e=event(i);e['target_message']['position_cm']=position;store.ingest(e)
    assert all(t['location'] is None for t in store.snapshot()['tasks'])


def test_inactivity_does_not_complete_task_and_last_known_is_marked():
    store=TaskStore('LIVE');store.ingest(event());store.transition('live:0:1','IN_PROGRESS')
    missing=event(active=False);missing['target_message']['position_cm']=None
    store.ingest(missing)
    task=store.snapshot()['active_task']
    assert task['task_status']=='IN_PROGRESS' and task['completion_source'] is None
    assert not task['location_current'] and task['location']=={'x':20,'y':10}
    count=len(store.events);store.ingest(missing);assert len(store.events)==count


@pytest.mark.parametrize('mode,source',[('DEMO','SIMULATED'),('LIVE','MANUAL')])
def test_transitions_history_and_completion_sources(mode,source):
    store=TaskStore(mode);store.ingest(event())
    with pytest.raises(ValueError):store.transition('live:0:1','COMPLETED')
    store.transition('live:0:1','IN_PROGRESS');store.transition('live:0:1','COMPLETED')
    task=store.snapshot()['tasks'][0]
    assert task['completion_source']==source and task['completed_at']
    assert store.snapshot()['metrics']['completed']==1
    assert store.snapshot()['metrics']['open']==0
    assert store.events[0]['title']=='Verification requested' and store.events[0]['source']==source
    with pytest.raises(ValueError):store.transition('live:0:1','PENDING')


@pytest.mark.parametrize('failure',['FAILED','UNREACHABLE'])
def test_failure_and_unreachable_review_can_return_to_queue(failure):
    store=TaskStore();store.ingest(event());store.transition('live:0:1',failure)
    assert not store.snapshot()['pending_queue']
    store.transition('live:0:1','PENDING');assert len(store.snapshot()['pending_queue'])==1


def test_filters_metrics_and_selected_details(client):
    step(client,2)
    s=client.get('/api/state?priority=HIGH&status=PENDING').get_json()
    assert len(s['tasks'])==1 and s['tasks'][0]['target_id']==2
    assert s['metrics']['total']==2 and len(s['pending_queue'])==2
    selected=client.get('/api/tasks/demo:0:2').get_json()
    assert selected['category']=='RED_SPILL' and selected['plan']['geometry_source']=='SIMULATION'
    assert selected['reachability']=='FULLY_REACHABLE'
    assert selected['plan']['coverage_segments']
    assert client.get('/api/tasks/nope').status_code==404
    assert client.get('/api/state?priority=MEDIUM').status_code==400


def test_demo_deterministic_sequence_queue_restart_and_complete(client):
    first=step(client,2)
    assert [t['target_id'] for t in first['pending_queue']]==[2,1]
    current=step(client);assert current['active_task']['target_id']==2
    done=step(client);assert done['metrics']['completed']==1 and done['tasks'][-1]['completion_source']=='SIMULATED'
    step(client,5)
    finished=client.get('/api/state').get_json()
    assert finished['metrics']['completed']==3 and finished['demo']['step']==9
    client.post('/api/demo/reset',json={})
    assert client.get('/api/state').get_json()['metrics']['total']==0
    again=step(client,2)
    assert [(t['target_id'],t['priority'],t['detected_at']) for t in first['pending_queue']]==[(t['target_id'],t['priority'],t['detected_at']) for t in again['pending_queue']]


def test_demo_play_pause_and_thread_cleanup(service):
    service.demo_interval=.04;service.start();service.demo_control('play')
    deadline=time.monotonic()+1
    while service.step<1 and time.monotonic()<deadline:time.sleep(.01)
    service.demo_control('pause');old=service.step;time.sleep(.25)
    assert service.step==old and old>0
    service.close();assert not service.ticker.is_alive()


def test_manual_live_status_route_cannot_assert_verified(service,client):
    service.set_mode('LIVE');service.ingest(event())
    assert client.post('/api/tasks/live:0:1/status',json={'status':'IN_PROGRESS'}).status_code==200
    assert client.post('/api/tasks/live:0:1/status',json={'status':'COMPLETED','completion_source':'VERIFIED'}).status_code==400
    assert client.post('/api/tasks/live:0:1/status',json={'status':'COMPLETED'}).status_code==200
    assert client.get('/api/tasks/live:0:1').get_json()['completion_source']=='MANUAL'
    assert client.post('/api/demo/play',json={}).status_code==400


def test_live_events_unknown_physical_reachability(service):
    service.set_mode('LIVE')
    e=event();e['target_message']['calibration_id']=service.calibration_id
    task=service.ingest(e)
    assert task['reachability']=='UNKNOWN' and task['cleaning_mode'] is None
    assert task['plan']['geometry_source']=='PHYSICAL' and task['plan']['reasons']
    assert not task['plan']['actionable'] and task['plan']['execution_authorization']=='NONE'


def test_no_duplicate_camera_across_tabs_mode_requests_and_retry(service,client):
    start=len(FakeLive.instances)
    for _ in range(5):
        client.post('/api/mode',json={'mode':'LIVE'});client.get('/api/state')
    assert len(FakeLive.instances)==start+1
    old=service.live
    assert client.post('/api/monitor/retry',json={}).status_code==200 and old.stopped
    assert len(FakeLive.instances)==start+2
    client.post('/api/mode',json={'mode':'DEMO'});assert service.live is None


def test_camera_unavailable_clears_feed_retains_task_and_manual_review(service,client):
    service.set_mode('LIVE');service.ingest(event())
    service.live_unavailable('Camera disconnected')
    s=client.get('/api/state').get_json()
    assert s['monitoring']['status']=='UNAVAILABLE' and s['metrics']['open']==1
    assert s['tasks'][0]['task_status']=='PENDING' and not s['tasks'][0]['active']
    assert client.get('/api/frame.jpg').status_code==503


def test_actual_frame_endpoint_and_confirmed_event_consumption(service,client):
    service.set_mode('LIVE')
    frame=np.zeros((720,1280,3),np.uint8)
    service.live_frame(frame,[event()])
    assert client.get('/api/frame.jpg').mimetype=='image/jpeg'
    s=client.get('/api/state').get_json()
    assert s['monitoring']['status']=='AVAILABLE' and s['tasks'][0]['target_id']==1
    service.live_frame(frame,[event()]);assert len(service.stores['LIVE'].tasks)==1


def test_monitoring_staleness_does_not_complete(service):
    service.set_mode('LIVE');service.ingest(event());service.last_frame_clock=time.monotonic()-5
    service.jpeg=b'old';service.start()
    deadline=time.monotonic()+1
    while service.jpeg and time.monotonic()<deadline:time.sleep(.01)
    assert service.jpeg is None and service.snapshot()['monitoring']['status']=='UNAVAILABLE'
    assert service.snapshot()['tasks'][0]['task_status']=='PENDING'


def test_bounded_events_and_capacity_never_drop_pending_targets():
    store=TaskStore(max_tasks=3)
    for i in range(3):store.ingest(event(i+1))
    with pytest.raises(ValueError,match='capacity'):store.ingest(event(4))
    assert len(store.tasks)==3
    for _ in range(200):store.event('Review requested')
    assert len(store.events)==160


def test_calibration_dimensions_loaded_not_hardcoded(service):
    original=json.loads(Path('config/camera_config.json').read_text())
    assert service.arena['floor']=={'width_cm':original['floor']['width_cm'],'depth_cm':original['floor']['depth_cm']}
    assert service.arena['wall']['height_cm']==original['wall']['height_cm']


def test_no_physical_control_endpoints_and_no_esp_imports(client):
    for url in ['/api/drive','/api/estop','/api/pump','/api/robot','/api/esp32','/api/execute-plan']:
        assert client.post(url,json={}).status_code==404
    for path in Path('dashboard').rglob('*.py'):
        source=path.read_text()
        assert 'robot_comm' not in source and 'RobotTracker' not in source
    page=client.get('/').get_data(as_text=True)
    for excluded in ['Robot Status','ESP32 Monitor','Battery','PWM','TX/RX']:
        assert excluded not in page


def test_offline_static_assets_and_same_origin_json_mutations(client):
    for path in ['/static/app.js','/static/styles.css','/static/favicon.svg']:
        response=client.get(path);assert response.status_code==200
    page=client.get('/').get_data(as_text=True)
    assert 'https://' not in page and 'http://' not in page
    assert client.post('/api/demo/step',data='{}').status_code==415
    assert client.post('/api/demo/step',json={},headers={'Origin':'https://other.example'}).status_code==403
    assert "script-src 'self'" in client.get('/').headers['Content-Security-Policy']


def test_cli_defaults_and_existing_commands():
    import main
    args=main.build_parser().parse_args(['dashboard'])
    assert args.mode=='demo' and args.host=='127.0.0.1' and args.port==5050
    assert args.headless
    for command in ['calibrate','track-robot','plan-cleaning','robot-client','detect-spills']:
        assert command in main.build_parser()._subparsers._group_actions[0].choices


@pytest.fixture
def live_inputs(tmp_path):
    cal=tmp_path/'cal.json';GroundCalibration([[0,0],[799,0],[799,599],[0,599]],80,60,800,600).save(cal)
    bg=tmp_path/'baseline.png';reference=BackgroundReference(np.full((600,800,3),230,np.uint8),confirmed_clean=True)
    reference._metadata.update(source=0,operator_registered=True);reference.save(bg)
    return cal,bg


@pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')
def test_live_pipeline_real_detector_tracker_localizer_with_mock_camera(live_inputs,monkeypatch):
    import dashboard.live as live
    cal,bg=live_inputs
    service=DashboardService(str(cal),live_factory=FakeLive);service.set_mode('LIVE')
    monitor=LiveMonitor(service,width=800,height=600,background=str(bg))
    scene=np.full((600,800,3),230,np.uint8);cv2.circle(scene,(400,300),18,(220,20,20),-1)
    reads={'count':0}
    def read():
        reads['count']+=1
        if reads['count']==4:monitor.stop_event.set()
        return scene.copy()
    camera=Mock();camera.__enter__=Mock(return_value=camera);camera.__exit__=Mock(return_value=False);camera.read.side_effect=read
    monkeypatch.setattr(live,'CameraManager',Mock(return_value=camera))
    before=bg.read_bytes();monitor.run()
    tasks=service.snapshot()['tasks']
    assert len(tasks)==1 and tasks[0]['category']=='NON_RED_SPILL' and tasks[0]['priority']=='LOW'
    assert tasks[0]['surface']=='FLOOR' and tasks[0]['location'] is not None
    assert tasks[0]['image_geometry']['contours']
    assert bg.read_bytes()==before
    camera.__exit__.assert_called_once()
    service.close()


@pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')
def test_live_camera_failure_is_graceful_and_never_registers_new_baseline(live_inputs,monkeypatch):
    import dashboard.live as live
    cal,bg=live_inputs
    service=DashboardService(str(cal),live_factory=FakeLive);service.set_mode('LIVE')
    camera=Mock();camera.__enter__=Mock(side_effect=RuntimeError('camera unavailable'));camera.__exit__=Mock(return_value=False)
    monkeypatch.setattr(live,'CameraManager',Mock(return_value=camera))
    before=hashlib.sha256(bg.read_bytes()).hexdigest()
    LiveMonitor(service,width=800,height=600,background=str(bg)).run()
    assert service.snapshot()['monitoring']['status']=='UNAVAILABLE'
    assert hashlib.sha256(bg.read_bytes()).hexdigest()==before
    service.close()


def test_same_record_key_cannot_silently_change_target_identity():
    store=TaskStore();store.ingest(event())
    with pytest.raises(ValueError,match='cannot change target ID'):store.ingest(event(2,key='live:0:1'))


def test_manual_demo_start_does_not_get_interrupted_by_script(service):
    service.demo_control('step');service.demo_control('step')
    service.stores['DEMO'].transition('demo:0:1','IN_PROGRESS')
    service.demo_control('step')
    assert service.snapshot()['active_task']['target_id']==1
    assert 'skipped' in service.snapshot()['demo']['description']


def test_missing_planning_configuration_is_unknown_not_demo_substitution():
    service=DashboardService(hardware='/tmp/hospital_guardian_no_such_hardware.json',live_factory=FakeLive)
    service.set_mode('LIVE');task=service.ingest(event())
    assert task['reachability']=='UNKNOWN'
    assert 'CONFIGURATION REQUIRED' in task['plan']['reasons'][0]
    assert task['plan']['geometry_source']=='PHYSICAL'
    service.close()


def test_manual_demo_action_pauses_script_and_labels_adjustment(client):
    step(client,2)
    client.post('/api/demo/play',json={})
    response=client.post('/api/tasks/demo:0:2/status',json={'status':'IN_PROGRESS'})
    assert response.status_code==200
    assert not response.get_json()['demo']['playing']
    assert 'adjusted manually' in response.get_json()['demo']['description']
