"""Actual Module 03 events / Module 04 pose ingestion, offline output protection."""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

import main
from tools.generate_cleaning_demo import generate_demo
from vision import TargetEventStore
from vision.cleaning_cli import load_targets
from test_cleaning_planner import cal,hardware,planner,target,rect,plan_floor,plan_wall


@pytest.fixture
def demo(tmp_path,cal):
    path=tmp_path/'source_calibration.json';cal.save(path)
    return generate_demo(tmp_path/'demo',path)


def read(path): return [json.loads(line) for line in path.read_text().splitlines()]


def command(demo,output,mode='simulation'):
    return ['plan-cleaning','--targets',str(demo/'targets.jsonl'),'--calibration',str(demo/'calibration.json'),
            '--mode',mode,'--output',str(output)]


def test_actual_offline_module03_04_integration_and_visualization(demo,monkeypatch):
    for name in ('imshow','waitKey','destroyAllWindows'): monkeypatch.setattr(cv2,name,lambda *a,**k:pytest.fail('Offline planner must not open GUI'))
    paths=[demo/'targets.jsonl',demo/'calibration.json',demo/'robot_pose.jsonl',Path('config/target_message_v1.schema.json'),Path('config/robot_pose_v1.schema.json')]
    before=[p.read_bytes() for p in paths]
    output=demo/'simulation.jsonl';directory=demo/'preview'
    assert main.main(command(demo,output)+['--robot-poses',str(demo/'robot_pose.jsonl'),'--visualize-dir',str(directory)])==0
    plans=read(output)
    assert [p['reachability'] for p in plans]==['FULLY_REACHABLE','PARTIALLY_REACHABLE','UNREACHABLE']
    assert all(p['configuration_id'] and p['calibration_id'] and p['geometry_source']=='SIMULATION' for p in plans)
    assert all(p['execution_authorization']=='NONE' and not p['actionable'] for p in plans)
    assert all(p['target_record_key'].startswith('module05_synthetic:') for p in plans)
    assert len(list(directory.glob('*.png')))==3
    assert cv2.imread(str(directory/'plan_000_target_1.png')).shape==(800,1400,3)
    assert [p.read_bytes() for p in paths]==before


def test_default_physical_mode_stays_unknown(demo):
    output=demo/'physical.jsonl'
    args=command(demo,output,'physical')
    args.remove('--mode');args.remove('physical')
    assert main.main(args)==0
    plans=read(output)
    assert all(p['reachability']=='UNKNOWN' and p['approach_pose'] is None and not p['coverage_segments'] for p in plans)
    assert all('UNCONFIGURED' in p['reasons'][0] for p in plans)


def test_output_overwrite_and_mode_confusion_rejected(demo):
    protected=demo/'targets.jsonl';before=protected.read_bytes()
    assert main.main(command(demo,protected))==1 and protected.read_bytes()==before
    output=demo/'new.jsonl'
    assert main.main(command(demo,output,'physical')+['--hardware','config/cleaning_demo.json'])==1
    assert not output.exists()


def test_latest_event_inactive_never_silently_planned(demo):
    path=demo/'targets.jsonl';records=read(path)
    last=records[0]
    inactive=dict(last,event_type='INACTIVE',active=False,
        target_message=dict(last['target_message'],active=False,geometry_cm=None,position_cm=None,
            last_known_geometry_cm=last['target_message']['geometry_cm']))
    with path.open('a') as stream: stream.write(json.dumps(inactive)+'\n')
    selected=load_targets(path)
    assert len(selected)==3
    output=demo/'inactive.jsonl'
    assert main.main(command(demo,output)+['--target-id','1'])==0
    assert read(output)[0]['reachability']=='UNKNOWN'
    override=demo/'override.jsonl'
    assert main.main(command(demo,override)+['--target-id','1','--allow-inactive'])==0
    p=read(override)[0]
    assert p['reachability']=='FULLY_REACHABLE' and p['target_validity']['inactive_override']
    assert not p['actionable']


def test_raw_frame_and_bare_v1_messages_supported(demo):
    events=read(demo/'targets.jsonl')
    frame=demo/'frame.json';frame.write_text(json.dumps({'localizations':[e['localization'] for e in events]}))
    output=demo/'raw.jsonl';args=command(demo,output);args[args.index('--targets')+1]=str(frame)
    assert main.main(args)==0 and len(read(output))==3
    message=demo/'message.json';message.write_text(json.dumps(events[0]['target_message']))
    output=demo/'compact.jsonl';args=command(demo,output);args[args.index('--targets')+1]=str(message)
    assert main.main(args)==0
    p=read(output)[0]
    assert p['geometric_quality']['source_geometry']=='compact_physical_contours'
    assert p['localization_status']=='UNVERIFIED'


def test_latest_lost_robot_pose_not_replaced_by_old_visible(demo):
    robot=read(demo/'robot_pose.jsonl')[0]
    lost=dict(robot,tracking_status='LOST',position_cm=None,heading_deg=None,marker_corners_pixels=None,marker_center_pixels=None)
    with (demo/'robot_pose.jsonl').open('a') as stream:stream.write(json.dumps(lost)+'\n')
    output=demo/'lost.jsonl'
    assert main.main(command(demo,output)+['--robot-poses',str(demo/'robot_pose.jsonl')])==0
    assert all(not p['robot_pose_validity']['current_pose_usable_for_offline_reference'] for p in read(output))
    assert all(p['route_to_approach_validated'] is False for p in read(output))


def test_as_of_requires_timezone_and_existing_visualizations_preserved(demo):
    output=demo/'bad.jsonl'
    assert main.main(command(demo,output)+['--as-of','2026-11-08T12:00:00'])==1
    assert not output.exists()
    directory=demo/'existing';directory.mkdir();(directory/'note.txt').write_text('keep')
    assert main.main(command(demo,output)+['--visualize-dir',str(directory)])==1
    assert (directory/'note.txt').read_text()=='keep'
