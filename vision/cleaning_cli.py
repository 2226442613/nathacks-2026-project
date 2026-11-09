"""Read-only Module 03/04 ingestion and offline Module 05 planning CLI."""
from pathlib import Path
from datetime import datetime
import hashlib
import json
import cv2

from .calibration import load_calibration
from .cleaning_config import CleaningHardware
from .cleaning_planner import CleaningPlanner
from .cleaning_visualization import render_cleaning_plan
from .serialization import json_line


def read_records(path):
    text=Path(path).read_text(encoding='utf-8')
    try:
        root=json.loads(text)
        return root if isinstance(root,list) else [root]
    except json.JSONDecodeError:
        return [json.loads(line) for line in text.splitlines() if line.strip()]


def load_targets(path):
    latest={}
    for value in read_records(path):
        records=(value['target_events'] if 'target_events' in value else value['localizations'] if 'localizations' in value else [value])
        for record in records:
            body=record.get('target_message',record)
            if 'target_id' not in body: raise ValueError('Input contains no Module 03 target record')
            key=record.get('record_key') or body.get('record_key') or (body.get('calibration_id'),body['target_id'])
            latest[key]=record
    if len(latest)>1000: raise ValueError('Too many planning targets')
    return list(latest.values())


def add_cleaning_commands(commands):
    parser=commands.add_parser('plan-cleaning',help='Module 05 offline contact/spray reachability and declarative coverage; no execution')
    parser.add_argument('--targets',required=True,help='Module 03 localization/message/event JSON or JSONL')
    parser.add_argument('--calibration',default='config/camera_config.json')
    parser.add_argument('--robot-poses',help='Optional Module 04 JSON/JSONL, latest observation only')
    parser.add_argument('--hardware',help='Explicit hardware JSON; defaults to physical template or demo config according to selected mode')
    parser.add_argument('--mode',choices=('physical','simulation'),default='physical')
    parser.add_argument('--target-id',type=int)
    parser.add_argument('--allow-inactive',action='store_true',help='Explicit historical-target planning override; never execution authorization')
    parser.add_argument('--as-of',help='Optional timezone-aware ISO timestamp for offline freshness evaluation')
    parser.add_argument('--output',required=True,help='NEW plan JSONL file; existing experiments are never overwritten')
    parser.add_argument('--visualize-dir',help='Optional NEW directory for floor/wall PNG diagrams; no camera/GUI required')
    parser.set_defaults(headless=True)


def run_cleaning_command(args):
    hardware_path=args.hardware or ('config/cleaning_demo.json' if args.mode=='simulation' else 'config/cleaning_hardware.json')
    output=Path(args.output)
    protected=[args.targets,args.calibration,args.robot_poses,hardware_path]
    if output.exists() or any(v and output.resolve()==Path(v).resolve() for v in protected): raise ValueError('Choose a new output path; planning cannot overwrite input/experimental records')
    directory=Path(args.visualize_dir) if args.visualize_dir else None
    if directory and directory.exists(): raise ValueError('Choose a new visualization directory')
    moment=None
    if args.as_of:
        stamp=datetime.fromisoformat(args.as_of.replace('Z','+00:00'))
        if stamp.tzinfo is None: raise ValueError('--as-of must include timezone')
        moment=stamp.timestamp()
    calibration=load_calibration(args.calibration)
    identity='sha256:'+hashlib.sha256(Path(args.calibration).read_bytes()).hexdigest()
    hardware=CleaningHardware(json.loads(Path(hardware_path).read_text()))
    planner=CleaningPlanner(calibration,hardware,mode=args.mode.upper(),calibration_id=identity)
    robot=None
    if args.robot_poses:
        observations=read_records(args.robot_poses)
        if observations: robot=observations[-1].get('robot_pose',observations[-1])
        if robot is not None and robot.get('message_type')!='ROBOT_POSE': raise ValueError('Robot input is not a Module 04 pose')
    targets=load_targets(args.targets)
    if args.target_id is not None: targets=[t for t in targets if t.get('target_message',t).get('target_id')==args.target_id]
    if not targets: raise ValueError('No target records selected')
    plans=[planner.plan(t,robot,timestamp=moment,allow_inactive=args.allow_inactive) for t in targets]
    lines=[json_line(plan.to_dict()) for plan in plans]
    # Compute and serialize before creating output artifacts.
    images=[render_cleaning_plan(plan,calibration,hardware) for plan in plans] if directory else []
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as stream: stream.write('\n'.join(lines)+'\n')
    if directory:
        directory.mkdir(parents=True)
        for index,image in enumerate(images):
            if not cv2.imwrite(str(directory/f'plan_{index:03d}_target_{plans[index].target_id}.png'),image): raise OSError('Cannot save planning visualization')
