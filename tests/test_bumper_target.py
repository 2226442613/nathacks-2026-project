"""Approximate front parking, fresh arrival checks and existing cleaning handoff."""
import json
import math

import numpy as np
import pytest

import main
from navigation.geometry import Arena,front_parking_candidates,front_bumper_metrics
from navigation.simulation import SimTransport


def spill():
    return {'surface':'FLOOR','position_cm':{'x':90.,'y':70.},
            'geometry_cm':{'bounds':dict(min_x=80.,min_y=60.,max_x=100.,max_y=80.)}}


def test_front_parking_uses_extent_and_13_cm_front_offset_from_all_sides():
    arena=Arena(width=200,depth=150)
    goals=front_parking_candidates(spill(),arena)
    assert [(p['x_cm'],p['y_cm'],p['heading_deg']) for p in goals]==[
        (62.,70.,0.),(118.,70.,180.),(90.,42.,90.),(90.,98.,270.)]
    for goal in goals:
        assert arena.safe(goal)
        assert front_bumper_metrics(spill(),goal,26.,15.2)==pytest.approx(
            dict(clearance_cm=5.,facing_error_deg=0.))


def test_finite_bumper_accounts_for_width_yaw_overlap_and_missing_geometry():
    target=spill();pose=dict(x_cm=62.,y_cm=70.,heading_deg=0.)
    assert front_bumper_metrics(target,{**pose,'x_cm':67.},26.,15.2)['clearance_cm']==0.
    assert front_bumper_metrics(target,{**pose,'y_cm':100.},26.,15.2)['clearance_cm']>5.
    assert front_bumper_metrics(target,{**pose,'heading_deg':45.},26.,15.2)['facing_error_deg']==45.
    with pytest.raises(ValueError,match='bounding geometry'):
        front_parking_candidates({'position_cm':{'x':90.,'y':70.}},Arena())
    # Do not clamp an impossible pose and silently sacrifice the requested gap.
    crowded={'geometry_cm':{'bounds':dict(min_x=20.,min_y=20.,max_x=40.,max_y=40.)}}
    assert not front_parking_candidates(crowded,Arena(width=60,depth=60))


@pytest.mark.parametrize('navigation_only',[False,True])
@pytest.mark.parametrize('fault',['near','far','heading','camera'])
def test_arrival_refresh_uses_new_robot_pose_and_blocks_cleaning_on_bad_clearance(tmp_path,monkeypatch,navigation_only,fault):
    import navigation.automatic as module
    plants=[];initialize=SimTransport.__init__;scan=module.SceneScanner.scan
    def capture(self,plant):
        initialize(self,plant);plants.append(plant)
    monkeypatch.setattr(SimTransport,'__init__',capture)
    calls=0
    def refresh(self,*args,**kwargs):
        nonlocal calls
        records=scan(self,*args,**kwargs);calls+=1
        if calls==2:  # The robot changes AFTER the arrival spill observation.
            plant=plants[0]
            if fault=='camera':plant.lost=True
            elif fault=='heading':plant.heading+=30
            else:
                delta=5. if fault=='near' else -5.
                plant.x+=delta*math.cos(math.radians(plant.heading))
                plant.y+=delta*math.sin(math.radians(plant.heading))
        return records
    monkeypatch.setattr(module.SceneScanner,'scan',refresh)
    out=tmp_path/'arrival_fault'
    assert main.main(['auto-clean','--output-dir',str(out)]+(['--navigation-only'] if navigation_only else []))==1
    log=(out/'wire.log').read_text()
    assert '[TX] STOP' in log and 'TARGET,' not in log and '[TX] ARRIVED' not in log
    events=[json.loads(s) for s in (out/'progress.jsonl').read_text().splitlines()]
    assert len([e for e in events if e['event_type']=='AUTO_TASK_SELECTED'])==1
    assert not any(e.get('state','').startswith('RETURNING') for e in events)
    reason=json.loads((out/'summary.json').read_text())['reason']
    assert ('camera disconnected' if fault=='camera' else 'bumper clearance/facing') in reason


def test_arrival_scan_can_see_known_spill_beyond_padding_but_never_through_robot_mask():
    from test_automatic_cleaning import geometry_scanner
    scanner,frame,_=geometry_scanner()
    target=next(r for r in scanner.scan() if r['category']=='RED_SPILL')
    broad=np.zeros(frame.shape[:2],np.uint8);broad[180:280,180:300]=255
    hard=np.zeros_like(broad)
    scanner.frame_source=lambda:(frame,broad,hard)
    assert scanner.scan()==[]
    assert any(r['category']=='RED_SPILL' for r in scanner.scan(verify_target=target))
    hard[:]=255
    assert scanner.scan(verify_target=target)==[]
