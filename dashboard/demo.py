"""Deterministic synthetic scenario; never represented as camera detections."""
from datetime import datetime,timezone

from .model import iso

STEPS=['Clean scene · ready to monitor','Blue target detected · LOW priority',
       'Red target detected · HIGH moves ahead','Red task starts · blue remains pending',
       'Red task completed · simulated verification requested','Wall target detected · LOW priority',
       'Next floor task starts','Floor task completed','Wall task starts','Wall task completed']
BASE=datetime(2026,11,8,9,41,tzinfo=timezone.utc).timestamp()


def demo_event(index,category,surface,arena,calibration_id,step):
    w=arena['floor']['width_cm'];d=arena['floor']['depth_cm'];wh=(arena.get('wall') or {}).get('height_cm',0)
    x,y=({1:(w*.30,d*.62),2:(w*.67,d*.46),3:(w*.63,wh*.30)})[index]
    # Irregular actual synthetic boundaries, rather than rectangle-only monitor boxes.
    points=[[x-5,y-2],[x-2,y-4],[x+3,y-3],[x+6,y],[x+4,y+3],[x,y+4],[x-4,y+2]]
    area=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1])))/2
    geometry={'width_cm':11.,'depth_cm' if surface=='FLOOR' else 'height_cm':8.,'area_cm2':area,
              'contours':[{'kind':'outer','points':points}],'trust_state':'UNVERIFIED','representation':'synthetic polygon'}
    timestamp=iso(BASE+step*4)
    body={'schema_version':1,'message_type':'TARGET_DETECTION','record_key':f'demo:0:{index}',
          'target_id':index,'category':category,'surface':surface,'position_cm':{'x':x,'y' if surface=='FLOOR' else 'h':y},
          'geometry_cm':geometry,'geometry_status':'UNVERIFIED','localization_status':'UNVERIFIED',
          'calibration_id':calibration_id,'coordinate_frame':{'frame_id':calibration_id,'units':'cm'},'units':'cm',
          'timestamp':timestamp,'last_observed_timestamp':timestamp,'valid_until':iso(BASE+step*4+2),
          'first_observed_timestamp':timestamp,'active':True,'physically_verified':False,'actionable':False}
    return {'record_key':body['record_key'],'target_message':body,'active':True,'first_seen':timestamp,
            'event_type':'UPSERT','event_schema_version':1,'localization':{'full_physical_contours_cm':geometry['contours'],'geometry_status':'UNVERIFIED'}}
