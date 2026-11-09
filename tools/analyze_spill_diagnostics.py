"""Summarize real diagnostic stages; optional independent pixel-box annotations.

Run: python -m tools.analyze_spill_diagnostics --run RUN [--labels LABELS.json]
This evaluates saved production outputs. It does not run or tune a detector.
"""
import argparse
import json
import math
from pathlib import Path


def iou(a,b):
    x,y,w,h = a
    X,Y,W,H = b
    intersection = max(0,min(x+w,X+W)-max(x,X))*max(0,min(y+h,Y+H)-max(y,Y))
    return intersection/(w*h+W*H-intersection)


def summarize(directory, labels=None):
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory/'frames.jsonl').read_text().splitlines()]
    report = {'frames':len(rows), 'candidate_counts':[r['metrics']['candidate_count'] for r in rows],
              'confirmed_counts':[r['metrics']['confirmed_count'] for r in rows],
              'geometry_states':[r['metrics']['physical_registration']['status'] for r in rows],
              'alignment_applied':[r['metrics']['background_alignment_applied'] for r in rows],
              'false_positives':None, 'classification_accuracy':None,
              'note':'No independent annotations: counts alone cannot establish false positives or accuracy.'}
    report['operator_marked_stage_counts'] = {
        stage:[{'frame':r['frame_index'],'confirmed':r['metrics']['confirmed_count']} for r in rows if r.get('operator_stage')==stage]
        for stage in 'ABCDEF' if any(r.get('operator_stage')==stage for r in rows)}
    if labels is None:
        return report
    if labels.get('schema_version') != 1 or labels.get('coordinate_system') != 'original_image_pixels':
        raise ValueError('Labels require schema_version 1 and original_image_pixels')
    segments = labels['segments']
    previous_end = -1
    per_stage = {}
    by_index = {r['frame_index']:r for r in rows}
    correct = matches = fp_total = expected_total = 0
    covered = set()
    occurrences = {}
    temporal = {}
    for segment in segments:
        start,end = segment['start_frame'],segment['end_frame']
        if any(isinstance(v,bool) or not isinstance(v,int) for v in (start,end)) or not 0 <= start <= end or start <= previous_end:
            raise ValueError('Segments must be non-overlapping increasing inclusive frame intervals')
        if any(index not in by_index for index in range(start,end+1)):
            raise ValueError('Segment references missing frames')
        previous_end = end
        stage = segment['stage']
        if stage not in ('A','B','C','D','E','F'):
            raise ValueError('Stage must be A, B, C, D, E or F')
        objects = segment['objects']
        if len({o['id'] for o in objects}) != len(objects):
            raise ValueError('Ground-truth IDs must be unique within each segment')
        for obj in objects:
            box = obj['bounding_box']
            if (not isinstance(obj['id'],str) or not obj['id'] or
                obj['category'] not in ('RED_SPILL','NON_RED_SPILL') or len(box)!=4 or
                any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in box) or
                min(box[:2])<0 or min(box[2:])<=0):
                raise ValueError('Object requires an ID, valid category and finite positive pixel box')
        stage_rows = []
        key = f'{stage}:{start}'
        for obj in objects:
            occurrence = f'{key}:{obj["id"]}'
            occurrences[occurrence] = {'stage':stage,'object_id':obj['id'],'start_frame':start,
                'first_confirmed_frame':None,'delay_frames':None,'delay_seconds':None,
                'target_ids':set(),'missed_frames':0,'duplicate_observations':0}
        for index in range(start,end+1):
            row = by_index[index]
            covered.add(index)
            targets = [t for t in row['record']['detections'] if t['confirmed']]
            pairs = sorted([(iou(o['bounding_box'],t['bounding_box']),oi,ti)
                            for oi,o in enumerate(objects) for ti,t in enumerate(targets)],reverse=True)
            used_objects,used_targets = set(),set()
            matched_objects = {}
            for overlap,oi,ti in pairs:
                if overlap < .25 or oi in used_objects or ti in used_targets:
                    continue
                used_objects.add(oi);used_targets.add(ti);matched_objects[oi]=ti
                matches += 1
                correct += objects[oi]['category'] == targets[ti]['category']
            false = len(targets)-len(used_targets)
            fp_total += false
            expected_total += len(objects)
            stage_rows.append({'frame':index,'confirmed':len(targets),'expected':len(objects),
                               'false_positives':false,'missed':len(objects)-len(used_objects)})
            for oi,obj in enumerate(objects):
                item = occurrences[f'{key}:{obj["id"]}']
                duplicates = sum(iou(obj['bounding_box'],t['bounding_box'])>=.25 for t in targets)
                item['duplicate_observations'] += max(0,duplicates-1)
                if oi not in matched_objects:
                    item['missed_frames'] += 1
                    continue
                target = targets[matched_objects[oi]]
                item['target_ids'].add(target['target_id'])
                if item['first_confirmed_frame'] is None:
                    item['first_confirmed_frame'] = index
                    item['delay_frames'] = index-start
                    item['delay_seconds'] = row['elapsed_seconds']-by_index[start]['elapsed_seconds']
            # Only supplied empty scenes establish unambiguous disappearance.
            if not objects:
                temporal[key] = {'stage':stage,'start_frame':start,
                    'first_zero_confirmed_frame':next((r['frame'] for r in stage_rows if r['confirmed']==0),None),
                    'frames_with_lingering_confirmed_targets':sum(r['confirmed']>0 for r in stage_rows)}
        per_stage[key] = stage_rows
    for item in occurrences.values():
        item['target_ids'] = sorted(item['target_ids'])
        item['id_fragmentation'] = max(0,len(item['target_ids'])-1)
    global_ids = {}
    for item in occurrences.values():
        global_ids.setdefault(item['object_id'],set()).update(item['target_ids'])
    report.update(false_positives=fp_total,matched_observations=matches,expected_observations=expected_total,
                  classification_accuracy=correct/matches if matches else None,
                  recall=matches/expected_total if expected_total else None,
                  per_stage=per_stage,objects=list(occurrences.values()),disappearance=list(temporal.values()),
                  id_stability={key:{'target_ids':sorted(value),'id_fragmentation':max(0,len(value)-1)}
                                for key,value in global_ids.items()},
                  covered_frames=len(covered),unlabeled_frames=len(rows)-len(covered),
                  missing_stages=sorted(set('ABCDEF')-{s['stage'] for s in segments}),
                  note='Independent supplied boxes matched one-to-one at IoU >= 0.25; accuracy is conditional on matched detections. No cm or motor validation.')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True)
    parser.add_argument('--labels',help='Independent A–F stage/object box annotation JSON')
    args = parser.parse_args()
    labels = json.loads(Path(args.labels).read_text()) if args.labels else None
    print(json.dumps(summarize(args.run,labels),indent=2,allow_nan=False))


if __name__ == '__main__':
    main()
