"""Direction-specific measured timing. References never silently become calibration."""
from copy import deepcopy
import json
import math
from pathlib import Path
from statistics import median
from vision.robot_calibration_wizard import write_new_json


def angle_error(target, actual):
    return (target-actual+180)%360-180


def finite(value, name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(name+' must be finite numeric')
    return float(value)


class MovementTiming:
    def __init__(self,data):
        if data.get('schema_version')!=1 or data.get('mode') not in {'PHYSICAL','SIMULATION'}:
            raise ValueError('Timing requires schema_version 1 and PHYSICAL/SIMULATION mode')
        self.data=deepcopy(data)
        for sample in data['samples']:
            self.validate_sample(sample)
        for sample in data.get('stopping_samples',[]):self.validate_stopping_sample(sample)
        for estimate in data.get('operator_estimates',[]):
            magnitude=finite(estimate['change_magnitude'],'Measured magnitude')
            if magnitude<=0:raise ValueError('Measured magnitude must be positive')
            # This validates units/range only; an unsigned turn is not a signed pose observation.
            self.validate_sample({**estimate,'observed_change':-magnitude if estimate['direction']=='B' else magnitude})
            if estimate.get('heading_sign_verified') is True and (type(estimate.get('heading_sign')) is not int or estimate['heading_sign'] not in {-1,1}):
                raise ValueError('Verified heading sign must be -1 or +1')

    @staticmethod
    def validate_stopping_sample(sample):
        if sample['direction'] not in {'F','B','L','R'}:raise ValueError('Direction must be F/B/L/R')
        for key in ('pose_latency_ms','stop_latency_ms','maximum_speed_per_second','coast_units'):
            value=finite(sample[key],key)
            if value<0 or (key=='maximum_speed_per_second' and value==0):raise ValueError('Invalid measured stopping bound: '+key)
        if type(sample['verified']) is not bool:raise ValueError('Verified must be boolean')
        if sample['units']!=('deg' if sample['direction'] in 'LR' else 'cm'):raise ValueError('Stopping units mismatch')

    def add_stopping(self,*,direction,pose_latency_ms,stop_latency_ms,maximum_speed_per_second,coast_units,verified,source):
        sample=dict(direction=direction,pose_latency_ms=pose_latency_ms,stop_latency_ms=stop_latency_ms,
                    maximum_speed_per_second=maximum_speed_per_second,coast_units=coast_units,verified=verified,
                    source=source,units='deg' if direction in 'LR' else 'cm')
        self.validate_stopping_sample(sample)
        self.data.setdefault('stopping_samples',[]).append(sample)

    def stopping(self,direction,observation_gap_seconds):
        """Measured upper bounds; no physical defaults or inferred maximum speed."""
        self.rate(direction)
        rows=[s for s in self.data.get('stopping_samples',[]) if s['direction']==direction and s['verified']]
        if not rows:raise ValueError('Uncalibrated stopping data: '+direction)
        bounds={k:max(s[k] for s in rows) for k in ('pose_latency_ms','stop_latency_ms','maximum_speed_per_second','coast_units')}
        bounds['margin']=bounds['maximum_speed_per_second']*(observation_gap_seconds+
            (bounds['pose_latency_ms']+bounds['stop_latency_ms'])/1000)+bounds['coast_units']
        return bounds

    @staticmethod
    def validate_sample(sample):
        if sample['direction'] not in {'F','B','L','R'}:raise ValueError('Direction must be F/B/L/R')
        duration=sample['duration_ms']
        if type(duration) is not int or not 1<=duration<=10000:raise ValueError('Duration must be 1..10000 ms')
        amount=finite(sample['observed_change'],'Observed change')
        if abs(amount)<.01 or (sample['direction'] in 'LR' and abs(amount)>=180):
            raise ValueError('Use a measurable nonzero change, turn samples below 180 degrees')
        if type(sample['verified']) is not bool:raise ValueError('Verified must be boolean')
        if sample.get('units')!=('deg' if sample['direction'] in 'LR' else 'cm'):
            raise ValueError('Sample units must match direction')
        if sample['direction']=='F' and amount<=0 or sample['direction']=='B' and amount>=0:
            raise ValueError('F must increase body-forward displacement; B must decrease it')

    def magnitude_rate(self,direction):
        samples=[s for s in self.data['samples'] if s['direction']==direction and s['verified']]
        estimates=[s for s in self.data.get('operator_estimates',[]) if s['direction']==direction and s['verified']]
        rates=[s['duration_ms']/abs(s['observed_change']) for s in samples]
        rates += [s['duration_ms']/s['change_magnitude'] for s in estimates]
        if not rates:raise ValueError('Uncalibrated direction: '+direction)
        return median(rates)

    def rate(self,direction):
        rate=self.magnitude_rate(direction)
        samples=[s for s in self.data['samples'] if s['direction']==direction and s['verified']]
        signs={1 if s['observed_change']>0 else -1 for s in samples}
        signs.update(s['heading_sign'] for s in self.data.get('operator_estimates',[]) if
            s['direction']==direction and s['verified'] and s.get('heading_sign_verified') is True)
        if direction in 'FB':signs.add(1 if direction=='F' else -1)
        if not signs:raise ValueError('Uncalibrated signed Module04 heading response: '+direction+'; physical magnitude recorded')
        if len(signs)!=1:raise ValueError('Conflicting measured direction signs: '+direction)
        return rate,signs.pop()

    def turn_direction(self,error):
        candidates=[]
        for direction in ('L','R'):
            try:rate,sign=self.rate(direction)
            except ValueError:continue
            if error*sign>0:candidates.append((rate,direction))
        if not candidates:raise ValueError('No calibrated turn direction for signed heading error')
        return min(candidates)[1]

    def add(self,*,direction,duration_ms,observed_change,verified,source):
        sample=dict(direction=direction,duration_ms=duration_ms,observed_change=observed_change,
                    units='deg' if direction in 'LR' else 'cm',verified=verified,source=source)
        self.validate_sample(sample)
        self.data['samples'].append(sample)
        return sample

    def summary(self):
        result={}
        for direction in ('F','B','L','R'):
            try:rate,sign=self.rate(direction);result[direction]={'ms_per_unit':rate,'sign':sign}
            except ValueError as exc:
                result[direction]={'status':'UNCALIBRATED','reason':str(exc)}
                try:
                    result[direction].update(ms_per_unit=self.magnitude_rate(direction),status='SIGNED_HEADING_REQUIRED')
                except ValueError:pass
        return {'mode':self.data['mode'],'directions':result,'samples':self.data['samples'],
                'stopping_samples':self.data.get('stopping_samples',[]),
                'operator_estimates':self.data.get('operator_estimates',[]),
                'references':self.data.get('references',[])}


def load_timing(path):
    return MovementTiming(json.loads(Path(path).read_text()))


def save_timing(path,timing):
    """Small immutable revisions; display the new path rather than replacing history."""
    path=Path(path);index=0
    while True:
        candidate=path if index==0 else path.with_name(f'{path.stem}_{index:03d}{path.suffix}')
        try:return write_new_json(candidate,timing.data)
        except FileExistsError:index+=1
