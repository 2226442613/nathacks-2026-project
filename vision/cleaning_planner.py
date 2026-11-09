"""Module 05 declarative contact coverage; no actuator/control/transport imports."""
from copy import deepcopy
from dataclasses import asdict,dataclass,field
from datetime import datetime,timezone
import hashlib
import math

import numpy as np

from .calibration import DualPlaneCalibration,GroundCalibration
from .localization import utc_timestamp
from .robot_tracking import robot_pose_is_current
from .serialization import json_native,json_line
from .cleaning_config import CleaningHardware
from .cleaning_geometry import (area,bounds,validate_rings,rotation,footprint,contained,clip_polygon,
    rectangle_set,covered_area,clipped_rings,intervals_union)


@dataclass(frozen=True)
class ApproachPose:
    x_cm: float
    y_cm: float
    heading_deg: float
    reference_frame: str | None
    tool: str
    target_id: int
    surface: str
    desired_contact_geometry: dict
    clearance_requirements: dict
    localization_status: str
    validity: dict=field(default_factory=dict)


@dataclass(frozen=True)
class CoverageSegment:
    segment_type: str
    required_tool: str | None
    start_pose: dict | None=None
    end_pose: dict | None=None
    waypoints: list=field(default_factory=list)
    coverage_metadata: dict=field(default_factory=dict)
    prerequisites: tuple=('Module 06 collision-free routing and physical controller validation required',)
    declarative_only: bool=True


@dataclass
class CleaningPlan:
    target_id: int | None
    surface: str
    timestamp: str
    calibration_id: str | None
    configuration_id: str
    geometry_source: str
    localization_status: str
    cleaning_mode: str | None=None
    reachability: str='UNKNOWN'
    reasons: list=field(default_factory=list)
    warnings: list=field(default_factory=list)
    required_tools: list=field(default_factory=list)
    approach_pose: dict | None=None
    coverage_segments: list=field(default_factory=list)
    metrics: dict=field(default_factory=dict)
    hardware_configuration: dict=field(default_factory=dict)
    target_geometry: list=field(default_factory=list)
    contact_reachable_rectangles_cm: list=field(default_factory=list)
    spray_reachable_rectangles_cm: list=field(default_factory=list)
    combined_cleanable_rectangles_cm: list=field(default_factory=list)
    unreachable_target_regions: list=field(default_factory=list)
    coordinate_frame: dict=field(default_factory=dict)
    source_provenance: dict=field(default_factory=dict)
    localization_trust: dict=field(default_factory=dict)
    target_record_key: str | None=None
    target_validity: dict=field(default_factory=dict)
    robot_pose_validity: dict=field(default_factory=dict)
    geometric_quality: dict=field(default_factory=dict)
    wall_contact_model: dict=field(default_factory=dict)
    schema_version: int=1
    message_type: str='CLEANING_PLAN'
    configuration_status: str='NOT_EVALUATED'
    execution_authorization: str='NONE'
    actionable: bool=False
    physical_execution_authorization: str='NONE'
    route_to_approach_validated: bool=False

    def to_dict(self):
        result=asdict(self);result['geometric_reachability']=self.reachability
        return json_native(result)


def target_record(value,allow_inactive=False):
    """Read Module 03 localization, v1 messages and append-only event snapshots."""
    value=value.to_dict() if hasattr(value,'to_dict') else deepcopy(value)
    event=value if 'target_message' in value else None
    record=deepcopy(event['target_message'] if event else value)
    localization=event.get('localization',{}) if event else value
    if event:
        record['active']=event['active']
        record['record_key']=event.get('record_key',record.get('record_key'))
        record.setdefault('calibration_quality',localization.get('calibration_quality'))
    geometry=record.get('geometry_cm')
    if geometry is None and allow_inactive and record.get('active') is False: geometry=record.get('last_known_geometry_cm')
    rings=localization.get('full_physical_contours_cm')
    if not rings and geometry:
        rings=geometry.get('contours')
        if not rings and geometry.get('contour'): rings=[{'kind':'outer','points':geometry['contour']}]
    record['_rings']=rings
    record['_geometry_metadata']={key:(geometry or {}).get(key) for key in ('representation','effective_tolerance_cm','area_method','payload_status','area_cm2')}
    record['_geometry_quality']=localization.get('geometry_status',record.get('geometry_status',(geometry or {}).get('trust_state','UNVERIFIED')))
    record['_source_geometry']='full_physical_contours_cm' if localization.get('full_physical_contours_cm') else 'compact_physical_contours'
    return record


class CleaningPlanner:
    def __init__(self,calibration,hardware,*,mode,calibration_id=None):
        if mode not in ('SIMULATION','PHYSICAL'): raise ValueError('Select SIMULATION or PHYSICAL mode explicitly')
        self.hardware=hardware if isinstance(hardware,CleaningHardware) else CleaningHardware(hardware)
        if self.hardware.mode!=mode: raise ValueError('Hardware source does not match selected planning mode; demo dimensions cannot become physical')
        self.calibration=calibration
        self.floor=calibration.floor if isinstance(calibration,DualPlaneCalibration) else calibration
        if not isinstance(self.floor,GroundCalibration): raise ValueError('Valid Module 01 floor calibration required')
        data=calibration.to_dict();type(calibration).from_dict(data)
        self.calibration_id=calibration_id or 'sha256:'+hashlib.sha256(json_line(data).encode()).hexdigest()
        self.width,self.depth=self.floor.width_cm,self.floor.height_cm
        self.h=self.hardware.data
        a,b=self.h['body']['safety_margin_cm'],self.h['body']['position_uncertainty_cm']
        self.margin=None if a is None or b is None else a+b
        self.side_envelope=self.hardware.side_collision_envelope()

    def pose(self,x,y,heading): return {'x_cm':float(x),'y_cm':float(y),'heading_deg':float(heading%360)}

    def safe_pose(self,pose):
        if self.margin is None: raise ValueError('UNCONFIGURED chassis clearance/uncertainty')
        b=self.h['body']
        if not contained(footprint(pose,b['chassis_length_cm'],b['chassis_width_cm'],margin=self.margin),self.width,self.depth):return False
        e=self.side_envelope
        return e is None or contained(footprint(pose,e['length_cm'],e['width_cm'],(e['offset_forward_cm'],e['offset_left_cm'])),self.width,self.depth)

    def safe_floor_tools(self,pose):
        s=self.h['floor_sponge']
        return self.safe_pose(pose) and contained(footprint(pose,s['contact_length_cm'],s['contact_width_cm'],
            (s['offset_forward_cm'],s['offset_lateral_cm'])),self.width,self.depth)

    def plan(self,target,robot_pose=None,*,timestamp=None,allow_inactive=False):
        moment=datetime.now(timezone.utc).timestamp() if timestamp is None else float(timestamp)
        if not math.isfinite(moment): raise ValueError('Planning time must be finite')
        record=target_record(target,allow_inactive)
        surface=record.get('surface','UNKNOWN');trust=record.get('localization_status','UNVERIFIED')
        if record.get('_geometry_quality')!='VERIFIED' or trust!='VERIFIED': trust='UNVERIFIED'
        plan=CleaningPlan(record.get('target_id'),surface,utc_timestamp(moment),self.calibration_id,self.hardware.identity,self.hardware.mode,trust)
        plan.target_record_key=record.get('record_key')
        plan.coordinate_frame={'frame_id':self.calibration_id,'units':'cm','floor_axes':{'x':'Module 01 +X','y':'Module 01 +Y'},'heading':'0=+X, positive toward +Y; [0,360)',
            'floor_width_cm':self.width,'floor_depth_cm':self.depth,'shared_floor_boundary':getattr(self.calibration,'floor_boundary',None),'wall_x_reversed':getattr(self.calibration,'wall_x_reversed',False)}
        plan.source_provenance={'target_calibration_id':record.get('calibration_id'),'target_localization_status':record.get('localization_status'),
            'geometry_trust':record.get('_geometry_quality'),'camera_registration':record.get('camera_registration'),'calibration_quality':record.get('calibration_quality')}
        plan.hardware_configuration=deepcopy(self.h)
        if self.hardware.mode=='SIMULATION': plan.warnings.append('SIMULATED HARDWARE: illustrative geometry, not measured robot dimensions')
        if self.side_envelope:plan.warnings.append('Side overhang uses a conservative observed STATIC collision box, not a floor-contact tool or verified sponge rotation sweep')
        if self.h.get('nozzle_observation'):plan.warnings.append('Approximate nozzle landing observation does not establish exact offsets, trajectory or calibrated spray coverage')
        plan.warnings+=list(record.get('quality_warnings',record.get('warnings',[])))
        plan.warnings+=['No obstacle-free route or physical cleaning effectiveness is established; all segments are declarative',
                        'Localization/calibration/actuator and contact-force validation remain prerequisites']
        plan.geometric_quality={'method':'continuous polygon clipping against unions of rectangular swept footprints',
            'numerical_tolerance_cm2':1e-8,'physical_accuracy_claim':False,'physical_cleaning_proven':False,'coverage_scope':'provided polygons and configured effective rectangular tool model only','source_geometry':record.get('_source_geometry'),
            'source_geometry_metadata':record.get('_geometry_metadata'),'holes_preserved':True,'disconnected_components_preserved':True,
            'stroke_strategy':'cardinal, rectangular effective tool model; holes may be traversed but never counted as target area'}
        active=record.get('active')
        fresh=None
        try:
            until=record.get('valid_until')
            if until:
                seen=record.get('last_observed_timestamp',record.get('timestamp'))
                fresh=bool(seen and datetime.fromisoformat(seen.replace('Z','+00:00')).timestamp()<=moment<=datetime.fromisoformat(until.replace('Z','+00:00')).timestamp())
            elif record.get('timestamp'): fresh=0<=moment-datetime.fromisoformat(record['timestamp'].replace('Z','+00:00')).timestamp()<=2
        except (ValueError,TypeError): plan.warnings.append('Target freshness metadata invalid')
        plan.target_validity={'active':active,'fresh':fresh,'inactive_override':bool(allow_inactive and active is False),
            'observation_timestamp':record.get('last_observed_timestamp',record.get('timestamp')),'offline_geometric_plan_only':True,
            'evaluated_as_of':utc_timestamp(moment),'evaluation_timestamp_supplied':timestamp is not None}
        if fresh is not True: plan.warnings.append('Target observation is stale or freshness is unknown; this is an offline geometric plan only')
        robot=robot_pose.to_dict() if hasattr(robot_pose,'to_dict') else deepcopy(robot_pose)
        robot_fresh=bool(robot and robot_pose_is_current(robot,timestamp=moment))
        robot_same=bool(robot and robot.get('calibration_id')==self.calibration_id)
        robot_usable=bool(robot_fresh and robot_same and robot.get('position_cm') is not None and robot.get('heading_deg') is not None)
        if robot_usable:
            try:
                json_native(robot)
                values=[robot['position_cm']['x'],robot['position_cm']['y'],robot['heading_deg']]
                robot_usable=all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values) and robot.get('localization_status') not in ('INVALID','UNAVAILABLE')
            except (ValueError,KeyError,TypeError): robot_usable=False
        plan.robot_pose_validity={'provided':robot is not None,'fresh':robot_fresh,'calibration_matches':robot_same,
            'current_pose_usable_for_offline_reference':robot_usable,'localization_status':None if not robot else robot.get('localization_status'),
            'navigation_to_approach_authorized':False}
        plan.localization_trust={'target':trust,'target_geometry':record.get('_geometry_quality'),'robot':None if not robot else robot.get('localization_status'),
            'robot_current':robot_usable,'target_current':fresh,'overall':'VERIFIED' if trust=='VERIFIED' and fresh is True and record.get('calibration_id')==self.calibration_id and robot_usable and robot.get('localization_status')=='VERIFIED' else 'UNVERIFIED'}
        plan.localization_status=plan.localization_trust['overall']
        if robot_usable: plan.source_provenance['robot_pose_quality']=robot.get('pose_quality')
        if not robot_usable: plan.warnings.append('Robot pose absent, stale or incompatible; coverage is an offline geometric plan, not a route from the robot')
        if isinstance(plan.target_id,bool) or not isinstance(plan.target_id,int) or plan.target_id<0:
            plan.reasons.append('Stable nonnegative tracked target ID required');return plan
        frame=record.get('coordinate_frame') or {}
        if not isinstance(frame,dict):
            plan.reasons.append('Invalid target coordinate-frame metadata');return plan
        if record.get('units',frame.get('units','cm'))!='cm':
            plan.reasons.append('Target coordinate units must be centimeters');return plan
        if active is False and not allow_inactive:
            plan.reasons.append('Target is inactive; explicit operator override required for historical planning');return plan
        if surface not in ('FLOOR','WALL'):
            plan.reasons.append('Ambiguous or unknown surface; no arbitrary physical plane selected');return plan
        if record.get('localization_status') in ('INVALID','UNAVAILABLE') or record.get('_geometry_quality') in ('INVALID','UNAVAILABLE'):
            plan.reasons.append('Invalid/unavailable source localization or geometry');return plan
        if record.get('calibration_id')!=self.calibration_id:
            plan.reasons.append('Target calibration identity missing or incompatible');return plan
        if frame.get('frame_id',self.calibration_id)!=self.calibration_id:
            plan.reasons.append('Target coordinate frame disagrees with calibration');return plan
        try:
            rings=validate_rings(record['_rings']);bb=bounds(rings)
            limit_y=self.depth if surface=='FLOOR' else self.calibration.wall.height_cm if isinstance(self.calibration,DualPlaneCalibration) else 0
            limit_x=self.width if surface=='FLOOR' else self.calibration.wall.width_cm if isinstance(self.calibration,DualPlaneCalibration) else 0
            if bb[0]<-1e-7 or bb[1]<-1e-7 or bb[2]>limit_x+1e-7 or bb[3]>limit_y+1e-7: raise ValueError('Target contour lies outside calibrated surface')
            plan.target_geometry=rings
            plan.metrics={'total_target_area_cm2':area(rings),'contact_reachable_area_cm2':None,'spray_reachable_area_cm2':None,
                'combined_cleanable_area_cm2':None,'estimated_cleanable_fraction':None,
                'lower_unreachable_area_cm2':None,'upper_unreachable_area_cm2':None}
            if surface=='WALL':
                lo,hi=self.h['wall_sponge']['min_contact_height_cm'],self.h['wall_sponge']['max_contact_height_cm']
                side=self.h['wall_sponge']['side'];protrusion=self.h['wall_sponge']['lateral_protrusion_cm'];width=self.h['body']['chassis_width_cm']
                plan.wall_contact_model={'side':side,'required_parallel_heading_deg':180 if side=='LEFT' else 0 if side=='RIGHT' else None,
                    'min_contact_height_cm':lo,'max_contact_height_cm':hi,'rotation_geometry_verified':False,
                    'uncompressed_centerline_reference_cm':None if width is None or protrusion is None else width/2+protrusion}
                if lo is not None:plan.metrics['lower_unreachable_area_cm2']=covered_area(rings,[[0,0,limit_x,min(limit_y,lo)]]) if lo>0 else 0.
                if lo is not None and hi is not None:
                    plan.wall_contact_model.update(fixed_height=True)
                    plan.metrics['upper_unreachable_area_cm2']=covered_area(rings,[[0,max(0,hi),limit_x,limit_y]]) if hi<limit_y else 0.
            tools=['floor_sponge','floor_nozzle'] if surface=='FLOOR' else ['wall_sponge','wall_nozzle']
            plan.required_tools=['FRONT_FLOOR_SPONGE','FIXED_FLOOR_SPRAY'] if surface=='FLOOR' else ['SIDE_ROTATING_SPONGE','ADJUSTABLE_WALL_SPRAY']
            plan.cleaning_mode=surface+'_CONTACT_CLEANING'
            absent=[name for name in tools if self.h[name]['installed'] is False]
            if absent:
                self._unreachable(plan,'Required mechanism not installed: '+', '.join(absent));return plan
            missing=self.hardware.missing(surface)
            if missing:
                plan.configuration_status='CONFIGURATION_REQUIRED'
                plan.reasons.append('CONFIGURATION_REQUIRED: UNCONFIGURED hardware/evidence: '+', '.join(missing));return plan
            plan.configuration_status='CONFIGURED'
            if self.width<=2*self.margin+self.h['body']['chassis_length_cm'] and self.depth<=2*self.margin+self.h['body']['chassis_width_cm']:
                self._unreachable(plan,'Chassis and clearance cannot fit in floor arena');return plan
            if surface=='FLOOR': self._floor(plan,robot if robot_usable else None)
            else: self._wall(plan,robot if robot_usable else None)
        except (ValueError,TypeError,KeyError,IndexError,AttributeError,np.linalg.LinAlgError) as exc:
            plan.reachability='UNKNOWN';plan.reasons.append(str(exc));plan.approach_pose=None;plan.coverage_segments=[]
        return plan

    def _unreachable(self,plan,reason):
        plan.reachability='UNREACHABLE';plan.reasons.append(reason)
        plan.metrics.update(combined_cleanable_area_cm2=0.,estimated_cleanable_fraction=0.)
        plan.unreachable_target_regions=deepcopy(plan.target_geometry)

    def _strokes(self,rings,heading,tool):
        """Plan cardinal forward strokes with body AND tool envelope containment."""
        r=rotation(heading);local=[{'kind':a['kind'],'points':(np.asarray(a['points']) @ r).tolist()} for a in rings]
        arena=np.array([[0,0],[self.width,0],[self.width,self.depth],[0,self.depth]]) @ r
        lo,hi=arena.min(axis=0),arena.max(axis=0)
        body=self.h['body'];offset=np.array([tool['offset_forward_cm'],tool['offset_lateral_cm']])
        length,width=tool['length'],tool['width']
        lower=np.maximum(lo+[body['chassis_length_cm']/2+self.margin,body['chassis_width_cm']/2+self.margin],lo+[length/2,width/2]-offset)
        upper=np.minimum(hi-[body['chassis_length_cm']/2+self.margin,body['chassis_width_cm']/2+self.margin],hi-[length/2,width/2]-offset)
        # The front sponge is fixed, so it remains present during wetting passes.
        sponge=self.h['floor_sponge']
        permanent_offset=np.array([sponge['offset_forward_cm'],sponge['offset_lateral_cm']])
        permanent_half=np.array([sponge['contact_length_cm']/2,sponge['contact_width_cm']/2])
        lower=np.maximum(lower,lo+permanent_half-permanent_offset)
        upper=np.minimum(upper,hi-permanent_half-permanent_offset)
        if self.side_envelope:
            e=self.side_envelope;offset_side=np.array([e['offset_forward_cm'],e['offset_left_cm']]);half=np.array([e['length_cm'],e['width_cm']])/2
            lower=np.maximum(lower,lo+half-offset_side);upper=np.minimum(upper,hi-half-offset_side)
        if np.any(upper-lower<1e-8): return [],[]
        bb=bounds(local);step=width*(1-self.h['planning']['pass_overlap_fraction'])
        count=max(1,math.ceil(max(0,bb[3]-bb[1]-width)/step)+1)
        if count>self.h['planning']['max_strokes']: raise ValueError('Required stroke count exceeds configured planner limit')
        rows=np.linspace(bb[1]+width/2,bb[3]-width/2,count) if count>1 else [(bb[1]+bb[3])/2]
        centers=sorted(set(float(np.clip(y-offset[1],lower[1],upper[1])) for y in rows))
        strokes=[];rects=[]
        for t in centers:
            y=t+offset[1];strip=[bb[0]-1,y-width/2,bb[2]+1,y+width/2]
            parts=clipped_rings(local,[strip])
            if area(parts)<=1e-9: continue
            intervals=[]
            for outer in (a for a in parts if a['kind']=='outer'):
                p=np.asarray(outer['points']);intervals.append([p[:,0].min(),p[:,0].max()])
            for xmin,xmax in intervals_union(intervals):
                start=float(np.clip(xmin-length/2-offset[0]-self.h['planning']['lead_in_cm'],lower[0],upper[0]))
                end=float(np.clip(xmax+length/2-offset[0]+self.h['planning']['lead_in_cm'],lower[0],upper[0]))
                if end-start<1e-8: continue
                a=np.array([start,t]) @ r.T;b=np.array([end,t]) @ r.T
                poses=[self.pose(*a,heading),self.pose(*b,heading)]
                if not all(self.safe_pose(p) for p in poses): raise ValueError('Proposed stroke violates inflated chassis containment')
                swept=np.array([[start+offset[0]-length/2,y-width/2],[end+offset[0]+length/2,y-width/2],
                                [end+offset[0]+length/2,y+width/2],[start+offset[0]-length/2,y+width/2]]) @ r.T
                low,high=swept.min(axis=0),swept.max(axis=0);rect=[float(low[0]),float(low[1]),float(high[0]),float(high[1])]
                if covered_area(rings,[rect])<=1e-9: continue
                strokes.append({'start':poses[0],'end':poses[1],'swept_rectangle_cm':rect});rects.append(rect)
                if len(strokes)>self.h['planning']['max_strokes']: raise ValueError('Stroke count exceeds configured planner limit')
        return strokes,rectangle_set(rects)

    def _floor(self,plan,robot):
        sponge=self.h['floor_sponge'];nozzle=self.h['floor_nozzle']
        angle=nozzle['fixed_orientation_deg']
        if abs(angle/90-round(angle/90))>1e-8: raise ValueError('Non-cardinal nozzle footprint orientation requires a calibrated aligned effective rectangle; unsupported model remains UNKNOWN')
        contact=dict(offset_forward_cm=sponge['offset_forward_cm'],offset_lateral_cm=sponge['offset_lateral_cm'],length=sponge['contact_length_cm'],width=sponge['contact_width_cm'])
        length,width=nozzle['coverage_length_cm'],nozzle['coverage_width_cm']
        if round(angle/90)%2: length,width=width,length
        spray=dict(offset_forward_cm=nozzle['offset_forward_cm'],offset_lateral_cm=nozzle['offset_lateral_cm'],length=length,width=width)
        candidates=[]
        for heading in (0,90,180,270):
            try:
                wipe,cr=self._strokes(plan.target_geometry,heading,contact);wet,sr=self._strokes(plan.target_geometry,heading,spray)
                if len(wipe)+len(wet)>self.h['planning']['max_strokes']: continue
            except ValueError as exc:
                if 'count exceeds' in str(exc): continue
                raise
            combined=rectangle_set(cr,sr,'intersection')
            amount=covered_area(plan.target_geometry,combined)
            turn=abs((heading-robot['heading_deg']+180)%360-180) if robot else heading
            candidates.append((amount,covered_area(plan.target_geometry,cr),-len(wipe)-len(wet),-turn,wipe,wet,cr,sr,combined))
        if not candidates: raise ValueError('All candidate orientations exceed configured stroke budget')
        best=max(candidates,key=lambda a:a[:4]);_,_,_,_,wipe,wet,cr,sr,combined=best
        self._metrics(plan,cr,sr,combined)
        if not combined or plan.metrics['combined_cleanable_area_cm2']<=1e-8:
            plan.reasons.append('No jointly sprayable and sponge-contactable target region within contained chassis/tool strokes');return
        self._segments(plan,wipe,wet,robot,contact)
        if plan.reachability=='PARTIALLY_REACHABLE': plan.reasons.append('Front-tool offsets, spray footprint or inflated arena constraints leave part of the actual target uncovered')

    def _metrics(self,plan,cr,sr,combined):
        total=plan.metrics['total_target_area_cm2']
        ca=min(total,covered_area(plan.target_geometry,cr));sa=min(total,covered_area(plan.target_geometry,sr));both=min(total,covered_area(plan.target_geometry,combined))
        plan.contact_reachable_rectangles_cm=cr;plan.spray_reachable_rectangles_cm=sr;plan.combined_cleanable_rectangles_cm=combined
        plan.metrics.update(contact_reachable_area_cm2=ca,spray_reachable_area_cm2=sa,combined_cleanable_area_cm2=both,estimated_cleanable_fraction=both/total)
        plan.reachability='UNREACHABLE' if both<=1e-8 else 'FULLY_REACHABLE' if total-both<=1e-8 else 'PARTIALLY_REACHABLE'
        remaining=rectangle_set([bounds(plan.target_geometry)],combined,'difference')
        plan.unreachable_target_regions=clipped_rings(plan.target_geometry,remaining)

    def _approach(self,plan,pose,tool,desired):
        return asdict(ApproachPose(**pose,reference_frame=self.calibration_id,tool=tool,target_id=plan.target_id,surface=plan.surface,
            desired_contact_geometry=desired,clearance_requirements={'body_margin_cm':self.h['body']['safety_margin_cm'],
                'position_uncertainty_cm':self.h['body']['position_uncertainty_cm'],'inflated_chassis_within_floor':self.safe_pose(pose)},
            localization_status=plan.localization_status,validity={'geometric_pose_valid':self.safe_pose(pose),'obstacles_checked':False,'execution_authorization':'NONE'}))

    def _segments(self,plan,wipe,wet,robot,contact_model,*,wall=False):
        if not wipe: return
        tool='SIDE_ROTATING_SPONGE' if wall else 'FRONT_FLOOR_SPONGE';spraytool='ADJUSTABLE_WALL_SPRAY' if wall else 'FIXED_FLOOR_SPRAY'
        first=wet[0]['start'] if wet else wipe[0]['start']
        desired=dict(contact_model,contact_stroke_start_pose=wipe[0]['start'])
        plan.approach_pose=self._approach(plan,first,spraytool if wet else tool,desired)
        current=None if not robot else self.pose(robot['position_cm']['x'],robot['position_cm']['y'],robot['heading_deg'])
        segments=[]
        for index,stroke in enumerate(wet):
            segments.append(CoverageSegment('APPROACH',None,current,stroke['start'],coverage_metadata={'route':None,'purpose':'wetting approach; route is delegated to Module 06'}))
            segments.append(CoverageSegment('ALIGN',spraytool,stroke['start'],stroke['start']))
            segments.append(CoverageSegment('SPRAY',spraytool,stroke['start'],stroke['end'],[stroke['start'],stroke['end']],
                {'swept_region':stroke['swept_rectangle_cm'],'spray_model':contact_model.get('nozzle',self.h['floor_nozzle']) if wall else self.h['floor_nozzle'],
                 'actuator_settings':None,'requires_calibrated_actuator_settings':True}))
            current=stroke['end']
        for stroke in wipe:
            segments.append(CoverageSegment('APPROACH',None,current,stroke['start'],coverage_metadata={'route':None,'purpose':'contact approach; route is delegated to Module 06'}))
            segments.append(CoverageSegment('ALIGN',tool,stroke['start'],stroke['start']))
            segments.append(CoverageSegment('CONTACT_CLEAN',tool,stroke['start'],stroke['end'],[stroke['start'],stroke['end']],
                {'swept_region':stroke['swept_rectangle_cm'],'contact_model':contact_model,'vertical_sponge_motion':False,
                 'engagement':'controlled fixed-height side contact; force calibration required' if wall else 'fixed front underside sponge translated by chassis'}))
            current=stroke['end']
        retreat=deepcopy(current)
        if wall: retreat['y_cm']+=self.h['planning']['retreat_cm']
        else:
            delta=rotation(current['heading_deg']) @ np.array([self.h['planning']['retreat_cm'],0])
            retreat['x_cm']+=float(delta[0]);retreat['y_cm']+=float(delta[1])
        if not (self.safe_pose(retreat) if wall else self.safe_floor_tools(retreat)):
            retreat=None;plan.warnings.append('Retreat endpoint not proven within arena; later routing must resolve withdrawal')
        segments.append(CoverageSegment('RETREAT',None,current,retreat,coverage_metadata={'route':None,'requires_collision_free_withdrawal':True}))
        segments.append(CoverageSegment('REQUEST_VERIFICATION',None,end_pose=retreat,coverage_metadata={'request':'later visual inspection; no cleanliness claim'}))
        plan.coverage_segments=[asdict(segment) for segment in segments]

    def _wall(self,plan,robot):
        if not isinstance(self.calibration,DualPlaneCalibration) or self.calibration.floor_boundary!='top': raise ValueError('Wall contact approach currently requires the calibrated top shared floor edge; native axes are not redefined')
        sponge,nozzle,body=self.h['wall_sponge'],self.h['wall_nozzle'],self.h['body']
        heading=0 if sponge['side']=='RIGHT' else 180;sign=1 if heading==0 else -1
        gap=sponge['lateral_protrusion_cm']-sponge['contact_compression_cm']
        if gap-self.margin<sponge['required_wall_clearance_cm']:
            self._unreachable(plan,'Sponge protrusion/compression cannot achieve contact while preserving required chassis-wall clearance and uncertainty');return
        y=body['chassis_width_cm']/2+gap
        low=body['chassis_length_cm']/2+self.margin;high=self.width-low
        if high<=low or not self.safe_pose(self.pose((low+high)/2,y,heading)):
            self._unreachable(plan,'Wall contact chassis footprint cannot fit the floor');return
        reverse=self.calibration.wall_x_reversed;ww=self.calibration.wall.width_cm;wh=self.calibration.wall.height_cm
        origin=self.calibration.wall_to_floor_base(0,0)[0]
        endpoint=self.calibration.wall_to_floor_base(ww,0)[0]
        def wallx(x): return (x-origin)/(endpoint-origin)*ww
        def floorx(x): return origin+(endpoint-origin)*x/ww
        offset=sign*sponge['offset_forward_cm'];cw=sponge['contact_width_cm']
        cx=sorted([wallx(low+offset-cw/2),wallx(high+offset+cw/2)])
        hmin,hmax=sponge['min_contact_height_cm'],sponge['max_contact_height_cm']
        cr=[[max(0,cx[0]),max(0,hmin),min(ww,cx[1]),min(wh,hmax)]] if min(wh,hmax)>max(0,hmin) else []
        distance=y+sign*nozzle['mount_offset_lateral_cm']
        sprayok=nozzle['safe_distance_min_cm']<=distance<=nozzle['safe_distance_max_cm']
        noffset=sign*(nozzle['mount_offset_forward_cm']+nozzle['impact_offset_forward_cm'])
        nx=sorted([wallx(low+noffset-nozzle['coverage_width_cm']/2),wallx(high+noffset+nozzle['coverage_width_cm']/2)])
        sr=[[max(0,nx[0]),max(0,nozzle['impact_min_height_cm']),min(ww,nx[1]),min(wh,nozzle['impact_max_height_cm'])]] if sprayok and min(wh,nozzle['impact_max_height_cm'])>max(0,nozzle['impact_min_height_cm']) else []
        cr=rectangle_set([r for r in cr if r[2]>r[0] and r[3]>r[1]]);sr=rectangle_set([r for r in sr if r[2]>r[0] and r[3]>r[1]])
        combined=rectangle_set(cr,sr,'intersection');self._metrics(plan,cr,sr,combined)
        total=plan.metrics['total_target_area_cm2']
        lower=covered_area(plan.target_geometry,[[0,0,ww,min(wh,hmin)]]) if hmin>0 else 0.
        upper=covered_area(plan.target_geometry,[[0,max(0,hmax),ww,wh]]) if hmax<wh else 0.
        plan.metrics.update(lower_unreachable_area_cm2=lower,upper_unreachable_area_cm2=upper)
        plan.wall_contact_model={'side':sponge['side'],'fixed_height':True,'min_contact_height_cm':hmin,'max_contact_height_cm':hmax,
            'chassis_wall_gap_cm':gap,'required_wall_clearance_cm':sponge['required_wall_clearance_cm'],
            'compression_cm':sponge['contact_compression_cm'],'contact_width_cm':cw,
            'nozzle':{'impact_height_band_cm':[nozzle['impact_min_height_cm'],nozzle['impact_max_height_cm']],
                      'lift_limits_cm':[nozzle['lift_min_cm'],nozzle['lift_max_cm']],
                      'rotation_limits_deg':[nozzle['rotation_min_deg'],nozzle['rotation_max_deg']],
                      'distance_cm':distance,'distance_valid':sprayok,'specific_lift_angle_solution':None,
                      'note':'Measured impact band is independent of lift height; settings/force limits require later calibration'}}
        if lower>1e-8: plan.reasons.append('Lower blind zone below fixed sponge minimum contact height')
        if upper>1e-8: plan.reasons.append('Upper blind zone above fixed sponge maximum contact height')
        if not sprayok: plan.reasons.append('Wall nozzle is outside its calibrated safe spray-distance interval')
        if plan.metrics['combined_cleanable_area_cm2']<=1e-8:
            plan.reasons.append('No target portion is both sprayable and wipeable; spray-only is not full cleaning');return
        # Stroke the actual outer-component X intervals of jointly reachable target.
        pieces=clipped_rings(plan.target_geometry,combined)
        intervals=intervals_union([(min(p[0] for p in r['points']),max(p[0] for p in r['points'])) for r in pieces if r['kind']=='outer'])
        wipe=[];wet=[]
        for x0,x1 in intervals:
            a,b=sorted([floorx(x0),floorx(x1)])
            for out,width,tooloffset,reach in ((wipe,cw,offset,cr),(wet,nozzle['coverage_width_cm'],noffset,sr)):
                start=float(np.clip(a-width/2-tooloffset-self.h['planning']['lead_in_cm'],low,high))
                end=float(np.clip(b+width/2-tooloffset+self.h['planning']['lead_in_cm'],low,high))
                if sign<0: start,end=end,start
                if abs(end-start)<1e-8: continue
                p,q=self.pose(start,y,heading),self.pose(end,y,heading)
                xbounds=sorted([wallx(start+tooloffset-width/2),wallx(start+tooloffset+width/2),wallx(end+tooloffset-width/2),wallx(end+tooloffset+width/2)])
                heightband=[hmin,hmax] if out is wipe else [nozzle['impact_min_height_cm'],nozzle['impact_max_height_cm']]
                out.append({'start':p,'end':q,'swept_rectangle_cm':[max(0,xbounds[0]),max(0,heightband[0]),min(ww,xbounds[-1]),min(wh,heightband[1])]})
        if len(wipe)+len(wet)>self.h['planning']['max_strokes']: raise ValueError('Wall stroke count exceeds configured planner limit')
        # Recheck against actual generated strokes, not merely theoretical band reach.
        self._metrics(plan,rectangle_set([p['swept_rectangle_cm'] for p in wipe]),rectangle_set([p['swept_rectangle_cm'] for p in wet]),
            rectangle_set(rectangle_set([p['swept_rectangle_cm'] for p in wipe]),rectangle_set([p['swept_rectangle_cm'] for p in wet]),'intersection'))
        self._segments(plan,wipe,wet,robot,plan.wall_contact_model,wall=True)
