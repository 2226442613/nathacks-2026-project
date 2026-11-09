"""Explicit hardware dimensions. null stays unknown; simulation never fills physical data."""
from copy import deepcopy
import hashlib
import math
from .serialization import json_line

SECTIONS={
    'body': ['chassis_length_cm','chassis_width_cm','safety_margin_cm','position_uncertainty_cm'],
    'floor_sponge':['offset_forward_cm','offset_lateral_cm','contact_length_cm','contact_width_cm'],
    'floor_nozzle':['mount_offset_forward_cm','mount_offset_lateral_cm','mount_height_cm','fixed_spray_azimuth_deg','fixed_spray_elevation_deg','offset_forward_cm','offset_lateral_cm','coverage_length_cm','coverage_width_cm','fixed_orientation_deg'],
    'wall_sponge':['offset_forward_cm','diameter_cm','contact_width_cm','min_contact_height_cm','max_contact_height_cm','lateral_protrusion_cm','required_wall_clearance_cm','contact_compression_cm'],
    'wall_nozzle':['mount_height_cm','mount_offset_forward_cm','mount_offset_lateral_cm','impact_offset_forward_cm','lift_min_cm','lift_max_cm','rotation_min_deg','rotation_max_deg','impact_min_height_cm','impact_max_height_cm','coverage_width_cm','safe_distance_min_cm','safe_distance_max_cm']}
POSITIVE={'chassis_length_cm','chassis_width_cm','contact_length_cm','contact_width_cm','coverage_length_cm','coverage_width_cm','diameter_cm'}
SIGNED={'offset_forward_cm','offset_lateral_cm','mount_offset_forward_cm','mount_offset_lateral_cm','impact_offset_forward_cm','rotation_min_deg','rotation_max_deg','fixed_spray_azimuth_deg','fixed_spray_elevation_deg'}


class CleaningHardware:
    def __init__(self,data):
        self.data=deepcopy(data)
        if data.get('schema_version')!=1 or data.get('geometry_source') not in ('SIMULATION','PHYSICAL'): raise ValueError('Explicit SIMULATION/PHYSICAL hardware source required')
        if not isinstance(data.get('measurements_validated'),bool): raise ValueError('measurements_validated must be boolean')
        for section,fields in SECTIONS.items():
            if not isinstance(data.get(section),dict): raise ValueError(f'Missing configuration section {section}')
            item=data[section]
            if any(item.get(key) is not None for key in ('footprint_polygon','contact_footprint_polygon','coverage_polygon_cm','footprint_polygon_cm')):
                raise ValueError('Custom tool/body polygons are unsupported in this rectangular model; configure a measured conservative effective rectangle')
            if section!='body' and item.get('installed') is not None and not isinstance(item['installed'],bool): raise ValueError('installed must be true/false/null')
            for key in fields:
                if key not in item: raise ValueError(f'Missing configuration field {section}.{key}')
                value=item[key]
                if value is None: continue
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value): raise ValueError(f'{section}.{key} must be finite numeric/null')
                if key in POSITIVE and value<=0: raise ValueError(f'{section}.{key} must be positive')
                if key not in POSITIVE|SIGNED and value<0: raise ValueError(f'{section}.{key} cannot be negative')
            for lo,hi in [('min_contact_height_cm','max_contact_height_cm'),('lift_min_cm','lift_max_cm'),('rotation_min_deg','rotation_max_deg'),('impact_min_height_cm','impact_max_height_cm'),('safe_distance_min_cm','safe_distance_max_cm')]:
                if item.get(lo) is not None and item.get(hi) is not None and item[lo]>item[hi]: raise ValueError(f'{section}: {lo} exceeds {hi}')
            if section=='floor_nozzle' and item['fixed_orientation_deg'] is not None and not 0<=item['fixed_orientation_deg']<360: raise ValueError('Fixed floor nozzle orientation must be in [0,360)')
            if section=='wall_nozzle':
                for key in ('rotation_min_deg','rotation_max_deg'):
                    if item[key] is not None and not -180<=item[key]<=180: raise ValueError('Wall nozzle rotation angles must be -180..180')
        for key,limit in (('fixed_spray_azimuth_deg',180),('fixed_spray_elevation_deg',90)):
            value=data['floor_nozzle'][key]
            if value is not None and not -limit<=value<=limit: raise ValueError('Fixed spray direction outside angular limits')
        if data['body'].get('reference_point')!='GROUND_CENTER' or data['body'].get('axes')!='FORWARD_LEFT': raise ValueError('Use Module 04 ground-center and forward/left axes')
        if data['wall_sponge'].get('side') not in ('LEFT','RIGHT',None): raise ValueError('Wall sponge side must be LEFT/RIGHT/null')
        for section in ('floor_nozzle','wall_nozzle'):
            if not isinstance(data[section].get('coverage_calibrated'),(bool,type(None))): raise ValueError('Coverage calibration must be true/false/null')
        if data['wall_nozzle'].get('orientation_convention') not in ('TOWARD_WALL',None): raise ValueError('Unknown wall-nozzle orientation convention')
        sponge=data['wall_sponge']
        if sponge['lateral_protrusion_cm'] is not None and sponge['contact_compression_cm'] is not None and sponge['contact_compression_cm']>sponge['lateral_protrusion_cm']: raise ValueError('Compression exceeds sponge protrusion')
        measured=sponge.get('measurements')
        if measured is not None:
            dimensions=measured['dimensions_cm']
            if not isinstance(dimensions,list) or len(dimensions)!=2:raise ValueError('Record both measured sponge dimensions without assigning axes')
            for key,value in [('dimension',v) for v in dimensions]+[(k,measured[k]) for k in ('front_edge_setback_cm','lowest_edge_height_cm')]:
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0 or (key=='dimension' and value==0):raise ValueError('Invalid measured sponge '+key)
            for key in ('rotation_about_center','effective_contact_verified','rotation_envelope_verified'):
                if type(measured[key]) is not bool:raise ValueError('Measured sponge flag must be boolean: '+key)
            if sponge['min_contact_height_cm'] is not None and sponge['min_contact_height_cm']<measured['lowest_edge_height_cm']:
                raise ValueError('Wall contact cannot be claimed below the measured lower edge')
        observation=data.get('nozzle_observation')
        if observation is not None:
            for key in ('height_cm','landing_ahead_cm'):
                value=observation[key]
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:raise ValueError('Invalid nozzle observation '+key)
        p=data.get('planning',{})
        if set(p)!={'pass_overlap_fraction','lead_in_cm','retreat_cm','max_strokes'}: raise ValueError('Explicit planning parameters required')
        for key,value in p.items():
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value): raise ValueError('Invalid planning parameter')
        if not 0<=p['pass_overlap_fraction']<.8 or p['lead_in_cm']<0 or p['retreat_cm']<0 or not isinstance(p['max_strokes'],int) or not 1<=p['max_strokes']<=512: raise ValueError('Planning parameter outside limits')
        self.identity='sha256:'+hashlib.sha256(json_line(self.data).encode()).hexdigest()

    @property
    def mode(self): return self.data['geometry_source']

    def side_collision_envelope(self):
        """Conservative STATIC exclusion box, never an effective contact patch.

        A measured panel diagonal bounds either unknown in-plane axis projection.
        The full chassis length is used only when it contains that static span.
        No panel rotation axis or rotating sweep is inferred.
        """
        sponge=self.data['wall_sponge'];measured=sponge.get('measurements')
        if sponge['installed'] is not True or not measured:return None
        length,width=self.data['body']['chassis_length_cm'],self.data['body']['chassis_width_cm']
        protrusion=sponge['lateral_protrusion_cm'];side=sponge['side']
        if any(v is None for v in (length,width,protrusion)) or side is None:return None
        span=math.hypot(*measured['dimensions_cm']);setback=measured['front_edge_setback_cm']
        if setback>length or span>length-setback or span>width+protrusion:
            raise ValueError('Measured static side panel requires a separately specified collision envelope')
        return {'length_cm':length,'width_cm':width+protrusion,'offset_forward_cm':0.,
                'offset_left_cm':protrusion/2*(1 if side=='LEFT' else -1),'scope':'STATIC_OBSERVED_PANEL_ONLY'}

    def missing(self,surface):
        tools=['floor_sponge','floor_nozzle'] if surface=='FLOOR' else ['wall_sponge','wall_nozzle']
        missing=[]
        if self.mode=='PHYSICAL' and not self.data['measurements_validated']: missing.append('measurements_validated')
        for name in ['body']+tools:
            item=self.data[name]
            for key in SECTIONS[name]:
                # Diameter is useful mounting metadata, but the independently
                # measured contact interval/width is authoritative, not diameter.
                if key=='diameter_cm' or (name=='floor_nozzle' and key in ('mount_offset_forward_cm','mount_offset_lateral_cm','mount_height_cm','fixed_spray_azimuth_deg','fixed_spray_elevation_deg')) or (name=='wall_nozzle' and key=='mount_height_cm'): continue
                if item[key] is None: missing.append(name+'.'+key)
            if name!='body' and item['installed'] is None: missing.append(name+'.installed')
        if surface=='WALL':
            if self.data['wall_sponge']['side'] is None: missing.append('wall_sponge.side')
            if self.data['wall_nozzle']['orientation_convention'] is None: missing.append('wall_nozzle.orientation_convention')
            measured=self.data['wall_sponge'].get('measurements')
            if measured:
                for key in ('effective_contact_verified','rotation_envelope_verified'):
                    if measured[key] is not True:missing.append('wall_sponge.measurements.'+key)
                if measured['rotation_axis'] is None:missing.append('wall_sponge.measurements.rotation_axis')
        for name in [n for n in tools if 'nozzle' in n]:
            if self.data[name]['coverage_calibrated'] is not True: missing.append(name+'.coverage_calibrated')
        return missing
