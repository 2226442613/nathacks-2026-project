"""Offline centimeter planning diagrams; visualization pixels never drive area metrics."""
import cv2
import numpy as np
from .cleaning_geometry import footprint,rotation


def _text(image,text,x,y,color=(235,235,235),scale=.47):
    cv2.putText(image,text,(x,y),0,scale,color,1,cv2.LINE_AA)


def render_cleaning_plan(plan,calibration,hardware):
    p=plan.to_dict() if hasattr(plan,'to_dict') else plan
    h=hardware.data if hasattr(hardware,'data') else hardware
    floor=calibration.floor if hasattr(calibration,'floor') else calibration
    image=np.full((800,1400,3),(28,28,28),np.uint8)
    title=f"{p['geometry_source']} | target {p['target_id']} | {p['surface']} | {p['reachability']} | {p['localization_status']} | EXECUTION: NONE"
    _text(image,title,20,28,(0,220,255),.62)
    if p['surface']=='WALL' and hasattr(calibration,'wall'):
        _surface(image,p,(30,75,800,465),calibration.wall.width_cm,calibration.wall.height_cm,wall=True)
        _floor_context(image,p,h,floor,(870,80,490,470))
    else:
        _surface(image,p,(30,75,900,550),floor.width_cm,floor.height_cm)
        _floor_context(image,p,h,floor,(30,75,900,550),draw_arena=False)
    metrics=p['metrics'];fraction=metrics.get('estimated_cleanable_fraction')
    summary='coverage UNKNOWN' if fraction is None else f"cleanable {fraction*100:.1f}% | target {metrics['total_target_area_cm2']:.2f} cm^2 | combined {metrics['combined_cleanable_area_cm2']:.2f} cm^2"
    _text(image,summary,25,710)
    _text(image,'Green: both | Blue: spray only | Amber: contact only | Red: neither | Gray: unknown | Holes preserved',25,732)
    for i,reason in enumerate(p['reasons'][:2]): _text(image,reason[:165],25,754+20*i,(100,170,255),.40)
    _text(image,'Geometric model only. Routes, force, water settings and actual cleaning effectiveness are NOT validated.',25,692,(150,190,255),.43)
    return image


def _transform(panel,width,height,wall=False):
    x,y,w,h=panel;scale=min((w-30)/width,(h-40)/height)
    def point(p): return [x+15+p[0]*scale,y+20+(height-p[1] if wall else p[1])*scale]
    return point,scale


def _surface(image,p,panel,width,height,wall=False):
    point,scale=_transform(panel,width,height,wall)
    def polygon(points): return np.rint([point(v) for v in points]).astype(np.int32)
    shape=np.zeros(image.shape[:2],np.int16)
    for r in p['target_geometry']:
        mask=np.zeros(image.shape[:2],np.uint8);cv2.fillPoly(mask,[polygon(r['points'])],1)
        shape+=mask.astype(np.int16)*(1 if r['kind']=='outer' else -1)
    target=shape>0
    def region_mask(rectangles):
        mask=np.zeros(image.shape[:2],np.uint8)
        for a,b,c,d in rectangles: cv2.fillPoly(mask,[polygon([[a,b],[c,b],[c,d],[a,d]])],255)
        return mask!=0
    contact=region_mask(p['contact_reachable_rectangles_cm'])
    spray=region_mask(p['spray_reachable_rectangles_cm'])
    image[target]=(100,100,100) if p['reachability']=='UNKNOWN' else (70,70,220)
    image[target&contact&~spray]=(30,160,220)
    image[target&spray&~contact]=(200,130,50)
    image[target&contact&spray]=(70,180,60)
    cv2.polylines(image,[polygon([[0,0],[width,0],[width,height],[0,height]])],True,(180,180,180),1)
    for r in p['target_geometry']: cv2.polylines(image,[polygon(r['points'])],True,(240,240,240),1)
    if wall:
        model=p['wall_contact_model']
        for name,color in (('min_contact_height_cm',(0,180,255)),('max_contact_height_cm',(0,180,255))):
            if name in model:
                y=model[name];cv2.line(image,tuple(np.rint(point([0,y])).astype(int)),tuple(np.rint(point([width,y])).astype(int)),color,1)
                at=point([0,y]);_text(image,f"sponge {name.split('_')[0]} H={y:g}",int(at[0])+3,int(at[1])-4,color,.40)
        if model:
            low,high=model['min_contact_height_cm'],model['max_contact_height_cm']
            for text,y in [('LOWER SPONGE BLIND ZONE',max(0,min(height,low))/2),('UPPER SPONGE BLIND ZONE',(max(0,min(height,high))+height)/2)]:
                at=point([0,y]);_text(image,text,int(at[0])+5,int(at[1]),(0,180,255),.38)
        band=model.get('nozzle',{}).get('impact_height_band_cm')
        if band:
            for y in band:
                a,b=point([0,y]),point([width,y]);cv2.line(image,tuple(np.rint(a).astype(int)),tuple(np.rint(b).astype(int)),(230,170,50),1)
            _text(image,f'Nozzle reachable H={band[0]:g}..{band[1]:g} cm (not lift positions)',panel[0],panel[1]+panel[3]+8,(230,170,50),.43)
        _text(image,'WALL X/H cm | H upward | below min / above max = sponge blind zones',panel[0],panel[1]-10)
    else:
        axes='+Y away from top wall' if p.get('coordinate_frame',{}).get('shared_floor_boundary')=='top' else 'native Module 01 +Y'
        _text(image,'FLOOR X/Y cm | '+axes+' (down this diagram)',panel[0],panel[1]-10)


def _floor_context(image,p,h,floor,panel,draw_arena=True):
    point,scale=_transform(panel,floor.width_cm,floor.height_cm)
    def polygon(points): return np.rint([point(v) for v in points]).astype(np.int32)
    if draw_arena:
        cv2.polylines(image,[polygon([[0,0],[floor.width_cm,0],[floor.width_cm,floor.height_cm],[0,floor.height_cm]])],True,(150,150,150),1)
        _text(image,'FLOOR: proposed approach / contained chassis',panel[0],panel[1]-10,(235,235,235),.44)
    approach=p['approach_pose'];body=h['body']
    if approach and body['chassis_length_cm'] is not None:
        cv2.polylines(image,[polygon(footprint(approach,body['chassis_length_cm'],body['chassis_width_cm']))],True,(255,255,255),2)
        at=point([approach['x_cm'],approach['y_cm']]);cv2.circle(image,tuple(np.rint(at).astype(int)),4,(0,255,255),-1)
        _text(image,'approach',int(at[0])+4,int(at[1])-5,(0,255,255),.40)
    for segment in p['coverage_segments']:
        if segment['segment_type'] not in ('SPRAY','CONTACT_CLEAN'): continue
        waypoints=segment['waypoints']
        if not waypoints: continue
        color=(230,140,70) if segment['segment_type']=='SPRAY' else (255,240,20)
        positions=[]
        for pose in waypoints:
            center=np.array([pose['x_cm'],pose['y_cm']])
            if p['surface']=='FLOOR':
                section=h['floor_sponge'] if segment['segment_type']=='CONTACT_CLEAN' else h['floor_nozzle']
                center+=rotation(pose['heading_deg']) @ [section['offset_forward_cm'],section['offset_lateral_cm']]
            positions.append(center)
        cv2.polylines(image,[polygon(positions)],False,color,2)
        start=waypoints[0]
        if p['surface']=='FLOOR':
            section=h['floor_sponge'] if segment['segment_type']=='CONTACT_CLEAN' else h['floor_nozzle']
            length=section.get('contact_length_cm',section.get('coverage_length_cm'))
            width=section.get('contact_width_cm',section.get('coverage_width_cm'))
            localpose=dict(start)
            if segment['segment_type']=='SPRAY':
                center=np.array([start['x_cm'],start['y_cm']])+rotation(start['heading_deg']) @ [section['offset_forward_cm'],section['offset_lateral_cm']]
                localpose.update(x_cm=center[0],y_cm=center[1],heading_deg=start['heading_deg']+section['fixed_orientation_deg'])
                offset=(0,0)
            else: offset=(section['offset_forward_cm'],section['offset_lateral_cm'])
            cv2.polylines(image,[polygon(footprint(localpose,length,width,offset))],True,color,1)
    _text(image,'Cyan: sponge strokes | Blue: wetting strokes | White: chassis',panel[0],panel[1]+panel[3]+18,(220,220,220),.40)
