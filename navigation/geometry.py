"""Conservative rectangle sweeps against arena boundaries and supplied obstacles."""
import math
import cv2
import numpy as np
from vision.cleaning_geometry import footprint,contained
from .timing import angle_error,finite


def spill_bounds(target):
    box=(target.get('geometry_cm') or {}).get('bounds')
    if box:
        values=[box[k] for k in ('min_x','min_y','max_x','max_y')]
    else:
        rings=target.get('target_geometry') or (target.get('geometry_cm') or {}).get('contours',[])
        points=np.array([p for r in rings if r['kind']=='outer' for p in r['points']],float)
        if points.size==0:raise ValueError('Spill bounding geometry required; centroid is not an edge')
        values=[*points.min(axis=0),*points.max(axis=0)]
    x,y,X,Y=[finite(v,'Spill bound') for v in values]
    if x>=X or y>=Y:raise ValueError('Invalid spill bounds')
    return np.array([x,y]),np.array([X,Y])


def _spill_polygons(target):
    rings=target.get('target_geometry') or (target.get('geometry_cm') or {}).get('contours',[])
    polygons=[np.asarray(r['points'],float) for r in rings if r['kind']=='outer']
    if not polygons:
        low,high=spill_bounds(target)
        polygons=[np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])]
    if any(p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all() for p in polygons):
        raise ValueError('Invalid spill contour')
    return polygons


def _segment_distance(p,q,a,b):
    """Distance from one finite segment to each of the contour edges (vectorized)."""
    edge=b-a;v=q-p
    t=np.clip(np.sum((p-a)*edge,axis=1)/np.maximum(np.sum(edge*edge,axis=1),1e-20),0,1)
    u=np.clip(np.sum((q-a)*edge,axis=1)/np.maximum(np.sum(edge*edge,axis=1),1e-20),0,1)
    r=np.clip((a-p) @ v/np.dot(v,v),0,1);s=np.clip((b-p) @ v/np.dot(v,v),0,1)
    distance=np.minimum.reduce([np.linalg.norm(p-a-t[:,None]*edge,axis=1),np.linalg.norm(q-a-u[:,None]*edge,axis=1),
                               np.linalg.norm(a-p-r[:,None]*v,axis=1),np.linalg.norm(b-p-s[:,None]*v,axis=1)])
    cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
    denominator=cross(v,edge);valid=np.abs(denominator)>1e-12
    fraction=np.divide(cross(a-p,edge),denominator,out=np.zeros(len(a)),where=valid)
    along=np.divide(cross(a-p,v),denominator,out=np.zeros(len(a)),where=valid)
    distance[valid & (fraction>=0) & (fraction<=1) & (along>=0) & (along<=1)]=0.
    return float(distance.min())


def front_parking_candidates(target,arena,clearance_cm=5.,*,include_unsafe=False,from_pose=None):
    low,high=spill_bounds(target);middle=(low+high)/2;offset=arena.length/2+clearance_cm
    if (low<0).any() or high[0]>arena.width or high[1]>arena.depth:raise ValueError('Spill bounds outside floor')
    goals=[(low[0]-offset,middle[1],0),(high[0]+offset,middle[1],180),
           (middle[0],low[1]-offset,90),(middle[0],high[1]+offset,270)]
    if from_pose is not None:
        points=np.concatenate(_spill_polygons(target))
        for h in (from_pose['heading_deg']%360,math.degrees(math.atan2(middle[1]-from_pose['y_cm'],middle[0]-from_pose['x_cm']))%360):
            if any(abs(angle_error(h,g[2]))<1e-6 for g in goals):continue
            v=np.array([math.cos(math.radians(h)),math.sin(math.radians(h))])
            center=middle-((middle-points) @ v).max()*v-offset*v
            goals.append((*center,h))
    result=[]
    for x,y,h in goals:
        pose=dict(x_cm=float(x),y_cm=float(y),heading_deg=float(h))
        gap=front_bumper_metrics(target,pose,arena.length,arena.body_width)['clearance_cm']
        if gap>clearance_cm+1e-7:
            # A concave/diagonal edge may be farther than its bounding box. Find
            # the first actual 5cm contour clearance, never claim the box is the spill.
            v=np.array([math.cos(math.radians(h)),math.sin(math.radians(h))])
            distance=float(np.dot(middle-np.array([x,y]),v));left=0.;right=None
            for step in np.linspace(0,distance,33)[1:]:
                trial={**pose,'x_cm':float(x+step*v[0]),'y_cm':float(y+step*v[1])}
                if front_bumper_metrics(target,trial,arena.length,arena.body_width)['clearance_cm']<=clearance_cm:
                    right=float(step);break
                left=float(step)
            if right is None:continue
            for _ in range(30):
                step=(left+right)/2;trial={**pose,'x_cm':float(x+step*v[0]),'y_cm':float(y+step*v[1])}
                if front_bumper_metrics(target,trial,arena.length,arena.body_width)['clearance_cm']>clearance_cm:left=step
                else:right=step
            pose.update(x_cm=float(x+left*v[0]),y_cm=float(y+left*v[1]))
        if include_unsafe or arena.safe(pose):result.append(pose)
    return result


def front_bumper_metrics(target,pose,length,width):
    """Distance from the finite front bumper to the actual detected outer contour."""
    low,high=spill_bounds(target);center=np.array([pose['x_cm'],pose['y_cm']]);a=math.radians(pose['heading_deg'])
    forward=np.array([math.cos(a),math.sin(a)]);left=np.array([-forward[1],forward[0]])
    bumper=center+length/2*forward;p=bumper-width/2*left;q=bumper+width/2*left
    polygons=_spill_polygons(target);a=np.concatenate(polygons);b=np.concatenate([np.roll(r,-1,axis=0) for r in polygons])
    distance=_segment_distance(p,q,a,b)
    body=np.asarray(footprint(pose,length,width),float)
    if (any(cv2.pointPolygonTest(np.float32(r),tuple(map(float,point)),False)>=0 for r in polygons for point in body) or
        any(cv2.pointPolygonTest(np.float32(body),tuple(map(float,point)),False)>=0 for point in a) or
        any(_segment_distance(body[i],body[(i+1)%4],a,b)<1e-9 for i in range(4))):distance=0.
    delta=(low+high)/2-center
    facing=abs(angle_error(math.degrees(math.atan2(delta[1],delta[0])),pose['heading_deg']))
    return {'clearance_cm':distance,'facing_error_deg':facing}


class Arena:
    def __init__(self,width=121.6,depth=68.4,length=26.,body_width=15.2,margin=1.5,obstacles=(),tool=None,side_tool=None):
        self.width,self.depth,self.length,self.body_width,self.margin=width,depth,length,body_width,margin
        for value in (width,depth,length,body_width):
            if finite(value,'Arena/body size')<=0:raise ValueError('Positive geometry required')
        if finite(margin,'Clearance margin')<0:raise ValueError('Nonnegative clearance required')
        self.obstacles=[]
        for box in obstacles:
            if len(box)!=4 or any(not math.isfinite(float(v)) for v in box) or box[0]>=box[2] or box[1]>=box[3]:
                raise ValueError('Obstacle must be [xmin,ymin,xmax,ymax]')
            x,y,X,Y=box;self.obstacles.append(np.float32([[x,y],[X,y],[X,Y],[x,Y]]))
        self.tool=tool
        self.side_tool=side_tool

    def corners(self,pose,extra=0):
        result=[footprint(pose,self.length,self.body_width,margin=self.margin+extra)]
        if self.tool:
            t=self.tool
            result.append(footprint(pose,t['contact_length_cm'],t['contact_width_cm'],
                (t['offset_forward_cm'],t['offset_lateral_cm']),margin=self.margin+extra))
        if self.side_tool:
            e=self.side_tool
            result.append(footprint(pose,e['length_cm'],e['width_cm'],(e['offset_forward_cm'],e['offset_left_cm']),margin=extra))
        return result

    def safe(self,pose,extra=0):
        for polygon in self.corners(pose,extra):
            if not contained(polygon,self.width,self.depth):return False
            for obstacle in self.obstacles:
                if cv2.intersectConvexConvex(np.float32(polygon),obstacle)[0]>1e-7:return False
        return True

    def check_segment(self,start,end,*,turn_change=None):
        if turn_change is None:
            if abs(angle_error(start['heading_deg'],end['heading_deg']))>1e-6:raise ValueError('Translation cannot change heading')
            travel=math.hypot(end['x_cm']-start['x_cm'],end['y_cm']-start['y_cm'])
            steps=max(1,math.ceil(travel/.25))
            delta=0.;inflation=.25 if travel>1e-9 else 0.
        else:
            if math.hypot(end['x_cm']-start['x_cm'],end['y_cm']-start['y_cm'])>1e-6:raise ValueError('Rotation must keep center fixed')
            steps=max(1,math.ceil(abs(turn_change)))
            delta=turn_change
            radius=math.hypot(self.length/2+self.margin,self.body_width/2+self.margin)
            if self.tool:
                t=self.tool;radius=max(radius,math.hypot(abs(t['offset_forward_cm'])+t['contact_length_cm']/2+self.margin,
                                                      abs(t['offset_lateral_cm'])+t['contact_width_cm']/2+self.margin))
            if self.side_tool:
                e=self.side_tool;radius=max(radius,math.hypot(abs(e['offset_forward_cm'])+e['length_cm']/2,
                                                            abs(e['offset_left_cm'])+e['width_cm']/2))
            inflation=radius*math.sin(math.radians(abs(delta)/steps))  # Actual unsampled angular interval; zero for no turn.
        for i in range(steps+1):
            fraction=i/steps
            pose={'x_cm':start['x_cm']+(end['x_cm']-start['x_cm'])*fraction,
                  'y_cm':start['y_cm']+(end['y_cm']-start['y_cm'])*fraction,
                  'heading_deg':start['heading_deg']+delta*fraction}
            if not self.safe(pose,inflation):
                bounds=[dict(min_x=float(p[:,0].min()),max_x=float(p[:,0].max()),
                             min_y=float(p[:,1].min()),max_y=float(p[:,1].max())) for p in self.corners(pose,inflation)]
                raise ValueError('Chassis/tool swept area intersects wall, floor boundary or obstacle; '+
                    f'pose={pose}; footprint_bounds_cm={bounds}; floor_cm=[0, 0, {self.width}, {self.depth}]; extra_margin_cm={inflation}')
