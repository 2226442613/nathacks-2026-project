"""Centimeter polygon areas and axis-aligned rectangle set operations.

Rectangular effective tool models and cardinal strokes permit continuous area
clipping instead of sampling target centers or treating bounds as target shape.
"""
import math
import cv2
import numpy as np

from .target_geometry import signed_area, has_crossings


def rotation(heading):
    angle=math.radians(heading)
    return np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])


def footprint(pose,length,width,offset=(0.,0.),margin=0.):
    local=np.array([[-length/2-margin,-width/2-margin],[length/2+margin,-width/2-margin],
                    [length/2+margin,width/2+margin],[-length/2-margin,width/2+margin]])+offset
    return local @ rotation(pose['heading_deg']).T+[pose['x_cm'],pose['y_cm']]


def contained(points,width,depth,tolerance=1e-7):
    p=np.asarray(points,float)
    return bool(np.isfinite(p).all() and (p>=-tolerance).all() and
                (p[:,0]<=width+tolerance).all() and (p[:,1]<=depth+tolerance).all())


def validate_rings(rings):
    if not isinstance(rings,list) or not rings: raise ValueError('Full physical contours required; bounds are insufficient')
    result=[]
    for r in rings:
        if r.get('kind') not in ('outer','hole'): raise ValueError('Contour kind must be outer/hole')
        p=np.asarray(r['points'],float)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all(): raise ValueError('Invalid/non-finite contour coordinates')
        if len(p)>1 and np.array_equal(p[0],p[-1]): p=p[:-1]
        # Dense cell-edge contours may repeat collinear points, but not collapse.
        if len(p)<3 or abs(signed_area(p))<=1e-10: raise ValueError('Collapsed physical contour')
        result.append({'kind':r['kind'],'points':p.tolist()})
    if sum(len(r['points']) for r in result)>30000: raise ValueError('Contour complexity exceeds planner limit')
    if has_crossings(result): raise ValueError('Self-intersecting or overlapping contour boundaries')
    outers=[np.float32(r['points']) for r in result if r['kind']=='outer']
    holes=[np.float32(r['points']) for r in result if r['kind']=='hole']
    if not outers: raise ValueError('No outer target contour')
    for hole in holes:
        if not any(cv2.pointPolygonTest(p,tuple(map(float,hole[0])),False)>=0 for p in outers): raise ValueError('Hole outside target outer contours')
    # Nested positive rings would double count. Module 03 islands inside holes
    # are valid, so reject nesting only when no intervening hole exists.
    for i,p in enumerate(outers):
        for j,q in enumerate(outers):
            if i!=j and cv2.pointPolygonTest(q,tuple(map(float,p[0])),False)>0 and not any(cv2.pointPolygonTest(h,tuple(map(float,p[0])),False)>0 for h in holes):
                raise ValueError('Overlapping/nested outer contours without a hole')
    if area(result)<=0: raise ValueError('Nonpositive target area')
    return result


def area(rings):
    return max(0.,sum((1 if r['kind']=='outer' else -1)*abs(signed_area(r['points'])) for r in rings))


def bounds(rings):
    points=np.concatenate([r['points'] for r in rings]);low,high=points.min(axis=0),points.max(axis=0)
    return [float(low[0]),float(low[1]),float(high[0]),float(high[1])]


def clip_polygon(points,rect):
    p=[np.asarray(v,float) for v in points]
    for axis,value,lower in ((0,rect[0],True),(0,rect[2],False),(1,rect[1],True),(1,rect[3],False)):
        out=[]
        if not p: break
        a=p[-1];inside_a=(a[axis]>=value if lower else a[axis]<=value)
        for b in p:
            inside_b=(b[axis]>=value if lower else b[axis]<=value)
            if inside_a!=inside_b:
                ratio=(value-a[axis])/(b[axis]-a[axis])
                point=a+ratio*(b-a);point[axis]=value;out.append(point)
            if inside_b: out.append(b)
            a,inside_a=b,inside_b
        p=out
    return np.asarray(p,float).reshape(-1,2)


def clipped_rings(rings,rectangles):
    result=[]
    for rect in rectangles:
        for r in rings:
            p=clip_polygon(r['points'],rect)
            if len(p)>=3 and abs(signed_area(p))>1e-10: result.append({'kind':r['kind'],'points':p.tolist()})
    return result


def covered_area(rings,rectangles):
    return area(clipped_rings(rings,rectangles))


def intervals_union(intervals):
    result=[]
    for a,b in sorted((float(a),float(b)) for a,b in intervals if b>a+1e-10):
        if result and a<=result[-1][1]+1e-9: result[-1][1]=max(result[-1][1],b)
        else: result.append([a,b])
    return result


def rectangle_set(a,b=None,operation='union'):
    """Disjoint vertical-slab rectangles for union/intersection/difference."""
    b=b or []
    rectangles=a+b
    if not rectangles: return []
    xs=sorted(set(v for r in rectangles for v in (r[0],r[2])))
    out=[]
    for x0,x1 in zip(xs,xs[1:]):
        if x1-x0<1e-10: continue
        x=(x0+x1)/2
        aa=intervals_union([(r[1],r[3]) for r in a if r[0]<=x<=r[2]])
        bb=intervals_union([(r[1],r[3]) for r in b if r[0]<=x<=r[2]])
        if operation=='union': ranges=intervals_union(aa+bb)
        else:
            ys=sorted(set(v for interval in aa+bb for v in interval));ranges=[]
            for y0,y1 in zip(ys,ys[1:]):
                y=(y0+y1)/2
                ina=any(lo<=y<=hi for lo,hi in aa);inb=any(lo<=y<=hi for lo,hi in bb)
                if ina and (inb if operation=='intersection' else not inb): ranges.append([y0,y1])
            ranges=intervals_union(ranges)
        out.extend([[x0,y0,x1,y1] for y0,y1 in ranges])
    return out
