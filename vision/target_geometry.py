"""Exact pixel-cell outlines and projective surface estimates (no robot control).

A foreground pixel covers [u-.5,u+.5] x [v-.5,v+.5]. Exposed cell
edges form oriented rings, including holes and disconnected islands. Mapping
these straight edges through a horizon-free homography preserves straightness;
shoelace area is exact for this raster footprint, not for the unknown real object.
"""
from collections import defaultdict
import math

import cv2
import numpy as np

from .serialization import json_line


def signed_area(points):
    p = np.asarray(points, dtype=float)
    # Translate to avoid cancellation for small objects at large coordinates.
    p = p-p[0]
    return float(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1))/2)


def image_geometry(mask, origin=(0,0)):
    component = np.asarray(mask)!=0
    edges = defaultdict(list)
    # Integer lattice vertices avoid floating-point joins. Diagonal-touching cells
    # are separate rings: take the rightmost available turn at their shared vertex.
    padded = np.pad(component,1)
    neighbors = (padded[:-2,1:-1],padded[1:-1,2:],padded[2:,1:-1],padded[1:-1,:-2])
    offsets = (((0,0),(1,0)),((1,0),(1,1)),((1,1),(0,1)),((0,1),(0,0)))
    for neighbor,(a,b) in zip(neighbors,offsets):
        ys,xs = np.nonzero(component & ~neighbor)
        for x,y in zip(xs.tolist(),ys.tolist()):
            edges[(x+a[0],y+a[1])].append((x+b[0],y+b[1]))
    rings=[]
    while edges:
        start = next(iter(edges))
        previous = start
        current = edges[start].pop()
        if not edges[start]: del edges[start]
        points=[start]
        while current!=start:
            points.append(current)
            candidates=edges[current]
            dx,dy=current[0]-previous[0],current[1]-previous[1]
            def turn(end):
                ex,ey=end[0]-current[0],end[1]-current[1]
                return (dx*ey-dy*ex,dx*ex+dy*ey)
            end=max(candidates,key=turn)
            candidates.remove(end)
            if not candidates: del edges[current]
            previous,current=current,end
        points=np.asarray(points,float)+np.asarray(origin)-.5
        rings.append({'kind':'outer' if signed_area(points)>0 else 'hole','points':points.tolist()})
    # Lossless mask persistence, separate from a display box or compact payload.
    runs=[]
    for y,row in enumerate(component):
        changes=np.flatnonzero(np.diff(np.r_[False,row,False].astype(np.int8)))
        runs.extend([y+origin[1],int(a)+origin[0],int(b)+origin[0]] for a,b in zip(changes[::2],changes[1::2]))
    return {'convention':'pixel_cell_edges_at_half_pixels; rings implicitly closed',
            'contours':rings,'mask_runs_y_xstart_xend_exclusive':runs}


def has_crossings(rings):
    """Reject new proper segment intersections (shared lattice vertices allowed)."""
    points=np.concatenate([np.asarray(r['points'],float) for r in rings])
    ends=np.concatenate([np.roll(np.asarray(r['points'],float),-1,axis=0) for r in rings])
    def cross(a,b): return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    for i,(a,b) in enumerate(zip(points,ends)):
        c,d=points[i+1:],ends[i+1:]
        if not len(c): continue
        x,y=cross(b-a,c-a),cross(b-a,d-a)
        z,w=cross(d-c,a-c),cross(d-c,b-c)
        if np.any((x*y < -1e-16)&(z*w < -1e-16)): return True
    return False


def physical_geometry(image, backward, mapper, *, surface, width, height, boundary_tolerance,
                      tolerance_cm=.2, max_points=512, max_bytes=32768):
    transformed=[]
    min_edge=math.inf
    for ring in image['contours']:
        pixels=np.asarray(ring['points'],float)
        homogeneous=np.column_stack((pixels,np.ones(len(pixels)))) @ backward.T
        den=homogeneous[:,2]
        if not ((den>1e-9).all() or (den < -1e-9).all()):
            raise ValueError('Contour crosses registration horizon')
        reference=homogeneous[:,:2]/den[:,None]
        # Use the actual Module 01 public API for every boundary vertex.
        points=np.asarray([mapper(*point) for point in reference],float)
        if not np.isfinite(points).all(): raise ValueError('Non-finite physical contour')
        if (points[:,0].min() < -boundary_tolerance or points[:,0].max()>width+boundary_tolerance or
                points[:,1].min() < -boundary_tolerance or points[:,1].max()>height+boundary_tolerance):
            raise ValueError('Full target contour extrapolates outside calibrated surface')
        lengths=np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)
        min_edge=min(min_edge,float(lengths.min()))
        transformed.append({'kind':ring['kind'],'points':points.tolist()})
    all_points=np.concatenate([r['points'] for r in transformed])
    low,high=all_points.min(axis=0),all_points.max(axis=0)
    axis='y' if surface=='FLOOR' else 'h'
    area=sum((1 if r['kind']=='outer' else -1)*abs(signed_area(r['points'])) for r in transformed)
    if not math.isfinite(area) or area<=0 or min_edge<=0:
        raise ValueError('Collapsed or invalid physical footprint')
    # Bound simplification below a quarter of the smallest projected pixel edge.
    # This conservative cap protects one-pixel strips; no extra tolerance is
    # introduced to satisfy payload budgets. Collinear runs still shrink well.
    effective=min(tolerance_cm,min_edge/4)
    compact=[]
    for ring in transformed:
        points=np.asarray(ring['points'],float)
        # Float32 approxPolyDP selects vertices; emit original float64 values.
        approximate=cv2.approxPolyDP(points.astype(np.float32),effective,True)[:,0]
        indices=[int(np.argmin(np.sum((points-p)**2,axis=1))) for p in approximate]
        candidate=points[indices]
        old,new=signed_area(points),signed_area(candidate)
        if len(candidate)<3 or old*new<=0 or abs(new-old)>abs(old)*.01:
            candidate=points
        compact.append({'kind':ring['kind'],'points':candidate.tolist()})
    if sum(len(r['points']) for r in compact)<=max_points and has_crossings(compact):
        compact=transformed
    geometry={'bounds':{'min_x':float(low[0]),'max_x':float(high[0]),f'min_{axis}':float(low[1]),f'max_{axis}':float(high[1])},
        'width_cm':float(high[0]-low[0]),'depth_cm' if surface=='FLOOR' else 'height_cm':float(high[1]-low[1]),
        'area_cm2':area,'area_method':'projected_union_of_pixel_cells_with_holes',
        'contours':compact,'contour':compact[0]['points'] if len(compact)==1 and compact[0]['kind']=='outer' else None,
        'simplification_tolerance_cm':tolerance_cm,'effective_tolerance_cm':effective,
        'representation':'compact_estimated_rings','payload_status':'COMPLETE'}
    if sum(len(r['points']) for r in compact)>max_points or len(json_line(geometry).encode())+64>max_bytes:
        geometry.update(contours=None,contour=None,payload_status='CONTOURS_OMITTED_LIMIT',
                        representation='summary_only_full_geometry_in_localization_record')
    return geometry,transformed
