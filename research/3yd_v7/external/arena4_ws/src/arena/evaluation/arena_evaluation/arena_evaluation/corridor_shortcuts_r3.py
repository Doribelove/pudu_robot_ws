"""Bounded swept-safe chord additions to initial and sole expanded corridors.

These augment the original corridor; they are not executable SE(2) paths.
Original endpoint connectors and topology route stay present. Smac and the
canonical audit remain responsible for all joins and the final trajectory.
"""
import math
from .reachable_endpoint_r3 import LocalConnector,ConnectorConfig,digest


def certified_chords(ctx, points, footprint, *, max_attempts=96, max_accepted=24):
    if max_attempts<0 or max_accepted<0:raise ValueError('nonnegative bounds required')
    if len(points)<3 or not max_attempts or not max_accepted:return [],0
    arc=[0.]
    for a,b in zip(points,points[1:]):arc.append(arc[-1]+math.dist(a,b))
    anchors=[0]
    for i in range(1,len(points)-1):
        if arc[i]-arc[anchors[-1]]>=5.:anchors.append(i)
    anchors.append(len(points)-1)
    options=[]
    for n,i in enumerate(anchors):
        for j in anchors[n+1:]:
            along=arc[j]-arc[i]
            if along>30.:break
            length=math.dist(points[i],points[j]);gain=along-length
            if length<1. or gain<1. or along<1.1*length:continue
            options.append((-gain,i,j,length))
    # Full Jackal with an additional .05m envelope, unknown remains collision.
    padding=.05
    x0=min(v[0] for v in footprint)-padding;x1=max(v[0] for v in footprint)+padding
    y0=min(v[1] for v in footprint)-padding;y1=max(v[1] for v in footprint)+padding
    envelope=((x0,y0),(x0,y1),(x1,y1),(x1,y0))
    checker=LocalConnector(ctx.hospital_map,envelope,ConnectorConfig())
    accepted=[];attempts=0;intervals=[]
    for negative_gain,i,j,length in sorted(options):
        if attempts>=max_attempts or len(accepted)>=max_accepted:break
        if any(i>=a and j<=b for a,b in intervals):continue
        yaw=math.atan2(points[j][1]-points[i][1],points[j][0]-points[i][0])
        poses=[(*points[i][:2],yaw),(*points[j][:2],yaw)]
        attempts+=1
        if not checker.safe(poses):continue
        cert={'algorithm':'swept-corridor-chords-v2','map_hash':ctx.map_sha256,
              'yaml_hash':getattr(ctx,'map_yaml_sha256',''),
              'resolution':ctx.hospital_map.resolution,'origin':ctx.hospital_map.origin,
              'footprint':footprint,'checked_envelope':envelope,
              'indices':[i,j],'poses':poses,'length_m':length,'removed_detour_m':-negative_gain,
              'scope':'constant-heading forward chord only; joins require Smac and PathAudit',
              'max_attempts':max_attempts,'max_accepted':max_accepted,'max_route_span_m':30.}
        cert['hash']=digest(cert);accepted.append(cert);intervals.append((i,j))
    return accepted,attempts
