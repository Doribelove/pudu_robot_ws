"""Audit selected-route L1 joins and full 0.30 m L2 targets."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def turn_angles(points):
    points=np.asarray(points,float);out=[]
    vectors=np.diff(points,axis=0)
    for a,b in zip(vectors,vectors[1:]):
        if np.linalg.norm(a)>1e-9 and np.linalg.norm(b)>1e-9:
            out.append(float(np.degrees(np.arccos(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1.,1.)))))
    return out


def self_intersections(points):
    points=np.asarray(points,float);a=points[:-1];b=points[1:];count=0
    for i in range(len(a)-2):
        c=a[i+2:];d=b[i+2:]
        bbox=(np.maximum(np.minimum(a[i],b[i]),np.minimum(c,d))<=
              np.minimum(np.maximum(a[i],b[i]),np.maximum(c,d))+1e-9).all(axis=1)
        if not np.any(bbox):continue
        c=c[bbox];d=d[bbox]
        side1=np.cross(b[i]-a[i],c-a[i]);side2=np.cross(b[i]-a[i],d-a[i])
        side3=np.cross(d-c,a[i]-c);side4=np.cross(d-c,b[i]-c)
        count+=int(np.count_nonzero((side1*side2<-1e-9)&(side3*side4<-1e-9)))
    return count


def main():
    root=ROOT/'results/r8_formal_10x3/cases';rows=[]
    for index in range(1,11):
        query=f'Q{index:02d}';case=root/(query+'_1')
        l1=json.loads((case/'l1_output.json').read_text())
        l2=json.loads((case/'l2_soft_target.json').read_text())
        p1=np.asarray(l1['right_reference_polyline'],float);p2=np.asarray(l2['points'],float)
        offsets=np.asarray([x['applied_offset_m'] for x in l2['records']],float)
        rows.append({'query_id':query,'l1_points':len(p1),'selected_junction_joins':len(l1['junction_joins']),
            'accepted_junction_joins':sum(bool(x['accepted']) for x in l1['junction_joins']),
            'l1_max_turn_deg':max(turn_angles(p1),default=0.),'l1_self_intersections':self_intersections(p1),
            'l2_soft_points':len(p2),'l2_min_offset_m':float(offsets.min()),'l2_max_offset_m':float(offsets.max()),
            'l2_max_segment_m':float(np.linalg.norm(np.diff(p2,axis=0),axis=1).max()),
            'l2_self_intersections':self_intersections(p2)})
    q6_r6=json.loads(Path('/home/robot/workspaces/3yd_v5_r6_20260921/results/r6_optimized_10x3/cases/Q06_1/l1_output.json').read_text())
    q6_r8=json.loads((root/'Q06_1/l1_output.json').read_text())
    result={'version':'3YD-V5-r8','queries':rows,
        'summary':{'all_joins_accepted':all(x['selected_junction_joins']==x['accepted_junction_joins'] for x in rows),
                   'all_offsets_exact_030':all(abs(x['l2_min_offset_m']-.3)<1e-8 and abs(x['l2_max_offset_m']-.3)<1e-8 for x in rows),
                   'no_l1_self_intersection':all(x['l1_self_intersections']==0 for x in rows),
                   'no_l2_self_intersection':all(x['l2_self_intersections']==0 for x in rows),
                   'q06_r6_points':len(q6_r6['right_reference_polyline']),
                   'q06_r8_points':len(q6_r8['right_reference_polyline']),
                   'q06_r6_max_turn_deg':max(turn_angles(q6_r6['right_reference_polyline'])),
                   'q06_r8_max_turn_deg':max(turn_angles(q6_r8['right_reference_polyline']))}}
    out=ROOT/'report/geometry_audit.json';out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result['summary'],ensure_ascii=False,indent=2))


if __name__=='__main__':main()
