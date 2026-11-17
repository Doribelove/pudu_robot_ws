"""Payload-bounded sparse rectangles; unchanged full effective-master ACK."""
import numpy as np

def rectangles(changed,max_payload=128000):
    changed=np.asarray(changed,bool);rr,cc=np.nonzero(changed)
    if not len(rr):return [],{'changed_cells':0,'published_cells':0,'baseline_cells':0,'baseline_messages':0}
    def bounds(y,x):return int(y.min()),int(y.max())+1,int(x.min()),int(x.max())+1
    def strips(box):
        y0,y1,x0,x1=box;width=x1-x0;out=[]
        # Also handle a map wider than the payload cap.
        for x in range(x0,x1,max_payload):
            end=min(x1,x+max_payload);step=max(1,max_payload//(end-x))
            out.extend((y,min(y1,y+step),x,end) for y in range(y0,y1,step))
        return out
    def score(rs):
        # Fixed pacing is expensive: avoid replacing one large message by
        # dozens of tiny updates. Deterministic cost, not a timing heuristic.
        return len(rs)*.015+sum((b-a)*(d-c) for a,b,c,d in rs)/20_000_000.
    def split(y,x,depth=0):
        box=bounds(y,x);whole=strips(box)
        if depth>=7 or len(y)<64:return whole
        y0,y1,x0,x1=box
        if len(y)>.65*(y1-y0)*(x1-x0):return whole
        best_score=score(whole);candidates=[]
        for axis,mid in ((y,(y0+y1)//2),(x,(x0+x1)//2)):
            left=axis<mid
            if not np.any(left) or np.all(left):continue
            simple=strips(bounds(y[left],x[left]))+strips(bounds(y[~left],x[~left]))
            candidates.append((score(simple),left))
        if not candidates:return whole
        value,left=min(candidates,key=lambda item:item[0])
        if value>best_score*1.15:return whole
        parts=split(y[left],x[left],depth+1)+split(y[~left],x[~left],depth+1)
        return parts if score(parts)<best_score-1e-12 else whole
    baseline=strips(bounds(rr,cc));chosen=sorted(split(rr,cc))
    return chosen,{'changed_cells':len(rr),'published_cells':sum((b-a)*(d-c) for a,b,c,d in chosen),
        'baseline_cells':sum((b-a)*(d-c) for a,b,c,d in baseline),'baseline_messages':len(baseline),
        'message_count':len(chosen),'max_payload':max_payload}
