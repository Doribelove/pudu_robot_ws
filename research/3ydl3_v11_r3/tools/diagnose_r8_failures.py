"""Offline failure evidence; never used as navigation performance/success."""
import json, math, os, time
from pathlib import Path
import numpy as np
from replay_curve import load

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/offline_failure_audit'

def plot():
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt,font_manager
    from matplotlib.patches import Polygon
    font='/home/robot/文档/ChatGPT/dddd-任务/output/3yd_v7_l1_workflow_20260930/fonts/simsun.ttc'
    font_manager.fontManager.addfont(font)
    plt.rcParams['font.family']=font_manager.FontProperties(fname=font).get_name()
    plt.rcParams['axes.unicode_minus']=False
    fig,axes=plt.subplots(2,2,figsize=(13,11),constrained_layout=True)
    cases=[('pudu_wanda_3f','Q09','start','万达2w Q09 起点'),('pudu_wanda_3f','Q09','goal','万达2w Q09 终点'),('mentor_map_20260825_005','A2B-07','goal','导师原图 A2B-07 终点'),('arena_medium_200x200_2d_005','A2B-04','goal','Arena中图 A2B-04 终点')]
    for ax,(mid,qid,side,title) in zip(axes.flat,cases):
        folder=ROOT/'results/diagnostic_initial'/mid/'curve_inputs'
        w,d,m=load(folder/(qid+'_000.npz'));pose=d[side];x,y=pose[:2];res=w.map.resolution;ox,oy=w.map.origin[:2];h,wd=w.hard.shape
        rad=4.;c0=max(0,int((x-rad-ox)/res));c1=min(wd,int((x+rad-ox)/res));r0=max(0,h-int((y+rad-oy)/res));r1=min(h,h-int((y-rad-oy)/res))
        colors=np.ones((r1-r0,c1-c0,3));colors[~d['mask'][r0:r1,c0:c1]]=[.78,.86,1];colors[~d['physical'][r0:r1,c0:c1]]=[.9,.8,.8];colors[w.hard[r0:r1,c0:c1]]=[.25,.25,.25]
        ax.imshow(colors,extent=[ox+c0*res,ox+c1*res,oy+(h-r1)*res,oy+(h-r0)*res])
        ax.plot(d['guide'][:,0],d['guide'][:,1],color='orange',label='L2软目标')
        if mid=='pudu_wanda_3f':
            td=np.load(sorted(folder.glob('*traffic*.npz'))[0]);p=td['poses'];bad=td['bad'];ax.plot(p[:,0],p[:,1],color='green',lw=.8,label='被拒绝曲线');ax.scatter(p[bad,0],p[bad,1],s=3,color='red',label='方向违规采样')
            for rid in np.unique(d['labels'][r0:r1,c0:c1]):
                rr,cc=np.where(d['labels'][r0:r1,c0:c1]==rid)
                if len(rr)>80:ax.text(ox+(c0+np.median(cc)) *res,oy+(h-r0-np.median(rr))*res,str(rid),fontsize=12,color='navy')
        half_l,half_w=w.vehicle[:2];corners=np.array([[half_l,half_w],[half_l,-half_w],[-half_l,-half_w],[-half_l,half_w]])
        c,s=math.cos(pose[2]),math.sin(pose[2]);xy=corners@np.array([[c,s],[-s,c]])+pose[:2];ax.add_patch(Polygon(xy,fill=False,color='red'))
        ax.arrow(x,y,.65*c,.65*s,color='red',width=.025)
        ax.set(xlim=(x-rad,x+rad),ylim=(y-rad,y+rad),title=title,xlabel='x / m',ylabel='y / m',aspect='equal');ax.legend(loc='lower left')
    fig.suptitle('诊断子集：灰色障碍，蓝色局部范围外，浅红色物理许可范围外；非导航成功结果')
    fig.savefig(ROOT/'report/failure_locations.png',dpi=150)

def certificates():
    from three_yd_v5 import curve_l3
    original=curve_l3.certify;records=[];ctx={}
    def observed(*args,**kwargs):
        try:return original(*args,**kwargs)
        except curve_l3.CurveFailure as e:
            tb=e.__traceback__
            while tb and tb.tb_frame.f_code.co_name!='certify':tb=tb.tb_next
            if tb:
                v=tb.tb_frame.f_locals;rec={'case':ctx['case'],'failure':str(e)}
                bad=v.get('bad',np.array([],bool));idx=np.flatnonzero(bad)
                rec['bad_count']=len(idx)
                if len(idx):
                    sel=idx[:12];u=(v['left'][sel]+v['right'][sel])/2
                    rec['intervals']=[{'u':float(t),'xy':args[0](t).tolist(),'width':float(v['h'][i]),'speed_lower':float(v['minimum'][i]),'curvature_bound':float(v['curvature'][i]),'error_bound':float(v['error'][i])} for i,t in zip(sel,u)]
                    rec['actual_curvature_max']=float(np.max(v.get('mk',[0.])))
                    rec['minimum_derivative_bound']=float(v['minimum'][idx].min())
                records.append(rec)
                np.savez(OUT/f"certificate_{len(records):03d}.npz",knots=args[0].t,coefficients=args[0].c)
            raise
    curve_l3.certify=observed
    for mid,q in [('mentor_map_20260825_005','A2B-07'),('arena_medium_200x200_2d_005','A2B-04')]:
        path=ROOT/'results/diagnostic_initial'/mid/'curve_inputs'/(q+'_000.npz')
        w,d,m=load(path);ctx['case']=mid+'/'+q
        result=curve_l3.optimize(w,d['mask'],d['guide'],d['start'],d['goal'],time.monotonic()+15,m['config'])
        print(ctx['case'],result[2].get('failure'),flush=True)
    (OUT/'certificates.json').write_text(json.dumps(records,indent=2)+'\n')
    print('certificate failures captured',len(records),flush=True)

if __name__=='__main__':
    os.sched_setaffinity(0,{14});OUT.mkdir(exist_ok=False)
    plot();certificates()
