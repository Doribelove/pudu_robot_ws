"""Independent replay, finite output sampling and local diagnostic figures."""
import json
import hashlib
import numpy as np
from .bezier import evaluate


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as stream:
        for b in iter(lambda:stream.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def sample_trajectory(trajectory,spacing=0.0069):
    if trajectory is None:
        return []
    rows=[];offset=0.
    for i,(cp,dt) in enumerate(zip(trajectory['control_points'],trajectory['durations_s'])):
        cp=np.asarray(cp)
        # Control-polygon length bounds parameter speed; subdivisions guarantee
        # position spacing <= spacing. Curvature already has a continuous bound.
        q=3*np.diff(cp,axis=0)
        n=max(2,int(np.ceil(np.max(np.linalg.norm(q,axis=1))/spacing))+1)
        u=np.linspace(0,1,n);p,v,a=evaluate(cp,u)
        for j in range(n if i==len(trajectory['control_points'])-1 else n-1):
            speed=float(np.linalg.norm(v[j])/dt)
            k=float(np.cross(v[j],a[j])/np.linalg.norm(v[j])**3)
            rows.append({'x':float(p[j,0]),'y':float(p[j,1]),'yaw':float(np.arctan2(v[j,1],v[j,0])),
                         'time_s':float(offset+u[j]*dt),'speed_mps':speed,'curvature_per_m':k,
                         'source':'stl_joint_trajectory','motion_direction':'forward','segment':i})
        offset+=dt
    return rows


def plot_result(scene,result,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig,ax=plt.subplots(figsize=(9,7),constrained_layout=True)
    if scene.metadata.get('source_paths'):
        from PIL import Image
        import yaml
        paths=scene.metadata['source_paths'];cfg=yaml.safe_load(open(paths[0]))
        img=np.asarray(Image.open(paths[1]).convert('L'));h,w=img.shape
        ox,oy,_=cfg['origin'];res=cfg['resolution']
        ax.imshow(img,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+w*res,oy,oy+h*res],origin='upper')
        boxes=np.array([r.bounds for r in scene.regions])
        if len(boxes):
            ax.set_xlim(boxes[:,0].min()-1,boxes[:,2].max()+1)
            ax.set_ylim(boxes[:,1].min()-1,boxes[:,3].max()+1)
    selected=set(result.get('trajectory',{}).get('route',[])) if result.get('trajectory') else set()
    for i,r in enumerate(scene.regions):
        x,y,X,Y=r.bounds
        ax.add_patch(Rectangle((x,y),X-x,Y-y,facecolor='#cce6df' if i in selected else '#dbe5ed',
                               edgecolor='#6c889a',alpha=.5,lw=.6))
        if len(scene.regions)<25:ax.text((x+X)/2,(y+Y)/2,r.id,fontsize=8,ha='center')
    rows=sample_trajectory(result.get('trajectory'))
    if rows:
        ax.plot([r['x'] for r in rows],[r['y'] for r in rows],color='#007a62',lw=2,label='STL-V0 generated trajectory')
        t=result['trajectory'];kn=np.asarray(t['knots']);vel=np.asarray(t['knot_velocity'])
        ax.scatter(kn[:,0],kn[:,1],s=24,color='#007a62',zorder=4,label='Optimized transitions')
        ax.quiver(kn[:,0],kn[:,1],vel[:,0],vel[:,1],angles='xy',scale_units='xy',scale=.7,width=.004)
    for p,label,col in [(scene.start,'Start','#246ba6'),(scene.goal,'Goal','#c15335')]:
        ax.scatter(p[0],p[1],s=55,color=col,label=label,zorder=5)
        ax.arrow(p[0],p[1],.6*np.cos(p[2]),.6*np.sin(p[2]),head_width=.12,color=col,length_includes_head=True)
    ax.set_aspect('equal');ax.autoscale_view();ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
    ax.set_title(scene.name+' | '+result['status']);ax.legend(loc='best',fontsize=8)
    fig.savefig(path,dpi=150);plt.close(fig)
