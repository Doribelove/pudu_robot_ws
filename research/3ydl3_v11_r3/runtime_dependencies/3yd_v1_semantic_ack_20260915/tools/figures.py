"""Data-derived evidence images. No text is rendered: SimSun is unavailable."""
import argparse,json,os
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from arena_3d_v1.semantic_world import SemanticWorld,WORK,json_write

GREEN='#087f5b';BLUE='#1971c2';PURPLE='#7048a5';ORANGE='#e67700'

def blank(ax):ax.set_axis_off();ax.set_aspect('equal')
def curve(ax,path,color=GREEN,width=2.0):
 p=np.load(path);ax.plot(p[:,0],p[:,1],color=color,lw=width,solid_capstyle='round');ax.scatter(*p[0,:2],color='black',s=13,zorder=9);ax.scatter(*p[-1,:2],color='black',s=15,marker='D',zorder=9)
 return p

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--real',required=True);parser.add_argument('--controlled',required=True);parser.add_argument('--dynamic',required=True);parser.add_argument('--output',required=True);args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True);os.sched_setaffinity(0,{14})
 controls=list(map(json.loads,(Path(args.controlled)/'runs.jsonl').read_text().splitlines()));sources={}
 cal=[r for r in controls if r['scene']=='calibration' and r['repetition']==1]
 fig,axes=plt.subplots(3,1,figsize=(12,5.2),gridspec_kw={'hspace':.12})
 for ax,arm in zip(axes,['A','B','C']):
  row=next(r for r in cal if r['arm']==arm and r['side']=='right');case=Path(row['case'])
  ax.add_patch(Rectangle((0,0),40,6,facecolor='#f8f9fa',edgecolor='#343a40',lw=1.5))
  ax.add_patch(Rectangle((0,1.125),40,1.25,facecolor='#fff3bf',edgecolor='none'))
  cells=np.load(case/'l2.npy');xy=np.column_stack([-1+(cells[:,1]+.5)*.05,-1+(160-cells[:,0]-.5)*.05]);ax.plot(xy[:,0],xy[:,1],color=BLUE,lw=1.2,ls='--')
  curve(ax,case/'path.npy');ax.set_xlim(-.5,40.5);ax.set_ylim(-.2,6.2);blank(ax)
  sources[arm]=str(case)
 fig.savefig(out/'controlled_abc.png',dpi=160,bbox_inches='tight',pad_inches=.08);plt.close(fig)
 fig,axes=plt.subplots(1,2,figsize=(11,5.5),gridspec_kw={'width_ratios':[5,1]})
 for ax,scene in zip(axes,['calibration','heldout']):
  width,height=(40,6) if scene=='calibration' else (8,48)
  ax.add_patch(Rectangle((0,0),width,height,facecolor='#f8f9fa',edgecolor='#343a40',lw=1.5))
  rows=[r for r in controls if r['scene']==scene and r['repetition']==1 and r['arm']=='C']
  for side,color in [('right',GREEN),('left',PURPLE)]:
   row=next(r for r in rows if r['side']==side);curve(ax,Path(row['case'])/'path.npy',color,2.1)
  ax.set_xlim(-.5,width+.5);ax.set_ylim(-.5,height+.5);blank(ax)
 fig.tight_layout();fig.savefig(out/'controlled_mirror.png',dpi=160,bbox_inches='tight',pad_inches=.08);plt.close(fig)
 cfg=yaml.safe_load((WORK/'config/acceptance.yaml').read_text());world=SemanticWorld(cfg['inputs']['map_yaml'],cfg['inputs']['semantic_path'],cache_root=WORK/'cache');m=world.map
 root=Path(args.real);rows=list(map(json.loads,(root/'runs.jsonl').read_text().splitlines()));names=['cmp2-04-multi-junction','cmp2-06-lane-to-parking','cmp2-08-parking-internal']
 extent=[m.origin[0],m.origin[0]+m.width*.05,m.origin[1],m.origin[1]+m.height*.05]
 # Selected route over the real semantic source; interior nodes and interfaces
 # are taken from the hash-verified reconstructed route, not drawn by hand.
 name=names[1];data=json.loads((root/'derived'/name/'l1.json').read_text());l1=np.load(root/'derived'/name/'l1_poses.npy')
 rgb=np.ones((*world.hard.shape,3),np.float32)*.96
 for k,f in world.features.items():
  color={'lane':(.77,.87,.96),'junction_area':(.98,.84,.67),'parking_area':(.89,.89,.80)}.get(f.semantic_class)
  if color:rgb[world.labels==k]=color
 rgb[world.hard]=(.15,.17,.19)
 fig,ax=plt.subplots(figsize=(7,8));ax.imshow(rgb,extent=extent,origin='upper');ax.plot(l1[:,0],l1[:,1],color=ORANGE,lw=2)
 for n in data['nodes']:
  x,y,t=n['pose'];ax.arrow(x,y,.6*np.cos(t),.6*np.sin(t),width=.04,head_width=.22,color=BLUE if n['from_region']==n['to_region'] else PURPLE,zorder=9)
 ax.set_xlim(l1[:,0].min()-2,l1[:,0].max()+2);ax.set_ylim(l1[:,1].min()-2,l1[:,1].max()+2);blank(ax);fig.tight_layout(pad=0);fig.savefig(out/'semantic_pose_graph.png',dpi=160,bbox_inches='tight',pad_inches=.08);plt.close(fig)
 fig,axes=plt.subplots(1,3,figsize=(13,6))
 for ax,name in zip(axes,names):
  row=next(r for r in rows if r['query_id']==name and r['arm']=='C' and r['repetition']==1 and r['success']);case=Path(row['outcome_path']).parent
  ax.imshow(~world.hard,extent=extent,cmap='gray',origin='upper');rc=np.load(case/'l2.npy');xy=world.world_points(rc[:,0],rc[:,1]);ax.plot(xy[:,0],xy[:,1],color=BLUE,lw=1.0,ls='--');p=curve(ax,case/'path.npy',GREEN,1.9)
  ax.set_xlim(p[:,0].min()-2,p[:,0].max()+2);ax.set_ylim(p[:,1].min()-2,p[:,1].max()+2);blank(ax)
 fig.tight_layout(pad=.4);fig.savefig(out/'real_routes.png',dpi=160,bbox_inches='tight',pad_inches=.08);plt.close(fig)
 dyn=Path(args.dynamic);initial=np.load(dyn/'initial/hard.npy');fig,axes=plt.subplots(3,1,figsize=(12,5.2),gridspec_kw={'hspace':.12})
 for ax,name in zip(axes,['initial','obstacle_added','obstacle_removed']):
  hard=np.load(dyn/name/'hard.npy');rgb=np.ones((*hard.shape,3))*.97;rgb[initial]=(.77,.79,.81);rgb[hard&~initial]=(.85,.2,.18)
  ax.imshow(rgb,extent=[-1,41,-1,7],origin='upper');curve(ax,dyn/name/'path.npy');ax.set_xlim(3,37);ax.set_ylim(.3,5.7);blank(ax)
 fig.savefig(out/'dynamic_recovery.png',dpi=160,bbox_inches='tight',pad_inches=.08);plt.close(fig)
 json_write(out/'figure_provenance.json',{'font_policy':'no image text; SimSun unavailable, no substitution used','real_run':args.real,'controlled_run':args.controlled,'dynamic_run':args.dynamic,'control_first_repeat_cases':sources,'real_query_ids':names,'semantic_route_signature':data['route_signature'],'images':['controlled_abc.png','controlled_mirror.png','semantic_pose_graph.png','real_routes.png','dynamic_recovery.png']})
if __name__=='__main__':main()
