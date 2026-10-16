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

if __name__=='__main__':main()
