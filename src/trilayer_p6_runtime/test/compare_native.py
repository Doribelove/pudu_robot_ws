#!/usr/bin/env python3
"""Valid-input numerical comparison against the frozen extension binaries."""
import argparse
import importlib.util
from pathlib import Path
import numpy as np

def module(prefix,name):
    paths=list(prefix.glob(name+'*.so'));assert len(paths)==1,(prefix,name)
    spec=importlib.util.spec_from_file_location(name,paths[0]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('installed',type=Path);p.add_argument('baseline',type=Path);a=p.parse_args()
    rng=np.random.default_rng(1306)
    old=module(a.baseline,'_nav2_effective_costmap');new=module(a.installed,'_nav2_effective_costmap')
    for i in range(20):
        source=rng.choice(np.array([0,0,0,0,254,255],np.uint8),size=(17,21))
        args=(source,21,17,.1,.5,3.,.18)
        assert new.inflate(*args)==old.inflate(*args)
    print('PASS 20 pinned effective-costmap byte comparisons')
    old=module(a.baseline,'_endpoint_geometry_r3');new=module(a.installed,'_endpoint_geometry_r3')
    grid=rng.choice(np.array([0,0,0,0,100,-1],np.int8),size=(30,32))
    footprint=[(-.1,-.08),(.1,-.08),(.1,.08),(-.1,.08)]
    ca=old.make_checker(grid,.1,0.,0.,footprint);cb=new.make_checker(grid,.1,0.,0.,footprint)
    for i in range(100):
        pose=(*rng.uniform(.1,2.8,size=2),rng.uniform(-3.14,3.14),.01)
        assert old.collision(ca,*pose)==new.collision(cb,*pose)
    print('PASS 100 endpoint collision predicate comparisons')
if __name__=='__main__':main()
