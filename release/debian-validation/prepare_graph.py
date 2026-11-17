#!/usr/bin/python3
"""Read package XML; write a local-only graph/order/rosdep draft. No installs."""
from pathlib import Path
import argparse,json,xml.etree.ElementTree as ET
p=argparse.ArgumentParser();p.add_argument('source_root',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
packages={}
for f in sorted((a.source_root/'src').glob('*/package.xml')):
 x=ET.parse(f).getroot();name=x.findtext('name');assert name not in packages
 assert not any(el.get('condition') for el in x),'Conditional metadata requires explicit ROS2/Humble evaluation'
 packages[name]={'path':str(f.parent.relative_to(a.source_root)),'version':x.findtext('version'),'depends':sorted({el.text for el in x if el.tag.endswith('depend')})}
assert len(packages)==12,packages.keys()
left=set(packages);order=[]
while left:
 ready=sorted(n for n in left if not (set(packages[n]['depends'])&left))
 assert ready,('cycle',left)
 order.extend(ready);left-=set(ready)
for name in packages:packages[name]['internal_depends']=sorted(set(packages[name]['depends'])&set(packages))
(a.output/'GRAPH.json').write_text(json.dumps({'order':order,'packages':packages},indent=2)+'\n')
(a.output/'order.txt').write_text('\n'.join(order)+'\n')
lines=['# CI-ONLY LOCAL OVERRIDES. NOT registered with upstream rosdep/rosdistro.']
for name in sorted(packages):lines += [name+':','  ubuntu:','    jammy: [ros-humble-'+name.replace('_','-')+']']
for name in ['cargo-1.85','python3-pybind11','python3-setuptools-scm','rustc-1.85']:
 lines += [name+':','  ubuntu:','    jammy: ['+name+']']
(a.output/'CI_ONLY_LOCAL_ROSDEP.yaml').write_text('\n'.join(lines)+'\n')
print('\n'.join(order))
