from pathlib import Path
import sys,json
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'tools'))
from run_v11_suite import run
D=R/'results/tenmap_pilot';D.mkdir(exist_ok=False)
for i,m in enumerate(json.loads((R/'MAPS.json').read_text())['maps']):
    arms=['control','candidate'] if i%2 else ['candidate','control']
    for arm in arms:run(D/m['map_id']/arm,m['map_id'],arm)
(D/'COMPLETE.json').write_text(json.dumps({'complete':True,'diagnostic_only':True,'rounds':1,'no_allocator_overrides':True})+'\n')
