import json,os
from run_v10r5_suite import run,ROOT
# Bounded repair diagnostic; never mixed into formal performance records.
for mid,qs in {'pudu_wanda_3f':['Q09'],'arena_medium_200x200_2d_005':['A2B-04'],'mentor_map_20260825_005':['A2B-03','A2B-07']}.items():
    if mid=='mentor_map_20260825_005':os.environ['THREEYD_R12_RECTANGLE_BAND_M']='2.4'
    run(ROOT/'results/remaining_repair_probe'/mid,mid,'r5','MLSEF',queries=qs)
    os.environ.pop('THREEYD_R12_RECTANGLE_BAND_M',None)
# Complete fixed10 sets in the production-intended mode.
for m in json.loads((ROOT/'MAPS.json').read_text())['maps']:
    run(ROOT/'results/pilot100'/m['map_id'],m['map_id'],'r5','MLSE')
