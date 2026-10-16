from run_v10r5_suite import run,ROOT
cases={'3yd_synth_trizone_c_v2':['Q01','Q10'],'pudu_wanda_3f':['Q08','Q09','Q10'],'arena_medium_200x200_2d_005':['A2B-04'],'mentor_map_20260825_005':['A2B-03','A2B-07']}
for mid,queries in cases.items():
    run(ROOT/'results/early_probe'/mid,mid,'r5','MLSE',queries=queries)
