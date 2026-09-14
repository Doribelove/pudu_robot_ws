"""Run the unchanged historical cmp2-04 computation in the isolated environment.

Only the optional plot is suppressed: SimSun is unavailable. No planner,
input, threshold, gate, arm, repetition, or timing function is replaced.
"""
import json,sys
from pathlib import Path
from three_d_v1_nav2 import strict_l2_ab

def no_font_plot(output,summary):
 (output/'plot_omitted.json').write_text(json.dumps({'reason':'SimSun unavailable; no fallback font allowed','numerical_computation_unchanged':True}))
strict_l2_ab._plot=no_font_plot
if __name__=='__main__':strict_l2_ab.main()
