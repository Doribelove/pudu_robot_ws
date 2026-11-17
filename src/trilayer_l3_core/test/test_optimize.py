#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Actual base-profile SQP and continuous native sweep, no mocks or ROS nodes."""
import argparse
from dataclasses import dataclass
from pathlib import Path
import sys,time,unittest
parser=argparse.ArgumentParser();parser.add_argument('--solver-deps',type=Path)
options,rest=parser.parse_known_args()
if options.solver_deps:sys.path.insert(0,str(options.solver_deps.resolve()))
sys.argv=[sys.argv[0]]+rest
import numpy as np
from scipy.interpolate import BSpline
from trilayer_l3_core.curve_l3 import optimize,certify,CONFIG,CurveFailure
from trilayer_l3_core.geometry import SweptChecker

@dataclass
class MapGeometry:
    resolution: float=.05
    origin: tuple=(0.,0.,0.)

@dataclass
class SyntheticWorld:
    hard: np.ndarray
    map: MapGeometry
    vehicle: tuple=(.265,.225,.4)

class FullBasePath(unittest.TestCase):
    def world(self):return SyntheticWorld(np.zeros((100,160),dtype=bool),MapGeometry())
    def run_case(self, config):
        world=self.world();mask=np.ones_like(world.hard)
        path,curve,log=optimize(world,mask,np.array([[1.,2.5],[6.,2.5]]),
            (1.,2.5,0.),(6.,2.5,0.),time.monotonic()+8,config)
        self.assertIsNotNone(path,log);self.assertTrue(log['success'])
        np.testing.assert_allclose(path[[0,-1],:2],[[1,2.5],[6,2.5]],atol=1e-8)
        self.assertLess(log['certificate']['max_curvature_bound_1pm'],1/world.vehicle[2])
        self.assertGreater(log['certificate']['body_sweep_groups'],0)
        spline=BSpline(curve['knots'],curve['coefficients'],curve['degree'])
        checked,certificate=certify(spline,world,mask,time.monotonic()+2,config,
                                   corridor=curve['corridor'],parameters=curve['parameters'])
        np.testing.assert_allclose(checked,path,atol=1e-12)
        self.assertIn('native rectangle sweep',certificate['bound_method'])
        print('BASE OPTIMIZE',{'iterations':len(log['iterations']),
            'quadratic_start':log['accepted_quadratic_initialization'],
            'intervals':certificate['intervals'],'length_m':log['length_m']})
        return world,mask,spline

    def test_default_frozen_config(self):self.run_case(CONFIG)

    def test_actual_sqp_and_collision_rejection(self):
        # Existing configuration switch, not a changed algorithm or mock solver.
        world,mask,spline=self.run_case(dict(CONFIG,certified_quadratic_start=False))
        world.hard[:,70]=True
        with self.assertRaisesRegex(CurveFailure,'CURVE_CONTINUOUS_BODY_OR_SCOPE'):
            certify(spline,world,mask,time.monotonic()+2)

    def test_native_sweep_rejects_between_free_endpoints(self):
        world=self.world();world.hard[:,70]=True
        checker=SweptChecker(world.hard,world.map.resolution,world.map.origin,*world.vehicle[:2])
        a=np.array([[1.,2.5,0.]]);b=np.array([[6.,2.5,0.]])
        self.assertTrue(checker.check(a));self.assertTrue(checker.check(b))
        self.assertFalse(checker.check(np.vstack((a,b))))

    def test_blocked_guide_cannot_succeed(self):
        world=self.world();world.hard[:,60:100]=True
        path,_,log=optimize(world,np.ones_like(world.hard),np.array([[1.,2.5],[6.,2.5]]),
            (1.,2.5,0.),(6.,2.5,0.),time.monotonic()+2)
        self.assertIsNone(path);self.assertFalse(log['success'])
        print('BLOCKED GUIDE:',log.get('failure'))

if __name__=='__main__':
    print('Solver mode:', 'explicit frozen binary dependencies' if options.solver_deps else 'installed dependencies')
    unittest.main(verbosity=2)
