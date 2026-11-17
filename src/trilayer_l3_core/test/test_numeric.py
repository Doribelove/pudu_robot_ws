#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import argparse
import importlib
import json
from pathlib import Path
import sys
import time
import unittest

parser=argparse.ArgumentParser()
parser.add_argument('--solver-deps',type=Path)
options,remaining=parser.parse_known_args()
if options.solver_deps:
    sys.path.insert(0,str(options.solver_deps.resolve()))
sys.argv=[sys.argv[0]]+remaining
import numpy as np
from scipy.interpolate import BSpline
from scipy import sparse
from trilayer_l3_core import curve_l3 as l3
from trilayer_l3_core import clarabel_setup,clarabel_solve


class NumericalCore(unittest.TestCase):
    def setUp(self):
        self.knots=np.r_[[0.]*4,1.,2.,[3.]*4]
        self.n=len(self.knots)-4
        self.u=np.linspace(.05,2.95,31)

    def test_basis_all_derivatives(self):
        errors=[]
        for order in range(4):
            actual=l3.basis(self.knots,self.u,order).toarray()
            expected=BSpline(self.knots,np.eye(self.n),3)(self.u,nu=order)
            errors.append(float(np.max(np.abs(actual-expected))))
            np.testing.assert_allclose(actual,expected,atol=1e-12)
        print('basis orders 0..3 maximum errors:',errors)

    def test_curvature_jacobian(self):
        B,D,E,F=[l3.basis(self.knots,self.u,k) for k in range(4)]
        z=np.r_[np.linspace(0,3,self.n),[0,.1,.3,.4,.2,0]]
        jac=l3.differential(B,D,E,F,z)[6].toarray()
        numerical=np.zeros_like(jac);eps=1e-6
        for i in range(len(z)):
            plus=z.copy();plus[i]+=eps
            minus=z.copy();minus[i]-=eps
            numerical[:,i]=(l3.differential(B,D,E,F,plus)[5]-l3.differential(B,D,E,F,minus)[5])/(2*eps)
        np.testing.assert_allclose(jac,numerical,atol=1e-6,rtol=1e-5)
        print('curvature Jacobian max error:',float(np.max(np.abs(jac-numerical))))

    def test_actual_constrained_qp(self):
        solver=clarabel_setup(sparse.eye(2,format='csc')*2,np.array([-4.,-2.]),
            sparse.eye(2,format='csc'),np.array([0.,0.]),np.array([1.,2.]),time.monotonic()+2,1e-8)
        solution=clarabel_solve(solver)
        self.assertIn(solution.info.status_val,(1,2))
        np.testing.assert_allclose(solution.x,[1.,1.],atol=1e-6)
        print('actual QP:',solution.x,solution.info.status)

    def test_installed_configuration(self):
        path=l3.ROOT/'config/l3.json'
        self.assertEqual(json.loads(path.read_text()),l3.CONFIG)
        self.assertEqual(l3.CONFIG['qp_solver'],'clarabel')
        print('configuration:',path)

if __name__=='__main__':
    print('core source:',l3.__file__)
    for name in ('numpy','scipy','cv2','osqp','clarabel'):
        module=importlib.import_module(name)
        print('dependency:',name,getattr(module,'__version__','unknown'),module.__file__)
    print('solver validation mode:', 'EXPLICIT FROZEN BINARY DEPS (not source-build proof)' if options.solver_deps else 'installed dependencies')
    unittest.main(verbosity=2)
