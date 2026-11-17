#!/usr/bin/env python3
"""Small deterministic ABI/geometry tests, not an end-to-end planner benchmark."""
import argparse
import ctypes as c
import math
from pathlib import Path
import random
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--library-dir', type=Path, required=True)
args, remaining = parser.parse_known_args()
D = c.c_double
I = c.c_int64
S = c.c_int16
B = c.c_uint8
C = c.c_int32
P = c.POINTER


class NativeContract(unittest.TestCase):
    def test_segment_against_python(self):
        fn = c.CDLL(str(args.library_dir / 'libv13_segment_cells.so')).segment_cells
        fn.argtypes = [P(D), P(D), P(D), I, P(I)]
        fn.restype = I
        rng = random.Random(13)
        for unused in range(100):
            a = [rng.uniform(-10, 10), rng.uniform(-10, 10)]
            b = [rng.uniform(-10, 10), rng.uniform(-10, 10)]
            t = [i / 200 for i in range(201)]
            points = []
            for u in t:
                p = tuple(math.floor(a[j] + (b[j] - a[j]) * u + .5) for j in range(2))
                if not points or p != points[-1]:
                    points.append(p)
            expected = list(dict.fromkeys(points))
            for p, q in zip(points, points[1:]):
                if p[0] != q[0] and p[1] != q[1]:
                    for v in [(p[0], q[1]), (q[0], p[1])]:
                        if v not in expected:
                            expected.append(v)
            out = (I * (len(t) * 6))()
            n = fn((D*2)(*a), (D*2)(*b), (D*len(t))(*t), len(t), out)
            self.assertEqual([(out[2*i], out[2*i+1]) for i in range(n)], expected)

    def test_dstar_cold_repair_and_budget(self):
        class Entry(c.Structure):
            _fields_ = [('a', D), ('b', D), ('token', c.c_uint64), ('id', I)]
        lib = c.CDLL(str(args.library_dir / 'libv13_dstar.so'))
        repair = lib.repair_dstar
        repair.argtypes = [I,I,I,P(S),P(D),P(D),P(D),P(D),D,I,P(B),P(I),
                           P(Entry),I,P(c.c_uint64),c.c_uint64,D,c.c_int,c.c_int,
                           c.c_int,c.c_int,P(I)]
        repair.restype = c.c_void_p
        lib.repair_export.argtypes = [c.c_void_p,P(Entry),P(c.c_uint64),P(c.c_uint64)]
        lib.repair_free.argtypes = [c.c_void_p]
        neighbors = (S * 24)(*([-32768]*24))
        neighbors[0],neighbors[8],neighbors[9],neighbors[17] = 1,1,-1,-1
        costs = (D*24)(*([1.]*24))
        heuristic = (D*3)(0,1,2)
        sides = (I*16)(*([-1]*16))
        opposite = (I*8)(1,0,0,0,0,0,0,0)
        for blocked_middle, seconds in [(False,1.),(True,1.),(False,0.)]:
            for indexed in [0,1]:
                g = (D*3)(math.inf,math.inf,math.inf)
                rhs = (D*3)(math.inf,math.inf,0)
                tokens = (c.c_uint64*3)(2**64-1,2**64-1,1)
                initial = (Entry*1)(Entry(2,0,1,2))
                blocked = (B*3)(0,int(blocked_middle),0)
                result = repair(3,0,2,neighbors,costs,heuristic,g,rhs,seconds,
                                100,blocked,sides,initial,1,tokens,2,0.,1,1,1,indexed,opposite)
                self.assertTrue(result)
                try:
                    heap = (Entry*16)()
                    stats = (c.c_uint64*10)()
                    lib.repair_export(result,heap,tokens,stats)
                    if seconds == 0:
                        self.assertEqual(stats[8],1)
                        self.assertTrue(all(math.isinf(v) for v in g))
                    elif blocked_middle:
                        self.assertTrue(math.isinf(g[0]))
                        self.assertEqual(g[2],0.)
                    else:
                        self.assertEqual(list(g),[2.,1.,0.])
                        self.assertEqual(stats[8],0)
                finally:
                    lib.repair_free(result)

    def test_rhs_and_extract(self):
        # Three states in a line, directed edges 0 -> 1 -> 2.
        neighbors = (S * 24)(*([-32768] * 24))
        neighbors[0] = 1
        neighbors[8] = 1
        costs = (D * 24)(*([1.] * 24))
        g = (D * 3)(2., 1., 0.)
        rhs = (D * 3)(math.inf, math.inf, 0.)
        blocked = (B * 3)(0, 0, 0)
        sides = (I * 16)(*([-1] * 16))
        fn = c.CDLL(str(args.library_dir / 'libv13_dstar.so')).update_affected_rhs
        fn.argtypes = [I, P(I), I, P(S), P(D), P(D), P(D), P(B), P(I)]
        fn.restype = c.c_uint64
        self.assertEqual(fn(2, (I*2)(0, 1), 2, neighbors, costs, g, rhs, blocked, sides), 2)
        self.assertEqual(list(rhs), [2., 1., 0.])
        extract = c.CDLL(str(args.library_dir / 'libv13_extract_path.so')).extract_path
        extract.argtypes = [I, I, I, I, P(S), P(D), P(D), P(B), P(I), P(C), P(C), P(I)]
        extract.restype = I
        rows, cols = (C*3)(0,0,0), (C*3)(0,1,2)
        out = (I*3)()
        def run():
            return extract(3,0,2,3,neighbors,costs,g,blocked,sides,rows,cols,out)
        self.assertEqual(run(), 3)
        self.assertEqual(list(out), [0,1,2])
        blocked[1] = 1
        self.assertEqual(run(), 0)
        fn(2, (I*2)(0,1), 2, neighbors,costs,g,rhs,blocked,sides)
        self.assertTrue(math.isinf(rhs[0]))
        self.assertTrue(math.isinf(rhs[1]))
        self.assertEqual(rhs[2], 0.)


if __name__ == '__main__':
    unittest.main(argv=['native-contract', *remaining])
