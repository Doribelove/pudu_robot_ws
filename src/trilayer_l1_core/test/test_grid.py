#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
import ast
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np
from trilayer_l1_core import GridGuideSearch


class GridContract(unittest.TestCase):
    def assert_path(self,path,grid,start,goal):
        self.assertIsNotNone(path)
        self.assertEqual(path[0],start);self.assertEqual(path[-1],goal)
        for cell in path:self.assertTrue(grid[cell])
        for a,b in zip(path,path[1:]):
            dr,dc=b[0]-a[0],b[1]-a[1]
            self.assertEqual(max(abs(dr),abs(dc)),1)
            if dr and dc:
                self.assertTrue(grid[a[0]+dr,a[1]])
                self.assertTrue(grid[a[0],a[1]+dc])

    def test_connected_and_no_state_leak(self):
        search=GridGuideSearch();grid=np.ones((11,15),bool)
        for mode in ('weighted_astar','dijkstra'):
            result=search.search(grid,(2,1),(8,13),mode=mode)
            self.assert_path(result.cells,grid,(2,1),(8,13))
            self.assertEqual(result.algorithm,mode)
            self.assertGreater(result.expanded,0)
            self.assertEqual(search.search(grid,(2,1),(2,1),mode=mode).cells,((2,1),))

    def test_disconnected_and_corner_cut_forbidden(self):
        for mode in ('weighted_astar','dijkstra'):
            grid=np.ones((9,9),bool);grid[:,4]=False
            self.assertIsNone(GridGuideSearch().search(grid,(4,1),(4,7),mode=mode).cells)
            diagonal=np.array([[True,False],[False,True]])
            self.assertIsNone(GridGuideSearch().search(diagonal,(0,0),(1,1),mode=mode).cells)

    def test_alternate_channel_after_obstacle_update(self):
        grid=np.ones((15,17),bool);grid[:,8]=False;grid[3,8]=grid[11,8]=True
        search=GridGuideSearch()
        for mode in ('weighted_astar','dijkstra'):
            first=search.search(grid,(7,2),(7,14),mode=mode).cells
            self.assert_path(first,grid,(7,2),(7,14))
            crossing=next(r for r,c in first if c==8)
            changed=grid.copy();changed[crossing,8]=False
            second=search.search(changed,(7,2),(7,14),mode=mode).cells
            self.assert_path(second,changed,(7,2),(7,14))
            self.assertNotIn((crossing,8),second)
            self.assertIn((11 if crossing==3 else 3,8),second)
            changed[:,8]=False
            self.assertIsNone(search.search(changed,(7,2),(7,14),mode=mode).cells)

    def test_budget_and_input_contract(self):
        grid=np.ones((20,20),bool)
        result=GridGuideSearch(expansion_budget=1).search(grid,(0,0),(19,19))
        self.assertIsNone(result.cells);self.assertEqual(result.expanded,1)
        for g,a,b in [(grid.astype(np.uint8),(0,0),(1,1)),(grid,(-1,0),(1,1)),
                      (grid,(0.,0),(1,1))]:
            with self.assertRaises(ValueError):GridGuideSearch().search(g,a,b)
        with self.assertRaises(ValueError):GridGuideSearch(max_cells=3).search(grid,(0,0),(1,1))
        with self.assertRaises(ValueError):GridGuideSearch().search(grid,(0,0),(1,1),mode='other')

    def test_coarsening_and_endpoint_snap(self):
        free=np.ones((3,3),bool)
        coarse=GridGuideSearch._coarsen(free,2)
        np.testing.assert_array_equal(coarse,[[True,False],[False,False]])
        free[0,0]=False
        self.assertEqual(GridGuideSearch._snap((0,0),free),(0,1))
        self.assertIsNone(GridGuideSearch._snap((0,0),np.zeros((3,3),bool)))

    def test_frozen_method_hashes(self):
        import trilayer_l1_core.frozen_search as frozen
        source=Path(frozen.__file__).read_text();lines=source.splitlines(keepends=True)
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef))
        manifest=json.loads((Path(__file__).parents[1]/'SOURCE_PROVENANCE.json').read_text())
        for name,record in manifest['extracted_methods'].items():
            node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
            start=min([node.lineno]+[n.lineno for n in node.decorator_list])
            raw=''.join(lines[start-1:node.end_lineno])
            self.assertEqual(hashlib.sha256(raw.encode()).hexdigest(),record['sha256'])


if __name__=='__main__':unittest.main(verbosity=2)
