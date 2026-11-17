# SPDX-License-Identifier: Apache-2.0
"""Map-only L1 grid search subset extracted from the P6 engineering baseline."""
from dataclasses import dataclass
import numpy as np
from .frozen_search import _FrozenFreeSpaceSearch


@dataclass(frozen=True)
class SearchResult:
    cells: tuple | None
    expanded: int
    algorithm: str


class GridGuideSearch(_FrozenFreeSpaceSearch):
    """Validated wrapper around unchanged P6 map-only search methods.

    True cells are traversable. Weighted A* is a soft guide, not an optimality
    guarantee. Dijkstra mode searches a whole component, ignoring the A* budget.
    Each search constructs fresh state so changed maps cannot reuse stale graphs.
    """
    def __init__(self, expansion_budget=400000, max_cells=1000000):
        for value in (expansion_budget,max_cells):
            if isinstance(value,bool) or not isinstance(value,int) or value < 1:
                raise ValueError('budgets must be positive integers')
        self.expansion_budget=expansion_budget
        self.max_cells=max_cells

    def search(self, free, start, goal, *, mode='weighted_astar'):
        grid=np.asarray(free)
        if grid.ndim!=2 or not grid.size or grid.dtype!=np.bool_:
            raise ValueError('free must be a nonempty 2D boolean grid')
        if grid.size>self.max_cells:
            raise ValueError('grid exceeds configured cell bound')
        grid=np.array(grid,copy=True,order='C')
        ends=[]
        for point in (start,goal):
            if len(point)!=2 or any(isinstance(v,(bool,np.bool_)) or not isinstance(v,(int,np.integer)) for v in point):
                raise ValueError('endpoints must be integer (row, column) pairs')
            cell=tuple(map(int,point))
            if not (0<=cell[0]<grid.shape[0] and 0<=cell[1]<grid.shape[1]) or not grid[cell]:
                raise ValueError('endpoint is outside free space')
            ends.append(cell)
        if mode not in ('weighted_astar','dijkstra'):
            raise ValueError('unknown search mode')
        engine=_FrozenFreeSpaceSearch()
        engine.goal_stop=False
        engine.expansion_budget=self.expansion_budget
        engine._graphs={1:self._compile_graph(grid)} if mode=='dijkstra' else {}
        path,expanded=engine._astar(ends[0],ends[1],grid,1)
        return SearchResult(None if path is None else tuple(path),expanded,mode)
