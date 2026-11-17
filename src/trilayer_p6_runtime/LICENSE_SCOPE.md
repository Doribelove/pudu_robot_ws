# Source ownership and license scope

The owner explicitly confirmed that their original planner code is theirs and
may be open-sourced. The portable glue and those original additions use Apache-2.0.
The parent directory's ROS package metadata alone does not relicense separately
written additions or establish that they were copied from upstream.

Read-only provenance check on 2026-11-17:
- Original checkout remote: https://github.com/voshch/arena-evaluation.git
- Inspected HEAD: `94762429bea19b84cab50a3d0910a736184738a0`.
- All locally available refs: 162 commits.
- Of the 37 arena_evaluation Python files in the P6 runtime envelope, only the
  zero-byte `__init__.py` is tracked at upstream HEAD.
- `semantic_map.py`, `semantic_rasterizer.py`, `semantic_constraint_core.py`,
  `se2_semantic_guide.py` and `planner_benchmark/map_utils.py` are local additions
  (untracked in that upstream checkout), with no exact-path history in any of
  those 162 commits. Relevant source text has no copied-code/copyright/license
  notice identifying another owner.

Combined with the owner's authorship statement, these facts support treating
the extracted planner/map/Dubins additions as original authorized code rather
than imposing generic BSD solely because the old surrounding ROS package says
BSD. This supersedes the earlier conservative generic-BSD placeholder. The
absence of an upstream LICENSE is not itself evidence of a prohibition.
This is a scoped evidence assessment, not a proof that every possible algorithm
or file ever used by the project has been audited.

Keep exact source hashes in the package provenance. Preserve any explicit
third-party source attribution discovered in individual files. Dependency
licenses remain separate; mathematical similarity to an algorithm is not alone
proof that its code was copied.

Known copied/adapted exception: `src/native/goal_dijkstra.cpp` adapts the
SciPy 1.8.0 Fibonacci heap. It retains BSD-3-Clause via
`licenses/SCIPY_LICENSE.txt` and the original source URL in
`licenses/THIRD_PARTY.md`. The package manifest records Apache-2.0 and
BSD-3-Clause. These notices must remain installed. Neither the owner statement
nor the Apache root license overrides this explicit attribution.
