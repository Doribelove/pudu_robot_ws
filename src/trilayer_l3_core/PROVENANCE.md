# Source provenance and precise modification

Baseline: Doribelove/pudu_robot_ws commit
`a98a639a8ac9b7b50c14f8a69c393706db4cc3ae`, directory
`3YDL3-V13-P6-delivery/runtime/3YDL3-V13-L2/base/runtime/base`.

| Source | Original SHA-256 |
|---|---|
| `src/three_yd_v5/curve_l3.py` | `4471030e38e7ae0dc90f332793e5719cac9d4bcbffff212093a542ddd43674c3` |
| `src/three_yd_v5/curve_qp.py` | `ea759f645b32c57f51e9a099d636e641bd423a2aa42fb067df02e000a4b0cb42` |
| `config/l3.json` | `3fb5520974702cf4eb0df8b1d267953206d42a0509154a503848ed82c91561ac` |

Original user-authored L3 sources are covered by the owner's Apache-2.0 release
authorization. The extracted arena_evaluation planner helpers are separately added original
files, not present in available upstream history; see THIRD_PARTY_SCOPE.md for
the ownership evidence. This does not relicense any attributed third-party code. The QP source and JSON are unchanged. In `curve_l3.py`, exactly
the original `ROOT=Path(__file__).resolve().parents[2]` is replaced by an import
of `ament_index_python.packages.get_package_share_directory` and
`ROOT=Path(get_package_share_directory('trilayer_l3_core'))`, plus a packaging
comment. Numerical functions and configuration values are unchanged.

The Python package namespace is now `trilayer_l3_core`. Relative imports inside
full `optimize`/`certify` paths refer to components not included in this subset;
those functions are not a supported public API. No full-planner compatibility
is inferred from passing numerical-kernel tests.

## Base-profile optimize/certify extension

The supported scope now includes the base `config/l3.json` optimize/certify path,
not the historical environment-injected full P6 profile. Additional changes:

- `curve_l3.py` imports `dubins_choices`, `dubins_edge_from_parameters`,
  `SweptChecker`, `densify`, and `collision_checker` from `.geometry` instead of
  research-package modules. The imported definitions retain their original
  bodies, except native shared-library location lookup.
- `geometry.py` is a mechanically extracted function/class closure, not mocked
  geometry. Source filenames, hashes and exact original extracted-definition
  hashes are in GEOMETRY_PROVENANCE.json. Only import layout and native library
  location change. No approximate collision replacement is used.
- `curve_native.py` is copied from the frozen source; only its private library
  locator changes to ament_index.
- `curve_rectangles.cpp` and `swept_rect.cpp` are copied unchanged from the listed
  source provenance and compiled with C++17, `-O3 -ffp-contract=off
  -fno-fast-math`. The swept source is recovered from the author's original
  semantic_dual_map workspace because the public frozen snapshot included its
  binary, not its source. This is a source recovery, not binary redistribution.

Earlier text describing optimize/certify as entirely unsupported records the
initial numerical-only milestone; this section supersedes that scope.
