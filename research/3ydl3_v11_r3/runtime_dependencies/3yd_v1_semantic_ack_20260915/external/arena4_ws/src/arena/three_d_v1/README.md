# arena_3d_v1

The only default production baseline is **3D-V1-r2-stable**. It is a thin,
fail-closed production interface over the accepted r2 implementation; it is
not a new algorithm revision.

Use `three_d_v1_plan`, `three_d_v1_cache_prebuild`,
`three_d_v1_cache_verify`, and `three_d_v1_validate_release` for production.
The r0/r1/r2 experiment commands are legacy/reproduction interfaces and are
never selected by the production factory.

See `docs/THREE_D_V1_STABLE_RUNTIME.md` and
`docs/THREE_D_V1_VERSION_POLICY.md`.
