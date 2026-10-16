# Migration and rollback

## Migration

- Replace direct `Layered3DV1R2Controller` construction with
  `arena_3d_v1.create_controller(...)` and omit `revision`.
- Replace the r2 acceptance YAML with `config/three_d_v1_stable.yaml`.
- Package each L1 route as a verified plan bundle, prebuild the stable cache
  offline, and verify it before deployment.
- Treat a cache miss/reject as normal safe degradation to deterministic A*;
  never add an online cache build to hide the miss.

Existing no-revision factory calls automatically resolve to stable. Explicit
r0/r1/r2 production requests now fail closed and must be moved to benchmark or
reproduction mode.

## Rollback

Rollback changes selection, not historical files:

1. Remove the stable deployment entry from the downstream launch/config.
2. Use the frozen r2 acceptance runner explicitly in a benchmark environment.
3. For production safety, force deterministic grid A* rather than selecting an
   old D* implementation through the production factory.
4. Preserve the failed stable cache and release directory for diagnosis.

Do not delete cache bundles automatically. Use
`three_d_v1_cache_verify --purge-obsolete-report` to produce a review list.
