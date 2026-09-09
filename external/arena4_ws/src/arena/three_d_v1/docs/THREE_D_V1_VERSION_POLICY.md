# 3D-V1 version policy

| Name | Status | Access |
|---|---|---|
| 3D-V1-r0 | Historical design and minimum validation | Explicit r0/legacy benchmark CLI only |
| 3D-V1-r1 | Historical lifecycle optimization; not promoted | Explicit r1 benchmark CLI only |
| r2 production acceptance | Promotion evidence and frozen research runner | Explicit r2 benchmark CLI only |
| 3D-V1-r2-stable | Only default production baseline | Default import, factory, config, and CLI |

The stable label does not create `3D-V2` or an algorithmic r3. The source
revision remains `r2-production-acceptance`.

Historical source, reports, and experiment directories are retained for
reproduction. They are not production candidates and the factory will not
select them implicitly. The original unrevisioned r0 command names remain as
documented compatibility aliases; explicit `three_d_v1_r0_*` aliases are
provided for new reproduction scripts.

If stable equivalence or any hard release gate fails, do not label the output
stable. Keep r2 production acceptance as a candidate and run a new write-once
release-candidate directory after fixing only the wrapper or integration.
