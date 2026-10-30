# Changelog

## Unreleased

- Reject empty, repeated or unknown query IDs before validating saved public-demo
  results, preventing empty success reports and duplicate counting of one case.
  Valid subsets and reordered unique selections continue to work.
- Reject mixed native-library hashes across all observed processes and paths in
  the public example's result validator. A matching copy can no longer hide an
  earlier incompatible copy with the same basename; matching duplicates remain
  valid, and every required library must still be present. The planner, native
  binaries, map bindings, and published demo.1 release are unchanged.

## 3ydl3-v11r4-demo.1 — 2026-10-19

First self-contained public example for the accepted V11-r4 architecture.

- Include synthetic garage B, all original fixed Q01–Q10, semantic/topology and parking-access bindings.
- Add a relocatable, private-namespace launcher with a default Q01 example and complete fixed-ten mode.
- Keep the accepted Nav2 1.1.19 server/plugin ABI together, with matching source and library hashes.
- Hash-pin Python wheels and transitive dependencies; pin Nav2 entry packages and the CI base-image digest.
- Automatically verify exact endpoints, original strict audit/ACK evidence, continuous curvature and exported curve/path consistency.
- Provide input-tampering/failure-path tests and a complete three-layer CI job.
- Clarify Apache-2.0 scope and retained third-party terms; add contribution and release instructions.

Scope: static global reference planning on the synthetic example. The accepted
V11-r4 source is retained; this packaging release adds no new planning algorithm
or hardware/performance claim. V11-r5 remains a separate experimental candidate.

## Historical snapshots

The previous public main entry was V11-r3, with source and standalone ten-map
results but no map distribution or arbitrary-directory deployment verification.
Those files remain under `research/3ydl3_v11_r3/` and `docs/3ydl3_v11_r3/`.
