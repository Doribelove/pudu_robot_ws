# V11-r4 public example validation

Date: 2026-10-19. Scope: the public synthetic garage B, its original fixed Q01–Q10,
and the accepted V11-r4 three-layer static reference-planning architecture.

- A hash-locked Python 3.10 environment was installed from the specified wheels.
- The launcher ran with the host home hidden, using only the packaged runtime,
  synthetic input binding, matching Nav2 1.1.19 SDK and the system Humble underlay.
- The default Q01 run passed, and a complete run returned **10/10 exact successes**,
  with no endpoint tolerance. See [local validation JSON](local-fixed10-validation.json).
- All ten results passed actual costmap ACK, original independent body/traffic/
  kinematic audits, continuous curvature ≤2.5 m⁻¹ and curve/output binding.
  The loaded Smac, costmap core and inflation-layer hashes match the package.
- **12 contract/failure tests** pass locally, including raster corruption,
  missing audits, bad ACK, excessive curvature, relaxed endpoints, detached
  output curves and mismatched native libraries.
- A fresh Git archive was extracted into a different directory whose path
  contains a space. Its manifest and a new isolated Q01 run passed.
- Owned navigation processes and shared-memory resources were cleaned up; no
  original research source, formal result or accepted input was modified.

A clean GitHub-hosted Humble-container run on commit `abf37ec` also completed
both jobs and all ten exact queries:
[CI run](https://github.com/Doribelove/pudu_robot_ws/actions/runs/39193377475).
The final release's own CI and downloadable validation artifact are linked in
its GitHub release notes. The [workflow](../../.github/workflows/public-demo.yml)
records resolved OS package versions and fails on missing or inconsistent results.

The first deployment attempts exposed an unavailable apt build and a system
Nav2/accepted-plugin ABI mismatch. Both were corrected in the public packaging;
the repository does not hide those failed workflow runs. Matching native source
and library fingerprints are shipped together.

These checks establish a runnable static example on the documented Linux amd64
platform. They do not extend the old ten-map study, certify real-robot operation,
claim new speedups, or validate the separate V11-r5 research candidate. The
optional developer native-rebuild command is separate from the tested prebuilt
runtime installation path.
