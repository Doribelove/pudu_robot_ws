# Public example releases

Tags have the form `3ydl3-v11r4-demo.N`: the research architecture and public
packaging revision are separate identifiers. A future architecture uses a new
version prefix and explicit migration notes.

Before tagging:

1. Review every added file against the publication scope: synthetic map and required runtime only. Preserve component licenses, source provenance and original fixed inputs.
2. Install `requirements.lock` with `--require-hashes`. Run `demo.py --verify-only` and `pytest tests/public_demo -q`.
3. Run the isolated dependency probe and full original Q01–Q10 in a clean Humble environment. Require 10/10 exact success, actual costmap ACK, original strict audits, continuous certificates and curve/output binding.
4. Record the commit, CI URL, resolved system packages and local reproduction environment. Distinguish the public ten-query run from the older ten-map, three-round study. Never convert packaging timings into research performance claims.
5. Update `CHANGELOG.md`, `CURRENT_WORK.json`, expected-result scope and any intentional manifest changes. Require `git diff --check` and both CI jobs to pass for the release commit.
6. Push the reviewed commit to `main`, create the tag and publish GitHub release notes with exact commands and validation evidence. Provide the release archive SHA-256. Do not include `.venv`, caches, run outputs or host credentials.

The archive should include `examples/3ydl3_garage_b`, root `LICENSE`, `LICENSING.md`
and `NOTICE`. It is runnable with the prerequisites documented in the example.
The package manifest checks the example's own bytes; the release-archive hash
checks the downloadable archive as a whole.

If a pinned apt build disappears, or any strict result changes, publish a new
packaging revision after validation. Do not move an existing tag or silently
replace the release archive. Architecture changes require fresh acceptance.
