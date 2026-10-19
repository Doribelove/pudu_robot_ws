# Contributing

Start with the [public synthetic-garage example](examples/3ydl3_garage_b/README.md).
A useful issue includes the release/commit, Linux/ROS/Python versions, the exact
command, the selected fixed query IDs, and `console.log` plus `validation.json`
when present. Do not attach real customer maps or credentials.

For changes to the public launcher or result checks:

```bash
python3 -m venv .demo-venv
.demo-venv/bin/pip install --require-hashes -r examples/3ydl3_garage_b/requirements.lock
OPENBLAS_NUM_THREADS=1 .demo-venv/bin/python -m pytest tests/public_demo -q
```

Changes to planner code, native libraries, dependencies or map bindings also need
the complete Q01–Q10 run in the documented Humble environment. Preserve all
failures and report endpoint class, ACK results and strict audit outcomes.
A successful import or L1 output alone does not establish three-layer success.

Accepted research snapshots remain historical evidence. Develop in a separate
copy/branch, explicitly record source changes, and never silently relax vehicle,
traffic, endpoint or continuous-curvature checks. Update the public manifest only
after reviewing intentional release changes; retain original provenance hashes.

Use a focused commit and explain the concrete behavior and validation. Sign off
your own commits with `git commit --signoff`. Keep third-party notices and follow
[license scope](LICENSING.md). AI-assisted changes should say what was assisted
and what was actually validated; do not imply unperformed manual or hardware tests.

See [RELEASING](docs/RELEASING.md) for release gates and
[upstream maintenance](docs/UPSTREAM_MAINTENANCE.md) for the current collaboration focus.
