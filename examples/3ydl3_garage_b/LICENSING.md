# License scope and attribution

The contributor has confirmed authorization to publish the added original
planning code and synthetic garage B. Newly added launcher, verifier, tests,
documentation and synthetic example data are licensed under Apache-2.0; see
[`../../LICENSE`](../../LICENSE). Original additions in `runtime/r4/src` are
also available under Apache-2.0, subject to the existing notices below.

This is a distribution of components with multiple licenses. The root license
does not relicense third-party code or the historical repository as a whole.

| Component | Applicable terms | Preserved evidence |
|---|---|---|
| Nav2 costmap sources, installed costmap libraries and plugins | BSD-3-Clause and Apache-2.0, as indicated by each original file | Source headers and `package.xml`; `licenses/Nav2-BSD.txt`, `licenses/Apache-2.0.txt` |
| `runtime/r4/native/goal_dijkstra.cpp` | SciPy BSD terms, including the retained copyright notices | `native/THIRD_PARTY.md`, `native/SCIPY_LICENSE.txt` |
| Arena integration/base planning packages in `runtime/v1/external` | Retain the original BSD declarations and any file-level notices; Arena upstream's MIT notice is also included | `package.xml`, source headers, `licenses/Arena-MIT.txt` |
| Earlier planning layers in `runtime/v5` | Original notices and the BSD declarations of their base packages; no third-party notices are removed | Source, package metadata and native component notices |
| NumPy, SciPy, Clarabel, OSQP, OpenCV, Pillow, PyYAML, psutil and their dependencies | Each downloaded distribution's own license | Exact distribution wheels are hash-pinned in `requirements.lock`; license files remain in the installed distributions |
| ROS 2 / system packages | Each Ubuntu/ROS package's own license | Installed copyright and package metadata; these OS distributions are not copied into this example |

The supplied x86_64 binaries are accompanied by the corresponding native source
and original notices. The native build entry is `maintenance/build_native.py`.
Changes to upstream files retain their original headers. For this public adapter,
only the environment and the one-map registry are adapted; accepted planner
source hashes are recorded in `SOURCE_PROVENANCE.json`.

The original synthetic map raster is losslessly compressed. The map/query/
parking-access binding is checked after decompression. No real facility map,
customer map or company document is part of this example.

Names such as PUDU, ROS, Nav2 and Arena identify compatibility or provenance;
this release does not claim endorsement by those organizations.
