# Runtime dependencies and unresolved release gates

- NumPy, SciPy, OpenCV Python bindings.
- OSQP is eagerly imported by the frozen L3 module, even when only Clarabel is
  used by a particular request. Preserve it as a dependency rather than silently
  modifying solver dispatch. Recorded frozen OSQP version: 0.6.7.post3.
- Clarabel is imported by the actual QP adapter; recorded version: 0.11.1.
- ament_index_python locates installed config resources.

Local Humble rosdep resolution found no rules for `python3-osqp` or
`python3-clarabel`. package.xml now depends on `trilayer_osqp_vendor` and
`trilayer_clarabel_vendor`, the separate source-built vendor packages.
The package is not bloom-ready until both vendors meet their release gates. Transitive qdldl dependency belongs to
OSQP's dependency graph, not a silently copied ELF artifact.

No solver wheel or binary has been copied into this source package. Tests may
explicitly receive `--solver-deps PATH` pointing at the frozen historical solver
bundle for algorithm parity testing. Such a result is not clean-OS dependency
resolution or proof of building the solver from source. Version changes need
numerical and end-to-end regression testing.

Full P6 optimize/certify additionally needs its seed/recovery/storage modules,
semantic world/map helpers, native corridor and swept collision libraries,
request state/configuration, and integration audits. These are intentionally not
claimed as present here.

## Implemented native/base-path dependency update

The base optimizer now includes mechanically extracted Dubins endpoint helpers,
actual SweptChecker/collision_checker, native corridor rectangles and native
semantic swept SAT. Both native libraries are built from source as private
package assets. C++ standard library is their only native runtime dependency.

Still excluded: `tail_*`, `failure_*`, `v12_tiles` and historical recovery/storage
strategies activated by external mode/profile overrides, full world/semantic
loaders, L1/L2, traffic audit and Nav2 ACK binding. The frozen base config does not
activate those optional imports. Positive synthetic base-path tests do not prove
those excluded strategies or the original full-P6 environment configuration.
