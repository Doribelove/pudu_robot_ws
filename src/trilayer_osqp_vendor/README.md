# TriLayer OSQP vendor

Offline source build of **OSQP 0.6.7.post3 + QDLDL 0.1.7.post5** for Python3 on
Ubuntu22.04/amd64. Original PyPI sdists, SHA256 and all source license notices are
included. No solver wheel/shared library is included in this source package.

CMake builds from the hash-pinned sources in its binary directory, extracts the
new wheels to a staging tree and installs Python extensions under this package's
ament prefix. No pip command, global install or download is performed. Build
logs and generated provenance remain in the build directory; provenance and
licenses are installed in share/trilayer_osqp_vendor.

Jammy build prerequisites: Python3 development headers, NumPy/SciPy, setuptools,
setuptools-scm, wheel, Python pybind11 + development headers, GCC and CMake. We
validated NumPy1.21.5, setuptools59.6.0/scm6.4.2, wheel0.37.1, pybind11 2.9.1 and
CMake3.22.1. The recipe invokes original setup.py with the known SCM version,
using a distribution-targeted NumPy ABI rather than upstream's NumPy2 PEP517
build-isolation requirement for general binary wheels. Solver source is unchanged.

The optional cache variable `TRILAYER_OSQP_BUILD_PYTHONPATH` identifies an explicit
build-only directory for pre-provisioned dependencies, e.g. an extracted Jammy
setuptools-scm package. It is not installed or added to runtime search paths.
Missing build dependencies fail before invoking upstream setup.py; this also
prevents qdldl's optional pybind11 pip-install fallback.

The installed-space test checks imported OSQP/QDLDL paths are inside this prefix,
then solves two constrained QPs, an infeasible QP and a QDLDL linear system.
Earlier source builds matched frozen P6 results on runtime NumPy1.21.5 and1.24.4.

## Release gates

- Only amd64 was verified; this prototype explicitly rejects unvalidated ARM.
- Jammy apt provides python3-setuptools-scm and python3-pybind11, but current
  rosdep lacks matching keys. `rosdep/proposed.yaml` is a proposal, not installed
  or submitted. Resolve upstream dependency rules before Bloom.
- Replace the placeholder maintainer address before release.
- OSQP/QDLDL are Apache-2.0; bundled AMD is BSD-3-Clause. Preserve all notices.
- This package is a solver dependency, not the full P6 planner or an apt release.

## Build-tree versus installed tests

Default CTest validates the newly built `vendor_work/python` modules and checks
their origin before installation; it does not need this package preinstalled.
An explicit post-install smoke test remains available by configuring
`-DTRILAYER_VENDOR_TEST_INSTALLED=ON`, then installing before that test runs.
The default OFF setting disables only the additional installed-layout check,
not numerical solver coverage. Do not enable it in a pre-install Debian build
phase. Source provenance and solver algorithms are unchanged.
