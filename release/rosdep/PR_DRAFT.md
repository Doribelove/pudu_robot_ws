# Draft 1: Add Python3 setuptools-scm and pybind11 helper rules

Please add `python3-setuptools-scm` and `python3-pybind11` to python.yaml.

Upstream:
- https://github.com/pypa/setuptools-scm
- https://github.com/pybind/pybind11

Purpose: offline source builds of Python extension dependencies need the setuptools_scm version helper and pybind11 Python build helper. Existing `pybind11-dev` maps to headers on Ubuntu, not the Python helper module. Both native package rules are distinct from pip installation and follow Python3 naming guidance.

Distribution links:
- Debian: https://packages.debian.org/trixie/python3-setuptools-scm
- Debian: https://packages.debian.org/trixie/python3-pybind11
- Ubuntu: https://packages.ubuntu.com/jammy/python3-setuptools-scm
- Ubuntu: https://packages.ubuntu.com/jammy/python3-pybind11

Generic Debian/Ubuntu mappings are proposed. Other platform mappings are not included in this minimal request and availability there is not asserted. Native Ubuntu packages are architecture all. Format, uniqueness and in-memory Jammy apt resolution tests pass against the recorded base commit.

# Draft 2, optional/separate: Versioned Rust/Cargo1.85 for Jammy/Noble

Request review of `rustc-1.85` and `cargo-1.85` rules targeting official Ubuntu Jammy/Noble versioned packages only.

Upstream: https://github.com/rust-lang/rust and https://github.com/rust-lang/cargo

Purpose: preserve a tested versioned compiler line for offline frozen-dependency numerical extensions requiring Rust>=1.84. Native official Jammy1.85.1 packages have passed an isolated amd64 build and solver regression. This does not request downloading a compiler during package build.

Distribution links:
- https://packages.ubuntu.com/jammy-updates/rustc-1.85
- https://packages.ubuntu.com/jammy-updates/cargo-1.85
- https://packages.ubuntu.com/search?keywords=rustc-1.85&searchon=names&exact=1&suite=all&section=all
- https://packages.ubuntu.com/search?keywords=cargo-1.85&searchon=names&exact=1&suite=all&section=all
- Debian: no equivalent versioned mapping proposed; Debian uses unversioned packages.

Important alternative: existing `cargo-latest` maps Jammy to cargo-1.91 and brings the matching compiler through native dependencies. We are not claiming this existing key is missing or inadequate in general. Using it here would need a fresh toolchain regression and versioned-path discovery changes. Reviewers may prefer that approach over new pinned keys. No unverified mapping is added for later Ubuntu releases lacking the1.85 package.

Both requests are drafts only, not submitted. ARM package availability has been checked; ARM solver execution has not.
