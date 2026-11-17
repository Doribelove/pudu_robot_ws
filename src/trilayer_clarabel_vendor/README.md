# trilayer_clarabel_vendor (experimental, not published)

Offline source-built Clarabel 0.11.1 Python extension for TriLayer. Pure source archive contains unchanged original Clarabel code and 144 selected Cargo crate sources; 189 inactive dependencies are explicitly marked, fail-closed **generated resolution-only stubs**, not upstream implementations. No Intel MKL implementation is bundled. Cargo.lock is unchanged. Both x86_64 and aarch64 Linux dependency closures are retained; only amd64 has been compiled and tested. This does not establish J6M/Orin compatibility.

Build directly with Cargo `--locked --offline --release --features python`, retaining defaults, faer, and SDP/scipy BLAS. No network, pip, maturin, wheel, toolchain installer, or global install occurs at build time. The extension and original Python shim are installed into this package's prefix; setup hooks expose that prefix. Build-directory source copies and checksums are verified. Cargo artifact records assert no stub was compiled.

## Toolchain blocker

Rust **and Cargo >=1.84** are mandatory at configuration and build time. The default unversioned Ubuntu Jammy Rust is insufficient. Official Jammy versioned rustc-1.85/cargo-1.85 packages have been identified; that route has now passed independent local amd64 build/testing. CMake prefers versioned executable names before unversioned fallbacks. `package.xml` records the version requirement; that does not make ROS buildfarm supply a compatible compiler. Official Jammy versioned debs have now passed an isolated build and four-case parity. The corresponding rosdep mappings and actual ROS buildfarm run remain release gates; local success alone does not complete them. Supply already-installed, audited offline executable paths explicitly; this package never downloads a compiler:

```
colcon build --packages-select trilayer_clarabel_vendor --cmake-args \
 -DTRILAYER_CLARABEL_CARGO=/path/to/rust/bin/cargo \
 -DTRILAYER_CLARABEL_RUSTC=/path/to/rust/bin/rustc \
 -DTRILAYER_CLARABEL_JOBS=2 -DBUILD_TESTING=ON
colcon test --packages-select trilayer_clarabel_vendor
```

Python development headers, numpy/scipy/cffi, C/C++ linker toolchain and ROS ament are required. Set Python3_EXECUTABLE if needed. The initial integration used the official Rust1.85 tar toolchain; the second clean build used SHA-verified official Jammy1.85.1 debs extracted locally without installation. Versioned apt dependencies are declared; their new rosdep rules are only proposals.

## Source and license provenance

`SOURCE_MANIFEST.json` gives archive and lock hashes. `provenance/` records platform closures, the filtering method, source identity, license inventory, and preceding standalone test results. Those preceding results are not substituted for this package's own build/test results. `BUILD_PROVENANCE.json` is generated on build and installed.

Retained third-party license files are in `third_party_notices/`. Thirteen missing independent license files were recovered separately by the coordinating agent and are in `supplemental_notices/` with exact commit provenance. The npyz notice is transparently assembled from upstream README copyright declarations and standard MIT grant, not claimed to be an upstream LICENSE file. Supplemental notices do not modify the archived vendor. Inventory records describe the original payload and therefore still indicate the original missing-file findings.

Apache-2.0 applies to glue/root; third-party expressions differ and include alternatives and LLVM/Unicode terms. Package license tags enumerate declared terms, not a statement that all alternatives are simultaneously required. There has been no comprehensive legal approval. Root/third-party notices and provenance must accompany redistribution; system libraries/toolchain require separate consideration.

## Release gates

Do not release this experiment yet. Remaining gates include complete distribution/legal review, real maintainer metadata, new rosdep keys and an actual ROS buildfarm run, and ARM compilation/runtime validation. Source archive is not a frozen wheel and no prebuilt extension is shipped in this source package.

## Local test evidence

First ament build using the isolated official Rust1.85 tar toolchain succeeded in 57.3s. CTest ran installed-prefix box/equality/infeasible/dense-faer contracts: 1/1 passed. Sourcing its install setup imported this prefix while retaining system NumPy1.21.5. Configuration with extracted Cargo1.75 was correctly rejected. Tests do not constitute release approval.

`TRILAYER_CLARABEL_TOOLCHAIN_LIBRARY_PATH` optionally supplies an explicit shared-library search path for extracted official deb toolchains. It is normally empty for properly installed system packages; it never modifies global linker configuration.

### Official Jammy versioned toolchain validation

Four packages (`rustc-1.85`, `cargo-1.85`, `libstd-rust-1.85`, `libstd-rust-1.85-dev`) version `1.85.1+dfsg0ubuntu2~bpo0-0ubuntu1.22.04.1` matched apt-cache official SHA256 metadata and were extracted, not installed. A fresh independent colcon build completed in 1m19s; installed-prefix CTest passed and all four frozen parity cases had solution-vector difference 0.0, including faer. `provenance/JAMMY_APT_METADATA.json` and `JAMMY_PARITY_COMPARISON.json` retain evidence. This removes the assumption that Jammy has no adequate official compiler, but does not imply new rosdep keys exist or the ROS buildfarm release has succeeded.

For the extracted-deb test only, executable overrides pointed into `root/usr/lib/rust-1.85/bin/`, with an explicit library path `root/usr/lib/rust-1.85/lib`. The extracted `usr/bin/cargo-1.85` wrapper hardcodes `/usr/lib` and therefore must not be used as though installed. Normal apt-installed versioned binaries do not need this extraction workaround.

## Build-tree versus installed tests

Default CTest validates the newly built `vendor_work/python` modules and checks
their origin before installation; it does not need this package preinstalled.
An explicit post-install smoke test remains available by configuring
`-DTRILAYER_VENDOR_TEST_INSTALLED=ON`, then installing before that test runs.
The default OFF setting disables only the additional installed-layout check,
not numerical solver coverage. Do not enable it in a pre-install Debian build
phase. Source provenance and solver algorithms are unchanged.
