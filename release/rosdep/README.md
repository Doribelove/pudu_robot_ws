# TriLayer rosdep proposal — prepared, not submitted

Checked against a fresh read-only clone of official `ros/rosdistro`, commit `677deb8b83f1996f93227608bb95f8e7f122b8e2`, on 2026-11-17. No global rosdep rules, user caches, apt installations, remote branches or PRs were changed.

## Actual missing rules and alternatives

| Requested dependency | Fresh official database | Proposed action |
|---|---|---|
| setuptools-scm Python module | No setuptools-scm/python3-setuptools-scm alias found | Add **python3-setuptools-scm** to python.yaml |
| pybind11 Python module | No python3-pybind11 key; **pybind11-dev** exists but Ubuntu maps only headers | Add **python3-pybind11** to python.yaml |
| pinned Rust1.85 compiler | rustc exists but maps unversioned rustc; no rustc-1.85 | Optional pinned key in base.yaml |
| pinned Cargo1.85 | cargo exists; **cargo-latest already exists**, Jammy maps cargo-1.91; no cargo-1.85 | Optional pinned key, only if keeping validated1.85 line |

`cargo-latest` is a real alternative, not a missing rule to invent. Its current Jammy package has rustc-1.91 (or exactly corresponding rustc) as an apt dependency. Switching could avoid both new Rust rules, but would require discovering versioned1.91 executables and rerunning the frozen solver regression. Do not silently switch from the tested1.85 toolchain just to reduce key count. No Rust1.91 solver build has been performed in this proposal.

Upstream [contribution rules](https://github.com/ros/rosdistro/blob/677deb8b83f1996f93227608bb95f8e7f122b8e2/CONTRIBUTING.md#guidelines-for-rosdep-rules) require Python3 keys prefixed `python3-` in python.yaml, native packages, minimal notation, sorted keys and no blank lines. Thus use `python3-setuptools-scm`, not a new unprefixed `setuptools-scm` key.

## Two independent patches

- `01-python-build-dependencies.patch`: adds two generic Debian/Ubuntu native package mappings (6 lines), supported by official package listings across currently listed suites.
- `02-optional-pinned-rust-1.85.patch`: adds only Jammy/Noble versioned mappings (6 lines). Official Ubuntu search lists1.85 packages in those two releases, not newer releases. No unsupported default/future mapping or fictitious Debian versioned package is invented. This is a **review proposal**, not guaranteed acceptable upstream: limited distro coverage and reuse of cargo-latest should be discussed explicitly.

Both apply to the pinned commit independently. The `rosdistro/` checkout has both proposals applied, without commits or remote writes. Rebase/check for newly added keys before submitting.

## Official package availability

Fresh HTML pages are retained under `evidence/`; `OFFICIAL_PACKAGE_EVIDENCE.json` records URLs, SHA256, versions and architecture tables.

- [python3-setuptools-scm](https://packages.ubuntu.com/jammy/python3-setuptools-scm):6.4.2-1, universe, Architecture **all**. Applies to amd64/arm64 installations.
- [python3-pybind11](https://packages.ubuntu.com/jammy/python3-pybind11):2.9.1-2, universe, Architecture **all**.
- [rustc-1.85](https://packages.ubuntu.com/jammy-updates/rustc-1.85) and [cargo-1.85](https://packages.ubuntu.com/jammy-updates/cargo-1.85):1.85.1+dfsg0ubuntu2~bpo0-0ubuntu1.22.04.1, universe, explicit **amd64 and arm64** rows.
- [cargo-1.91](https://packages.ubuntu.com/jammy-updates/cargo-1.91):1.91.1+dfsg~22.04-0ubuntu0.22.04.3, universe, explicit amd64/arm64 rows; existing cargo-latest alternative.
- Debian generic Python mappings checked using [setuptools-scm](https://packages.debian.org/trixie/python3-setuptools-scm) and [pybind11](https://packages.debian.org/trixie/python3-pybind11). Ubuntu all-suite searches are retained too.

**Architecture availability is not ARM solver compilation/runtime validation.** Jammy-updates/universe must be available to the build environment. Earlier isolated amd64 validation downloaded and SHA-verified the1.85 debs, but this proposal does not install or fetch ARM binaries.

## Validation

```
/usr/bin/python3 /tmp/trilayer-rosdep-proposal/validate.py
```

Requires existing PyYAML and rosdep Python modules. Uses in-memory rosdep definitions, never `rosdep init/update/install`, never global configuration. Checks proposed-key uniqueness across checkout YAML, Jammy apt resolution, official format checker on changed files, patch reverse-applicability and git whitespace checks. `VALIDATION.json` and `validation.log` record PASS. This is not a full upstream network repo-check or ROS buildfarm run.

`PR_DRAFT.md` provides review text. Nothing has been published.
