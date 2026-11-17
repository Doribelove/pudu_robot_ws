# TriLayer native helpers (P6 subset)

This package compiles three unchanged project-authored P6 C++ sources into
private shared libraries under `lib/trilayer_native_helpers`. It requires a C++17
compiler and ament_cmake; it does not require the historical workspace, binary
wheels, NumPy, a map registry, or a robot connection.

- `libv13_dstar.so`: resumable D* repair and affected-rhs refresh.
- `libv13_segment_cells.so`: order-preserving segment cell enumeration.
- `libv13_extract_path.so`: directed-graph path extraction.

See `SOURCE_PROVENANCE.json` for the exact upstream source hashes. Preserve
`-ffp-contract=off -fno-fast-math`; both affect numerical behavior.

Tests exercise 100 deterministic randomized segment cases, a small directed
path and blocked case, and D* cold repair/unreachable/deadline behavior with both
heap modes. The same tests pass against the historical binary and rebuilt binary.
These are small helper-level tests, not the 100Q P6 navigation regression.

The original C ABI remains an internal trusted-input interface: callers must
supply valid array lengths and valid graph indices. This package does not expose
an untrusted network API. The full P6 Python integration and L1/L3 remain separate
porting work. The provisional manifest maintainer must be replaced with the
actual public release contact before Bloom publication.
