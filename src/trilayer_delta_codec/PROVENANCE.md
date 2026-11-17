# Source provenance

`src/delta_snapshot.hpp` is copied byte-for-byte from the frozen public P6 source:
`3YDL3-V13-P6-delivery/runtime/3YDL3-V13-L2/base/runtime/base/native/delta_snapshot.hpp`
in `Doribelove/pudu_robot_ws` at `a98a639a8ac9b7b50c14f8a69c393706db4cc3ae`.

SHA-256: `7715d95339251e27f43acb51f3b15be60d6c65c8638965f3b746b7d9f489e0c9`.

It is also identical to the helper used by the historical modified Nav2 costmap.
This is the owner's original protocol helper, released under Apache-2.0 with the
owner's authorization. No entire Nav2 package or third-party source is copied
or relicensed here. Wrapper and tests are Apache-2.0. OpenSSL is a dynamically
linked system dependency, with its own distribution license.

Protocol implementation remains unchanged. The wrapper validates input lengths,
integer ranges, token syntax and handle lifecycle, and serializes native calls.
