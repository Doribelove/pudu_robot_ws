# TriLayer delta codec

Standalone encoder only, not a navigation node or planner. Uses the unchanged
P6 tile snapshot encoder with SHA-256 and OpenSSL. No stock Nav2 package override.

```python
from trilayer_delta_codec import DeltaCodec
with DeltaCodec(epoch=123) as state:
    packet = state.encode(bytes([254])*256, 16, 16, identity=1,
                          token='a'*32, want_epoch=0, base=0, sparse_full=True)
```

`DeltaCodec(epoch, libpath=None)` finds its private library with ament_index when
installed; `libpath` supports isolated build tests. `encode(data,w,h,identity,
token,want_epoch,base,sparse_full=False)` returns immutable packet bytes. `data`
must be bytes, dimensions positive and product at most 400,000,000. Token must
be 32 lowercase hexadecimal characters. Identity and epoch/base are unsigned
64-bit. State epoch must be nonzero for P6 decoder compatibility; requested
`want_epoch=0` is valid for the initial full request. Wrong epoch/base, changed owner/identity/dimensions force a full reset;
they are not errors. Invalid data raises ValueError; native/closed-state errors
raise RuntimeError. `close()` is idempotent. Calls on one object are serialized.

C ABI exports `tl_delta_create`, `tl_delta_encode`, `tl_delta_close`,
`tl_delta_free`, `tl_delta_error`. Handles are registry integers, not raw object
pointers. Every successful encoded buffer must be returned to `tl_delta_free`;
unknown/double-freed buffers are rejected. Exceptions are caught. As with any C
ABI, pointer arguments must reference valid readable/writable regions of their
declared size; use the Python interface for untrusted application inputs.

The package's maintainer address is a placeholder and must be replaced before
release. This package alone does not prove P6 runtime or bridge compatibility.

Build with colcon, then run `python3 test/test_codec.py [path/to/library.so]`.
Tests decode and hash-check packets independently; they do not run a robot.
