# SPDX-License-Identifier: Apache-2.0
import ctypes as C
import os
import threading


def _uint(value, bits, name):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < 2**bits:
        raise ValueError(name + ' must be uint' + str(bits))
    return value


class DeltaCodec:
    def __init__(self, epoch, libpath=None):
        _uint(epoch, 64, 'epoch')
        if epoch == 0: raise ValueError('epoch must be nonzero')
        if libpath is None:
            from ament_index_python.packages import get_package_prefix
            libpath = os.path.join(get_package_prefix('trilayer_delta_codec'),
                                   'lib', 'trilayer_delta_codec', 'libtrilayer_delta_codec.so')
        self._lib = C.CDLL(os.fspath(libpath))
        self._lock = threading.Lock()
        self._handle = C.c_uint64()
        lib = self._lib
        lib.tl_delta_create.argtypes = [C.c_uint64, C.POINTER(C.c_uint64)]
        lib.tl_delta_create.restype = C.c_int
        lib.tl_delta_close.argtypes = [C.c_uint64]; lib.tl_delta_close.restype = C.c_int
        lib.tl_delta_free.argtypes = [C.c_void_p]; lib.tl_delta_free.restype = C.c_int
        lib.tl_delta_error.argtypes = []; lib.tl_delta_error.restype = C.c_char_p
        lib.tl_delta_encode.argtypes = [C.c_uint64,C.c_void_p,C.c_size_t,C.c_uint32,C.c_uint32,
            C.c_uint64,C.c_char_p,C.c_size_t,C.c_uint64,C.c_uint64,C.c_int,
            C.POINTER(C.c_void_p),C.POINTER(C.c_size_t)]
        lib.tl_delta_encode.restype = C.c_int
        self._check(lib.tl_delta_create(epoch, C.byref(self._handle)))

    def _check(self, code):
        if code:
            raise RuntimeError(self._lib.tl_delta_error().decode('utf-8', errors='replace'))

    def encode(self, data, w, h, identity, token, want_epoch, base, sparse_full=False):
        for value, bits, name in ((w,32,'w'),(h,32,'h'),(identity,64,'identity'),
                                  (want_epoch,64,'want_epoch'),(base,64,'base')):
            _uint(value,bits,name)
        if not isinstance(data, bytes):
            raise ValueError('data must be bytes')
        if not w or not h or w*h > 400000000 or len(data) != w*h:
            raise ValueError('invalid dimensions/data length')
        if not isinstance(token,str) or len(token)!=32 or any(c not in '0123456789abcdef' for c in token):
            raise ValueError('token must be 32 lowercase hexadecimal characters')
        if not isinstance(sparse_full,bool):
            raise ValueError('sparse_full must be bool')
        with self._lock:
            if not self._handle.value: raise RuntimeError('codec is closed')
            output=C.c_void_p(); length=C.c_size_t()
            self._check(self._lib.tl_delta_encode(self._handle,data,len(data),w,h,identity,
                token.encode('ascii'),32,want_epoch,base,int(sparse_full),C.byref(output),C.byref(length)))
            try: return C.string_at(output,length.value)
            finally: self._check(self._lib.tl_delta_free(output))

    def close(self):
        with self._lock:
            if self._handle.value:
                self._check(self._lib.tl_delta_close(self._handle)); self._handle.value=0

    def __enter__(self): return self
    def __exit__(self,*args): self.close()
    def __del__(self):
        try: self.close()
        except Exception: pass
