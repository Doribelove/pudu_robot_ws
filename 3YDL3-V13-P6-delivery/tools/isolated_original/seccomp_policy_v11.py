"""Review-only seccomp generator; no execution at import."""
import ctypes,errno,os,fcntl
def export_filter():
 lib=ctypes.CDLL('libseccomp.so.2',use_errno=True)
 lib.seccomp_init.argtypes=[ctypes.c_uint32];lib.seccomp_init.restype=ctypes.c_void_p
 lib.seccomp_syscall_resolve_name.argtypes=[ctypes.c_char_p];lib.seccomp_syscall_resolve_name.restype=ctypes.c_int
 lib.seccomp_rule_add.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_int,ctypes.c_uint]
 lib.seccomp_export_bpf.argtypes=[ctypes.c_void_p,ctypes.c_int]
 lib.seccomp_release.argtypes=[ctypes.c_void_p]
 class Cmp(ctypes.Structure):_fields_=[('arg',ctypes.c_uint),('op',ctypes.c_uint),('a',ctypes.c_uint64),('b',ctypes.c_uint64)]
 lib.seccomp_rule_add_array.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.c_int,ctypes.c_uint,ctypes.POINTER(Cmp)]
 ctx=lib.seccomp_init(0x7fff0000)
 blocked=[]
 for name in ('mount','umount2','unshare','setns','pivot_root','fsopen','fsconfig','fsmount','fspick','open_tree','move_mount','mount_setattr'):
  n=lib.seccomp_syscall_resolve_name(name.encode())
  if n>=0:
   assert lib.seccomp_rule_add(ctx,0x50000|errno.EPERM,n,0)==0;blocked.append(name)
 n=lib.seccomp_syscall_resolve_name(b'clone3')
 if n>=0:assert lib.seccomp_rule_add(ctx,0x50000|errno.ENOSYS,n,0)==0
 # Forbid every namespace-creating clone; ordinary subprocess/thread clone allowed.
 for bit in (0x00020000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x00000080):
  cmp=Cmp(0,7,bit,bit);assert lib.seccomp_rule_add_array(ctx,0x50000|errno.EPERM,lib.seccomp_syscall_resolve_name(b'clone'),1,ctypes.byref(cmp))==0
 # No host pathname/abstract Unix IPC, even through read-only mounted socket paths.
 cmp=Cmp(0,4,1,0);assert lib.seccomp_rule_add_array(ctx,0x50000|errno.EPERM,lib.seccomp_syscall_resolve_name(b'socket'),1,ctypes.byref(cmp))==0
 fd=os.memfd_create('a2b_seccomp',os.MFD_ALLOW_SEALING);assert lib.seccomp_export_bpf(ctx,fd)==0;lib.seccomp_release(ctx);os.lseek(fd,0,0)
 fcntl.fcntl(fd,fcntl.F_ADD_SEALS,fcntl.F_SEAL_WRITE|fcntl.F_SEAL_GROW|fcntl.F_SEAL_SHRINK|fcntl.F_SEAL_SEAL)
 return fd,blocked
