# SPDX-License-Identifier: Apache-2.0
"""Read-only free-space guard for this service's own output filesystem."""
import shutil
class DiskSpaceLow(RuntimeError):
    def __init__(self,free,required):
        self.free_bytes=free;self.required_bytes=required
        super().__init__('DISK_SPACE_LOW: free_bytes=%d required_bytes=%d'%(free,required))

def require_space(output,min_free_mib=512):
    if type(min_free_mib) is not int or min_free_mib<512:raise ValueError('min_free_mib must be an integer at least 512')
    free=shutil.disk_usage(output).free
    required=min_free_mib*1024*1024
    if free<required:raise DiskSpaceLow(free,required)
    return {'free_bytes':free,'required_bytes':required}

def dispatch_checked(backend,request,output,min_free_mib):
    # Always allow status/close; neither accepts new planner work.
    if request['op'] in ('plan','update'):require_space(output,min_free_mib)
    return backend.dispatch(request)
