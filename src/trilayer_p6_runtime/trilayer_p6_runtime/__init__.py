"""Explicit activation of the private, provenance-locked P6 Python source tree."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import importlib
import json
import os
import sys

FAMILIES=('three_yd_v3','three_yd_v4','three_yd_v5','three_yd_v13','arena_3d_v1','arena_evaluation')
LAYERS=('profiles/p6/src','base/src','base/runtime/base/src','base/runtime')

@dataclass(frozen=True)
class RuntimeConfig:
    master_service: str
    cache_root: Path


def _resource_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('trilayer_p6_runtime'))/'resources'


def activate(config, *, resource_root=None):
    """Select P6 source/profile explicitly; do not source a historical env script.

    One P6 instance per interpreter. Existing incompatible historical namespaces
    or algorithm settings cause a failure rather than silently mixing versions.
    This imports classes only and does not launch ROS or instantiate a planner.
    """
    if not isinstance(config,RuntimeConfig):raise TypeError('RuntimeConfig required')
    if not config.master_service.startswith('/') or ' ' in config.master_service:
        raise ValueError('master_service must be an explicit absolute ROS service name')
    cache=Path(config.cache_root)
    if not cache.is_absolute():raise ValueError('cache_root must be absolute')
    root=Path(resource_root).resolve() if resource_root else _resource_directory().resolve()
    tree=root/'p6'
    profile=json.loads((root/'p6_profile.json').read_text())
    for name,module in list(sys.modules.items()):
        if name.split('.')[0] in FAMILIES:
            path=getattr(module,'__file__',None)
            if path and not Path(path).resolve().is_relative_to(tree):
                raise RuntimeError('P6 namespace already loaded outside selected source: '+name)
    bindings={**profile,'TRILAYER_MASTER_SERVICE':config.master_service,
              'TRILAYER_RUNTIME_ROOT':str(tree),
              'THREEYD_V12_INDEX_ROOT':str(cache/'indices'),
              'THREEYD_V12_PLANE_ROOT':str(cache/'planes'),
              'V13_L2_DELIVERY_ROOT':str(tree)}
    for key,value in bindings.items():
        if key in os.environ and os.environ[key]!=value:
            raise RuntimeError('conflicting runtime setting: '+key)
    os.environ.update(bindings)
    selected=[str(tree/p) for p in LAYERS]
    sys.path[:]=selected+[p for p in sys.path if p not in selected]
    # pkgutil.extend_path in historical package initializers would otherwise
    # admit unrelated installed copies. Restrict each package to this overlay.
    for family in FAMILIES:
        locations=[str(tree/layer/family) for layer in LAYERS if (tree/layer/family).is_dir()]
        if locations:
            package=importlib.import_module(family)
            package.__path__=locations
    from three_yd_v13.service import IncrementalService
    from three_yd_v5.curve_session import CurveSession
    import three_yd_v13.schedule
    import three_yd_v5.p6_hint
    from three_yd_v5.curve_backend import settings
    mode,cfg,digest=settings()
    if mode!='independent':raise RuntimeError('P6 independent L3 profile not selected')
    from .guards import install_runtime_guards
    guards=install_runtime_guards()
    sources={}
    for name,module in sys.modules.items():
        path=getattr(module,'__file__',None)
        if name.split('.')[0] in FAMILIES and path:
            p=Path(path).resolve()
            if not p.is_relative_to(tree):raise RuntimeError('external P6 import: '+name)
            sources[name]={'relative_path':str(p.relative_to(tree)),
                           'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
    return {'IncrementalService':IncrementalService,'CurveSession':CurveSession,
            'profile':'p6','arm':'H-I','l3_config_hash':digest,'sources':sources,
            'master_service':config.master_service,'resource_root':str(root),'guards':guards}
