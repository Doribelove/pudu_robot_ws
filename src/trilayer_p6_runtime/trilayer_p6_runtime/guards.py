"""Reject unsupported research entrypoints before any launch or private-path I/O.

These guards do not substitute results or change selected production geometry.
Frozen source files stay byte-identical; activation binds supported behavior.
"""
import importlib

class UnsupportedResearchFeature(RuntimeError):
    pass

# Only entrypoints with research-only map/report defaults are restricted.
BLOCKED = {
    'arena_evaluation.two_layer_v1_formal_benchmark': (
        '_set_map', '_source_manifest', '_report', 'run_formal', 'build_parser', 'main'),
    'arena_evaluation.l1_l3_corridor_hybrid_validity': (
        '_load_tasks', '_context', '_source_commit', '_source_manifest', '_load_topology',
        '_run_attempt', '_finalize_root_summary', 'build_parser', 'main'),
    'arena_evaluation.layered_architecture_paired_benchmark': (
        '_configure_map', '_load_tasks', '_context', '_source_commit', '_source_manifest',
        '_historical_reference_stats', '_worker', '_paired_summary', 'build_parser', 'main'),
}

def _reject(label):
    def rejected(*args, **kwargs):
        raise UnsupportedResearchFeature(
            label + ' is a research-only entrypoint unavailable in the portable P6 API; '
            'use plan_map/plan_semantic with explicit input and output paths')
    rejected.__name__ = label.rsplit('.', 1)[-1]
    rejected._trilayer_unsupported = True
    return rejected

def install_runtime_guards():
    modules = {name: importlib.import_module(name) for name in BLOCKED}
    adaptive = modules['arena_evaluation.two_layer_v1_formal_benchmark']
    pure_function = adaptive.build_adaptive_corridor_mask
    for name, names in BLOCKED.items():
        module = modules[name]
        for function in names:
            if hasattr(module, function):
                setattr(module, function, _reject(name + '.' + function))
    # Fail at construction, before creating a template/delegate or starting ROS.
    crop = importlib.import_module('three_yd_v5.r11_crop')
    crop.StorageCropSession.__init__ = _reject('StorageCropSession')
    assert adaptive.build_adaptive_corridor_mask is pure_function
    return {'storage_crop': 'rejected_before_construction',
            'benchmark_entrypoints': 'rejected',
            'adaptive_corridor_function_unchanged': True}
