"""Installed resource adapters; never consult a historical workspace path."""
from pathlib import Path

def native_path(filename):
    allowed={'libv13_dstar.so','libv13_segment_cells.so','libv13_extract_path.so'}
    if filename == 'libsemantic_sweep.so':
        from ament_index_python.packages import get_package_prefix
        path=Path(get_package_prefix('trilayer_l3_core'))/'lib/trilayer_l3_core'/filename
        if not path.is_file(): raise RuntimeError('Missing source-built semantic sweep')
        return str(path)
    if filename not in allowed:
        raise RuntimeError('Native helper has not been ported: '+filename)
    from ament_index_python.packages import get_package_prefix
    path=Path(get_package_prefix('trilayer_native_helpers'))/'lib/trilayer_native_helpers'/filename
    if not path.is_file():raise RuntimeError('Missing installed native helper: '+filename)
    return str(path)
