# SPDX-License-Identifier: Apache-2.0
"""Check the Python environment actually selected inside the demo namespace."""
from importlib import import_module, metadata
import json
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parent
SITE_PACKAGES = Path('/venv/lib/python3.10/site-packages')
IMPORT_NAMES = {'Pillow': 'PIL', 'PyYAML': 'yaml', 'opencv-python-headless': 'cv2'}


def verify_environment(requirements=ROOT / 'requirements.lock', site_packages=SITE_PACKAGES):
    if sys.version_info[:2] != (3, 10):
        raise ValueError('PYTHON_ENVIRONMENT_MISMATCH: Python 3.10 is required')
    site_packages = Path(site_packages).resolve()
    packages, errors = {}, []
    for line in Path(requirements).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, pinned = line.split('==', 1)
        expected = pinned.split()[0]
        try:
            distribution = metadata.distribution(name)
        except metadata.PackageNotFoundError:
            errors.append(f'{name}: missing distribution (expected {expected})')
            continue
        if distribution.version != expected:
            errors.append(f'{name}: expected {expected}, found {distribution.version}')
            continue
        if Path(distribution.locate_file('')).resolve() != site_packages:
            errors.append(f'{name}: distribution is outside the selected venv')
            continue
        module_name = IMPORT_NAMES.get(name, name.replace('-', '_'))
        try:
            module = import_module(module_name)
        except Exception as error:
            errors.append(f'{name}: import failed: {error}')
            continue
        source = getattr(module, '__file__', None)
        if source is None or not Path(source).resolve().is_relative_to(site_packages):
            errors.append(f'{name}: imported module is outside the selected venv: {source}')
            continue
        packages[name] = {'version': distribution.version, 'module': module_name, 'file': source}
    if errors:
        raise ValueError('PYTHON_ENVIRONMENT_MISMATCH: ' + '; '.join(errors))
    return {'python': platform.python_version(), 'packages': packages}


if __name__ == '__main__':
    print(json.dumps(verify_environment(), indent=2))
