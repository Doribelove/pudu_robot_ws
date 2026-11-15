"""Exercise dependency metadata and real imports without ROS or package installs."""
from pathlib import Path
import shutil
import sys
import sysconfig

import pytest

ROOT = Path(__file__).resolve().parents[2] / 'examples/3ydl3_garage_b'
sys.path.insert(0, str(ROOT))
import check_environment


@pytest.fixture
def dependency(tmp_path, monkeypatch):
    site = tmp_path / 'site-packages'
    site.mkdir()
    info = site / 'demo_dependency-1.2.3.dist-info'
    info.mkdir()
    (info / 'METADATA').write_text('Metadata-Version: 2.1\nName: demo-dependency\nVersion: 1.2.3\n')
    module = site / 'demo_dependency.py'
    module.write_text('value = 42\n')
    requirements = tmp_path / 'requirements.lock'
    requirements.write_text('# Exact test dependency\ndemo-dependency==1.2.3 --hash=sha256:fixture\n')
    monkeypatch.syspath_prepend(str(site))
    yield site, info, module, requirements
    sys.modules.pop('demo_dependency', None)


def test_dependency_from_selected_environment(dependency):
    site, _, module, requirements = dependency
    result = check_environment.verify_environment(requirements, site)
    assert result['packages']['demo-dependency'] == {
        'version': '1.2.3', 'module': 'demo_dependency', 'file': str(module)}


@pytest.mark.parametrize('change, message', [
    ('missing', 'missing distribution'),
    ('version', 'expected 1.2.3, found 1.2.4'),
    ('missing_module', 'import failed'),
    ('module_outside', 'imported module is outside the selected venv'),
    ('metadata_outside', 'distribution is outside the selected venv'),
])
def test_dependency_drift_rejected(tmp_path, monkeypatch, dependency, change, message):
    site, info, module, requirements = dependency
    outside = tmp_path / 'system-packages'
    outside.mkdir()
    if change == 'missing':
        shutil.rmtree(info)
    elif change == 'version':
        (info / 'METADATA').write_text('Metadata-Version: 2.1\nName: demo-dependency\nVersion: 1.2.4\n')
    elif change == 'missing_module':
        module.unlink()
    else:
        shutil.move(str(module if change == 'module_outside' else info), outside)
        monkeypatch.syspath_prepend(str(outside))
    with pytest.raises(ValueError, match=message):
        check_environment.verify_environment(requirements, site)


def test_installed_hash_locked_environment():
    site = Path(sysconfig.get_paths()['purelib'])
    result = check_environment.verify_environment(site_packages=site)
    assert result['packages']['PyYAML']['version'] == '6.0.2'
    assert result['packages']['opencv-python-headless']['module'] == 'cv2'
    assert result['packages']['Pillow']['module'] == 'PIL'
