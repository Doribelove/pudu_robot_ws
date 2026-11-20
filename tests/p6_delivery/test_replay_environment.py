import os
from pathlib import Path
import runpy
import subprocess
import tempfile
import unittest
from unittest.mock import patch


DELIVERY = Path(__file__).resolve().parents[2] / '3YDL3-V13-P6-delivery'


@unittest.skipUnless(os.name == 'posix', 'P6 replay uses POSIX and Bash')
class ReplayEnvironmentTests(unittest.TestCase):
    def test_replay_sources_selected_base_environment(self):
        runner = runpy.run_path(str(DELIVERY / 'tools/run_static_regression.py'))
        package = DELIVERY / 'runtime/3YDL3-V13-L2'
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp) / 'custom base.bash'
            base.write_text('export P6_TEST_BASE=loaded\n', encoding='utf-8')
            with patch.dict(os.environ, V13_RUNTIME_ENV=str(base), V13_PROFILE='p7-r1'):
                env = runner['environment'](package)
                self.assertEqual(env.get('V13_RUNTIME_ENV'), str(base))
                command = runner['shell'](package, ['bash', '-c', 'echo "$P6_TEST_BASE $V13_PROFILE"'])
                result = subprocess.run(['bash', '-c', command], env=env, text=True,
                                        capture_output=True, check=True)
        self.assertEqual(result.stdout.strip(), 'loaded p6')
