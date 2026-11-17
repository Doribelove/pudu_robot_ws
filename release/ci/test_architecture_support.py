#!/usr/bin/env python3
"""Static/unit checks only: no solver build, apt setup, ROS or remote workflow."""
import ast,hashlib,importlib.util,json,unittest
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[2]
class ArchitectureSupport(unittest.TestCase):
 def test_osqp_platform_boundary(self):
  path=ROOT/'src/trilayer_osqp_vendor/scripts/build_vendor.py'
  spec=importlib.util.spec_from_file_location('vendor_platform_review',path)
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  for machine in ('x86_64','aarch64'):module.validate_platform('Linux',machine)
  for system,machine in [('Windows','x86_64'),('Darwin','aarch64'),('Linux','armv7l'),('Linux','ppc64le'),('Linux','AMD64')]:
   with self.assertRaises(RuntimeError):module.validate_platform(system,machine)
 def test_native_matrix_and_artifacts(self):
  workflow=yaml.load((ROOT/'release/ci/trilayer-humble.yml').read_text(),Loader=yaml.BaseLoader)
  job=workflow['jobs']['humble'];rows=job['strategy']['matrix']['include']
  self.assertEqual({(r['architecture'],r['runner']) for r in rows},{('amd64','ubuntu-22.04'),('arm64','ubuntu-22.04-arm')})
  self.assertEqual(job['strategy']['fail-fast'],'false')
  self.assertEqual(job['runs-on'],'${{ matrix.runner }}')
  self.assertEqual(workflow['permissions'],{'contents':'read'})
  artifact=next(x for x in job['steps'] if x.get('uses','').startswith('actions/upload-artifact@'))['with']['name']
  names={artifact.replace('${{ matrix.architecture }}',r['architecture']) for r in rows}
  self.assertEqual(len(names),2);self.assertIn('${{ github.run_attempt }}',artifact)
 def test_evidence_scope_and_manifest_source_hashes(self):
  proof=json.loads((ROOT/'release/ci/ARCHITECTURE_VALIDATION.json').read_text())
  self.assertEqual(proof['arm64']['commit'],'322401744b079e11ced8bf54f69785147d0b6d5a')
  self.assertEqual(proof['arm64']['tests_passed'],47)
  self.assertEqual(proof['arm64']['uname_machine'],'aarch64')
  self.assertEqual(proof['arm64']['dpkg_architecture'],'arm64')
  self.assertIn('Not Horizon J6M/BPU validation',proof['limits'])
  manifest=json.loads((ROOT/'src/trilayer_clarabel_vendor/SOURCE_MANIFEST.json').read_text())
  self.assertEqual(hashlib.sha256((ROOT/'src/trilayer_clarabel_vendor/sources'/manifest['source_archive']).read_bytes()).hexdigest(),manifest['sha256'])
  self.assertEqual(set(manifest['tested_targets']),{'x86_64-unknown-linux-gnu','aarch64-unknown-linux-gnu'})
 def test_build_provenance_does_not_claim_runtime_execution(self):
  script=(ROOT/'src/trilayer_clarabel_vendor/scripts/build_vendor.py').read_text()
  tree=ast.parse(script)
  proof=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='proof' for t in n.targets))
  values={k.value:v for k,v in zip(proof.keys,proof.values) if isinstance(k,ast.Constant)}
  self.assertIs(values['runtime_tests_performed_by_build'].value,False)
  self.assertNotIn('ARM_validation',values)
 def test_ci_scripts_restrict_to_two_architectures(self):
  for name in ('install-deps.sh','build-and-test.sh'):
   content=(ROOT/'release/ci'/name).read_text()
   self.assertIn('case "$(uname -m)" in x86_64|aarch64)',content)
if __name__=='__main__':unittest.main(verbosity=2)
