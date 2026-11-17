#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Build fixed solver sources offline into an owned CMake binary directory."""
import argparse
import hashlib
import importlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import zipfile


def validate_platform(system,machine):
    if system!='Linux' or machine not in ('x86_64','aarch64'):
        raise RuntimeError('Supported source-build targets: Linux x86_64 and aarch64; board-specific hardware is not validated')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source-root',type=Path,required=True)
    parser.add_argument('--work',type=Path,required=True)
    args=parser.parse_args();root=args.source_root.resolve();work=args.work.resolve()
    validate_platform(platform.system(),platform.machine())
    if work==root or root.is_relative_to(work):raise RuntimeError('work must not contain sources')
    marker=work/'.trilayer_osqp_owned'
    if work.exists() and any(work.iterdir()) and not marker.is_file():
        raise RuntimeError('refusing to clear unowned build directory')
    work.mkdir(parents=True,exist_ok=True);marker.write_text('trilayer_osqp_vendor\n')
    for child in work.iterdir():
        if child==marker:continue
        if child.is_dir() and not child.is_symlink():shutil.rmtree(child)
        else:child.unlink()
    wheels=work/'wheels';stage=work/'python';wheels.mkdir();stage.mkdir()
    # Prevent upstream qdldl setup.py's optional pip fallback before executing it.
    for name in ['numpy','scipy','setuptools','setuptools_scm','wheel','pybind11']:
        importlib.import_module(name)
    packages={name:version(name) for name in ['numpy','scipy','setuptools','setuptools_scm','wheel','pybind11']}
    manifest=json.loads((root/'SOURCES_AND_LICENSES.json').read_text());sources=manifest['sources']
    env={**os.environ,'PYTHONNOUSERSITE':'1','PIP_NO_INDEX':'1','PIP_DISABLE_PIP_VERSION_CHECK':'1'}
    built=[]
    for name in ['qdldl','osqp']:
        info=sources[name];archive=root/'sources'/info['filename']
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=info['sha256']:
            raise RuntimeError('source hash mismatch: '+name)
        prefix=name+'-'+info['version']
        with tarfile.open(archive) as tar:
            for member in tar.getmembers():
                parts=Path(member.name).parts
                if not parts or parts[0]!=prefix or '..' in parts or member.issym() or member.islnk():
                    raise RuntimeError('unexpected archive member: '+member.name)
            tar.extractall(work)
        with (work/(name+'-build.log')).open('w') as log:
            subprocess.run([sys.executable,'setup.py','bdist_wheel','--dist-dir',str(wheels)],
                cwd=work/prefix,env={**env,'SETUPTOOLS_SCM_PRETEND_VERSION':info['version']},
                stdout=log,stderr=subprocess.STDOUT,check=True)
        results=list(wheels.glob(name+'-'+info['version']+'-*.whl'))
        if len(results)!=1:raise RuntimeError('unexpected wheel results for '+name)
        wheel=results[0]
        with zipfile.ZipFile(wheel) as z:
            for member in z.infolist():
                parts=Path(member.filename).parts
                if Path(member.filename).is_absolute() or '..' in parts:
                    raise RuntimeError('unexpected wheel member')
                if any(p.endswith('.data') for p in parts):
                    raise RuntimeError('wheel .data scheme needs explicit install handling')
            z.extractall(stage)
        built.append({'name':name,'version':info['version'],'wheel_sha256':hashlib.sha256(wheel.read_bytes()).hexdigest()})
    native=sorted(str(p.relative_to(stage)) for p in stage.rglob('*.so'))
    if len(native)!=2:raise RuntimeError('expected exactly two solver native extensions: '+repr(native))
    record={'schema':'trilayer-osqp-vendor-source-build-v1','architecture':platform.machine(),
            'python':platform.python_version(),'build_packages':packages,'sources':sources,
            'built':built,'native_modules':native,'network_used_during_compile':False,
            'build_method':'original setup.py bdist_wheel; distribution-targeted NumPy ABI; fixed SCM version'}
    (work/'complete.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
