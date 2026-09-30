"""Build the standalone CPU search kernel; no ROS or other planner rebuild."""
import hashlib
import json
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
source = root / 'native/goal_dijkstra.cpp'
library = root / 'native/libgoal_dijkstra.so'
command = ['g++', '-std=c++17', '-O3', '-fPIC', '-shared',
           '-Wall', '-Wextra', str(source), '-o', str(library)]
subprocess.run(command, check=True)
(root / 'native/BUILD.json').write_text(json.dumps({
    'command': command,
    'compiler': subprocess.check_output(['g++', '--version'], text=True).splitlines()[0],
    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
}, indent=2) + '\n')
print(library)
