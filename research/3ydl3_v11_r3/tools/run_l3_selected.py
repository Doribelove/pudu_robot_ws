"""3YDL3-V8 default independent entry; inherits saved ASCHNT+OGR+PLMX."""
import os,sys
from pathlib import Path
a=sys.argv[1:]
if not any(v=='--l3-backend' or v.startswith('--l3-backend=') for v in a):a=['--l3-backend','independent',*a]
os.execv(sys.executable,[sys.executable,str(Path(__file__).with_name('run_v9_selected.py')),*a])
