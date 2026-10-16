"""Diagnostic oracle only: compare full pre-bound bytes to r6 conversion."""
import hashlib,json
from pathlib import Path

def install(destination):
    from three_yd_v5 import tailcost_occupancy
    from three_yd_v5.v9_storage import occupancy_on_free_support
    original=tailcost_occupancy.occupancy_on_free_cells
    out=Path(destination)
    def checked(source,soft,hard,rows=96):
        actual=original(source,soft,hard,rows)
        expected=occupancy_on_free_support(source,soft,hard,rows)
        ha=hashlib.sha256(actual).hexdigest();hb=hashlib.sha256(expected).hexdigest()
        assert actual.shape==expected.shape and actual.dtype==expected.dtype and ha==hb
        with out.open('a') as f:f.write(json.dumps({'shape':list(actual.shape),'cells':actual.size,'sha256':ha,'exact_original_occupancy_bytes':True})+'\n')
        return actual
    tailcost_occupancy.occupancy_on_free_cells=checked
