"""Diagnostic-only exact road field oracle; excluded from formal timing."""
import hashlib,json
from pathlib import Path

def install(destination):
    from three_yd_v5 import r4_fields
    original=r4_fields.road_fields;out=Path(destination);out.parent.mkdir(parents=True,exist_ok=True)
    def checked(controller,view,bbox,chunk=65536):
        actual=original(controller,view,bbox,chunk)
        if not controller.generic_mode:
            expected=r4_fields._road_fields_original(controller,view,bbox,chunk)
            hashes=[]
            for a,b in zip(actual,expected):
                assert a.shape==b.shape and a.dtype==b.dtype
                ha=hashlib.sha256(a).hexdigest();hb=hashlib.sha256(b).hexdigest()
                assert ha==hb,('R6_ROAD_FIELD_DIFFERENCE',bbox)
                hashes.append(ha)
            with out.open('a') as f:f.write(json.dumps({'bbox':bbox,'shape':list(actual[0].shape),'sha256':hashes,'exact_original_field_bytes':True})+'\n')
        return actual
    r4_fields.road_fields=checked
