"""Versioned P0 contracts. Validation makes no planner/safety decisions."""
from dataclasses import asdict, dataclass
from enum import Enum
import math

SCHEMA = '3YDL3-V12-r0-contracts-v1'


class FailureCode(str, Enum):
    INVALID_INPUT = 'V12_INVALID_INPUT'
    TILE_MISSING = 'V12_TILE_MISSING'
    TILE_CORRUPT = 'V12_TILE_CORRUPT'
    TILE_BUDGET = 'V12_TILE_BUDGET'
    INDEX_MISSING = 'V12_INDEX_MISSING'
    INDEX_STALE = 'V12_INDEX_STALE'
    INDEX_CORRUPT = 'V12_INDEX_CORRUPT'
    L1_NO_ROUTE = 'V12_L1_NO_ROUTE'
    L2_NO_ROUTE = 'V12_L2_NO_ROUTE'
    L3_NO_SOLUTION = 'V12_L3_NO_SOLUTION'
    REQUEST_DEADLINE = 'V12_REQUEST_DEADLINE'
    ACK_MISMATCH = 'V12_ACK_MISMATCH'
    CERTIFICATE_REJECTED = 'V12_CERTIFICATE_REJECTED'
    BINDING_MISMATCH = 'V12_BINDING_MISMATCH'
    INFRASTRUCTURE_ERROR = 'V12_INFRASTRUCTURE_ERROR'
    UNCLASSIFIED_FAILURE = 'V12_UNCLASSIFIED_FAILURE'


class ContractError(ValueError):
    def __init__(self, code, detail):
        self.code = FailureCode(code)
        super().__init__(self.code.value + ':' + str(detail))


@dataclass(frozen=True)
class TileSpec:
    map_sha256: str
    shape: tuple
    resolution_m: float
    origin: tuple
    tile_cells: int = 256
    cache_budget_bytes: int = 8 * 2**20

    def __post_init__(self):
        if (len(self.map_sha256) != 64 or any(c not in '0123456789abcdef' for c in self.map_sha256)
                or len(self.shape) != 2 or any(type(v) is not int or v <= 0 for v in self.shape)
                or not math.isfinite(self.resolution_m) or self.resolution_m <= 0
                or len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin)
                or abs(self.origin[2]) > 1e-12
                or type(self.tile_cells) is not int or self.tile_cells < 1
                or type(self.cache_budget_bytes) is not int or self.cache_budget_bytes < self.tile_cells**2):
            raise ContractError(FailureCode.INVALID_INPUT, 'tile specification')


@dataclass(frozen=True)
class RegionPolicy:
    region_id: str
    mode: str = 'none'
    weight: float = 0.
    side: str | None = None
    desired_clearance_m: float | None = None
    wall_id: str | None = None
    hard_traffic_policy_id: str = 'inherited'

    def __post_init__(self):
        if (not self.region_id or self.mode not in ('none', 'center', 'edge')
                or not math.isfinite(self.weight) or self.weight < 0
                or (self.mode == 'none' and self.weight != 0)
                or (self.mode == 'edge' and (self.side not in ('left', 'right') or not self.wall_id
                    or self.desired_clearance_m is None or not math.isfinite(self.desired_clearance_m)
                    or self.desired_clearance_m <= 0))):
            raise ContractError(FailureCode.INVALID_INPUT, 'region policy')


@dataclass(frozen=True)
class SourceSpan:
    u0: float
    u1: float
    generator: str
    topology_ref: str
    grid_ref: str
    policy_ref: str
    attempt_id: str
    certificate_ref: str
    region_ids: tuple = ()


def validate_spans(spans, domain):
    """Half-open interiors, shared endpoints; contributing layers are separate."""
    if not spans or not all(math.isfinite(x) for x in domain) or domain[0] >= domain[1]:
        raise ContractError(FailureCode.INVALID_INPUT, 'empty provenance/domain')
    previous = domain[0]
    for span in spans:
        if (not math.isfinite(span.u0) or not math.isfinite(span.u1) or span.u1 <= span.u0
                or span.u0 != previous or span.generator not in ('topology', 'grid', 'kinematic_curve')
                or any(not getattr(span, key) for key in ('topology_ref', 'grid_ref', 'policy_ref',
                                                         'attempt_id', 'certificate_ref'))):
            raise ContractError(FailureCode.INVALID_INPUT, 'gap/overlap/invalid source span')
        previous = span.u1
    if previous != domain[1]:
        raise ContractError(FailureCode.INVALID_INPUT, 'provenance domain not covered')
    return {'schema': SCHEMA, 'domain': list(domain), 'spans': [asdict(x) for x in spans]}


def failure_record(code, *, stage, attempt_id, detail, elapsed_s, remaining_s,
                   legacy_code=None):
    code = FailureCode(code)
    if not stage or not attempt_id or not all(math.isfinite(x) and x >= 0 for x in (elapsed_s, remaining_s)):
        raise ContractError(FailureCode.INVALID_INPUT, 'failure record')
    return {'schema': SCHEMA, 'failure_code': code.value, 'stage': stage,
            'attempt_id': attempt_id, 'detail': str(detail), 'elapsed_s': elapsed_s,
            'remaining_s': remaining_s, 'legacy_code': legacy_code,
            'infrastructure_failure': code in (FailureCode.INVALID_INPUT, FailureCode.TILE_MISSING,
                FailureCode.TILE_CORRUPT, FailureCode.TILE_BUDGET, FailureCode.INDEX_MISSING,
                FailureCode.INDEX_STALE, FailureCode.INDEX_CORRUPT, FailureCode.ACK_MISMATCH,
                FailureCode.BINDING_MISMATCH, FailureCode.INFRASTRUCTURE_ERROR)}


def planner_failure_record(legacy_code, *, stage, attempt_id, elapsed_s, remaining_s):
    """Lossless wire record: a bounded search rejection is not a no-solution proof."""
    text=str(legacy_code);upper=text.upper();code=None
    for known in FailureCode:
        if upper==known.value or upper.startswith(known.value+':'):
            code=known;break
    if code is None:
        if 'ACK' in upper:code=FailureCode.ACK_MISMATCH
        elif any(x in upper for x in ('BINDING','VERSION','CONTENT','FRAME_MISMATCH')):
            code=FailureCode.BINDING_MISMATCH
        elif any(x in upper for x in ('TOTAL_REQUEST','REQUEST_DEADLINE','QUERYBUDGET')):
            code=FailureCode.REQUEST_DEADLINE
        elif stage=='infrastructure':code=FailureCode.INFRASTRUCTURE_ERROR
        elif stage=='L1':code=FailureCode.L1_NO_ROUTE
        elif stage=='L2':code=FailureCode.L2_NO_ROUTE
        elif any(x in upper for x in ('CERTIFICATE','SWEPT_BODY','FOOTPRINT','STEERING')):
            code=FailureCode.CERTIFICATE_REJECTED
        elif stage=='L3':code=FailureCode.L3_NO_SOLUTION
        else:code=FailureCode.UNCLASSIFIED_FAILURE
    out=failure_record(code,stage=stage,attempt_id=attempt_id,detail=text,
        elapsed_s=elapsed_s,remaining_s=remaining_s,legacy_code=text)
    out['physical_infeasibility_proven']=False
    return out
