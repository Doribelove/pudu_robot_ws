import json
from types import SimpleNamespace

import pytest

from arena_evaluation import smac_contract_r3 as contract
from arena_evaluation import exact_ack_r3 as ack
from arena_evaluation.unified_four_backends_smoke import FOOTPRINT


def valid_parameters():
    planner={'GridBased.'+k:v for k,v in contract.PLANNER_CONTRACT.items()}
    costmap={**contract.COSTMAP_CONTRACT,'footprint':json.dumps(FOOTPRINT)}
    return planner,costmap


def test_pinned_costmap_float_literal_is_matched_exactly():
    assert contract.COSTMAP_CONTRACT['footprint_padding']==0.009999999776482582
    planner,costmap=valid_parameters()
    assert contract.validate_runtime_contract(planner,costmap,FOOTPRINT,'map')['verified']
    costmap['footprint_padding']=.02
    with pytest.raises(ValueError,match='footprint_padding'):
        contract.validate_runtime_contract(planner,costmap,FOOTPRINT,'map')


@pytest.mark.parametrize('key,value',list(contract.PLANNER_CONTRACT.items()))
def test_explicit_frozen_values_override_unrelated_profile_defaults(key,value):
    result=contract.frozen_planner_overrides({'benchmark_instrumentation':True})
    assert result[key]==value and result['benchmark_instrumentation'] is True


@pytest.mark.parametrize('key,bad',[
    ('angle_quantization_bins',72),('angle_quantization_bins',48.),
    ('minimum_turning_radius',.39),('motion_model_for_search','REEDS_SHEPP'),
    ('allow_unknown',True),('max_iterations',1000001),('downsample_costmap',True),
])
def test_conflicting_override_rejected_before_launch(key,bad):
    with pytest.raises(ValueError,match='SMAC_SAFETY_CONFIG_CONFLICT'):
        contract.frozen_planner_overrides({key:bad})


@pytest.mark.parametrize('section,key,bad',[
    ('planner','GridBased.angle_quantization_bins',72),
    ('planner','GridBased.motion_model_for_search','REEDS_SHEPP'),
    ('planner','GridBased.minimum_turning_radius',.39),
    ('planner','GridBased.allow_unknown',True),
    ('planner','GridBased.max_iterations',1000001),
    ('costmap','resolution',.1),('costmap','footprint_padding',0.),
    ('costmap','track_unknown_space',False),('costmap','footprint','[[0.1,0.1]]'),
])
def test_runtime_mismatch_fails_closed(section,key,bad):
    planner,costmap=valid_parameters();(planner if section=='planner' else costmap)[key]=bad
    with pytest.raises(ValueError,match='RUNTIME_SAFETY_CONTRACT_MISMATCH'):
        contract.validate_runtime_contract(planner,costmap,FOOTPRINT,'map')


def test_runtime_binding_deterministic_and_map_specific():
    planner,costmap=valid_parameters()
    a=contract.validate_runtime_contract(planner,costmap,FOOTPRINT,'one')
    b=contract.validate_runtime_contract(dict(reversed(list(planner.items()))),costmap,FOOTPRINT,'one')
    c=contract.validate_runtime_contract(planner,costmap,FOOTPRINT,'two')
    assert a==b and a['verified'] and a['sha256']!=c['sha256']


@pytest.mark.parametrize('receipt',[None,{}, {'verified':False}, {'verified':True,'map_hash':'wrong'}])
def test_missing_runtime_attestation_cannot_update_map_or_start_search(receipt):
    session=object.__new__(ack.ExactAckSmacSession)
    session.ctx=SimpleNamespace(map_sha256='map');session.request_id='q'
    session.runtime_safety_contract=receipt;session._local_mask_info={}
    session.update_local_mask=lambda _:pytest.fail('Unverified contract reached map publication')
    result=session.plan(None,SimpleNamespace(backend='Smac',version='frozen'),allowed_mask=object())
    assert result.failure_code=='RUNTIME_SAFETY_CONTRACT_REQUIRED'
    assert result.diagnostics['planner_search_started'] is False
