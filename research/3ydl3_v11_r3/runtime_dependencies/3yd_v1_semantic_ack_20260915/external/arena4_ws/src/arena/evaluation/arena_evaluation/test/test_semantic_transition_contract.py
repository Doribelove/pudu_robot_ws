import numpy as np
import pytest

from arena_evaluation.semantic_transition_contract import audit_transition_samples, resample_path


def lane_case(length, error=.2, side=True):
    station=np.unique(np.r_[np.arange(0.,length,.025),length])
    n=len(station)
    return dict(path_length_m=length,station_m=station,lane_mask=np.ones(n,bool),
                lane_error_m=np.full(n,error),lane_correct_side=np.full(n,side))


def test_short_and_exact_twelve_use_full_semantics():
    for length in (10.553370568,12.):
        result=audit_transition_samples(**lane_case(length))
        assert result["short_path_full_semantics"]
        assert result["active_interval_m"]==[0.,length]
        assert result["semantic_gate_passed"]
        assert result["active_window"]==result["full_path"]


def test_long_path_reports_distinct_full_and_active_results():
    case=lane_case(20.)
    case["lane_error_m"][:240]=2.
    case["lane_error_m"][561:]=2.
    result=audit_transition_samples(**case)
    assert result["active_interval_m"]==[6.,14.]
    assert result["semantic_gate_passed"]
    assert not result["full_path"]["semantic_gate_passed"]


def test_class_disappearing_in_transition_is_invalid_not_inherited():
    case=lane_case(20.)
    case["lane_mask"][200:]=False
    result=audit_transition_samples(**case)
    assert not result["semantic_gate_passed"]
    assert result["active_window"]["classes"]["lane"]["failure_reason"]=="EMPTY_APPLICABLE_REGION"
    assert result["active_window"]["classes"]["parking"]["contract_metric_status"]=="NOT_APPLICABLE"


def test_no_applicable_classes_is_invalid():
    case=lane_case(10.)
    case["lane_mask"][:]=False
    result=audit_transition_samples(**case)
    assert result["contract_metric_status"]=="INVALID_CONTRACT_METRIC"
    assert not result["semantic_gate_passed"]


def test_parking_denominator_is_separate_and_threshold_strict():
    case=lane_case(.1)
    case["lane_mask"]=np.array([True,True,True,False,False])
    case["parking_mask"]=~case["lane_mask"]
    case["parking_normalized_deviation"]=np.array([np.nan,np.nan,np.nan,.20,.30])
    result=audit_transition_samples(**case)
    assert result["active_window"]["classes"]["lane"]["sample_count"]==3
    parking=result["active_window"]["classes"]["parking"]
    assert parking["sample_count"]==2
    assert parking["parking_center_normalized_deviation_p50"]==.25
    assert parking["parking_center_band_ratio"]==.5
    assert not result["semantic_gate_passed"]
    case["parking_normalized_deviation"][-1]=.25
    assert audit_transition_samples(**case)["semantic_gate_passed"]


def test_nonfinite_applicable_field_is_not_silently_removed():
    case=lane_case(10.)
    case["lane_error_m"][5]=np.nan
    result=audit_transition_samples(**case)
    assert result["contract_metric_status"]=="INVALID_CONTRACT_METRIC"


def test_duplicate_sampling_and_invalid_grid_are_rejected():
    case=lane_case(10.)
    case["station_m"][3]=case["station_m"][2]
    with pytest.raises(ValueError,match="fixed global"):
        audit_transition_samples(**case)


def test_resampler_density_invariance_and_diagnostics_do_not_change_gate():
    sparse=[dict(x=0.,y=0.,yaw=0.),dict(x=10.,y=0.,yaw=0.)]
    dense=[dict(x=x,y=0.,yaw=0.) for x in np.linspace(0.,10.,101)]
    a,sa=resample_path(sparse)
    b,sb=resample_path(dense)
    assert np.array_equal(sa,sb)
    assert a==b
    result=audit_transition_samples(**lane_case(10.),raw_xy=[[0.,0.],[0.,0.],[10.,0.]],baseline_path_length_m=8.)
    assert result["semantic_gate_passed"]
    assert result["manipulation_risk_diagnostics"]["zero_translation_step_count"]==1
    assert result["manipulation_risk_diagnostics"]["path_length_ratio_vs_baseline"]==1.25
