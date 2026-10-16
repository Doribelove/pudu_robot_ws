from arena_evaluation import layered_2d_v2_pipeline as v2
from arena_evaluation import path_audit


def test_v2_uses_single_shared_canonical_pathaudit_class():
    assert v2.PathAuditor is path_audit.PathAuditor


def test_heading_bin_change_is_only_declared_smac_ablation():
    assert v2.ANGLE_QUANTIZATION_BINS == 48
    assert v2.BASE_PADDING_M == 2.0
    assert v2.TURN_PADDING_M == 4.0
