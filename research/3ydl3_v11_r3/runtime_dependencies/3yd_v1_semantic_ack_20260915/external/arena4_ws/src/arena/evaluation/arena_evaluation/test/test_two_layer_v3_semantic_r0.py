from copy import deepcopy

from arena_evaluation import semantic_applicability_v3 as applicability
from arena_evaluation import two_layer_v3_semantic_r0_benchmark as subject


def test_frozen_targeted_query_and_evidence_bindings_pass():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    targeted = subject._load_targeted(subject.DEFAULT_TARGETED, config)
    assert targeted["architecture_id"] == "2A-V3"
    assert targeted["query_hash"] == subject._targeted_hash(targeted["queries"])
    assert [row["query_id"] for row in targeted["queries"]] == [
        "v3-applicable-mirror-positive",
        "r3-mirror-2-negative",
        "cmp2-02-lane-south",
    ]


def test_targeted_hash_is_sensitive_to_endpoint_and_order():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    targeted = subject._load_targeted(subject.DEFAULT_TARGETED, config)
    original = subject._targeted_hash(targeted["queries"])
    changed = deepcopy(targeted["queries"])
    changed[0]["goal"][0] += 0.05
    assert subject._targeted_hash(changed) != original
    assert subject._targeted_hash(list(reversed(targeted["queries"]))) != original


def test_v3_provider_is_explicit_map_cell_provider():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    assert callable(subject._provider(config))


def test_frozen_positive_is_applicability_not_path_acceptance():
    config, _ = subject._load_config(subject.DEFAULT_CONFIG)
    targeted = subject._load_targeted(subject.DEFAULT_TARGETED, config)
    verification = targeted["intent_validation"][0]["verification"]
    assert verification["replayable_forward_only_se2_chain"] is True
    assert verification["deterministic_replay_gate_passed"] is True
    assert "strict_acceptance_gate_passed" not in verification
