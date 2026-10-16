import hashlib
import json

import pytest

from arena_evaluation import semantic_transition_naturalness_aggregate as aggregate


def _write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def _frozen_query(query):
    query_file = aggregate._resolve_frozen_config(
        aggregate.TARGETED_QUERY_SET_FILENAME,
        aggregate.TARGETED_QUERY_SET_SHA256,
    )
    queries, _, _ = aggregate.load_query_set(
        query_file,
        actual_map_hash=aggregate.MAP_HASH,
        actual_semantic_map_hash=aggregate.SEMANTIC_MAP_HASH,
    )
    return next(item.as_dict() for item in queries if item.query_id == query)


def _seal_inputs(tmp_path, monkeypatch):
    inputs = tmp_path / "inputs"
    hashes = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(inputs.iterdir())
        if path.suffix in (".json", ".npz") and path.name != "artifact_hashes.json"
    }
    manifest = inputs / "artifact_hashes.json"
    _write_json(manifest, hashes)
    monkeypatch.setattr(
        aggregate,
        "FROZEN_INPUT_MANIFEST_SHA256",
        hashlib.sha256(manifest.read_bytes()).hexdigest(),
    )


def _fixture(tmp_path, query, passed):
    inputs = tmp_path / "inputs"
    inputs.mkdir(exist_ok=True)
    meta = inputs / f"{query}.json"
    npz = inputs / f"{query}.npz"
    npz.write_bytes((query + "-npz").encode())
    npz_hash = hashlib.sha256(npz.read_bytes()).hexdigest()
    semantic_dir = tmp_path / "conversion_v1"
    semantic_dir.mkdir(exist_ok=True)
    semantic_file = semantic_dir / "semantic_map_v1.json"
    if not semantic_file.exists():
        semantic_file.write_text("{}\n", encoding="utf-8")
    meta_payload = {
        "query": _frozen_query(query),
        "targeted_query_content_hash": aggregate.TARGETED_QUERY_CONTENT_HASH,
        "map_hash": aggregate.MAP_HASH,
        "semantic_map_hash": aggregate.SEMANTIC_MAP_HASH,
        "npz_sha256": npz_hash,
        "expected_master_hash": "master",
        "route_hash": "route",
    }
    _write_json(meta, meta_payload)
    directory = tmp_path / query
    directory.mkdir()
    candidate = tmp_path / f"{query}-candidate"
    candidate.mkdir()
    (candidate / "path.json").write_text("[]\n", encoding="utf-8")
    (candidate / "controls.json").write_text("{}\n", encoding="utf-8")
    verification = {
        "query_id": query,
        "protocol_id": aggregate.VERIFIER_PROTOCOL_ID,
        "contract_revision": aggregate.CONTRACT_REVISION,
        "candidate": str(candidate),
        "gates": {
            "hard_safety_gate_passed": True,
            "targeted_binding_gate_passed": True,
            "r2_semantic_gate_passed": True,
            "naturalness_gate_passed": passed,
            "strict_acceptance_gate_passed": passed,
        },
        "bindings": {
            "input_meta_sha256": hashlib.sha256(meta.read_bytes()).hexdigest(),
            "input_npz_sha256": npz_hash,
            "declared_input_npz_sha256": npz_hash,
            "map_hash": aggregate.MAP_HASH,
            "semantic_map_hash": aggregate.SEMANTIC_MAP_HASH,
            "expected_master_hash": "master",
            "route_hash": "route",
            "semantic_map_file_sha256": hashlib.sha256(semantic_file.read_bytes()).hexdigest(),
            "path_sha256": hashlib.sha256((candidate / "path.json").read_bytes()).hexdigest(),
            "controls_sha256": hashlib.sha256((candidate / "controls.json").read_bytes()).hexdigest(),
        },
        "candidate_evaluation": {
            "gates": {
                "hard_safety_gate_passed": True,
                "targeted_binding_gate_passed": True,
                "r2_semantic_gate_passed": True,
                "naturalness_gate_passed": passed,
                "strict_acceptance_gate_passed": passed,
            },
            "path_length_m": 10.0,
            "r2_semantic_audit": {"active_window": {"classes": {"lane": {
                "correct_side_ratio": 1.0,
                "target_band_ratio": 1.0,
                "lateral_error_p50_m": 0.1,
            }}}},
            "naturalness_audit": {
                "failure_codes": [] if passed else ["GOAL_PLANE_OVERSHOOT"],
                "target_sample_attribution": {"after_goal_plane_count": 0 if passed else 2},
            },
            "revisit_audit": {
                "revisit_screen_passed": passed,
                "status": "NO_REVISIT_DETECTED" if passed else "DETOUR_JUSTIFICATION_REQUIRED",
            },
        },
        "verification_input_snapshot_match": True,
        "verification_snapshot": {"snapshot_sha256": "fixture-snapshot"},
        "gate_dependency_sha256": aggregate._current_gate_dependency_hashes(),
    }
    verification_file = directory / "verification.json"
    _write_json(verification_file, verification)
    _write_json(directory / "manifest.json", {
        "query_id": query,
        "protocol_id": aggregate.VERIFIER_PROTOCOL_ID,
        "implementation_revision": aggregate.VERIFIER_IMPLEMENTATION_REVISION,
        "source_sha256": hashlib.sha256(
            (aggregate.Path(aggregate.__file__).with_name(
                "semantic_transition_naturalness_verify.py"
            )).read_bytes()
        ).hexdigest(),
        "necessity_audit_source_sha256": hashlib.sha256(
            aggregate.Path(aggregate.audit_path_necessity.__code__.co_filename).read_bytes()
        ).hexdigest(),
        "revisit_audit_source_sha256": hashlib.sha256(
            aggregate.Path(aggregate.audit_revisits.__code__.co_filename).read_bytes()
        ).hexdigest(),
        "gate_dependency_sha256": aggregate._current_gate_dependency_hashes(),
        "verification_input_snapshot_sha256": "fixture-snapshot",
        "verification_input_snapshot_match": True,
        "bound_input_hashes": verification["bindings"],
        "verification_sha256": hashlib.sha256(verification_file.read_bytes()).hexdigest(),
    })
    return directory


def test_aggregate_requires_all_three_and_fails_closed(tmp_path, monkeypatch):
    directories = [
        _fixture(tmp_path, query, passed=query != aggregate.REQUIRED_QUERIES[0])
        for query in aggregate.REQUIRED_QUERIES
    ]
    _seal_inputs(tmp_path, monkeypatch)
    result = aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)
    assert result["offline_pass_count"] == 2
    assert not result["targeted_offline_gate_passed"]
    assert not result["online_eligible"]
    assert result["online_stage"] == "NOT_RUN_OFFLINE_GATE_FAILED"
    assert result["failure_query_ids"] == [aggregate.REQUIRED_QUERIES[0]]


def test_aggregate_rejects_wrong_query_order(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    with pytest.raises(ValueError, match="query order/content mismatch"):
        aggregate.aggregate(
            inputs=tmp_path / "inputs",
            result_directories=[directories[1], directories[0], directories[2]],
        )


def test_aggregate_rejects_tampered_verification(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    with (directories[0] / "verification.json").open("a", encoding="utf-8") as stream:
        stream.write(" ")
    with pytest.raises(ValueError, match="verification hash mismatch"):
        aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)


def test_aggregate_rejects_inconsistent_derived_strict_gate(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    path = directories[0] / "verification.json"
    value = json.loads(path.read_text())
    value["gates"]["hard_safety_gate_passed"] = False
    value["candidate_evaluation"]["gates"]["hard_safety_gate_passed"] = False
    _write_json(path, value)
    manifest = json.loads((directories[0] / "manifest.json").read_text())
    manifest["verification_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    _write_json(directories[0] / "manifest.json", manifest)
    with pytest.raises(ValueError, match="derived strict gate mismatch"):
        aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)


def test_aggregate_exposes_targeted_binding_gate(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    result = aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)
    assert result["targeted_offline_gate_passed"]
    assert all(row["targeted_binding_gate_passed"] for row in result["per_query"])


def test_aggregate_binding_failure_is_a_strict_query_failure(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    verification_path = directories[0] / "verification.json"
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    for gates in (verification["gates"], verification["candidate_evaluation"]["gates"]):
        gates["targeted_binding_gate_passed"] = False
        gates["strict_acceptance_gate_passed"] = False
    _write_json(verification_path, verification)
    manifest_path = directories[0] / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["verification_sha256"] = hashlib.sha256(verification_path.read_bytes()).hexdigest()
    _write_json(manifest_path, manifest)
    result = aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)
    assert result["offline_pass_count"] == 2
    assert result["per_query"][0]["targeted_binding_gate_passed"] is False
    assert result["failure_query_ids"] == [aggregate.REQUIRED_QUERIES[0]]


def test_aggregate_write_rejects_post_check_candidate_drift(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    _seal_inputs(tmp_path, monkeypatch)
    result = aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)
    candidate = tmp_path / f"{aggregate.REQUIRED_QUERIES[0]}-candidate" / "path.json"
    candidate.write_text("[1]\n", encoding="utf-8")
    output = tmp_path / "aggregate-output"
    with pytest.raises(RuntimeError, match="changed"):
        aggregate.write_artifacts(output, result, "reproduce")
    assert not output.exists()


def test_aggregate_rejects_query_pose_drift_even_with_claimed_target_hash(tmp_path, monkeypatch):
    directories = [_fixture(tmp_path, query, True) for query in aggregate.REQUIRED_QUERIES]
    meta = tmp_path / "inputs" / f"{aggregate.REQUIRED_QUERIES[0]}.json"
    value = json.loads(meta.read_text(encoding="utf-8"))
    value["query"]["start"][0] += 0.05
    _write_json(meta, value)
    _seal_inputs(tmp_path, monkeypatch)
    # Keep the verifier binding internally consistent so the exact frozen-query
    # comparison, rather than a generic hash mismatch, is what rejects the run.
    verification_path = directories[0] / "verification.json"
    verification = json.loads(verification_path.read_text(encoding="utf-8"))
    verification["bindings"]["input_meta_sha256"] = hashlib.sha256(meta.read_bytes()).hexdigest()
    _write_json(verification_path, verification)
    manifest_path = directories[0] / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["verification_sha256"] = hashlib.sha256(verification_path.read_bytes()).hexdigest()
    manifest["bound_input_hashes"] = verification["bindings"]
    _write_json(manifest_path, manifest)
    with pytest.raises(ValueError, match="frozen query pose/content mismatch"):
        aggregate.aggregate(inputs=tmp_path / "inputs", result_directories=directories)


def test_resolver_prefers_valid_ament_share_and_rejects_hash_drift(tmp_path, monkeypatch):
    share = tmp_path / "share" / "arena_evaluation"
    config = share / "config"
    config.mkdir(parents=True)
    frozen = aggregate.SOURCE_CONFIG_DIRECTORY / aggregate.TARGETED_QUERY_SET_FILENAME
    installed = config / aggregate.TARGETED_QUERY_SET_FILENAME
    installed.write_bytes(frozen.read_bytes())
    monkeypatch.setattr(aggregate, "_ament_share_directory", lambda: share)
    assert aggregate._resolve_frozen_config(
        aggregate.TARGETED_QUERY_SET_FILENAME,
        aggregate.TARGETED_QUERY_SET_SHA256,
    ) == installed
    installed.write_text("drift\n", encoding="utf-8")
    with pytest.raises(ValueError, match="frozen config hash mismatch"):
        aggregate._resolve_frozen_config(
            aggregate.TARGETED_QUERY_SET_FILENAME,
            aggregate.TARGETED_QUERY_SET_SHA256,
        )


def test_resolver_supports_source_fallback_and_hash_checked_explicit_path(
    tmp_path, monkeypatch
):
    missing_share = tmp_path / "missing-share"
    monkeypatch.setattr(aggregate, "_ament_share_directory", lambda: missing_share)
    source = aggregate.SOURCE_CONFIG_DIRECTORY / aggregate.TARGETED_QUERY_SET_FILENAME
    assert aggregate._resolve_frozen_config(
        aggregate.TARGETED_QUERY_SET_FILENAME,
        aggregate.TARGETED_QUERY_SET_SHA256,
    ) == source

    explicit = tmp_path / "explicit-targeted.yaml"
    explicit.write_bytes(source.read_bytes())
    assert aggregate._resolve_frozen_config(
        aggregate.TARGETED_QUERY_SET_FILENAME,
        aggregate.TARGETED_QUERY_SET_SHA256,
        explicit=explicit,
    ) == explicit
    explicit.write_text("drift\n", encoding="utf-8")
    with pytest.raises(ValueError, match="frozen config hash mismatch"):
        aggregate._resolve_frozen_config(
            aggregate.TARGETED_QUERY_SET_FILENAME,
            aggregate.TARGETED_QUERY_SET_SHA256,
            explicit=explicit,
        )
