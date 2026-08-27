from __future__ import annotations

import hashlib
import time
from pathlib import Path

import yaml

from arena_evaluation.planner_benchmark.provenance import (
    build_code_manifest,
    sha256_file,
    sha256_path,
    write_code_manifest,
)


def test_sha256_file_and_directory_are_deterministic(tmp_path: Path):
    first = tmp_path / "a.py"
    second = tmp_path / "nested" / "b.py"
    second.parent.mkdir()
    first.write_text("alpha\n", encoding="utf-8")
    second.write_text("beta\n", encoding="utf-8")
    assert sha256_file(first) == hashlib.sha256(b"alpha\n").hexdigest()
    digest = sha256_path(tmp_path)
    # A directory digest includes names as well as content and is stable on a
    # second call, which is important for ignored external/ source trees.
    assert digest == sha256_path(tmp_path)
    second.write_text("changed\n", encoding="utf-8")
    assert sha256_path(tmp_path) != digest


def test_code_manifest_records_required_groups_and_hashes(tmp_path: Path):
    source = tmp_path / "benchmark.py"
    hybrid = tmp_path / "hybrid.py"
    validator = tmp_path / "validator.py"
    resource = tmp_path / "resources.py"
    test_source = tmp_path / "test_case.py"
    protocol = tmp_path / "protocol.yaml"
    queries = tmp_path / "queries.yaml"
    for path in (source, hybrid, validator, resource, test_source):
        path.write_text(path.name, encoding="utf-8")
    protocol.write_text("schema_version: 1\n", encoding="utf-8")
    queries.write_text("queries: []\n", encoding="utf-8")
    manifest = build_code_manifest(
        repo_root=tmp_path,
        benchmark_sources=[source],
        hybrid_sources=[hybrid],
        validator_sources=[validator],
        resource_sources=[resource],
        test_sources=[test_source],
        protocol=protocol,
        core_queries=queries,
        command=["planner", "--stage", "run"],
        extra={"dynamic_obstacles": False, "experiment_version": "v2"},
    )
    assert manifest["dynamic_obstacles"] is False
    assert manifest["experiment_version"] == "v2"
    assert manifest["protocol_sha256"] == sha256_file(protocol)
    assert manifest["core_queries_sha256"] == sha256_file(queries)
    assert manifest["benchmark_source_sha256"]
    assert manifest["source_groups"]["resource_monitor"]["files"]
    assert manifest["command"] == ["planner", "--stage", "run"]

    output = tmp_path / "code_manifest.yaml"
    written = write_code_manifest(
        output,
        repo_root=tmp_path,
        benchmark_sources=[source],
        protocol=protocol,
        core_queries=queries,
    )
    assert output.exists()
    assert yaml.safe_load(output.read_text(encoding="utf-8"))["protocol_sha256"] == written["protocol_sha256"]

