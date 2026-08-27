"""Reproducibility metadata helpers for standalone planner benchmarks.

The benchmark sources live below ``external/`` and that tree may be ignored by
the repository.  A Git commit alone is therefore insufficient provenance.  The
helpers in this module record deterministic SHA256 digests for the actual files
used by a run, together with the repository state and dependency versions.
They are intentionally independent of ROS so they can also be used by smoke
tests and offline report generation.
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import yaml


_IGNORED_PARTS = {"__pycache__", ".pytest_cache", ".git"}


def sha256_file(path: str | Path) -> str:
    """Return the SHA256 digest of *path* without loading it into memory."""

    file_path = Path(path)
    digest = hashlib.sha256()
    with file_path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _iter_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        return []
    return sorted(
        item
        for item in path.rglob("*")
        if item.is_file() and not any(part in _IGNORED_PARTS for part in item.parts)
    )


def sha256_path(path: str | Path) -> str:
    """Hash a file or a directory deterministically.

    For directories, both the relative file name and its content digest are
    included.  This avoids collisions between directories containing the same
    bytes under different names and makes the result stable across machines.
    """

    root = Path(path)
    if root.is_file():
        return sha256_file(root)
    files = _iter_files(root)
    digest = hashlib.sha256()
    for item in files:
        relative = item.relative_to(root).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(sha256_file(item).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def path_hashes(paths: Iterable[str | Path]) -> dict[str, str]:
    """Return path-to-digest entries, skipping missing paths explicitly."""

    result: dict[str, str] = {}
    for raw_path in paths:
        path = Path(raw_path).resolve()
        if path.exists():
            result[str(path)] = sha256_path(path)
    return result


def _git(repo_root: Path, *args: str) -> str:
    try:
        completed = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return ""
    return completed.stdout.strip() if completed.returncode == 0 else ""


def git_metadata(repo_root: str | Path) -> dict[str, Any]:
    """Collect commit and dirty-state information without mutating the repo."""

    root = Path(repo_root).resolve()
    status = _git(root, "status", "--porcelain")
    return {
        "repository_root": str(root),
        "git_commit": _git(root, "rev-parse", "HEAD") or None,
        "git_branch": _git(root, "symbolic-ref", "--short", "-q", "HEAD") or None,
        "git_dirty": bool(status),
        "git_status_lines": len(status.splitlines()) if status else 0,
    }


def dependency_versions(names: Sequence[str] = ("numpy", "scipy", "Pillow", "pandas", "PyYAML")) -> dict[str, Optional[str]]:
    """Return installed package versions, preserving ``None`` when absent."""

    versions: dict[str, Optional[str]] = {
        "python": platform.python_version(),
    }
    for name in names:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def build_code_manifest(
    *,
    repo_root: str | Path,
    benchmark_sources: Iterable[str | Path] = (),
    hybrid_sources: Iterable[str | Path] = (),
    validator_sources: Iterable[str | Path] = (),
    resource_sources: Iterable[str | Path] = (),
    test_sources: Iterable[str | Path] = (),
    protocol: str | Path | None = None,
    core_queries: str | Path | None = None,
    command: Sequence[str] | str | None = None,
    started_at: str | None = None,
    ended_at: str | None = None,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a serializable v2 provenance manifest.

    Each group stores both a group digest and the individual file digests.  A
    missing optional group is represented by ``None`` rather than an invented
    hash, making incomplete provenance visible in reports.
    """

    def group(name: str, values: Iterable[str | Path]) -> dict[str, Any]:
        entries = path_hashes(values)
        if not entries:
            return {"sha256": None, "files": {}}
        # Hash the canonical path/digest mapping, independent of input order.
        encoded = "\n".join(f"{path}\0{digest}" for path, digest in sorted(entries.items())).encode("utf-8")
        return {"sha256": hashlib.sha256(encoded).hexdigest(), "files": entries}

    groups = {
        "benchmark": group("benchmark", benchmark_sources),
        "hybrid_astar": group("hybrid_astar", hybrid_sources),
        "validator": group("validator", validator_sources),
        "resource_monitor": group("resource_monitor", resource_sources),
        "tests": group("tests", test_sources),
    }
    protocol_hash = sha256_path(protocol) if protocol is not None and Path(protocol).exists() else None
    query_hash = sha256_path(core_queries) if core_queries is not None and Path(core_queries).exists() else None
    command_value: list[str] | str | None
    if isinstance(command, tuple):
        command_value = list(command)
    elif isinstance(command, list):
        command_value = list(command)
    else:
        command_value = command
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        **git_metadata(repo_root),
        "python": sys.version,
        "dependencies": dependency_versions(),
        "command": command_value,
        "started_at": started_at,
        "ended_at": ended_at,
        "source_groups": groups,
        # Flat aliases make the required fields easy to consume from shell and
        # pandas without knowing the nested group layout.
        "benchmark_source_sha256": groups["benchmark"]["sha256"],
        "hybrid_astar_source_sha256": groups["hybrid_astar"]["sha256"],
        "validator_source_sha256": groups["validator"]["sha256"],
        "resource_monitor_source_sha256": groups["resource_monitor"]["sha256"],
        "test_source_sha256": groups["tests"]["sha256"],
        "protocol_sha256": protocol_hash,
        "core_queries_sha256": query_hash,
        "protocol": str(Path(protocol).resolve()) if protocol is not None else None,
        "core_queries": str(Path(core_queries).resolve()) if core_queries is not None else None,
    }
    if extra:
        manifest.update(dict(extra))
    return manifest


def write_code_manifest(path: str | Path, **kwargs: Any) -> dict[str, Any]:
    """Build and atomically write a YAML code manifest, returning its value."""

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest = build_code_manifest(**kwargs)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    temporary.replace(output)
    return manifest

