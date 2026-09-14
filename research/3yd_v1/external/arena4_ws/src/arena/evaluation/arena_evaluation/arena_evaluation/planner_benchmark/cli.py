from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import CONFIG_ROOT
from .runner import BenchmarkInputError, run_benchmark


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the independent Nav2 global planner microbenchmark")
    parser.add_argument("--protocol", default=str(CONFIG_ROOT / "planner_benchmark_protocol.yaml"))
    parser.add_argument("--queries", default=str(CONFIG_ROOT / "planner_benchmark_queries_hospital.yaml"))
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--planner", choices=["navfn", "smac_hybrid", "all"], default="all")
    parser.add_argument("--config-variant", choices=["product", "normalized", "all"], default="all")
    parser.add_argument("--warmups", type=int, default=None)
    parser.add_argument("--repetitions", type=int, default=None)
    parser.add_argument("--timeout", type=float, default=None)
    parser.add_argument("--query-id", action="append", dest="query_ids", help="Run only this query ID; may be repeated")
    parser.add_argument("--validate-only", action="store_true")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        import yaml
        protocol = yaml.safe_load(Path(args.protocol).read_text())
        planners = ["navfn", "smac_hybrid"] if args.planner == "all" else [args.planner]
        variants = ["product", "normalized"] if args.config_variant == "all" else [args.config_variant]
        output = run_benchmark(
            protocol_path=args.protocol,
            queries_path=args.queries,
            output_dir=args.output_dir,
            planners=planners,
            config_variants=variants,
            warmups=int(protocol.get("warmup_runs", 3) if args.warmups is None else args.warmups),
            repetitions=int(protocol.get("measured_runs", 5) if args.repetitions is None else args.repetitions),
            timeout=float(protocol.get("external_timeout_seconds", 5.0) if args.timeout is None else args.timeout),
            validate_only=args.validate_only,
            query_ids=args.query_ids,
        )
        print(f"benchmark output: {output}")
        return 0
    except (BenchmarkInputError, ValueError, OSError) as exc:
        print(f"planner_benchmark: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
