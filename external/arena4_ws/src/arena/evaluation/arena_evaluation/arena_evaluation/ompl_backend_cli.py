"""Process boundary for the compiled OMPL planner adapter."""

from __future__ import annotations

import argparse
import sys


def main(argv=None) -> int:
    from . import _ompl_planner_backend as backend

    values = list(sys.argv[1:] if argv is None else argv)
    if values == ["--version"]:
        print(f"OMPL {backend.version()}")
        return 0
    return int(backend.run(values))


if __name__ == "__main__":
    raise SystemExit(main())
