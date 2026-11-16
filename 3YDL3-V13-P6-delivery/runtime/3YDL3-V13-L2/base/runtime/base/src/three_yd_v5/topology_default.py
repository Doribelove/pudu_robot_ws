"""Select the default L1 topology implementation for the r14 planning flow."""
import os

from .topology import V5Topology
from .topology_astar import AStarTopology

SEARCH_MODE = os.environ.get("THREEYD_L1_SEARCH", "astar")
if SEARCH_MODE == "astar":
    DefaultTopology = AStarTopology
elif SEARCH_MODE == "legacy":
    DefaultTopology = V5Topology
else:
    raise ValueError("UNKNOWN_THREEYD_L1_SEARCH: " + SEARCH_MODE)
