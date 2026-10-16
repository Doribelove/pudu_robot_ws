"""Default public API for the stable 3D-V1 global-planning architecture."""

# These four compatibility constants are intentionally retained for the
# untouched r0 reproduction modules, which import them during module loading.
# They are no longer part of the package's default public export surface.
ARCHITECTURE_ID = "3D-V1"
IMPLEMENTATION_REVISION = "r0-production-substrate-l2-dstar-v1"  # legacy internal
PARENT_ARCHITECTURE = "2A-V1-r2"  # legacy internal
PROTOCOL_VERSION = "PLN-02-3D-V1-DYNAMIC-EXT-V1"  # legacy internal

from .stable_contract import (  # noqa: E402
    DEFAULT_STABLE_CONFIG,
    PRODUCTION_BASELINE_ID,
    PROTOCOL_ID,
    RELEASE_CANDIDATE_ID,
    SOURCE_REVISION,
)
from .production_runtime import (  # noqa: E402
    controller_class,
    create_controller,
    production_selection,
    resolve_revision,
)
from .stable_pipeline import (  # noqa: E402
    Layered3DV1StableController,
    StableProductionL3Adapter,
)

# Backward-compatible generic class name now resolves to the only production
# default. Historical classes remain importable from their revisioned modules.
Layered3DV1Controller = Layered3DV1StableController
REVISION_ID = PRODUCTION_BASELINE_ID

__all__ = [
    "ARCHITECTURE_ID",
    "DEFAULT_STABLE_CONFIG",
    "Layered3DV1Controller",
    "Layered3DV1StableController",
    "PRODUCTION_BASELINE_ID",
    "PROTOCOL_ID",
    "RELEASE_CANDIDATE_ID",
    "REVISION_ID",
    "SOURCE_REVISION",
    "StableProductionL3Adapter",
    "controller_class",
    "create_controller",
    "production_selection",
    "resolve_revision",
]
