"""Model generation for the WaLTER robot.

Two independent pipelines share this package: ``parametric`` (fully parameterized,
WS and FS) and ``hardware`` (high-fidelity FS model of the real robot).
"""

from .cli import main
from .hardware import HardwareModel
from .parametric import CONFIG_ROOT, ParametricModel, config_dir

# Kept so code written against the original ModelGenerator API keeps working.
GenerateModel = ParametricModel

__all__ = [
    "ParametricModel",
    "HardwareModel",
    "GenerateModel",
    "CONFIG_ROOT",
    "config_dir",
    "main",
]
