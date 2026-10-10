"""Fully parameterized WaLTER model, built from YAML configs in WS and FS sizes."""

from pathlib import Path

from .generator import ParametricModel

CONFIG_ROOT = Path(__file__).parent / "configs"
__all__ = ["ParametricModel", "CONFIG_ROOT", "config_dir", "build"]


def config_dir(full_scale: bool = False) -> Path:
    return CONFIG_ROOT / ("FS" if full_scale else "WS")


def build(full_scale: bool = False) -> ParametricModel:
    """Builds the parametric model from the WS configs, or FS if ``full_scale``."""
    configs = config_dir(full_scale)
    return ParametricModel(
        configs / "model_config.yaml",
        configs / "motor_config.yaml",
    )
