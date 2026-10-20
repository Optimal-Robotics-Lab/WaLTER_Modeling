"""High-fidelity full-scale WaLTER model built from hardware data and CAD meshes."""

from .generator import DEFAULT_CONFIG, MESH_DIR, HardwareModel

__all__ = ["HardwareModel", "DEFAULT_CONFIG", "MESH_DIR"]
