import argparse
from pathlib import Path
from .ModelGenerator import GenerateModel

CONFIG_ROOT = Path(__file__).parent / "parametric_configs"
__all__ = ["GenerateModel", "CONFIG_ROOT", "config_dir", "main"]


def config_dir(full_scale: bool = False) -> Path:
    return CONFIG_ROOT / ("FS" if full_scale else "WS")


def main() -> None:
    parser = argparse.ArgumentParser(description="WaLTER Model Generator")
    parser.add_argument("--fs", action="store_true", help="Generate FS instead of WS")
    parser.add_argument("-o", "--output", type=Path, default=Path("WaLTER_Model.xml"), help="Path to write the generated MJCF XML")
    args = parser.parse_args()

    configs = config_dir(full_scale=args.fs)

    generator = GenerateModel(
        configs/"model_config.yaml",
        configs/"motor_config.yaml",
    )
    args.output.write_text(generator.model_xml)
