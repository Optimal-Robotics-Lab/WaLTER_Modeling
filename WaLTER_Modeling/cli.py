"""``generate-walter`` command: exports a WaLTER model to an MJCF XML file.

Each model pipeline is a subcommand, so pipeline-specific options (such as the
parametric model's ``--fs``) only appear where they apply.
"""

import argparse
from pathlib import Path

from . import parametric
from .hardware import HardwareModel

DEFAULT_PARAMETRIC_OUTPUT = Path("WaLTER_Model.xml")
DEFAULT_HARDWARE_OUTPUT = Path("WaLTER_Hardware.xml")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="generate-walter",
        description="WaLTER Model Generator",
    )
    model_parsers = parser.add_subparsers(dest="model", required=True)

    parametric_parser = model_parsers.add_parser(
        "parametric",
        help="Parametric model (WS by default)",
    )
    parametric_parser.add_argument(
        "--fs",
        action="store_true",
        help="Generate FS instead of WS",
    )
    parametric_parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_PARAMETRIC_OUTPUT,
        help="Path to write the generated MJCF XML",
    )

    hardware_parser = model_parsers.add_parser(
        "hardware",
        help="High-fidelity FS hardware model",
    )
    hardware_parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=DEFAULT_HARDWARE_OUTPUT,
        help="Path to write the generated MJCF XML",
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    """Builds the requested model and writes its XML to ``--output``."""
    args = build_parser().parse_args(argv)

    if args.model == "parametric":
        model = parametric.build(full_scale=args.fs)
    else:
        model = HardwareModel()

    args.output.write_text(model.model_xml)
    print(f"Wrote {args.output}")
