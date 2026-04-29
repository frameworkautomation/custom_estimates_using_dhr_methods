"""Automated URDF export via SW2URDF add-in.

Primary path: invoke SW2URDF's COM interface if available.
Fallback: generate SW2URDF config CSV and prompt user to run the GUI export once.
After first GUI export, the assembly has saved SW2URDF data and future exports
can run programmatically via File > Export > URDF menu-equivalent commands.

Usage:
    python sw_export_urdf.py --assembly "C:/path/robot.SLDASM" \
                             --constraints pipeline/constraints/robot.json \
                             --output-dir pipeline/urdf/fanuc_robot
"""
import argparse
import csv
import json
import os
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from lib.sw_connection import SolidWorksConnection
from lib.status_writer import StatusWriter


def try_com_urdf_export(sw_app, model, output_dir):
    """Attempt to invoke the SW2URDF add-in via COM.

    Returns True on success, False if not available.
    """
    try:
        # SW2URDF add-in is registered under "SW2URDF.Exporter"
        addin = sw_app.GetAddInObject("SW2URDF.Exporter")
        if addin is None:
            return False
        addin.ExportURDFAssembly(output_dir)
        return True
    except Exception as e:
        print(f"COM URDF export unavailable: {e}", file=sys.stderr)
        return False


def write_sw2urdf_config_csv(constraints, output_path):
    """Write a CSV that can be imported into SW2URDF GUI to set up the link tree."""
    with open(output_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["link_name", "parent_link", "joint_name", "joint_type", "axis_x", "axis_y", "axis_z", "lower", "upper"])

        # Base link first (no parent, no joint)
        w.writerow(["base_link", "", "", "", "", "", "", "", ""])

        for joint in constraints.get("joints", []):
            axis = joint.get("axis") or [0, 0, 0]
            limits = joint.get("limits") or {}
            w.writerow([
                joint.get("child", ""),
                joint.get("parent", "base_link"),
                joint.get("id", ""),
                joint.get("type", ""),
                axis[0] if len(axis) > 0 else 0,
                axis[1] if len(axis) > 1 else 0,
                axis[2] if len(axis) > 2 else 0,
                limits.get("lower", ""),
                limits.get("upper", ""),
            ])


def main():
    parser = argparse.ArgumentParser(description="Export URDF via SW2URDF")
    parser.add_argument("--assembly", required=True)
    parser.add_argument("--constraints", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--status-dir", default="pipeline/status")
    args = parser.parse_args()

    status = StatusWriter(Path(args.status_dir), "sw_export_urdf")
    status.start(message=f"Preparing URDF for {os.path.basename(args.assembly)}")

    try:
        with open(args.constraints) as f:
            constraints = json.load(f)

        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Always write the CSV config — useful as a record and GUI fallback
        csv_path = output_dir / "sw2urdf_config.csv"
        write_sw2urdf_config_csv(constraints, csv_path)
        status.progress(current=1, total=2, message=f"Wrote config: {csv_path}")

        with SolidWorksConnection() as sw:
            model = sw.open_assembly(args.assembly)

            # Try COM export first
            com_success = try_com_urdf_export(sw.sw_app, model, str(output_dir))

            if com_success:
                status.complete(
                    message=f"URDF exported to {output_dir}",
                    result={"method": "com_api", "output_dir": str(output_dir)},
                )
                print(f"URDF exported via COM to {output_dir}")
            else:
                status.complete(
                    message=(f"SW2URDF COM unavailable. Config CSV at {csv_path}. "
                             f"Run File > Export > URDF in SW GUI once."),
                    result={
                        "method": "csv_fallback",
                        "csv_path": str(csv_path),
                        "manual_step_required": True,
                    },
                )
                print(f"Fallback: CSV written to {csv_path}")
                print("Open the assembly in SolidWorks and run File > Export > URDF")

    except Exception as e:
        status.error(message=str(e), exception=e)
        traceback.print_exc()
        raise


if __name__ == "__main__":
    main()
