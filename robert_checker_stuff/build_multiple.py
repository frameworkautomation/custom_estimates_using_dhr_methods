"""
Build movement scripts for multiple machines.

Calls build_for_machine() from build_machine_cone_movements.py for each config,
then creates a top-level run_all_machines program.

Usage:
    python robert_checker_stuff/build_multiple.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/build_multiple.py --robodk-ip 172.23.208.1 \
        --configs config1.json config2.json
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT,
    INSTRUCTION_CALL_PROGRAM,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from build_machine_cone_movements import connect, find_robot, build_for_machine

DEFAULT_CONFIGS = [
    os.path.join(SCRIPT_DIR, "setup_machine_cone_programs_config.json"),
    os.path.join(SCRIPT_DIR, "setup_machine_cone_programs_config_alternate_side.json"),
]


def main():
    ap = argparse.ArgumentParser(
        description="Build movement scripts for multiple machines"
    )
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--configs", nargs="+", default=DEFAULT_CONFIGS,
                    help="Config JSON files (default: machine 3 + machine 4)")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)
    print(f"[INFO] Robot: {robot.Name()}")

    run_all_names = []
    for config_path in args.configs:
        assert os.path.exists(config_path), f"Config not found: {config_path}"
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        name = build_for_machine(RDK, robot, config)
        run_all_names.append(name)

    # Top-level run_all_machines
    if len(run_all_names) > 1:
        run_all_name = "run_all_machines"
        existing = RDK.Item(run_all_name, ITEM_TYPE_PROGRAM)
        if existing.Valid() and existing.InstructionCount() > 0:
            print(f"\n  [CACHE] {run_all_name}")
        else:
            if not existing.Valid():
                run_all = RDK.AddProgram(run_all_name, robot)
            else:
                run_all = existing
            for name in run_all_names:
                run_all.RunInstruction(name, INSTRUCTION_CALL_PROGRAM)
            print(f"\n  [OK]   {run_all_name} ({len(run_all_names)} machines)")

    print("\n[DONE] All machines complete.")


if __name__ == "__main__":
    main()
