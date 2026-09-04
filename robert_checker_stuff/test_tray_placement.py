"""
Test script: import one garment tray STEP into a rack slot to check alignment.

Imports the tray into Rack1GarmentTray1Slot1Base at identity pose and prints
the slot's children so we can see the relationship between the tray geometry
and the existing approach frames.

Run:
    python robert_checker_stuff/test_tray_placement.py
    python robert_checker_stuff/test_tray_placement.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/test_tray_placement.py --slot Rack1GarmentTray3Slot2Base

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import eye, Pose_2_TxyzRxyz

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)

# CHECK_LATER: this path assumes clones/ is populated. If the tray STEP moves
# or the repo structure changes, update this.
DEFAULT_TRAY_STEP = os.path.join(
    REPO_ROOT, "clones", "trays", "Garment Tray Atomic Cell", "V1",
    "garment-tray-atomic-cell-V1.STEP"
)

DEFAULT_SLOT = "Rack1GarmentTray1Slot1Base"


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def print_children(item, depth=0):
    """Print item tree with poses."""
    pose = item.Pose()
    txyz = Pose_2_TxyzRxyz(pose)
    pos_str = f"x:{txyz[0]:.1f} y:{txyz[1]:.1f} z:{txyz[2]:.1f}"
    print(f"{'  ' * depth}{item.Name()} [{item.Type()}] ({pos_str})")
    for child in item.Childs():
        print_children(child, depth + 1)


def main():
    parser = argparse.ArgumentParser(description="Test tray placement in rack slot")
    parser.add_argument("--robodk-ip", default=None)
    parser.add_argument("--slot", default=DEFAULT_SLOT,
                        help=f"Slot frame name (default: {DEFAULT_SLOT})")
    parser.add_argument("--step-file", default=DEFAULT_TRAY_STEP,
                        help="Path to tray STEP file")
    parser.add_argument("--remove", action="store_true",
                        help="Remove previously placed test tray instead of adding")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    # Find the slot frame
    slot = RDK.Item(args.slot, ITEM_TYPE_FRAME)
    assert slot.Valid(), f"Slot frame not found: {args.slot}"

    print(f"\n[INFO] Slot: {args.slot}")
    print(f"[INFO] Slot world pose: {Pose_2_TxyzRxyz(slot.PoseAbs())}")
    print(f"\n--- Before ---")
    print_children(slot)

    if args.remove:
        # Remove any test tray objects from the slot
        removed = 0
        for child in slot.Childs():
            if child.Type() == ITEM_TYPE_OBJECT and "tray" in child.Name().lower():
                print(f"[REMOVE] {child.Name()}")
                child.Delete()
                removed += 1
        print(f"[DONE] Removed {removed} tray object(s)")
        return

    # Import tray STEP — convert WSL path to Windows path for RoboDK
    abs_step = os.path.abspath(args.step_file)
    assert os.path.exists(abs_step), f"STEP file not found: {abs_step}"

    # RoboDK runs on Windows, so /mnt/c/... must become C:\...
    win_step = abs_step
    if abs_step.startswith("/mnt/"):
        drive = abs_step[5]
        win_step = f"{drive.upper()}:{abs_step[6:]}".replace("/", "\\")

    print(f"\n[IMPORT] Loading {win_step} into {args.slot}...")
    tray = RDK.AddFile(win_step, slot)
    assert tray.Valid(), f"Failed to import STEP file"

    # Place at identity relative to slot
    tray.setPose(eye(4))
    tray.setName(f"test_tray_in_{args.slot}")
    print(f"[OK] Tray placed at identity in slot frame")

    print(f"\n--- After ---")
    print_children(slot)

    print(f"\n[INFO] Check RoboDK visually. If the tray is misaligned:")
    print(f"  - Note the offset needed")
    print(f"  - The tray STEP origin may not match the slot origin")
    print(f"  - Run with --remove to clean up, then adjust")


if __name__ == "__main__":
    main()
