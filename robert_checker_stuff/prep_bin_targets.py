"""Create RoboDK targets from bin cone frames for use with robert_end_checker.

The checker expects ITEM_TYPE_TARGET items. The bin cone station has
ITEM_TYPE_FRAME items. This script creates targets at each frame's
PoseAbs() with globally unique names.

Usage:
    python robert_checker_stuff/prep_bin_targets.py --robodk-ip 172.23.208.1

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""
import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_ROBOT
from robodk.robomath import Pose_2_TxyzRxyz
import argparse

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
BIN_PARENT = "Cone_Bin_Frame"
BIN_SUBFRAME = "bottom_corner"
CHILD_SUFFIXES = [
    "suction_offset_2", "suction_offset_1", "suction_position",
    "before_pickup_offset", "cone_pickup_pose", "post_pickup_above",
]
TARGET_FOLDER = "BinSanityCheckTargets"


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


def collect_frames(parent):
    result = {}
    try:
        for child in parent.Childs():
            if child.Type() == ITEM_TYPE_FRAME:
                result[child.Name()] = child
            result.update(collect_frames(child))
    except Exception:
        pass
    return result


def main():
    ap = argparse.ArgumentParser(description="Create targets from bin cone frames")
    ap.add_argument("--robodk-ip", default=None)
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)
    assert robot is not None, f"Robot not found: {ROBOT_NAMES}"
    print(f"Robot: {robot.Name()}")

    # Clean old targets
    old_folder = RDK.Item(TARGET_FOLDER, ITEM_TYPE_FRAME)
    if old_folder.Valid():
        old_folder.Delete()
        print(f"[CLEAN] Deleted old {TARGET_FOLDER}")

    folder = RDK.AddFrame(TARGET_FOLDER)

    # Find bin cones
    bin_frame = RDK.Item(BIN_PARENT, ITEM_TYPE_FRAME)
    assert bin_frame.Valid(), f"{BIN_PARENT} not found"

    bottom = None
    for child in bin_frame.Childs():
        if child.Name() == BIN_SUBFRAME:
            bottom = child
            break
    assert bottom is not None, f"{BIN_SUBFRAME} not found under {BIN_PARENT}"

    created = 0
    for child in bottom.Childs():
        if child.Type() != ITEM_TYPE_FRAME or not child.Name().startswith("cone_"):
            continue
        cone_name = child.Name()
        all_frames = collect_frames(child)
        print(f"\n  {cone_name}: {len(all_frames)} frames found")

        for suffix in CHILD_SUFFIXES:
            frame = all_frames.get(f"{cone_name}_{suffix}") or all_frames.get(suffix)
            if frame is None:
                print(f"    [WARN] {suffix} not found")
                continue

            target_name = f"bin_{cone_name}_{suffix}"
            pose = frame.PoseAbs()
            xyz = Pose_2_TxyzRxyz(pose)[:3]
            tgt = RDK.AddTarget(target_name, folder, robot)
            tgt.setPose(pose)
            created += 1
            print(f"    {target_name} at [{xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f}]")

    print(f"\n[DONE] Created {created} targets in {TARGET_FOLDER}")


if __name__ == "__main__":
    main()
