"""
RoboDK Verification: compare world XYZ of a robot's TCP to an arbitrary item.

Usage:
  python robodk_code/check_world_positions.py --robot "Fanuc R2000iC 125L" \
      --target "base_cone_grab_0" [--tol 0.01]

Exit code 0 on match within tolerance (Euclidean mm), 1 on miss.

How world position is read:
  - Robot TCP in world: `robot.Pose()`.
    For a rail-mounted robot with `robot.setTool(tool)` active, RoboDK's
    `robot.Pose()` already returns the TCP pose in the ABSOLUTE (world)
    reference frame -- the rail joint contribution is included. This was
    verified live: moving the rail joint by +500 mm shifts robot.Pose() X
    by exactly +500 mm, and the value matches the target's PoseAbs() after
    a successful MoveJ to that target (see moving_to_cones.py).
  - Arbitrary item in world: `item.PoseAbs()`.
"""

import argparse
import sys

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
from robodk.robomath import Pose_2_TxyzRxyz


def tcp_world_xyz(robot):
    """Return TCP world XYZ as (x, y, z) in mm. Requires a tool to be active."""
    tool = robot.getLink(ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError(
            f"Robot '{robot.Name()}' has no active tool. "
            "Call robot.setTool(tool) before running this check."
        )
    xyz = Pose_2_TxyzRxyz(robot.Pose())
    return xyz[0], xyz[1], xyz[2], tool.Name()


def item_world_xyz(item):
    """Return any item's world XYZ as (x, y, z) in mm via PoseAbs()."""
    xyz = Pose_2_TxyzRxyz(item.PoseAbs())
    return xyz[0], xyz[1], xyz[2]


def main():
    ap = argparse.ArgumentParser(description="Compare robot TCP world position to an item's world position.")
    ap.add_argument("--robot", required=True, help="Robot item name (e.g. 'Fanuc R2000iC 125L').")
    ap.add_argument("--target", required=True, help="Item to compare against (target, frame, object, etc.).")
    ap.add_argument("--tol", type=float, default=0.01, help="Euclidean tolerance in mm (default 0.01).")
    args = ap.parse_args()

    RDK = Robolink()

    robot = RDK.Item(args.robot, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print(f"ERROR: robot '{args.robot}' not found.")
        sys.exit(2)

    target = RDK.Item(args.target)
    if not target.Valid():
        print(f"ERROR: item '{args.target}' not found.")
        sys.exit(2)

    rx, ry, rz, tool_name = tcp_world_xyz(robot)
    tx, ty, tz = item_world_xyz(target)
    dx, dy, dz = rx - tx, ry - ty, rz - tz
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
    verdict = "PASS" if dist <= args.tol else "FAIL"

    print("=" * 64)
    print(f"Robot:  {robot.Name()}   (tool: {tool_name})")
    print(f"Target: {target.Name()}")
    print("-" * 64)
    print(f"Robot TCP (world):   X={rx:.4f}  Y={ry:.4f}  Z={rz:.4f} mm")
    print(f"Target    (world):   X={tx:.4f}  Y={ty:.4f}  Z={tz:.4f} mm")
    print(f"Delta (TCP - target): dX={dx:.4f}  dY={dy:.4f}  dZ={dz:.4f} mm")
    print(f"|delta| = {dist:.4f} mm   tolerance = {args.tol:.4f} mm")
    print(f"Verdict: {verdict}")
    print("=" * 64)

    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
