"""
RoboDK Verification: compare a robot's TCP position to an arbitrary item.

Prints coordinates in both the absolute world frame and relative to the
`WorldFrame` item in the station (these are equal when WorldFrame is at
identity, but the script reports them independently so any divergence is
visible).

Runs in two modes:
  1. CLI:  python robodk_code/check_world_positions.py --robot "Fanuc R2000iC 125L" --target "base_cone_grab_0" [--tol 0.01]
  2. No-args (RoboDK script runner / IDLE):  uses the CONFIG block below.

Exit code: 0 on match within tolerance (Euclidean mm), 1 on miss, 2 on error.

How world position is read:
  - Robot TCP in world: `robot.Pose()`.
    For the rail-mounted Fanuc here, with a tool active, RoboDK's
    `robot.Pose()` already returns the TCP pose in the ABSOLUTE (world)
    reference frame -- the rail joint contribution is included.
  - Arbitrary item in world: `item.PoseAbs()`.
  - Relative to WorldFrame:  invH(WorldFrame.PoseAbs()) * <world pose>.
"""

import argparse
import sys

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
from robodk.robomath import Pose_2_TxyzRxyz, invH

# ── CONFIG (used when no CLI args are passed) ────────────────────────────────
CONFIG_ROBOT  = "Fanuc R2000iC 125L"
CONFIG_TARGET = "base_cone_grab_0"
CONFIG_TOL    = 0.01
# ─────────────────────────────────────────────────────────────────────────────


def tcp_world_pose(robot):
    """Return the robot's TCP pose in world coords as a Mat. Requires an active tool."""
    tool = robot.getLink(ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError(
            f"Robot '{robot.Name()}' has no active tool. "
            "Call robot.setTool(tool) before running this check."
        )
    return robot.Pose(), tool.Name()


def xyz(pose):
    """Pull (x, y, z) in mm from a 4x4 pose Mat."""
    v = Pose_2_TxyzRxyz(pose)
    return v[0], v[1], v[2]


def parse_args():
    ap = argparse.ArgumentParser(
        description="Compare robot TCP position to an item's position (world + WorldFrame-relative)."
    )
    ap.add_argument("--robot",  default=None, help=f"Robot item name (default: {CONFIG_ROBOT!r}).")
    ap.add_argument("--target", default=None, help=f"Item to compare against (default: {CONFIG_TARGET!r}).")
    ap.add_argument("--tol",    type=float, default=None, help=f"Euclidean tolerance in mm (default: {CONFIG_TOL}).")
    args = ap.parse_args()
    return (
        args.robot  if args.robot  is not None else CONFIG_ROBOT,
        args.target if args.target is not None else CONFIG_TARGET,
        args.tol    if args.tol    is not None else CONFIG_TOL,
    )


def main():
    robot_name, target_name, tol = parse_args()

    RDK = Robolink()

    robot = RDK.Item(robot_name, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print(f"ERROR: robot '{robot_name}' not found.")
        sys.exit(2)

    target = RDK.Item(target_name)
    if not target.Valid():
        print(f"ERROR: item '{target_name}' not found.")
        sys.exit(2)

    world_frame = RDK.Item("WorldFrame")
    if not world_frame.Valid():
        print("ERROR: 'WorldFrame' item not found in station.")
        sys.exit(2)

    tcp_pose_world, tool_name = tcp_world_pose(robot)
    target_pose_world = target.PoseAbs()

    wf_inv = invH(world_frame.PoseAbs())
    tcp_pose_wf    = wf_inv * tcp_pose_world
    target_pose_wf = wf_inv * target_pose_world

    rx_w,  ry_w,  rz_w  = xyz(tcp_pose_world)
    tx_w,  ty_w,  tz_w  = xyz(target_pose_world)
    rx_f,  ry_f,  rz_f  = xyz(tcp_pose_wf)
    tx_f,  ty_f,  tz_f  = xyz(target_pose_wf)

    dx, dy, dz = rx_w - tx_w, ry_w - ty_w, rz_w - tz_w
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5
    verdict = "PASS" if dist <= tol else "FAIL"

    print("=" * 72)
    print(f"Robot:  {robot.Name()}   (tool: {tool_name})")
    print(f"Target: {target.Name()}")
    print("-" * 72)
    print("World frame (absolute):")
    print(f"  Robot TCP:   X={rx_w:10.4f}  Y={ry_w:10.4f}  Z={rz_w:10.4f} mm")
    print(f"  Target:      X={tx_w:10.4f}  Y={ty_w:10.4f}  Z={tz_w:10.4f} mm")
    print("Relative to WorldFrame item:")
    print(f"  Robot TCP:   X={rx_f:10.4f}  Y={ry_f:10.4f}  Z={rz_f:10.4f} mm")
    print(f"  Target:      X={tx_f:10.4f}  Y={ty_f:10.4f}  Z={tz_f:10.4f} mm")
    print("-" * 72)
    print(f"Delta (TCP - target, world): dX={dx:.4f}  dY={dy:.4f}  dZ={dz:.4f} mm")
    print(f"|delta| = {dist:.4f} mm   tolerance = {tol:.4f} mm")
    print(f"Verdict: {verdict}")
    print("=" * 72)

    sys.exit(0 if verdict == "PASS" else 1)


if __name__ == "__main__":
    main()
