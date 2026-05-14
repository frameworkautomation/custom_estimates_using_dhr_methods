"""
RoboDK Script: Move Fanuc R2000iC 125L TCP (pickup_point) to TARGET_NAME.
Free rotation around target Z axis. Confirmed working — delta ~0.
"""

import sys
sys.path.append("C:/RoboDK/Python")  # so external Python can find the robodk package

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, ITEM_TYPE_TOOL
from robodk.robomath import *
import tkinter as tk
from tkinter import messagebox

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
TARGET_NAME = "cone_grab_0"   # change to cone_grab_1, cone_grab_2, etc.
Z_STEPS     = 36              # Z rotation samples (36 = every 10 deg)
# ──────────────────────────────────────────────────────────────────────────────


def blocking_popup(title, message):
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(title, message, parent=root)
    root.destroy()


def pose_from_z_axis(reference_pose):
    """
    Build a 4x4 pose with the same position and Z axis as reference_pose
    but with X/Y axes as an arbitrary orthonormal frame (Gram-Schmidt).
    Only position and Z axis direction are constrained — rotation around Z is free.
    """
    pos = reference_pose.Pos()
    z = [reference_pose[0,2], reference_pose[1,2], reference_pose[2,2]]
    arbitrary = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
    dot = sum(arbitrary[i]*z[i] for i in range(3))
    x = [arbitrary[i] - dot*z[i] for i in range(3)]
    x_norm = sum(v**2 for v in x)**0.5
    x = [v/x_norm for v in x]
    y = [z[1]*x[2] - z[2]*x[1],
         z[2]*x[0] - z[0]*x[2],
         z[0]*x[1] - z[1]*x[0]]
    return Mat([
        [x[0], y[0], z[0], pos[0]],
        [x[1], y[1], z[1], pos[1]],
        [x[2], y[2], z[2], pos[2]],
        [0,    0,    0,    1     ],
    ])


def solve_ik_free_z(robot, tcp_pose_world, tool_offset, rail_offset_x, preferred_joints=None, z_steps=36):
    """
    Solve IK with position + Z axis constrained, free rotation around Z.
    Rotates the TCP target around its own Z axis, back-calculates the flange
    for each rotation, applies rail correction, collects all SolveIK_All
    solutions, returns joint solution closest to preferred_joints.
    """
    if preferred_joints is None:
        preferred_joints = [0.0] * 7

    canonical_tcp = pose_from_z_axis(tcp_pose_world)

    best        = None
    best_dist   = float("inf")
    best_angle  = 0.0
    found_any   = False

    for i in range(z_steps):
        angle_rad = (2.0 * pi * i) / z_steps

        # 1. Rotate TCP target around its own Z axis
        rotated_tcp = canonical_tcp * rotz(angle_rad)

        # 2. Back-calculate flange pose from rotated TCP
        flange = rotated_tcp * invH(tool_offset)

        # 3. Apply rail offset correction
        flange_for_ik = transl(-rail_offset_x, 0, 0) * flange

        # 4. Solve IK
        solutions = robot.SolveIK_All(flange_for_ik)
        if solutions is None or len(solutions) == 0:
            continue
        found_any = True
        for sol in solutions:
            joints = list(sol)
            if len(joints) < 6:
                continue
            padded = joints + [0.0] * (7 - len(joints))
            dist = sum((padded[j] - preferred_joints[j]) ** 2 for j in range(7))
            if dist < best_dist:
                best_dist  = dist
                best       = padded
                best_angle = angle_rad * 180.0 / pi

    if not found_any:
        raise RuntimeError("IK solver returned no solutions across all Z rotations.")
    if best is None:
        raise RuntimeError("Could not select a valid IK solution.")

    print(f"[INFO] Best Z rotation: {best_angle:.1f} deg  joint dist={best_dist:.2f}")
    return best


def main():
    RDK = Robolink()

    # ── 1. Robot ──────────────────────────────────────────────────────────────
    robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        for item in RDK.ItemList(ITEM_TYPE_ROBOT):
            if "R2000" in item.Name():
                robot = item
                break
    if not robot.Valid():
        raise RuntimeError("Robot not found.")
    print(f"[INFO] Using robot: {robot.Name()}")

    # ── 2. Save current tool so we can restore it after ───────────────────────
    original_tool = robot.getLink(ITEM_TYPE_TOOL)
    print(f"[INFO] Original tool: {original_tool.Name() if original_tool.Valid() else 'none'}")

    # ── 3. Load pickup_point tool ─────────────────────────────────────────────
    tool = RDK.Item("pickup_point", ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError("Tool 'pickup_point' not found. Run the GH script first.")
    tool_offset = tool.PoseTool()
    tool_xyz = Pose_2_TxyzRxyz(tool_offset)
    print(f"[INFO] pickup_point TCP offset:  "
          f"X={tool_xyz[0]:.3f}  Y={tool_xyz[1]:.3f}  Z={tool_xyz[2]:.3f} mm")
    robot.setTool(tool)
    print("[INFO] Tool set to pickup_point.")

    # ── 4. Rail offset ────────────────────────────────────────────────────────
    rail_base_offset_x = 0.0
    item = robot
    while item.Valid():
        if "RailMechanismBase" in item.Name():
            rail_base_offset_x = Pose_2_TxyzRxyz(item.PoseAbs())[0]
            print(f"[INFO] RailMechanismBase abs X offset: {rail_base_offset_x:.3f} mm")
            break
        item = item.Parent()

    # ── 5. Target ─────────────────────────────────────────────────────────────
    target = RDK.Item(TARGET_NAME, ITEM_TYPE_TARGET)
    if not target.Valid():
        raise RuntimeError(f"Target '{TARGET_NAME}' not found.")
    target_pose_world = target.PoseAbs()
    tgt = Pose_2_TxyzRxyz(target_pose_world)
    tgt_x, tgt_y, tgt_z   = tgt[0], tgt[1], tgt[2]
    tgt_rx, tgt_ry, tgt_rz = tgt[3], tgt[4], tgt[5]
    print(f"[INFO] Target (world):  X={tgt_x:.3f}  Y={tgt_y:.3f}  Z={tgt_z:.3f} mm")

    # ── 6. IK — free Z rotation ───────────────────────────────────────────────
    # solve_ik_free_z rotates around the TARGET Z axis, back-calculates the
    # flange pose for each rotation, applies the rail correction, and picks
    # the joint solution closest to current joints across all rotations.
    current_joints_7 = (robot.Joints().tolist() + [0.0] * 7)[:7]
    print(f"[INFO] Solving IK with free Z rotation ({Z_STEPS} steps)...")
    joints_to_target = solve_ik_free_z(
        robot, target_pose_world, tool_offset, rail_base_offset_x, current_joints_7, Z_STEPS
    )
    print(f"[INFO] IK solution: {[round(j, 4) for j in joints_to_target]}")
    print(f"[INFO] d7={joints_to_target[6]:.3f} mm")

    # ── 9. Move ───────────────────────────────────────────────────────────────
    robot.setSpeed(200)
    robot.setSpeedJoints(200)
    print("[INFO] Moving...")
    robot.MoveJ(joints_to_target)
    print("[INFO] Move complete.")

    # ── 10. Record ────────────────────────────────────────────────────────────
    actual_joints = robot.Joints().tolist()
    actual_pose   = robot.Pose()
    actual_xyz    = Pose_2_TxyzRxyz(actual_pose)
    tcp_x, tcp_y, tcp_z   = actual_xyz[0], actual_xyz[1], actual_xyz[2]
    tcp_rx, tcp_ry, tcp_rz = actual_xyz[3], actual_xyz[4], actual_xyz[5]

    print("=" * 60)
    print(f"[RECORD] TCP (pickup_point, robot.Pose()):")
    print(f"         X={tcp_x:.4f} mm  Y={tcp_y:.4f} mm  Z={tcp_z:.4f} mm")
    print(f"         Rx={tcp_rx:.4f} deg  Ry={tcp_ry:.4f} deg  Rz={tcp_rz:.4f} deg")
    print(f"[RECORD] Target (world):")
    print(f"         X={tgt_x:.4f} mm  Y={tgt_y:.4f} mm  Z={tgt_z:.4f} mm")
    print(f"         Rx={tgt_rx:.4f} deg  Ry={tgt_ry:.4f} deg  Rz={tgt_rz:.4f} deg")
    print(f"[RECORD] Delta TCP - target:")
    print(f"         dX={tcp_x-tgt_x:.4f} mm  dY={tcp_y-tgt_y:.4f} mm  dZ={tcp_z-tgt_z:.4f} mm")
    print("=" * 60)

    # ── 11. Blocking popup ────────────────────────────────────────────────────
    blocking_popup(
        title=f"Robot at {TARGET_NAME}",
        message=(
            f"Joints: {[round(j,2) for j in actual_joints]}\n\n"
            f"TCP (pickup_point, world):\n"
            f"  X={tcp_x:.3f}  Y={tcp_y:.3f}  Z={tcp_z:.3f} mm\n\n"
            f"Target (world):\n"
            f"  X={tgt_x:.3f}  Y={tgt_y:.3f}  Z={tgt_z:.3f} mm\n\n"
            f"Delta:\n"
            f"  dX={tcp_x-tgt_x:.3f}  dY={tcp_y-tgt_y:.3f}  dZ={tcp_z-tgt_z:.3f} mm\n\n"
            "Click OK to return to zero."
        )
    )

    # ── 12. Return to zero and restore original tool ──────────────────────────
    robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print("[INFO] All axes at zero.")
    if original_tool.Valid():
        robot.setTool(original_tool)
        print(f"[INFO] Tool restored to: {original_tool.Name()}")
    print("[INFO] Done.")


if __name__ == "__main__":
    main()

