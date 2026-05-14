"""
RoboDK script: move pickup_point TCP to base_cone_grab_<N> with axis 7 (rail) LOCKED.

Hard constraints (from caller):
  1. Axis 7 (rail) must not move during the grab approach. Joints 0..5 only.
  2. Rotationally invariant around the target's Z axis. Sample N rotations, pick
     the IK solution closest to current joints.
  3. Sequence: start -> base_cone_grab_<N> -> all-zero joints (j7 also -> 0 at home).
  4. Verify success: print TCP world, target world, delta.

Key facts about this 7-DOF setup (verified against the live station, NOT guessed):
  * The robot is "Fanuc R2000iC 125L" with j7 = the linear rail (RailMechanism).
  * The rail is a pure +X translation in world: setJoints(j7=L) moves robot base
    to world (-100 + L, -25, 1550.5).
  * RoboDK's SolveIK_All for this robot expects the flange pose in WORLD
    coordinates (NOT robot-base frame), and freely chooses j7 to make the
    6-axis arm reachable. We CANNOT pass joints_approx with a 7th element to
    pin j7 -- that argument isn't honored across all RoboDK builds. Instead:
        IK invariance: IK(P_world + dX_x_hat) returns the same j1..j6 with
        j7_returned = j7_natural + dX.
    So to force IK to return j7=L: shift the world flange by
        delta = L - j7_natural
    where j7_natural is the j7 the solver picks unshifted. After the shift the
    solver returns j7~=L (within float precision) and j1..j6 that we use as-is.
  * SolveIK_All returns a Mat sized (7, num_sols). np.array(mat) transposes to
    (num_sols, 7), one solution per row. (The reference script's `for s in
    sols: list(s)[:6]` iteration was WRONG -- it iterated rows-of-Mat which
    are joint-across-sols, not full solutions. That's the bug we don't repeat.)
  * If j7_natural is at the rail limit (0 or 3650), the IK is rail-saturated:
    the returned solution doesn't actually reach the requested pose. We detect
    this by FK-verifying every candidate solution (TCP world vs target world).

Some base_cone_grab_<N> targets may be PHYSICALLY UNREACHABLE at the locked j7
(the 6-axis arm can't reach the cone tray from where the rail is parked). In
that case the script prints a clear error and does NOT silently move j7.
"""

import sys
sys.path.append("C:/RoboDK/Python")

import math
import numpy as np
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET
from robodk.robomath import Mat, Pose_2_TxyzRxyz, invH, transl, rotz

# ── CONFIG ────────────────────────────────────────────────────────────────────
TARGET_NAME = "base_cone_grab_0"
Z_STEPS     = 360         # 1 deg resolution
FK_TOL_MM   = 0.5         # FK position error tolerance for valid solutions
J7_TOL_MM   = 0.05        # tolerance for "did we actually lock j7"
# ──────────────────────────────────────────────────────────────────────────────


def pose_from_z_axis(reference_pose):
    """Build a 4x4 pose at the same position and Z direction as reference_pose,
    with X/Y forming an arbitrary orthonormal basis. Only Z direction matters;
    rotation around Z is freely chosen by the caller."""
    pos = reference_pose.Pos()
    z = [reference_pose[0, 2], reference_pose[1, 2], reference_pose[2, 2]]
    arbitrary = [1.0, 0.0, 0.0] if abs(z[0]) < 0.9 else [0.0, 1.0, 0.0]
    dot = sum(arbitrary[i] * z[i] for i in range(3))
    x = [arbitrary[i] - dot * z[i] for i in range(3)]
    xn = sum(v ** 2 for v in x) ** 0.5
    x = [v / xn for v in x]
    y = [z[1] * x[2] - z[2] * x[1],
         z[2] * x[0] - z[0] * x[2],
         z[0] * x[1] - z[1] * x[0]]
    return Mat([
        [x[0], y[0], z[0], pos[0]],
        [x[1], y[1], z[1], pos[1]],
        [x[2], y[2], z[2], pos[2]],
        [0,    0,    0,    1     ],
    ])


def solutions_as_rows(mat_sols):
    """SolveIK_All returns a Mat of shape (num_joints, num_sols). Convert to a
    (num_sols, num_joints) numpy array so each row is a full solution. Returns
    None when there are no solutions."""
    arr = np.array(mat_sols)
    if arr.size == 0:
        return None
    if arr.ndim == 1:
        arr = arr.reshape(1, -1)
    return arr


def solve_ik_locked_j7(robot, tcp_world, tool_offset, j7_locked, tgt_world,
                       preferred_joints6, z_steps):
    """For each rotation around the target Z axis: solve IK with rail free,
    shift the input pose so the solver returns j7 = j7_locked, FK-verify, then
    pick the 6-joint solution closest to preferred_joints6."""
    canonical = pose_from_z_axis(tcp_world)
    tgt_xyz = Pose_2_TxyzRxyz(tgt_world)

    best        = None
    best_dist   = float("inf")
    best_angle  = None
    n_reachable = 0

    for i in range(z_steps):
        angle = (2.0 * math.pi * i) / z_steps
        flange_world = canonical * rotz(angle) * invH(tool_offset)

        # 1) Unshifted IK to learn j7_natural for this orientation.
        sols0 = solutions_as_rows(robot.SolveIK_All(flange_world))
        if sols0 is None:
            continue
        j7_natural = float(sols0[0, 6])

        # 2) Shift world flange so the solver returns j7 = j7_locked.
        delta = j7_locked - j7_natural
        if abs(delta) < 1e-6:
            sols_use = sols0
        else:
            shifted = transl(delta, 0, 0) * flange_world
            sols_use = solutions_as_rows(robot.SolveIK_All(shifted))
            if sols_use is None:
                continue

        for row in sols_use:
            if len(row) < 7:
                continue
            j6 = [float(v) for v in row[:6]]
            j7_returned = float(row[6])
            if any(math.isnan(v) or math.isinf(v) for v in j6):
                continue
            if abs(j7_returned - j7_locked) > J7_TOL_MM:
                # rail saturated -- IK lied. Skip.
                continue

            sol7 = j6 + [j7_locked]
            fk_flange = robot.SolveFK(sol7)
            tcp_check = fk_flange * tool_offset
            t_xyz = Pose_2_TxyzRxyz(tcp_check)
            err = math.sqrt(
                (t_xyz[0] - tgt_xyz[0]) ** 2 +
                (t_xyz[1] - tgt_xyz[1]) ** 2 +
                (t_xyz[2] - tgt_xyz[2]) ** 2
            )
            if err > FK_TOL_MM:
                continue

            n_reachable += 1
            dist = sum((j6[k] - preferred_joints6[k]) ** 2 for k in range(6))
            if dist < best_dist:
                best_dist  = dist
                best       = sol7
                best_angle = math.degrees(angle)

    return best, best_dist, best_angle, n_reachable


def main():
    RDK = Robolink()
    robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot 'Fanuc R2000iC 125L' not found.")

    tool = RDK.Item("pickup_point", ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError("Tool 'pickup_point' not found.")
    robot.setTool(tool)
    tool_offset = tool.PoseTool()

    target = RDK.Item(TARGET_NAME, ITEM_TYPE_TARGET)
    if not target.Valid():
        raise RuntimeError(f"Target '{TARGET_NAME}' not found.")
    target_pose_world = target.PoseAbs()

    # Snapshot starting joints -- j7 is the rail position we must preserve.
    start_joints = robot.Joints().tolist()
    j7_locked   = start_joints[6]
    preferred6  = start_joints[:6]
    print(f"[INFO] Start joints : {[round(j, 4) for j in start_joints]}")
    print(f"[INFO] Locking j7   = {j7_locked:.4f}")
    print(f"[INFO] Target       : {TARGET_NAME}")
    t_xyz = Pose_2_TxyzRxyz(target_pose_world)
    print(f"[INFO] Target world : X={t_xyz[0]:.3f}  Y={t_xyz[1]:.3f}  Z={t_xyz[2]:.3f}  Rx={t_xyz[3]:.4f}  Ry={t_xyz[4]:.4f}  Rz={t_xyz[5]:.4f}")

    print(f"[INFO] Solving IK with j7 locked, {Z_STEPS} Z-rotation samples...")
    best, dist, best_angle, n_reach = solve_ik_locked_j7(
        robot, target_pose_world, tool_offset, j7_locked,
        target_pose_world, preferred6, Z_STEPS,
    )

    if best is None:
        print("=" * 60)
        print(f"[FAIL] No IK solution exists with j7 locked at {j7_locked:.3f}.")
        print(f"       Reachable orientations scanned: {n_reach} / {Z_STEPS}.")
        print(f"       The 6-axis arm cannot reach '{TARGET_NAME}' from this rail position.")
        print( "       Refusing to move j7 silently (per hard-constraint #1).")
        print("=" * 60)
        return

    print(f"[INFO] Solutions passing FK + j7-lock checks: {n_reach}")
    print(f"[INFO] Picked angle  : {best_angle:.2f} deg  (joint-dist^2 = {dist:.3f})")
    print(f"[INFO] Picked joints : {[round(j, 4) for j in best]}")

    # ── Move 1: start -> base_cone_grab_<N> ──────────────────────────────────
    robot.setSpeed(200)
    robot.setSpeedJoints(200)
    print("[INFO] Moving to target (j7 locked)...")
    robot.MoveJ(best)

    final_joints   = robot.Joints().tolist()
    final_flange   = robot.SolveFK(final_joints)
    final_tcp      = final_flange * tool_offset
    tcp_xyz        = Pose_2_TxyzRxyz(final_tcp)
    delta = (tcp_xyz[0] - t_xyz[0],
             tcp_xyz[1] - t_xyz[1],
             tcp_xyz[2] - t_xyz[2])

    print("=" * 60)
    print(f"[VERIFY] Initial j7 : {j7_locked:.6f}")
    print(f"[VERIFY] Final j7   : {final_joints[6]:.6f}")
    print(f"[VERIFY] j7 delta   : {final_joints[6] - j7_locked:.6f}  (should be ~0)")
    print(f"[VERIFY] TCP world  : X={tcp_xyz[0]:.4f}  Y={tcp_xyz[1]:.4f}  Z={tcp_xyz[2]:.4f}")
    print(f"[VERIFY] Target     : X={t_xyz[0]:.4f}  Y={t_xyz[1]:.4f}  Z={t_xyz[2]:.4f}")
    print(f"[VERIFY] dX,dY,dZ   : {delta[0]:.4f}  {delta[1]:.4f}  {delta[2]:.4f}  mm")
    print("=" * 60)

    # ── Move 2: home (all axes incl. j7 to 0) ────────────────────────────────
    print("[INFO] Returning home (all axes -> 0)...")
    robot.MoveJ([0.0] * 7)
    home_joints = robot.Joints().tolist()
    print(f"[INFO] Joints at home: {[round(j, 6) for j in home_joints]}")
    print(f"[INFO] Final j7 after home: {home_joints[6]:.6f}  (should be 0)")
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
