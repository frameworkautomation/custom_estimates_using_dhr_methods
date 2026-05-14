"""
Test case: reach base_cone_grab_0 with j7 LOCKED, matching BOTH position AND
the target's Z-axis direction (rotation around Z is free for a cone grab, so
that's the angle that matters).

Why custom IK is needed:
  RoboDK's analytic SolveIK / SolveIK_All cannot find an EXACT solution at
  j7=0 satisfying all joint limits — every closed-form solution it returns
  with j7 locked needs j5 outside its ±125° range. The target is at the edge
  of the arm's reach at this rail position. But an APPROXIMATE configuration
  is reachable (the operator demonstrated this manually: ~7 mm and ~6° off
  with sane joint values).

  This script implements a damped least-squares (Levenberg-Marquardt) update
  on a 6-component error vector — 3 components for position (mm) and 3 for
  the Z-axis direction (unitless, weighted by Z_WEIGHT_MM_PER_UNIT so the two
  error types converge on the same scale). Numerical Jacobian via finite
  differences over joints 0..5; j7 held fixed.

Reference frame:
  robot.Pose() is read with the active reference pinned to WorldFrame for
  the duration of the run (and restored on exit). MoveJ-to-target items in
  RoboDK can silently swap the active reference, which corrupts subsequent
  Pose() reads.

Side effects on success: leaves the robot at the converged joints (the
iterative refinement uses setJoints, so the robot ends up there naturally).
"""

import sys
sys.path.append("C:/RoboDK/Python")

import math
import numpy as np
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME

# ── CONFIG ───────────────────────────────────────────────────────────────────
TARGET_NAME            = "base_cone_grab_0"
POS_TOL_MM             = 0.5     # success: position error within this
ANGLE_TOL_DEG          = 2.0     # success: Z-axis angle within this
MAX_ITERS              = 200
FD_STEP_DEG            = 0.05    # finite-difference step on joints (degrees)
LM_DAMPING             = 0.01    # Levenberg-Marquardt damping
STEP_SCALE             = 0.8     # step size relative to LM solution
Z_WEIGHT_MM_PER_UNIT   = 57.3    # 1 rad in z-err ≈ this many mm in pos-err
# ─────────────────────────────────────────────────────────────────────────────


def fmt_joints(j):
    return "[" + ", ".join(f"{x:.3f}" for x in j) + "]"


def read_limits(robot):
    lim = robot.JointLimits()
    lo = [float(lim[0][i, 0]) for i in range(7)]
    hi = [float(lim[1][i, 0]) for i in range(7)]
    return lo, hi


def pos_and_z(pose_mat):
    """Pull position (mm, 3-vec) and Z axis (unit 3-vec) from a 4x4 pose Mat."""
    pos = np.array([pose_mat[0, 3], pose_mat[1, 3], pose_mat[2, 3]], dtype=float)
    zax = np.array([pose_mat[0, 2], pose_mat[1, 2], pose_mat[2, 2]], dtype=float)
    return pos, zax


def actual_vec(robot, joints):
    """Set joints, read robot.Pose(), return [pos (3), z_axis (3)] as a 6-vec.
    Caller must have set active reference frame to WorldFrame."""
    robot.setJoints(joints)
    p, z = pos_and_z(robot.Pose())
    return np.concatenate([p, z])


def numerical_jacobian(robot, joints, h_deg, ndof=6):
    """6 x ndof Jacobian: derivative of (pos, z_axis) wrt each joint."""
    base = actual_vec(robot, joints)
    J = np.zeros((6, ndof))
    for k in range(ndof):
        jp = list(joints)
        jp[k] += h_deg
        plus = actual_vec(robot, jp)
        J[:, k] = (plus - base) / h_deg
    robot.setJoints(joints)  # restore
    return J, base


def angle_between(z1, z2):
    """Angle in degrees between two unit-ish vectors."""
    n1 = np.linalg.norm(z1); n2 = np.linalg.norm(z2)
    if n1 == 0 or n2 == 0:
        return float("nan")
    c = float(np.dot(z1, z2) / (n1 * n2))
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def clip_to_limits(joints, lo, hi):
    return [min(hi[k], max(lo[k], joints[k])) for k in range(len(joints))]


def custom_ik_pos_and_zaxis(robot, target_pose_mat, seed_joints,
                            pos_tol=POS_TOL_MM, angle_tol_deg=ANGLE_TOL_DEG,
                            max_iters=MAX_ITERS, fd_step_deg=FD_STEP_DEG,
                            lm_damping=LM_DAMPING, step_scale=STEP_SCALE,
                            z_weight=Z_WEIGHT_MM_PER_UNIT, verbose=True):
    """Damped LSQ iterative IK targeting position + Z-axis direction.
    Holds joint 7 fixed at seed_joints[6]. Returns (joints, pos_err_mm, angle_deg, converged)."""
    target_pos, target_z = pos_and_z(target_pose_mat)
    target_z /= max(np.linalg.norm(target_z), 1e-12)
    target_vec = np.concatenate([target_pos, target_z])

    lo, hi = read_limits(robot)
    j7_locked = seed_joints[6]
    joints = list(seed_joints)

    W = np.diag([1.0, 1.0, 1.0, z_weight, z_weight, z_weight])

    for it in range(max_iters):
        J, actual = numerical_jacobian(robot, joints, fd_step_deg, ndof=6)
        err = target_vec - actual
        pos_err = float(np.linalg.norm(err[:3]))
        angle_deg = angle_between(actual[3:], target_z)

        if verbose and (it < 3 or it % 10 == 9):
            print(f"  iter {it:3d}: pos_err={pos_err:.4f} mm  angle={angle_deg:.4f} deg  "
                  f"joints={[round(j,3) for j in joints[:6]]}")

        if pos_err < pos_tol and angle_deg < angle_tol_deg:
            if verbose:
                print(f"[INFO] Converged at iter {it}: pos_err={pos_err:.4f} mm  angle={angle_deg:.4f} deg")
            return joints, pos_err, angle_deg, True

        # Weighted LM: minimize ||W*err||² with weighted Jacobian.
        WJ = W @ J
        We = W @ err
        A = WJ.T @ WJ + lm_damping * np.eye(6)
        b = WJ.T @ We
        try:
            dq = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            if verbose:
                print(f"[INFO] Singular Jacobian at iter {it}, stopping.")
            break

        # Step with backtracking: if no improvement, halve the step.
        scale = step_scale
        improved = False
        prev_obj = float(np.dot(We, We))
        for _ in range(5):
            cand = list(joints)
            for k in range(6):
                cand[k] += scale * dq[k]
            cand = clip_to_limits(cand, lo, hi)
            cand[6] = j7_locked
            cand_vec = actual_vec(robot, cand)
            cand_err = target_vec - cand_vec
            cand_obj = float(np.dot(W @ cand_err, W @ cand_err))
            if cand_obj < prev_obj:
                joints = cand
                improved = True
                break
            scale *= 0.5
        if not improved:
            if verbose:
                print(f"[INFO] No improvement at iter {it} (objective stuck).")
            break

    # Final state
    final_vec = actual_vec(robot, joints)
    pos_err = float(np.linalg.norm(target_pos - final_vec[:3]))
    angle_deg = angle_between(final_vec[3:], target_z)
    converged = pos_err < pos_tol and angle_deg < angle_tol_deg
    return joints, pos_err, angle_deg, converged


def main():
    RDK = Robolink()

    robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot 'Fanuc R2000iC 125L' not found.")
    tool = RDK.Item("pickup_point", ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError("Tool 'pickup_point' not found.")
    robot.setTool(tool)

    target = RDK.Item(TARGET_NAME, ITEM_TYPE_TARGET)
    if not target.Valid():
        raise RuntimeError(f"Target '{TARGET_NAME}' not found.")

    world_frame = RDK.Item("WorldFrame")
    if not world_frame.Valid():
        raise RuntimeError("'WorldFrame' item not found in station.")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    try:
        seed = list(robot.Joints().tolist())
        j7_locked = seed[6]
        target_pose = target.PoseAbs()
        tgt_pos, tgt_z = pos_and_z(target_pose)
        tgt_z /= max(np.linalg.norm(tgt_z), 1e-12)

        print(f"[INFO] Seed joints      : {fmt_joints(seed)}")
        print(f"[INFO] j7 locked at     : {j7_locked:.4f}")
        print(f"[INFO] Target world XYZ : X={tgt_pos[0]:.4f}  Y={tgt_pos[1]:.4f}  Z={tgt_pos[2]:.4f}")
        print(f"[INFO] Target Z-axis    : ({tgt_z[0]:.5f}, {tgt_z[1]:.5f}, {tgt_z[2]:.5f})")
        print(f"[INFO] Tolerances       : pos < {POS_TOL_MM} mm, angle < {ANGLE_TOL_DEG} deg")
        print("[INFO] Running custom IK...")

        result, pos_err, angle_deg, converged = custom_ik_pos_and_zaxis(
            robot, target_pose, seed,
        )

        # Read final state
        robot.setJoints(result)
        final_pose = robot.Pose()
        final_pos, final_z = pos_and_z(final_pose)
        dx, dy, dz = final_pos - tgt_pos
        print("=" * 64)
        print(f"[VERIFY] Final joints   : {fmt_joints(result)}")
        print(f"[VERIFY] j7 vs seed     : {result[6]:.6f} (seed {j7_locked:.6f}, delta {result[6]-j7_locked:.6f})")
        print(f"[VERIFY] TCP world XYZ  : X={final_pos[0]:.4f}  Y={final_pos[1]:.4f}  Z={final_pos[2]:.4f}")
        print(f"[VERIFY] Position delta : dX={dx:.4f}  dY={dy:.4f}  dZ={dz:.4f}  |d|={pos_err:.4f} mm")
        print(f"[VERIFY] Z-axis angle   : {angle_deg:.4f} deg")
        print("=" * 64)
        if converged:
            print(f"[PASS] Within {POS_TOL_MM} mm and {ANGLE_TOL_DEG} deg with j7 unchanged.")
        else:
            print(f"[FAIL] pos_err={pos_err:.4f} mm  angle={angle_deg:.4f} deg  "
                  f"(tol {POS_TOL_MM} mm / {ANGLE_TOL_DEG} deg)")
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)


if __name__ == "__main__":
    main()
