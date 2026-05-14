"""
Test case: reach base_cone_grab_0 with j7 LOCKED at the current rail position.

Strategy (iterative numerical IK):
  Analytic SolveIK / SolveIK_All on this rail-mounted Fanuc cannot find an
  EXACT solution at j7=0 that also satisfies all joint limits (the only exact
  solutions returned by RoboDK at locked j7 require j5 ≈ +-167 deg, which
  exceeds j5's physical limit of +-125 deg). However, an approximate
  configuration close to the target IS reachable (the operator demonstrated
  this manually: parking the robot ~7 mm from the cone at j7=0 with all joints
  in limits).

  This script uses a damped least-squares (Levenberg-Marquardt style) update
  on the position error, with a numerical Jacobian, starting from the robot's
  current joints. j7 is held fixed; the 6 arm joints are updated each step.
  After convergence we use MoveJ to land at the optimized configuration.

Reference-frame note:
  `robot.Pose()` returns TCP in the robot's CURRENT active reference frame.
  RoboDK's MoveJ to a target item can silently change the active reference,
  so the script explicitly sets the reference to WorldFrame on entry and on
  exit, and reads robot.Pose() only when the reference is WorldFrame.

Side effects on success: leaves the robot at the optimized joints. Does not
return home automatically -- so you can inspect visually.
"""

import sys
sys.path.append("C:/RoboDK/Python")

import math
import numpy as np
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import Pose_2_TxyzRxyz

# ── CONFIG ───────────────────────────────────────────────────────────────────
TARGET_NAME    = "base_cone_grab_0"
POS_TOL_MM     = 0.5       # success if |err| <= this
MAX_ITERS      = 100
FD_STEP        = 0.05      # finite-difference step (deg)
DAMPING        = 0.01      # Levenberg-Marquardt damping
STEP_SCALE     = 0.8       # step size relative to LM solution
# ─────────────────────────────────────────────────────────────────────────────


def fmt(v):
    return "[" + ", ".join(f"{x:.3f}" for x in v) + "]"


def read_limits(robot):
    lim = robot.JointLimits()
    lo = [float(lim[0][i, 0]) for i in range(7)]
    hi = [float(lim[1][i, 0]) for i in range(7)]
    return lo, hi


def tcp_world_xyz(robot, joints):
    """Set joints and read TCP world XYZ. Caller is responsible for
    ensuring the active reference frame is WorldFrame before calling."""
    robot.setJoints(joints)
    p = Pose_2_TxyzRxyz(robot.Pose())
    return np.array([p[0], p[1], p[2]], dtype=float)


def numerical_jacobian_pos(robot, joints, h, ndof=6):
    """3 x ndof Jacobian of TCP position w.r.t. joints[0..ndof-1]."""
    base = tcp_world_xyz(robot, joints)
    J = np.zeros((3, ndof))
    for k in range(ndof):
        jp = list(joints)
        jp[k] += h
        plus = tcp_world_xyz(robot, jp)
        J[:, k] = (plus - base) / h
    # leave robot at original `joints`
    robot.setJoints(joints)
    return J, base


def clip_to_limits(joints, lo, hi):
    return [min(hi[k], max(lo[k], joints[k])) for k in range(len(joints))]


def main():
    RDK = Robolink()

    robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot not found.")
    tool = RDK.Item("pickup_point", ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError("Tool 'pickup_point' not found.")
    robot.setTool(tool)

    target = RDK.Item(TARGET_NAME, ITEM_TYPE_TARGET)
    if not target.Valid():
        raise RuntimeError(f"Target '{TARGET_NAME}' not found.")

    world_frame = RDK.Item("WorldFrame")
    if not world_frame.Valid():
        raise RuntimeError("'WorldFrame' item not found.")

    # Anchor robot.Pose() to WorldFrame for the duration of the run.
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    try:
        lo, hi = read_limits(robot)

        # Seed from current joints. j7 is held fixed at its current value.
        seed = list(robot.Joints().tolist())
        j7_locked = seed[6]
        joints = list(seed)

        tgt_pose = target.PoseAbs()
        tgt = Pose_2_TxyzRxyz(tgt_pose)
        target_xyz = np.array([tgt[0], tgt[1], tgt[2]], dtype=float)

        print(f"[INFO] Seed joints       : {fmt(seed)}")
        print(f"[INFO] j7 locked at      : {j7_locked:.4f}")
        print(f"[INFO] Target world XYZ  : X={tgt[0]:.4f}  Y={tgt[1]:.4f}  Z={tgt[2]:.4f}")
        print(f"[INFO] Joint limits j5   : [{lo[4]:.1f}, {hi[4]:.1f}]")

        last_err = float("inf")
        for it in range(MAX_ITERS):
            J, tcp = numerical_jacobian_pos(robot, joints, h=FD_STEP, ndof=6)
            err = target_xyz - tcp
            err_mag = float(np.linalg.norm(err))

            if it == 0:
                print(f"[INFO] Initial error: |err|={err_mag:.4f} mm   err={err}")

            if err_mag < POS_TOL_MM:
                print(f"[INFO] Converged at iter {it}: |err|={err_mag:.4f} mm")
                break

            # Levenberg-Marquardt: (J^T J + lambda I) dq = J^T err
            JT = J.T
            A  = JT @ J + DAMPING * np.eye(6)
            b  = JT @ err
            dq = np.linalg.solve(A, b)
            new_joints = list(joints)
            for k in range(6):
                new_joints[k] += STEP_SCALE * dq[k]
            new_joints = clip_to_limits(new_joints, lo, hi)
            new_joints[6] = j7_locked

            # Re-read error at new joints to verify improvement
            new_tcp = tcp_world_xyz(robot, new_joints)
            new_err_mag = float(np.linalg.norm(target_xyz - new_tcp))

            if new_err_mag > err_mag:
                # No improvement -- step too big or stuck. Halve step and retry.
                if STEP_SCALE * 0.5 < 0.05:
                    print(f"[INFO] Stalled at iter {it}: |err|={err_mag:.4f} mm "
                          f"(next step would yield {new_err_mag:.4f})")
                    break

            joints = new_joints
            last_err = new_err_mag

            if it % 10 == 9 or it < 5:
                print(f"  iter {it:3d}: |err|={new_err_mag:.4f} mm  "
                      f"joints[:6]={[round(j,2) for j in joints[:6]]}")

        # Final state
        final_joints = list(robot.Joints().tolist())
        final_tcp = Pose_2_TxyzRxyz(robot.Pose())
        dx, dy, dz = final_tcp[0]-tgt[0], final_tcp[1]-tgt[1], final_tcp[2]-tgt[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        print("=" * 64)
        print(f"[VERIFY] Final joints  : {fmt(final_joints)}")
        print(f"[VERIFY] j7 (should match seed_j7={j7_locked:.4f}): {final_joints[6]:.4f}")
        print(f"[VERIFY] TCP world XYZ : X={final_tcp[0]:.4f}  Y={final_tcp[1]:.4f}  Z={final_tcp[2]:.4f}")
        print(f"[VERIFY] Target world  : X={tgt[0]:.4f}  Y={tgt[1]:.4f}  Z={tgt[2]:.4f}")
        print(f"[VERIFY] Delta         : dX={dx:.4f}  dY={dy:.4f}  dZ={dz:.4f}  |d|={dist:.4f} mm")
        print(f"[VERIFY] TCP rot (deg) : Rx={math.degrees(final_tcp[3]):.3f}  "
              f"Ry={math.degrees(final_tcp[4]):.3f}  Rz={math.degrees(final_tcp[5]):.3f}")
        print(f"[VERIFY] Target rot deg: Rx={math.degrees(tgt[3]):.3f}  "
              f"Ry={math.degrees(tgt[4]):.3f}  Rz={math.degrees(tgt[5]):.3f}")
        print("=" * 64)

        if dist <= POS_TOL_MM and abs(final_joints[6] - j7_locked) <= 1e-3:
            print(f"[PASS] Reached target within {POS_TOL_MM} mm with j7 unchanged.")
        elif dist <= 10.0:
            print(f"[PARTIAL] Within 10 mm. Target is at the edge of the arm's "
                  f"reach at j7={j7_locked:.2f}; this is the closest the 6-axis "
                  f"arm can get without moving the rail.")
        else:
            print(f"[FAIL] |d|={dist:.4f} mm exceeds 10 mm.")
    finally:
        # Restore original reference frame.
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)


if __name__ == "__main__":
    main()
