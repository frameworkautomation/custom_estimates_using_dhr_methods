"""
Move pickup_point TCP to base_cone_grab_<N> with j7 LOCKED and SETTABLE
accuracy. Uses the custom IK function from test_reach_base_cone (damped
least-squares matching position + Z-axis direction).

This is the "production" mover: same intent as move_to_base_cone_grab.py
but uses the working iterative IK instead of the broken analytic approach.

Settable accuracy:
  - CONFIG block at top -- defaults are reasonable for cone grabbing.
  - CLI overrides:
      --target NAME       which base_cone_grab to reach (default cone 0)
      --pos-tol  MM       position tolerance (default 0.5 mm)
      --angle-tol DEG     Z-axis angle tolerance (default 2.0 deg)
      --max-iters N       max IK iterations before giving up (default 200)
      --no-home           skip the return-to-zero MoveJ at the end
      --dry-run           solve and report, but DO NOT MoveJ
      --no-pause          suppress the modal popups (headless)

Pauses by default:
  * After the IK solver returns: popup shows the converged joints and
    both error values; the MoveJ doesn't happen until you click OK.
  * After the MoveJ to the grab pose completes: popup pauses before the
    return-to-zero so you can inspect.
  * On failure (no solution within tolerance): popup shows the closest
    achievable pos/angle error; no MoveJ is performed.

Strategy:
  1. Seed from the robot's CURRENT joints. j7 is held fixed at its current
     value. Run custom IK; if convergence fails, report the achievable
     pos/angle error and exit non-zero (do not move).
  2. On success, MoveJ to the converged joints.
  3. Optionally return home.
"""

import sys
sys.path.append("C:/RoboDK/Python")

import argparse
import os
import tkinter as tk
from tkinter import messagebox

import numpy as np

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME

# Reuse the IK from the test script.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from test_reach_base_cone import custom_ik_pos_and_zaxis, pos_and_z, fmt_joints


def blocking_popup(title, message):
    """Show a modal popup that blocks until the user clicks OK. Used so the
    user can inspect the solution + the robot's pose before the next move."""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(title, message, parent=root)
    root.destroy()

# ── CONFIG (used when no CLI override) ───────────────────────────────────────
CONFIG_TARGET     = "base_cone_grab_0"
CONFIG_POS_TOL    = 0.5    # mm
CONFIG_ANGLE_TOL  = 2.0    # deg
CONFIG_MAX_ITERS  = 200    # max IK iterations (LM steps) before giving up
CONFIG_GO_HOME    = True   # after the grab, return to [0]*7
CONFIG_DRY_RUN    = False
CONFIG_PAUSE      = True   # show modal popups at the solution and after the MoveJ
SPEED_MM_S        = 200
SPEED_J_DEG_S     = 200
# ─────────────────────────────────────────────────────────────────────────────


def parse_args():
    ap = argparse.ArgumentParser(description="Move pickup_point to a base_cone_grab target with j7 locked + settable accuracy.")
    ap.add_argument("--target",    default=None,             help=f"base_cone_grab name (default {CONFIG_TARGET!r})")
    ap.add_argument("--pos-tol",   default=None, type=float, help=f"position tolerance mm (default {CONFIG_POS_TOL})")
    ap.add_argument("--angle-tol", default=None, type=float, help=f"Z-axis angle tol deg (default {CONFIG_ANGLE_TOL})")
    ap.add_argument("--max-iters", default=None, type=int,   help=f"max IK iterations (default {CONFIG_MAX_ITERS})")
    ap.add_argument("--no-home",   action="store_true",      help="skip the return-to-zero MoveJ at the end")
    ap.add_argument("--dry-run",   action="store_true",      help="solve and report only, do not MoveJ")
    ap.add_argument("--no-pause",  action="store_true",      help="suppress the modal popups (headless)")
    a = ap.parse_args()
    return {
        "target":    a.target    if a.target    is not None else CONFIG_TARGET,
        "pos_tol":   a.pos_tol   if a.pos_tol   is not None else CONFIG_POS_TOL,
        "angle_tol": a.angle_tol if a.angle_tol is not None else CONFIG_ANGLE_TOL,
        "max_iters": a.max_iters if a.max_iters is not None else CONFIG_MAX_ITERS,
        "go_home":   (not a.no_home) and CONFIG_GO_HOME,
        "dry_run":   a.dry_run   or CONFIG_DRY_RUN,
        "pause":     (not a.no_pause) and CONFIG_PAUSE,
    }


def main():
    cfg = parse_args()

    RDK = Robolink()
    robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot 'Fanuc R2000iC 125L' not found.")
    tool = RDK.Item("pickup_point", ITEM_TYPE_TOOL)
    if not tool.Valid():
        raise RuntimeError("Tool 'pickup_point' not found.")
    robot.setTool(tool)

    target = RDK.Item(cfg["target"], ITEM_TYPE_TARGET)
    if not target.Valid():
        raise RuntimeError(f"Target '{cfg['target']}' not found.")

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

        print(f"[INFO] Target              : {cfg['target']}")
        print(f"[INFO] Seed joints (current): {fmt_joints(seed)}")
        print(f"[INFO] j7 locked at        : {j7_locked:.4f}")
        print(f"[INFO] Target world XYZ    : X={tgt_pos[0]:.4f}  Y={tgt_pos[1]:.4f}  Z={tgt_pos[2]:.4f}")
        print(f"[INFO] Target Z-axis       : ({tgt_z[0]:.5f}, {tgt_z[1]:.5f}, {tgt_z[2]:.5f})")
        print(f"[INFO] Tolerances          : pos < {cfg['pos_tol']} mm, angle < {cfg['angle_tol']} deg")
        print(f"[INFO] Max IK iterations   : {cfg['max_iters']}")
        print(f"[INFO] Dry run             : {cfg['dry_run']}")
        print(f"[INFO] Go home after grab  : {cfg['go_home']}")
        print("[INFO] Running custom IK...")

        result, pos_err, angle_deg, converged = custom_ik_pos_and_zaxis(
            robot, target_pose, seed,
            pos_tol=cfg["pos_tol"],
            angle_tol_deg=cfg["angle_tol"],
            max_iters=cfg["max_iters"],
            verbose=True,
        )

        print("=" * 64)
        print(f"[RESULT] Converged         : {converged}")
        print(f"[RESULT] Final joints      : {fmt_joints(result)}")
        print(f"[RESULT] Position error    : {pos_err:.4f} mm   (tol {cfg['pos_tol']} mm)")
        print(f"[RESULT] Z-axis angle err  : {angle_deg:.4f} deg  (tol {cfg['angle_tol']} deg)")
        print(f"[RESULT] j7 vs seed        : delta {result[6] - j7_locked:.6f} mm")
        print("=" * 64)

        if not converged:
            print(f"[FAIL] Could not reach '{cfg['target']}' within tolerances "
                  f"({cfg['pos_tol']} mm / {cfg['angle_tol']} deg) at locked j7={j7_locked:.2f}. "
                  f"Best achievable from this seed: pos_err={pos_err:.4f} mm  "
                  f"angle_err={angle_deg:.4f} deg.")
            print("[FAIL] No move performed.")
            if cfg["pause"]:
                blocking_popup(
                    title=f"No solution for {cfg['target']}",
                    message=(
                        f"Could not converge within tolerance.\n\n"
                        f"Tolerances:  pos < {cfg['pos_tol']} mm,  angle < {cfg['angle_tol']} deg\n"
                        f"Best from this seed:\n"
                        f"  position error = {pos_err:.4f} mm\n"
                        f"  Z-axis angle   = {angle_deg:.4f} deg\n\n"
                        f"j7 locked at {j7_locked:.3f} mm\n"
                        f"Iterations cap: {cfg['max_iters']}\n\n"
                        "No move performed. Click OK to dismiss."
                    ),
                )
            sys.exit(1)

        # Pause #1: show solution + errors. User clicks OK before MoveJ.
        if cfg["pause"]:
            blocking_popup(
                title=f"IK solution for {cfg['target']}",
                message=(
                    f"Converged.\n\n"
                    f"Position error: {pos_err:.4f} mm   (tol {cfg['pos_tol']} mm)\n"
                    f"Z-axis angle:   {angle_deg:.4f} deg  (tol {cfg['angle_tol']} deg)\n"
                    f"j7 vs seed:     delta {result[6] - j7_locked:.6f} mm\n\n"
                    f"Final joints:\n  {fmt_joints(result)}\n\n"
                    "Click OK to MoveJ to this configuration."
                ),
            )

        if cfg["dry_run"]:
            print("[INFO] Dry run -- not calling MoveJ.")
            # custom_ik_pos_and_zaxis already left the robot at `result` via setJoints;
            # for a strict dry run, restore the seed.
            robot.setJoints(seed)
            print("[INFO] Restored robot to seed joints.")
            return

        # MoveJ to the converged joints. The robot is already at `result`
        # from the iterative refinement (setJoints), but a MoveJ here gives
        # a real motion (controlled, with speed) for production use and also
        # validates that the joints command is accepted by the controller.
        robot.setSpeed(SPEED_MM_S)
        robot.setSpeedJoints(SPEED_J_DEG_S)
        print("[INFO] MoveJ to converged joints...")
        robot.MoveJ(result)
        print("[INFO] Move complete.")

        # Pause #2: after MoveJ completes, before returning home.
        if cfg["pause"]:
            home_note = "Click OK to return home (all joints to 0)." if cfg["go_home"] else "Click OK to finish."
            blocking_popup(
                title=f"At {cfg['target']}",
                message=(
                    f"MoveJ to grab pose complete.\n\n"
                    f"Position error: {pos_err:.4f} mm\n"
                    f"Z-axis angle:   {angle_deg:.4f} deg\n"
                    f"j7 = {result[6]:.3f} mm (locked)\n\n"
                    f"{home_note}"
                ),
            )

        if cfg["go_home"]:
            print("[INFO] Returning home (all joints incl. j7 -> 0)...")
            robot.MoveJ([0.0] * 7)
            print("[INFO] At home.")

        # Final summary line — errors prominent, easy to grep / log.
        print("=" * 64)
        print(f"[SUMMARY] target={cfg['target']}  "
              f"pos_err={pos_err:.4f} mm  angle_err={angle_deg:.4f} deg  "
              f"j7_locked={j7_locked:.3f}  homed={cfg['go_home']}")
        print("=" * 64)
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)


if __name__ == "__main__":
    main()
