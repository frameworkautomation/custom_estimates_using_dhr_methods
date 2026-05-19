# Cone Placement Bug Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix `moving_a_cone.py` so the cone mesh ends up at the correct world position after pick-and-place.

**Architecture:** One-line fix — replace `cone_mesh.setPose(tgt_grab_pose)` with `cone_mesh.setPose(invH(world_frame.PoseAbs()) * tgt_grab_pose)` to correctly convert the world-space target pose into the local frame of `world_frame`. Add diagnostic logging to verify the fix.

**Tech Stack:** Python, RoboDK Python API (`robomath.invH`, `Pose_2_TxyzRxyz`)

---

### Task 1: Switch to the correct branch

**Files:**
- No files changed

- [ ] **Step 1: Verify current branch**

```bash
git branch
```
Expected: you are on `determining_how_position_gripper` (not `collision_free_path_planning`). If not, run:
```bash
git checkout determining_how_position_gripper
```

---

### Task 2: Fix coordinate frame calculation and add diagnostics

**Files:**
- Modify: `robodk_code/moving_a_cone.py:811-816`

- [ ] **Step 1: Open `robodk_code/moving_a_cone.py` and locate the detach block**

Find this block (around line 811):
```python
        # Detach cone to world frame and snap to exact destination
        if cone_mesh is not None:
            cone_mesh.setParentStatic(world_frame)
            cone_mesh.setPose(tgt_grab_pose)   # world_frame is at origin, so local = world
            RDK.Render(True)
            _log("[INFO] Cone mesh placed at destination.")
```

- [ ] **Step 2: Replace with corrected version including diagnostics**

```python
        # Detach cone to world frame and snap to exact destination
        if cone_mesh is not None:
            wf_abs = world_frame.PoseAbs()
            wf_xyz = _pose_xyz(wf_abs)
            _log(f"[DEBUG] world_frame.PoseAbs() XYZ = ({wf_xyz[0]:.3f}, {wf_xyz[1]:.3f}, {wf_xyz[2]:.3f})")

            local_pose = invH(wf_abs) * tgt_grab_pose
            cone_mesh.setParentStatic(world_frame)
            cone_mesh.setPose(local_pose)
            RDK.Render(True)

            actual_abs = cone_mesh.PoseAbs()
            ax, ay, az = _pose_xyz(actual_abs)
            tx, ty, tz = _pose_xyz(tgt_grab_pose)
            err = math.sqrt((tx-ax)**2 + (ty-ay)**2 + (tz-az)**2)
            _log(f"[DEBUG] cone target  XYZ = ({tx:.3f}, {ty:.3f}, {tz:.3f})")
            _log(f"[DEBUG] cone actual  XYZ = ({ax:.3f}, {ay:.3f}, {az:.3f})")
            _log(f"[DEBUG] placement error   = {err:.3f} mm")
            _log("[INFO] Cone mesh placed at destination.")
```

- [ ] **Step 3: Verify `invH` is already imported**

Check line ~30 of `moving_a_cone.py`:
```python
from robodk.robomath import transl, invH, rotz, Pose_2_TxyzRxyz, eye
```
`invH` is already imported. No change needed.

- [ ] **Step 4: Run a pick-and-place in --mode ai and check the debug log**

```bash
cd /mnt/c/Users/samst/Framework/clones/custom_estimates_using_dhr_methods
python robodk_code/moving_a_cone.py --mode ai --base 0 --dest 0
```

Open the debug log at `robo_dk_output/move_debug_<timestamp>.txt`. Verify:
- `placement error` is near 0 mm (should be < 1 mm)
- `cone actual XYZ` matches `cone target XYZ`
- `world_frame.PoseAbs() XYZ` is logged (use this to understand if it was non-zero)

- [ ] **Step 5: Commit**

```bash
git add robodk_code/moving_a_cone.py
git commit -m "fix: correct cone placement coordinate frame using invH(world_frame.PoseAbs())"
```
