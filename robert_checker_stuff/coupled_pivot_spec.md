# Coupled Pivot Solver Spec — Task 4c

## Forward Sequence

| Step | Tool | Move | From | To | Constraint |
|------|------|------|------|-----|------------|
| F1 | suction | JMove | suction_offset_2 | suction_offset_1 | Search A: unconstrained |
| F2 | suction | LMove | suction_offset_1 | suction_position | Search B: coupled Z-rotation |
| F3 | suction | LMove | suction_position | pivot_position | Search B: pickup TCP must land on before_pickup_offset |
| F4 | — | tool switch | — | — | same joints as F3 end, suction → pickup |
| F5 | pickup | LMove | before_pickup_offset | cone_pickup_pose | Search B: orientation preserved, no config flip |
| F6 | pickup | LMove | cone_pickup_pose | post_pickup_above | Search C: Z-free |

## Poses to Set Up in RoboDK

| Pose | Description | Setup |
|------|-------------|-------|
| suction_offset_2 | Safe JMove target away from bin area | Manual (RoboDK) |
| suction_offset_1 | LMove-safe approach to suction position | Manual (RoboDK) |
| suction_position | Where vacuum grabs the string on the cone | Manual (RoboDK) |
| before_pickup_offset | Where pickup tool needs to be before cone grab | Manual (RoboDK) |
| cone_pickup_pose | Where pickup tool grabs the cone | Manual (RoboDK) |
| post_pickup_above | 30cm above cone hole, post-pickup safe position | Manual (RoboDK) |
| pivot_position | Derived: suction TCP on cone + pickup TCP on before_pickup_offset | Solver computes |

## Three Searches

| Search | Steps | Search Variable | Description |
|--------|-------|-----------------|-------------|
| Search A | F1 | free | JMove to suction_offset_1 — unconstrained, solve independently |
| Search B | F2–F5 | single Z-rotation | Coupled chain: suction_position → pivot_position → tool switch → cone_pickup_pose. One Z-rotation must make all poses reachable with consistent joint configs |
| Search C | F6 | Z-free | LMove from cone_pickup_pose to post_pickup_above — solve independently with Z-rotation freedom |

## Constraints

1. **F1**: Unconstrained — any reachable config works, JMove so no path continuity needed.
2. **F2**: LMove with suction tool. suction_position is a known target pose from config.
3. **F3**: The critical coupled constraint. LMove from suction_position to pivot_position. The pivot_position is defined by: at this joint configuration, the pickup TCP coincides with before_pickup_offset. The Z-rotation applied to the suction grab pose determines where the pivot lands.
4. **F4**: Zero-movement tool switch. Same joints as F3 end. Only the active tool changes.
5. **F5**: LMove from before_pickup_offset to cone_pickup_pose. Orientation must be preserved throughout the linear move. Joint config flags (REAR/LOWERARM/FLIP) must match F3's end config — no flips allowed.
6. **F6**: LMove to post_pickup_above. Z-rotation is free — just needs to be reachable.

## Pipeline

1. **Assert prereqs:** Verify all 6 manual poses exist in the RoboDK station. Assert on each — fail loudly with the missing pose name if any are absent.

2. **Geometry step:** Compute pivot_position — given the known TCP offsets of suction tool and pickup tool on the end effector, find the robot pose where suction TCP = suction_position and pickup TCP = before_pickup_offset. Pure math, no IK.

3. **Search B (coupled Z-rotation sweep):** For each Z-rotation angle (step size from config, e.g. 5 degrees):
   - Rotate suction_position by angle about its Z-axis
   - Recompute pivot_position for that rotated suction pose
   - Solve IK for the F2–F5 chain (suction_position → pivot_position → before_pickup_offset → cone_pickup_pose)
   - Check all reachable with consistent joint config (REAR/LOWERARM/FLIP flags match, no config flips)
   - If yes → store winning angle + all joint solutions, stop

4. **Search A:** Solve F1 independently (JMove, unconstrained)

5. **Search C:** Solve F6 independently (Z-free rotation sweep)

6. **Assemble results:** Combine all three searches into a single result. If any search fails, the cone is marked infeasible.

7. **Cache results:** Write winning Z-rotation angle, all joint solutions per step, and pass/fail to results JSON.

8. **Program population:** Build the RoboDK program using the cached joint solutions, applying the consistent Z-rotation from Search B across F2–F5.

## Future Concerns

- **J1 wrap near +185 / -185 limits:** Search B may need to constrain J1 to avoid wrapping across the ±185 boundary between consecutive poses. Not blocking now.
- **Wrist configuration flips:** Same concern — may need to lock wrist config flags across the chain. Not blocking now.
- **Collisions:** Not checked in this iteration. Future work.

---

## Implementation Plan

### File: `robert_checker_stuff/coupled_pivot_demo.py`

Single new script (follows the pattern of `back_bin_reachability_demo.py`). Connects to RoboDK, runs the solver, builds a program you can step through.

### Step 1: Assert prereqs

Connect to RoboDK, find the robot, then assert that all 6 manual poses exist as frames in the station:

- `suction_offset_2`
- `suction_offset_1`
- `suction_position`
- `before_pickup_offset`
- `cone_pickup_pose`
- `post_pickup_above`

Also assert both tools exist: `pickup` and `knotting` (suction tool).

Fail loudly with the missing name if any are absent.

### Step 2: Read poses from station

Read the `PoseAbs()` (or `PoseWrt(robot_base)`) of each frame. These are the input poses for the solver.

### Step 3: Geometry — compute pivot_position for a given Z-rotation

For a given Z-rotation angle applied to suction_position:
1. Rotate suction_position pose about its Z-axis by the angle
2. Read the TCP offset of the suction tool and the pickup tool from RoboDK (`tool.PoseTool()`)
3. The pivot_position is the robot flange pose where:
   - flange × suction_TCP = rotated suction_position (the suction tip is on the cone)
   - flange × pickup_TCP = some position (we need this to equal before_pickup_offset)
4. Compute: `flange_pose = rotated_suction_position × inv(suction_TCP)`
5. Then: `pickup_at_pivot = flange_pose × pickup_TCP`
6. The pivot is valid if `pickup_at_pivot ≈ before_pickup_offset` — but since we're sweeping Z-rotation, we don't check equality here; instead we use `flange_pose` as the IK target and check if the whole chain solves.

Actually — re-reading the sequence: the pivot IS the flange pose. The robot moves to make the suction tip land on the rotated suction_position, and at that same flange pose the pickup tool happens to be at before_pickup_offset. The Z-rotation is the search variable that makes both true simultaneously.

So for each Z angle:
- `rotated_suction = suction_position × rotz(angle)`
- `flange_at_pivot = rotated_suction × inv(suction_TCP)`
- `pickup_result = flange_at_pivot × pickup_TCP`
- Check: does `pickup_result` position ≈ `before_pickup_offset` position? (within tolerance)

Wait — this is over-constrained. The Z-rotation changes where the suction tip points, but the pickup TCP position relative to the flange is fixed. So we're looking for the Z angle where the pickup TCP lands on before_pickup_offset.

### Step 4: Search B — coupled Z-rotation sweep

For each angle in `range(0, 360, step_deg)`:
1. Compute `rotated_suction = suction_position × rotz(angle)`
2. Compute `flange_at_pivot = rotated_suction × inv(suction_TCP)`
3. Compute `pickup_at_pivot = flange_at_pivot × pickup_TCP`
4. Check if `pickup_at_pivot` position is close to `before_pickup_offset` position (e.g. within 5mm)
5. If geometry passes, solve IK for the chain:
   - Set tool = suction → solve IK at `suction_offset_1` pose (F2 start, seeded from F1)
   - Set tool = suction → solve IK at `rotated_suction` (F2 end / F3 start)
   - Set tool = suction → solve IK at `flange_at_pivot` (F3 end = pivot_position)
   - Check joint config matches across F2–F3
   - Set tool = pickup → solve IK at `before_pickup_offset` (F5 start, same joints as pivot)
   - Set tool = pickup → solve IK at `cone_pickup_pose` (F5 end)
   - Check joint config matches across F5
6. If all pass → winner. Store angle + all joints.

### Step 5: Search A — solve F1

Solve IK for `suction_offset_1` with suction tool, unconstrained (try_ik with home seed). This is a JMove so no config continuity needed.

### Step 6: Search C — solve F6

Solve IK for `post_pickup_above` with pickup tool, Z-free sweep (reuse `solve_with_z_sweep` from `setup_base_movements.py`).

### Step 7: Build RoboDK program

Create a program `coupled_pivot_demo` (delete old one if re-running) with:

```
Set tool = suction
MoveJ → suction_offset_2 target (home/safe)
MoveJ → suction_offset_1 target         (F1)
MoveL → suction_position target          (F2)
MoveL → pivot_position target            (F3)
Set tool = pickup                         (F4)
MoveL → cone_pickup_pose target          (F5)
MoveL → post_pickup_above target         (F6)
MoveJ → home                             (return)
```

Each target is created in a `coupled_pivot_targets` folder with the solved joints baked in.

### Step 8: Print results

Print a summary: which Z-rotation won, joints at each step, pass/fail. Save to `coupled_pivot_results.json`.

### IK helpers

Reuse from existing code:
- `try_ik()` from `setup_base_movements.py` (6-DOF, OptimAxes with j2/j3 constraints)
- `solve_with_z_sweep()` from `setup_base_movements.py` (for Search C)
- `_solve_ik_locked_j7()` from `robert_end_checker.py` (if on 7-DOF station)
- `robot.JointsConfig()` for config flag checking

The script will import or copy the helpers it needs. Since `setup_base_movements.py` is a standalone script (not a library), the helpers will be copied into `coupled_pivot_demo.py` or extracted into a shared module.

### CLI

```
python robert_checker_stuff/coupled_pivot_demo.py
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --step-deg 5
```
