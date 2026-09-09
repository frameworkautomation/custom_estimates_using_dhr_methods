# Coupled Pivot Solver Spec — Task 4c

## Forward Sequence

| Step | Tool | Move | From | To | Constraint |
|------|------|------|------|-----|------------|
| F1 | suction | JMove | suction_offset_2 | suction_offset_1 | Search A: unconstrained |
| F2 | suction | LMove | suction_offset_1 | suction_position | Search B: coupled Z-rotation |
| F3 | suction | LMove | suction_position | pivot_as_suction_tcp | Search B: derived pose — flange puts pickup TCP on before_pickup_offset |
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
| pivot_as_suction_tcp | Derived: `before_pickup_offset × inv(pickup_TCP) × suction_TCP` — where suction tip sits when pickup is on before_pickup_offset | Solver computes |

## Three Searches

| Search | Steps | Search Variable | Description |
|--------|-------|-----------------|-------------|
| Search A | F1 | free | JMove to suction_offset_1 — unconstrained, solve independently |
| Search B | F2–F5 | single Z-rotation | Coupled chain: suction_position → pivot_as_suction_tcp → tool switch → cone_pickup_pose. One Z-rotation must make all poses reachable with consistent joint configs |
| Search C | F6 | Z-free | LMove from cone_pickup_pose to post_pickup_above — solve independently with Z-rotation freedom |

## Constraints

1. **F1**: Unconstrained — any reachable config works, JMove so no path continuity needed.
2. **F2**: LMove with suction tool. suction_position is a known target pose from config.
3. **F3**: The critical coupled constraint. LMove from suction_position to `pivot_as_suction_tcp`. This pose is computed as `before_pickup_offset × inv(pickup_TCP) × suction_TCP` — the suction tip position when the flange is positioned so pickup TCP lands on before_pickup_offset. It's deterministic (computed once per cone), not searched.
4. **F4**: Zero-movement tool switch. Same joints as F3 end. Only the active tool changes.
5. **F5**: LMove from before_pickup_offset to cone_pickup_pose. Orientation must be preserved throughout the linear move. Joint config flags (REAR/LOWERARM/FLIP) must match F3's end config — no flips allowed.
6. **F6**: LMove to post_pickup_above. Z-rotation is free — just needs to be reachable.

## Pipeline

1. **Assert prereqs:** Discover all cones in the bin. For each cone, assert all 6 manual pose frames exist. Cache tree locations (cone → 6 frame item refs). Assert both tools (`pickup`, `knotting`) exist.

2. **Compute pickup-to-suction transform (once):**
   - `suction_TCP = knotting_tool.PoseTool()`
   - `pickup_TCP = pickup_tool.PoseTool()`
   - `T_pickup_to_suction = inv(pickup_TCP) × suction_TCP`

3. **For each cone — Search B (coupled Z-rotation sweep):** For each θ in `range(0, 360, step_deg)`:
   - `rotated_suction = suction_position × rotz(θ)`
   - `pivot_as_suction_tcp = before_pickup_offset × T_pickup_to_suction` (recomputed at each θ)
   - Solve IK chain: `suction_offset_1` → `rotated_suction` → `pivot_as_suction_tcp` (suction tool)
   - FK verify pivot: at pivot_joints, switch to pickup, check achieved pickup TCP ≈ `before_pickup_offset` (within tolerance)
   - Continue chain: `before_pickup_offset` → `cone_pickup_pose` (pickup tool)
   - Check joint config flags consistent across chain
   - If all pass → store θ + all joint solutions, stop

4. **For each cone — Search A:** Solve F1 independently (JMove to suction_offset_1, unconstrained)

5. **For each cone — Search C:** Solve F6 independently (LMove from cone_pickup_pose to post_pickup_above, Z-free sweep)

6. **Assemble results:** Combine all three searches per cone. If any search fails, cone is marked infeasible.

7. **Program population:** For each feasible cone, build a `<cone>_coupled_pivot` RoboDK program with baked-in joint targets from the solutions.

## Future Concerns

- **J1 wrap near +185 / -185 limits:** Search B may need to constrain J1 to avoid wrapping across the ±185 boundary between consecutive poses. Not blocking now.
- **Wrist configuration flips:** Same concern — may need to lock wrist config flags across the chain. Not blocking now.
- **Collisions:** Not checked in this iteration. Future work.

---

## Implementation Plan

### File: `robert_checker_stuff/coupled_pivot_demo.py`

Single new script (follows the pattern of `back_bin_reachability_demo.py`). Connects to RoboDK, runs the solver, builds a program you can step through.

### Step 1: Assert prereqs

Connect to RoboDK, find the robot, then:

**Assert tools exist:**
- `pickup`
- `knotting` (suction tool)

**Assert the 6 manual pose frames exist for every cone in the bin.** The script discovers all cones in the bin (same pattern as `setup_base_movements.py`), then for each cone asserts that these 6 child frames exist under it:

- `<cone>_suction_offset_2`
- `<cone>_suction_offset_1`
- `<cone>_suction_position`
- `<cone>_before_pickup_offset`
- `<cone>_cone_pickup_pose`
- `<cone>_post_pickup_above`

Fail loudly with the missing name if any are absent.

**Cache tree locations:** After discovery, save a dict mapping each cone name to its 6 frame RoboDK item references (and their tree paths) so we don't re-traverse the station tree later. This cache is used by all subsequent steps.

### Step 2: Read poses from station

Read the `PoseAbs()` (or `PoseWrt(robot_base)`) of each frame. These are the input poses for the solver.

### Step 2.5: Compute pickup-to-suction transform (once)

Read both tool TCP offsets from RoboDK:
- `suction_TCP = knotting_tool.PoseTool()` — flange → suction tip
- `pickup_TCP = pickup_tool.PoseTool()` — flange → pickup tip

Compute the fixed rigid transform from pickup tip frame to suction tip frame:
```
T_pickup_to_suction = inv(pickup_TCP) × suction_TCP
```

This transform is constant for the entire run — it only depends on the tool geometry.

### Step 3: Geometry — pivot_as_suction_tcp is deterministic, not searched

**Equation:**
```
pivot_as_suction_tcp = before_pickup_offset × T_pickup_to_suction
```

This gives the suction tip pose when the pickup tip is at `before_pickup_offset`. Computed once per cone, does NOT change during the Z-rotation sweep.

**Why this works:**

When the robot is at the pivot pose (flange at F):
- pickup tip = `F × pickup_TCP = before_pickup_offset`
- suction tip = `F × suction_TCP`

Substitute `F = before_pickup_offset × inv(pickup_TCP)`:
```
suction tip = before_pickup_offset × inv(pickup_TCP) × suction_TCP
            = before_pickup_offset × T_pickup_to_suction
            = pivot_as_suction_tcp
```

**Verification (round-trip):** from pivot_as_suction_tcp, recover pickup position:
```
pivot_as_suction_tcp × inv(T_pickup_to_suction)
= before_pickup_offset × T_pickup_to_suction × inv(T_pickup_to_suction)
= before_pickup_offset  ✓
```

So at F3's end, tool switch (F4, same joints) puts pickup TCP exactly at before_pickup_offset.

**What Z-rotation does:**
The cone is round so the suction grab orientation has freedom about Z. For each angle θ:
- `rotated_suction = suction_position × rotz(θ)` — grab the string at this orientation
- The robot must LMove (suction tool) from `rotated_suction` to `pivot_as_suction_tcp`
- Then tool switch (same joints)
- Then LMove (pickup tool) from `before_pickup_offset` to `cone_pickup_pose`

The search finds a θ where the entire chain is IK-reachable with consistent joint configs.

### Step 4: Search B — coupled Z-rotation sweep

**For each angle θ in `range(0, 360, step_deg)`:**
1. `rotated_suction = suction_position × rotz(θ)`
2. `pivot_as_suction_tcp = before_pickup_offset × T_pickup_to_suction` (recomputed at each θ)
3. Solve IK chain (all must succeed with consistent joint configs):
   - Set tool = suction → solve IK at `suction_offset_1` (F2 from-pose)
   - Set tool = suction → solve IK at `rotated_suction` (F2 to-pose = grab)
   - Set tool = suction → solve IK at `pivot_as_suction_tcp` (F3 to-pose = pivot) → get pivot_joints
   - FK verify pivot: set pivot_joints, switch to pickup tool, read achieved pickup TCP. Check `‖achieved_pickup_tcp − before_pickup_offset‖ < tolerance` (e.g. 5mm). If drift too large, reject this θ.
   - Check joint config flags match across F2–F3
   - Set tool = pickup → solve IK at `cone_pickup_pose` seeded from pivot_joints (F5 to-pose)
   - Check joint config flags match across F4–F5
4. If all pass → winner. Store θ + all joint solutions.

### Step 5: Search A — solve F1

Solve IK for `suction_offset_1` with suction tool, unconstrained (try_ik with home seed). This is a JMove so no config continuity needed.

### Step 6: Search C — solve F6

Solve IK for `post_pickup_above` with pickup tool, Z-free sweep (reuse `solve_with_z_sweep` from `setup_base_movements.py`).

### Step 7: Build RoboDK programs

For each cone in the bin, create a program `<cone>_coupled_pivot` (delete old one if re-running) with:

```
Set tool = suction
MoveJ → <cone>_suction_offset_2 target (home/safe)
MoveJ → <cone>_suction_offset_1 target         (F1)
MoveL → <cone>_suction_position target          (F2)
MoveL → <cone>_pivot_as_suction_tcp target       (F3)
Set tool = pickup                                (F4)
MoveL → <cone>_cone_pickup_pose target          (F5)
MoveL → <cone>_post_pickup_above target         (F6)
MoveJ → home                                    (return)
```

Each target is created in a `coupled_pivot_targets` folder with the solved joints baked in. Uses the cached tree locations from Step 1 to find each cone's frames.

### Step 8: Print results

Print a summary: which Z-rotation won, joints at each step, pass/fail per cone.

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
