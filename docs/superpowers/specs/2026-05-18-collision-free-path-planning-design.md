# Collision-Free Path Planning Design
**Date:** 2026-05-18
**Branch:** `collision_free_path_planning`
**Status:** Approved

---

## Overview

Two deliverables:

1. **Bug fix** in `moving_a_cone.py` — cone mesh placed at wrong world position after pick-and-place because `setPose` uses the wrong coordinate frame.
2. **Collision-free path planner** — offline checker that tests all possible paths between human-specified waypoints and cone targets, finds collision-free gateway waypoints per destination group, and writes a plan that `moving_a_cone.py` reads at runtime.

---

## 1. Bug Fix — `moving_a_cone.py`

**Location:** lines ~811–816
**Root cause:** After releasing the cone mesh, code does:
```python
cone_mesh.setParentStatic(world_frame)
cone_mesh.setPose(tgt_grab_pose)  # assumes world_frame.PoseAbs() == identity
```
`setPose` sets the local pose relative to the parent. If `world_frame.PoseAbs()` is not identity, the cone ends up in the wrong world position.

**Fix:**
```python
cone_mesh.setParentStatic(world_frame)
cone_mesh.setPose(invH(world_frame.PoseAbs()) * tgt_grab_pose)
```

Add diagnostic logging of `world_frame.PoseAbs()` and `cone_mesh.PoseAbs()` after the set to verify placement.

---

## 2. Collision-Free Path Planner

### Architecture

```
robo_dk_output/path_config.yaml        ← human edits (waypoints, groups, collision pairs)
         │
         ▼
robodk_code/check_collision_free_paths.py   ← offline checker
         │  - resolves target names to joint values via RoboDK
         │  - attaches cone mesh to tool (worst-case geometry)
         │  - tests all directed edges with robot.MoveJ_Test()
         │    (RoboDK interpolates full trajectory; every intermediate
         │     configuration along the arc is checked, not just endpoints)
         │  - finds gateway waypoints per destination group
         │  - excludes any cone with IK failure or no collision-free path
         │  - writes path_plan.yaml
         ▼
robo_dk_output/path_plan.yaml          ← auto-generated, never hand-edited
         │
         ▼
robodk_code/moving_a_cone.py           ← reads path_plan.yaml at runtime
                                          no IK, no collision testing at execution time

robodk_code/robot_controller.py        ← mixin pipeline (used by checker and mover)
```

### robot_controller.py

Modelled directly on the knitwear-cell mixin pattern. Three mixins, composed into a `Robot` class:

```python
Robot(OptimKinematicsModel, MoveJTestModel)   # checker: IK + test, never executes
Robot(OptimKinematicsModel, MoveJModel)        # mover: IK + execute, no test
```

**`OptimKinematicsModel`** — computes target joints via RoboDK OptimAxes (Algorithm 3 DLS). Mirrors the existing `solve_ik` / `solve_ik_free_j7` logic, extracted into a reusable mixin. Accepts a flag for j7-constrained vs j7-free.

**`MoveJTestModel`** — calls `robot.MoveJ_Test(current_joints, target_joints)`. RoboDK internally interpolates the full joint-space arc and collision-checks at every step. Results cached by hash(from_joints + to_joints) to a local dict, written to `path_plan.yaml` edge cache on completion. No Redis — plain file cache.

**`MoveJModel`** — calls `robot.MoveJ(target)` only if the previous mixin did not set a collision result. Same pattern as knitwear-cell.

### Graph Construction

Nodes:
- Home (always included)
- All human-specified `routing_candidates` from `path_config.yaml`
- All base cone approach + grab positions (from station targets)
- All destination cone approach + grab positions (from station targets)

Edges:
- Every directed pair of nodes is tested with `MoveJ_Test`
- Cone mesh is attached to tool before all tests (worst-case geometry — if clear with
  cone, clear without)
- Tool is set per-waypoint if a `tool` override is specified, otherwise `default_tool`
- Approach ↔ grab edges are tested in **both directions** as separate edges, since the
  cone is attached on the retract leg (grab → approach) but not on the entry leg
  (approach → grab). Both must be collision-free for a cone to be usable.
- If `MoveJ_Test` returns 0: edge is collision-free
- If non-zero: edge blocked; RoboDK has tested every interpolated intermediate
  configuration along the full joint-space arc

### Gateway Discovery

For each destination group (human-defined in config):
- A waypoint qualifies as a **gateway** for that group if it has a collision-free edge to **every** cone approach in the group
- A cone with IK failure is excluded from gateway computation and from the usable list
- A group with no valid gateway is flagged; its cones are all marked `tested: false, reason: no_collision_free_path`

For each base cone:
- A waypoint qualifies as a **gateway** if it has a collision-free edge to that cone's approach
- Same exclusion rules apply

### Exclusion Rules

A cone is excluded (`tested: false`) for either of these reasons:
- `ik_failed` — IK did not converge (existing check from `check_base_cone_reachability.py`)
- `no_collision_free_path` — IK succeeded but no routing_candidate has a clear path to this cone

Any cone marked `tested: false` for any reason **never appears** in `moving_a_cone.py`'s selectable list. Any cone not present in `path_plan.yaml` is treated as untested and excluded.

---

## 3. File Formats

### path_config.yaml

```yaml
default_tool: "pickup_closed"
cone_mesh_template: "base_cone_0"   # mesh attached during all collision checks

waypoints:
  home:
    joints: [0, 0, 0, 0, 0, 0, 0]
  transport:
    joints: [0, -55, 30, 0, -30, -90, 0]
  curtain_safe_1:
    target: "approach_machine_1_curtain_safe"  # resolved to joints by checker
    tool: "pickup_open"                         # optional per-waypoint tool override

routing_candidates:
  - home
  - transport
  - curtain_safe_1

destination_groups:
  machine_1:
    cones: [cone_grab_1, cone_grab_2, cone_grab_3]
  machine_2:
    cones: [cone_grab_4, cone_grab_5, cone_grab_6]

collision_enable:
  - [robot, "machine_enclosure"]
collision_disable: []
```

### path_plan.yaml

```yaml
generated: "2026-05-18T14:32:00"
config_hash: "abc123..."   # hash of path_config.yaml; stale if mismatch

edge_cache:
  "home|transport":
    from_joints: [0, 0, 0, 0, 0, 0, 0]
    to_joints: [0, -55, 30, 0, -30, -90, 0]
    collision_free: true
  "transport|base_cone_grab_0_approach":
    from_joints: [...]
    to_joints: [...]
    collision_free: true
  "curtain_safe_1|cone_grab_1_approach":
    from_joints: [...]
    to_joints: [...]
    collision_free: true

base_cones:
  base_cone_grab_0:
    tested: true
    approach_joints: [...]
    grab_joints: [...]
    gateways: [transport]
  base_cone_grab_1:
    tested: false
    reason: "ik_failed"

destination_groups:
  machine_1:
    gateways: [curtain_safe_1]
    cones:
      cone_grab_1:
        tested: true
        approach_joints: [...]
        grab_joints: [...]
      cone_grab_2:
        tested: false
        reason: "no_collision_free_path"
  machine_2:
    gateways: []
    cones:
      cone_grab_4:
        tested: false
        reason: "no_collision_free_path"
```

---

## 4. moving_a_cone.py Integration

On startup:
- Load `path_plan.yaml`; abort with clear error if missing
- Check `config_hash` against current `path_config.yaml`; warn if stale
- Filter selectable base cones to `tested: true` only
- Filter selectable destination cones to `tested: true` only

Execution sequence for base cone N → destination cone M:
```
home
→ base_gateway_waypoint(s)    (from path_plan.yaml base_cones[N].gateways)
→ base_approach               (pre-computed joints)
→ base_grab                   (pre-computed joints; attach cone mesh)
→ base_approach               (retract — same pose, cone now attached)
→ base_gateway_waypoint(s)    (reverse back out through base gateways)
→ dest_gateway_waypoint(s)    (from path_plan.yaml destination_groups[group].gateways)
→ dest_approach               (pre-computed joints)
→ dest_grab                   (pre-computed joints; detach cone mesh, fix position)
→ dest_approach               (retract — same pose, cone now detached)
→ dest_gateway_waypoint(s)    (reverse back out through dest gateways)
→ home
```

Note: `approach → grab` and `grab → approach` are tested as separate directed edges.
The cone mesh is attached during the retract from base grab, so the `grab → approach`
edge is tested with cone attached (worst-case geometry).

All joints come from `path_plan.yaml`. No IK at execution time. No collision testing at execution time.

---

## 5. Branch Strategy

- **`determining_how_position_gripper`** — bug fix only (`moving_a_cone.py` cone placement)
- **`collision_free_path_planning`** — branched from `determining_how_position_gripper` after the fix is merged; contains all new files (`robot_controller.py`, `check_collision_free_paths.py`, `path_config.yaml`, `path_plan.yaml` schema, `moving_a_cone.py` plan-reading integration)

---

## 6. Ambiguity Resolutions

**Multiple gateways for a group:** If a destination group has more than one valid gateway, `moving_a_cone.py` picks the one that shares a tested edge with the base cone's gateway (minimising total hops). If none share an edge, pick the first listed gateway.

**Stale config hash:** If `path_plan.yaml` config hash does not match the current `path_config.yaml`, warn and continue — the plan may still be valid for cones that haven't changed. Abort only if the plan file is missing entirely.

**Finding a cone's group:** `moving_a_cone.py` looks up which destination group a selected cone belongs to by searching `destination_groups` in `path_plan.yaml` by cone name. If not found, treat as untested and exclude.
