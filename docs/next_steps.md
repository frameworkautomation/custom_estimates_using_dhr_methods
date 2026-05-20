# What To Do Next

## How to resume a Claude session

Open the repo, start a new Claude Code session. It reads `CLAUDE.md` automatically —
full context is there. Nothing else needed.

---

## What you need to do (in order)

### 1. Populate `robo_dk_output/path_config.yaml`

Open the file. It has a template. You need to fill in:

**Waypoints** — 3 or 4 positions along the rail covering your first 1-2 machines.
Each needs explicit joint values `[j1, j2, j3, j4, j5, j6, j7]`. The simplest way
is to manually jog the robot in RoboDK to a safe transit pose, then copy the joints
at each j7 position you care about.

Example for 3 j7 positions:
```yaml
waypoints:
  home:
    joints: [0, 0, 0, 0, 0, 0, 0]
  transport_j7_0:
    joints: [0, -55, 30, 0, -30, -90, 0]
  transport_j7_500:
    joints: [0, -55, 30, 0, -30, -90, 500]
  transport_j7_1000:
    joints: [0, -55, 30, 0, -30, -90, 1000]

routing_candidates:
  - home
  - transport_j7_0
  - transport_j7_500
  - transport_j7_1000
```

**Destination groups** — which `cone_grab_*` targets belong to which machine.
Look at the station item names in RoboDK and list them:
```yaml
destination_groups:
  machine_1:
    cones: [cone_grab_1, cone_grab_2, cone_grab_3]
  machine_2:
    cones: [cone_grab_4, cone_grab_5, cone_grab_6]
```

### 2. Check base cone IK exists

```bash
ls ik_solutions/
```

If empty, run:
```bash
python robodk_code/check_base_cone_reachability.py
```
(RoboDK must be open with the station loaded.)

### 3. Run the collision evaluator

```bash
python robodk_code/check_collision_free_paths.py
```

This connects to RoboDK, attaches the cone mesh to the tool, tests every directed
edge between all nodes, finds gateway waypoints per machine group, and writes
`robo_dk_output/path_plan.yaml`. Takes a while — leave it running.

### 4. Visualise the result in RoboDK

```bash
python robodk_code/load_path_plan_to_robodk.py
```

This adds a `PathPlan/` frame to the RoboDK station tree with every planned
approach and grab position as a joint target. Inspect it visually — does it look right?
Skipped cones (IK failed or no collision-free path) are printed to console.

### 5. Test end-to-end

```bash
python robodk_code/moving_a_cone.py --mode ai --base 0 --dest 0
```

Check `robo_dk_output/move_debug_<timestamp>.txt` for placement error — should be < 1mm.

---

## What Claude needs to do next (code)

Nothing blocking — all code is complete. Future work when you're ready:

| Task | What it is |
|------|-----------|
| `generate_path_config.py` | Walks RoboDK item tree, auto-discovers cone groups by parent frame, merges into path_config.yaml without overwriting manual sections |
| Grasshopper waypoint export | GhPython component: define waypoints visually as Rhino planes, solve IK, write to path_config.yaml |
| `grab_family` in moving_a_cone.py | At runtime, pick the base cone family member nearest the robot's current j7 — avoids unnecessary rail travel |
| `pose_family` chain checker | When a set of adjacent j7 nodes all have the same arm pose AND a tested collision-free chain between them, mark them as a family to skip redundant arm tests |
| Investigate runtime collision guard | DHR runs `MoveJ_Test` live before each move (Redis-cached). We pre-compute offline. Investigate: (1) does DHR's `collision_checking: false` disable this in production? (2) should we add a runtime guard in `moving_a_cone.py` as a safety net for environment changes? See CLAUDE.md for the full safety argument. |
| Automated obstacle avoidance (long-term) | Options: dense PRM pre-sample, guided RRT using `MoveJ_Test` as oracle, or DHR-style `OptimizationApproachMachine_N` frames. See CLAUDE.md future section for detail. Not needed yet. |

---

## Path planning / IK questions

**"Do you have a solver to create paths between joints that might have obstacles?"**

No — and neither does the current system. The current approach is:
- Human specifies waypoints (transit poses) that they believe navigate around obstacles
- `check_collision_free_paths.py` validates every edge between them using `MoveJ_Test`
- If an edge collides, it's marked blocked — you add more intermediate waypoints and re-run

RoboDK's `MoveJ_Test` checks whether a straight joint-space interpolation between two
configurations is collision-free. It does NOT find a path around obstacles.

**"What about obstacle-avoiding IK on 7 joints?"**

This is a full motion planning problem (RRT, PRM, OMPL etc.). RoboDK does not expose
this directly. Options if needed later:

- **MoveL_Test / path planning in RoboDK Pro** — RoboDK Pro has some built-in avoidance
- **OMPL via ROS MoveIt** — full motion planning, requires ROS integration, significant work
- **Manual waypoints** — the current approach. Works well when a human can see the obstacles
  and add transit poses. For 28 machines with relatively uniform layout, this is probably
  sufficient — each machine zone gets 1-2 gateway waypoints that navigate the curtain/enclosure

For this project the manual waypoint approach is the right call. The robot cell has a
known, static layout. You add waypoints once, re-run the checker, done.
