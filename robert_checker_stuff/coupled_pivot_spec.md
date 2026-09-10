# Coupled Pivot Solver Spec — Task 4c (Simplified Sequence)

## Key Insight (from Robert)

The suction offset's purpose is to avoid convoluted wrist movements — NOT to
manage string slack. This means the robot just needs an LMove from a slight
offset to the grab pose, then retracts back to that offset before pivoting.
No separate "pivot_before" approach pose needed.

## Forward Sequence

| Step | Tool | Move | From | To | Notes |
|------|------|------|------|----|-------|
| F1 | knotting | JMove | home | suction_offset_1 | Safe approach, Z-free |
| F2 | knotting | JMove | suction_offset_1 | suction_offset_2 | Closer approach |
| F3 | knotting | LMove | suction_offset_2 | suction_position | Grab the string |
| F4 | knotting | LMove | suction_position | suction_offset_2 | Retract with string |
| F5 | knotting | LMove | suction_offset_2 | pivot_after | Move to pivot pose |
| F6 | — | tool switch | — | — | Same joints, knotting → pickup |
| F7 | pickup | LMove | before_pickup_offset | cone_pickup_pose | Grab the cone |
| F8 | pickup | LMove | cone_pickup_pose | post_pickup_above | Lift out |
| F9 | pickup | JMove | post_pickup_above | home | Return |

## Poses

| Pose | Description | Setup |
|------|-------------|-------|
| suction_offset_1 | Safe JMove target away from bin area | Manual (RoboDK) |
| suction_offset_2 | LMove-safe approach/retract near suction position | Manual (RoboDK) |
| suction_position | Where vacuum grabs the string on the cone | Manual (RoboDK) |
| before_pickup_offset | Where pickup tool needs to be before cone grab | Manual (RoboDK) |
| cone_pickup_pose | Where pickup tool grabs the cone | Manual (RoboDK) |
| post_pickup_above | Above cone hole, post-pickup safe position | Manual (RoboDK) |
| pivot_after | Derived: `rotated_before_pickup × T_pickup_to_suction` | Solver computes |

## Three Independent Sweeps

| Sweep | Tool | Poses Solved | What it proves |
|-------|------|-------------|----------------|
| Suction | knotting | suction_offset_2, suction_position | F2–F4 chain is reachable |
| Pivot | knotting | pivot_after | F5 is reachable (derived from before_pickup_offset) |
| Pickup | pickup | cone_pickup_pose, post_pickup_above | F7–F8 chain is reachable |

Each sweep tries all thetas (0°, 15°, 30°, ...) × 2 seeds (J1=+180, J1=-180).

## Matching

Solutions from suction and pivot sweeps need matching wrist configs
`[REAR, LOWERARM, FLIP]` because F4→F5 is an LMove (suction_offset_2 → pivot_after).
Pickup solutions also need matching config because the tool switch at F6 preserves
joint configuration.

**Config overlap across all three groups = viable full sequence.**

## Seeds

Two seeds per theta:
- `seeded_at_p180`: J1=+180, rest=0
- `seeded_at_n180`: J1=-180, rest=0

## RoboDK Folder Structure

```
discovered_targets/
  <cone_name>/
    suction_solutions/
      seeded_at_p180/
        config_R0_L0_F0/
          suction_offset_2_theta_060
          suction_position_theta_060
    pivot_solutions/
      seeded_at_p180/
        config_R0_L0_F0/
          pivot_after_theta_060
    pickup_solutions/
      seeded_at_p180/
        config_R0_L0_F0/
          cone_pickup_pose_theta_060
          post_pickup_above_theta_060
```

## Pipeline

1. **Assert prereqs:** Discover cones, assert 6 child frames each, cache refs.
2. **Read poses:** `PoseAbs()` for each frame.
3. **Compute T_pickup_to_suction:** `inv(pickup_TCP) × suction_TCP` (once).
4. **Clean up:** Delete old `discovered_targets` folder.
5. **Phase 4 — Suction sweep:** For each theta × seed, solve offset_2 + suction_position.
6. **Phase 4b — Pickup sweep:** For each theta × seed, solve cone_pickup + post_pickup_above.
7. **Phase 5 — Pivot sweep:** For each theta × seed, compute pivot_after, solve IK.
8. **Save solutions:** Create folder hierarchy with joint targets in RoboDK.
9. **Check config overlap:** Find configs present in all three groups.
10. **Phase 6 — Summary:** Print per-cone counts and overall results.

## Pivot Geometry

**pivot_after** = where suction TCP sits when pickup TCP is on before_pickup_offset:
```
pivot_after = rotated_before_pickup × inv(pickup_TCP) × suction_TCP
            = rotated_before_pickup × T_pickup_to_suction
```

## Deferred Work (Phase D)

- **LMove verification:** Verify matched pairs can execute LMoves between
  consecutive poses without IK failures along the path.
- **Program building:** Build RoboDK programs from verified pairs.
- **Collision checking:** Not implemented in this iteration.

## CLI

```
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --step-deg 10
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --skip 3 4
```

Default step size: 15°.
