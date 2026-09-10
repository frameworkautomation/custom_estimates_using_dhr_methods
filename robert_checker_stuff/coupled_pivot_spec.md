# Coupled Pivot Solver Spec — Task 4c (Reworked: Decoupled Search)

## Key Change from v1

The pivot is split into two poses (**pivot_before** and **pivot_after**), the
search is decoupled into independent suction and pivot sweeps, and viable pairs
are matched by wrist configuration. All solutions are saved as RoboDK joint
targets in a structured folder hierarchy.

## Poses

| Pose | Description | Setup |
|------|-------------|-------|
| suction_offset_1 | Safe JMove target away from bin area | Manual (RoboDK) |
| suction_offset_2 | LMove-safe approach to suction position | Manual (RoboDK) |
| suction_position | Where vacuum grabs the string on the cone | Manual (RoboDK) |
| before_pickup_offset | Where pickup tool needs to be before cone grab | Manual (RoboDK) |
| cone_pickup_pose | Where pickup tool grabs the cone | Manual (RoboDK) |
| post_pickup_above | 30cm above cone hole, post-pickup safe position | Manual (RoboDK) |
| pivot_after | Derived: `rotated_before_pickup × T_pickup_to_suction` — suction TCP when pickup is on before_pickup_offset | Solver computes |
| pivot_before | Derived: `pivot_after × rotx(90°)` — approach pose rotated 90° about local X from pivot_after | Solver computes |

## Two Independent Sweeps

| Sweep | Poses Solved | Search Variable | Description |
|-------|-------------|-----------------|-------------|
| Suction sweep | suction_offset_1, suction_offset_2, suction_position | Z-rotation × 2 seeds | All three must solve at same theta + seed |
| Pivot sweep | pivot_before, pivot_after | Z-rotation × 2 seeds | Both must solve at same theta + seed |

## Seeds

Two seeds are tried per theta angle:
- `seeded_at_p180`: J1=+180, rest=0
- `seeded_at_n180`: J1=-180, rest=0

Each seed may produce different wrist configurations (REAR/LOWERARM/FLIP).

## Matching (Phase C)

After both sweeps complete, suction and pivot solutions are matched by wrist
config `[REAR, LOWERARM, FLIP]`. A match means the suction chain and pivot
chain can potentially be connected with LMoves (same arm configuration = no
config flips mid-path).

## RoboDK Folder Structure

```
discovered_targets/
  <cone_name>/
    suction_solutions/
      seeded_at_p180/
        config_R0_L0_F0/
          suction_offset_1_theta_060
          suction_offset_2_theta_060
          suction_position_theta_060
        config_R0_L0_F1/
          ...
      seeded_at_n180/
        ...
    pivot_solutions/
      seeded_at_p180/
        config_R0_L0_F0/
          pivot_before_theta_060
          pivot_after_theta_060
        ...
      seeded_at_n180/
        ...
```

Each target is a joint target (`setAsJointTarget()` with solved joints baked in).

## Pipeline

1. **Assert prereqs:** Discover cones, assert 6 child frames each, cache refs.
2. **Read poses:** `PoseAbs()` for each frame.
3. **Compute T_pickup_to_suction:** `inv(pickup_TCP) × suction_TCP` (once).
4. **Clean up:** Delete old `discovered_targets` folder if it exists.
5. **Per cone — Phase A (suction sweep):** For each theta × seed, solve IK for
   suction_offset_1, suction_offset_2, suction_position. Record all successes
   with wrist config.
6. **Per cone — Phase B (pivot sweep):** For each theta × seed, compute
   pivot_after and pivot_before, solve IK for both. Record all successes with
   wrist config.
7. **Save solutions:** Create folder hierarchy with joint targets in RoboDK.
8. **Per cone — Phase C (match):** Find suction/pivot pairs with matching
   wrist config. Print matches to console.
9. **Summary:** Print per-cone counts and overall results.

## Pivot Geometry

**pivot_after** = where suction TCP sits when pickup TCP is on before_pickup_offset:
```
pivot_after = rotated_before_pickup × inv(pickup_TCP) × suction_TCP
            = rotated_before_pickup × T_pickup_to_suction
```

**pivot_before** = approach pose, rotated 90° about local X from pivot_after:
```
pivot_before = pivot_after × rotx(π/2)
```

This 90° approach angle is hardcoded as `PIVOT_APPROACH_ANGLE_DEG = 90`.

## Deferred Work (Phase D)

- **LMove verification:** Verify that matched pairs can actually execute LMoves
  between consecutive poses without IK failures along the path.
- **Program building:** Build RoboDK programs from verified pairs (removed from
  this iteration — premature until LMove verification is done).
- **Collision checking:** Not implemented in this iteration.

## CLI

```
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1
python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --step-deg 10
```

Default step size: 15° (was 5° in v1).
