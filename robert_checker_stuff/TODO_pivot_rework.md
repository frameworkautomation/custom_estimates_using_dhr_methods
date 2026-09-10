# TODO: Pivot Rework — Split pivot into before/after + decouple from pickup

## Context

The current coupled pivot solver tries to find ONE theta where the entire chain works. This is too constrained. The pivot needs to be split into two poses and the search needs to be decoupled.

## Changes Needed

### 1. Split pivot into pivot_before and pivot_after

- **pivot_after**: same as current pivot math — `rotated_before_pickup * T_pickup_to_suction`. This is where the suction tip is when the pickup tip is at before_pickup_offset.
- **pivot_before**: same pose as pivot_after but rotated about the red axis (local X-axis). This is the approach pose before the pivot rotation happens.

### 2. Decouple the search — don't require one theta for everything

Instead of finding one theta where the whole chain works:

- **Phase A**: For each theta, check suction poses (suction_offset_1, suction_offset_2, suction_position). Track all thetas that work + their wrist configurations.
- **Phase B**: For each theta, check pivot_before and pivot_after. Track all thetas that work + their wrist configurations.
- **Phase C**: Find matching pairs — a suction theta and a pivot theta where:
  1. Wrist configurations match between suction_position and pivot_before
  2. An LMove exists between suction_position and pivot_before (path feasibility, not just endpoint reachability)

### 3. Track wrist configurations

For every solved pose, record `robot.JointsConfig()` — the [REAR, LOWERARM, FLIP] flags. Matching pairs must have the same wrist config to ensure LMove feasibility between them.

### 4. LMove feasibility check (later stage)

After finding candidate pairs from Phase A + B matching, verify an LMove path exists between:
- suction_position (at suction theta) → pivot_before (at pivot theta)
- pivot_before → pivot_after (the actual pivot rotation)
- pivot_after → before_pickup_offset (tool switch happens here)

This is a separate stage — don't block on it for the initial rework.

## Summary of new search structure

```
Phase A: sweep theta for suction chain
  → list of (theta, joints_offset1, joints_offset2, joints_suction, wrist_cfg)

Phase B: sweep theta for pivot pair
  → list of (theta, joints_pivot_before, joints_pivot_after, wrist_cfg)

Phase C: match pairs where wrist_cfg matches between suction end and pivot start

Phase D (later): verify LMove paths between matched pairs
```
