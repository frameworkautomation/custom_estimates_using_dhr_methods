# DHR Movement Reference Tables

## 1. Transport / Rotation Poses (AbsoluteJointKinematicsModel)

All share J2-J6; only J1 differs. J7 (rail) is left unchanged.

| Pose name | J1 | J2 | J3 | J4 | J5 | J6 | J7 | Purpose |
|-----------|-----|-----|-----|-----|-----|-----|-----|---------|
| `transport` | 0 | -50 | 15 | 0 | -15 | -90 | unchanged | Arm folded compact along rail (facing +Y) |
| `transport_reversed_right` | -180 | -50 | 15 | 0 | -15 | -90 | unchanged | Arm flipped to face -Y side (even-ID machines) |
| `transport_reversed_left` | +180 | -50 | 15 | 0 | -15 | -90 | unchanged | Arm flipped to face -Y side (odd-ID machines) |
| `home` | 0 | -45 | 0 | -180 | 1 | 0 | 0.01 | Full home (all 7 joints specified) |

Source: `clones/knitwear-cell/src/main/robot/state_machine.py` lines 12-45

## 2. Machine Zone Transit Sequence

Full sequence for entering/exiting a machine zone:

| Step | Trigger / State | Frame | Move type | Tool | Purpose |
|------|----------------|-------|-----------|------|---------|
| 1 | `move_on_rail_optimization_approach_machine_N` | `OptimizationApproachMachine{N}` | rail-only MoveJ | — | Slide rail to machine's optimal X |
| 2 | `transport` | — (absolute joints) | MoveJ | — | Fold arm to compact transport pose |
| 3 | (machine 2 only) `transport_reversed_right` | — (absolute joints) | MoveJ | — | Flip J1 to -180 for opposite-side machine |
| 4 | `approach_machine_N_curtain_safe_1` | `ApproachMachine{N}CurtainSafe` | **MoveJ** (enter) | GrabbingGripper | High pulled-back pose above machine zone |
| 5 | (work: grab/release sequence — all MoveL) | slot-specific frames | MoveL | GrabbingGripper | Approach, below, base, up, approach |
| 6 | `approach_machine_N_curtain_safe_2` | same frame as step 4 | **MoveL** (exit) | GrabbingGripper | Linear retract out of machine zone |
| 7 | (machine 2 only) `transport_reversed_right` | — | MoveJ | — | Flip J1 back |
| 8 | `transport` | — | MoveJ | — | Return to compact pose |

### CurtainSafe Frame Poses (local to Machine{N}Base)

| Frame | x | y | z | rx | ry | rz |
|-------|---|---|---|-----|-----|-----|
| `ApproachMachine1CurtainSafe` | 700 | -1750 | 2500 | -90 | 0 | -90 |
| `ApproachMachine2CurtainSafe` | 700 | -1750 | 2500 | -90 | 0 | -90 |
| `ApproachCart1CurtainSafe` | 1150 | -1500 | 1400 | -180 | -75 | -180 |
| `ApproachRack1CurtainSafe` | 1600 | 136 | 2200 | 0 | -90 | 0 |

### OptimizationApproach Frames (rail positioning — local to Machine{N}Base)

| Frame | x | y | z | Purpose |
|-------|---|---|---|---------|
| `OptimizationApproachMachine1` | -1350 | 0 | 0 | Rail X for machine 1 |
| `OptimizationApproachMachine1Shifted` | 400 | 0 | 0 | Shifted rail X (garment tray high) |
| `OptimizationApproachMachine2` | -300 | 0 | 0 | Rail X for machine 2 |
| `OptimizationApproachMachine2Shifted` | 1400 | 0 | 0 | Shifted rail X for machine 2 |

## 3. Buffer (Garment Tray Shelf) Movement Sequence

DHR's buffer is a garment tray staging shelf, NOT a cone bin. Our cone bin uses a similar pattern.

### Enter buffer
| Step | Trigger | Frame | Move type | Purpose |
|------|---------|-------|-----------|---------|
| 1 | `approach_buffer_1` | `ApproachBuffer` | MoveJ | Buffer corridor entry waypoint |
| 2 | `approach_buffer_2_1` | `ApproachBuffer2` | MoveJ | Buffer corridor second waypoint (different wrist orientation) |

### Grab from buffer slot (all MoveL, _2 suffix)
| Step | Trigger | Frame | Local offset from SlotBase | Move type | Purpose |
|------|---------|-------|---------------------------|-----------|---------|
| 1 | `approach_buffer_1_garment_tray_1_slot_N_2` | `ApproachBuffer1GarmentTray1SlotN` | x=50, z=-720 | MoveL | Far approach |
| 2 | `approach_..._below_2` | `...SlotNBelow` | x=-15, z=-90 | MoveL | Fine approach below tray |
| 3 | `buffer_1_..._slot_N_base_2` | `...SlotNBase` | (origin) | MoveL | Grip position |
| 4 | `grab_tray` | — | — | I/O | Close gripper |
| 5 | `approach_..._up_2` | `...SlotNUp` | x=35 | MoveL | Lift after grab |
| 6 | `approach_buffer_1_garment_tray_1_slot_N_2` | `ApproachBuffer1GarmentTray1SlotN` | x=50, z=-720 | MoveL | Clear of shelf |

### Release to buffer slot (all MoveL — note different order from grab)
| Step | Trigger | Frame | Move type | Purpose |
|------|---------|-------|-----------|---------|
| 1 | `approach_..._2` | `Approach...SlotN` | MoveL | Far approach |
| 2 | `approach_..._up_2` | `...SlotNUp` | MoveL | Line up from above |
| 3 | `..._base_2` | `...SlotNBase` | MoveL | Lower to place position |
| 4 | `release_tray` | — | I/O | Open gripper |
| 5 | `approach_..._below_2` | `...SlotNBelow` | MoveL | Pull back below |
| 6 | `approach_..._2` | `Approach...SlotN` | MoveL | Clear |

### Exit buffer (reverse of enter)
| Step | Trigger | Frame | Move type | Purpose |
|------|---------|-------|-----------|---------|
| 1 | `approach_buffer_2_1` | `ApproachBuffer2` | MoveJ | |
| 2 | `approach_buffer_1` | `ApproachBuffer` | MoveJ | Fully clear of buffer zone |

### Buffer Frame Poses (local to BufferBase at x=2000, y=0, z=-920 from robot base)

| Frame | x | y | z | rx | ry | rz |
|-------|---|---|---|-----|-----|-----|
| `ApproachBuffer` | 0 | 400 | 870 | 0 | 55 | 180 |
| `ApproachBuffer2` | 0 | 400 | 870 | 180 | -55 | -180 |
| `Buffer1GarmentTray1Slot1Base` | -980.9 | -382.1 | 236.0 | -178.9 | -65.3 | -179.7 |
| `Buffer1GarmentTray1Slot2Base` | -986.2 | 381.4 | 239.2 | -178.9 | -65.3 | -179.7 |

### Slot child frame offsets (same for both slots)

| Child | x | y | z | Purpose |
|-------|---|---|---|---------|
| `Approach...SlotN` | 50 | 0 | -720 | Far approach (720mm away along slot Z) |
| `Approach...SlotNBelow` | -15 | 0 | -90 | Fine approach 90mm below slot |
| `Approach...SlotNUp` | 35 | 0 | 0 | Lift position 35mm above slot |

## 4. Key Design Patterns

- **MoveJ for large transits** (transport, curtain-safe entry, buffer corridor)
- **MoveL for precision work** (all slot-level approach/grab/lift/retract)
- **Same frame, two states:** State1 = MoveJ (enter), State2 = MoveL (exit/retract)
- **Grab order:** Approach -> Below -> Base [grab] -> Up -> Approach
- **Release order:** Approach -> Up -> Base [release] -> Below -> Approach (NOT reverse of grab)
- **No generic path reversal** — forward and return are manually coded as explicit sequences
- **Transport pose always first** before any large rotation or rail movement
