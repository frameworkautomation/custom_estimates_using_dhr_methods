# DHR Movement Reference Tables

Extracted from DHR's `move_task.py`, `state_machine.py`, and `robodk.yaml`.
Use as a template for our bin/cone programs.

## 1. Transport / Rotation Poses

| Pose | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|------|-----|-----|-----|-----|-----|-----|-----|
| `transport` | 0 | -50 | 15 | 0 | -15 | -90 | unchanged |
| `transport_reversed_right` | -180 | -50 | 15 | 0 | -15 | -90 | unchanged |
| `transport_reversed_left` | +180 | -50 | 15 | 0 | -15 | -90 | unchanged |
| `home` | 0 | -45 | 0 | -180 | 1 | 0 | 0.01 |

## 2. DHR: Enter Machine Zone → Grab Tray → Exit

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  [rail slide to OptimizationApproachM{N}]  —                 j7 only, arm unchanged
 2  MoveJ  transport [0,-50,15,0,-15,-90]             —                 fold arm compact
 3  MoveJ  transport_reversed [-180,-50,15,0,-15,-90] —                 (machine 2 only) flip J1
 4  MoveJ  ApproachMachine{N}CurtainSafe              GrabbingGripper   high safe pose above machine
 5  MoveL  Approach{slot}                             GrabbingGripper   far approach to slot
 6  MoveL  Approach{slot}Below                        GrabbingGripper   fine approach below
 7  MoveL  {slot}Base                                 GrabbingGripper   grip position
 8  I/O    grab_tray                                  —                 close gripper
 9  MoveL  Approach{slot}Up                           GrabbingGripper   lift after grab
10  MoveL  Approach{slot}                             GrabbingGripper   clear of slot
11  MoveL  ApproachMachine{N}CurtainSafe              GrabbingGripper   linear exit from zone
12  MoveJ  transport_reversed                         —                 (machine 2 only)
13  MoveJ  transport                                  —                 fold arm back
```

## 3. DHR: Enter Machine Zone → Release Tray → Exit

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  [rail slide]                               —                 j7 only
 2  MoveJ  transport                                  —                 fold arm
 3  MoveJ  transport_reversed                         —                 (machine 2 only)
 4  MoveJ  ApproachMachine{N}CurtainSafe              GrabbingGripper   enter zone
 5  MoveL  Approach{slot}                             GrabbingGripper   far approach
 6  MoveL  Approach{slot}Up                           GrabbingGripper   line up from above
 7  MoveL  {slot}Base                                 GrabbingGripper   lower to place
 8  I/O    release_tray                               —                 open gripper
 9  MoveL  Approach{slot}Below                        GrabbingGripper   pull back below
10  MoveL  Approach{slot}                             GrabbingGripper   clear
11  MoveL  ApproachMachine{N}CurtainSafe              GrabbingGripper   linear exit
12  MoveJ  transport_reversed                         —                 (machine 2 only)
13  MoveJ  transport                                  —                 fold arm back
```

## 4. DHR: Enter Buffer → Grab Tray → Exit Buffer

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  ApproachBuffer                             GrabbingGripper   buffer corridor waypoint 1
 2  MoveJ  ApproachBuffer2                            GrabbingGripper   buffer corridor waypoint 2
 3  MoveL  ApproachBuffer1GarmentTray1Slot{N}         GrabbingGripper   far approach (z=-720 from base)
 4  MoveL  Approach...Slot{N}Below                    GrabbingGripper   fine approach (z=-90 from base)
 5  MoveL  Buffer1GarmentTray1Slot{N}Base             GrabbingGripper   grip position
 6  I/O    grab_tray                                  —                 close gripper
 7  MoveL  Approach...Slot{N}Up                       GrabbingGripper   lift (x=+35 from base)
 8  MoveL  ApproachBuffer1GarmentTray1Slot{N}         GrabbingGripper   clear of shelf
 9  MoveJ  ApproachBuffer2                            GrabbingGripper   exit corridor
10  MoveJ  ApproachBuffer                             GrabbingGripper   fully clear
```

## 5. DHR: Enter Buffer → Release Tray → Exit Buffer

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  ApproachBuffer                             GrabbingGripper   buffer corridor waypoint 1
 2  MoveJ  ApproachBuffer2                            GrabbingGripper   buffer corridor waypoint 2
 3  MoveL  ApproachBuffer1GarmentTray1Slot{N}         GrabbingGripper   far approach
 4  MoveL  Approach...Slot{N}Up                       GrabbingGripper   line up from above
 5  MoveL  Buffer1GarmentTray1Slot{N}Base             GrabbingGripper   lower to place
 6  I/O    release_tray                               —                 open gripper
 7  MoveL  Approach...Slot{N}Below                    GrabbingGripper   pull back below
 8  MoveL  ApproachBuffer1GarmentTray1Slot{N}         GrabbingGripper   clear
 9  MoveJ  ApproachBuffer2                            GrabbingGripper   exit corridor
10  MoveJ  ApproachBuffer                             GrabbingGripper   fully clear
```

## 6. DHR: Pick Up Gripper from Slot

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  transport                                  ToolChanger       fold arm
 2  MoveJ  ApproachGrabbingGripperSlot                ToolChanger       above slot (z=-300 from slot)
 3  MoveL  GrabbingGripperSlot                        ToolChanger       at slot
 4  I/O    attach gripper                             —                 lock tool changer
 5  MoveL  ApproachGrabbingGripperSlot                ToolChanger       retract
```

## 7. DHR: Return Gripper to Slot

```
 #  Move   Target                                    Tool              Notes
 1  MoveJ  ApproachGrabbingGripperSlot                ToolChanger       above slot
 2  MoveL  GrabbingGripperSlot                        ToolChanger       at slot
 3  I/O    detach gripper                             —                 release tool changer
 4  MoveL  ApproachGrabbingGripperSlot                ToolChanger       retract
 5  MoveJ  transport                                  ToolChanger       fold arm
```

## 8. Our Current Bin Program (back_bin_reachability_demo.py)

```
 #  Move   Target                                    Tool              Notes
--- Phase 1: Pick up gripper ---
 1  MoveJ  home [0,-50,15,0,-15,-90]                  ToolChanger
 2  MoveJ  ApproachGrabbingGripperSlot                ToolChanger
 3  MoveL  GrabbingGripperSlot                        ToolChanger
 4  call   attach_gripper
 5  MoveL  ApproachGrabbingGripperSlot                ToolChanger       retract
--- Phase 2: Grab bin ---
 6  MoveJ  home                                       GrabbingGripper
 7  MoveJ  ApproachConeBinBuffer                      GrabbingGripper   coarse approach
 8  MoveL  ApproachConeBinBufferBelow                  GrabbingGripper   fine approach
 9  MoveL  Cone_Bin_Frame                              GrabbingGripper   at grab
10  call   grab_cone_bin_buffer
--- Phase 3: Retract with bin ---
11  MoveL  ApproachConeBinBufferUp                     GrabbingGripper   lift
12  MoveL  ApproachConeBinBuffer                       GrabbingGripper   clear
13  MoveJ  home                                        GrabbingGripper
14  pause  5s
--- Phase 4: Return bin ---
15  MoveJ  ApproachConeBinBuffer                       GrabbingGripper   coarse approach
16  MoveL  ApproachConeBinBufferBelow                  GrabbingGripper   fine approach
17  MoveL  Cone_Bin_Frame                              GrabbingGripper   at place
18  call   release_cone_bin_buffer
19  MoveL  ApproachConeBinBufferUp                     GrabbingGripper   lift
20  MoveL  ApproachConeBinBuffer                       GrabbingGripper   clear
21  MoveJ  home                                        GrabbingGripper
--- Phase 5: Return gripper ---
22  MoveJ  ApproachGrabbingGripperSlot                 ToolChanger
23  MoveL  GrabbingGripperSlot                         ToolChanger
24  call   detach_gripper
25  MoveL  ApproachGrabbingGripperSlot                 ToolChanger       retract
26  MoveJ  home                                        ToolChanger
```

### Differences from DHR pattern:
- We use the same approach sequence for grab AND release (DHR uses different order — see sections 4 vs 5)
- We go home→ApproachConeBinBuffer directly; DHR goes transport→corridor_wp1→corridor_wp2→far_approach
- We have no "corridor waypoints" equivalent to DHR's ApproachBuffer + ApproachBuffer2

## Frame Poses Reference

### CurtainSafe (local to Machine{N}Base)

| Frame | x | y | z | rx | ry | rz |
|-------|---|---|---|-----|-----|-----|
| `ApproachMachine1CurtainSafe` | 700 | -1750 | 2500 | -90 | 0 | -90 |
| `ApproachMachine2CurtainSafe` | 700 | -1750 | 2500 | -90 | 0 | -90 |
| `ApproachCart1CurtainSafe` | 1150 | -1500 | 1400 | -180 | -75 | -180 |
| `ApproachRack1CurtainSafe` | 1600 | 136 | 2200 | 0 | -90 | 0 |

### Buffer frames (local to BufferBase at x=2000, y=0, z=-920 from robot base)

| Frame | x | y | z | rx | ry | rz |
|-------|---|---|---|-----|-----|-----|
| `ApproachBuffer` | 0 | 400 | 870 | 0 | 55 | 180 |
| `ApproachBuffer2` | 0 | 400 | 870 | 180 | -55 | -180 |
| `Slot1Base` | -980.9 | -382.1 | 236.0 | -178.9 | -65.3 | -179.7 |
| `Slot2Base` | -986.2 | 381.4 | 239.2 | -178.9 | -65.3 | -179.7 |

### Buffer slot child offsets (same for both slots)

| Child | x | y | z | Purpose |
|-------|---|---|---|---------|
| `Approach...SlotN` | 50 | 0 | -720 | Far approach |
| `Approach...SlotNBelow` | -15 | 0 | -90 | Below slot |
| `Approach...SlotNUp` | 35 | 0 | 0 | Lift position |

### OptimizationApproach (local to Machine{N}Base, X-only for rail)

| Frame | x | Purpose |
|-------|---|---------|
| `OptimizationApproachMachine1` | -1350 | Rail X for machine 1 |
| `OptimizationApproachMachine1Shifted` | 400 | Shifted (garment tray high) |
| `OptimizationApproachMachine2` | -300 | Rail X for machine 2 |
| `OptimizationApproachMachine2Shifted` | 1400 | Shifted for machine 2 |
