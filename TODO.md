# TODO List

## Branch: `placing_cones_no_rhino`

1. ~~**Get simplified cone mesh** — get simplified cone mesh from another folder for use in RoboDK scripts~~ DONE
2. ~~**Cone placement scripts (replacing Grasshopper)** — RoboDK scripts to place cones inside bins and on top of machines, with proper cone names and frames~~ DONE

## Branch: `back_bin_reachability`

3. ~~**Back bin reachability demo** — demonstrate that the robot (at j7=0, no rail) can reach the back bin position, pick up cone_bin_buffer, and return home~~ DONE
4. **End effector rear reachability** — verify end effector can reach items in the rear of the robot (knotter, cone picker-upper)
   - 4a. IK optimizer constraints (no side-flipping) — when searching for poses, optimizer must ensure robot doesn't flip sides while using end effector
   - 4b. J1 continuity constraint (no 360-degree wrap) — ensure robot doesn't rotate ~360 degrees about J1 between yarn pickup and cone pickup. May not be needed — Robert provided better knotter movement specifications
   - 4c. **Coupled feasibility search for vacuum-to-pickup pivot** — the movement sequence is: vacuum base cone → vacuum offset → vacuum rotate into pickup offset → pickup. The transition to pickup requires pivoting about the vacuum point, which is a series of MoveL instructions with an end effector change. These moves must be searched for feasibility **together** (not individually) — find a Z-rotation where the entire series of L-moves is feasible, not just each pose in isolation.
5. **Run DHR's movement sequence with back bin present** — step through DHR's existing state machine code with the back bin in position, visually check for collisions during the specific programmed path *(not a coding task — run and observe)*
6. **Envelope boundary check** — verify the robot's reachable workspace (at any j7 position, any arm config) does not penetrate the back wall or robo fence. Separate from task 5: this checks the static geometry envelope, not just one movement sequence *(human task — visual inspection in RoboDK)*

## Branch: `auto_collision_check`

7. **Automatic collision check for rear boxes/bin** — scripted collision detection for rear box and bin positions

## Branch: `machine_reachability`

8. ~~**End effector access on 8-gauge machine** — check reachability for knotter, cone picking, and cutting on 8-gauge machine~~ DONE

## Branch: `dhr_framework_cone_movement`

9. **Add cone movement to DHR's code** — integrate our end effector into their state machine (end effector up and down), use their pipeline for collision checking, understand their IO setup
   - All AI-generated code must be clearly marked as AI-written (with or without human review). This code runs on a physical robot — everything gets human review.
   - Cone positions and frames from `place_cones.py` need to be reflected in DHR's station and YAML config longer term
   - Mounting plate, bin, cone positions in bin, and cone array positions need to be added to DHR's `robodk.yaml` build config
   - **All cone frames and child frames (grip, Cut, suck, etc.) must have globally unique names** — currently duplicated per cone per machine. `RDK.Item(name)` grabs the first match globally. Need a renaming script to prefix with machine + cone name (e.g. `m3_cone_front_closest_grip`). Applies to both the source station and extracted stations.

## Branch: `plc_end_effector`

10. **Understand PLC interaction with end effector** — how the PLC communicates with and controls the end effector, and how to program it
    - Pogo pin / tool changer IO runs at **24V DC** (confirmed with Atenas)

## Immediate TODOs (branch: `back_bin_reachability`)

- **Fix suction_offset_1 for cone_02 and cone_12** — these two cones' suction_offset_1 frames are unreachable at any Z-rotation. Need to reposition them in RoboDK to a reachable spot near the bin.
- **Verify new bin position can be gripped** — check that the robot can reach and grip the bin at its new position (Robert's preferred config where robot doesn't enter cells).

## Known Issues

- **RoboDK program MoveL vs Python API MoveL** — RoboDK program MoveL instructions (target items) pre-compute the entire linear path with a fixed joint configuration and fail if any point along the path is unreachable in that config. Python API `robot.MoveL(pose)` solves IK step-by-step from current joints with `OptimAxes` active, so j7 can flex along the path. Use Python script programs (DHR's approach) instead of RoboDK program instructions for movement sequences involving MoveL with external axes.

## Low Priority

- **Fix update_clones.sh timeout** — replace `git remote show origin` with local branch lookup to avoid ~8 min network timeout
