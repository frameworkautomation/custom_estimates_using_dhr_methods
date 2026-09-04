# TODO List

## Branch: `placing_cones_no_rhino`

1. **Get simplified cone mesh** — get simplified cone mesh from another folder for use in RoboDK scripts
2. **Cone placement scripts (replacing Grasshopper)** — RoboDK scripts to place cones inside bins and on top of machines, with proper cone names and frames

## Branch: `back_box_reachability`

3. **Back box reachability check** — check if DHR's box picker-upper mechanism can reach the back box position on the machine, with robot at j7=0
4. **End effector rear reachability** — verify end effector can reach items in the rear of the robot (knotter, cone picker-upper)
   - 4a. IK optimizer constraints (no side-flipping) — when searching for poses, optimizer must ensure robot doesn't flip sides while using end effector
   - 4b. J1 continuity constraint (no 360-degree wrap) — ensure robot doesn't rotate ~360 degrees about J1 between yarn pickup and cone pickup. May not be needed — Robert provided better knotter movement specifications
5. **Run DHR's code against back position** — run DHR's existing code with the back bin positioned in the rear, visually check for collisions *(not a coding task — constraint that affects the work)*
6. **Visual collision check** — verify mechanism doesn't go through back wall or hit robo fence *(human task)*

## Branch: `auto_collision_check`

7. **Automatic collision check for rear boxes/bin** — scripted collision detection for rear box and bin positions

## Branch: `machine_reachability`

8. **End effector access on 8-gauge machine** — check reachability for knotter, cone picking, and cutting on 8-gauge machine

## Branch: `dhr_framework_cone_movement`

9. **Add cone movement to DHR's code** — integrate our end effector into their state machine (end effector up and down), use their pipeline for collision checking, understand their IO setup
   - All AI-generated code must be clearly marked as AI-written (with or without human review). This code runs on a physical robot — everything gets human review.

## Low Priority

- **Fix update_clones.sh timeout** — replace `git remote show origin` with local branch lookup to avoid ~8 min network timeout
