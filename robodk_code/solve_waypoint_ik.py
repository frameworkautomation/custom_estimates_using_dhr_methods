"""
solve_waypoint_ik.py

For every Cartesian waypoint in all_waypoints.yaml that lacks joints:,
solve IK using the BFS-nearest already-solved waypoint as arm-config seed.
Writes joints and ik_collision_verified back to all_waypoints.yaml.

j7 handling:
  - j7: null (or absent)  → j7 is free; SolveIK_All picks optimal rail position
  - j7: <number>          → j7 constrained; OptimAxes pins rail to that value.
                            Raises RuntimeError if no solution found.

Seeding:
  BFS seeds = all waypoints that already have joints in all_waypoints.yaml.
  On first run (nothing solved yet), home joints from path_config.yaml are used
  as fallback arm-config seed for all waypoints.

Collision check:
  After solving, robot is set to candidate joints and rdk.Collisions() is called.
  This is a STATIC check only — path collision testing happens later in
  check_collision_free_paths.py.
  If collisions found, rdk.CollisionItems() reports which items are colliding.

Requires RoboDK running with the station loaded.

Usage:
    python robodk_code/solve_waypoint_ik.py
    python robodk_code/solve_waypoint_ik.py --robodk-ip 172.23.208.1
    python robodk_code/solve_waypoint_ik.py --dry-run   # solve but don't write
    python robodk_code/solve_waypoint_ik.py --force     # re-solve already-solved

Workflow:
    1. Export from GH -> base_cone_waypoints.yaml, machine_cone_waypoints.yaml
    2. python robodk_code/amalgamate_waypoints.py
    3. python robodk_code/solve_waypoint_ik.py         <- this script
    4. python robodk_code/check_collision_free_paths.py
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)

from robodk_code.waypoint_ik_utils import (
    build_pose,
    bfs_solve_order,
    joint_distance,
    load_all_waypoints,
    save_all_waypoints,
    resolve_output_yaml,
    load_home_joints,
)

ROBOT_NAME = "Fanuc R2000iC 125L"
PATH_CONFIG = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")

# OptimAxes config: j7 very strongly pinned, all other joints free.
# Same pattern as check_base_cone_reachability.py.
OPT_AXES_PIN_J7 = {
    "AbsJnt_7": 0, "AbsOn_7": 1, "AbsW_7": 1000,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1..7": 1, "RelW_1..7": 50,
}


def connect(ip: str):
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    rdk = Robolink(ip)
    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found in station.")
    return rdk, robot


def solve_constrained_j7(robot, pose_mat, j7_value: float, seed_joints: list) -> list:
    """Solve IK with j7 pinned to j7_value using OptimAxes.

    Returns [j1..j7] rounded to 4dp.
    Raises RuntimeError if no solution or j7 drifts > 2 degrees from target.
    """
    from robodk.robomath import Mat
    seed = list(seed_joints)
    seed[6] = j7_value

    robot.setParam("OptimAxes", OPT_AXES_PIN_J7)
    robot.setJoints(seed)
    try:
        robot.MoveJ(Mat(pose_mat))
    except Exception as e:
        robot.setParam("OptimAxes", {})
        raise RuntimeError(f"OptimAxes MoveJ failed: {e}")

    robot.setParam("OptimAxes", {})
    result = robot.Joints().list()

    if abs(result[6] - j7_value) > 2.0:
        raise RuntimeError(
            f"j7 drifted to {result[6]:.2f} (target {j7_value:.2f}) — "
            "pose unreachable with rail constrained at this position"
        )
    return [round(float(j), 4) for j in result]


def solve_free_j7(robot, pose_mat, seed_joints: list) -> list:
    """Solve IK with j7 free. Returns candidate joint lists sorted closest-to-seed first.

    Uses SolveIK_All at the seed's current j7 position.
    Returns empty list if no solution found.
    """
    from robodk.robomath import Mat
    robot.setJoints(seed_joints)

    all_sols_mat = robot.SolveIK_All(Mat(pose_mat))

    candidates = []
    try:
        n_rows = all_sols_mat.rows
        n_cols = all_sols_mat.cols
        for col in range(n_cols):
            sol = [round(float(all_sols_mat[row][col]), 4) for row in range(n_rows)]
            if len(sol) == 7:
                candidates.append(sol)
    except Exception:
        flat = all_sols_mat.list()
        if len(flat) == 7:
            candidates = [[round(float(j), 4) for j in flat]]

    candidates.sort(key=lambda j: joint_distance(j, seed_joints))
    return candidates


def check_static_collision(rdk, robot, joints: list) -> tuple:
    """Set robot to joints and check for static collisions.

    Returns (is_clear: bool, colliding_names: list[str]).
    colliding_names is empty when is_clear is True.
    """
    robot.setJoints(joints)
    n = rdk.Collisions()
    if n == 0:
        return True, []
    items = rdk.CollisionItems()
    names = [item.Name() for item in items if item.Valid()]
    return False, names


def main():
    parser = argparse.ArgumentParser(
        description="Solve IK for all Cartesian waypoints in all_waypoints.yaml"
    )
    parser.add_argument("--robodk-ip", default="localhost")
    parser.add_argument("--dry-run", action="store_true",
                        help="Solve but do not write results to YAML")
    parser.add_argument("--force", action="store_true",
                        help="Re-solve waypoints that already have joints")
    args = parser.parse_args()

    yaml_path = resolve_output_yaml(REPO_ROOT)
    print(f"Loading: {yaml_path}")
    waypoints, edges = load_all_waypoints(yaml_path)

    wp_by_name = {w["name"]: w for w in waypoints}

    # Collect already-solved waypoints as BFS seeds
    solved_joints = {}  # name -> [j1..j7]
    for wp in waypoints:
        if isinstance(wp.get("joints"), list) and not args.force:
            solved_joints[wp["name"]] = [float(j) for j in wp["joints"]]

    # Fallback arm-config seed for cold start (no solved waypoints yet)
    home_joints = load_home_joints(PATH_CONFIG)
    print(f"Pre-solved in yaml: {len(solved_joints)}")
    print(f"Fallback seed (home): {home_joints}")

    # BFS order — only over unsolved waypoints
    bfs_order = bfs_solve_order(set(solved_joints.keys()), edges)
    to_solve = [
        (name, parent)
        for name, parent in bfs_order
        if name in wp_by_name and (args.force or not isinstance(wp_by_name[name].get("joints"), list))
    ]

    # If nothing is pre-solved, BFS returns empty — solve all waypoints with home seed
    if not to_solve and not solved_joints:
        to_solve = [(w["name"], None) for w in waypoints
                    if not isinstance(w.get("joints"), list) or args.force]
        print(f"Cold start — solving all {len(to_solve)} waypoints with home seed")
    else:
        print(f"Waypoints to solve: {len(to_solve)} / {len(waypoints)} total")

    if not to_solve:
        print("Nothing to solve — all waypoints already have joints.")
        return

    rdk, robot = connect(args.robodk_ip)
    print(f"Connected: {robot.Name()}")

    # Set world frame for Cartesian IK — matches pattern in moving_a_cone.py
    world_frame = rdk.Item("World")
    if not world_frame.Valid():
        # Try finding it as the first frame
        from robodk.robolink import ITEM_TYPE_FRAME
        world_frame = rdk.Item("", ITEM_TYPE_FRAME)
    original_frame = robot.PoseFrame()
    robot.setPoseFrame(world_frame)

    n_solved = 0
    n_unreachable = 0
    n_error = 0

    try:
        for wp_name, parent_name in to_solve:
            wp = wp_by_name[wp_name]

            # Arm-config seed: BFS parent joints, or home as fallback
            seed_joints = solved_joints.get(parent_name, home_joints) if parent_name else home_joints

            pose_mat = build_pose(wp)
            j7_raw = wp.get("j7")
            has_constrained_j7 = j7_raw is not None and j7_raw != "null"

            if has_constrained_j7:
                j7_val = float(j7_raw)
                try:
                    joints = solve_constrained_j7(robot, pose_mat, j7_val, seed_joints)
                except RuntimeError as e:
                    print(f"  [ERROR] {wp_name}: j7={j7_val} constrained IK failed — {e}")
                    n_error += 1
                    raise  # j7-constrained failure is always fatal

                clear, colliding = check_static_collision(rdk, robot, joints)
                if clear:
                    wp["joints"] = joints
                    wp["ik_collision_verified"] = True
                    solved_joints[wp_name] = joints
                    n_solved += 1
                    print(f"  [OK]         {wp_name}  j7={j7_val:.1f} (constrained)")
                else:
                    raise RuntimeError(
                        f"{wp_name}: j7-constrained solution collides. "
                        f"Colliding items: {colliding}. "
                        "Check tool geometry or waypoint position."
                    )

            else:
                # j7 free
                candidates = solve_free_j7(robot, pose_mat, seed_joints)
                accepted = None
                last_collision_info = []

                for candidate in candidates:
                    clear, colliding = check_static_collision(rdk, robot, candidate)
                    if clear:
                        accepted = candidate
                        break
                    last_collision_info = colliding

                if accepted:
                    wp["joints"] = accepted
                    wp["ik_collision_verified"] = True
                    solved_joints[wp_name] = accepted
                    n_solved += 1
                    print(f"  [OK]         {wp_name}  j7={accepted[6]:.1f} (free, seed={parent_name or 'home'})")
                else:
                    wp["reachable"] = False
                    collision_note = (
                        f"colliding items: {last_collision_info}" if last_collision_info
                        else "no IK solutions found"
                    )
                    wp["note"] = (
                        (wp.get("note", "") + " ").lstrip() +
                        f"# TODO: unreachable — {collision_note}. "
                        "Investigate tool geometry or alternative approach direction."
                    ).strip()
                    n_unreachable += 1
                    print(f"  [UNREACHABLE] {wp_name}: {len(candidates)} candidates, {collision_note}")

    finally:
        robot.setPoseFrame(original_frame)

    already_had = len(waypoints) - len(to_solve)
    print(f"\n--- Summary ---")
    print(f"  Solved:              {n_solved}")
    print(f"  Unreachable:         {n_unreachable}")
    print(f"  Errors (fatal):      {n_error}")
    print(f"  Already had joints:  {already_had}")

    if not args.dry_run:
        save_all_waypoints(yaml_path, waypoints, edges)
        print(f"\n[OK] Written to {yaml_path}")
    else:
        print("\n[DRY RUN] Not writing to YAML.")


if __name__ == "__main__":
    main()
