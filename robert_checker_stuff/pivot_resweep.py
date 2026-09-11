"""
Targeted re-sweep for a single cone + single phase.

Instead of running the full 15-minute pipeline, re-sweep just one component
for one cone and save the results to the station.

Usage:
    # Re-sweep suction for cone_in_bin_12_frame
    python robert_checker_stuff/pivot_resweep.py --robodk-ip 172.23.208.1 \
        --cone cone_in_bin_12_frame --phase suction

    # Re-sweep pivot for cone_in_bin_00_frame with finer step
    python robert_checker_stuff/pivot_resweep.py --robodk-ip 172.23.208.1 \
        --cone cone_in_bin_00_frame --phase pivot --step-deg 5

    # Re-sweep pickup for all cones
    python robert_checker_stuff/pivot_resweep.py --robodk-ip 172.23.208.1 \
        --cone all --phase pickup

    # Re-build program for one cone (uses saved targets from station)
    python robert_checker_stuff/pivot_resweep.py --robodk-ip 172.23.208.1 \
        --cone cone_in_bin_12_frame --phase program

Phases: suction, pivot, pickup, program

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_FOLDER,
)
from robodk.robomath import invH, Pose_2_TxyzRxyz, eye

# Import everything from coupled_pivot_demo
from coupled_pivot_demo import (
    ROBOT_NAMES, SUCTION_TOOL_NAME, PICKUP_TOOL_NAME,
    CHILD_SUFFIXES, TARGET_FOLDER_NAME,
    PROGRAM_FOLDER_NAME, PROGRAM_TARGETS_SUBFOLDER, PROGRAM_PROGRAMS_SUBFOLDER,
    SWEEP_SEEDS, HOME_SEED,
    connect, find_robot,
    discover_bin_cones, assert_cone_frames,
    sweep_suction, sweep_pivot, sweep_pickup,
    print_attempt_report, config_key,
    _save_solution_group, get_or_create_folder,
    load_solutions_from_station, find_config_overlap,
    find_viable_triplet, build_cone_program,
    try_ik_z_sweep,
    save_cone_original_poses, create_attach_detach_scripts,
    load_program_config, get_preferred_theta,
    _find_human_target,
)


def main():
    ap = argparse.ArgumentParser(
        description="Targeted re-sweep for one cone + one phase"
    )
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--cone", required=True,
                    help="Cone name (e.g. cone_in_bin_12_frame) or 'all'")
    ap.add_argument("--phase", required=True,
                    choices=["suction", "pivot", "pickup", "program"],
                    help="Which phase to re-run")
    ap.add_argument("--step-deg", type=float, default=15.0,
                    help="Z-rotation step size in degrees (default: 15)")
    ap.add_argument("--config", default=None,
                    help="Path to pivot_program_config.json for angle-biased selection")
    ap.add_argument("--no-config", action="store_true",
                    help="Ignore --config and use original lowest-theta selection")
    args = ap.parse_args()
    if args.no_config:
        args.config = None

    RDK = connect(args.robodk_ip)
    RDK._setTimeout(300)

    # ── Setup ──
    robot = find_robot(RDK)
    assert robot is not None, f"Robot not found. Tried: {ROBOT_NAMES}"

    suction_tool = RDK.Item(SUCTION_TOOL_NAME, ITEM_TYPE_TOOL)
    assert suction_tool.Valid(), f"Tool '{SUCTION_TOOL_NAME}' not found"

    pickup_tool = RDK.Item(PICKUP_TOOL_NAME, ITEM_TYPE_TOOL)
    assert pickup_tool.Valid(), f"Tool '{PICKUP_TOOL_NAME}' not found"

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    robot.setPoseFrame(world_frame)

    # Discover cones
    cones = discover_bin_cones(RDK)
    cone_cache = {}
    for cone_name, cone_item in cones:
        cone_cache[cone_name] = assert_cone_frames(cone_name, cone_item)

    # Filter to requested cone(s)
    if args.cone == "all":
        target_cones = list(cone_cache.keys())
    else:
        assert args.cone in cone_cache, (
            f"Cone '{args.cone}' not found. Available: {list(cone_cache.keys())}"
        )
        target_cones = [args.cone]

    # Read poses
    cone_poses = {}
    for cone_name in target_cones:
        poses = {}
        for suffix, item in cone_cache[cone_name].items():
            poses[suffix] = item.PoseAbs()
        cone_poses[cone_name] = poses

    # Compute T_pickup_to_suction
    suction_TCP = suction_tool.PoseTool()
    pickup_TCP = pickup_tool.PoseTool()
    T_pickup_to_suction = invH(pickup_TCP) * suction_TCP

    print(f"\nPhase: {args.phase}")
    print(f"Cones: {target_cones}")
    print(f"Step: {args.step_deg} deg\n")

    # ── Run the requested phase ──
    if args.phase == "suction":
        for cone_name in target_cones:
            print(f"=== {cone_name}: suction sweep ===")
            sols, attempts = sweep_suction(robot, RDK, suction_tool,
                                           cone_poses[cone_name], args.step_deg)
            print(f"  {len(sols)} solutions")
            print_attempt_report("suction", attempts, ["offset2", "suction"])

            # Save to station
            root = get_or_create_folder(RDK, TARGET_FOLDER_NAME)
            cone_folder = get_or_create_folder(RDK, cone_name, parent=root)
            # Delete old suction_solutions subfolder
            for child in cone_folder.Childs():
                if child.Name() == "suction_solutions" and child.Type() == ITEM_TYPE_FOLDER:
                    child.Delete()
                    break
            _save_solution_group(RDK, robot, cone_folder, "suction_solutions", sols)
            print(f"  Saved to {TARGET_FOLDER_NAME}/{cone_name}/suction_solutions/")

    elif args.phase == "pivot":
        for cone_name in target_cones:
            print(f"=== {cone_name}: pivot sweep ===")
            sols, attempts = sweep_pivot(robot, RDK, suction_tool,
                                         cone_poses[cone_name],
                                         T_pickup_to_suction, args.step_deg)
            print(f"  {len(sols)} solutions")
            print_attempt_report("pivot", attempts, ["pivot_after"])

            root = get_or_create_folder(RDK, TARGET_FOLDER_NAME)
            cone_folder = get_or_create_folder(RDK, cone_name, parent=root)
            for child in cone_folder.Childs():
                if child.Name() == "pivot_solutions" and child.Type() == ITEM_TYPE_FOLDER:
                    child.Delete()
                    break
            _save_solution_group(RDK, robot, cone_folder, "pivot_solutions", sols)
            print(f"  Saved to {TARGET_FOLDER_NAME}/{cone_name}/pivot_solutions/")

    elif args.phase == "pickup":
        for cone_name in target_cones:
            print(f"=== {cone_name}: pickup sweep ===")
            sols, attempts = sweep_pickup(robot, RDK, pickup_tool,
                                          cone_poses[cone_name], args.step_deg)
            print(f"  {len(sols)} solutions")
            print_attempt_report("pickup", attempts,
                                 ["cone_pickup", "post_pickup_above"])

            root = get_or_create_folder(RDK, TARGET_FOLDER_NAME)
            cone_folder = get_or_create_folder(RDK, cone_name, parent=root)
            for child in cone_folder.Childs():
                if child.Name() == "pickup_solutions" and child.Type() == ITEM_TYPE_FOLDER:
                    child.Delete()
                    break
            _save_solution_group(RDK, robot, cone_folder, "pickup_solutions", sols)
            print(f"  Saved to {TARGET_FOLDER_NAME}/{cone_name}/pickup_solutions/")

    elif args.phase == "program":
        program_config = load_program_config(args.config)
        if program_config:
            print(f"[CONFIG] Loaded program config from {args.config}")

        for cone_name in target_cones:
            print(f"=== {cone_name}: build program ===")

            # Load from station
            loaded = load_solutions_from_station(RDK, cone_name)
            print(f"  Loaded: suction={len(loaded['suction_sols'])} "
                  f"pivot={len(loaded['pivot_sols'])} "
                  f"pickup={len(loaded['pickup_sols'])}")

            s_sols = loaded["suction_sols"]
            p_sols = loaded["pivot_sols"]
            pk_sols = loaded["pickup_sols"]

            if not (s_sols and p_sols and pk_sols):
                print(f"  [SKIP] Missing solutions — run sweeps first")
                continue

            overlap = find_config_overlap(s_sols, p_sols, pk_sols)
            if not overlap:
                print(f"  [SKIP] No config overlap")
                continue

            # Look up per-cone preferred thetas from config
            s_pref = get_preferred_theta(program_config, cone_name, "suction")
            p_pref = get_preferred_theta(program_config, cone_name, "pivot")
            pk_pref = get_preferred_theta(program_config, cone_name, "pickup")

            s_sol, p_sol, pk_sol, cfg = find_viable_triplet(
                robot, RDK, suction_tool, pickup_tool, s_sols, p_sols, pk_sols,
                suction_preferred=s_pref, pivot_preferred=p_pref,
                pickup_preferred=pk_pref,
            )
            if s_sol is None:
                print(f"  [SKIP] No viable triplet found")
                continue

            # Solve offset_1
            robot.setPoseTool(suction_tool)
            o1_joints, _, _ = try_ik_z_sweep(
                robot, RDK, cone_poses[cone_name]["suction_offset_1"],
                label=f"{cone_name}_offset1"
            )
            if o1_joints is None:
                print(f"  [SKIP] suction_offset_1 unreachable")
                continue

            # Setup folders
            root = get_or_create_folder(RDK, PROGRAM_FOLDER_NAME)
            target_folder = get_or_create_folder(RDK, PROGRAM_TARGETS_SUBFOLDER, parent=root)
            program_folder = get_or_create_folder(RDK, PROGRAM_PROGRAMS_SUBFOLDER, parent=root)

            # Attach/detach scripts
            save_cone_original_poses(RDK, {cone_name: cone_cache[cone_name]})
            attach_scripts = create_attach_detach_scripts(
                RDK, cone_cache, [cone_name]
            )

            t_transport = _find_human_target(RDK, "transport")
            assert t_transport is not None, "Target 'transport' not found under WorldFrame/human_made_targets"
            t_reversed_right = _find_human_target(RDK, "Reversed_right")
            assert t_reversed_right is not None, "Target 'Reversed_right' not found under WorldFrame/human_made_targets"

            prog = build_cone_program(
                robot, RDK, cone_name, suction_tool, pickup_tool,
                s_sol, p_sol, pk_sol, o1_joints,
                target_folder, program_folder, attach_scripts,
                t_transport=t_transport, t_reversed_right=t_reversed_right,
            )
            print(f"  [PROG] {prog.Name()}: {prog.InstructionCount()} instructions")
            print(f"  config={cfg} s={s_sol['theta_deg']:.0f} "
                  f"p={p_sol['theta_deg']:.0f} pk={pk_sol['theta_deg']:.0f}")

    print("\nDone.")


if __name__ == "__main__":
    main()
