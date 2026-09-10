"""
Coupled pivot solver demo — task 4c (simplified sequence).

Three independent sweeps (suction, pivot, pickup) find reachable poses at each
theta × seed. Solutions with matching wrist configs can be combined into a full
sequence. All solutions saved as RoboDK joint targets for visual inspection.

Forward sequence:
  F1: JMove home → suction_offset_1 (Z-free, independent)
  F2: JMove suction_offset_1 → suction_offset_2
  F3: LMove suction_offset_2 → suction_position (grab string)
  F4: LMove suction_position → suction_offset_2 (retract with string)
  F5: LMove suction_offset_2 → pivot_after (pivot pose, knotting tool)
  F6: tool switch (same joints, knotting → pickup)
  F7: LMove before_pickup_offset → cone_pickup_pose (grab cone)
  F8: LMove cone_pickup_pose → post_pickup_above (lift out)
  F9: JMove post_pickup_above → home

See coupled_pivot_spec.md for the full specification.

Usage:
    python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --step-deg 10

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import math
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON,
    ITEM_TYPE_FOLDER, INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import transl, rotz, invH, Mat, Pose_2_TxyzRxyz, eye

# ── CONFIG ──────────────────────────────────────────────────────────────────

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
SUCTION_TOOL_NAME = "knotting"
PICKUP_TOOL_NAME = "pickup"
BIN_PARENT_NAME = "Cone_Bin_Frame"
BIN_CONE_SUBFRAME = "bottom_corner"

EXPECTED_CONE_COUNT = 6

# The 6 child frame suffixes each cone must have
CHILD_SUFFIXES = [
    "suction_offset_1",
    "suction_offset_2",
    "suction_position",
    "before_pickup_offset",
    "cone_pickup_pose",
    "post_pickup_above",
]

# IK settings — EXACT match to proven robert_end_checker.py config
# Uses 7-DOF config with j7 locked at 0 (robot still has 7 joints after extraction)
_OPT_AXES_LOCKED = {
    "AbsOn_7": 1, "AbsW_7": 100,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}
HOME_SEED = [0.0] * 7  # 7-DOF seed matching the proven checker

SWEEP_SEEDS = {
    "seeded_at_p180": [180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "seeded_at_n180": [-180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}

TRANSPORT_JOINTS = [0, -50, 15, 0, -15, -90, 0]  # 7-DOF (j7=0)

# FK verification tolerance
FK_TOL_MM = 5.0

TARGET_FOLDER_NAME = "discovered_targets"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ── CONNECT ─────────────────────────────────────────────────────────────────

def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


# ── IK HELPERS (copied from setup_base_movements.py) ───────────────────────

_last_ik_error = None


def _solve_ik_locked_j7(robot, RDK, pose, j7_target=0.0, seed=None):
    """Solve IK with j7 locked — EXACT copy of proven robert_end_checker pattern."""
    global _last_ik_error
    if seed is None:
        seed = HOME_SEED
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    robot.setParam("OptimAxes", props)

    robot.setJoints(seed)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(seed)
        if len(joints) < 6:
            _last_ik_error = f"got {len(joints)} joints"
            return None, False
        return joints, True
    except Exception as e:
        _last_ik_error = str(e)
        robot.setJoints(seed)
        return None, False


def try_ik(robot, RDK, pose, label="", seed=None):
    """Single IK attempt using the proven locked-j7 pattern."""
    joints, ok = _solve_ik_locked_j7(robot, RDK, pose, seed=seed)
    if ok:
        return joints
    if label:
        xyz = Pose_2_TxyzRxyz(pose)[:3]
        print(f"      [try_ik] {label} FAILED target=[{xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f}] err={_last_ik_error}")
    return None


def try_ik_z_sweep(robot, RDK, pose, N=72, label=""):
    """Z-rotation sweep using the proven pattern. Returns (joints, rotated_pose, angle_deg) or (None, None, None)."""
    for i in range(N):
        angle_deg = 360.0 * i / N
        angle_rad = angle_deg * math.pi / 180.0
        rotated_pose = pose * rotz(angle_rad)
        joints, ok = _solve_ik_locked_j7(robot, RDK, rotated_pose)
        if not ok:
            continue
        # FK verify
        robot.MoveJ(joints)
        achieved = robot.Pose()
        t = Pose_2_TxyzRxyz(rotated_pose)
        a = Pose_2_TxyzRxyz(achieved)
        fk_err = math.sqrt(sum((t[k] - a[k]) ** 2 for k in range(3)))
        robot.setJoints(HOME_SEED)
        if fk_err > 50.0:
            continue
        if label:
            print(f"      [z_sweep] {label} OK at {angle_deg:.0f} deg (fk={fk_err:.1f}mm)")
        return joints, rotated_pose, angle_deg

    if label:
        xyz = Pose_2_TxyzRxyz(pose)[:3]
        print(f"      [z_sweep] {label} FAILED all {N} angles target=[{xyz[0]:.0f},{xyz[1]:.0f},{xyz[2]:.0f}] err={_last_ik_error}")
    return None, None, None


def fk_verify(robot, joints, expected_pose, tol_mm=FK_TOL_MM):
    """Set joints, read achieved TCP pose, compare position to expected. Returns (err_mm, ok)."""
    robot.setJoints(joints)
    achieved = robot.Pose()
    t = Pose_2_TxyzRxyz(expected_pose)
    a = Pose_2_TxyzRxyz(achieved)
    err = math.sqrt(sum((t[k] - a[k]) ** 2 for k in range(3)))
    return err, err <= tol_mm


def get_config_flags(robot, joints):
    """Return JointsConfig as a list [REAR, LOWERARM, FLIP]."""
    cfg = robot.JointsConfig(joints)
    try:
        return cfg.list()[:3]
    except AttributeError:
        return list(cfg)[:3]


def config_key(cfg):
    """Convert config flags [R, L, F] to a string key like 'R0_L0_F0'."""
    return f"R{int(cfg[0])}_L{int(cfg[1])}_F{int(cfg[2])}"


# ── CONE DISCOVERY ──────────────────────────────────────────────────────────

def discover_bin_cones(RDK):
    """Find all cone_ frames under Cone_Bin_Frame/bottom_corner.

    Returns a sorted list of (cone_name, cone_item) tuples.
    """
    bin_frame = RDK.Item(BIN_PARENT_NAME, ITEM_TYPE_FRAME)
    assert bin_frame.Valid(), f"Frame '{BIN_PARENT_NAME}' not found in station"

    # Navigate to bottom_corner subframe
    bottom_corner = None
    for child in bin_frame.Childs():
        if child.Name() == BIN_CONE_SUBFRAME and child.Type() == ITEM_TYPE_FRAME:
            bottom_corner = child
            break
    assert bottom_corner is not None, (
        f"Frame '{BIN_CONE_SUBFRAME}' not found under '{BIN_PARENT_NAME}'"
    )

    cones = []
    for child in bottom_corner.Childs():
        if child.Type() == ITEM_TYPE_FRAME and child.Name().startswith("cone_"):
            cones.append((child.Name(), child))

    return sorted(cones, key=lambda x: x[0])


def _collect_all_frames(parent):
    """Recursively collect all (name, item) pairs under parent. Single API traversal."""
    result = {}
    try:
        for child in parent.Childs():
            try:
                if child.Type() == ITEM_TYPE_FRAME:
                    result[child.Name()] = child
                result.update(_collect_all_frames(child))
            except Exception:
                continue
    except Exception:
        pass
    return result


def assert_cone_frames(cone_name, cone_item):
    """Assert all 6 child frames exist for a cone. Returns dict of suffix -> item.

    Collects all frames under the cone in one traversal, then looks up by name.
    """
    all_frames = _collect_all_frames(cone_item)

    frames = {}
    for suffix in CHILD_SUFFIXES:
        # Try prefixed name first, then bare suffix
        child = all_frames.get(f"{cone_name}_{suffix}") or all_frames.get(suffix)
        assert child is not None, (
            f"Missing child frame '{suffix}' (or '{cone_name}_{suffix}') "
            f"under cone '{cone_name}'. Found: {list(all_frames.keys())}"
        )
        frames[suffix] = child
    return frames


# ── FOLDER HELPERS ──────────────────────────────────────────────────────────

def get_or_create_folder(RDK, name, parent=None):
    if parent is not None:
        for child in parent.Childs():
            if child.Name() == name and child.Type() == ITEM_TYPE_FOLDER:
                return child
        existing_ids = {f.item for f in RDK.ItemList(ITEM_TYPE_FOLDER)}
        RDK.Command("AddFolder", name)
        folder = None
        for f in RDK.ItemList(ITEM_TYPE_FOLDER):
            if f.item not in existing_ids and f.Name() == name:
                folder = f
                break
        assert folder is not None, f"Failed to create folder '{name}'"
        folder.setParent(parent)
        return folder

    existing = RDK.Item(name, ITEM_TYPE_FOLDER)
    if existing.Valid():
        return existing
    RDK.Command("AddFolder", name)
    folder = RDK.Item(name, ITEM_TYPE_FOLDER)
    assert folder.Valid(), f"Failed to create folder '{name}'"
    return folder


def delete_if_exists(RDK, name, item_type):
    item = RDK.Item(name, item_type)
    if item.Valid():
        item.Delete()
        return True
    return False


# ── SWEEP FUNCTIONS (decoupled) ─────────────────────────────────────────────

def sweep_suction(robot, RDK, suction_tool, poses, step_deg):
    """Independent sweep over suction chain: suction_offset_2 and suction_position.

    suction_offset_1 is a JMove (solved independently, not part of this sweep).
    The LMove chain is: suction_offset_2 → suction_position → suction_offset_2 (retract).
    We need both offset_2 and suction_position reachable at the same theta+seed.

    Returns (solutions, attempts) where attempts tracks per-pose pass/fail.
    """
    offset2_pose = poses["suction_offset_2"]
    suction_pose = poses["suction_position"]

    robot.setPoseTool(suction_tool)
    n_steps = int(360 / step_deg)
    solutions = []
    attempts = []

    for i in range(n_steps):
        theta_deg = step_deg * i
        theta_rad = theta_deg * math.pi / 180.0
        rz = rotz(theta_rad)

        rotated_offset2 = offset2_pose * rz
        rotated_suction = suction_pose * rz

        for seed_name, seed in SWEEP_SEEDS.items():
            rec = {"theta_deg": theta_deg, "seed_name": seed_name,
                   "offset2": False, "suction": False}

            j_offset2 = try_ik(robot, RDK, rotated_offset2, seed=seed)
            if j_offset2 is None:
                attempts.append(rec)
                continue
            rec["offset2"] = True

            j_suction = try_ik(robot, RDK, rotated_suction, seed=seed)
            if j_suction is None:
                attempts.append(rec)
                continue
            rec["suction"] = True

            cfg = get_config_flags(robot, j_offset2)
            sol = {
                "theta_deg": theta_deg,
                "seed_name": seed_name,
                "joints": {
                    "suction_offset_2": j_offset2,
                    "suction_position": j_suction,
                },
                "wrist_cfg": cfg,
            }
            solutions.append(sol)
            rec["ok"] = True
            attempts.append(rec)

    return solutions, attempts


def sweep_pivot(robot, RDK, suction_tool, poses, T_pickup_to_suction, step_deg):
    """Independent sweep over pivot_after pose (suction tool).

    pivot_after = rotated_before_pickup * T_pickup_to_suction
    This is where the suction TCP sits when the pickup TCP is on before_pickup_offset.
    The robot LMoves here from suction_offset_2 after retracting from the grab.

    Returns (solutions, attempts) where attempts tracks per-pose pass/fail.
    """
    before_pickup_pose = poses["before_pickup_offset"]

    robot.setPoseTool(suction_tool)
    n_steps = int(360 / step_deg)
    solutions = []
    attempts = []

    for i in range(n_steps):
        theta_deg = step_deg * i
        theta_rad = theta_deg * math.pi / 180.0
        rz = rotz(theta_rad)

        rotated_before_pickup = before_pickup_pose * rz
        pivot_after = rotated_before_pickup * T_pickup_to_suction

        for seed_name, seed in SWEEP_SEEDS.items():
            rec = {"theta_deg": theta_deg, "seed_name": seed_name,
                   "pivot_after": False}

            j_after = try_ik(robot, RDK, pivot_after, seed=seed)
            if j_after is None:
                attempts.append(rec)
                continue
            rec["pivot_after"] = True

            cfg = get_config_flags(robot, j_after)
            sol = {
                "theta_deg": theta_deg,
                "seed_name": seed_name,
                "joints": {
                    "pivot_after": j_after,
                },
                "wrist_cfg": cfg,
            }
            solutions.append(sol)
            rec["ok"] = True
            attempts.append(rec)

    return solutions, attempts


def sweep_pickup(robot, RDK, pickup_tool, poses, step_deg):
    """Independent sweep over pickup chain: cone_pickup_pose and post_pickup_above.

    Uses pickup tool. For each theta × seed, solve IK for both pickup poses.
    Returns (solutions, attempts) where attempts tracks per-pose pass/fail.
    """
    pickup_pose = poses["cone_pickup_pose"]
    post_above_pose = poses["post_pickup_above"]

    robot.setPoseTool(pickup_tool)
    n_steps = int(360 / step_deg)
    solutions = []
    attempts = []

    for i in range(n_steps):
        theta_deg = step_deg * i
        theta_rad = theta_deg * math.pi / 180.0
        rz = rotz(theta_rad)

        rotated_pickup = pickup_pose * rz
        rotated_post = post_above_pose * rz

        for seed_name, seed in SWEEP_SEEDS.items():
            rec = {"theta_deg": theta_deg, "seed_name": seed_name,
                   "cone_pickup": False, "post_pickup_above": False}

            j_pickup = try_ik(robot, RDK, rotated_pickup, seed=seed)
            if j_pickup is None:
                attempts.append(rec)
                continue
            rec["cone_pickup"] = True

            j_post = try_ik(robot, RDK, rotated_post, seed=seed)
            if j_post is None:
                attempts.append(rec)
                continue
            rec["post_pickup_above"] = True

            cfg = get_config_flags(robot, j_pickup)
            sol = {
                "theta_deg": theta_deg,
                "seed_name": seed_name,
                "joints": {
                    "cone_pickup_pose": j_pickup,
                    "post_pickup_above": j_post,
                },
                "wrist_cfg": cfg,
            }
            solutions.append(sol)
            rec["ok"] = True
            attempts.append(rec)

    return solutions, attempts


def print_attempt_report(label, attempts, pose_keys):
    """Print a pass/fail grid for each theta × seed, broken down by pose."""
    # Aggregate: how many times did each pose individually pass?
    total = len(attempts)
    if total == 0:
        print(f"    [{label}] No attempts")
        return

    pass_counts = {k: sum(1 for a in attempts if a.get(k)) for k in pose_keys}
    full_pass = sum(1 for a in attempts if a.get("ok"))

    print(f"    [{label}] {total} attempts, {full_pass} full passes")
    for k in pose_keys:
        print(f"      {k}: {pass_counts[k]}/{total} passed")

    # Show the failure grid (only failed attempts)
    failed = [a for a in attempts if not a.get("ok")]
    if not failed:
        return
    # Group failures by which pose was the first to fail
    first_fail = {}
    for a in failed:
        for k in pose_keys:
            if not a.get(k):
                first_fail.setdefault(k, []).append(a)
                break
    for k in pose_keys:
        if k in first_fail:
            thetas = sorted(set(a["theta_deg"] for a in first_fail[k]))
            print(f"      first failure at {k} ({len(first_fail[k])}x): "
                  f"thetas={[f'{t:.0f}' for t in thetas[:8]]}{'...' if len(thetas) > 8 else ''}")


# ── SAVE SOLUTIONS TO STATION ───────────────────────────────────────────────

def _save_solution_group(RDK, robot, parent_folder, group_name, solutions):
    """Save a list of solution dicts into a named subfolder with seed/config hierarchy."""
    if not solutions:
        return
    group_folder = get_or_create_folder(RDK, group_name, parent=parent_folder)
    for sol in solutions:
        seed_folder = get_or_create_folder(RDK, sol["seed_name"], parent=group_folder)
        cfg_name = f"config_{config_key(sol['wrist_cfg'])}"
        cfg_folder = get_or_create_folder(RDK, cfg_name, parent=seed_folder)

        theta_str = f"{sol['theta_deg']:03.0f}"
        for pose_name, joints in sol["joints"].items():
            tgt_name = f"{pose_name}_theta_{theta_str}"
            tgt = RDK.AddTarget(tgt_name, cfg_folder, robot)
            tgt.setJoints(joints)
            tgt.setAsJointTarget()


def save_solutions_to_station(RDK, robot, cone_name, suction_solutions, pivot_solutions, pickup_solutions):
    """Create RoboDK folder hierarchy with joint targets for all viable solutions.

    Structure:
        discovered_targets/<cone_name>/<group>/<seed>/config_<cfg>/<target>
    """
    root_folder = get_or_create_folder(RDK, TARGET_FOLDER_NAME)
    cone_folder = get_or_create_folder(RDK, cone_name, parent=root_folder)

    _save_solution_group(RDK, robot, cone_folder, "suction_solutions", suction_solutions)
    _save_solution_group(RDK, robot, cone_folder, "pivot_solutions", pivot_solutions)
    _save_solution_group(RDK, robot, cone_folder, "pickup_solutions", pickup_solutions)


# ── LOAD SOLUTIONS FROM STATION ──────────────────────────────────────────────

def _parse_config_folder_name(name):
    """Parse 'config_R0_L0_F0' → [0.0, 0.0, 0.0] or None."""
    if not name.startswith("config_R"):
        return None
    parts = name[len("config_"):].split("_")
    if len(parts) != 3:
        return None
    try:
        return [float(p[1]) for p in parts]  # R0 -> 0.0, L1 -> 1.0, F0 -> 0.0
    except (IndexError, ValueError):
        return None


def _parse_target_name(name):
    """Parse 'suction_offset_2_theta_060' → ('suction_offset_2', 60.0) or None."""
    idx = name.rfind("_theta_")
    if idx < 0:
        return None, None
    pose_name = name[:idx]
    try:
        theta = float(name[idx + len("_theta_"):])
    except ValueError:
        return None, None
    return pose_name, theta


def load_solutions_from_station(RDK, cone_name):
    """Read back solutions from discovered_targets/<cone_name>/ in the station.

    Returns dict: {"suction_sols": [...], "pivot_sols": [...], "pickup_sols": [...]}
    Each sol has: theta_deg, seed_name, joints (dict), wrist_cfg (list).
    """
    root = RDK.Item(TARGET_FOLDER_NAME, ITEM_TYPE_FOLDER)
    if not root.Valid():
        return {"suction_sols": [], "pivot_sols": [], "pickup_sols": []}

    cone_folder = None
    for child in root.Childs():
        if child.Name() == cone_name and child.Type() == ITEM_TYPE_FOLDER:
            cone_folder = child
            break
    if cone_folder is None:
        return {"suction_sols": [], "pivot_sols": [], "pickup_sols": []}

    group_map = {
        "suction_solutions": "suction_sols",
        "pivot_solutions": "pivot_sols",
        "pickup_solutions": "pickup_sols",
    }

    result = {"suction_sols": [], "pivot_sols": [], "pickup_sols": []}

    for group_folder in cone_folder.Childs():
        if group_folder.Type() != ITEM_TYPE_FOLDER:
            continue
        result_key = group_map.get(group_folder.Name())
        if result_key is None:
            continue

        # Walk: group_folder / seed_folder / config_folder / targets
        for seed_folder in group_folder.Childs():
            if seed_folder.Type() != ITEM_TYPE_FOLDER:
                continue
            seed_name = seed_folder.Name()

            for cfg_folder in seed_folder.Childs():
                if cfg_folder.Type() != ITEM_TYPE_FOLDER:
                    continue
                cfg = _parse_config_folder_name(cfg_folder.Name())
                if cfg is None:
                    continue

                # Group targets by theta
                by_theta = {}
                for tgt in cfg_folder.Childs():
                    if tgt.Type() != ITEM_TYPE_TARGET:
                        continue
                    pose_name, theta = _parse_target_name(tgt.Name())
                    if pose_name is None:
                        continue
                    by_theta.setdefault(theta, {})[pose_name] = tgt

                for theta, targets in sorted(by_theta.items()):
                    joints_dict = {}
                    for pose_name, tgt in targets.items():
                        raw = tgt.Joints()
                        try:
                            joints_dict[pose_name] = raw.list()
                        except AttributeError:
                            joints_dict[pose_name] = list(raw)

                    result[result_key].append({
                        "theta_deg": theta,
                        "seed_name": seed_name,
                        "joints": joints_dict,
                        "wrist_cfg": cfg,
                    })

    return result


# ── MATCH PAIRS BY WRIST CONFIG ─────────────────────────────────────────────

def find_config_overlap(suction_solutions, pivot_solutions, pickup_solutions):
    """Report which wrist configs have solutions across all three sweeps.

    Actual pairing (which thetas to combine) is deferred to LMove verification.
    For now we just confirm overlap exists.

    Returns dict of config_key -> {"suction": N, "pivot": N, "pickup": N}.
    """
    groups = {
        "suction": suction_solutions,
        "pivot": pivot_solutions,
        "pickup": pickup_solutions,
    }
    by_cfg = {}  # group_name -> {cfg_key -> count}
    for gname, sols in groups.items():
        counts = {}
        for s in sols:
            k = config_key(s["wrist_cfg"])
            counts[k] = counts.get(k, 0) + 1
        by_cfg[gname] = counts

    all_cfgs = set()
    for counts in by_cfg.values():
        all_cfgs.update(counts.keys())

    # Overlap = configs present in ALL three groups
    overlap = {}
    for cfg in sorted(all_cfgs):
        present = {g: by_cfg[g].get(cfg, 0) for g in groups}
        if all(v > 0 for v in present.values()):
            overlap[cfg] = present
            print(f"    [OVERLAP] config={cfg}: "
                  f"{present['suction']} suction, {present['pivot']} pivot, {present['pickup']} pickup")

    if not overlap:
        # Show partial overlap to help debug
        for cfg in sorted(all_cfgs):
            present = {g: by_cfg[g].get(cfg, 0) for g in groups}
            missing = [g for g, v in present.items() if v == 0]
            has = [f"{g}={v}" for g, v in present.items() if v > 0]
            print(f"    [PARTIAL] config={cfg}: {', '.join(has)} — missing: {', '.join(missing)}")

    return overlap


# ── PROGRAM BUILDING ────────────────────────────────────────────────────────

PROGRAM_FOLDER_NAME = "pivot_programs"


def pick_best_solution(solutions, target_cfg, required_theta=None):
    """Pick the first solution matching config (and optionally theta). Prefer lowest theta."""
    for sol in sorted(solutions, key=lambda s: s["theta_deg"]):
        if config_key(sol["wrist_cfg"]) != target_cfg:
            continue
        if required_theta is not None and sol["theta_deg"] != required_theta:
            continue
        return sol
    return None


def test_lmove(robot, RDK, from_joints, to_joints, tool):
    """Test if an LMove from from_joints to to_joints succeeds.

    Sets the robot to from_joints, then attempts MoveL to the pose
    corresponding to to_joints. Returns (True, "") or (False, error_str).
    Also reports J5 values at endpoints to help diagnose singularity issues.
    """
    robot.setPoseTool(tool)
    from_j5 = from_joints[4]
    to_j5 = to_joints[4]
    j5_info = f"J5: {from_j5:.1f}→{to_j5:.1f}"

    # Check if J5 crosses zero between endpoints (sign change)
    j5_crosses_zero = (from_j5 * to_j5 < 0)

    robot.setJoints(from_joints)
    robot.setJoints(to_joints)
    target_pose = robot.Pose()
    robot.setJoints(from_joints)
    try:
        robot.MoveL(target_pose)
        robot.setJoints(HOME_SEED)
        return True, ""
    except Exception as e:
        robot.setJoints(HOME_SEED)
        err = str(e) if str(e) else "MoveL rejected"
        if j5_crosses_zero:
            err += f" (J5 crosses zero: {j5_info})"
        else:
            err += f" ({j5_info})"
        return False, err


# J5 singularity threshold — reject solutions where J5 is within this many degrees of 0
J5_SINGULARITY_THRESHOLD_DEG = 5.0


def check_j5_singularity(joints, threshold=J5_SINGULARITY_THRESHOLD_DEG):
    """Check if J5 is near zero (wrist singularity). Returns True if safe."""
    j5 = joints[4]  # 0-indexed: j1=0, j2=1, ..., j5=4
    return abs(j5) > threshold


def verify_all_lmoves(robot, RDK, suction_tool, pickup_tool,
                       suction_sol, pivot_sol, pickup_sol):
    """Verify every LMove pair in the full sequence. Returns (ok, fail_step).

    LMove pairs tested:
      F3: offset_2 → suction (knotting)
      F4: suction → offset_2 (knotting)
      F5: offset_2 → pivot_after (knotting)
      F7: pivot_after → cone_pickup (pickup tool — after tool switch)
      F8: cone_pickup → post_pickup (pickup)

    Also checks J5 singularity for all poses involved in LMoves.
    """
    s = suction_sol["joints"]
    p = pivot_sol["joints"]
    pk = pickup_sol["joints"]

    # Check J5 singularity on all LMove-involved joints
    lmove_joints = [
        ("suction_offset_2", s["suction_offset_2"]),
        ("suction_position", s["suction_position"]),
        ("pivot_after", p["pivot_after"]),
        ("cone_pickup_pose", pk["cone_pickup_pose"]),
        ("post_pickup_above", pk["post_pickup_above"]),
    ]
    for name, joints in lmove_joints:
        if not check_j5_singularity(joints):
            return False, f"J5 singularity at {name} (J5={joints[4]:.1f}°)"

    # Test each LMove pair
    steps = [
        ("F3 offset2→suction", s["suction_offset_2"], s["suction_position"], suction_tool),
        ("F4 suction→offset2", s["suction_position"], s["suction_offset_2"], suction_tool),
        ("F5 offset2→pivot", s["suction_offset_2"], p["pivot_after"], suction_tool),
        ("F7 pivot→pickup", p["pivot_after"], pk["cone_pickup_pose"], pickup_tool),
        ("F8 pickup→post", pk["cone_pickup_pose"], pk["post_pickup_above"], pickup_tool),
    ]
    for label, from_j, to_j, tool in steps:
        ok, err = test_lmove(robot, RDK, from_j, to_j, tool)
        if not ok:
            return False, f"{label}: {err}"

    return True, ""


def find_viable_triplet(robot, RDK, suction_tool, pickup_tool,
                        suction_sols, pivot_sols, pickup_sols):
    """Find a (suction, pivot, pickup) triplet where ALL LMoves pass.

    Tests in stages to avoid redundant work:
      1. Filter suction solutions: F3 (offset2→suction) + F4 (suction→offset2)
      2. For each passing suction, try pivot solutions: F5 (offset2→pivot)
      3. For each passing suction+pivot, try pickup solutions: F7 (pivot→pickup) + F8 (pickup→post)

    Returns (suction_sol, pivot_sol, pickup_sol, cfg_key) or (None, None, None, None).
    """
    suction_by_cfg = {}
    for s in suction_sols:
        k = config_key(s["wrist_cfg"])
        suction_by_cfg.setdefault(k, []).append(s)

    pivot_by_cfg = {}
    for p in pivot_sols:
        k = config_key(p["wrist_cfg"])
        pivot_by_cfg.setdefault(k, []).append(p)

    pickup_by_cfg = {}
    for pk in pickup_sols:
        k = config_key(pk["wrist_cfg"])
        pickup_by_cfg.setdefault(k, []).append(pk)

    shared_cfgs = set(suction_by_cfg) & set(pivot_by_cfg) & set(pickup_by_cfg)

    for cfg in sorted(shared_cfgs):
        s_list = sorted(suction_by_cfg[cfg], key=lambda x: x["theta_deg"])
        p_list = sorted(pivot_by_cfg[cfg], key=lambda x: x["theta_deg"])
        pk_list = sorted(pickup_by_cfg[cfg], key=lambda x: x["theta_deg"])

        # Stage 1: filter suction solutions that pass F3+F4
        valid_suction = []
        for s_sol in s_list:
            sj = s_sol["joints"]
            # J5 check on suction poses
            if not check_j5_singularity(sj["suction_offset_2"]):
                continue
            if not check_j5_singularity(sj["suction_position"]):
                continue
            # F3: offset2 → suction
            ok, err = test_lmove(robot, RDK, sj["suction_offset_2"], sj["suction_position"], suction_tool)
            if not ok:
                print(f"    [F3 FAIL] suction theta={s_sol['theta_deg']:.0f}: {err}")
                continue
            # F4: suction → offset2
            ok, err = test_lmove(robot, RDK, sj["suction_position"], sj["suction_offset_2"], suction_tool)
            if not ok:
                print(f"    [F4 FAIL] suction theta={s_sol['theta_deg']:.0f}: {err}")
                continue
            valid_suction.append(s_sol)

        if not valid_suction:
            print(f"    [SKIP] config={cfg}: no suction solutions pass F3+F4")
            continue

        print(f"    {len(valid_suction)}/{len(s_list)} suction solutions pass F3+F4")

        # Stage 2: for each valid suction, try pivot solutions for F5
        for s_sol in valid_suction:
            sj = s_sol["joints"]

            # Sort pivots by theta distance from this suction
            pivots_by_dist = sorted(p_list,
                key=lambda p: abs(p["theta_deg"] - s_sol["theta_deg"]))

            for p_sol in pivots_by_dist:
                pj = p_sol["joints"]
                if not check_j5_singularity(pj["pivot_after"]):
                    continue
                ok, err = test_lmove(robot, RDK, sj["suction_offset_2"], pj["pivot_after"], suction_tool)
                if not ok:
                    print(f"    [F5 FAIL] s={s_sol['theta_deg']:.0f} p={p_sol['theta_deg']:.0f}: {err}")
                    continue

                # Stage 3: try pickup solutions for F7+F8
                for pk_sol in pk_list:
                    pkj = pk_sol["joints"]
                    if not check_j5_singularity(pkj["cone_pickup_pose"]):
                        continue
                    if not check_j5_singularity(pkj["post_pickup_above"]):
                        continue
                    # F7: pivot → pickup (pickup tool after tool switch)
                    ok, err = test_lmove(robot, RDK, pj["pivot_after"], pkj["cone_pickup_pose"], pickup_tool)
                    if not ok:
                        continue  # don't spam — lots of pickup combos
                    # F8: pickup → post
                    ok, err = test_lmove(robot, RDK, pkj["cone_pickup_pose"], pkj["post_pickup_above"], pickup_tool)
                    if not ok:
                        continue

                    print(f"    [VERIFIED] s={s_sol['theta_deg']:.0f} "
                          f"p={p_sol['theta_deg']:.0f} "
                          f"pk={pk_sol['theta_deg']:.0f} — all LMoves pass")
                    return s_sol, p_sol, pk_sol, cfg

                print(f"    [F7/F8 FAIL] s={s_sol['theta_deg']:.0f} p={p_sol['theta_deg']:.0f}: "
                      f"no pickup solution passed")

    return None, None, None, None


PROGRAM_FOLDER_NAME = "pivot_programs"
PROGRAM_TARGETS_SUBFOLDER = "targets"
PROGRAM_PROGRAMS_SUBFOLDER = "programs"


def build_cone_program(robot, RDK, cone_name, suction_tool, pickup_tool,
                       suction_sol, pivot_sol, pickup_sol, offset1_joints,
                       target_folder, program_folder):
    """Build one RoboDK program for the full pivot sequence of a cone.

    Sequence:
      F1: JMove home → suction_offset_1 (knotting)
      F2: JMove suction_offset_1 → suction_offset_2
      F3: LMove suction_offset_2 → suction_position (grab string)
      F4: LMove suction_position → suction_offset_2 (retract)
      F5: LMove suction_offset_2 → pivot_after (pivot — same theta as suction)
      F6: tool switch knotting → pickup
      F7: LMove before_pickup_offset → cone_pickup_pose (grab cone)
      F8: LMove cone_pickup_pose → post_pickup_above (lift)
      F9: JMove post_pickup_above → home
    """
    prog_name = f"{cone_name}_pivot_sequence"

    # Clean up old program
    for ptype in [ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON]:
        old = RDK.Item(prog_name, ptype)
        if old.Valid():
            old.Delete()

    # Create cone-specific target subfolder
    cone_tgt_folder = get_or_create_folder(RDK, cone_name, parent=target_folder)

    def make_target(name, joints):
        tgt = RDK.AddTarget(name, cone_tgt_folder, robot)
        tgt.setJoints(joints)
        tgt.setAsJointTarget()
        return tgt

    s_joints = suction_sol["joints"]
    p_joints = pivot_sol["joints"]
    pk_joints = pickup_sol["joints"]

    t_home = make_target(f"{cone_name}_home", TRANSPORT_JOINTS)
    t_offset1 = make_target(f"{cone_name}_suction_offset_1", offset1_joints)
    t_offset2 = make_target(f"{cone_name}_suction_offset_2", s_joints["suction_offset_2"])
    t_suction = make_target(f"{cone_name}_suction_position", s_joints["suction_position"])
    t_pivot = make_target(f"{cone_name}_pivot_after", p_joints["pivot_after"])
    t_pickup = make_target(f"{cone_name}_cone_pickup", pk_joints["cone_pickup_pose"])
    t_post = make_target(f"{cone_name}_post_pickup", pk_joints["post_pickup_above"])

    # Build program
    prog = RDK.AddProgram(prog_name, robot)

    prog.RunInstruction(f"# {cone_name} pivot sequence", 0)
    prog.RunInstruction(f"# suction theta={suction_sol['theta_deg']:.0f} "
                        f"pivot theta={pivot_sol['theta_deg']:.0f} "
                        f"pickup theta={pickup_sol['theta_deg']:.0f}", 0)

    # F1-F2: JMove approach (knotting tool)
    prog.setPoseTool(suction_tool)
    prog.MoveJ(t_home)
    prog.MoveJ(t_offset1)
    prog.MoveJ(t_offset2)

    # F3: LMove grab string
    prog.MoveL(t_suction)

    # F4: LMove retract to offset_2
    prog.MoveL(t_offset2)

    # F5: LMove to pivot (same theta as suction — orientations compatible)
    prog.MoveL(t_pivot)

    # F6: tool switch
    prog.setPoseTool(pickup_tool)

    # F7: LMove to cone pickup
    prog.MoveL(t_pickup)

    # F8: LMove lift out
    prog.MoveL(t_post)

    # F9: JMove home
    prog.MoveJ(t_home)

    # Move program into programs subfolder
    prog.setParent(program_folder)

    return prog


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Coupled pivot solver demo (task 4c)")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--step-deg", type=float, default=15.0,
                    help="Z-rotation step size in degrees (default: 15)")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 3 4 4b 5 6 7")
    ap.add_argument("--non-verbose", action="store_true",
                    help="Suppress per-angle diagnostic output")
    args = ap.parse_args()
    args.verbose = not args.non_verbose

    skip = {s.upper() for s in args.skip}
    if skip:
        print(f"[SKIP] Skipping phases: {', '.join(sorted(skip))}")

    RDK = connect(args.robodk_ip)
    RDK._setTimeout(300)  # 5 min — IK solver loop can be slow

    # ── Step 1: Assert prereqs ──────────────────────────────────────────
    print("\n[STEP 1] Assert prerequisites...")

    robot = find_robot(RDK)
    assert robot is not None, f"Robot not found. Tried: {ROBOT_NAMES}"
    print(f"  Robot: {robot.Name()}")

    robot_base = robot.Parent()
    assert robot_base.Valid(), "Robot has no valid parent frame"
    print(f"  Base:  {robot_base.Name()}")

    suction_tool = RDK.Item(SUCTION_TOOL_NAME, ITEM_TYPE_TOOL)
    assert suction_tool.Valid(), f"Tool '{SUCTION_TOOL_NAME}' not found"
    print(f"  Suction tool: {suction_tool.Name()}")

    pickup_tool = RDK.Item(PICKUP_TOOL_NAME, ITEM_TYPE_TOOL)
    assert pickup_tool.Valid(), f"Tool '{PICKUP_TOOL_NAME}' not found"
    print(f"  Pickup tool:  {pickup_tool.Name()}")

    # Discover cones
    cones = discover_bin_cones(RDK)
    assert len(cones) == EXPECTED_CONE_COUNT, (
        f"Expected {EXPECTED_CONE_COUNT} cones under '{BIN_PARENT_NAME}', "
        f"found {len(cones)}: {[c[0] for c in cones]}"
    )
    print(f"  Cones ({len(cones)}): {[c[0] for c in cones]}")

    # Assert child frames for each cone
    cone_cache = {}  # cone_name -> {suffix: item}
    for cone_name, cone_item in cones:
        frames = assert_cone_frames(cone_name, cone_item)
        cone_cache[cone_name] = frames
        print(f"  {cone_name}: all {len(CHILD_SUFFIXES)} child frames OK")

    print("[OK] All prerequisites met.\n")

    # ── Step 2: Read poses ──────────────────────────────────────────────
    print("[STEP 2] Reading poses from station...")

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        print("  [WARN] WorldFrame not found — creating at identity")
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    robot.setPoseFrame(world_frame)
    print(f"  PoseFrame set to: {world_frame.Name()}")

    cone_poses = {}  # cone_name -> {suffix: Mat}
    for cone_name, frames in cone_cache.items():
        poses = {}
        for suffix, item in frames.items():
            poses[suffix] = item.PoseAbs()
        cone_poses[cone_name] = poses
        print(f"  {cone_name}: read {len(poses)} poses")

    # ── Step 2.5: Compute pickup-to-suction transform ───────────────────
    print("\n[STEP 2.5] Computing pickup-to-suction transform...")

    suction_TCP = suction_tool.PoseTool()
    pickup_TCP = pickup_tool.PoseTool()
    T_pickup_to_suction = invH(pickup_TCP) * suction_TCP

    t_full = Pose_2_TxyzRxyz(T_pickup_to_suction)
    print(f"  suction TCP: {Pose_2_TxyzRxyz(suction_TCP)}")
    print(f"  pickup TCP:  {Pose_2_TxyzRxyz(pickup_TCP)}")
    print(f"  T_pickup_to_suction: pos=[{t_full[0]:.1f}, {t_full[1]:.1f}, {t_full[2]:.1f}] "
          f"rot=[{t_full[3]:.1f}, {t_full[4]:.1f}, {t_full[5]:.1f}]")

    # ── Phase 3: Clean up old folder ─────────────────────────────────────
    if "3" not in skip:
        print("\n── Phase 3: Cleaning up old discovered_targets folder ──")
        old_folder = RDK.Item(TARGET_FOLDER_NAME, ITEM_TYPE_FOLDER)
        if old_folder.Valid():
            old_folder.Delete()
            print(f"  [CLEAN] Deleted old '{TARGET_FOLDER_NAME}' folder")
        else:
            print(f"  No previous '{TARGET_FOLDER_NAME}' folder found")
    else:
        print("\n── Phase 3: SKIPPED ──")

    # ── Diagnostic: verify robot setup ──────────────────────────────────
    print("\n[DIAG] Robot setup before solving:")
    robot_base_pose = Pose_2_TxyzRxyz(robot.PoseAbs())
    print(f"  Robot base (world): [{robot_base_pose[0]:.0f}, {robot_base_pose[1]:.0f}, {robot_base_pose[2]:.0f}]")
    robot.setPoseTool(suction_tool)
    suction_tcp_xyz = Pose_2_TxyzRxyz(robot.PoseTool())[:3]
    print(f"  Active tool TCP: [{suction_tcp_xyz[0]:.0f}, {suction_tcp_xyz[1]:.0f}, {suction_tcp_xyz[2]:.0f}]")
    print(f"  Pose frame: WorldFrame={world_frame.Valid()}")
    # Quick reachability test — try to reach the first cone's suction_offset_2
    first_cone = list(cone_poses.keys())[0]
    test_pose = cone_poses[first_cone]["suction_offset_2"]
    test_xyz = Pose_2_TxyzRxyz(test_pose)[:3]
    dist = math.sqrt(sum((test_xyz[i] - robot_base_pose[i]) ** 2 for i in range(3)))
    print(f"  First target (suction_offset_2): [{test_xyz[0]:.0f}, {test_xyz[1]:.0f}, {test_xyz[2]:.0f}]")
    print(f"  Distance from robot base: {dist:.0f}mm (robot reach: 3024mm)")

    # ── Phases 4-6: Decoupled sweeps per cone ─────────────────────────
    all_results = {}

    # ── Phase 4: Suction sweep ────────────────────────────────────────
    if "4" not in skip:
        print(f"\n── Phase 4: Suction sweep (step_deg={args.step_deg}) ──")
        for cone_name, poses in cone_poses.items():
            print(f"\n  === {cone_name} ===")
            suction_sols, suction_attempts = sweep_suction(robot, RDK, suction_tool, poses, args.step_deg)
            print(f"    {len(suction_sols)} suction solutions")
            print_attempt_report("suction", suction_attempts,
                                 ["offset2", "suction"])
            all_results.setdefault(cone_name, {})["suction_sols"] = suction_sols
    else:
        print("\n── Phase 4: SKIPPED ──")
        for cone_name in cone_poses:
            all_results.setdefault(cone_name, {})["suction_sols"] = []

    # ── Phase 4b: Pickup sweep ────────────────────────────────────────
    if "4B" not in skip:
        print(f"\n── Phase 4b: Pickup sweep (step_deg={args.step_deg}) ──")
        for cone_name, poses in cone_poses.items():
            print(f"\n  === {cone_name} ===")
            pickup_sols, pickup_attempts = sweep_pickup(robot, RDK, pickup_tool, poses, args.step_deg)
            print(f"    {len(pickup_sols)} pickup solutions")
            print_attempt_report("pickup", pickup_attempts,
                                 ["cone_pickup", "post_pickup_above"])
            all_results[cone_name]["pickup_sols"] = pickup_sols
    else:
        print("\n── Phase 4b: SKIPPED ──")
        for cone_name in cone_poses:
            all_results[cone_name]["pickup_sols"] = []

    # ── Phase 5: Pivot sweep + save + overlap ─────────────────────────
    if "5" not in skip:
        print(f"\n── Phase 5: Pivot sweep (step_deg={args.step_deg}) + save + overlap ──")
        for cone_name, poses in cone_poses.items():
            print(f"\n  === {cone_name} ===")
            pivot_sols, pivot_attempts = sweep_pivot(robot, RDK, suction_tool, poses, T_pickup_to_suction, args.step_deg)
            print(f"    {len(pivot_sols)} pivot pairs")
            print_attempt_report("pivot", pivot_attempts,
                                 ["pivot_after"])
            all_results[cone_name]["pivot_sols"] = pivot_sols

            suction_sols = all_results[cone_name]["suction_sols"]
            pickup_sols = all_results[cone_name]["pickup_sols"]

            # Save all three to station
            if suction_sols or pivot_sols or pickup_sols:
                save_solutions_to_station(RDK, robot, cone_name, suction_sols, pivot_sols, pickup_sols)
                print(f"    Saved to station under {TARGET_FOLDER_NAME}/{cone_name}/")

            # Check config overlap across all three
            print(f"  [Match] Checking config overlap (suction × pivot × pickup)...")
            overlap = find_config_overlap(suction_sols, pivot_sols, pickup_sols)
            all_results[cone_name]["overlap"] = overlap
    else:
        print("\n── Phase 5: SKIPPED ──")
        for cone_name in cone_poses:
            all_results[cone_name]["pivot_sols"] = []
            all_results[cone_name]["overlap"] = {}

    # ── Phase 6: Summary ──────────────────────────────────────────────
    if "6" not in skip:
        print(f"\n{'='*60}")
        print(f"DECOUPLED PIVOT SOLVER RESULTS")
        print(f"{'='*60}")
        print(f"Step size: {args.step_deg} deg")
        print(f"Seeds: {list(SWEEP_SEEDS.keys())}")
        print(f"Cones tested: {len(all_results)}")
        print()

        for cone_name, res in all_results.items():
            suction_sols = res.get("suction_sols", [])
            pivot_sols = res.get("pivot_sols", [])
            pickup_sols = res.get("pickup_sols", [])
            overlap = res.get("overlap", {})
            has_overlap = len(overlap) > 0
            status = "HAS OVERLAP" if has_overlap else "NO OVERLAP"
            print(f"  {cone_name}: {status}")
            print(f"    Suction solutions: {len(suction_sols)}")
            print(f"    Pivot solutions:   {len(pivot_sols)}")
            print(f"    Pickup solutions:  {len(pickup_sols)}")
            if overlap:
                for cfg, counts in overlap.items():
                    print(f"    Config {cfg}: {counts['suction']} suction × {counts['pivot']} pivot × {counts['pickup']} pickup")
            else:
                suction_cfgs = set(config_key(s["wrist_cfg"]) for s in suction_sols)
                pivot_cfgs = set(config_key(s["wrist_cfg"]) for s in pivot_sols)
                pickup_cfgs = set(config_key(s["wrist_cfg"]) for s in pickup_sols)
                if suction_cfgs:
                    print(f"    Suction configs: {sorted(suction_cfgs)}")
                if pivot_cfgs:
                    print(f"    Pivot configs:   {sorted(pivot_cfgs)}")
                if pickup_cfgs:
                    print(f"    Pickup configs:  {sorted(pickup_cfgs)}")

        print(f"\n{'='*60}")
        cones_with_overlap = sum(1 for r in all_results.values() if r.get("overlap"))
        print(f"Cones with config overlap: {cones_with_overlap} / {len(all_results)}")
        if cones_with_overlap > 0:
            print(f"Inspect targets in RoboDK: {TARGET_FOLDER_NAME}/ → right-click target → Move to Target")
            print(f"Next step: LMove verification (Phase D) to find viable theta pairings")
        else:
            print("No config overlap found for any cone.")
    else:
        print("\n── Phase 6: SKIPPED ──")

    # ── Phase 7: Build RoboDK programs ────────────────────────────────
    if "7" not in skip:
        print(f"\n── Phase 7: Build RoboDK programs ──")

        # If sweeps were skipped, load solutions from the saved station tree
        for cone_name in cone_poses:
            res = all_results.get(cone_name, {})
            has_data = (res.get("suction_sols") or res.get("pivot_sols")
                        or res.get("pickup_sols"))
            if not has_data:
                print(f"  [LOAD] Loading saved targets for {cone_name} from station...")
                loaded = load_solutions_from_station(RDK, cone_name)
                all_results.setdefault(cone_name, {}).update(loaded)
                print(f"    suction={len(loaded['suction_sols'])} "
                      f"pivot={len(loaded['pivot_sols'])} "
                      f"pickup={len(loaded['pickup_sols'])}")

        # Recompute overlap for any cone that doesn't have it yet
        for cone_name, res in all_results.items():
            if not res.get("overlap"):
                s = res.get("suction_sols", [])
                p = res.get("pivot_sols", [])
                pk = res.get("pickup_sols", [])
                if s or p or pk:
                    res["overlap"] = find_config_overlap(s, p, pk)

        # Clean up old program folder
        old_prog_folder = RDK.Item(PROGRAM_FOLDER_NAME, ITEM_TYPE_FOLDER)
        if old_prog_folder.Valid():
            old_prog_folder.Delete()
            print(f"  [CLEAN] Deleted old '{PROGRAM_FOLDER_NAME}' folder")
        root_folder = get_or_create_folder(RDK, PROGRAM_FOLDER_NAME)
        target_folder = get_or_create_folder(RDK, PROGRAM_TARGETS_SUBFOLDER, parent=root_folder)
        program_folder = get_or_create_folder(RDK, PROGRAM_PROGRAMS_SUBFOLDER, parent=root_folder)

        built = 0
        skipped = 0
        for cone_name, res in all_results.items():
            suction_sols = res.get("suction_sols", [])
            pivot_sols = res.get("pivot_sols", [])
            pickup_sols = res.get("pickup_sols", [])

            # Find triplet: matching config + ALL LMoves verified (incl. J5 singularity)
            s_sol, p_sol, pk_sol, cfg = find_viable_triplet(
                robot, RDK, suction_tool, pickup_tool,
                suction_sols, pivot_sols, pickup_sols
            )

            if s_sol is None:
                # Report why
                s_cfgs = set(config_key(s["wrist_cfg"]) for s in suction_sols)
                p_cfgs = set(config_key(p["wrist_cfg"]) for p in pivot_sols)
                pk_cfgs = set(config_key(pk["wrist_cfg"]) for pk in pickup_sols)
                shared = s_cfgs & p_cfgs & pk_cfgs
                if not shared:
                    print(f"  [SKIP] {cone_name} — no config overlap across all three")
                else:
                    print(f"  [SKIP] {cone_name} — shared configs {sorted(shared)} but all LMoves failed")
                    print(f"         Consider using JMove for suction_offset_2 → pivot_after instead")
                skipped += 1
                continue

            print(f"  [MATCH] {cone_name}: config={cfg} "
                  f"suction+pivot theta={s_sol['theta_deg']:.0f} "
                  f"pickup theta={pk_sol['theta_deg']:.0f}")

            # Solve suction_offset_1 independently (JMove, Z-free)
            robot.setPoseTool(suction_tool)
            o1_joints, _, o1_angle = try_ik_z_sweep(
                robot, RDK, cone_poses[cone_name]["suction_offset_1"],
                label=f"{cone_name}_offset1"
            )
            if o1_joints is None:
                print(f"  [SKIP] {cone_name} — suction_offset_1 unreachable (Z-free sweep)")
                skipped += 1
                continue

            prog = build_cone_program(
                robot, RDK, cone_name, suction_tool, pickup_tool,
                s_sol, p_sol, pk_sol, o1_joints,
                target_folder, program_folder
            )
            n_ins = prog.InstructionCount()
            print(f"  [PROG] {cone_name}: {n_ins} instructions")
            built += 1

        print(f"\n  Built: {built}, Skipped: {skipped}")
        if built > 0:
            print(f"  Step through in RoboDK: right-click program → Run step-by-step")
    else:
        print("\n── Phase 7: SKIPPED ──")


if __name__ == "__main__":
    main()
