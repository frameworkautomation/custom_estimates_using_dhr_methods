"""
Coupled pivot solver demo — task 4c.

Solves the vacuum-to-pickup pivot sequence for each cone in the bin.
The robot grabs the string with suction, pivots so the pickup tool aligns
with the cone, then picks it up. A single Z-rotation must make the entire
chain feasible.

See coupled_pivot_spec.md for the full specification.

Usage:
    python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/coupled_pivot_demo.py --robodk-ip 172.23.208.1 --step-deg 5

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

TRANSPORT_JOINTS = [0, -50, 15, 0, -15, -90, 0]  # 7-DOF (j7=0)

# FK verification tolerance
FK_TOL_MM = 5.0

TARGET_FOLDER_NAME = "coupled_pivot_targets"

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


def _solve_ik_locked_j7(robot, RDK, pose, j7_target=0.0):
    """Solve IK with j7 locked — EXACT copy of proven robert_end_checker pattern."""
    global _last_ik_error
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    robot.setParam("OptimAxes", props)

    robot.setJoints(HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED)
        if len(joints) < 6:
            _last_ik_error = f"got {len(joints)} joints"
            return None, False
        return joints, True
    except Exception as e:
        _last_ik_error = str(e)
        robot.setJoints(HOME_SEED)
        return None, False


def try_ik(robot, RDK, pose, label=""):
    """Single IK attempt using the proven locked-j7 pattern."""
    joints, ok = _solve_ik_locked_j7(robot, RDK, pose)
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


# ── SEARCH B: COUPLED Z-ROTATION SWEEP ─────────────────────────────────────

def search_b_coupled(robot, RDK, robot_base, suction_tool, pickup_tool,
                     poses, T_pickup_to_suction, step_deg, verbose=True):
    """Coupled Z-rotation sweep over suction_position.

    Uses the proven 7-DOF locked-j7 IK pattern from robert_end_checker.py.
    Returns (theta_deg, step_joints) or (None, None) on failure.
    """
    suction_pose = poses["suction_position"]
    offset1_pose = poses["suction_offset_2"]
    before_pickup_pose = poses["before_pickup_offset"]
    cone_pickup_pose_val = poses["cone_pickup_pose"]

    # pivot_as_suction_tcp is deterministic — does not change with theta
    pivot_as_suction_tcp = before_pickup_pose * T_pickup_to_suction

    n_steps = int(360 / step_deg)

    for i in range(n_steps):
        theta_deg = step_deg * i
        theta_rad = theta_deg * math.pi / 180.0

        # Rotate all suction poses around their Z axis by theta
        rz = rotz(theta_rad)
        rotated_suction = suction_pose * rz
        rotated_offset1 = offset1_pose * rz

        # ── F2: suction_offset_2 -> rotated_suction (suction tool) ──
        robot.setPoseTool(suction_tool)

        lbl = f"offset1@{theta_deg:.0f}" if verbose else ""
        j_offset1 = try_ik(robot, RDK, rotated_offset1, label=lbl)
        if j_offset1 is None:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  offset1=FAIL")
            continue

        lbl = f"suction@{theta_deg:.0f}" if verbose else ""
        j_suction = try_ik(robot, RDK, rotated_suction, label=lbl)
        if j_suction is None:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  offset1=ok  suction=FAIL")
            continue

        # ── F3: rotated_suction -> pivot_as_suction_tcp (suction tool) ──
        lbl = f"pivot@{theta_deg:.0f}" if verbose else ""
        j_pivot = try_ik(robot, RDK, pivot_as_suction_tcp, label=lbl)
        if j_pivot is None:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  offset1=ok  suction=ok  pivot=FAIL")
            continue

        # ── FK verify pivot: switch to pickup, check TCP ≈ before_pickup_offset ──
        robot.setPoseTool(pickup_tool)
        robot.setJoints(j_pivot)
        achieved_pickup = robot.Pose()
        t = Pose_2_TxyzRxyz(before_pickup_pose)
        a = Pose_2_TxyzRxyz(achieved_pickup)
        pivot_err = math.sqrt(sum((t[k] - a[k]) ** 2 for k in range(3)))
        robot.setJoints(HOME_SEED)

        if pivot_err > FK_TOL_MM:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  offset1=ok  suction=ok  pivot=ok  fk_err={pivot_err:.1f}mm FAIL")
            continue

        # ── Check config consistency F2-F3 ──
        cfg_offset1 = get_config_flags(robot, j_offset1)
        cfg_suction = get_config_flags(robot, j_suction)
        cfg_pivot = get_config_flags(robot, j_pivot)
        if cfg_suction != cfg_pivot:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  ...  cfg_mismatch suction={cfg_suction} pivot={cfg_pivot}")
            continue

        # ── F5: before_pickup_offset -> cone_pickup_pose (pickup tool) ──
        robot.setPoseTool(pickup_tool)
        lbl = f"pickup@{theta_deg:.0f}" if verbose else ""
        j_pickup = try_ik(robot, RDK, cone_pickup_pose_val, label=lbl)
        if j_pickup is None:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  ...  cfg=ok  pickup=FAIL")
            continue

        # ── Check config consistency F4-F5 ──
        cfg_pickup = get_config_flags(robot, j_pickup)
        if cfg_pivot != cfg_pickup:
            if verbose:
                print(f"    [B] theta={theta_deg:5.0f}  ...  pickup_cfg_mismatch pivot={cfg_pivot} pickup={cfg_pickup}")
            continue

        # All passed!
        step_joints = {
            "suction_offset_2": j_offset1,
            "rotated_suction": j_suction,
            "pivot_as_suction_tcp": j_pivot,
            "cone_pickup_pose": j_pickup,
        }
        print(f"    [B] theta={theta_deg:5.0f}  SUCCESS  fk={pivot_err:.1f}mm  cfg={cfg_pivot}")
        return theta_deg, step_joints

    print(f"    [B] FAILED — no theta found in {n_steps} steps")
    return None, None


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Coupled pivot solver demo (task 4c)")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--step-deg", type=float, default=5.0,
                    help="Z-rotation step size in degrees (default: 5)")
    ap.add_argument("--non-verbose", action="store_true",
                    help="Suppress per-angle diagnostic output")
    args = ap.parse_args()
    args.verbose = not args.non_verbose

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

    # ── Step 3: Compute pivot_as_suction_tcp per cone ───────────────────
    print("\n[STEP 3] Computing pivot_as_suction_tcp for each cone...")

    for cone_name, poses in cone_poses.items():
        before_pickup = poses["before_pickup_offset"]
        pivot = before_pickup * T_pickup_to_suction
        poses["pivot_as_suction_tcp"] = pivot
        bxyz = Pose_2_TxyzRxyz(before_pickup)[:3]
        pxyz = Pose_2_TxyzRxyz(pivot)[:3]
        delta = [pxyz[i] - bxyz[i] for i in range(3)]
        print(f"  {cone_name}: before_pickup=[{bxyz[0]:.0f},{bxyz[1]:.0f},{bxyz[2]:.0f}] "
              f"pivot=[{pxyz[0]:.0f},{pxyz[1]:.0f},{pxyz[2]:.0f}] "
              f"delta=[{delta[0]:.0f},{delta[1]:.0f},{delta[2]:.0f}]")

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

    # ── Steps 4-6: Solve all three searches per cone ────────────────────
    print(f"\n[STEP 4-6] Solving (step_deg={args.step_deg})...")

    results = {}  # cone_name -> {search_a, search_b, search_c, feasible}

    for cone_name, poses in cone_poses.items():
        print(f"\n  === {cone_name} ===")
        result = {"feasible": False}

        # ── Search B (coupled) ──
        print(f"  [Search B] Coupled Z-rotation sweep...")
        theta, step_joints = search_b_coupled(
            robot, RDK, robot_base, suction_tool, pickup_tool,
            poses, T_pickup_to_suction, args.step_deg,
            verbose=args.verbose
        )
        if theta is None:
            print(f"  [SKIP] {cone_name} — Search B failed, skipping A and C")
            results[cone_name] = result
            continue
        result["search_b"] = {"theta_deg": theta, "joints": step_joints}

        # ── Search A (F1: JMove to suction_offset_1, Z-free sweep) ──
        print(f"  [Search A] Solve suction_offset_1 (Z-free sweep)...")
        robot.setPoseTool(suction_tool)
        a_joints, a_pose, a_angle = try_ik_z_sweep(
            robot, RDK, poses["suction_offset_1"], label="offset2"
        )
        if a_joints is None:
            print(f"    [A] FAILED — suction_offset_1 unreachable")
            results[cone_name] = result
            continue
        print(f"    [A] SUCCESS at {a_angle:.0f} deg")
        result["search_a"] = {"joints": a_joints}

        # ── Search C (F6: post_pickup_above, Z-free sweep) ──
        print(f"  [Search C] Solve post_pickup_above (Z-free sweep)...")
        robot.setPoseTool(pickup_tool)
        c_joints, c_pose, c_angle = try_ik_z_sweep(
            robot, RDK, poses["post_pickup_above"], label="post_pickup"
        )
        if c_joints is None:
            print(f"    [C] FAILED — post_pickup_above unreachable")
            results[cone_name] = result
            continue
        print(f"    [C] SUCCESS at theta={c_angle:.0f} deg")
        result["search_c"] = {"joints": c_joints, "theta_deg": c_angle, "pose": c_pose}

        result["feasible"] = True
        results[cone_name] = result

    # ── Step 7: Build RoboDK programs ───────────────────────────────────
    print("\n[STEP 7] Building RoboDK programs...")

    # Clean up old targets/programs
    old_folder = RDK.Item(TARGET_FOLDER_NAME, ITEM_TYPE_FOLDER)
    if old_folder.Valid():
        old_folder.Delete()
        print(f"  [CLEAN] Deleted old '{TARGET_FOLDER_NAME}' folder")

    for cone_name in cone_poses:
        prog_name = f"{cone_name}_coupled_pivot"
        for ptype in [ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON]:
            delete_if_exists(RDK, prog_name, ptype)

    target_folder = get_or_create_folder(RDK, TARGET_FOLDER_NAME)

    feasible_count = 0
    for cone_name, result in results.items():
        if not result["feasible"]:
            print(f"  [SKIP] {cone_name} — not feasible")
            continue

        feasible_count += 1
        prog_name = f"{cone_name}_coupled_pivot"
        search_a = result["search_a"]
        search_b = result["search_b"]
        search_c = result["search_c"]
        b_joints = search_b["joints"]

        # Create joint targets
        def make_joint_target(name, joints):
            tgt = RDK.AddTarget(name, target_folder, robot)
            tgt.setJoints(joints)
            tgt.setAsJointTarget()
            return tgt

        t_home = make_joint_target(f"{cone_name}_home", TRANSPORT_JOINTS)
        t_offset2 = make_joint_target(f"{cone_name}_suction_offset_1", search_a["joints"])
        t_offset1 = make_joint_target(f"{cone_name}_suction_offset_2", b_joints["suction_offset_2"])
        t_suction = make_joint_target(f"{cone_name}_rotated_suction", b_joints["rotated_suction"])
        t_pivot = make_joint_target(f"{cone_name}_pivot", b_joints["pivot_as_suction_tcp"])
        t_pickup = make_joint_target(f"{cone_name}_cone_pickup", b_joints["cone_pickup_pose"])
        t_post = make_joint_target(f"{cone_name}_post_pickup_above", search_c["joints"])

        # Build program
        prog = RDK.AddProgram(prog_name, robot)
        prog.setPoseFrame(robot_base)

        prog.RunInstruction(f"# {cone_name} coupled pivot sequence", 0)

        # F1: suction tool, JMove to safe position then approach
        prog.setPoseTool(suction_tool)
        prog.MoveJ(t_home)
        prog.MoveJ(t_offset2)
        prog.MoveJ(t_offset1)

        # F2: LMove to suction grab
        prog.MoveL(t_suction)

        # F3: LMove to pivot
        prog.MoveL(t_pivot)

        # F4: tool switch (same joints, different tool)
        prog.setPoseTool(pickup_tool)

        # F5: LMove to cone pickup
        prog.MoveL(t_pickup)

        # F6: LMove to post-pickup above
        prog.MoveL(t_post)

        # Return home
        prog.MoveJ(t_home)

        n_ins = prog.InstructionCount()
        print(f"  [PROG] {prog_name}: {n_ins} instructions, theta={search_b['theta_deg']:.0f} deg")

    # ── Step 8: Print summary ───────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"COUPLED PIVOT SOLVER RESULTS")
    print(f"{'='*60}")
    print(f"Step size: {args.step_deg} deg")
    print(f"Cones tested: {len(results)}")
    print(f"Feasible: {feasible_count} / {len(results)}")
    print()

    for cone_name, result in results.items():
        if result["feasible"]:
            b = result["search_b"]
            c = result["search_c"]
            print(f"  {cone_name}: FEASIBLE")
            print(f"    Search B theta: {b['theta_deg']:.0f} deg")
            print(f"    Search C theta: {c['theta_deg']:.0f} deg")
        else:
            reasons = []
            if "search_b" not in result:
                reasons.append("Search B failed")
            if "search_a" not in result:
                reasons.append("Search A failed")
            if "search_c" not in result:
                reasons.append("Search C failed")
            print(f"  {cone_name}: INFEASIBLE — {', '.join(reasons)}")

    print(f"\n{'='*60}")
    if feasible_count > 0:
        print(f"Programs created. Step through in RoboDK: right-click -> Run step-by-step")
    else:
        print("No feasible cones found.")


if __name__ == "__main__":
    main()
