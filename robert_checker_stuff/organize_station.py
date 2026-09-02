"""
Organize the extracted station for base cone movement sequence testing.

Phase 1 — create Offset_relative_to_schematic frame:
  - Child of WorldFrame, at identity (same position as WorldFrame)
  - All items except WorldFrame, RobotBase, and skip list are reparented under it

Phase 2 — organize items into bin_positioner frame hierarchy:
  - bin_positioner (frame, under Offset_relative_to_schematic)
    - bin_0_group (frame)
      - bin_0 (object)
      - <cone>_frame — per cone with cone object + targets
    - bin_1_group (frame)
      - bin_1 (object)
      - <cone>_frame — per cone with cone object + targets
  Grouping controlled by organize_station_config.json

Caching: frames, folders, item placements, and programs are reused if they already exist.

Usage:
    python robert_checker_stuff/organize_station.py
    python robert_checker_stuff/organize_station.py --robodk-ip 172.23.208.1
"""

import sys
import os
import re
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT,
    ITEM_TYPE_FOLDER, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME,
    ITEM_TYPE_STATION,
)
from robodk.robomath import eye, Mat, Pose_2_TxyzRxyz
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "organize_station_config.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_grab$")
STRING_GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_string_grab$")

OFFSET_FRAME_NAME = "Offset_relative_to_schematic"
SKIP_REPARENT = {"WorldFrame", "RobotBase", OFFSET_FRAME_NAME,
                 "Rack1GarmentTrayBase", "FrontWall", "RobotPedestal"}


# ── CONFIG ────────────────────────────────────────────────────────────────────

def load_config(path):
    assert os.path.exists(path), f"Config not found: {path}"
    with open(path, "r", encoding="utf-8") as f:
        config = json.load(f)
    assert "bin_groups" in config, "Config missing 'bin_groups'"
    assert len(config["bin_groups"]) > 0, "bin_groups is empty"
    return config


# ── CONNECT ───────────────────────────────────────────────────────────────────

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
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


# ── FRAMES & FOLDERS ─────────────────────────────────────────────────────────

def get_or_create_frame(RDK, name, parent):
    """Return existing frame under parent, or create one at identity."""
    for child in parent.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_FRAME:
            return child
    frame = RDK.AddFrame(name, parent)
    frame.setPose(eye(4))
    print(f"[CREATE] Created frame '{name}' under '{parent.Name()}'")
    return frame


def get_or_create_folder(RDK, name, parent=None):
    """Return existing folder named `name`, or create one."""
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
        print(f"[CREATE] Created folder '{name}' under '{parent.Name()}'")
        return folder

    existing = RDK.Item(name, ITEM_TYPE_FOLDER)
    if existing.Valid():
        return existing

    RDK.Command("AddFolder", name)
    folder = RDK.Item(name, ITEM_TYPE_FOLDER)
    assert folder.Valid(), f"Failed to create folder '{name}'"
    print(f"[CREATE] Created folder '{name}'")
    return folder


# ── HELPERS ───────────────────────────────────────────────────────────────────

def move_item_to_parent(item, new_parent):
    """Reparent item, preserving world pose. Returns True if moved."""
    parent = item.Parent()
    if parent.Valid() and parent.Name() == new_parent.Name():
        return False
    world_pose = item.PoseAbs()
    item.setParent(new_parent)
    item.setPoseAbs(world_pose)
    return True


def discover_cone_names(RDK):
    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
    target_names = {t.Name() for t in all_targets}

    grab_names = {n for n in target_names if GRAB_PATTERN.match(n)}
    string_grab_names = {n for n in target_names if STRING_GRAB_PATTERN.match(n)}

    grab_cones = {n.rsplit("_grab", 1)[0] for n in grab_names}
    string_grab_cones = {n.rsplit("_string_grab", 1)[0] for n in string_grab_names}

    paired = sorted(grab_cones & string_grab_cones)
    return paired


# ── PHASE 1: OFFSET FRAME ────────────────────────────────────────────────────

def create_offset_frame(RDK):
    existing = RDK.Item(OFFSET_FRAME_NAME, ITEM_TYPE_FRAME)
    if existing.Valid():
        print(f"[CACHE] '{OFFSET_FRAME_NAME}' already exists")
        return existing

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    assert world_frame.Valid(), "WorldFrame not found"

    offset_frame = RDK.AddFrame(OFFSET_FRAME_NAME, world_frame)
    offset_frame.setPose(eye(4))
    print(f"[CREATE] Created '{OFFSET_FRAME_NAME}' under WorldFrame")
    return offset_frame


def reparent_items_under_offset_frame(RDK, offset_frame):
    station = RDK.ActiveStation()
    children = station.Childs()

    moved = 0
    for child in children:
        name = child.Name()
        if name in SKIP_REPARENT:
            continue
        if child.Type() == ITEM_TYPE_ROBOT:
            continue
        if child.Type() == ITEM_TYPE_STATION:
            continue

        world_pose = child.PoseAbs()
        child.setParent(offset_frame)
        child.setPoseAbs(world_pose)
        moved += 1

    print(f"[REPARENT] Moved {moved} item(s) under '{OFFSET_FRAME_NAME}'")


# ── POSE HELPERS ─────────────────────────────────────────────────────────────

def pose_to_axes(pose):
    """Extract X, Y, Z axis vectors and position from a 4x4 pose matrix."""
    m = np.array(pose.rows)
    x_axis = m[0:3, 0]
    y_axis = m[0:3, 1]
    z_axis = m[0:3, 2]
    pos = m[0:3, 3]
    return x_axis, y_axis, z_axis, pos


def axes_to_pose(x_axis, y_axis, z_axis, pos):
    """Build a 4x4 pose matrix from axis vectors and position."""
    return Mat([
        [x_axis[0], y_axis[0], z_axis[0], pos[0]],
        [x_axis[1], y_axis[1], z_axis[1], pos[1]],
        [x_axis[2], y_axis[2], z_axis[2], pos[2]],
        [0, 0, 0, 1],
    ])


def compute_frame_pose(RDK, ref_target_name, offset_x_mm, offset_y_mm,
                       flip_x=False, flip_z=False):
    """Compute a frame pose based on a reference target's orientation and offsets."""
    ref = RDK.Item(ref_target_name, ITEM_TYPE_TARGET)
    assert ref.Valid(), f"Reference target '{ref_target_name}' not found"

    ref_pose = ref.PoseAbs()
    x_axis, y_axis, z_axis, ref_pos = pose_to_axes(ref_pose)

    # Apply flips
    new_x = -x_axis if flip_x else x_axis.copy()
    new_z = -z_axis if flip_z else z_axis.copy()
    # Recompute Y to maintain right-handedness
    new_y = np.cross(new_z, new_x)

    # Compute position: ref position + offsets along ref axes
    new_pos = ref_pos + offset_x_mm * x_axis + offset_y_mm * y_axis

    return axes_to_pose(new_x, new_y, new_z, new_pos)


# ── PHASE 2: BIN POSITIONER HIERARCHY ───────────────────────────────────────

def organize_bin_groups(RDK, config, offset_frame):
    """Create bin_positioner hierarchy from config.

    Structure:
      Offset_relative_to_schematic/
        bin_positioner/
          bin_0_group/
            bin_0 (object)
            cone_frame/ (folder)
              Base_Right_0 (object)
              Base_Right_0_grab (target)
              ...
          bin_1_group/
            bin_1 (object)
            cone_frame/ (folder)
              alt_Base_Right_0 (object)
              ...
    """
    positioner = get_or_create_frame(RDK, "bin_positioner", offset_frame)
    positioner.setVisible(True)

    # Set bin_positioner pose from config
    bp_cfg = config.get("bin_positioner", {})
    if "reference_target" in bp_cfg:
        bp_pose = compute_frame_pose(
            RDK,
            bp_cfg["reference_target"],
            bp_cfg.get("offset_along_ref_x_mm", 0),
            bp_cfg.get("offset_along_ref_y_mm", 0),
            flip_x=bp_cfg.get("flip_x", False),
            flip_z=bp_cfg.get("flip_z", False),
        )
        positioner.setPoseAbs(bp_pose)
        txyz = Pose_2_TxyzRxyz(bp_pose)
        print(f"  [POSE] bin_positioner at x={txyz[0]:.1f} y={txyz[1]:.1f} z={txyz[2]:.1f}")

    for group_cfg in config["bin_groups"]:
        frame_name = group_cfg["frame_name"]
        bin_name = group_cfg["bin_object"]
        cone_pat = re.compile(group_cfg["cone_pattern"])
        target_pat = re.compile(group_cfg["target_pattern"])

        # Create group frame under bin_positioner
        group_frame = get_or_create_frame(RDK, frame_name, positioner)
        group_frame.setVisible(True)

        # Set group frame pose from config
        ref_target = group_cfg.get("reference_target")
        if ref_target:
            group_pose = compute_frame_pose(
                RDK,
                ref_target,
                group_cfg.get("offset_along_ref_x_mm", 0),
                group_cfg.get("offset_along_ref_y_mm", 0),
            )
            group_frame.setPoseAbs(group_pose)
            txyz = Pose_2_TxyzRxyz(group_pose)
            print(f"  [POSE] {frame_name} at x={txyz[0]:.1f} y={txyz[1]:.1f} z={txyz[2]:.1f}")

        # Move bin object into group frame
        bin_obj = RDK.Item(bin_name, ITEM_TYPE_OBJECT)
        if bin_obj.Valid():
            if move_item_to_parent(bin_obj, group_frame):
                print(f"  [MOVE] {bin_name} → {frame_name}")
        else:
            print(f"  [WARN] Bin '{bin_name}' not found")

        # Find matching cone objects and create a cone_frame for each
        all_objects = RDK.ItemList(ITEM_TYPE_OBJECT)
        all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
        moved_cones = 0
        moved_targets = 0

        for obj in all_objects:
            cone_name = obj.Name()
            if not cone_pat.match(cone_name):
                continue

            # Create a frame for this specific cone, positioned at its string_grab target
            cone_frame = get_or_create_frame(RDK, f"{cone_name}_frame", group_frame)
            cone_frame.setVisible(True)

            # Set cone frame pose to the string_grab target location
            string_grab = RDK.Item(f"{cone_name}_string_grab", ITEM_TYPE_TARGET)
            if string_grab.Valid():
                cone_frame.setPoseAbs(string_grab.PoseAbs())

            # Move cone object into its frame
            if move_item_to_parent(obj, cone_frame):
                moved_cones += 1

            # Move matching targets (grab + string_grab) into the same frame
            for tgt in all_targets:
                tgt_name = tgt.Name()
                if tgt_name == f"{cone_name}_grab" or tgt_name == f"{cone_name}_string_grab":
                    if move_item_to_parent(tgt, cone_frame):
                        moved_targets += 1

        print(f"  [{frame_name}] {moved_cones} cone(s), {moved_targets} target(s) organized")


# ── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Organize extracted station for base cone movement testing")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 1 2")
    args = ap.parse_args()

    skip = {s.upper() for s in args.skip}
    if skip:
        print(f"[SKIP] Skipping phases: {', '.join(sorted(skip))}")

    config = load_config(args.config)
    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # ── Phase 1: create offset frame and reparent items ───────────────
    if "1" not in skip:
        print("\n── Phase 1: Create Offset_relative_to_schematic frame ──")
        offset_frame = create_offset_frame(RDK)
        reparent_items_under_offset_frame(RDK, offset_frame)
    else:
        print("\n── Phase 1: SKIPPED ──")
        offset_frame = RDK.Item(OFFSET_FRAME_NAME, ITEM_TYPE_FRAME)

    # ── Phase 2: organize into bin_positioner hierarchy ───────────────
    if "2" not in skip:
        print("\n── Phase 2: Organize into bin_positioner hierarchy ──")
        organize_bin_groups(RDK, config, offset_frame)
    else:
        print("\n── Phase 2: SKIPPED ──")

    print("\n[DONE] Station organization complete.")


if __name__ == "__main__":
    main()
