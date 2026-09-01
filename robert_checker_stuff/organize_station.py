"""
Organize the extracted station for base cone movement sequence testing.

Phase 1 — create Offset_relative_to_schematic frame:
  - Child of WorldFrame, at identity (same position as WorldFrame)
  - All items except WorldFrame, RobotBase, and skip list are reparented under it

Phase 2 — organize items into bin_positioner frame hierarchy:
  - bin_positioner (frame, under Offset_relative_to_schematic)
    - bin_0_group (frame)
      - bin_0 (object)
      - cone_frame (folder) — contains Base_Right_*, Base_Left_* cones + targets
    - bin_1_group (frame)
      - bin_1 (object)
      - cone_frame (folder) — contains alt_Base_Right_*, alt_Base_Left_* cones + targets
  Grouping controlled by organize_station_config.json

Phase 3 — create empty programs for each grab/string_grab pair:
  - One program per base cone, named after the cone (e.g. "Base_Right_0")
  - All programs go under a "programs" folder

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
from robodk.robomath import eye

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

    for group_cfg in config["bin_groups"]:
        frame_name = group_cfg["frame_name"]
        bin_name = group_cfg["bin_object"]
        cone_pat = re.compile(group_cfg["cone_pattern"])
        target_pat = re.compile(group_cfg["target_pattern"])

        # Create group frame under bin_positioner
        group_frame = get_or_create_frame(RDK, frame_name, positioner)
        group_frame.setVisible(True)

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

            # Create a frame for this specific cone
            cone_frame = get_or_create_folder(RDK, f"{cone_name}_frame", parent=group_frame)
            cone_frame.setVisible(True)

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


# ── PHASE 3: PROGRAMS ────────────────────────────────────────────────────────

def create_programs(RDK, robot, cone_names, programs_folder):
    created = 0
    skipped = 0

    for cone_name in cone_names:
        existing = RDK.Item(cone_name, ITEM_TYPE_PROGRAM)
        if existing.Valid():
            skipped += 1
            continue
        prog = RDK.AddProgram(cone_name, robot)
        prog.setParent(programs_folder)
        created += 1

    total = created + skipped
    print(f"[programs] {total} program(s): {created} created, {skipped} already exist")


# ── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Organize extracted station for base cone movement testing")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 1 2 3")
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

    # ── Phase 3: create programs for grab/string_grab pairs ───────────
    cone_names = discover_cone_names(RDK)

    if "3" not in skip:
        print("\n── Phase 3: Create programs for base cone pairs ──")
        print(f"[DISCOVER] {len(cone_names)} base cone(s) with grab + string_grab pairs")
        for name in cone_names:
            print(f"  - {name}")

        programs_folder = get_or_create_folder(RDK, "programs")
        programs_folder.setVisible(True)
        create_programs(RDK, robot, cone_names, programs_folder)
    else:
        print("\n── Phase 3: SKIPPED ──")

    print("\n[DONE] Station organization complete.")


if __name__ == "__main__":
    main()
