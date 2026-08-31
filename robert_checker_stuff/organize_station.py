"""
Organize the extracted station for base cone movement sequence testing.

Phase 1 — create Offset_relative_to_schematic frame:
  - Child of WorldFrame, at identity (same position as WorldFrame)
  - All items except WorldFrame and RobotBase are reparented under it

Phase 2 — organize items into visible GUI folders:
  - extracted_targets  — all Target items (grab + string_grab only)
  - cones              — Base cone objects (Base_Right_*, Base_Left_*, alt_Base_*)
  - bins               — Bin objects (bin_*)

Phase 3 — create empty programs for each grab/string_grab pair:
  - One program per base cone, named after the cone (e.g. "Base_Right_0")
  - All programs go under a "programs" folder

Phase 4 — create offset targets for each grab and string_grab:
  - offset_before_for_<name> — offset along -Z before the target (approach)
  - offset_after_for_<name>  — offset along -Z after the target (retract)
  - Distances from config (grab: 50/300mm, string_grab: 50/50mm)
  - Stored under auto_generated_offsets/before and auto_generated_offsets/after

Reads settings from setup_base_movements_config.json (same directory).

Caching: folders, item placements, and programs are reused if they already exist.

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
from robodk.robomath import transl, eye

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "setup_base_movements_config.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

CONE_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+$")
BIN_PATTERN = re.compile(r"^bin_\d+$")
GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_grab$")
STRING_GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_string_grab$")
EXTRACTED_TARGET_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_(grab|string_grab)$")

OFFSET_FRAME_NAME = "Offset_relative_to_schematic"
SKIP_REPARENT = {"WorldFrame", "RobotBase", OFFSET_FRAME_NAME}

FOLDER_DEFS = {
    "extracted_targets": {
        "item_type": ITEM_TYPE_TARGET,
        "filter": EXTRACTED_TARGET_PATTERN,
    },
    "cones": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": CONE_PATTERN,
    },
    "bins": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": BIN_PATTERN,
    },
}


# ── CONFIG ────────────────────────────────────────────────────────────────────

def load_config(path):
    assert os.path.exists(path), f"Config not found: {path}"
    with open(path, "r", encoding="utf-8") as f:
        config = json.load(f)

    offsets = config.get("offsets_mm")
    assert offsets is not None, "Config missing 'offsets_mm'"
    for key in ("grab", "string_grab"):
        assert key in offsets, f"offsets_mm missing '{key}'"
        for direction in ("before", "after"):
            assert direction in offsets[key], f"offsets_mm.{key} missing '{direction}'"

    print(f"[CONFIG] offsets_mm.grab: before={offsets['grab']['before']}mm, after={offsets['grab']['after']}mm")
    print(f"[CONFIG] offsets_mm.string_grab: before={offsets['string_grab']['before']}mm, after={offsets['string_grab']['after']}mm")
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


# ── FOLDERS ───────────────────────────────────────────────────────────────────

def get_or_create_folder(RDK, name, parent=None):
    """Return existing folder named `name`, or create one.

    When parent is given, checks children of parent first to handle duplicate
    folder names at different levels.
    """
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

def items_matching(RDK, item_type, pattern):
    all_items = RDK.ItemList(item_type)
    if pattern is None:
        return all_items
    return [it for it in all_items if pattern.match(it.Name())]


def move_item_to_folder(item, folder):
    parent = item.Parent()
    if parent.Valid() and parent.Name() == folder.Name():
        return False
    world_pose = item.PoseAbs()
    item.setParent(folder)
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
    """Create Offset_relative_to_schematic frame under WorldFrame.

    Reparents all station-root children (except WorldFrame and RobotBase)
    under this frame, preserving their world poses.
    """
    # Check if already exists
    existing = RDK.Item(OFFSET_FRAME_NAME, ITEM_TYPE_FRAME)
    if existing.Valid():
        print(f"[CACHE] '{OFFSET_FRAME_NAME}' already exists")
        return existing

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    assert world_frame.Valid(), "WorldFrame not found"

    offset_frame = RDK.AddFrame(OFFSET_FRAME_NAME, world_frame)
    offset_frame.setPose(eye(4))  # identity — same position as WorldFrame
    print(f"[CREATE] Created '{OFFSET_FRAME_NAME}' under WorldFrame")

    return offset_frame


def reparent_items_under_offset_frame(RDK, offset_frame):
    """Move all station-root items (except WorldFrame, RobotBase, and the offset
    frame itself) under the offset frame, preserving world poses."""
    station = RDK.ActiveStation()
    children = station.Childs()

    moved = 0
    for child in children:
        name = child.Name()
        if name in SKIP_REPARENT:
            continue
        # Skip the robot (it's under RobotBase)
        if child.Type() == ITEM_TYPE_ROBOT:
            continue
        # Skip station-level items that shouldn't move
        if child.Type() == ITEM_TYPE_STATION:
            continue

        world_pose = child.PoseAbs()
        child.setParent(offset_frame)
        child.setPoseAbs(world_pose)
        moved += 1

    print(f"[REPARENT] Moved {moved} item(s) under '{OFFSET_FRAME_NAME}'")


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


# ── PHASE 4: OFFSET TARGETS ─────────────────────────────────────────────────

def create_offset_target(RDK, robot, source_target, offset_name, offset_mm, folder):
    existing = RDK.Item(offset_name, ITEM_TYPE_TARGET)
    if existing.Valid():
        return existing, False

    source_pose = source_target.Pose()
    offset_pose = source_pose * transl(0, 0, -offset_mm)

    target = RDK.AddTarget(offset_name, folder, robot)
    target.setPose(offset_pose)
    return target, True


def create_offset_targets(RDK, robot, offsets_config, before_folder, after_folder):
    created = 0
    skipped = 0

    offset_rules = [
        (GRAB_PATTERN, offsets_config["grab"]),
        (STRING_GRAB_PATTERN, offsets_config["string_grab"]),
    ]

    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)

    for target_item in all_targets:
        name = target_item.Name()
        for pattern, distances in offset_rules:
            if not pattern.match(name):
                continue

            before_name = f"offset_before_for_{name}"
            _, was_created = create_offset_target(
                RDK, robot, target_item, before_name, distances["before"], before_folder
            )
            created += was_created
            skipped += (not was_created)

            after_name = f"offset_after_for_{name}"
            _, was_created = create_offset_target(
                RDK, robot, target_item, after_name, distances["after"], after_folder
            )
            created += was_created
            skipped += (not was_created)

            break

    total = created + skipped
    print(f"[offsets] {total} offset target(s): {created} created, {skipped} already exist")


# ── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Organize extracted station for base cone movement testing")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 1 2 3 4")
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

    # ── Phase 2: organize items into folders ──────────────────────────
    if "2" not in skip:
        print("\n── Phase 2: Organize items into folders ──")
        for folder_name, spec in FOLDER_DEFS.items():
            # Create folders under the offset frame so they move with it
            folder = get_or_create_folder(RDK, folder_name, parent=offset_frame)
            matched = items_matching(RDK, spec["item_type"], spec["filter"])

            moved = 0
            skipped_count = 0
            for item in matched:
                if move_item_to_folder(item, folder):
                    moved += 1
                else:
                    skipped_count += 1

            total = moved + skipped_count
            print(f"[{folder_name}] {total} item(s): {moved} moved, {skipped_count} already in place")

        for folder_name in FOLDER_DEFS:
            for child in offset_frame.Childs():
                if child.Name() == folder_name:
                    child.setVisible(True)
                    break
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

    # ── Phase 4: create offset targets ────────────────────────────────
    if "4" not in skip:
        print("\n── Phase 4: Create offset targets (before/after) ──")
        offsets_parent = get_or_create_folder(RDK, "auto_generated_offsets", parent=offset_frame)
        before_folder = get_or_create_folder(RDK, "before", parent=offsets_parent)
        after_folder = get_or_create_folder(RDK, "after", parent=offsets_parent)
        offsets_parent.setVisible(True)
        before_folder.setVisible(True)
        after_folder.setVisible(True)

        create_offset_targets(RDK, robot, config["offsets_mm"], before_folder, after_folder)
    else:
        print("\n── Phase 4: SKIPPED ──")

    print("\n[DONE] Station organization complete.")


if __name__ == "__main__":
    main()
