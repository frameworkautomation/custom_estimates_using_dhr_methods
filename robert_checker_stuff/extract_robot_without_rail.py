"""
Extract specified items from a source RoboDK station into a new, clean station.

The main use case: copy the robot arm at its j7=0 position WITHOUT the linear
rail, plus any other items (tools, objects, frames) listed in the config.
This gives a repeatable starting point for movement-sequence testing.

The config supports both explicit items (by name) and pattern-based discovery
(regex matching against all items of a given type in the source station).

Approach: Copy/Paste via the RoboDK API. Items are copied one at a time from
the source station and pasted into the destination. For the robot, Copy/Paste
strips the rail mechanism, giving a clean 6-DOF arm.

Usage:
    python robert_checker_stuff/extract_robot_without_rail.py
    python robert_checker_stuff/extract_robot_without_rail.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/extract_robot_without_rail.py --source my_station.rdk --dest output.rdk
"""

import sys
import os
import re
import json
import argparse
import math

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT, ITEM_TYPE_STATION,
)
from robodk.robomath import Pose_2_TxyzRxyz

# ── DEFAULTS ────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
SAVES_DIR = os.path.join(PROJECT_DIR, "robo_dk_saves")

DEFAULT_SOURCE = os.path.join(SAVES_DIR, "generated_from_dhr_clone.rdk")
DEFAULT_DEST = os.path.join(SAVES_DIR, "for_robert_relative_to_base.rdk")
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "station_extract_config.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

TYPE_MAP = {
    "robot": ITEM_TYPE_ROBOT,
    "tool": ITEM_TYPE_TOOL,
    "frame": ITEM_TYPE_FRAME,
    "target": ITEM_TYPE_TARGET,
    "object": ITEM_TYPE_OBJECT,
}


def to_robodk_path(path):
    """Convert path for RoboDK. Handles WSL /mnt/c/... -> C:/... conversion."""
    abs_path = os.path.abspath(path)
    try:
        if abs_path.startswith("/mnt/"):
            parts = abs_path.split("/")
            drive = parts[2].upper()
            rest = "/".join(parts[3:])
            return f"{drive}:/{rest}"
    except (IndexError, AttributeError):
        pass
    return abs_path


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


def find_robot_in_station(RDK, name):
    """Find the robot by name, trying known aliases. Returns item or None."""
    candidates = [name] if name not in ROBOT_NAMES else ROBOT_NAMES
    for rname in candidates:
        r = RDK.Item(rname, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


# ── PATTERN RESOLUTION ──────────────────────────────────────────────────────

def resolve_patterns(RDK, patterns):
    """Expand pattern entries into concrete item dicts by scanning the station.

    Each pattern has a regex and a type. We scan all items of that type in the
    station and return matches as explicit item dicts.
    """
    resolved = []
    for pat in patterns:
        regex = re.compile(pat["regex"])
        type_str = pat["type"]
        item_type = TYPE_MAP[type_str]
        parent_name = pat.get("parent")

        # Get all items of this type
        all_of_type = RDK.ItemList(item_type)
        matched = []
        for item in all_of_type:
            name = item.Name()
            if not regex.match(name):
                continue
            # If parent is specified, check that the item's parent matches
            if parent_name:
                parent = item.Parent()
                if not parent.Valid() or parent.Name() != parent_name:
                    continue
            matched.append({"name": name, "type": type_str})

        comment = pat.get("comment", pat["regex"])
        print(f"  Pattern '{comment}': {len(matched)} match(es)")
        for m in matched:
            print(f"    - {m['name']}")

        assert len(matched) > 0, \
            f"Pattern '{pat['regex']}' (type={type_str}) matched 0 items in source station"
        resolved.extend(matched)

    return resolved


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Extract items from a source RoboDK station into a clean new station"
    )
    ap.add_argument("--source", default=DEFAULT_SOURCE,
                    help=f"Source .rdk file (default: {os.path.basename(DEFAULT_SOURCE)})")
    ap.add_argument("--dest", default=DEFAULT_DEST,
                    help=f"Destination .rdk file (default: {os.path.basename(DEFAULT_DEST)})")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--use-current", action="store_true",
                    help="Use the currently open station instead of loading --source")
    args = ap.parse_args()

    # ── Load config ──────────────────────────────────────────────────────
    assert os.path.exists(args.config), f"Config file not found: {args.config}"
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    explicit_items = config.get("items", [])
    patterns = config.get("patterns", [])
    # Support both single string and list forms
    subtree_single = config.get("copy_subtree_from")
    subtree_list = config.get("copy_subtrees_from", [])
    subtrees_from = subtree_list if subtree_list else ([subtree_single] if subtree_single else [])

    assert len(explicit_items) > 0 or len(patterns) > 0 or subtrees_from, \
        "Config has no items, patterns, or copy_subtree(s)_from to extract"

    # Validate explicit items
    for item_cfg in explicit_items:
        assert "name" in item_cfg, f"Item missing 'name': {item_cfg}"
        assert "type" in item_cfg, f"Item '{item_cfg['name']}' missing 'type'"
        assert item_cfg["type"] in TYPE_MAP, \
            f"Unknown type '{item_cfg['type']}' for item '{item_cfg['name']}'"

    print(f"[CONFIG] {len(explicit_items)} explicit item(s), {len(patterns)} pattern(s)")
    if subtrees_from:
        print(f"[CONFIG] Subtree copies: {subtrees_from}")

    # ── Connect to RoboDK ────────────────────────────────────────────────
    RDK = connect(args.robodk_ip)

    # ── Load source station ──────────────────────────────────────────────
    if args.use_current:
        src_station = RDK.ActiveStation()
        assert src_station.Valid(), "No active station in RoboDK"
        print(f"\n[LOAD] Using current station: '{src_station.Name()}'")
    else:
        assert os.path.exists(args.source), f"Source station not found: {args.source}"
        source_path = to_robodk_path(args.source)
        print(f"\n[LOAD] Opening source station: {source_path}")
        src_station = RDK.AddFile(source_path)
        assert src_station.Valid(), f"Failed to load source station: {args.source}"
        print(f"[LOAD] Source station loaded: '{src_station.Name()}'")

    # ── Resolve patterns into concrete items ─────────────────────────────
    if patterns:
        print(f"\n[PATTERNS] Resolving {len(patterns)} pattern(s)...")
        pattern_items = resolve_patterns(RDK, patterns)
    else:
        pattern_items = []

    # Merge: explicit items first, then pattern-discovered items
    # Deduplicate by (name, type)
    seen = set()
    all_items = []
    for item_cfg in explicit_items + pattern_items:
        key = (item_cfg["name"], item_cfg["type"])
        if key not in seen:
            seen.add(key)
            all_items.append(item_cfg)

    print(f"\n[TOTAL] {len(all_items)} item(s) to extract")

    # ── Verify all items exist in source ─────────────────────────────────
    # Items that will be auto-created if missing from source
    AUTO_CREATE_FRAMES = {"WorldFrame"}

    print("\n[VERIFY] Checking all items exist in source station...")
    for item_cfg in all_items:
        name = item_cfg["name"]
        type_str = item_cfg["type"]
        if type_str == "robot":
            robot = find_robot_in_station(RDK, name)
            assert robot is not None, \
                f"Robot '{name}' not found in source station. Tried: {ROBOT_NAMES}"
        elif type_str == "frame" and name in AUTO_CREATE_FRAMES:
            item = RDK.Item(name, TYPE_MAP[type_str])
            if not item.Valid():
                print(f"  [INFO] '{name}' not in source — will auto-create at origin")
        else:
            item = RDK.Item(name, TYPE_MAP[type_str])
            assert item.Valid(), \
                f"Item '{name}' (type={type_str}) not found in source station"
    print(f"[VERIFY] All {len(all_items)} items confirmed in source")

    # ── Verify subtree sources exist ─────────────────────────────────────
    subtree_data = []  # list of (name, world_pose) tuples
    for st_name in subtrees_from:
        st_item = RDK.Item(st_name, ITEM_TYPE_FRAME)
        if not st_item.Valid():
            st_item = RDK.Item(st_name)
        if not st_item.Valid():
            print(f"\n[ERROR] Subtree source '{st_name}' not found.")
            print(f"  Available frames in station:")
            for f in RDK.ItemList(ITEM_TYPE_FRAME):
                print(f"    - {f.Name()}")
            assert False, f"copy_subtree_from item '{st_name}' not found in source station"
        st_pose = st_item.PoseAbs()
        st_txyz = Pose_2_TxyzRxyz(st_pose)
        print(f"[VERIFY] Subtree root '{st_item.Name()}' found at "
              f"x={st_txyz[0]:.1f} y={st_txyz[1]:.1f} z={st_txyz[2]:.1f}")
        subtree_data.append((st_name, st_pose))

    # ── Set j7=0 on the source robot before subtree copy ─────────────────
    # The robot comes via the subtree copy (e.g. Fanuc R2000iC 125LBase),
    # NOT as a separate Copy/Paste. We just need to set j7=0 so the subtree
    # copies with the robot at the correct rail position.
    robot_src = find_robot_in_station(RDK, ROBOT_NAMES[0])
    if robot_src:
        src_joints = robot_src.Joints()
        try:
            jlist = src_joints.list()
        except AttributeError:
            jlist = list(src_joints)

        if len(jlist) >= 7:
            jlist_j7zero = list(jlist)
            jlist_j7zero[6] = 0.0
            robot_src.setJoints(jlist_j7zero)

        robot_base_world = robot_src.PoseAbs()
        base_txyz = Pose_2_TxyzRxyz(robot_base_world)
        print(f"\n[READ] Robot '{robot_src.Name()}' base at j7=0: "
              f"x={base_txyz[0]:.1f} y={base_txyz[1]:.1f} z={base_txyz[2]:.1f}")

    # Keep a reference to the source station before creating the new one
    src_station_ref = src_station

    # ── Create destination station ───────────────────────────────────────
    print(f"\n[CREATE] Creating new station...")
    dest_station_ref = RDK.AddStation("for_robert_relative_to_base")

    # ── Copy/Paste subtrees ─────────────────────────────────────────────
    for st_name, st_pose in subtree_data:
        # Switch to source to copy the subtree root (brings all children)
        RDK.setActiveStation(src_station_ref)
        st_item = RDK.Item(st_name, ITEM_TYPE_FRAME)
        if not st_item.Valid():
            st_item = RDK.Item(st_name)
        st_item.Copy()

        # Switch to dest and paste
        RDK.setActiveStation(dest_station_ref)
        pasted_subtree = RDK.Paste()
        assert pasted_subtree.Valid(), f"Paste() failed for subtree '{st_name}'"

        # Position at original world pose
        pasted_subtree.setPose(st_pose)

        children = pasted_subtree.Childs()
        print(f"[SUBTREE] Pasted '{pasted_subtree.Name()}' with {len(children)} direct child(ren)")

    # ── Verify robot came via subtree ─────────────────────────────────────
    RDK.setActiveStation(dest_station_ref)
    robots_in_dest = RDK.ItemList(ITEM_TYPE_ROBOT)
    assert len(robots_in_dest) >= 1, \
        "No robot found in destination after subtree paste — check that the robot's parent frame is in copy_subtrees_from"
    robot_dst = robots_in_dest[0]
    try:
        dst_jlist = robot_dst.Joints().list()
    except AttributeError:
        dst_jlist = list(robot_dst.Joints())
    assert len(dst_jlist) == 6, \
        f"Expected 6-DOF robot but got {len(dst_jlist)} joints — rail mechanism may have been included in subtree"
    robot_dst.setJoints([0.0] * 6)
    print(f"[ROBOT] Found 6-DOF robot '{robot_dst.Name()}' in subtree (parent: '{robot_dst.Parent().Name()}')")

    # ── Copy/Paste remaining items one at a time ─────────────────────────
    # We need to switch back to the source station to Copy each item,
    # then switch to dest to Paste it.
    non_robot_items = [i for i in all_items if i["type"] != "robot"]

    if non_robot_items:
        print(f"\n[COPY] Copying {len(non_robot_items)} non-robot item(s)...")

        for i, item_cfg in enumerate(non_robot_items):
            name = item_cfg["name"]
            type_str = item_cfg["type"]
            item_type = TYPE_MAP[type_str]

            # Switch to source
            RDK.setActiveStation(src_station_ref)

            src_item = RDK.Item(name, item_type)

            # Auto-create frames that don't exist in source
            if not src_item.Valid() and type_str == "frame" and name in AUTO_CREATE_FRAMES:
                RDK.setActiveStation(dest_station_ref)
                station_dst = RDK.ActiveStation()
                auto_frame = RDK.AddFrame(name, station_dst)
                # Identity pose = origin with no rotation
                from robodk.robomath import eye
                auto_frame.setPose(eye(4))
                print(f"  [AUTO] Created '{name}' at origin")
                continue

            assert src_item.Valid(), \
                f"Item '{name}' (type={type_str}) not found in source station"

            world_pose = src_item.PoseAbs()
            src_item.Copy()

            # Switch to dest
            RDK.setActiveStation(dest_station_ref)

            pasted = RDK.Paste()
            assert pasted.Valid(), f"Paste() failed for '{name}'"

            # Position at original world pose
            # For frames: setPose places relative to parent (station = world)
            # For objects/targets: setPoseAbs should work
            if type_str == "frame":
                pasted.setPose(world_pose)
            else:
                pasted.setPoseAbs(world_pose)

            if (i + 1) % 10 == 0 or (i + 1) == len(non_robot_items):
                print(f"  [{i+1}/{len(non_robot_items)}] Copied '{name}' ({type_str})")

        # Stay on dest station
        RDK.setActiveStation(dest_station_ref)

    # ── Save ─────────────────────────────────────────────────────────────
    dest_path = to_robodk_path(args.dest)
    print(f"\n[SAVE] Saving to: {dest_path}")
    RDK.Save(dest_path)

    if os.path.exists(args.dest):
        size = os.path.getsize(args.dest)
        print(f"[SAVE] Written: {args.dest} ({size:,} bytes)")
        if size < 5000:
            print(f"[WARN] File is suspiciously small ({size} bytes) — "
                  f"RoboDK free license may have truncated the save.")
    else:
        print(f"[WARN] Save file not found — save may have failed")

    # ── Final verification ───────────────────────────────────────────────
    print(f"\n[VERIFY] Querying destination station...")
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    print(f"  Robots: {[r.Name() for r in robots]}")
    assert len(robots) == 1, f"Expected exactly 1 robot but found {len(robots)}: {[r.Name() for r in robots]}"

    frames = RDK.ItemList(ITEM_TYPE_FRAME)
    print(f"  Frames: {[f.Name() for f in frames]}")
    tools = RDK.ItemList(ITEM_TYPE_TOOL)
    print(f"  Tools:  {[t.Name() for t in tools]}")
    objects = RDK.ItemList(ITEM_TYPE_OBJECT)
    print(f"  Objects: {[o.Name() for o in objects]}")
    targets = RDK.ItemList(ITEM_TYPE_TARGET)
    print(f"  Targets: {len(targets)} target(s)")
    for t in targets:
        print(f"    - {t.Name()}")

    all_station_items = RDK.ItemList()
    print(f"  Total items: {len(all_station_items)}")

    # Verify robot DOF
    for r in robots:
        try:
            nj = len(r.Joints().list())
        except AttributeError:
            nj = len(list(r.Joints()))
        assert nj == 6, f"Robot '{r.Name()}' has {nj} DOF, expected 6"

    # Verify expected counts from config
    expected_objects = [i for i in all_items if i["type"] == "object"]
    expected_targets = [i for i in all_items if i["type"] == "target"]
    expected_frames = [i for i in all_items if i["type"] == "frame"]

    assert len(objects) >= len(expected_objects), \
        f"Expected at least {len(expected_objects)} objects, got {len(objects)}"
    assert len(targets) >= len(expected_targets), \
        f"Expected at least {len(expected_targets)} targets, got {len(targets)}"

    print(f"\n[DONE] Station extraction complete. "
          f"{len(robots)} robot(s), {len(objects)} object(s), "
          f"{len(targets)} target(s), {len(frames)} frame(s), {len(tools)} tool(s).")


if __name__ == "__main__":
    main()
