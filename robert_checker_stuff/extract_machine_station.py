"""
Extract a minimal station from the full DHR station using a config file.

Copies items listed in the config (with children) into a new station.
Preserves mechanism linkages (robot-rail coupling survives copy/paste).
Optionally filters out children whose names contain a specified string.

Usage:
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1 --config my_config.json
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1 --dest my_output.rdk

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_STATION

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "machine_extract_config.json")
DEFAULT_DEST = os.path.join(REPO_ROOT, "robo_dk_saves", "rdk_machine_reach_testing.rdk")


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


def wsl_to_win(path):
    if path.startswith("/mnt/"):
        drive = path[5]
        return f"{drive.upper()}:{path[6:]}".replace("/", "\\")
    return path


def remove_matching_children(item, exclude_str):
    """Recursively remove children whose names contain exclude_str (case-insensitive)."""
    exclude_lower = exclude_str.lower()
    for child in item.Childs():
        if exclude_lower in child.Name().lower():
            print(f"  [REMOVE] {child.Name()}")
            child.Delete()
        else:
            remove_matching_children(child, exclude_str)


def main():
    parser = argparse.ArgumentParser(
        description="Extract a minimal station from config."
    )
    parser.add_argument("--robodk-ip", default=None)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--dest", default=DEFAULT_DEST)
    args = parser.parse_args()

    # Load config
    assert os.path.exists(args.config), f"Config not found: {args.config}"
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    station_name = config.get("station_name", "ExtractedStation")
    items_cfg = config["items"]

    RDK = connect(args.robodk_ip)

    # Clean up any leftover station with the same name
    for s in RDK.ItemList(ITEM_TYPE_STATION):
        if s.Name() == station_name:
            s.Delete()
            print(f"[CLEAN] Deleted leftover '{station_name}'")

    source = RDK.ActiveStation()
    assert source.Valid(), "No active station"
    print(f"[INFO] Source: {source.Name()}")
    print(f"[INFO] {len(items_cfg)} items to extract\n")

    # Verify all items exist
    for item_cfg in items_cfg:
        name = item_cfg["name"]
        item = RDK.Item(name)
        if item.Valid():
            print(f"[FOUND] {name}")
        else:
            print(f"[WARN] {name} NOT FOUND — will skip")

    # Create destination station
    dest = RDK.AddStation(station_name)
    print(f"\n[INFO] Created '{station_name}'")

    # Copy each item: switch to source, find, copy, switch to dest, paste
    for item_cfg in items_cfg:
        name = item_cfg["name"]
        exclude = item_cfg.get("exclude_containing")

        RDK.setActiveStation(source)
        item = RDK.Item(name)
        if not item.Valid():
            print(f"[SKIP] {name}")
            continue

        item.Copy()
        RDK.setActiveStation(dest)
        pasted = dest.Paste()

        if not pasted.Valid():
            print(f"[FAIL] Could not paste {name}")
            continue

        print(f"[COPY] {name}")

        if exclude:
            remove_matching_children(pasted, exclude)

    # Save
    RDK.setActiveStation(dest)
    win_dest = wsl_to_win(os.path.abspath(args.dest))
    # CHECK_LATER: Save may fail without paid license on multi-robot stations.
    RDK.Save(win_dest)
    print(f"\n[DONE] Saved to {args.dest}")


if __name__ == "__main__":
    main()
