"""Modifications applied to the RoboDK station after loading.

Functions are called selectively by setup_station.py depending on
whether a pre-modified save already exists.
"""
from robodk import *
from robolink import *
import os
import re
import json
from datetime import datetime

PROJECT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
OUTPUT_DIR  = os.path.join(PROJECT_DIR, "robo_dk_output")

YARN_TRAY_RE = re.compile(r"^Machine\d+YarnTray\d+Slot\d+(Base)?$")


def _get_cone_items(RDK):
    """Return list of (name, item) for all matching cone items."""
    results = []
    for item in RDK.ItemList():
        try:
            if item.Valid():
                name = item.Name()
                if YARN_TRAY_RE.match(name):
                    results.append((name, item))
        except Exception:
            continue
    return results


def record_cone_positions(RDK):
    """Write the absolute pose of every cone item to robo_dk_output/cone_positions.json."""
    cones = _get_cone_items(RDK)

    records = []
    for name, item in cones:
        try:
            pose = item.PoseAbs()
            xyzrpw = Pose_2_xyzrpw(pose)
            records.append({
                "name": name,
                "x": xyzrpw[0],
                "y": xyzrpw[1],
                "z": xyzrpw[2],
                "rx": xyzrpw[3],
                "ry": xyzrpw[4],
                "rz": xyzrpw[5],
            })
        except Exception as e:
            print(f"Could not record pose for {name}: {e}")

    output = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "count": len(records),
        "cones": records,
    }

    filepath = os.path.join(OUTPUT_DIR, "cone_positions.json")
    with open(filepath, "w") as f:
        json.dump(output, f, indent=2)

    print(f"Recorded {len(records)} cone positions to: {filepath}")


def delete_cones(RDK):
    """Delete all matching cone items and log to robo_dk_output/modifications.txt."""
    # Collect names first (handles go stale); re-fetch by name before deleting
    names_to_delete = [name for name, _ in _get_cone_items(RDK)]

    deleted = []
    for name in names_to_delete:
        item = RDK.Item(name)
        if item.Valid():
            item.Delete()
            deleted.append(name)
            print(f"Deleted: {name}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    message = (
        f"=== Modifications ===\n\n"
        f"Timestamp: {timestamp}\n\n"
        f"Deleted {len(deleted)} cone item(s):\n"
    )
    for name in deleted:
        message += f"  {name}\n"

    filepath = os.path.join(OUTPUT_DIR, "modifications.txt")
    with open(filepath, "w") as f:
        f.write(message)

    print(f"Deleted {len(deleted)} items. Log: {filepath}")
