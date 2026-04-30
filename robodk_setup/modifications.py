"""Modifications applied to the RoboDK station after loading.

Called automatically by setup_station.py — do not run standalone.
Logs deleted items to robo_dk_output/modifications.txt.
"""
from robodk import *
from robolink import *
import os
import re
from datetime import datetime

PROJECT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
OUTPUT_DIR  = os.path.join(PROJECT_DIR, "robo_dk_output")

if "RDK" not in globals():
    RDK = Robolink()

# Matches: Machine<N>YarnTray<N>Slot<N>Base (Base optional for naming inconsistencies)
YARN_TRAY_RE = re.compile(r"^Machine\d+YarnTray\d+Slot\d+(Base)?$")

# Collect names first (strings are stable; item handles can go stale between calls)
names_to_delete = []
for item in RDK.ItemList():
    try:
        if item.Valid():
            name = item.Name()
            if YARN_TRAY_RE.match(name):
                names_to_delete.append(name)
    except Exception:
        continue

# Re-fetch each item by name just before deleting
deleted_names = []
for name in names_to_delete:
    item = RDK.Item(name)
    if item.Valid():
        item.Delete()
        deleted_names.append(name)
        print(f"Deleted: {name}")

timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
message = f"=== Modifications ===\n\nTimestamp: {timestamp}\n\nDeleted {len(deleted_names)} yarn tray item(s):\n"
for name in deleted_names:
    message += f"  {name}\n"

print(message)

filepath = os.path.join(OUTPUT_DIR, "modifications.txt")
with open(filepath, "w") as f:
    f.write(message)

print(f"Output saved to: {filepath}")
