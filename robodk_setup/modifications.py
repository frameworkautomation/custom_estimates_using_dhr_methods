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

if "RDK" not in dir():
    RDK = Robolink()

# Matches all yarn tray items anywhere in the station (Machine, Rack, Cart, etc.)
YARN_TRAY_RE = re.compile(r"YarnTray")

# Collect first, then delete (avoid mutating list while iterating)
# item.Valid() guards against invalid handles that ItemList() can include
to_delete = [item for item in RDK.ItemList() if item.Valid() and YARN_TRAY_RE.match(item.Name())]

deleted_names = []
for item in to_delete:
    name = item.Name()
    print(f"Deleting: {name}")
    item.Delete()
    deleted_names.append(name)

timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
message = f"=== Modifications ===\n\nTimestamp: {timestamp}\n\nDeleted {len(deleted_names)} yarn tray item(s):\n"
for name in deleted_names:
    message += f"  {name}\n"

print(message)

filepath = os.path.join(OUTPUT_DIR, "modifications.txt")
with open(filepath, "w") as f:
    f.write(message)

print(f"Output saved to: {filepath}")
