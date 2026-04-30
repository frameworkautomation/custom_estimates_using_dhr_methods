"""Print all item names currently in the RoboDK station and save to robo_dk_output/.

Run from RoboDK: Tools > Run Script > list_items
Helps identify the exact names to target in modifications.py.
"""
from robodk import *
from robolink import *
import os
from datetime import datetime

PROJECT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
OUTPUT_DIR  = os.path.join(PROJECT_DIR, "robo_dk_output")

if "RDK" not in globals():
    RDK = Robolink()

itemlist = RDK.ItemList()
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

message = f"=== RoboDK Station Items ===\n\nTimestamp: {timestamp}\n\n"
for item in itemlist:
    try:
        if item.Valid():
            message += f"  [{item.Type()}] {item.Name()}\n"
    except Exception:
        continue
message += f"\nTotal: {len(itemlist)} item(s)"

print(message)

filepath = os.path.join(OUTPUT_DIR, "station_items.txt")
with open(filepath, "w") as f:
    f.write(message)

print(f"Output saved to: {filepath}")
