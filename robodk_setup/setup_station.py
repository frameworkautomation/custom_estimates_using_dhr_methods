"""Load the saved TestStationFanuc station into RoboDK, apply modifications, and save.

RoboDK must be open and running before you run this script.

Run via the caller script (recommended):
    Copy robodk_setup/setup_station_caller.py to C:\RoboDK\Scripts\ once,
    then: Tools > Run Script > setup_station_caller

Or run directly from RoboDK's built-in IDE:
    Tools > Run Script > select this file
"""
from robodk import *
from robolink import *
import os
import runpy
from datetime import datetime

PROJECT_DIR   = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STATION_FILE  = os.path.join(PROJECT_DIR, "robo_dk_saves", "TestStationFanuc.rdk")
MODIFICATIONS = os.path.join(PROJECT_DIR, "robodk_setup", "modifications.py")
LIST_ITEMS    = os.path.join(PROJECT_DIR, "robodk_setup", "list_items.py")
OUTPUT_DIR    = os.path.join(PROJECT_DIR, "robo_dk_output")

# Save filename: set to a string for an explicit title, or None to use date-time.
SAVE_TITLE = "all_dhr_cones_removed"

RDK = Robolink()

assert os.path.exists(STATION_FILE), (
    f"Station file not found: {STATION_FILE}\n"
    "Make sure robo_dk_saves/TestStationFanuc.rdk exists in the repo directory."
)

RDK.AddFile(STATION_FILE)
print(f"Loaded station: {STATION_FILE}")

# List all items before modifications
print("Listing station items...")
runpy.run_path(LIST_ITEMS)

# Apply modifications
print("Running modifications...")
runpy.run_path(MODIFICATIONS)

# Save the result to robo_dk_saves/
save_dir = os.path.dirname(STATION_FILE)
if SAVE_TITLE:
    save_name = SAVE_TITLE + ".rdk"
else:
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_name = "TestStationFanuc_" + ts + ".rdk"
save_path = os.path.join(save_dir, save_name)
RDK.Save(save_path)
print(f"Saved station: {save_path}")

# Write summary to robo_dk_output/
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
summary = f"=== Setup Station ===\n\nTimestamp: {timestamp}\n\nLoaded: {STATION_FILE}\nSaved:  {save_path}\n"
print(summary)

filepath = os.path.join(OUTPUT_DIR, "setup_station.txt")
with open(filepath, "w") as f:
    f.write(summary)

print(f"Output saved to: {filepath}")
RDK.ShowMessage(summary + f"\nOutput log:\n{filepath}")
