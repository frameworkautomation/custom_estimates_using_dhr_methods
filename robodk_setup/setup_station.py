"""Apply modifications to the currently open RoboDK station and save the result.

IMPORTANT: Open the station manually in RoboDK first before running this script.
    File > Open > robo_dk_saves/TestStationFanuc.rdk

Then run via the caller script:
    Copy robodk_setup/setup_station_caller.py to C:\RoboDK\Scripts\ once,
    then: Tools > Run Script > setup_station_caller
"""
from robodk import *
from robolink import *
import os
import sys
import runpy
import traceback
from datetime import datetime

PROJECT_DIR      = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
SAVED_MODIFIED   = os.path.join(PROJECT_DIR, "robo_dk_saves", "all_dhr_cones_removed.rdk")
LIST_ITEMS       = os.path.join(PROJECT_DIR, "robodk_setup", "list_items.py")
OUTPUT_DIR       = os.path.join(PROJECT_DIR, "robo_dk_output")
ERROR_LOG        = os.path.join(OUTPUT_DIR, "error.txt")
ROBODK_SETUP_DIR = os.path.join(PROJECT_DIR, "robodk_setup")

def write_error(e):
    msg = traceback.format_exc()
    print(msg)
    with open(ERROR_LOG, "w") as f:
        f.write(msg)
    print(f"Error written to: {ERROR_LOG}")

try:
    RDK = Robolink()

    if ROBODK_SETUP_DIR not in sys.path:
        sys.path.insert(0, ROBODK_SETUP_DIR)
    import modifications

    # Check if cones are already gone by looking for matching items
    cone_items = modifications._get_cone_items(RDK)

    if not cone_items:
        print("No cone items found — station already modified or wrong station open.")
        print("Make sure TestStationFanuc.rdk is open in RoboDK.")
    else:
        print(f"Found {len(cone_items)} cone items. Running modifications...")

        runpy.run_path(LIST_ITEMS, init_globals={"RDK": RDK})

        modifications.record_cone_positions(RDK)
        modifications.delete_cones(RDK)

        print(f"Saving to: {SAVED_MODIFIED}")
        RDK.Save(SAVED_MODIFIED)

        if os.path.exists(SAVED_MODIFIED):
            size = os.path.getsize(SAVED_MODIFIED)
            print(f"Save confirmed ({size} bytes): {SAVED_MODIFIED}")
        else:
            print(f"WARNING: file not found after save: {SAVED_MODIFIED}")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary = f"=== Setup Station ===\n\nTimestamp: {timestamp}\n\nCones found: {len(cone_items)}\n"
    with open(os.path.join(OUTPUT_DIR, "setup_station.txt"), "w") as f:
        f.write(summary)
    print(summary)

except Exception as e:
    write_error(e)
