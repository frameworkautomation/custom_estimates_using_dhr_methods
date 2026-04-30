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
import sys
import runpy
import traceback
from datetime import datetime

PROJECT_DIR      = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STATION_FILE     = os.path.join(PROJECT_DIR, "robo_dk_saves", "TestStationFanuc.rdk")
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

    # Add robodk_setup/ to path so modifications.py can be imported as a module
    if ROBODK_SETUP_DIR not in sys.path:
        sys.path.insert(0, ROBODK_SETUP_DIR)
    import modifications

    if os.path.exists(SAVED_MODIFIED):
        # Fast path: pre-modified station already exists, skip modifications
        print(f"Pre-modified station found, loading directly: {SAVED_MODIFIED}")
        RDK.AddFile(SAVED_MODIFIED)
    else:
        # Slow path: load base station, record cone positions, delete cones, save
        assert os.path.exists(STATION_FILE), (
            f"Station file not found: {STATION_FILE}\n"
            "Make sure robo_dk_saves/TestStationFanuc.rdk exists in the repo directory."
        )

        print(f"Loading base station: {STATION_FILE}")
        RDK.AddFile(STATION_FILE)

        print("Listing station items...")
        runpy.run_path(LIST_ITEMS, init_globals={"RDK": RDK})

        print("Recording cone positions...")
        modifications.record_cone_positions(RDK)

        print("Deleting cones...")
        modifications.delete_cones(RDK)

        print(f"Saving modified station: {SAVED_MODIFIED}")
        save_dir = os.path.dirname(SAVED_MODIFIED)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        RDK.Save(SAVED_MODIFIED)
        if os.path.exists(SAVED_MODIFIED):
            print(f"Save confirmed: {SAVED_MODIFIED}")
        else:
            print(f"WARNING: RDK.Save() returned but file not found: {SAVED_MODIFIED}")
            # Try saving the active station item directly
            station = RDK.ActiveStation()
            print(f"Active station: {station.Name()}")
            station.Save(SAVED_MODIFIED)
            print(f"Station.Save() called. File exists: {os.path.exists(SAVED_MODIFIED)}")

    # Write run summary
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary = (
        f"=== Setup Station ===\n\n"
        f"Timestamp: {timestamp}\n\n"
        f"Loaded: {SAVED_MODIFIED if os.path.exists(SAVED_MODIFIED) else STATION_FILE}\n"
    )
    with open(os.path.join(OUTPUT_DIR, "setup_station.txt"), "w") as f:
        f.write(summary)
    print(summary)

except Exception as e:
    write_error(e)
