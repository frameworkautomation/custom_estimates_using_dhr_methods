"""Load TestStationFanuc into RoboDK, apply modifications, and save.

RoboDK must be open and running before you run this script.

Run via the caller script (recommended):
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

    if ROBODK_SETUP_DIR not in sys.path:
        sys.path.insert(0, ROBODK_SETUP_DIR)
    import modifications

    # Use pre-modified save if it exists and is a real file (not an empty stub)
    if os.path.exists(SAVED_MODIFIED) and os.path.getsize(SAVED_MODIFIED) > 100000:
        load_file = SAVED_MODIFIED
        run_modifications = False
    else:
        load_file = STATION_FILE
        run_modifications = True

    print(f"Loading: {load_file}")
    RDK.AddFile(load_file)

    # Find the station we just loaded and make it the active station for saving
    stations = [s for s in RDK.ItemList(ITEM_TYPE_STATION) if s.Valid()]
    print(f"Open stations: {[s.Name() for s in stations]}")
    if stations:
        RDK.setActiveStation(stations[-1])
        print(f"Active station set to: {stations[-1].Name()}")

    if run_modifications:
        print("Listing station items...")
        runpy.run_path(LIST_ITEMS, init_globals={"RDK": RDK})

        print("Recording cone positions...")
        modifications.record_cone_positions(RDK)

        print("Deleting cones...")
        modifications.delete_cones(RDK)

        print(f"Saving to: {SAVED_MODIFIED}")
        RDK.Save(SAVED_MODIFIED)

        size = os.path.getsize(SAVED_MODIFIED) if os.path.exists(SAVED_MODIFIED) else 0
        print(f"Save result: {size} bytes at {SAVED_MODIFIED}")
    else:
        print("Loaded pre-modified station, skipping modifications.")

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary = f"=== Setup Station ===\n\nTimestamp: {timestamp}\nLoaded: {load_file}\nModifications run: {run_modifications}\n"
    with open(os.path.join(OUTPUT_DIR, "setup_station.txt"), "w") as f:
        f.write(summary)
    print(summary)

except Exception as e:
    write_error(e)
