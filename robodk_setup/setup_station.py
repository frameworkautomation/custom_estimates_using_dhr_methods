"""Load TestStationFanuc into RoboDK and apply modifications.

NOTE ON SAVING: RoboDK's free license limits stations to 1 robot.
TestStationFanuc contains a Fanuc robot + linear rail (2 robots), which
prevents saving via the API. All progress is tracked in steps.json instead.

RoboDK must be open and running before you run this script.

Run via the caller script:
    Copy robodk_setup/setup_station_caller.py to C:\RoboDK\Scripts\ once,
    then: Tools > Run Script > setup_station_caller
"""
from robodk import *
from robolink import *
import os
import sys
import json
import traceback
from datetime import datetime

PROJECT_DIR      = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STATION_FILE     = os.path.join(PROJECT_DIR, "robo_dk_saves", "TestStationFanuc.rdk")
OUTPUT_DIR       = os.path.join(PROJECT_DIR, "robo_dk_output")
STEPS_FILE       = os.path.join(OUTPUT_DIR, "steps.json")
ERROR_LOG        = os.path.join(OUTPUT_DIR, "error.txt")
ROBODK_SETUP_DIR = os.path.join(PROJECT_DIR, "robodk_setup")


def load_steps():
    if os.path.exists(STEPS_FILE):
        with open(STEPS_FILE) as f:
            return json.load(f)
    return {"station_loaded": False, "cone_positions_recorded": False, "cones_deleted": False}


def save_steps(steps):
    steps["last_updated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(STEPS_FILE, "w") as f:
        json.dump(steps, f, indent=2)
    print(f"Steps saved: {steps}")


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

    steps = load_steps()

    # --- Step 1: Load station ---
    if not steps["station_loaded"]:
        print(f"Loading station: {STATION_FILE}")
        RDK.AddFile(STATION_FILE)
        stations = [s for s in RDK.ItemList(ITEM_TYPE_STATION) if s.Valid()]
        if stations:
            RDK.setActiveStation(stations[-1])
            print(f"Active station: {stations[-1].Name()}")
        steps["station_loaded"] = True
        # Reset downstream steps since we loaded a fresh station
        steps["cones_deleted"] = False
        save_steps(steps)
    else:
        print("Station already loaded this session, skipping.")

    # --- Step 2: Record cone positions (only once — don't overwrite good data) ---
    if not steps["cone_positions_recorded"]:
        cone_items = modifications._get_cone_items(RDK)
        if cone_items:
            print(f"Recording positions of {len(cone_items)} cones...")
            modifications.record_cone_positions(RDK)
            steps["cone_positions_recorded"] = True
            save_steps(steps)
        else:
            print("No cones found to record.")
    else:
        print("Cone positions already recorded, skipping.")

    # --- Step 3: Delete cones (check live — station can't be saved) ---
    cone_items = modifications._get_cone_items(RDK)
    if cone_items:
        print(f"Deleting {len(cone_items)} cones...")
        modifications.delete_cones(RDK)
        steps["cones_deleted"] = True
        save_steps(steps)
    else:
        print("Cones already deleted this session, skipping.")
        steps["cones_deleted"] = True
        save_steps(steps)

except Exception as e:
    write_error(e)
