"""Load the saved TestStationFanuc station into RoboDK, apply modifications, and save.

RoboDK must be open and running before you run this script.

Run via the caller script (recommended):
    Copy robodk_setup/setup_station_caller.py to C:\RoboDK\Scripts\ once,
    then: Tools > Run Script > setup_station_caller

Or run directly from RoboDK's built-in IDE:
    Tools > Run Script > select this file
"""
import os
import runpy
import datetime
from robodk.robolink import Robolink

PROJECT_DIR  = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STATION_FILE = os.path.join(PROJECT_DIR, "robo_dk_saves", "TestStationFanuc.rdk")
MODIFICATIONS = os.path.join(PROJECT_DIR, "robodk_setup", "modifications.py")

# Save filename: set to a string for an explicit title, or None to use date-time.
SAVE_TITLE = "all_dhr_cones_removed"

RDK = Robolink()
try:
    RDK.COM.settimeout(None)
except Exception:
    pass

assert os.path.exists(STATION_FILE), (
    "Station file not found: " + STATION_FILE + "\n"
    "Make sure robo_dk_saves/TestStationFanuc.rdk exists in the repo directory."
)

RDK.AddFile(STATION_FILE)
print("Loaded station: " + STATION_FILE)

# Apply modifications
print("Running modifications...")
runpy.run_path(MODIFICATIONS)

# Save the result
save_dir = os.path.dirname(STATION_FILE)
if SAVE_TITLE:
    save_name = SAVE_TITLE + ".rdk"
else:
    ts = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_name = "TestStationFanuc_" + ts + ".rdk"
save_path = os.path.join(save_dir, save_name)
RDK.Save(save_path)
print("Saved station: " + save_path)
