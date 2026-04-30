"""Load the saved TestStationFanuc station into RoboDK.

RoboDK must be open and running before you run this script.

Run via the caller script (recommended):
    Copy robodk_setup/setup_station_caller.py to C:\RoboDK\Scripts\ once,
    then: Tools > Run Script > setup_station_caller

Or run directly from RoboDK's built-in IDE:
    Tools > Run Script > select this file
"""
import os
from robodk.robolink import Robolink

PROJECT_DIR  = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STATION_FILE = os.path.join(PROJECT_DIR, "robo_dk_saves", "TestStationFanuc.rdk")

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
