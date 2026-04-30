"""Drop this file into RoboDK's Scripts folder once and leave it there.
It loads the saved TestStationFanuc station from the repo into RoboDK.

RoboDK Scripts folder is typically:
    C:\RoboDK\Scripts\

One-time setup:
    Copy this file to C:\RoboDK\Scripts\setup_station_caller.py
    Then run it from RoboDK: Tools > Run Script > setup_station_caller
"""
import os
from robodk.robolink import Robolink

STATION_FILE = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_saves\TestStationFanuc.rdk"

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
