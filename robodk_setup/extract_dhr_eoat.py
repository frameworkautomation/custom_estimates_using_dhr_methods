"""Export the MaintenanceGripper from a running RoboDK station as STL.

Run this while RoboDK is open with TestStationFanuc loaded.

Three ways to run:
  1. RoboDK:  Tools > Run Script > extract_dhr_eoat
  2. Rhino:   -_RunPythonScript (C:\...\robodk_setup\extract_dhr_eoat.py)
  3. Shell:   python robodk_setup/extract_dhr_eoat.py

Output: extracted_assets/dhr_end_effector/MaintenanceGripper.stl
Skipped if that folder already contains any files.
"""
import sys
sys.path.append(r"C:\RoboDK\Python")

from robodk.robolink import Robolink
import os

PROJECT_DIR      = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
OUT_DIR          = os.path.join(PROJECT_DIR, "extracted_assets", "dhr_end_effector")
OUT_STL          = os.path.join(OUT_DIR, "MaintenanceGripper.stl")

if not os.path.exists(OUT_DIR):
    os.makedirs(OUT_DIR)

existing = [f for f in os.listdir(OUT_DIR) if not f.startswith(".")]
if existing:
    print("DHR end effector already extracted, skipping.")
else:
    RDK = Robolink()
    item = RDK.Item("MaintenanceGripper")
    if not item.Valid():
        print("ERROR: MaintenanceGripper not found in station. Is TestStationFanuc loaded?")
    else:
        print(f"Exporting MaintenanceGripper to: {OUT_STL}")
        item.Export(OUT_STL)
        if os.path.exists(OUT_STL):
            print("  -> OK")
        else:
            print("  FAILED (no output file)")
