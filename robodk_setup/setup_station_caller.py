"""Drop this file into RoboDK's Scripts folder once and leave it there.
It calls the actual setup script from the repo so you never need to move it again.

RoboDK Scripts folder is typically:
    C:\RoboDK\Scripts\

One-time setup:
    Copy this file to C:\RoboDK\Scripts\setup_station_caller.py
    Then run it from RoboDK: Tools > Run Script > setup_station_caller
"""
import runpy
runpy.run_path(r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robodk_setup\setup_station.py")
