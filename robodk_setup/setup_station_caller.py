"""Drop this file into RoboDK's Scripts folder once and leave it there.
It calls the actual setup script from the repo so you never need to move it again.

RoboDK Scripts folder is typically:
    C:\RoboDK\Scripts\
"""
import runpy
runpy.run_path(r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robodk_setup\setup_station.py")
