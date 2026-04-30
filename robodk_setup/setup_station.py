"""Set up the knitting cell station in RoboDK.

RoboDK must be open and running before you run this script.

Run from RoboDK's built-in IDE:
    Tools > Run Script > select this file

Or from the command line (requires robodk package installed):
    python setup_station.py
"""
import os
from robodk.robolink import Robolink

PROJECT_DIR  = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STEPS_DIR    = os.path.join(PROJECT_DIR, "steps_from_SolidWorks")
CLONES_DIR   = os.path.join(PROJECT_DIR, "clones")

ROBODK_SETUP = os.path.join(PROJECT_DIR, "robodk_setup")
FANUC_ROBOT  = os.path.join(ROBODK_SETUP, "Fanuc R-2000iC 125L.robot")

CELL_LAYOUT  = os.path.join(STEPS_DIR, "atomic-knitting-machine-tending-cell",
                             "V2-adapted-to-Factory-1", "Layout-V2-1-29-2026.step")
SHIMA_STL    = os.path.join(CLONES_DIR, "knitting-machines", "Shima Seiki SWG-XR",
                             "3D Scan", "OBJ", "3DModel", "Shima Seiki SWG-XR.STL")

RDK = Robolink()
# Large STEP files take a long time to import — increase the socket timeout
RDK.COM.settimeout(300)


def load_fanuc():
    """Load Fanuc R-2000iC 125L from robodk_setup/."""
    assert os.path.exists(FANUC_ROBOT), (
        "Fanuc robot file not found at: " + FANUC_ROBOT + "\n"
        "Copy 'Fanuc R-2000iC 125L.robot' from your Downloads into the robodk_setup/ folder.\n"
        "Download it from RoboDK: File > Open Online Library > search 'R-2000iC 125L' > download."
    )
    robot = RDK.AddFile(FANUC_ROBOT)
    robot.setName("Fanuc R-2000iC 125L")
    print("Loaded robot: " + FANUC_ROBOT)
    return robot


def load_cell_layout():
    """Load the V2 factory cell layout as static geometry."""
    if not os.path.exists(CELL_LAYOUT):
        print("ERROR: Cell layout STEP not found: " + CELL_LAYOUT)
        print("Run the Rhino conversion first.")
        return None
    layout = RDK.AddFile(CELL_LAYOUT)
    layout.setName("Cell Layout V2")
    print("Loaded cell layout: " + CELL_LAYOUT)
    return layout


def load_shima(count=1):
    """Load the Shima Seiki STL and place count instances of it."""
    if not os.path.exists(SHIMA_STL):
        print("ERROR: Shima Seiki STL not found: " + SHIMA_STL)
        return []

    instances = []
    for i in range(count):
        obj = RDK.AddFile(SHIMA_STL)
        obj.setName("Shima Seiki SWG-XR " + str(i + 1))
        instances.append(obj)
        print("Loaded Shima Seiki instance " + str(i + 1))
    return instances


def main():
    print("Setting up knitting cell station in RoboDK...")

    robot  = load_fanuc()
    layout = load_cell_layout()   # loads even if robot failed
    shimas = load_shima(count=1)  # change count to match number of machines in your cell

    print("")
    print("Done. Next steps:")
    print("  1. Position the robot and Shima instances to match the cell layout")
    print("  2. Right-click robot > Teach targets for each pick/place position")
    print("  3. Right-click robot > Add program to sequence the moves")
    print("  4. Run the program to get cycle time")


main()
