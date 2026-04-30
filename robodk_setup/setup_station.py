"""Set up the knitting cell station in RoboDK.

RoboDK must be open and running before you run this script.

Run from RoboDK's built-in IDE:
    Tools > Run Script > select this file

Or from the command line (requires robodk package installed):
    python setup_station.py
"""
import os
import math
from robodk.robolink import Robolink
from robodk.robomath import rotx, roty, rotz, transl

PROJECT_DIR  = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
STEPS_DIR    = os.path.join(PROJECT_DIR, "steps_from_SolidWorks")
CLONES_DIR   = os.path.join(PROJECT_DIR, "clones")

FANUC_ROBOT  = r"C:\RoboDK\Library\Fanuc-R-2000iC-125L.robot"

CELL_LAYOUT  = os.path.join(STEPS_DIR, "atomic-knitting-machine-tending-cell",
                             "V2-adapted-to-Factory-1", "Layout-V2-1-29-2026.step")
SHIMA_STL    = os.path.join(CLONES_DIR, "knitting-machines", "Shima Seiki SWG-XR",
                             "3D Scan", "OBJ", "3DModel", "Shima Seiki SWG-XR.STL")

# MACHINE POSITIONS (mm) relative to world origin, one tuple per machine.
# Update these once you know the layout — check the PDF or the loaded cell layout STEP.
SHIMA_POSITIONS = [
    (0, 0, 0),   # machine 1 — placeholder
    # (x, y, z), # machine 2 — add more as needed
]

RDK = Robolink()
# Remove socket timeout entirely — large STEP files can take several minutes
try:
    RDK.COM.settimeout(None)
except Exception:
    pass


def load_fanuc():
    """Load Fanuc R-2000iC 125L from robodk_setup/."""
    assert os.path.exists(FANUC_ROBOT), (
        "Fanuc robot file not found at: " + FANUC_ROBOT + "\n"
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


def load_shima():
    """Load the Shima Seiki STL for each position in SHIMA_POSITIONS."""
    if not os.path.exists(SHIMA_STL):
        print("ERROR: Shima Seiki STL not found: " + SHIMA_STL)
        return []

    # STL scan axes don't always match RoboDK — rotate 90deg around X to stand it upright.
    # Adjust SHIMA_ORIENTATION if it still looks wrong after running.
    SHIMA_ORIENTATION = rotx(math.pi / 2)

    instances = []
    for i, (x, y, z) in enumerate(SHIMA_POSITIONS):
        obj = RDK.AddFile(SHIMA_STL)
        obj.setName("Shima Seiki SWG-XR " + str(i + 1))
        obj.setPose(transl(x, y, z) * SHIMA_ORIENTATION)
        instances.append(obj)
        print("Loaded Shima Seiki " + str(i + 1) + " at (" + str(x) + ", " + str(y) + ", " + str(z) + ")")
    return instances


def main():
    print("Setting up knitting cell station in RoboDK...")

    robot  = load_fanuc()
    layout = load_cell_layout()   # loads even if robot failed
    shimas = load_shima()

    print("")
    print("Done. Next steps:")
    print("  1. Position the robot and Shima instances to match the cell layout")
    print("  2. Right-click robot > Teach targets for each pick/place position")
    print("  3. Right-click robot > Add program to sequence the moves")
    print("  4. Run the program to get cycle time")


main()
