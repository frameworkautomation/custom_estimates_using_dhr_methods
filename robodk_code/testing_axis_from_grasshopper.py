"""
RoboDK Script: Animate the moving part of the end effector between
open and closed positions, with popups at each state.

import_angle  = the angle the STL was modelled at (its natural rest pose)
open_angle    = open position relative to import_angle
closed_angle  = closed position relative to import_angle

The actual RoboDK rotation applied is:
    axis_offset * rotz((import_angle + delta) * pi / 180)
where delta is open_angle or closed_angle.
"""

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL
from robodk.robomath import *
import tkinter as tk
from tkinter import messagebox

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAME  = "Fanuc R2000iC 125L"
TOOL_OPEN   = "pickup_point"   # tool active when gripper is open
TOOL_CLOSED = "pickup_closed"  # tool active when gripper is closed
# ─────────────────────────────────────────────────────────────────────────────


def blocking_popup(title, message):
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    messagebox.showinfo(title, message, parent=root)
    root.destroy()


def main():
    try:
        RDK = Robolink()
        RDK.Item("")  # probe connection
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        RDK = Robolink(robodk_ip="172.23.208.1")

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot '" + ROBOT_NAME + "' not found.")

    def set_tool(name):
        tool = RDK.Item(name, ITEM_TYPE_TOOL)
        if not tool.Valid():
            print("[WARN] Tool '" + name + "' not found — skipping tool swap.")
            return
        robot.setTool(tool)
        print("[INFO] Tool set to '" + name + "'")

    # Find MovingPart by searching for an item whose name starts with "MovingPart|"
    # Angles are encoded in the name: "MovingPart|open=0|closed=9|import=0"
    moving = None
    for item in RDK.ItemList():
        if item.Name().startswith("MovingPart|"):
            moving = item
            break
    if moving is None:
        # Fallback: try exact name
        moving = RDK.Item("MovingPart")
    if not moving.Valid():
        raise RuntimeError("'MovingPart' not found — run the GH script first.")

    # Parse angles from name
    def parse_angles(item):
        name = item.Name()
        parts = name.split("|")
        angles = {}
        for part in parts[1:]:
            key, val = part.split("=")
            angles[key.strip()] = float(val.strip())
        if "open" not in angles or "closed" not in angles or "import" not in angles:
            raise RuntimeError(
                "Could not parse angles from MovingPart name: '" + name + "'. "
                "Re-run the GH script."
            )
        return angles["open"], angles["closed"], angles["import"]

    open_angle, closed_angle, import_angle = parse_angles(moving)

    print("[INFO] open_angle=" + str(open_angle) + " deg")
    print("[INFO] closed_angle=" + str(closed_angle) + " deg")
    print("[INFO] import_angle=" + str(import_angle) + " deg (STL natural pose)")

    # Recover axis_offset by backing out the import_angle rotation
    # that was baked in during GH export:
    #   moving.Pose() = axis_offset * rotz(import_angle)
    #   axis_offset   = moving.Pose() * invH(rotz(import_angle))
    axis_offset = moving.Pose() * invH(rotz(import_angle * pi / 180.0))

    def set_angle(delta_deg):
        """
        Rotate the moving part to (import_angle + delta_deg).
        import_angle cancels out the STL's natural pose,
        delta_deg is the desired angle relative to that.
        """
        total_rad = (import_angle + delta_deg) * pi / 180.0
        moving.setPose(axis_offset * rotz(total_rad))
        RDK.Render()

    # ── Move to open ──────────────────────────────────────────────────────────
    print("[INFO] Setting gripper to open (" + str(open_angle) + " deg relative to import)...")
    set_angle(open_angle)
    set_tool(TOOL_OPEN)

    blocking_popup(
        "Gripper Open",
        "Gripper is at OPEN position.\n\n"
        "import_angle : " + str(import_angle) + " deg\n"
        "open_angle   : " + str(open_angle) + " deg (relative)\n"
        "Total angle  : " + str(import_angle + open_angle) + " deg\n"
        "Tool         : " + TOOL_OPEN + "\n\n"
        "Click OK to move to closed."
    )

    # ── Move to closed ────────────────────────────────────────────────────────
    print("[INFO] Setting gripper to closed (" + str(closed_angle) + " deg relative to import)...")
    set_angle(closed_angle)
    set_tool(TOOL_CLOSED)

    blocking_popup(
        "Gripper Closed",
        "Gripper is at CLOSED position.\n\n"
        "import_angle  : " + str(import_angle) + " deg\n"
        "closed_angle  : " + str(closed_angle) + " deg (relative)\n"
        "Total angle   : " + str(import_angle + closed_angle) + " deg\n"
        "Tool          : " + TOOL_CLOSED + "\n\n"
        "Click OK to return to import (rest) position."
    )

    # ── Return to import (rest) position ──────────────────────────────────────
    print("[INFO] Returning to import angle (" + str(import_angle) + " deg)...")
    set_angle(0.0)   # delta=0 means exactly at import_angle
    set_tool(TOOL_OPEN)

    print("[INFO] Done.")


if __name__ == "__main__":
    main()
