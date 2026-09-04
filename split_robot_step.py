"""
Rhino Python script to split the Fanuc R-2000iC 125L STEP into per-link STL files.

Run inside Rhino:
  1. Open Rhino
  2. File > Open > fanuc-r-2000ic-125l-1.snapshot.36/FANUC R-2000iC 125L.STEP
  3. Tools > PythonScript > Run > select this file

The script will prompt you to select each link's geometry one at a time.
Each selection is exported as a watertight STL to robot_urdf/meshes_grabcad/.

Link order:
  1. base (stationary base, doesn't rotate)
  2. j1 (turntable/shoulder, rotates around vertical Z axis)
  3. j2 (upper arm, big link going up)
  4. j3 (forearm, the long horizontal link)
  5. j4 (wrist roll, thin cylindrical section)
  6. j5 (wrist pitch, small joint)
  7. j6 (flange, the end plate)

Select ALL surfaces/polysurfaces that belong to each link, then press Enter.
"""

import rhinoscriptsyntax as rs
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else rs.DocumentPath()
if not SCRIPT_DIR:
    SCRIPT_DIR = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"

OUT_DIR = os.path.join(SCRIPT_DIR, "robot_urdf", "meshes_grabcad")
if not os.path.exists(OUT_DIR):
    os.makedirs(OUT_DIR)

LINKS = ["base", "j1", "j2", "j3", "j4", "j5", "j6"]
DESCRIPTIONS = [
    "BASE - stationary base (bottom, doesn't rotate)",
    "J1 - turntable/shoulder (rotates around vertical axis)",
    "J2 - upper arm (big link going up from shoulder)",
    "J3 - forearm (long horizontal link + wrist housing)",
    "J4 - wrist roll (thin cylindrical section)",
    "J5 - wrist pitch (small joint between J4 and flange)",
    "J6 - flange (end plate / tool mount)",
]

print("\n" + "=" * 60)
print("  Fanuc R-2000iC 125L — Split into per-link STL")
print("  Output: {}".format(OUT_DIR))
print("=" * 60)

for link_name, desc in zip(LINKS, DESCRIPTIONS):
    print("\n--- Select: {} ---".format(desc))
    print("    Select all surfaces/polysurfaces for this link, then press Enter.")
    print("    Press Escape to skip this link.\n")

    objs = rs.GetObjects(
        "Select geometry for {} ({})".format(link_name, desc),
        filter=8 + 16,  # surfaces + polysurfaces
        preselect=False,
        select=True,
    )

    if not objs:
        print("  SKIPPED: {}".format(link_name))
        continue

    # Join into single mesh and export
    rs.UnselectAllObjects()
    rs.SelectObjects(objs)

    stl_path = os.path.join(OUT_DIR, "{}.stl".format(link_name))

    # Export selected as STL
    # Use Rhino command with settings for binary STL
    cmd = '-_Export "{}" _Enter _Enter'.format(stl_path)
    rs.Command(cmd)

    rs.UnselectAllObjects()

    if os.path.exists(stl_path):
        size = os.path.getsize(stl_path)
        print("  EXPORTED: {} ({:,} bytes)".format(stl_path, size))
    else:
        print("  FAILED: {} not created".format(stl_path))

print("\n" + "=" * 60)
print("  Done! STLs saved to: {}".format(OUT_DIR))
print("  Use with swept_volume.py --mesh-dir robot_urdf/meshes_grabcad")
print("=" * 60)
