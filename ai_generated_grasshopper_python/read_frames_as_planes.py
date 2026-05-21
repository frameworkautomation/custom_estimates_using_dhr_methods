# GhPython component (Rhino 8 / CPython 3): RoboDK frames as Rhino Planes
#
# Inputs:
#   yaml_path  (str)   -- path to robodk.yaml (optional, uses default if blank)
#   group      (str)   -- "rail_points" or "approach_frames" (default: "rail_points")
#   scale      (float) -- unit scale for x/y/z (default 1.0 mm; use 0.001 for metres)
#
# Outputs:
#   planes      -- list of Rhino.Geometry.Plane
#   origins     -- list of Rhino.Geometry.Point3d
#   frame_names -- list of str

import sys
sys.path.insert(0, r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\misc_extraction_utils")

from robodk_yaml_reader import load_all_frames, filter_frames
import Rhino.Geometry as rg

try:
    yaml_path
except NameError:
    yaml_path = None

try:
    group
except NameError:
    group = None

try:
    scale
except NameError:
    scale = None

if group is None or group == "":
    group = "rail_points"

if scale is None:
    scale = 1.0

PATTERNS = {
    "rail_points":     r"^OptimizationApproachMachine\d+$",
    "approach_frames": r"CurtainSafe$",
}

if group not in PATTERNS:
    raise KeyError("group must be one of: {}".format(list(PATTERNS.keys())))

all_frames = load_all_frames(yaml_path)
entries    = filter_frames(all_frames, PATTERNS[group])

planes      = []
origins     = []
frame_names = []

for e in entries:
    origin = rg.Point3d(e["x"] * scale, e["y"] * scale, e["z"] * scale)
    xaxis  = rg.Vector3d(e["xaxis"][0], e["xaxis"][1], e["xaxis"][2])
    yaxis  = rg.Vector3d(e["yaxis"][0], e["yaxis"][1], e["yaxis"][2])

    planes.append(rg.Plane(origin, xaxis, yaxis))
    origins.append(origin)
    frame_names.append(e["name"])
