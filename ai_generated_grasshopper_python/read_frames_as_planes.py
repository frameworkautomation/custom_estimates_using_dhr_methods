# GhPython component: Read scraped RoboDK frames and output as Rhino Planes
#
# Inputs:
#   json_path  (str)   -- path to scraped_frames.json
#                         e.g. r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\scraped_frames.json"
#   group      (str)   -- which group to load: "rail_points", "approach_frames", or "all_frames"
#                         default "rail_points"
#   scale      (float) -- unit scale factor applied to x/y/z (default 1.0 for mm->mm,
#                         use 0.001 to convert mm to metres if your Rhino doc is in metres)
#
# Outputs:
#   planes      -- list of Rhino.Geometry.Plane, one per frame
#   origins     -- list of Rhino.Geometry.Point3d  (plane origins only, useful for display)
#   frame_names -- list of str, frame names in the same order

import json
import Rhino.Geometry as rg

# ── defaults (used if inputs are not wired in GhPython component) ─────────────
try:
    json_path
except NameError:
    json_path = None

try:
    group
except NameError:
    group = None

try:
    scale
except NameError:
    scale = None

# ── read the JSON ─────────────────────────────────────────────────────────────
if not json_path:
    json_path = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\scraped_frames.json"

with open(json_path, "r") as f:
    data = json.load(f)

if group is None or group == "":
    group = "rail_points"

if group not in data:
    available = list(data.keys())
    raise KeyError(f"Group '{group}' not found in JSON. Available groups: {available}")

entries = data[group]

if scale is None:
    scale = 1.0

# ── build planes ──────────────────────────────────────────────────────────────
planes      = []
origins     = []
frame_names = []

for e in entries:
    ox = e["x"] * scale
    oy = e["y"] * scale
    oz = e["z"] * scale

    origin = rg.Point3d(ox, oy, oz)

    xaxis  = rg.Vector3d(e["xaxis"][0], e["xaxis"][1], e["xaxis"][2])
    yaxis  = rg.Vector3d(e["yaxis"][0], e["yaxis"][1], e["yaxis"][2])

    plane = rg.Plane(origin, xaxis, yaxis)

    planes.append(plane)
    origins.append(origin)
    frame_names.append(e["name"])
