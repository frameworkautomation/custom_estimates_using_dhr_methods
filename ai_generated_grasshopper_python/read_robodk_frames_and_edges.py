# GhPython component (Rhino 8 / CPython 3): RoboDK frames and edges
#
# Replaces both read_frames_as_planes.py and read_edges_as_lines.py.
#
# Inputs:
#   yaml_path  (str)   -- path to robodk.yaml (optional, uses default if blank)
#   group      (str)   -- one of:
#                           "rail_points"     -- OptimizationApproachMachine* frames
#                           "approach_frames" -- *CurtainSafe frames
#                           "rail_edges"      -- consecutive rail-point connections
#                           "approach_edges"  -- rail-point -> matching CurtainSafe
#                         Default: "rail_points"
#   scale      (float) -- unit scale for x/y/z (default 1.0 mm; use 0.001 for metres)
#
# Outputs (frame groups populate planes/origins/frame_names; edge groups populate the rest):
#   planes      -- list of Rhino.Geometry.Plane  (frame groups only)
#   origins     -- list of Rhino.Geometry.Point3d (frame groups only)
#   frame_names -- list of str                   (frame groups only)
#   lines       -- list of Rhino.Geometry.Line   (edge groups only)
#   edge_names  -- list of str                   (edge groups only)
#   from_pts    -- list of Rhino.Geometry.Point3d (edge groups only)
#   to_pts      -- list of Rhino.Geometry.Point3d (edge groups only)

import sys
sys.path.insert(0, r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\misc_extraction_utils")

from robodk_yaml_reader import load_all_frames, filter_frames
from scrape_edges_to_json import build_rail_edges, build_approach_edges, RAIL_PATTERN, APPROACH_PATTERN
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

FRAME_PATTERNS = {
    "rail_points":     RAIL_PATTERN,
    "approach_frames": APPROACH_PATTERN,
}

EDGE_GROUPS = {"rail_edges", "approach_edges"}
VALID_GROUPS = set(FRAME_PATTERNS) | EDGE_GROUPS

if group not in VALID_GROUPS:
    raise KeyError("group must be one of: {}".format(sorted(VALID_GROUPS)))

# Initialise all outputs so GH never sees unbound names
planes      = []
origins     = []
frame_names = []
lines       = []
edge_names  = []
from_pts    = []
to_pts      = []

all_frames = load_all_frames(yaml_path)

if group in FRAME_PATTERNS:
    entries = filter_frames(all_frames, FRAME_PATTERNS[group])
    for e in entries:
        origin = rg.Point3d(e["x"] * scale, e["y"] * scale, e["z"] * scale)
        xaxis  = rg.Vector3d(e["xaxis"][0], e["xaxis"][1], e["xaxis"][2])
        yaxis  = rg.Vector3d(e["yaxis"][0], e["yaxis"][1], e["yaxis"][2])
        planes.append(rg.Plane(origin, xaxis, yaxis))
        origins.append(origin)
        frame_names.append(e["name"])

else:
    rail_frames_list     = filter_frames(all_frames, RAIL_PATTERN)
    approach_frames_list = filter_frames(all_frames, APPROACH_PATTERN)

    if group == "rail_edges":
        entries = build_rail_edges(rail_frames_list)
    else:  # approach_edges
        entries = build_approach_edges(rail_frames_list, approach_frames_list)

    for e in entries:
        pt_from = rg.Point3d(e["from"]["x"] * scale, e["from"]["y"] * scale, e["from"]["z"] * scale)
        pt_to   = rg.Point3d(e["to"]["x"]   * scale, e["to"]["y"]   * scale, e["to"]["z"]   * scale)
        lines.append(rg.Line(pt_from, pt_to))
        from_pts.append(pt_from)
        to_pts.append(pt_to)
        edge_names.append(e["name"])
