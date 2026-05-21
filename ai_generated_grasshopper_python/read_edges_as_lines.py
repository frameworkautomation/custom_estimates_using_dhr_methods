# GhPython component (Rhino 8 / CPython 3): RoboDK edges as Rhino Lines
#
# Inputs:
#   yaml_path  (str)   -- path to robodk.yaml (optional, uses default if blank)
#   group      (str)   -- "rail_edges" or "approach_edges" (default: "rail_edges")
#   scale      (float) -- unit scale for x/y/z (default 1.0 mm; use 0.001 for metres)
#
# Outputs:
#   lines       -- list of Rhino.Geometry.Line
#   edge_names  -- list of str
#   from_pts    -- list of Rhino.Geometry.Point3d
#   to_pts      -- list of Rhino.Geometry.Point3d

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
    group = "rail_edges"

if scale is None:
    scale = 1.0

all_frames      = load_all_frames(yaml_path)
rail_frames     = filter_frames(all_frames, RAIL_PATTERN)
approach_frames = filter_frames(all_frames, APPROACH_PATTERN)

if group == "rail_edges":
    entries = build_rail_edges(rail_frames)
elif group == "approach_edges":
    entries = build_approach_edges(rail_frames, approach_frames)
else:
    raise KeyError("group must be 'rail_edges' or 'approach_edges'")

lines      = []
edge_names = []
from_pts   = []
to_pts     = []

for e in entries:
    pt_from = rg.Point3d(e["from"]["x"] * scale, e["from"]["y"] * scale, e["from"]["z"] * scale)
    pt_to   = rg.Point3d(e["to"]["x"]   * scale, e["to"]["y"]   * scale, e["to"]["z"]   * scale)

    lines.append(rg.Line(pt_from, pt_to))
    from_pts.append(pt_from)
    to_pts.append(pt_to)
    edge_names.append(e["name"])
