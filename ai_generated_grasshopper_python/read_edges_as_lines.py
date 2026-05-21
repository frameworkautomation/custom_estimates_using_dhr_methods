# GhPython component: Read scraped RoboDK edges and output as Rhino Lines
#
# Inputs:
#   json_path   (str)   -- path to scraped_edges.json
#   group       (str)   -- which group: "rail_edges" or "approach_edges"
#                          default "rail_edges"
#   scale       (float) -- unit scale (default 1.0 = mm; use 0.001 for mm->metres)
#
# Outputs:
#   lines       -- list of Rhino.Geometry.Line
#   edge_names  -- list of str
#   from_pts    -- list of Rhino.Geometry.Point3d
#   to_pts      -- list of Rhino.Geometry.Point3d

import json
import Rhino.Geometry as rg

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

if not json_path:
    json_path = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\scraped_edges.json"

with open(json_path, "r") as f:
    data = json.load(f)

if group is None or group == "":
    group = "rail_edges"

if group not in data:
    available = list(data.keys())
    raise KeyError("Group '{}' not found. Available: {}".format(group, available))

entries = data[group]

if scale is None:
    scale = 1.0

lines      = []
edge_names = []
from_pts   = []
to_pts     = []

for e in entries:
    fx = e["from"]["x"] * scale
    fy = e["from"]["y"] * scale
    fz = e["from"]["z"] * scale

    tx = e["to"]["x"] * scale
    ty = e["to"]["y"] * scale
    tz = e["to"]["z"] * scale

    pt_from = rg.Point3d(fx, fy, fz)
    pt_to   = rg.Point3d(tx, ty, tz)
    line    = rg.Line(pt_from, pt_to)

    lines.append(line)
    from_pts.append(pt_from)
    to_pts.append(pt_to)
    edge_names.append(e["name"])
