"""
make_gh_wrapper.py

Generates GhPython-compatible scripts from our JSON reader logic.

GhPython import compatibility notes
-------------------------------------
Rhino 7  -- IronPython 2.7
    sys.path.append() works for pure-Python .py files.
    PyYAML (yaml module) is NOT available -- importing robodk_yaml_reader.py
    directly in GhPython would fail because yaml is missing.
    Solution: read the pre-generated JSON files (scraped_frames.json,
    scraped_edges.json) instead of importing the YAML reader.
    This is what read_frames_as_planes.py and read_edges_as_lines.py already do.

Rhino 8  -- CPython 3 (via ScriptEditor or GhPython with CPython backend)
    Full import works. Can sys.path.insert the repo root and import
    robodk_yaml_reader directly. But the JSON approach still works and
    is simpler -- no sys.path magic needed in the component.

Conclusion: the JSON-based GhPython scripts (read_frames_as_planes.py,
read_edges_as_lines.py) are the right approach for both Rhino versions.
Run the scrapers once to generate/update the JSON files, then Grasshopper
reads from those.

This script re-generates those GhPython files from a template if needed.
Run it after changing the JSON schema in the scrapers.

Usage:
    python misc_extraction_utils/make_gh_wrapper.py
    python misc_extraction_utils/make_gh_wrapper.py --list
    python misc_extraction_utils/make_gh_wrapper.py --target frames
    python misc_extraction_utils/make_gh_wrapper.py --target edges
"""

import argparse
import os
import textwrap

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
GH_DIR    = os.path.join(REPO_ROOT, "ai_generated_grasshopper_python")

DEFAULT_FRAMES_JSON = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\scraped_frames.json"
DEFAULT_EDGES_JSON  = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\scraped_edges.json"


# ── Templates ─────────────────────────────────────────────────────────────────

FRAMES_TEMPLATE = '''\
# GhPython component: Read scraped RoboDK frames and output as Rhino Planes
#
# Inputs:
#   json_path  (str)   -- path to scraped_frames.json
#   group      (str)   -- which group: "rail_points", "approach_frames", or "all_frames"
#                         default "rail_points"
#   scale      (float) -- unit scale (default 1.0 = mm; use 0.001 for mm->metres)
#
# Outputs:
#   planes      -- list of Rhino.Geometry.Plane
#   origins     -- list of Rhino.Geometry.Point3d
#   frame_names -- list of str

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
    json_path = r"{default_json}"

with open(json_path, "r") as f:
    data = json.load(f)

if group is None or group == "":
    group = "rail_points"

if group not in data:
    available = list(data.keys())
    raise KeyError("Group '{{}}' not found. Available: {{}}".format(group, available))

entries = data[group]

if scale is None:
    scale = 1.0

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
    plane  = rg.Plane(origin, xaxis, yaxis)

    planes.append(plane)
    origins.append(origin)
    frame_names.append(e["name"])
'''

EDGES_TEMPLATE = '''\
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
    json_path = r"{default_json}"

with open(json_path, "r") as f:
    data = json.load(f)

if group is None or group == "":
    group = "rail_edges"

if group not in data:
    available = list(data.keys())
    raise KeyError("Group '{{}}' not found. Available: {{}}".format(group, available))

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
'''

TARGETS = {
    "frames": {
        "output":   os.path.join(GH_DIR, "read_frames_as_planes.py"),
        "template": FRAMES_TEMPLATE,
        "default":  DEFAULT_FRAMES_JSON,
    },
    "edges": {
        "output":   os.path.join(GH_DIR, "read_edges_as_lines.py"),
        "template": EDGES_TEMPLATE,
        "default":  DEFAULT_EDGES_JSON,
    },
}


def generate(target_name, json_path_override=None):
    t = TARGETS[target_name]
    json_path = json_path_override or t["default"]
    content = t["template"].format(default_json=json_path)
    out = t["output"]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"[OK] Written: {out}")


def parse_args():
    p = argparse.ArgumentParser(description="Generate GhPython component scripts.")
    p.add_argument("--target", choices=list(TARGETS.keys()) + ["all"],
                   default="all", help="Which script to generate (default: all)")
    p.add_argument("--json-path", default=None,
                   help="Override the default JSON path embedded in the script")
    p.add_argument("--list", action="store_true", help="List available targets")
    return p.parse_args()


def main():
    args = parse_args()
    if args.list:
        for name, t in TARGETS.items():
            print(f"  {name:10}  ->  {t['output']}")
        return
    targets = list(TARGETS.keys()) if args.target == "all" else [args.target]
    for name in targets:
        generate(name, args.json_path)


if __name__ == "__main__":
    main()
