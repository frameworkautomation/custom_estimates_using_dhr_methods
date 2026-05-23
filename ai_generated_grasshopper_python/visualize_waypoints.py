# GhPython component (Rhino 8 / CPython 3)
#
# Reads all_waypoints.yaml (the amalgamated file produced by amalgamate_waypoints.py)
# and displays every waypoint as a Plane and every edge as a Line in Grasshopper.
#
# Which file to read is determined by robo_dk_output/waypoint_sources.json ("output" key).
# To add more source YAMLs, edit waypoint_sources.json and re-run amalgamate_waypoints.py.
#
# ── What is a waypoint? ───────────────────────────────────────────────────────
# A waypoint is a named robot pose (position + orientation) stored in the YAML.
# Each waypoint has:
#   - A name  (e.g. "base_cone_grab_0", "base_cone_grab_0_approach")
#   - XYZ position in mm, expressed in robot-local coordinates (relative to the
#     robot base frame at j7=0).  Add base_origin to convert to Rhino world space.
#   - RX/RY/RZ orientation in degrees (ZYX Euler: rotate X first, then Y, then Z).
#   - move_type: how the robot moves to this pose.
#       MoveJ  — joint-space move (fast, curved path, used for approach waypoints)
#       MoveL  — linear Cartesian move (straight line, used for final grab/place)
#
# ── What is an edge? ─────────────────────────────────────────────────────────
# An edge is a directed connection between two waypoints that the robot may
# traverse.  Edges are bidirectional in storage (A→B and B→A are separate entries)
# because the robot configuration when entering a zone differs from when exiting.
# Each edge has a `tested` field:
#   null  — not yet tested for collisions in RoboDK
#   true  — tested and collision-free (safe to traverse)
#   false — tested and collides (must not traverse)
#
# ── GH Inputs ────────────────────────────────────────────────────────────────
#   base_origin (Point3d, optional)
#       The robot base origin expressed in Rhino world space (mm).
#       Waypoint XYZ coordinates are robot-local, so they are offset by this
#       point to place them correctly in the Rhino model.
#       Default: world origin (0, 0, 0) — use this if your Rhino robot model
#       sits at the world origin.
#
# ── GH Outputs ───────────────────────────────────────────────────────────────
#   planes        — list of Plane, one per waypoint.
#                   Origin = waypoint XYZ (offset by base_origin).
#                   XAxis/YAxis = orientation from ZYX Euler angles.
#                   Use these to preview approach directions.
#
#   names         — list of str, parallel to `planes`.
#                   Waypoint name, e.g. "base_cone_grab_0_approach".
#
#   move_types    — list of str, parallel to `planes`.
#                   "MoveJ" or "MoveL" — use this to colour-code the planes.
#
#   edge_lines    — list of Line, one per edge in the YAML.
#                   Draws from the origin of the `from` waypoint to the origin
#                   of the `to` waypoint.  Visualise as curves in GH.
#
#   edge_names    — list of str, parallel to `edge_lines`.
#                   "from_name -> to_name" label for each edge.
#
#   edge_statuses — list of str, parallel to `edge_lines`.
#                   "null" / "true" / "false" — use this to colour edges:
#                   grey = untested, green = clear, red = collision.

import math
import Rhino.Geometry as rg

# ── path_config.yaml is the single source of truth ───────────────────────────
_REPO_ROOT = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
_YAML_PATH = _REPO_ROOT + r"\robo_dk_output\path_config.yaml"

# ── Initialise all outputs so GH never sees an unbound name ──────────────────
planes        = []
names         = []
move_types    = []
edge_lines    = []
edge_names    = []
edge_statuses = []

# ── Optional GH input: base_origin ───────────────────────────────────────────
try:
    _base_origin = base_origin  # noqa: F821
except NameError:
    _base_origin = None

# Resolve Guid → Point3d if GH passes an object reference
import System
import scriptcontext as sc
if _base_origin is not None and isinstance(_base_origin, System.Guid):
    _obj = sc.doc.Objects.FindId(_base_origin)
    if _obj:
        _geo = _obj.Geometry
        if isinstance(_geo, rg.Point):
            _base_origin = _geo.Location
        elif hasattr(_geo, 'GetBoundingBox'):
            _base_origin = _geo.GetBoundingBox(True).Center
if _base_origin is not None and not isinstance(_base_origin, rg.Point3d):
    _base_origin = None

_base_x = _base_origin.X if _base_origin is not None else 0.0
_base_y = _base_origin.Y if _base_origin is not None else 0.0
_base_z = _base_origin.Z if _base_origin is not None else 0.0

# ── PyYAML import ─────────────────────────────────────────────────────────────
try:
    import yaml
except ImportError:
    raise ImportError(
        "PyYAML not found. Install it into Rhino 8 Python from an external terminal:\n"
        r'  C:\Users\samst\.rhinocode\py39-rh8\python.exe -m pip install pyyaml'
    )

def _f(wp, key):
    """Extract a float value from a waypoint dict."""
    v = wp.get(key, 0.0)
    try:
        return float(v)
    except (TypeError, ValueError):
        return 0.0


def _plane_from_zyx(ox, oy, oz, rx_deg, ry_deg, rz_deg):
    """Build a Rhino Plane from ZYX Euler angles (degrees).

    Convention matches RoboDK: R = Rz * Ry * Rx.
    Column 0 of R → XAxis, column 1 → YAxis.
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    x_axis = rg.Vector3d(cz*cy,         sz*cy,         -sy)
    y_axis = rg.Vector3d(cz*sy*sx-sz*cx, sz*sy*sx+cz*cx, cy*sx)
    return rg.Plane(rg.Point3d(ox, oy, oz), x_axis, y_axis)


# ── Main ──────────────────────────────────────────────────────────────────────
def _run():
    print(f"Reading: {_YAML_PATH}")
    with open(_YAML_PATH, 'r') as fh:
        data = yaml.safe_load(fh)
    print(f"File read ok")
    print(f"YAML parsed: {type(data)}, keys={list(data.keys()) if isinstance(data, dict) else 'N/A'}")
    if not data:
        print("ERROR: YAML is empty or None")
        return

    # path_config.yaml stores waypoints as a dict (name → attrs), not a list
    raw_wps = data.get('waypoints') or {}
    if isinstance(raw_wps, dict):
        wp_list = [{"name": k, **v} for k, v in raw_wps.items() if isinstance(v, dict)]
    else:
        wp_list = raw_wps  # fallback if already a list
    print(f"Waypoints in YAML: {len(wp_list)}")
    print(f"base_origin offset: ({_base_x}, {_base_y}, {_base_z})")

    name_to_pt = {}  # for edge endpoint lookup

    for wp in wp_list:
        if not isinstance(wp, dict):
            continue
        # Skip joints-only waypoints (home, transport) — no Cartesian pose to draw
        if 'x' not in wp and 'y' not in wp and 'z' not in wp:
            continue
        name      = str(wp.get('name', ''))
        move_type = str(wp.get('move_type', ''))

        ox = _base_x + _f(wp, 'x')
        oy = _base_y + _f(wp, 'y')
        oz = _base_z + _f(wp, 'z')

        plane = _plane_from_zyx(ox, oy, oz, _f(wp, 'rx'), _f(wp, 'ry'), _f(wp, 'rz'))
        planes.append(plane)
        names.append(name)
        move_types.append(move_type)
        name_to_pt[name] = rg.Point3d(ox, oy, oz)

    print(f"Planes built: {len(planes)}")


    for edge in (data.get('edges') or []):
        if not isinstance(edge, dict):
            continue
        from_pt = name_to_pt.get(str(edge.get('from', '')))
        to_pt   = name_to_pt.get(str(edge.get('to',   '')))
        if from_pt is None or to_pt is None:
            continue
        tested = edge.get('tested')
        edge_lines.append(rg.Line(from_pt, to_pt))
        edge_names.append("{} -> {}".format(edge['from'], edge['to']))
        edge_statuses.append('null' if tested is None else str(tested).lower())


try:
    _run()
except Exception as _e:
    import traceback
    raise RuntimeError("visualize_waypoints error:\n{}".format(traceback.format_exc())) from _e
