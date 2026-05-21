# GhPython component (Rhino 8 / CPython 3)
#
# Reads base_cone_waypoints.yaml and displays every waypoint as a Plane and
# every edge (connection between waypoints) as a Line in Grasshopper.
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

import re
import math
import Rhino.Geometry as rg

# ── YAML path (no input needed — always reads from the repo output folder) ────
_YAML_PATH = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\base_cone_waypoints.yaml"

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

# ── Inline-key parsing ────────────────────────────────────────────────────────
# The YAML writes x/y/z on one line: "    x: 0.0  y: 0.0  z: 500.0"
# PyYAML parses that as a single string value for key 'x', not three keys.
# We extract numbers with regex instead.
_XYZ_RE = re.compile(
    r'x\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
    r'[,\s]+'
    r'y\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
    r'[,\s]+'
    r'z\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
)
_RXRYRZ_RE = re.compile(
    r'rx\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
    r'[,\s]+'
    r'ry\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
    r'[,\s]+'
    r'rz\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)'
)
_KEY_RE = re.compile(
    r'^\s*(x|y|z|rx|ry|rz)\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*$'
)


def _extract_floats(block, keys):
    """Pull named float values out of a raw text block.

    Handles both "x: 1.0  y: 2.0  z: 3.0" on one line and one key per line.
    Returns {key: float} for whichever keys are found.
    """
    result = {}
    m = _XYZ_RE.search(block)
    if m and {'x','y','z'} <= keys:
        result['x'], result['y'], result['z'] = float(m.group(1)), float(m.group(2)), float(m.group(3))
    m = _RXRYRZ_RE.search(block)
    if m and {'rx','ry','rz'} <= keys:
        result['rx'], result['ry'], result['rz'] = float(m.group(1)), float(m.group(2)), float(m.group(3))
    for line in block.splitlines():
        m = _KEY_RE.match(line)
        if m and m.group(1) in keys and m.group(1) not in result:
            result[m.group(1)] = float(m.group(2))
    return result


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
    with open(_YAML_PATH, 'r') as fh:
        raw = fh.read()

    data = yaml.safe_load(raw)
    if not data:
        return

    # Build per-waypoint raw text blocks for inline-key extraction
    wp_block_map = {}
    for block in re.split(r'(?=\n  - name:)', raw):
        m = re.search(r'name\s*:\s*(\S+)', block)
        if m:
            wp_block_map[m.group(1)] = block

    name_to_pt = {}  # for edge endpoint lookup

    for wp in (data.get('waypoints') or []):
        if not isinstance(wp, dict):
            continue
        name      = str(wp.get('name', ''))
        move_type = str(wp.get('move_type', ''))
        block     = wp_block_map.get(name, '')

        def _f(key):
            v = wp.get(key)
            if v is not None:
                try: return float(v)
                except (TypeError, ValueError): pass
            return _extract_floats(block, {key}).get(key, 0.0)

        ox = _base_x + _f('x')
        oy = _base_y + _f('y')
        oz = _base_z + _f('z')

        plane = _plane_from_zyx(ox, oy, oz, _f('rx'), _f('ry'), _f('rz'))
        planes.append(plane)
        names.append(name)
        move_types.append(move_type)
        name_to_pt[name] = rg.Point3d(ox, oy, oz)

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
