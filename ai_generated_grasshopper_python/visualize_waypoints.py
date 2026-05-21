"""
GhPython component — Rhino 8 CPython
Reads base_cone_waypoints.yaml and outputs waypoints as Planes and edges as Lines.

GH Inputs:
    yaml_path   (str, optional)    — path to YAML file
    base_origin (Point3d, optional)— robot base origin; if set, coords are robot-local
    scale       (float, optional)  — unit scale multiplier, default 1.0

GH Outputs:
    planes        — list of Rhino.Geometry.Plane, one per waypoint
    origins       — list of Rhino.Geometry.Point3d, one per waypoint
    names         — list of str, waypoint names
    move_types    — list of str, move_type per waypoint
    edge_lines    — list of Rhino.Geometry.Line, one per edge
    edge_names    — list of str, "from -> to" per edge
    edge_statuses — list of str, tested field ("null", "true", "false")
"""

import sys
import re
import math

# ---------------------------------------------------------------------------
# Initialise all outputs so GH never sees an unbound name
# ---------------------------------------------------------------------------
planes        = []
origins       = []
names         = []
move_types    = []
edge_lines    = []
edge_names    = []
edge_statuses = []

# ---------------------------------------------------------------------------
# Handle GH input defaults (NameError means the input socket is unconnected)
# ---------------------------------------------------------------------------
try:
    _yaml_path = yaml_path  # noqa: F821
except NameError:
    _yaml_path = None

if not _yaml_path:
    _yaml_path = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\base_cone_waypoints.yaml"

try:
    _base_origin = base_origin  # noqa: F821
except NameError:
    _base_origin = None

try:
    _scale = float(scale)  # noqa: F821
except NameError:
    _scale = 1.0
except (TypeError, ValueError):
    _scale = 1.0

# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
try:
    import yaml
except ImportError:
    raise ImportError(
        "PyYAML not found. Install it into Rhino 8 Python:\n"
        r'  C:\Users\samst\.rhinocode\py39-rh8\python.exe -m pip install pyyaml'
    )

import Rhino.Geometry as rg  # noqa: E402  (available in GhPython)

# ---------------------------------------------------------------------------
# Helper: parse a 'x: V  y: V  z: V' style line with regex
#   PyYAML treats 'x: 0.0  y: 1.0  z: 2.0' as a string value for key 'x',
#   so we handle it ourselves via regex scanning of the raw text.
# ---------------------------------------------------------------------------
_XYZ_RE  = re.compile(
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

# Individual key patterns used when keys are on separate lines
_KEY_RE = re.compile(
    r'^\s*(x|y|z|rx|ry|rz)\s*:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*$'
)


def _extract_floats_from_text(text_block, keys):
    """
    Extract named float values from a block of text lines.
    Handles both 'key: val  key2: val2' on one line and one-key-per-line.
    Returns a dict {key: float} for the keys found.
    """
    result = {}

    # First pass: try combined-line regex for xyz and rxryrz
    xyz_m = _XYZ_RE.search(text_block)
    if xyz_m and 'x' in keys and 'y' in keys and 'z' in keys:
        result['x'] = float(xyz_m.group(1))
        result['y'] = float(xyz_m.group(2))
        result['z'] = float(xyz_m.group(3))

    rxryrz_m = _RXRYRZ_RE.search(text_block)
    if rxryrz_m and 'rx' in keys and 'ry' in keys and 'rz' in keys:
        result['rx'] = float(rxryrz_m.group(1))
        result['ry'] = float(rxryrz_m.group(2))
        result['rz'] = float(rxryrz_m.group(3))

    # Second pass: individual lines for anything not yet found
    for line in text_block.splitlines():
        m = _KEY_RE.match(line)
        if m:
            k = m.group(1)
            if k in keys and k not in result:
                result[k] = float(m.group(2))

    return result


# ---------------------------------------------------------------------------
# Helper: build Rhino Plane from ZYX Euler angles (degrees)
#   Convention: R = Rz * Ry * Rx  (rotation applied X first, then Y, then Z)
# ---------------------------------------------------------------------------
def _plane_from_zyx_euler(ox, oy, oz, rx_deg, ry_deg, rz_deg):
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    # R = Rz * Ry * Rx
    # Column 0 (XAxis)
    x0 = cz * cy
    x1 = sz * cy
    x2 = -sy
    # Column 1 (YAxis)
    y0 = cz * sy * sx - sz * cx
    y1 = sz * sy * sx + cz * cx
    y2 = cy * sx
    # Column 2 (ZAxis) — not needed for Plane constructor but kept for reference
    # z0 = cz * sy * cx + sz * sx
    # z1 = sz * sy * cx - cz * sx
    # z2 = cy * cx

    origin = rg.Point3d(ox, oy, oz)
    x_axis = rg.Vector3d(x0, x1, x2)
    y_axis = rg.Vector3d(y0, y1, y2)
    return rg.Plane(origin, x_axis, y_axis)


# ---------------------------------------------------------------------------
# Main logic
# ---------------------------------------------------------------------------
def _run():
    # Read raw text (needed for inline key parsing) and also parse via yaml
    with open(_yaml_path, 'r') as fh:
        raw_text = fh.read()

    data = yaml.safe_load(raw_text)
    if data is None:
        return

    # --- Base origin offset ---
    base_x = _base_origin.X if _base_origin is not None else 0.0
    base_y = _base_origin.Y if _base_origin is not None else 0.0
    base_z = _base_origin.Z if _base_origin is not None else 0.0

    # --- Waypoints ---
    # Build a name -> Point3d lookup for edge endpoint resolution
    name_to_origin = {}

    wp_list = data.get('waypoints') or []
    # Rebuild per-waypoint raw text blocks for inline-key extraction.
    # Strategy: split raw_text into blocks by lines starting with '  - name:'
    # (two-space indent list item).
    wp_blocks = re.split(r'(?=\n  - name:)', raw_text)

    # Build a map from waypoint name to its raw block
    wp_block_map = {}
    for block in wp_blocks:
        nm = re.search(r'name\s*:\s*(\S+)', block)
        if nm:
            wp_block_map[nm.group(1)] = block

    for wp in wp_list:
        if not isinstance(wp, dict):
            continue
        name = str(wp.get('name', ''))
        move_type = str(wp.get('move_type', ''))

        # Extract x,y,z,rx,ry,rz — try the parsed dict first (works when keys
        # are on individual lines), fall back to regex on the raw block.
        def _get(key, default=0.0):
            val = wp.get(key)
            if val is not None:
                try:
                    return float(val)
                except (TypeError, ValueError):
                    pass
            # Fall back: regex scan the raw block for this waypoint
            block = wp_block_map.get(name, '')
            found = _extract_floats_from_text(block, {key})
            return found.get(key, default)

        x  = _get('x')  * _scale
        y  = _get('y')  * _scale
        z  = _get('z')  * _scale
        rx = _get('rx')
        ry = _get('ry')
        rz = _get('rz')

        ox = base_x + x
        oy = base_y + y
        oz = base_z + z

        plane = _plane_from_zyx_euler(ox, oy, oz, rx, ry, rz)
        origin_pt = rg.Point3d(ox, oy, oz)

        planes.append(plane)
        origins.append(origin_pt)
        names.append(name)
        move_types.append(move_type)
        name_to_origin[name] = origin_pt

    # --- Edges ---
    edge_list = data.get('edges') or []
    for edge in edge_list:
        if not isinstance(edge, dict):
            continue
        from_name = str(edge.get('from', ''))
        to_name   = str(edge.get('to',   ''))
        tested    = edge.get('tested')
        status    = 'null' if tested is None else str(tested).lower()

        from_pt = name_to_origin.get(from_name)
        to_pt   = name_to_origin.get(to_name)
        if from_pt is not None and to_pt is not None:
            edge_lines.append(rg.Line(from_pt, to_pt))
            edge_names.append("{} -> {}".format(from_name, to_name))
            edge_statuses.append(status)


try:
    _run()
except Exception as _e:
    import traceback
    # Surface the error as a GH runtime message rather than a silent failure
    raise RuntimeError(
        "visualize_waypoints.py error:\n{}".format(traceback.format_exc())
    ) from _e
