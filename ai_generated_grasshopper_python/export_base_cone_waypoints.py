# GhPython component (Rhino 8 / CPython 3)
# Export base cone grab points + approach offsets to YAML and import STLs into RoboDK.
#
# GH Inputs:
#   cones               : geometry (DataTree one branch per cone, or flat list)
#   grab_points         : flat list of Planes, one per cone — cone grab planes
#   approach_points     : flat list of Planes, one per cone — cone approach planes (pre-computed in GH)
#   string_grab_points  : flat list of Planes, one per cone
#   string_approach_points : flat list of Planes, one per cone — string approach planes (pre-computed in GH)
#   bins                : geometry (flat list or DataTree, one branch per bin)
#   cone_names          : flat list of str — one name per cone; used for RoboDK items and YAML (optional, defaults to "base_cone_0", ...)
#   bin_names           : flat list of str — one name per bin; used for RoboDK items (optional, defaults to "bin_0", ...)
#   base_origin         : Point — robot base origin in Rhino model space
#   yaml_path           : str   — output YAML path (default: repo/robo_dk_output/base_cone_waypoints.yaml)
#   cone_color          : Colour (optional, default orange)
#   string_color        : Colour (optional, default yellow)
#   bin_color           : Colour (optional, default blue)
#   update_and_amalgamate_waypoints : Boolean — if True, runs amalgamate_waypoints.py then import_waypoints_to_robodk.py after export
#   trigger             : Boolean
#
# GH Outputs:
#   grab_planes         : list of Plane — cone grab planes (passthrough)
#   approach_planes     : list of Plane — cone approach planes (passthrough)
#   str_planes          : list of Plane — string grab planes (passthrough)
#   str_approach_planes : list of Plane — string approach planes (passthrough)
#   stl_paths           : list of str  — STL files written
#   yaml_out            : str          — path to written YAML

import os
import sys
import json
import math
import struct
import Rhino.Geometry as rg
import scriptcontext as sc
import System

# ── End effector names for robert_end_checker_config.json ──────────────────────
PICKUP_END_EFFECTOR_NAME   = "pickup"
KNOTTING_END_EFFECTOR_NAME = "knotting"
SOURCE_SCRIPT = "export_base_cone_waypoints"

REPO_ROOT = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
CHECKER_CONFIG_PATH = os.path.join(REPO_ROOT, "robert_checker_stuff", "robert_end_checker_config.json")

# ── Optional inputs with defaults ────────────────────────────────────────────
try:
    yaml_path
except NameError:
    yaml_path = None

try:
    approach_points
except NameError:
    approach_points = None

try:
    string_approach_points
except NameError:
    string_approach_points = None

try:
    strings
except NameError:
    strings = None

try:
    cone_names
except NameError:
    cone_names = None

try:
    bin_names
except NameError:
    bin_names = None

try:
    update_and_amalgamate_waypoints
except NameError:
    update_and_amalgamate_waypoints = False

if yaml_path is None or yaml_path == "":
    yaml_path = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\base_cone_waypoints.yaml"

# ── Helpers (unchanged from original) ────────────────────────────────────────

def flatten_input(gh_input, exclude_type=None):
    result = []
    if hasattr(gh_input, 'Branches'):
        for branch in gh_input.Branches:
            for item in branch:
                if item is not None:
                    result.append(item)
    elif hasattr(gh_input, '__iter__') and (exclude_type is None or not isinstance(gh_input, exclude_type)):
        for item in gh_input:
            if item is not None:
                result.append(item)
    elif gh_input is not None:
        result.append(gh_input)
    return result

def resolve_first_point(gh_input):
    def to_pt(item):
        if isinstance(item, System.Guid):
            rhino_obj = sc.doc.Objects.FindId(item)
            if rhino_obj:
                geo = rhino_obj.Geometry
                if isinstance(geo, rg.Point): return geo.Location
                elif hasattr(geo, 'Location'): return geo.Location
                else: return geo.GetBoundingBox(True).Center
            return None
        if isinstance(item, rg.Point3d): return item
        if isinstance(item, rg.Point): return item.Location
        if hasattr(item, 'X') and hasattr(item, 'Y') and hasattr(item, 'Z'):
            return rg.Point3d(item.X, item.Y, item.Z)
        return None
    items = flatten_input(gh_input, exclude_type=rg.Point3d)
    if not items:
        return to_pt(gh_input)
    for item in items:
        pt = to_pt(item)
        if pt is not None:
            return pt
    return None

def resolve_plane(item):
    if item is None:
        return None
    if isinstance(item, rg.Plane):
        return item
    if isinstance(item, System.Guid):
        rhino_obj = sc.doc.Objects.FindId(item)
        if rhino_obj:
            geo = rhino_obj.Geometry
            if isinstance(geo, rg.Plane): return geo
            if isinstance(geo, rg.Point): item = geo.Location
            elif hasattr(geo, 'Location'): item = geo.Location
            else: item = geo.GetBoundingBox(True).Center
        else:
            print(f"  Could not find Guid: {item}")
            return None
    if isinstance(item, rg.Point3d):
        return rg.Plane(item, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    if isinstance(item, rg.Point):
        return rg.Plane(item.Location, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    if hasattr(item, 'X') and hasattr(item, 'Y') and hasattr(item, 'Z'):
        return rg.Plane(rg.Point3d(item.X, item.Y, item.Z), rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    print(f"  Unknown plane type: {type(item)}")
    return None

def resolve_color(gh_color, default_rgba):
    if gh_color is None:
        return default_rgba
    def color_from_raw(raw):
        if raw is None: return None
        if type(raw).__name__ == 'Color' or (hasattr(raw,'R') and hasattr(raw,'G') and hasattr(raw,'B') and hasattr(raw,'A')):
            return [raw.R/255.0, raw.G/255.0, raw.B/255.0, raw.A/255.0]
        if isinstance(raw,(list,tuple)) and all(isinstance(v,(int,float)) for v in raw):
            vals = list(raw)[:4]; vals = [v/255.0 for v in vals] if any(v>1.0 for v in vals) else vals
            while len(vals)<4: vals.append(1.0)
            return vals
        return None
    if hasattr(gh_color,'Branches'):
        try: return color_from_raw(list(gh_color.Branches[0])[0]) or default_rgba
        except: return default_rgba
    result = color_from_raw(gh_color)
    if result: return result
    if hasattr(gh_color,'__iter__') and not isinstance(gh_color,str):
        try:
            for item in gh_color:
                if item is not None:
                    result = color_from_raw(item)
                    if result: return result
                    break
        except: pass
    return default_rgba

def diagnose_tree(name, gh_input):
    if gh_input is None:
        print(f"  {name}: None")
        return
    if hasattr(gh_input, 'Branches'):
        print(f"  {name}: DataTree {len(gh_input.Branches)} branches")
        for j, (path, branch) in enumerate(zip(gh_input.Paths, gh_input.Branches)):
            print(f"    branch[{j}] path={path} items={len(branch)}")
    elif hasattr(gh_input, '__iter__') and not isinstance(gh_input, (rg.Mesh, rg.Point3d, rg.Plane)):
        items = list(gh_input)
        print(f"  {name}: list len={len(items)}")
    else:
        print(f"  {name}: {type(gh_input).__name__}")

# ── Pose conversion ───────────────────────────────────────────────────────────

def plane_to_xyzrpw_robot_local(plane, base_orig):
    """Convert Rhino Plane to robot-local XYZ + ZYX Euler angles (degrees).

    base_orig is a Point3d marking the robot base in Rhino world space.
    If None, returns world-space coordinates.
    """
    ox = plane.Origin.X - (base_orig.X if base_orig else 0.0)
    oy = plane.Origin.Y - (base_orig.Y if base_orig else 0.0)
    oz = plane.Origin.Z - (base_orig.Z if base_orig else 0.0)

    xax, yax, zax = plane.XAxis, plane.YAxis, plane.ZAxis

    # ZYX Euler extraction from rotation matrix columns
    sy = -xax.Z
    cy = math.sqrt(xax.X**2 + xax.Y**2)
    if cy > 1e-6:
        rx = math.atan2(yax.Z, zax.Z)
        ry = math.atan2(sy, cy)
        rz = math.atan2(xax.Y, xax.X)
    else:
        rx = math.atan2(-zax.Y, yax.Y)
        ry = math.atan2(sy, cy)
        rz = 0.0

    return (
        round(ox, 4), round(oy, 4), round(oz, 4),
        round(math.degrees(rx), 6), round(math.degrees(ry), 6), round(math.degrees(rz), 6),
    )

# ── STL writer (unchanged) ────────────────────────────────────────────────────

def build_and_write_stl(geo_list, stl_path):
    mesh_list = rg.Mesh()
    for geo in geo_list:
        if isinstance(geo, rg.Mesh):
            geo.Faces.ConvertQuadsToTriangles(); mesh_list.Append(geo)
        elif isinstance(geo, rg.Brep):
            meshes = rg.Mesh.CreateFromBrep(geo, rg.MeshingParameters.Coarse)
            if meshes:
                for m in meshes: m.Faces.ConvertQuadsToTriangles(); mesh_list.Append(m)
        elif isinstance(geo, System.Guid):
            rhino_obj = sc.doc.Objects.FindId(geo)
            if rhino_obj:
                obj_geo = rhino_obj.Geometry
                if isinstance(obj_geo, rg.Mesh):
                    obj_geo.Faces.ConvertQuadsToTriangles(); mesh_list.Append(obj_geo)
                elif isinstance(obj_geo, rg.Brep):
                    meshes = rg.Mesh.CreateFromBrep(obj_geo, rg.MeshingParameters.Coarse)
                    if meshes:
                        for m in meshes: m.Faces.ConvertQuadsToTriangles(); mesh_list.Append(m)
    mesh_list.Faces.ConvertQuadsToTriangles()
    triangles = []
    for i in range(mesh_list.Faces.Count):
        face = mesh_list.Faces[i]
        triangles.append((mesh_list.Vertices[face.A], mesh_list.Vertices[face.B], mesh_list.Vertices[face.C]))
    os.makedirs(os.path.dirname(stl_path), exist_ok=True)
    with open(stl_path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(triangles)))
        for v0, v1, v2 in triangles:
            f.write(struct.pack('<fff', 0, 0, 0))
            f.write(struct.pack('<fff', v0.X, v0.Y, v0.Z))
            f.write(struct.pack('<fff', v1.X, v1.Y, v1.Z))
            f.write(struct.pack('<fff', v2.X, v2.Y, v2.Z))
            f.write(struct.pack('<H', 0))
    return len(triangles)

# ── YAML writer ───────────────────────────────────────────────────────────────

def write_waypoints_yaml(waypoints, edges, path):
    """Write waypoints and edges to a valid YAML file."""
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    x: {w['x']}")
        lines.append(f"    y: {w['y']}")
        lines.append(f"    z: {w['z']}")
        lines.append(f"    rx: {w['rx']}")
        lines.append(f"    ry: {w['ry']}")
        lines.append(f"    rz: {w['rz']}")
        lines.append(f"    frame: robot_local")
        lines.append(f"    move_type: {w['move_type']}")
        j7_val = w.get('j7')
        lines.append(f"    j7: {'null' if j7_val is None else j7_val}")
        if w.get('z_axis_free'):
            lines.append(f"    z_axis_free: true")
        if w.get('special_conditions'):
            conds = ", ".join(w['special_conditions'])
            lines.append(f"    special_conditions: [{conds}]")
        lines.append(f"    source: grasshopper")
        if w.get('note'):
            lines.append(f"    note: \"{w['note']}\"")

    lines.append("")
    lines.append("edges:")
    for e in edges:
        lines.append(f"  - from: {e['from']}")
        lines.append(f"    to:   {e['to']}")
        lines.append(f"    tested: null")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

# ── Main ──────────────────────────────────────────────────────────────────────

try:
    trigger
except NameError:
    trigger = True

# Initialise outputs so GH never gets an error even if trigger is off
grab_planes         = []
approach_planes     = []
str_planes          = []
str_approach_planes = []
stl_paths           = []
yaml_out            = ""

print("=== INPUT DIAGNOSTICS ===")
diagnose_tree("cones",                   cones)
diagnose_tree("grab_points",             grab_points)
diagnose_tree("approach_points",         approach_points)
diagnose_tree("string_grab_points",      string_grab_points)
diagnose_tree("string_approach_points",  string_approach_points)
diagnose_tree("strings",                 strings)
diagnose_tree("bins",                    bins if 'bins' in dir() else None)
print("=========================")

raw_cone_names = flatten_input(cone_names) if cone_names is not None else []
raw_bin_names  = flatten_input(bin_names)  if bin_names  is not None else []
def cone_name(i):
    if i < len(raw_cone_names) and raw_cone_names[i]:
        return str(raw_cone_names[i])
    return f"base_cone_{i}"
def bin_name(i):
    if i < len(raw_bin_names) and raw_bin_names[i]:
        return str(raw_bin_names[i])
    return f"bin_{i}"

# Collect inputs
cone_branches = []
if hasattr(cones, 'Branches') and len(cones.Branches) > 1:
    for branch in cones.Branches:
        items = [item for item in branch if item is not None]
        if items: cone_branches.append(items)
    print(f"MODE A: {len(cone_branches)} cone branches from DataTree")
else:
    flat = flatten_input(cones)
    for item in flat: cone_branches.append([item])
    print(f"MODE B: {len(cone_branches)} cones from flat list")

bin_branches = []
if 'bins' in dir() and bins is not None:
    if hasattr(bins, 'Branches') and len(bins.Branches) > 1:
        for branch in bins.Branches:
            items = [item for item in branch if item is not None]
            if items: bin_branches.append(items)
    else:
        flat = flatten_input(bins)
        for item in flat: bin_branches.append([item])

string_branches = []
if strings is not None:
    if hasattr(strings, 'Branches') and len(strings.Branches) > 1:
        for branch in strings.Branches:
            items = [item for item in branch if item is not None]
            if items: string_branches.append(items)
    else:
        flat = flatten_input(strings)
        for item in flat: string_branches.append([item])

cone_grab_planes_raw     = [resolve_plane(p) for p in flatten_input(grab_points)]
cone_approach_planes_raw = [resolve_plane(p) for p in flatten_input(approach_points)] if approach_points is not None else []
str_grab_planes_raw      = [resolve_plane(p) for p in flatten_input(string_grab_points)]
str_approach_planes_raw  = [resolve_plane(p) for p in flatten_input(string_approach_points)] if string_approach_points is not None else []
base_orig                = resolve_first_point(base_origin) if 'base_origin' in dir() else None

num_cones = len(cone_branches)
num_bins  = len(bin_branches)
print(f"num_cones={num_cones}  cone_grabs={len(cone_grab_planes_raw)}  cone_approaches={len(cone_approach_planes_raw)}")
print(f"str_grabs={len(str_grab_planes_raw)}  str_approaches={len(str_approach_planes_raw)}")
print(f"base_origin: {base_orig}")

# Always populate output planes from inputs
grab_planes         = [p for p in cone_grab_planes_raw     if p is not None]
approach_planes     = [p for p in cone_approach_planes_raw if p is not None]
str_planes          = [p for p in str_grab_planes_raw      if p is not None]
str_approach_planes = [p for p in str_approach_planes_raw  if p is not None]

if trigger:
    print("Rebuilding STLs and YAML...")

    os.makedirs("C:/temp/base_cones", exist_ok=True)
    os.makedirs("C:/temp/bins", exist_ok=True)
    os.makedirs("C:/temp/base_strings", exist_ok=True)

    # Write STLs
    for i, branch_geos in enumerate(cone_branches):
        path = f"C:/temp/base_cones/{cone_name(i)}.stl"
        tris = build_and_write_stl(branch_geos, path)
        stl_paths.append(path)
        print(f"  {cone_name(i)}: {tris} triangles -> {path}")

    bin_stl_paths = []
    for i, branch_geos in enumerate(bin_branches):
        path = f"C:/temp/bins/{bin_name(i)}.stl"
        tris = build_and_write_stl(branch_geos, path)
        bin_stl_paths.append(path)
        print(f"  {bin_name(i)}: {tris} triangles -> {path}")

    string_stl_paths = []
    for i, branch_geos in enumerate(string_branches):
        path = f"C:/temp/base_strings/{cone_name(i)}_string.stl"
        tris = build_and_write_stl(branch_geos, path)
        string_stl_paths.append(path)
        print(f"  {cone_name(i)}_string: {tris} triangles -> {path}")

    # Build output planes and YAML entries
    waypoints = []
    edges     = []

    for i, pl in enumerate(cone_grab_planes_raw):
        if pl is None:
            print(f"  WARNING: cone_grab_{i} plane is None — skipping")
            continue

        ap = cone_approach_planes_raw[i] if i < len(cone_approach_planes_raw) else None
        if ap is None:
            print(f"  WARNING: cone_grab_{i} approach plane is None — skipping")
            continue

        grab_name     = f"{cone_name(i)}_grab"
        approach_name = f"{cone_name(i)}_grab_approach"

        x,y,z,rx,ry,rz      = plane_to_xyzrpw_robot_local(pl, base_orig)
        ax,ay,az,arx,ary,arz = plane_to_xyzrpw_robot_local(ap, base_orig)

        waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                          "move_type":"MoveJ","j7":None,"z_axis_free":True,
                          "special_conditions":["attached_to_base"],
                          "note":"cone approach - robot-relative, j7 free, rotation around cone Z axis free"})
        waypoints.append({"name": grab_name,     "x":x, "y":y, "z":z, "rx":rx, "ry":ry, "rz":rz,
                          "move_type":"MoveL","j7":None,
                          "special_conditions":["attached_to_base"],
                          "note":"cone grab - robot-relative, j7 free"})

        # Bidirectional edges — enter (approach→grab) and exit (grab→approach)
        edges.append({"from": approach_name, "to": grab_name})
        edges.append({"from": grab_name,     "to": approach_name})

        print(f"  cone_grab_{i}: grab=({x:.1f},{y:.1f},{z:.1f})  approach=({ax:.1f},{ay:.1f},{az:.1f})")

    for i, pl in enumerate(str_grab_planes_raw):
        if pl is None:
            print(f"  WARNING: str_grab_{i} plane is None — skipping")
            continue

        ap = str_approach_planes_raw[i] if i < len(str_approach_planes_raw) else None
        if ap is None:
            print(f"  WARNING: str_grab_{i} approach plane is None — skipping")
            continue

        grab_name     = f"{cone_name(i)}_string_grab"
        approach_name = f"{cone_name(i)}_string_grab_approach"

        x,y,z,rx,ry,rz      = plane_to_xyzrpw_robot_local(pl, base_orig)
        ax,ay,az,arx,ary,arz = plane_to_xyzrpw_robot_local(ap, base_orig)

        waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                          "move_type":"MoveJ","j7":None,"z_axis_free":True,
                          "special_conditions":["attached_to_base"],
                          "note":"string grab approach - robot-relative, j7 free, rotation around cone Z axis free"})
        waypoints.append({"name": grab_name,     "x":x, "y":y, "z":z, "rx":rx, "ry":ry, "rz":rz,
                          "move_type":"MoveL","j7":None,
                          "special_conditions":["attached_to_base"],
                          "note":"string grab - robot-relative, j7 free"})

        edges.append({"from": approach_name, "to": grab_name})
        edges.append({"from": grab_name,     "to": approach_name})

    # Write YAML
    write_waypoints_yaml(waypoints, edges, yaml_path)
    yaml_out = yaml_path
    print(f"\n[OK] {len(waypoints)} waypoints, {len(edges)} edges -> {yaml_path}")
    print(f"[OK] {len(stl_paths)} STL files written")

    # ── Write robert_end_checker_config.json ───────────────────────────────────
    if os.path.exists(CHECKER_CONFIG_PATH):
        with open(CHECKER_CONFIG_PATH, "r", encoding="utf-8") as f:
            checker_config = json.load(f)
    else:
        checker_config = {}
    if "end_effectors" not in checker_config:
        checker_config["end_effectors"] = []

    # Build pickup points (base cone grab + approach)
    pickup_points = []
    for i, pl in enumerate(cone_grab_planes_raw):
        if pl is None:
            continue
        grab_name     = f"{cone_name(i)}_grab"
        approach_name = f"{cone_name(i)}_grab_approach"
        pickup_points.append({
            "name": grab_name,
            "type": "point",
            "name_path": f"WaypointTargets/{grab_name}",
            "source_script": SOURCE_SCRIPT,
            "special_track_conditions": {"type": "Locked_at_j7_0"}
        })
        ap = cone_approach_planes_raw[i] if i < len(cone_approach_planes_raw) else None
        if ap is not None:
            pickup_points.append({
                "name": approach_name,
                "type": "point",
                "name_path": f"WaypointTargets/{approach_name}",
                "source_script": SOURCE_SCRIPT,
                "special_track_conditions": {"type": "Locked_at_j7_0"}
            })

    # Build knotting points (string grab + approach)
    knotting_points = []
    for i, pl in enumerate(str_grab_planes_raw):
        if pl is None:
            continue
        grab_name     = f"{cone_name(i)}_string_grab"
        approach_name = f"{cone_name(i)}_string_grab_approach"
        knotting_points.append({
            "name": grab_name,
            "type": "point",
            "name_path": f"WaypointTargets/{grab_name}",
            "source_script": SOURCE_SCRIPT,
            "special_track_conditions": {"type": "Locked_at_j7_0"}
        })
        ap = str_approach_planes_raw[i] if i < len(str_approach_planes_raw) else None
        if ap is not None:
            knotting_points.append({
                "name": approach_name,
                "type": "point",
                "name_path": f"WaypointTargets/{approach_name}",
                "source_script": SOURCE_SCRIPT,
                "special_track_conditions": {"type": "Locked_at_j7_0"}
            })

    def upsert_end_effector(config, ee_name, new_points, source):
        """Replace only points from this source_script, keep points from other sources."""
        for ee in config["end_effectors"]:
            if ee["end_effector_name"] == ee_name:
                other_points = [p for p in ee["paths_and_points_to_check"] if p.get("source_script") != source]
                ee["paths_and_points_to_check"] = other_points + new_points
                return
        config["end_effectors"].append({
            "end_effector_name": ee_name,
            "paths_and_points_to_check": new_points
        })

    upsert_end_effector(checker_config, PICKUP_END_EFFECTOR_NAME, pickup_points, SOURCE_SCRIPT)
    upsert_end_effector(checker_config, KNOTTING_END_EFFECTOR_NAME, knotting_points, SOURCE_SCRIPT)

    with open(CHECKER_CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(checker_config, f, indent=2)
    print(f"[OK] robert_end_checker_config.json updated: {len(pickup_points)} pickup, {len(knotting_points)} knotting points")

    # ── Import STLs into RoboDK ───────────────────────────────────────────────
    try:
        sys.path.append("C:/RoboDK/Python")
        from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT
        from robodk.robomath import Mat
        RDK = Robolink()
        RDK.Item("")  # ping

        _cone_rgba   = resolve_color(cone_color   if 'cone_color'   in dir() else None, [1.0, 0.5, 0.0, 1.0])
        _string_rgba = resolve_color(string_color if 'string_color' in dir() else None, [1.0, 1.0, 0.0, 1.0])
        _bin_rgba    = resolve_color(bin_color    if 'bin_color'    in dir() else None, [0.5, 0.5, 0.5, 1.0])

        robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
        if not robot.Valid():
            # Robot was replaced — find the first robot in the station
            all_robots = RDK.ItemList(ITEM_TYPE_ROBOT, False)
            robot = all_robots[0] if all_robots else None
            if robot and robot.Valid():
                print(f"  RoboDK: 'Fanuc R2000iC 125L' not found, using '{robot.Name()}' instead")
            else:
                robot = None
        # Parent cones to the moving rail carriage so they travel with j7.
        # Try "RailMechanism" by name first (the moving part), then fall back
        # to the robot's direct parent.
        rail_mech = RDK.Item("RailMechanism")
        if rail_mech.Valid():
            robot_base = rail_mech
            print(f"  RoboDK: parenting under 'RailMechanism' (moving carriage)")
        elif robot is not None and robot.Valid():
            robot_base = robot.Parent()
            if robot_base is not None and robot_base.Valid():
                print(f"  RoboDK: 'RailMechanism' not found, parenting under '{robot_base.Name()}'")
            else:
                robot_base = None
                print("  RoboDK: robot base not found, objects at world level")
        else:
            robot_base = None
            print("  RoboDK: no robot or rail found, objects at world level")

        station = RDK.Item("", 1)

        def identity():
            return Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        def purge_all(name):
            for _ in range(50):
                item = RDK.Item(name)
                if not item.Valid(): break
                item.Delete()

        def add_file_and_get_item(stl_path):
            before = set(it.item for it in RDK.ItemList(ITEM_TYPE_OBJECT, False))
            RDK.AddFile(stl_path)
            for it in RDK.ItemList(ITEM_TYPE_OBJECT, False):
                if it.item not in before:
                    return it
            return RDK.Item(os.path.splitext(os.path.basename(stl_path))[0])

        # Purge only items added by previous runs — delete parent frames
        purge_all("BaseCones")
        purge_all("BaseBins")

        # BaseCones frame parented to robot_base so it moves with the rail
        parent = robot_base if robot_base is not None else station
        base_cones_frame = RDK.AddFrame("BaseCones", parent)
        base_cones_frame.setPose(identity())

        base_bins_frame = RDK.AddFrame("BaseBins", parent)
        base_bins_frame.setPose(identity())

        for i, path in enumerate(stl_paths):
            obj = add_file_and_get_item(path)
            if obj.Valid():
                obj.setName(cone_name(i))
                obj.setParentStatic(base_cones_frame)
                obj.setColor(_cone_rgba)
                print(f"  RoboDK: {cone_name(i)} imported")
            else:
                print(f"  RoboDK: FAILED {os.path.basename(path)}")

        for i, path in enumerate(string_stl_paths):
            obj = add_file_and_get_item(path)
            if obj.Valid():
                obj.setName(f"{cone_name(i)}_string")
                obj.setParentStatic(base_cones_frame)
                obj.setColor(_string_rgba)
                print(f"  RoboDK: {cone_name(i)}_string imported")
            else:
                print(f"  RoboDK: FAILED {os.path.basename(path)}")

        for i, path in enumerate(bin_stl_paths):
            obj = add_file_and_get_item(path)
            if obj.Valid():
                obj.setName(bin_name(i))
                obj.setParentStatic(base_bins_frame)
                obj.setColor(_bin_rgba)
                print(f"  RoboDK: {bin_name(i)} imported")
            else:
                print(f"  RoboDK: FAILED {os.path.basename(path)}")

        print("[OK] STLs imported into RoboDK")
    except Exception as e:
        print(f"[WARN] RoboDK import skipped: {e}")

    # ── Amalgamate waypoints + re-import to RoboDK ────────────────────────────
    if update_and_amalgamate_waypoints:
        import subprocess
        REPO   = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
        PYTHON = r"C:\Users\samst\.rhinocode\py39-rh8\python.exe"
        amalgamate_script = os.path.join(REPO, "robodk_code", "amalgamate_waypoints.py")
        import_script     = os.path.join(REPO, "robodk_code", "import_waypoints_to_robodk.py")

        print("\n--- amalgamate_waypoints ---")
        r1 = subprocess.run([PYTHON, amalgamate_script], capture_output=True, text=True)
        print(r1.stdout)
        if r1.returncode != 0:
            print(f"[ERROR] amalgamate_waypoints failed (rc={r1.returncode}):\n{r1.stderr}")
        else:
            print("\n--- import_waypoints_to_robodk ---")
            r2 = subprocess.run([PYTHON, import_script], capture_output=True, text=True)
            print(r2.stdout)
            if r2.returncode != 0:
                print(f"[ERROR] import_waypoints_to_robodk failed (rc={r2.returncode}):\n{r2.stderr}")
