# GhPython component (Rhino 8 / CPython 3)
# Export base cone grab points + approach offsets to YAML. No RoboDK connection.
#
# GH Inputs:
#   cones               : geometry (DataTree one branch per cone, or flat list)
#   grab_points         : flat list of Planes, one per cone — cone grab planes
#   approach_points     : flat list of Planes, one per cone — cone approach planes (pre-computed in GH)
#   string_grab_points  : flat list of Planes, one per cone
#   string_approach_points : flat list of Planes, one per cone — string approach planes (pre-computed in GH)
#   bins                : geometry (flat list or DataTree, one branch per bin)
#   base_origin         : Point — robot base origin in Rhino model space
#   yaml_path           : str   — output YAML path (default: repo/robo_dk_output/base_cone_waypoints.yaml)
#   cone_color          : Colour (optional, default orange)
#   string_color        : Colour (optional, default yellow)
#   bin_color           : Colour (optional, default blue)
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
import math
import struct
import Rhino.Geometry as rg
import scriptcontext as sc
import System

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
    """Write waypoints and edges to a human-readable YAML file."""
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    x: {w['x']}  y: {w['y']}  z: {w['z']}")
        lines.append(f"    rx: {w['rx']}  ry: {w['ry']}  rz: {w['rz']}")
        lines.append(f"    frame: robot_local")
        lines.append(f"    move_type: {w['move_type']}")
        lines.append(f"    j7: {w['j7']}")
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

# Initialise outputs so GH never gets an error even if trigger is off
grab_planes         = []
approach_planes     = []
str_planes          = []
str_approach_planes = []
stl_paths           = []
yaml_out            = ""

if trigger and sc.sticky.get("last_bc_stl_path", "") != "":
    sc.sticky["last_bc_stl_path"] = ""
    print("Trigger activated, forcing re-run")

print("=== INPUT DIAGNOSTICS ===")
diagnose_tree("cones",                   cones)
diagnose_tree("grab_points",             grab_points)
diagnose_tree("approach_points",         approach_points)
diagnose_tree("string_grab_points",      string_grab_points)
diagnose_tree("string_approach_points",  string_approach_points)
diagnose_tree("bins",                    bins if 'bins' in dir() else None)
print("=========================")

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

if not trigger and sc.sticky.get("last_bc_stl_path", "") != "":
    # Not triggered — still populate output planes from what we have
    grab_planes         = [p for p in cone_grab_planes_raw     if p is not None]
    approach_planes     = [p for p in cone_approach_planes_raw if p is not None]
    str_planes          = [p for p in str_grab_planes_raw      if p is not None]
    str_approach_planes = [p for p in str_approach_planes_raw  if p is not None]
    print("Trigger not set — skipping STL/YAML write, outputting planes only.")
else:
    print("Rebuilding STLs and YAML...")

    os.makedirs("C:/temp/base_cones", exist_ok=True)
    os.makedirs("C:/temp/bins", exist_ok=True)

    # Write STLs
    for i, branch_geos in enumerate(cone_branches):
        path = f"C:/temp/base_cones/base_cone_{i}.stl"
        tris = build_and_write_stl(branch_geos, path)
        stl_paths.append(path)
        print(f"  base_cone_{i}: {tris} triangles -> {path}")

    bin_stl_paths = []
    for i, branch_geos in enumerate(bin_branches):
        path = f"C:/temp/bins/bin_{i}.stl"
        tris = build_and_write_stl(branch_geos, path)
        bin_stl_paths.append(path)
        print(f"  bin_{i}: {tris} triangles -> {path}")

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

        grab_planes.append(pl)
        approach_planes.append(ap)

        grab_name     = f"base_cone_grab_{i}"
        approach_name = f"base_cone_grab_{i}_approach"

        x,y,z,rx,ry,rz      = plane_to_xyzrpw_robot_local(pl, base_orig)
        ax,ay,az,arx,ary,arz = plane_to_xyzrpw_robot_local(ap, base_orig)

        waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                          "move_type":"MoveJ","j7":0.0,"note":"cone approach"})
        waypoints.append({"name": grab_name,     "x":x, "y":y, "z":z, "rx":rx, "ry":ry, "rz":rz,
                          "move_type":"MoveL","j7":0.0,"note":"cone grab"})

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

        str_planes.append(pl)
        str_approach_planes.append(ap)

        grab_name     = f"base_str_grab_{i}"
        approach_name = f"base_str_grab_{i}_approach"

        x,y,z,rx,ry,rz      = plane_to_xyzrpw_robot_local(pl, base_orig)
        ax,ay,az,arx,ary,arz = plane_to_xyzrpw_robot_local(ap, base_orig)

        waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                          "move_type":"MoveJ","j7":0.0,"note":"string grab approach"})
        waypoints.append({"name": grab_name,     "x":x, "y":y, "z":z, "rx":rx, "ry":ry, "rz":rz,
                          "move_type":"MoveL","j7":0.0,"note":"string grab"})

        edges.append({"from": approach_name, "to": grab_name})
        edges.append({"from": grab_name,     "to": approach_name})

    # Write YAML
    write_waypoints_yaml(waypoints, edges, yaml_path)
    yaml_out = yaml_path
    print(f"\n[OK] {len(waypoints)} waypoints, {len(edges)} edges -> {yaml_path}")
    print(f"[OK] {len(stl_paths)} STL files written")

    # ── Import STLs into RoboDK ───────────────────────────────────────────────
    try:
        sys.path.append("C:/RoboDK/Python")
        from robodk.robolink import Robolink, ITEM_TYPE_OBJECT
        RDK = Robolink()
        RDK.Item("")  # ping

        # Remove any previously imported base cone objects
        for item in RDK.ItemList(ITEM_TYPE_OBJECT):
            if item.Name().startswith("base_cone_") or item.Name().startswith("bin_"):
                item.Delete()

        for path in stl_paths:
            item = RDK.AddFile(path)
            if item.Valid():
                print(f"  RoboDK: imported {os.path.basename(path)}")
            else:
                print(f"  RoboDK: FAILED to import {os.path.basename(path)}")

        for path in bin_stl_paths:
            item = RDK.AddFile(path)
            if item.Valid():
                print(f"  RoboDK: imported {os.path.basename(path)}")
            else:
                print(f"  RoboDK: FAILED to import {os.path.basename(path)}")

        print("[OK] STLs imported into RoboDK")
    except Exception as e:
        print(f"[WARN] RoboDK import skipped: {e}")

    sc.sticky["last_bc_stl_path"] = stl_paths[0] if stl_paths else ""
