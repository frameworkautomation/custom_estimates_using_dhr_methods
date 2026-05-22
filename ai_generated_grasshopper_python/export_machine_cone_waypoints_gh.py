# GhPython component (Rhino 8 / CPython 3)
# Export machine cone grab points to YAML and import STLs + targets into RoboDK.
#
# GH Inputs:
#   cones                  : geometry (DataTree one branch per cone, or flat list)
#   grab_points            : flat list of Planes — cone grab planes
#   approach_points        : flat list of Planes — cone approach planes (pre-computed in GH)
#   string_grab_points     : flat list of Planes — string grab planes
#   string_approach_points : flat list of Planes — string approach planes (pre-computed in GH)
#   strings                : geometry (flat list)
#   cone_color             : Colour (optional, default red)
#   string_color           : Colour (optional, default green)
#   trigger                : Boolean (default True)
#
# GH Outputs:
#   a : last STL path written (str)

from Grasshopper import DataTree
import Rhino
import Rhino.Geometry as rg
import scriptcontext as sc
import System
import struct
import os
import sys
import math

# ── Optional inputs with defaults ─────────────────────────────────────────────
try:
    trigger
except NameError:
    trigger = True

try:
    approach_points
except NameError:
    approach_points = None

try:
    string_approach_points
except NameError:
    string_approach_points = None

YAML_PATH = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robo_dk_output\machine_cone_waypoints.yaml"

# ── Output default ─────────────────────────────────────────────────────────────
a = ""

# ── Helpers ────────────────────────────────────────────────────────────────────

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

def diagnose_tree(name, gh_input):
    if hasattr(gh_input, 'Branches'):
        print(f"  {name}: DataTree {len(gh_input.Branches)} branches")
        for j, (path, branch) in enumerate(zip(gh_input.Paths, gh_input.Branches)):
            print(f"    branch[{j}] path={path} items={len(branch)}")
    elif hasattr(gh_input, '__iter__') and not isinstance(gh_input, (rg.Mesh, rg.Point3d, rg.Plane)):
        print(f"  {name}: list len={len(list(gh_input))}")
    else:
        print(f"  {name}: {type(gh_input).__name__}")

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
        if type(raw).__name__ == 'Color' or (hasattr(raw,'R') and hasattr(raw,'G') and hasattr(raw,'B')):
            return [raw.R/255.0, raw.G/255.0, raw.B/255.0, raw.A/255.0]
        if isinstance(raw, (list, tuple)) and all(isinstance(v, (int, float)) for v in raw):
            vals = list(raw)[:4]
            if any(v > 1.0 for v in vals): vals = [v/255.0 for v in vals]
            while len(vals) < 4: vals.append(1.0)
            return vals
        return None
    if hasattr(gh_color, 'Branches'):
        try: return color_from_raw(list(gh_color.Branches[0])[0]) or default_rgba
        except: return default_rgba
    result = color_from_raw(gh_color)
    if result: return result
    if hasattr(gh_color, '__iter__') and not isinstance(gh_color, str):
        try:
            for item in gh_color:
                if item is not None:
                    result = color_from_raw(item)
                    if result: return result
                    break
        except: pass
    return default_rgba

def plane_to_xyzrpw(plane):
    """Convert Rhino Plane to world-space XYZ + ZYX Euler angles (degrees)."""
    ox, oy, oz = plane.Origin.X, plane.Origin.Y, plane.Origin.Z
    xax, yax, zax = plane.XAxis, plane.YAxis, plane.ZAxis
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
    return (round(ox,4), round(oy,4), round(oz,4),
            round(math.degrees(rx),6), round(math.degrees(ry),6), round(math.degrees(rz),6))

def write_waypoints_yaml(waypoints, edges, path):
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    x: {w['x']}")
        lines.append(f"    y: {w['y']}")
        lines.append(f"    z: {w['z']}")
        lines.append(f"    rx: {w['rx']}")
        lines.append(f"    ry: {w['ry']}")
        lines.append(f"    rz: {w['rz']}")
        lines.append(f"    frame: world")
        lines.append(f"    move_type: {w['move_type']}")
        lines.append(f"    j7: null")
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

# ── Collect inputs ─────────────────────────────────────────────────────────────

print("=== INPUT DIAGNOSTICS ===")
diagnose_tree("cones",                   cones)
diagnose_tree("grab_points",             grab_points)
diagnose_tree("approach_points",         approach_points)
diagnose_tree("string_grab_points",      string_grab_points)
diagnose_tree("string_approach_points",  string_approach_points)
diagnose_tree("strings",                 strings)
print("=========================")

cone_rgba   = resolve_color(cone_color   if 'cone_color'   in dir() else None, [1.0, 0.0, 0.0, 1.0])
string_rgba = resolve_color(string_color if 'string_color' in dir() else None, [0.0, 1.0, 0.0, 1.0])

cone_branches = []
if hasattr(cones, 'Branches') and len(cones.Branches) > 1:
    for branch in cones.Branches:
        items = [item for item in branch if item is not None]
        if items: cone_branches.append(items)
    print(f"MODE A: {len(cone_branches)} branches from DataTree")
else:
    flat = flatten_input(cones, exclude_type=rg.Mesh)
    for item in flat: cone_branches.append([item])
    print(f"MODE B: {len(cone_branches)} cones from flat list")

strings_list           = flatten_input(strings, exclude_type=rg.Mesh)
cone_grab_planes       = [resolve_plane(p) for p in flatten_input(grab_points)]
cone_approach_planes   = [resolve_plane(p) for p in flatten_input(approach_points)] if approach_points is not None else []
str_grab_planes        = [resolve_plane(p) for p in flatten_input(string_grab_points)]
str_approach_planes    = [resolve_plane(p) for p in flatten_input(string_approach_points)] if string_approach_points is not None else []

num_cones = len(cone_branches)
print(f"num_cones={num_cones}  cone_grabs={len(cone_grab_planes)}  cone_approaches={len(cone_approach_planes)}")
print(f"str_grabs={len(str_grab_planes)}  str_approaches={len(str_approach_planes)}")

if trigger:
    print("Rebuilding...")

    os.makedirs("C:/temp/cones", exist_ok=True)
    cone_stl_paths = []
    for i, branch_geos in enumerate(cone_branches):
        path = f"C:/temp/cones/cone_{i}.stl"
        tris = build_and_write_stl(branch_geos, path)
        cone_stl_paths.append(path)
        print(f"  cone_{i}: {tris} triangles -> {path}")

    strings_stl_path = "C:/temp/strings.stl"
    string_tris = build_and_write_stl(strings_list, strings_stl_path)
    print(f"Strings: {string_tris} triangles -> {strings_stl_path}")

    # ── YAML export ───────────────────────────────────────────────────────────
    waypoints = []
    edges     = []

    for i, pl in enumerate(cone_grab_planes):
        if pl is None:
            print(f"  WARNING: cone_grab_{i} plane is None — skipping")
            continue
        ap = cone_approach_planes[i] if i < len(cone_approach_planes) else None
        if ap is None:
            print(f"  WARNING: cone_grab_{i} approach plane is None — skipping YAML entry")
        grab_name     = f"cone_grab_{i}"
        approach_name = f"cone_grab_{i}_approach"
        x,y,z,rx,ry,rz = plane_to_xyzrpw(pl)
        waypoints.append({"name": grab_name, "x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz,
                          "move_type":"MoveL","note":"machine cone place"})
        if ap is not None:
            ax,ay,az,arx,ary,arz = plane_to_xyzrpw(ap)
            waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                              "move_type":"MoveJ","note":"machine cone approach"})
            edges.append({"from": approach_name, "to": grab_name})
            edges.append({"from": grab_name,     "to": approach_name})

    for i, pl in enumerate(str_grab_planes):
        if pl is None:
            print(f"  WARNING: str_grab_{i} plane is None — skipping")
            continue
        ap = str_approach_planes[i] if i < len(str_approach_planes) else None
        grab_name     = f"string_grab_{i}"
        approach_name = f"string_grab_{i}_approach"
        x,y,z,rx,ry,rz = plane_to_xyzrpw(pl)
        waypoints.append({"name": grab_name, "x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz,
                          "move_type":"MoveL","note":"machine string grab"})
        if ap is not None:
            ax,ay,az,arx,ary,arz = plane_to_xyzrpw(ap)
            waypoints.append({"name": approach_name, "x":ax,"y":ay,"z":az,"rx":arx,"ry":ary,"rz":arz,
                              "move_type":"MoveJ","note":"machine string approach"})
            edges.append({"from": approach_name, "to": grab_name})
            edges.append({"from": grab_name,     "to": approach_name})

    write_waypoints_yaml(waypoints, edges, YAML_PATH)
    print(f"[OK] {len(waypoints)} waypoints, {len(edges)} edges -> {YAML_PATH}")

    # ── RoboDK import ─────────────────────────────────────────────────────────
    try:
        sys.path.append("C:/RoboDK/Python")
        from robodk.robolink import Robolink
        from robodk.robomath import Mat

        def identity():
            return Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        def plane_to_pose(pl):
            x, y, z, o = pl.XAxis, pl.YAxis, pl.ZAxis, pl.Origin
            return Mat([[x.X,y.X,z.X,o.X],[x.Y,y.Y,z.Y,o.Y],[x.Z,y.Z,z.Z,o.Z],[0,0,0,1]])

        RDK = Robolink()
        station = RDK.Item("", 1)
        print(f"Station: {station.Name()}")

        robot = RDK.Item("Fanuc R2000iC 125L")
        if not robot.Valid(): robot = RDK.Item("", 4)
        print(f"Robot: {robot.Name()}")

        world_frame = RDK.Item("WorldFrame")
        if not world_frame.Valid():
            world_frame = RDK.AddFrame("WorldFrame", station)
            world_frame.setPose(identity())
            print("Created WorldFrame")

        def purge_all(name):
            count = 0
            for _ in range(50):
                item = RDK.Item(name)
                if not item.Valid(): break
                item.Delete()
                count += 1
            if count: print(f"  purged {count}x '{name}'")

        def add_file_and_get_item(stl_path):
            from robodk.robolink import ITEM_TYPE_OBJECT
            before = set(it.item for it in RDK.ItemList(ITEM_TYPE_OBJECT, False))
            RDK.AddFile(stl_path)
            for it in RDK.ItemList(ITEM_TYPE_OBJECT, False):
                if it.item not in before:
                    return it
            stem = os.path.splitext(os.path.basename(stl_path))[0]
            return RDK.Item(stem)

        def make_target(name, pl, parent_frame):
            t = RDK.AddTarget(name, world_frame, robot)
            t.setAsCartesianTarget()
            t.setPoseAbs(plane_to_pose(pl))
            t.setParent(parent_frame)
            t2 = RDK.Item(name)
            t2.setRobot(robot)
            print(f"    {name}: parent='{t2.Parent().Name()}' valid={t2.Valid()}")

        # Purge old items
        purge_all("strings")
        for i in range(200):
            purge_all(f"cone_grab_{i}")
            purge_all(f"string_grab_{i}")
            purge_all(f"Cone_{i}")
            purge_all(f"cone_{i}")
        purge_all("Cones")

        # Rebuild
        cones_frame = RDK.AddFrame("Cones", station)
        cones_frame.setPose(identity())
        print("Created Cones frame")

        strings_item = add_file_and_get_item(strings_stl_path)
        if strings_item.Valid():
            strings_item.setColor(string_rgba)
            strings_item.setParentStatic(cones_frame)
            print("Strings: ok")
        else:
            print("WARNING: strings item not valid")

        for i in range(num_cones):
            cone_frame = RDK.AddFrame(f"Cone_{i}", cones_frame)
            cone_frame.setPose(identity())

            cone_obj = add_file_and_get_item(cone_stl_paths[i])
            if cone_obj.Valid():
                cone_obj.setColor(cone_rgba)
                cone_obj.setParentStatic(cone_frame)
                print(f"  Cone_{i}: STL ok")
            else:
                print(f"  WARNING: Cone_{i} STL not valid")

            if i < len(cone_grab_planes) and cone_grab_planes[i] is not None:
                make_target(f"cone_grab_{i}", cone_grab_planes[i], RDK.Item(f"Cone_{i}"))
            else:
                print(f"  WARNING: Cone_{i} missing cone_grab plane")

            if i < len(str_grab_planes) and str_grab_planes[i] is not None:
                make_target(f"string_grab_{i}", str_grab_planes[i], RDK.Item(f"Cone_{i}"))
            else:
                print(f"  WARNING: Cone_{i} missing string_grab plane")

        print(f"Done -- {num_cones} cones in tree")
        a = cone_stl_paths[0] if cone_stl_paths else ""

    except Exception as e:
        print(f"RoboDK error: {e}")
        import traceback
        traceback.print_exc()
