from Grasshopper import DataTree
import Rhino
import Rhino.Geometry as rg
import scriptcontext as sc
import System
import struct
import os
import sys

# ---------------------------------------------------------------------------
# Script 2 — End effector with moving joint and 4 auto-computed TCP tools
# GH inputs:
#   static_geometry    : breps/meshes for static part of EE
#   moving_geometry    : breps/meshes for moving part of EE
#   flange_plane       : plane at robot flange (tool0) origin
#   joint_axis_plane   : plane at pivot point, Z = rotation axis
#   joint_open_angle   : float (degrees) — open position
#   joint_closed_angle : float (degrees) — closed position
#   import_angle       : float (degrees) — display angle in RoboDK
#   reference_angle    : float (degrees) — angle at which pickup/knotting planes are defined
#   pickup_plane       : plane at pickup TCP at reference_angle
#   knotting_plane     : plane at knotting TCP at reference_angle
#   pickup_on_moving   : bool — True if pickup is on the moving part
#   knotting_on_moving : bool — True if knotting is on the moving part
#   trigger            : bool
#
# The script auto-computes open and closed positions by rotating
# pickup_plane and knotting_plane around the joint axis.
# ---------------------------------------------------------------------------

if trigger:
    sc.sticky["last_ee_stl_path"] = ""
    print("Trigger activated, forcing re-run")

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

def resolve_plane(item):
    if item is None:
        return None
    if isinstance(item, rg.Plane):
        return item
    if isinstance(item, System.Guid):
        rhino_obj = sc.doc.Objects.FindId(item)
        if rhino_obj:
            geo = rhino_obj.Geometry
            if isinstance(geo, rg.Plane):
                return geo
            if hasattr(geo, 'FrameAt'):
                ok, frame = geo.FrameAt(0, 0)
                if ok:
                    return frame
            centre = geo.GetBoundingBox(True).Center
            return rg.Plane(centre, rg.Vector3d.ZAxis)
        print(f"  Could not find Guid: {item}")
        return None
    if isinstance(item, rg.Point3d):
        return rg.Plane(item, rg.Vector3d.ZAxis)
    if isinstance(item, rg.Point):
        return rg.Plane(item.Location, rg.Vector3d.ZAxis)
    if hasattr(item, 'X') and hasattr(item, 'Y') and hasattr(item, 'Z'):
        return rg.Plane(rg.Point3d(item.X, item.Y, item.Z), rg.Vector3d.ZAxis)
    print(f"  Unknown plane type: {type(item)}")
    return None

def resolve_first_plane(gh_input):
    items = flatten_input(gh_input, exclude_type=rg.Plane)
    if not items:
        return resolve_plane(gh_input)
    for item in items:
        pl = resolve_plane(item)
        if pl is not None:
            return pl
    return None

def resolve_bool(gh_input):
    if gh_input is None:
        return None
    items = flatten_input(gh_input)
    if items:
        return bool(items[0])
    return None

def resolve_float(gh_input):
    if gh_input is None:
        return None
    if isinstance(gh_input, (int, float)):
        return float(gh_input)
    if isinstance(gh_input, str):
        try:
            return float(gh_input.strip())
        except ValueError:
            return None
    items = flatten_input(gh_input)
    if items:
        val = items[0]
        if isinstance(val, str):
            try:
                return float(val.strip())
            except ValueError:
                return None
        try:
            return float(val)
        except (TypeError, ValueError):
            return None
    return None

def build_mesh_from_geo_list(geo_list):
    mesh_list = rg.Mesh()
    for geo in geo_list:
        if isinstance(geo, rg.Mesh):
            geo.Faces.ConvertQuadsToTriangles()
            mesh_list.Append(geo)
        elif isinstance(geo, rg.Brep):
            meshes = rg.Mesh.CreateFromBrep(geo, rg.MeshingParameters.Coarse)
            if meshes:
                for m in meshes:
                    m.Faces.ConvertQuadsToTriangles()
                    mesh_list.Append(m)
        elif isinstance(geo, System.Guid):
            rhino_obj = sc.doc.Objects.FindId(geo)
            if rhino_obj:
                obj_geo = rhino_obj.Geometry
                if isinstance(obj_geo, rg.Mesh):
                    obj_geo.Faces.ConvertQuadsToTriangles()
                    mesh_list.Append(obj_geo)
                elif isinstance(obj_geo, rg.Brep):
                    meshes = rg.Mesh.CreateFromBrep(obj_geo, rg.MeshingParameters.Coarse)
                    if meshes:
                        for m in meshes:
                            m.Faces.ConvertQuadsToTriangles()
                            mesh_list.Append(m)
    mesh_list.Faces.ConvertQuadsToTriangles()
    return mesh_list

def write_stl(mesh, stl_path):
    triangles = []
    for i in range(mesh.Faces.Count):
        face = mesh.Faces[i]
        triangles.append((
            mesh.Vertices[face.A],
            mesh.Vertices[face.B],
            mesh.Vertices[face.C]
        ))
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

def write_minimal_stl(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', 1))
        f.write(struct.pack('<fff', 0, 0, 1))
        for _ in range(3):
            f.write(struct.pack('<fff', 0, 0, 0))
        f.write(struct.pack('<H', 0))

# ---------------------------------------------------------------------------
# Collect inputs
# ---------------------------------------------------------------------------
static_geo_list   = flatten_input(static_geometry)           if 'static_geometry'   in dir() else []
moving_geo_list   = flatten_input(moving_geometry)           if 'moving_geometry'   in dir() else []
resolved_flange   = resolve_first_plane(flange_plane)        if 'flange_plane'      in dir() else None
resolved_axis     = resolve_first_plane(joint_axis_plane)    if 'joint_axis_plane'  in dir() else None
resolved_pickup   = resolve_first_plane(pickup_plane)        if 'pickup_plane'      in dir() else None
resolved_knot     = resolve_first_plane(knotting_plane)      if 'knotting_plane'    in dir() else None
pickup_moving     = resolve_bool(pickup_on_moving   if 'pickup_on_moving'   in dir() else None)
knotting_moving   = resolve_bool(knotting_on_moving if 'knotting_on_moving' in dir() else None)
open_angle        = resolve_float(joint_open_angle   if 'joint_open_angle'   in dir() else None)
closed_angle      = resolve_float(joint_closed_angle if 'joint_closed_angle' in dir() else None)
import_angle_val  = resolve_float(import_angle       if 'import_angle'       in dir() else None)
ref_angle_val     = resolve_float(reference_angle    if 'reference_angle'    in dir() else None)

# Defaults
open_angle       = open_angle       if open_angle       is not None else 0.0
closed_angle     = closed_angle     if closed_angle     is not None else 90.0
import_angle_val = import_angle_val if import_angle_val is not None else 0.0
ref_angle_val    = ref_angle_val    if ref_angle_val    is not None else 0.0

print(f"Static geo items   : {len(static_geo_list)}")
print(f"Moving geo items   : {len(moving_geo_list)}")
print(f"flange_plane       : {resolved_flange}")
print(f"joint_axis_plane   : {resolved_axis}")
print(f"open_angle         : {open_angle} deg")
print(f"closed_angle       : {closed_angle} deg")
print(f"import_angle       : {import_angle_val} deg")
print(f"reference_angle    : {ref_angle_val} deg")
print(f"pickup_plane       : {resolved_pickup}")
print(f"knotting_plane     : {resolved_knot}")
print(f"pickup_on_moving   : {pickup_moving}")
print(f"knotting_on_moving : {knotting_moving}")

# ---------------------------------------------------------------------------
# Gate
# ---------------------------------------------------------------------------
if not trigger and sc.sticky.get("last_ee_stl_path", "") != "":
    a = sc.sticky.get("last_ee_stl_path", "")
    print("Trigger not set — skipping.")
else:
    print("Rebuilding...")

    static_stl_path = "C:/temp/end_effector_static.stl"
    moving_stl_path = "C:/temp/end_effector_moving.stl"

    if static_geo_list:
        static_mesh = build_mesh_from_geo_list(static_geo_list)
        static_tris = write_stl(static_mesh, static_stl_path)
        print(f"Static STL: {static_tris} triangles -> {static_stl_path}")
    else:
        print("WARNING: no static geometry")

    if moving_geo_list:
        moving_mesh = build_mesh_from_geo_list(moving_geo_list)
        moving_tris = write_stl(moving_mesh, moving_stl_path)
        print(f"Moving STL: {moving_tris} triangles -> {moving_stl_path}")
    else:
        print("WARNING: no moving geometry")

    try:
        sys.path.append("C:/RoboDK/Python")
        from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
        from robodk.robomath import Mat, invH, rotz, pi

        def identity():
            return Mat([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        def plane_to_mat(plane):
            x = plane.XAxis
            y = plane.YAxis
            z = plane.Normal
            o = plane.Origin
            return Mat([
                [x.X, y.X, z.X, o.X],
                [x.Y, y.Y, z.Y, o.Y],
                [x.Z, y.Z, z.Z, o.Z],
                [0,   0,   0,   1  ],
            ])

        def offset_mat(from_plane, to_plane):
            """Pose of to_plane expressed in from_plane frame."""
            return invH(plane_to_mat(from_plane)) * plane_to_mat(to_plane)

        RDK = Robolink()
        station = RDK.Item("", 1)
        print(f"Station: {station.Name()}")

        def purge_all(name):
            count = 0
            for _ in range(50):
                item = RDK.Item(name)
                if not item.Valid():
                    break
                item.Delete()
                count += 1
            if count:
                print(f"  purged {count}x '{name}'")

        def add_file_and_get_item(path):
            before = set(it.item for it in RDK.ItemList(5, False))
            RDK.AddFile(path)
            for it in RDK.ItemList(5, False):
                if it.item not in before:
                    print(f"  -> loaded '{it.Name()}' id={it.item}")
                    return it
            stem = os.path.splitext(os.path.basename(path))[0]
            fb = RDK.Item(stem)
            print(f"  -> fallback '{stem}' valid={fb.Valid()}")
            return fb

        # Purge old items
        for name in ["end_effector_static", "end_effector_moving", "end_effector",
                     "EndEffector", "MovingPart",
                     "pickup_open", "pickup_closed",
                     "knotting_open", "knotting_closed",
                     "pickup_point", "knotting_point"]:
            purge_all(name)

        # --- Find the actual 6-axis robot arm, not the rail mechanism ---
        robot = None
        for rname in ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]:
            r = RDK.Item(rname, ITEM_TYPE_ROBOT)
            if r.Valid():
                robot = r
                break
        if robot is None:
            # Last resort: find any robot that isn't a mechanism
            for r in RDK.ItemList(ITEM_TYPE_ROBOT, False):
                if "mechanism" not in r.Name().lower() and "door" not in r.Name().lower():
                    robot = r
                    print(f"  Using robot '{robot.Name()}' (fallback)")
                    break
        if robot is None:
            robot = RDK.Item("", ITEM_TYPE_ROBOT)
            print(f"  WARNING: using '{robot.Name()}' as robot (may be mechanism)")
        print(f"Robot: {robot.Name()}")

        ee_parent = robot if (robot is not None and robot.Valid()) else station

        # EE organisational frame
        ee_frame = RDK.AddFrame("EndEffector", ee_parent)
        ee_frame.setPose(identity())

        # Static geometry
        if static_geo_list:
            static_obj = add_file_and_get_item(static_stl_path)
            if static_obj.Valid():
                static_obj.setColor([0.6, 0.6, 0.65, 1.0])
                static_obj.setParent(ee_frame)
                print("Static EE: ok")

        # Flange plane — default to world origin XY if not connected
        fo = resolved_flange if resolved_flange is not None else rg.Plane(
            rg.Point3d(0, 0, 0), rg.Vector3d.ZAxis)

        # Moving part frame
        if resolved_axis is not None:
            axis_offset  = offset_mat(fo, resolved_axis)
            import_rad   = import_angle_val * pi / 180.0
            moving_frame = RDK.AddFrame("MovingPart", ee_frame)
            moving_frame.setPose(axis_offset * rotz(import_rad))

            if moving_geo_list:
                moving_obj = add_file_and_get_item(moving_stl_path)
                if moving_obj.Valid():
                    moving_obj.setColor([0.6, 0.6, 0.65, 1.0])
                    moving_obj.setParent(moving_frame)
                    moving_obj.setPose(invH(axis_offset))
                    print("Moving EE: ok")

            angle_name = "MovingPart|open=" + str(open_angle) + "|closed=" + str(closed_angle) + "|import=" + str(import_angle_val)
            moving_frame.setName(angle_name)
            print("Joint angles encoded: " + angle_name)
        else:
            print("WARNING: no joint_axis_plane — moving part not created")

        # ── Tool creation ────────────────────────────────────────────────────
        def compute_tcp_offset(tcp_plane, on_moving, target_angle_deg):
            if not on_moving or resolved_axis is None:
                return offset_mat(fo, tcp_plane)
            else:
                delta_rad   = (target_angle_deg - ref_angle_val) * pi / 180.0
                flange_mat  = plane_to_mat(fo)
                axis_mat    = plane_to_mat(resolved_axis)
                tcp_mat     = plane_to_mat(tcp_plane)
                tcp_in_axis = invH(axis_mat) * tcp_mat
                rotated_tcp = rotz(delta_rad) * tcp_in_axis
                return invH(flange_mat) * axis_mat * rotated_tcp

        def create_tool(name, tcp_plane, on_moving, target_angle_deg):
            if tcp_plane is None:
                print(f"WARNING: {name} — no plane, skipping")
                return
            purge_all(name)
            offset = compute_tcp_offset(tcp_plane, on_moving, target_angle_deg)
            stl = "C:/temp/" + name + ".stl"
            write_minimal_stl(stl)
            tool = RDK.AddFile(stl, robot)
            tool.setName(name)
            tool.setPoseTool(offset)
            check = tool.PoseTool()
            print(name + " offset: (" + str(round(offset[0,3],1)) + ", " + str(round(offset[1,3],1)) + ", " + str(round(offset[2,3],1)) + ")")
            print(name + " readback: (" + str(round(check[0,3],1)) + ", " + str(round(check[1,3],1)) + ", " + str(round(check[2,3],1)) + ")")

        pm  = pickup_moving   or False
        km  = knotting_moving or False

        create_tool("pickup_open",     resolved_pickup, pm, open_angle)
        create_tool("pickup_closed",   resolved_pickup, pm, closed_angle)
        create_tool("knotting_open",   resolved_knot,   km, open_angle)
        create_tool("knotting_closed", resolved_knot,   km, closed_angle)

        print("Done")

    except Exception as e:
        print(f"RoboDK error: {e}")
        import traceback
        traceback.print_exc()

    sc.sticky["last_ee_stl_path"] = static_stl_path
    a = static_stl_path
