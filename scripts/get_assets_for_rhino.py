import sys
sys.path.append(r'C:\RoboDK\Python')

from robodk import robolink, robomath
import Rhino.Geometry as rg
import os
import json
from datetime import datetime

PROJECT_DIR    = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods"
EXTRACTED_DIR  = os.path.join(PROJECT_DIR, "extracted_assets", "get_assets_for_rhino")

RDK = robolink.Robolink()

target_points = []
target_planes = []
target_names = []
tool_positions = []
tool_names = []
object_positions = []
object_names = []
path_points = []
time_total = 0
info = ""


def pose_to_plane(pose):
    x, y, z = pose[0,3], pose[1,3], pose[2,3]
    origin = rg.Point3d(x, y, z)
    x_axis = rg.Vector3d(pose[0,0], pose[1,0], pose[2,0])
    y_axis = rg.Vector3d(pose[0,1], pose[1,1], pose[2,1])
    return rg.Plane(origin, x_axis, y_axis)


def pose_to_point(pose):
    return rg.Point3d(pose[0,3], pose[1,3], pose[2,3])


def point_to_dict(p):
    return {"x": p.X, "y": p.Y, "z": p.Z}


def plane_to_dict(pl):
    return {
        "origin":  {"x": pl.Origin.X,  "y": pl.Origin.Y,  "z": pl.Origin.Z},
        "x_axis":  {"x": pl.XAxis.X,   "y": pl.XAxis.Y,   "z": pl.XAxis.Z},
        "y_axis":  {"x": pl.YAxis.X,   "y": pl.YAxis.Y,   "z": pl.YAxis.Z},
        "normal":  {"x": pl.Normal.X,  "y": pl.Normal.Y,  "z": pl.Normal.Z},
    }


if run:
    log = []

    # ---- 1. TARGETS ----
    targets = RDK.ItemList(robolink.ITEM_TYPE_TARGET)
    for t in targets:
        try:
            pose = t.Pose()
            target_points.append(pose_to_point(pose))
            target_planes.append(pose_to_plane(pose))
            target_names.append(t.Name())
        except:
            pass
    log.append(f"Targets: {len(target_points)}")

    # ---- 2. TOOLS ----
    tools = RDK.ItemList(robolink.ITEM_TYPE_TOOL)
    for t in tools:
        try:
            pose = t.PoseAbs()
            tool_positions.append(pose_to_point(pose))
            tool_names.append(t.Name())
        except:
            pass
    log.append(f"Tools: {len(tool_positions)}")

    # ---- 3. OBJECTS ----
    objects = RDK.ItemList(robolink.ITEM_TYPE_OBJECT)
    for o in objects:
        try:
            pose = o.PoseAbs()
            object_positions.append(pose_to_point(pose))
            object_names.append(o.Name())
        except:
            pass
    log.append(f"Objects: {len(object_positions)}")

    # ---- 4. PROGRAM PATH + TIMING ----
    programs = RDK.ItemList(robolink.ITEM_TYPE_PROGRAM)
    robots = RDK.ItemList(robolink.ITEM_TYPE_ROBOT)
    robot = robots[0] if robots else None

    for prog in programs:
        try:
            result = prog.InstructionListJoints(
                mm_step=2,
                deg_step=2,
                save_to_file=None,
                robot=robot,
                flags=0
            )
            data = result[0]
            for row in data.rows():
                t = row[12]
                if robot:
                    joints = robomath.Mat([row[:6]])
                    pose = robot.SolveFK(joints)
                    path_points.append(pose_to_point(pose))
                    time_total = max(time_total, t)
        except:
            pass
    log.append(f"Path points: {len(path_points)}")
    log.append(f"Total program time: {round(time_total, 2)}s")

    info = " | ".join(log)

    # ---- SAVE TO extracted_assets/get_assets_for_rhino/<timestamp>/ ----
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir = os.path.join(EXTRACTED_DIR, timestamp)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    payload = {
        "timestamp": timestamp,
        "info": info,
        "targets": [
            {"name": target_names[i], "point": point_to_dict(target_points[i]), "plane": plane_to_dict(target_planes[i])}
            for i in range(len(target_names))
        ],
        "tools": [
            {"name": tool_names[i], "position": point_to_dict(tool_positions[i])}
            for i in range(len(tool_names))
        ],
        "objects": [
            {"name": object_names[i], "position": point_to_dict(object_positions[i])}
            for i in range(len(object_names))
        ],
        "path": [point_to_dict(p) for p in path_points],
        "time_total_s": round(time_total, 4),
    }

    out_file = os.path.join(out_dir, "assets.json")
    with open(out_file, "w") as f:
        json.dump(payload, f, indent=2)

    info += f" | Saved to: {out_file}"

else:
    info = "Click run to fetch from RoboDK"
