"""
Visualize robot swept volume using PyBullet with actual robot geometry.

Loads a URDF, replays joint positions from a CSV, and renders the robot
at every recorded position as a transparent ghost.

Usage:
    # Step 1 (Windows, needs RoboDK): export robot meshes + generate URDF
    py -3.12 export_robot_urdf.py

    # Step 2 (WSL or Windows): visualize swept volume
    python swept_volume.py robo_dk_output/joint_recordings/joints_*_j7zero.csv
    python swept_volume.py --urdf robot_urdf/fanuc_r2000ic.urdf joints.csv
    python swept_volume.py --export swept_volume.stl joints.csv
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

RECORDINGS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "robo_dk_output", "joint_recordings"
)
DEFAULT_URDF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "robot_urdf", "fanuc_r2000ic.urdf"
)


def find_latest_j7zero_csv():
    if not os.path.isdir(RECORDINGS_DIR):
        return None
    csvs = sorted(Path(RECORDINGS_DIR).glob("joints_*_j7zero.csv"))
    return str(csvs[-1]) if csvs else None


def read_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description="Visualize robot swept volume")
    parser.add_argument("csv_path", nargs="?", default=None)
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="Path to robot URDF")
    parser.add_argument("--mesh-dir", default=None, help="Directory with link mesh files (overrides URDF visual data)")
    parser.add_argument("--export", type=str, default=None, help="Export swept volume as STL")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--sample-every", type=int, default=1, help="Show every Nth pose")
    parser.add_argument("--voxel-mm", type=float, default=10.0, help="Voxel resolution in mm for watertight remesh (default 10)")
    args = parser.parse_args()

    csv_path = args.csv_path or find_latest_j7zero_csv()
    if not csv_path or not os.path.exists(csv_path):
        print("[ERROR] No recording found.")
        sys.exit(1)

    if not os.path.exists(args.urdf):
        print(f"[ERROR] URDF not found at: {args.urdf}")
        print(f"        Run first (Windows): py -3.12 export_robot_urdf.py")
        sys.exit(1)

    rows = read_csv(csv_path)
    if not rows:
        print("[ERROR] CSV is empty.")
        sys.exit(1)

    print(f"[INFO] Loading {len(rows)} positions from {os.path.basename(csv_path)}")
    print(f"[INFO] Using URDF: {args.urdf}")

    import pybullet as p
    import pybullet_data

    if args.headless:
        client = p.connect(p.DIRECT)
    else:
        client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    p.setPhysicsEngineParameter(enableConeFriction=0, numSolverIterations=0)
    p.loadURDF("plane.urdf", [0, 0, -0.01])

    # Load the robot URDF once to figure out joint mapping
    urdf_dir = os.path.dirname(os.path.abspath(args.urdf))
    test_robot = p.loadURDF(args.urdf, [0, 0, 0], useFixedBase=True,
                            flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

    n_pybullet_joints = p.getNumJoints(test_robot)
    movable_joints = []
    for i in range(n_pybullet_joints):
        info = p.getJointInfo(test_robot, i)
        joint_type = info[2]
        if joint_type != p.JOINT_FIXED:
            movable_joints.append(i)
            print(f"  Joint {i}: {info[1].decode()} type={joint_type}")

    print(f"[INFO] URDF has {len(movable_joints)} movable joints")
    p.removeBody(test_robot)

    import trimesh

    # Load STL meshes directly with trimesh (indexed by PyBullet link index)
    # PyBullet visual data tells us which mesh file belongs to which link
    fk_robot = p.loadURDF(args.urdf, [0, 0, 0], useFixedBase=True,
                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    for li in range(-1, n_pybullet_joints):
        p.changeDynamics(fk_robot, li, mass=0)

    # Load mesh for each link
    # Map: PyBullet link index -> mesh file name
    # URDF visual data gives us: link 1=base, 2=j1, 3=j2, ... 7=j6
    LINK_MESH_NAMES = {1: "base", 2: "j1", 3: "j2", 4: "j3", 5: "j4", 6: "j5", 7: "j6"}

    mesh_dir = args.mesh_dir or os.path.join(os.path.dirname(os.path.abspath(args.urdf)), "meshes")

    link_base_meshes = {}
    for link_idx, name in LINK_MESH_NAMES.items():
        # Try DAE first, then STL
        for ext in [".dae", ".stl"]:
            candidate = os.path.join(mesh_dir, name + ext)
            if os.path.exists(candidate):
                try:
                    m = trimesh.load(candidate, force='mesh')
                    # Get visual frame offset from URDF
                    visual_data = p.getVisualShapeData(fk_robot)
                    for vis in visual_data:
                        if vis[1] == link_idx:
                            T = np.eye(4)
                            T[:3, 3] = vis[5]
                            T[:3, :3] = np.array(p.getMatrixFromQuaternion(vis[6])).reshape(3, 3)
                            m.apply_transform(T)
                            break
                    link_base_meshes[link_idx] = m
                    print(f"  Loaded link {link_idx}: {name}{ext} ({len(m.vertices)} verts)")
                    break
                except Exception as e:
                    print(f"  [WARN] {candidate}: {e}")

    print(f"[INFO] Loaded {len(link_base_meshes)} link meshes from disk")

    if not link_base_meshes:
        print("[ERROR] No link meshes loaded.")
        p.disconnect()
        sys.exit(1)

    import manifold3d

    # Voxel-remesh each link mesh to make it watertight, extract outer shell,
    # simplify, then convert to manifold — all per-link before the union loop
    pitch = args.voxel_mm / 1000.0  # mm to meters
    target_faces_per_link = 3000
    print(f"[INFO] Voxel remeshing links at {args.voxel_mm}mm, simplifying to ~{target_faces_per_link} faces/link...")
    link_manifolds = {}
    for link_idx, m in link_base_meshes.items():
        try:
            # Step 1a: voxelize and marching cubes
            vox = m.voxelized(pitch=pitch)
            watertight = vox.marching_cubes
            watertight.apply_scale(vox.pitch[0])
            watertight.apply_translation(vox.transform[:3, 3])

            # Step 1b: keep only outer shell
            components = watertight.split(only_watertight=False)
            if len(components) > 1:
                watertight = max(components, key=lambda c: c.volume if c.is_watertight else c.area)

            # Step 1c: convert to manifold and simplify
            man = manifold3d.Manifold(manifold3d.Mesh(
                vert_properties=np.array(watertight.vertices, dtype=np.float32),
                tri_verts=np.array(watertight.faces, dtype=np.uint32),
            ))
            if man.is_empty():
                print(f"  Link {link_idx}: manifold empty, skipping")
                continue

            if man.num_tri() > target_faces_per_link:
                man = man.simplify(target_faces_per_link)

            link_manifolds[link_idx] = man
            print(f"  Link {link_idx}: {man.num_vert()} verts, {man.num_tri()} tris")
        except Exception as e:
            print(f"  Link {link_idx}: failed ({e}), skipping")

    print(f"[INFO] {len(link_manifolds)} links ready for boolean union")

    # For each pose: set joints, get link world transforms, transform and union
    total_to_process = len([i for i in range(0, len(rows), args.sample_every)])
    positions_used = 0
    skipped = 0
    running_union = None
    seen_joints = set()

    for idx, row in enumerate(rows):
        if idx % args.sample_every != 0:
            continue

        joints_deg = [
            float(row["j1"]), float(row["j2"]), float(row["j3"]),
            float(row["j4"]), float(row["j5"]), float(row["j6"]),
        ]
        j7_mm = float(row["j7"])

        cache_key = tuple(joints_deg) + (j7_mm,)
        if cache_key in seen_joints:
            skipped += 1
            positions_used += 1
            if positions_used % 5 == 0 or positions_used == total_to_process:
                print(f"  [{positions_used}/{total_to_process}] ({skipped} skipped as duplicates)", flush=True)
            continue
        seen_joints.add(cache_key)

        joint_values = [j7_mm / 1000.0]
        joint_values.extend([math.radians(j) for j in joints_deg])

        # Set joints in PyBullet for FK
        for ji, mi in enumerate(movable_joints):
            if ji < len(joint_values):
                p.resetJointState(fk_robot, mi, joint_values[ji])

        # Transform each link manifold to world pose and union into this pose's shape
        pose_union = None
        for link_idx, base_man in link_manifolds.items():
            if link_idx == -1:
                pos, orn = p.getBasePositionAndOrientation(fk_robot)
            else:
                state = p.getLinkState(fk_robot, link_idx)
                pos = state[4]
                orn = state[5]

            rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            T = np.zeros((3, 4), dtype=np.float64)
            T[:3, :3] = rot
            T[:3, 3] = pos
            transformed = base_man.transform(T)

            if pose_union is None:
                pose_union = transformed
            else:
                pose_union = pose_union + transformed

        # Union this pose into the running total
        if pose_union is not None:
            if running_union is None:
                running_union = pose_union
            else:
                running_union = running_union + pose_union

        positions_used += 1
        if positions_used % 5 == 0 or positions_used == total_to_process:
            unique = len(seen_joints)
            verts = running_union.num_vert() if running_union else 0
            print(f"  [{positions_used}/{total_to_process}] {unique} unique poses unioned ({verts} verts), {skipped} skipped", flush=True)

    p.disconnect()

    if running_union is None or running_union.is_empty():
        print("[ERROR] Boolean union produced empty result.")
        sys.exit(1)

    # Convert back to trimesh
    mesh_out = running_union.to_mesh()
    combined = trimesh.Trimesh(
        vertices=np.array(mesh_out.vert_properties, dtype=np.float64),
        faces=np.array(mesh_out.tri_verts, dtype=np.int64),
    )
    print(f"[INFO] Final mesh: {len(combined.vertices)} vertices, {len(combined.faces)} faces")

    bounds = combined.bounds
    print(f"[INFO] Bounds (meters):")
    print(f"       X: {bounds[0][0]:.3f} to {bounds[1][0]:.3f} ({bounds[1][0]-bounds[0][0]:.3f}m)")
    print(f"       Y: {bounds[0][1]:.3f} to {bounds[1][1]:.3f} ({bounds[1][1]-bounds[0][1]:.3f}m)")
    print(f"       Z: {bounds[0][2]:.3f} to {bounds[1][2]:.3f} ({bounds[1][2]-bounds[0][2]:.3f}m)")

    export_path = args.export or "swept_volume.stl"
    combined.export(export_path)
    print(f"[INFO] Swept volume exported to: {export_path}")
    print(f"[DONE] Open in Rhino: {os.path.abspath(export_path)}")


if __name__ == "__main__":
    main()
