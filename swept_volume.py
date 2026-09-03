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
    parser.add_argument("--export", type=str, default=None, help="Export swept volume as STL")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--sample-every", type=int, default=1, help="Show every Nth pose")
    parser.add_argument("--speed", type=float, default=0.0, help="Playback delay in seconds (0=all at once)")
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

    # Load a ghost robot for each sampled position
    ghost_ids = []
    positions_used = 0
    total_to_load = len([i for i in range(0, len(rows), args.sample_every)])

    for idx, row in enumerate(rows):
        if idx % args.sample_every != 0:
            continue

        joints_deg = [
            float(row["j1"]), float(row["j2"]), float(row["j3"]),
            float(row["j4"]), float(row["j5"]), float(row["j6"]),
        ]
        j7_mm = float(row["j7"])

        # Convert to radians for revolute, meters for prismatic
        joint_values = [j7_mm / 1000.0]  # j7 prismatic in meters
        joint_values.extend([math.radians(j) for j in joints_deg])

        # Load a new robot instance for this pose
        ghost = p.loadURDF(args.urdf, [0, 0, 0], useFixedBase=True,
                           flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

        # Set joint positions
        for ji, mi in enumerate(movable_joints):
            if ji < len(joint_values):
                p.resetJointState(ghost, mi, joint_values[ji])

        # Disable dynamics so ghosts don't move
        for link_idx in range(-1, n_pybullet_joints):
            p.changeDynamics(ghost, link_idx, mass=0)
            try:
                p.changeVisualShape(ghost, link_idx, rgbaColor=[0.3, 0.5, 0.8, 0.15])
            except Exception:
                pass

        ghost_ids.append(ghost)
        positions_used += 1

        if positions_used % 5 == 0 or positions_used == total_to_load:
            print(f"  [{positions_used}/{total_to_load}] loaded", flush=True)

        if args.speed > 0 and not args.headless:
            p.stepSimulation()
            time.sleep(args.speed)

    print(f"[INFO] Rendered {positions_used} ghost robots")

    # Export swept volume mesh if requested
    if args.export:
        try:
            import trimesh
            all_vertices = []

            for ghost_id in ghost_ids:
                for link_idx in range(-1, n_pybullet_joints):
                    try:
                        mesh_data = p.getMeshData(ghost_id, link_idx)
                        if mesh_data and len(mesh_data[1]) > 0:
                            verts = np.array(mesh_data[1])
                            all_vertices.append(verts)
                    except Exception:
                        pass

            if all_vertices:
                all_verts = np.vstack(all_vertices)
                cloud = trimesh.PointCloud(all_verts)
                hull = cloud.convex_hull
                hull.export(args.export)
                print(f"[INFO] Swept volume exported to: {args.export}")
                print(f"       Volume: {hull.volume:.4f} m^3")
            else:
                print("[WARN] No mesh data available for export")
        except ImportError:
            print("[WARN] trimesh/scipy not installed — cannot export STL")

    if not args.headless:
        p.resetDebugVisualizerCamera(4.0, 45, -30, [0, 0, 1.0])
        print(f"\n[INFO] Middle mouse = orbit, scroll = zoom, Ctrl+middle = pan")
        print(f"[INFO] Close window or Ctrl+C to exit")
        try:
            while True:
                time.sleep(1/30)  # just redraw, no physics stepping
        except KeyboardInterrupt:
            pass

    p.disconnect()


if __name__ == "__main__":
    main()
