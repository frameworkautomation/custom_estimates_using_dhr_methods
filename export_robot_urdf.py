"""
Export robot link meshes from RoboDK and generate a URDF for PyBullet.

Connects to a running RoboDK instance, exports each robot link as STL,
and writes a URDF file that PyBullet can load.

Usage (Windows PowerShell, RoboDK must be running):
    py -3.12 export_robot_urdf.py
    py -3.12 export_robot_urdf.py --output-dir robot_urdf
"""

import argparse
import math
import os
import sys


def main():
    parser = argparse.ArgumentParser(description="Export robot from RoboDK as URDF")
    parser.add_argument("--output-dir", default="robot_urdf", help="Output directory")
    parser.add_argument("--robodk-ip", default="localhost")
    parser.add_argument("--robodk-port", type=int, default=20502)
    args = parser.parse_args()

    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_OBJECT
    from robodk.robomath import Pose_2_TxyzRxyz

    rdk = Robolink(args.robodk_ip, args.robodk_port)
    print(f"[INFO] Connected to RoboDK")

    robot = rdk.ItemUserPick("Select robot", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print("[ERROR] No robot selected")
        sys.exit(1)

    robot_name = robot.Name()
    n_joints = len(robot.Joints().list())
    print(f"[INFO] Robot: {robot_name} ({n_joints} joints)")

    # Create output directories
    mesh_dir = os.path.join(args.output_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    # Get all robot links by traversing the robot tree
    def get_links_recursive(item, depth=0):
        """Get all child items of the robot."""
        links = []
        children = item.Childs()
        for child in children:
            item_type = child.Type()
            name = child.Name()
            links.append({
                "item": child,
                "name": name,
                "type": item_type,
                "depth": depth,
            })
            links.extend(get_links_recursive(child, depth + 1))
        return links

    all_items = get_links_recursive(robot)
    print(f"[INFO] Found {len(all_items)} child items in robot tree")

    # Export each item that has geometry
    exported = []
    for info in all_items:
        item = info["item"]
        name = info["name"]
        safe_name = name.replace(" ", "_").replace("/", "_").replace("\\", "_")
        stl_path = os.path.join(mesh_dir, f"{safe_name}.stl")
        abs_stl = os.path.abspath(stl_path)

        try:
            # Try exporting as STL
            result = item.setParam("Export", abs_stl)
            if os.path.exists(stl_path) and os.path.getsize(stl_path) > 100:
                size = os.path.getsize(stl_path)
                print(f"  [OK] {name} -> {safe_name}.stl ({size:,} bytes)")
                exported.append({"name": name, "safe_name": safe_name, "stl": stl_path})
            else:
                # Try alternate export method
                try:
                    item.Save(abs_stl)
                    if os.path.exists(stl_path) and os.path.getsize(stl_path) > 100:
                        size = os.path.getsize(stl_path)
                        print(f"  [OK] {name} -> {safe_name}.stl ({size:,} bytes)")
                        exported.append({"name": name, "safe_name": safe_name, "stl": stl_path})
                    else:
                        print(f"  [SKIP] {name} (no geometry)")
                except Exception:
                    print(f"  [SKIP] {name} (no geometry)")
        except Exception as e:
            print(f"  [SKIP] {name} ({e})")

    if not exported:
        print("\n[WARN] No meshes exported. Trying direct robot link export...")
        # Alternative: use robot joints to identify links
        # Export the robot item itself
        robot_stl = os.path.join(mesh_dir, "robot_full.stl")
        try:
            robot.setParam("Export", os.path.abspath(robot_stl))
            if os.path.exists(robot_stl):
                size = os.path.getsize(robot_stl)
                print(f"  [OK] Full robot -> robot_full.stl ({size:,} bytes)")
                exported.append({"name": "robot_full", "safe_name": "robot_full", "stl": robot_stl})
        except Exception as e:
            print(f"  [FAIL] {e}")

    # Also try to get joint info from RoboDK
    print(f"\n[INFO] Robot joint limits:")
    try:
        lower_limits, upper_limits = robot.JointLimits()
        lower = lower_limits.list() if hasattr(lower_limits, 'list') else list(lower_limits)
        upper = upper_limits.list() if hasattr(upper_limits, 'list') else list(upper_limits)
        for i, (lo, hi) in enumerate(zip(lower, upper)):
            jtype = "prismatic" if i == n_joints - 1 else "revolute"
            print(f"  J{i+1}: [{lo:.1f}, {hi:.1f}] ({jtype})")
    except Exception as e:
        print(f"  Could not get limits: {e}")
        lower = [-180] * n_joints
        upper = [180] * n_joints

    # Write a simple URDF
    urdf_path = os.path.join(args.output_dir, "fanuc_r2000ic.urdf")
    write_urdf(urdf_path, mesh_dir, exported, n_joints, lower, upper)
    print(f"\n[DONE] URDF written to: {urdf_path}")
    print(f"       Meshes in: {mesh_dir}/")
    print(f"       {len(exported)} mesh files exported")


# DH parameters for Fanuc R-2000iC 125L
FANUC_DH = [
    # a(mm), alpha(rad), d(mm), theta_offset(rad)
    (312.0,  -math.pi/2,  780.0,  0.0),
    (1075.0,  0.0,           0.0, -math.pi/2),
    (225.0,  -math.pi/2,    0.0,   0.0),
    (0.0,     math.pi/2,  1280.0,  0.0),
    (0.0,    -math.pi/2,    0.0,   0.0),
    (0.0,     0.0,          215.0,  0.0),
]


def write_urdf(path, mesh_dir, exported_meshes, n_joints, lower_limits, upper_limits):
    """Write a URDF file for the robot."""

    # Map exported meshes to links if possible
    mesh_files = {e["safe_name"]: e["stl"] for e in exported_meshes}

    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append('<robot name="fanuc_r2000ic_125l">')

    # World link (fixed)
    lines.append('  <link name="world"/>')

    # Rail joint (prismatic, j7)
    lines.append('  <joint name="j7_rail" type="prismatic">')
    lines.append('    <parent link="world"/>')
    lines.append('    <child link="rail_cart"/>')
    lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('    <axis xyz="1 0 0"/>')
    lo = lower_limits[6] / 1000.0 if len(lower_limits) > 6 else 0.0
    hi = upper_limits[6] / 1000.0 if len(upper_limits) > 6 else 20.0
    lines.append(f'    <limit lower="{lo}" upper="{hi}" effort="1000" velocity="1.0"/>')
    lines.append('  </joint>')

    # Rail cart link
    lines.append('  <link name="rail_cart">')
    lines.append('    <visual><geometry><box size="0.5 0.5 0.1"/></geometry>')
    lines.append('      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>')
    lines.append('    </visual>')
    lines.append('  </link>')

    # 6 arm links + joints
    link_names = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

    prev_link = "rail_cart"
    for i in range(6):
        a, alpha, d, theta_off = FANUC_DH[i]
        joint_name = joint_names[i]
        link_name = link_names[i + 1]

        # Convert DH to URDF origin (approximate)
        x = a / 1000.0
        y = 0.0
        z = d / 1000.0

        if i == 0:
            # First joint sits on top of rail cart
            parent = prev_link
        else:
            parent = link_names[i]

        lines.append(f'  <joint name="{joint_name}" type="revolute">')
        lines.append(f'    <parent link="{parent}"/>')
        lines.append(f'    <child link="{link_name}"/>')
        lines.append(f'    <origin xyz="{x:.4f} {y:.4f} {z:.4f}" rpy="{alpha:.4f} 0 {theta_off:.4f}"/>')
        lines.append(f'    <axis xyz="0 0 1"/>')
        lo_j = math.radians(lower_limits[i]) if i < len(lower_limits) else -math.pi
        hi_j = math.radians(upper_limits[i]) if i < len(upper_limits) else math.pi
        lines.append(f'    <limit lower="{lo_j:.4f}" upper="{hi_j:.4f}" effort="1000" velocity="2.0"/>')
        lines.append(f'  </joint>')

        # Link with visual
        # Use exported mesh if available, otherwise a simple cylinder
        mesh_found = False
        for ename, epath in mesh_files.items():
            if f"link{i+1}" in ename.lower() or f"j{i+1}" in ename.lower():
                rel_path = os.path.relpath(epath, os.path.dirname(os.path.abspath(path)))
                lines.append(f'  <link name="{link_name}">')
                lines.append(f'    <visual><geometry>')
                lines.append(f'      <mesh filename="{rel_path}" scale="0.001 0.001 0.001"/>')
                lines.append(f'    </geometry></visual>')
                lines.append(f'  </link>')
                mesh_found = True
                break

        if not mesh_found:
            # Fallback: colored cylinder
            colors = [
                "0.7 0.0 0.0", "0.0 0.7 0.0", "0.0 0.0 0.7",
                "0.7 0.7 0.0", "0.7 0.0 0.7", "0.0 0.7 0.7",
            ]
            # Approximate link length from DH
            length = max(0.1, math.sqrt(a**2 + d**2) / 1000.0)
            radius = 0.08 if i < 3 else 0.05
            lines.append(f'  <link name="{link_name}">')
            lines.append(f'    <visual>')
            lines.append(f'      <origin xyz="0 0 {length/2:.4f}" rpy="0 0 0"/>')
            lines.append(f'      <geometry><cylinder radius="{radius}" length="{length:.4f}"/></geometry>')
            lines.append(f'      <material name="color{i}"><color rgba="{colors[i]} 0.7"/></material>')
            lines.append(f'    </visual>')
            lines.append(f'    <inertial>')
            lines.append(f'      <mass value="1.0"/>')
            lines.append(f'      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>')
            lines.append(f'    </inertial>')
            lines.append(f'  </link>')

        prev_link = link_name

    lines.append('</robot>')

    with open(path, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()
