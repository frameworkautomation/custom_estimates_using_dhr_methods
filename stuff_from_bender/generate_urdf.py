"""Generate URDF from SolidWorks constraint JSON.

Reads pipeline/constraints/<assembly>.json and produces a standard .urdf file
that can be loaded by ROS, Gazebo, PyBullet, or Three.js (via urdf-loader).

Usage:
    python3 pipeline/urdf/generate_urdf.py \
        --input pipeline/constraints/fanuc_robot.json \
        --output pipeline/urdf/fanuc_robot.urdf \
        --meshes pipeline/urdf/meshes/
"""
import json
import math
import os
import sys
import argparse
from xml.etree.ElementTree import Element, SubElement, tostring, ElementTree
from xml.dom.minidom import parseString

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from sw_scripts.lib.mate_classifier import Mate, classify_mate_group


def reclassify_unknown_joints(data):
    """Re-run the current mate_classifier against stored joints.

    Constraint JSONs snapshot classifier output from the SW inspection run.
    When the classifier is improved, existing JSONs still carry the old
    verdicts. This rebuilds Mate objects from raw_mates for each joint's
    source_mates and re-classifies. Runs on any type that might be wrong
    (unknown or prismatic); a meaningful new verdict updates the joint in
    place. Revolute joints are left alone (they're usually right and we
    don't want to accidentally demote them).
    """
    raw_by_name = {m.get("name"): m for m in data.get("raw_mates", [])}
    for j in data.get("joints", []):
        if j.get("type") not in ("unknown", "prismatic"):
            continue
        mates = []
        for mn in j.get("source_mates", []):
            rm = raw_by_name.get(mn)
            if not rm:
                continue
            mates.append(Mate(
                name=rm.get("name", mn),
                mate_type=rm.get("mate_type"),
                component_a=rm.get("component_a", ""),
                component_b=rm.get("component_b", ""),
                axis=rm.get("axis"),
                origin=rm.get("origin"),
                limits=rm.get("limits"),
            ))
        if not mates:
            continue
        result = classify_mate_group(mates)
        if not result or result.get("type") in (None, "unknown"):
            continue
        j["type"] = result["type"]
        if result.get("axis") is not None:
            j["axis"] = result["axis"]
            j["has_axis"] = True
        if result.get("origin") is not None and j.get("origin") is None:
            j["origin"] = result["origin"]
        if result.get("limits") is not None and j.get("limits") is None:
            j["limits"] = result["limits"]


def extract_pivot_origins(data):
    """Extract real joint pivot locations from mate geometry.

    Priority order:
    1. Concentric mate origins — cylindrical face centers = true rotation pivots.
    2. Coincident mate origins — face contact points = fallback pivot locations
       (where two parts physically touch).

    Returns: dict mapping (parent_name, child_name) → [x, y, z] in meters.
    Both the entity0 and entity1 origins are averaged for robustness.
    """
    pivots = {}

    # First pass: concentric mates (highest priority — true rotation centers)
    for m in data.get("raw_mates", []):
        if m.get("mate_type_name") != "concentric":
            continue
        entities = m.get("entities", [])
        if len(entities) < 2:
            continue
        e0_origin = entities[0].get("origin")
        e1_origin = entities[1].get("origin")
        if not e0_origin or not e1_origin:
            continue
        avg = [(e0_origin[i] + e1_origin[i]) / 2.0 for i in range(3)]
        comp_a = m.get("component_a", "")
        comp_b = m.get("component_b", "")
        pivots[(comp_a, comp_b)] = avg
        pivots[(comp_b, comp_a)] = avg

    # Second pass: coincident mates (fallback for pairs without concentric).
    # The coincident origin is where two faces touch — a valid pivot point
    # for joints constrained by coincident + angle mates (e.g., wrist joints).
    for m in data.get("raw_mates", []):
        if m.get("mate_type_name") != "coincident":
            continue
        entities = m.get("entities", [])
        if len(entities) < 2:
            continue
        e0_origin = entities[0].get("origin")
        e1_origin = entities[1].get("origin")
        if not e0_origin or not e1_origin:
            continue
        comp_a = m.get("component_a", "")
        comp_b = m.get("component_b", "")
        # Only use if no concentric pivot already exists for this pair
        if (comp_a, comp_b) not in pivots:
            avg = [(e0_origin[i] + e1_origin[i]) / 2.0 for i in range(3)]
            pivots[(comp_a, comp_b)] = avg
            pivots[(comp_b, comp_a)] = avg

    return pivots


def extract_coincident_axes(data):
    """Extract rotation axes from coincident mate face normals.

    For joints without concentric mates, the coincident mate's face normal
    IS the rotation axis — parts rotate around the normal of their contact face.

    Returns: dict mapping (comp_a, comp_b) → [ax, ay, az] unit vector.
    Only includes pairs that do NOT have a concentric mate (those already
    have axis data from the concentric geometry).
    """
    # Concentric pairs already have good axis data
    concentric_pairs = set()
    for m in data.get("raw_mates", []):
        if m.get("mate_type_name") == "concentric":
            concentric_pairs.add(frozenset([m["component_a"], m["component_b"]]))

    axes = {}
    for m in data.get("raw_mates", []):
        if m.get("mate_type_name") != "coincident":
            continue
        entities = m.get("entities", [])
        if len(entities) < 2:
            continue
        e0_axis = entities[0].get("axis")
        if not e0_axis:
            continue
        comp_a = m.get("component_a", "")
        comp_b = m.get("component_b", "")
        pair = frozenset([comp_a, comp_b])
        if pair in concentric_pairs:
            continue
        if (comp_a, comp_b) not in axes:
            axes[(comp_a, comp_b)] = e0_axis
            axes[(comp_b, comp_a)] = e0_axis

    return axes


def analyze_kinematic_tree(data):
    """Build the kinematic tree from constraint JSON.

    Returns:
        {
            "root": "part_name",
            "links": {
                "part_name": {
                    "parent": "parent_name" or None,
                    "joint": {...} or None,
                    "children": ["child1", ...],
                    "fixed_parts": ["fixed1", ...],
                    "mesh_files": ["file1.SLDPRT", ...],
                }
            },
            "part_to_link": { "part_name": "link_name" },
        }
    """
    parts = data.get("parts", [])
    joints = data.get("joints", [])
    fixed_rels = data.get("fixed_relationships", [])

    # Build concentric pivot lookup to prioritize concentric-backed joints.
    # Concentric mates define real geometric pivots (cylindrical face axes).
    # Joints backed by concentrics should take priority over angle-limit or
    # coincident-only joints when determining the kinematic chain.
    concentric_pairs = set()
    for m in data.get("raw_mates", []):
        if m.get("mate_type_name") == "concentric":
            concentric_pairs.add(frozenset([m["component_a"], m["component_b"]]))

    def joint_has_concentric(j):
        return frozenset([j["parent"], j["child"]]) in concentric_pairs

    # Build the kinematic chain using BFS over concentric mates from root.
    # Concentric mates define the real physical joints. BFS ensures the
    # direction (parent→child) follows the chain outward from base to tip,
    # regardless of alphabetical ordering in the SW mate data.
    bfs_root = None  # will be set by concentric BFS if data available
    concentric_edges = []  # [(compA, compB, joint_data, pivot_origin)]
    for j in joints:
        pair = frozenset([j["parent"], j["child"]])
        if pair in concentric_pairs:
            pivot = None
            for m in data.get("raw_mates", []):
                if m.get("mate_type_name") == "concentric" and \
                   frozenset([m["component_a"], m["component_b"]]) == pair:
                    e0 = m["entities"][0]["origin"]
                    e1 = m["entities"][1]["origin"]
                    pivot = [(e0[i]+e1[i])/2 for i in range(3)]
                    break
            concentric_edges.append((j["parent"], j["child"], j, pivot))

    # Build undirected adjacency from concentric edges
    concentric_adj = {}
    for a, b, j, pivot in concentric_edges:
        concentric_adj.setdefault(a, []).append((b, j, pivot))
        concentric_adj.setdefault(b, []).append((a, j, pivot))

    # BFS from root candidate (most-connected parent-only part)
    # First pass: find root from concentric graph
    all_concentric_nodes = set()
    for a, b, j, pivot in concentric_edges:
        all_concentric_nodes.add(a)
        all_concentric_nodes.add(b)

    child_to_parent = {}
    child_to_joint = {}
    parent_set = set()
    child_set = set()

    # BFS from the root (will be determined below) over concentric edges
    bfs_root_candidates = [n for n in all_concentric_nodes
                           if sum(1 for a, b, _, _ in concentric_edges
                                  if a == n or b == n) > 0]
    # Pick the node with most concentric connections that has lowest avg pivot Z
    # (closest to base)
    if bfs_root_candidates:
        bfs_root = bfs_root_candidates[0]  # temporary
        # Find which node reaches the most via concentric edges.
        # Break ties by picking the node with the lowest average
        # pivot Z coordinate (closest to the robot base).
        scores = []
        for candidate in bfs_root_candidates:
            visited = set()
            queue_bfs = [candidate]
            while queue_bfs:
                cur = queue_bfs.pop(0)
                if cur in visited:
                    continue
                visited.add(cur)
                for nb, _, _ in concentric_adj.get(cur, []):
                    if nb not in visited:
                        queue_bfs.append(nb)
            # Average Z of pivots reachable from this candidate
            avg_z = 0
            z_count = 0
            for a, b, _, piv in concentric_edges:
                if piv and (a == candidate or b == candidate):
                    avg_z += piv[2]
                    z_count += 1
            avg_z = avg_z / z_count if z_count > 0 else 999
            scores.append((len(visited), -avg_z, candidate))

        scores.sort(reverse=True)  # most reachable first, then lowest Z
        bfs_root = scores[0][2]

        # BFS to assign parent→child directions
        visited = {bfs_root}
        queue = [bfs_root]
        while queue:
            cur = queue.pop(0)
            parent_set.add(cur)
            for nb, j, pivot in concentric_adj.get(cur, []):
                if nb not in visited:
                    visited.add(nb)
                    child_to_parent[nb] = cur
                    # Create joint with correct parent→child direction
                    directed_joint = dict(j)
                    directed_joint["parent"] = cur
                    directed_joint["child"] = nb
                    child_to_joint[nb] = directed_joint
                    child_set.add(nb)
                    parent_set.add(cur)
                    queue.append(nb)

    # Now add non-concentric joints for remaining unassigned children
    for j in joints:
        p, c = j["parent"], j["child"]
        parent_set.add(p)
        child_set.add(c)
        if c not in child_to_parent:
            child_to_parent[c] = p
            child_to_joint[c] = j

    # Resolve fixed_relationships direction
    fixed_child_to_parent = {}
    known = parent_set | child_set
    unresolved = list(fixed_rels)
    for _ in range(10):
        remaining = []
        for f in unresolved:
            a, b = f["parent"], f["child"]
            if a in known or a in fixed_child_to_parent:
                fixed_child_to_parent[b] = a
                known.add(b)
            elif b in known or b in fixed_child_to_parent:
                fixed_child_to_parent[a] = b
                known.add(a)
            else:
                remaining.append(f)
        if len(remaining) == len(unresolved):
            break
        unresolved = remaining
    for f in unresolved:
        fixed_child_to_parent.setdefault(f["child"], f["parent"])

    # Root = the BFS root from the concentric chain (the node that reaches
    # the most other nodes via concentric mates). This is ALWAYS the correct
    # kinematic base because concentric mates define the real physical joints.
    all_children = child_set | set(fixed_child_to_parent.keys())
    parts_by_name = {p["name"]: p for p in parts}

    # bfs_root was set in the concentric BFS above
    root = bfs_root

    if root is None:
        # Fallback: pick part with lowest Z
        candidates = [p for p in parts if p["name"] not in all_children]
        if candidates:
            def z_pos(p):
                xf = p.get("transform")
                return xf[11] if xf and len(xf) > 11 else 0
            root = min(candidates, key=z_pos)["name"]

    # Other parent-only parts (e.g., BalancerCase) that aren't the root
    # become children of the root via synthetic fixed joints.
    root_candidates = [p["name"] for p in parts
                       if p["name"] not in all_children and p["name"] in parent_set]
    for secondary_root in root_candidates:
        if secondary_root != root and secondary_root not in child_to_parent:
            child_to_parent[secondary_root] = root
            child_to_joint[secondary_root] = {
                "id": "fixed_{}".format(sanitize_name(secondary_root)),
                "type": "fixed",
                "parent": root,
                "child": secondary_root,
                "axis": None,
                "has_axis": False,
                "origin": None,
                "limits": None,
                "source_mates": [],
            }
            child_set.add(secondary_root)

    # Map every part to its link
    part_to_link = {root: root}
    for c in child_to_joint:
        part_to_link[c] = c
    # BFS for fixed parts
    queue = list(part_to_link.keys())
    while queue:
        cur = queue.pop(0)
        for fc, fp in fixed_child_to_parent.items():
            if fp == cur and fc not in part_to_link:
                part_to_link[fc] = part_to_link[cur]
                queue.append(fc)
    # Orphans by proximity
    link_positions = {}
    for name in part_to_link:
        p = parts_by_name.get(name)
        if p and p.get("transform") and len(p["transform"]) >= 12:
            link_positions[name] = (p["transform"][9], p["transform"][10], p["transform"][11])
    for p in parts:
        if p["name"] not in part_to_link:
            if p.get("transform") and link_positions:
                px, py, pz = p["transform"][9], p["transform"][10], p["transform"][11]
                best = min(link_positions, key=lambda k: sum((a-b)**2 for a,b in zip(link_positions[k], (px,py,pz))))
                part_to_link[p["name"]] = part_to_link.get(best, root)
            else:
                part_to_link[p["name"]] = root

    # Build links dict
    links = {}
    for name in set(part_to_link.values()):
        p = parts_by_name.get(name)
        links[name] = {
            "parent": child_to_parent.get(name),
            "joint": child_to_joint.get(name),
            "children": [],
            "fixed_parts": [],
            "mesh_files": [parts_by_name[name]["filename"]] if name in parts_by_name and "filename" in parts_by_name[name] else [],
        }
    for name, info in links.items():
        if info["parent"] and info["parent"] in links:
            links[info["parent"]]["children"].append(name)
    for p in parts:
        link = part_to_link[p["name"]]
        if p["name"] != link:
            if link in links:
                links[link]["fixed_parts"].append(p["name"])
                if p.get("filename"):
                    links[link]["mesh_files"].append(p["filename"])

    return {
        "root": root,
        "links": links,
        "part_to_link": part_to_link,
    }


def decompose_transform(xf):
    """Extract xyz (meters) and rpy (radians) from a 16-float SW transform.

    SW layout: [R00,R10,R20, R01,R11,R21, R02,R12,R22, Tx,Ty,Tz, scale, 0,0,0]
    (column-major rotation, translation in meters)
    """
    if not xf or len(xf) < 12:
        return [0, 0, 0], [0, 0, 0]

    # Translation
    xyz = [xf[9], xf[10], xf[11]]

    # Rotation matrix (column-major → row-major)
    r00, r10, r20 = xf[0], xf[1], xf[2]
    r01, r11, r21 = xf[3], xf[4], xf[5]
    r02, r12, r22 = xf[6], xf[7], xf[8]

    # Extract Euler angles (ZYX convention = rpy)
    sy = math.sqrt(r00*r00 + r10*r10)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw   = math.atan2(r10, r00)
    else:
        roll  = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw   = 0.0

    return xyz, [roll, pitch, yaw]


def compute_relative_origin(parent_xf, child_xf):
    """Compute the child's origin RELATIVE to the parent.

    URDF joints need <origin xyz="..." rpy="..."/> expressed in the
    parent link's coordinate frame, not in world.

    T_relative = T_parent⁻¹ × T_child
    """
    def xf_to_matrix(xf):
        """Convert 16-float SW transform to 4x4 list-of-lists."""
        if not xf or len(xf) < 12:
            return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        return [
            [xf[0], xf[3], xf[6], xf[9]],
            [xf[1], xf[4], xf[7], xf[10]],
            [xf[2], xf[5], xf[8], xf[11]],
            [0,     0,     0,     1],
        ]

    def mat_inverse(m):
        """Inverse of a 4x4 rigid-body transform (R^T, -R^T * t)."""
        r = [[m[0][0],m[1][0],m[2][0]],
             [m[0][1],m[1][1],m[2][1]],
             [m[0][2],m[1][2],m[2][2]]]
        t = [m[0][3], m[1][3], m[2][3]]
        inv_t = [-(r[0][0]*t[0]+r[0][1]*t[1]+r[0][2]*t[2]),
                 -(r[1][0]*t[0]+r[1][1]*t[1]+r[1][2]*t[2]),
                 -(r[2][0]*t[0]+r[2][1]*t[1]+r[2][2]*t[2])]
        return [
            [r[0][0],r[0][1],r[0][2],inv_t[0]],
            [r[1][0],r[1][1],r[1][2],inv_t[1]],
            [r[2][0],r[2][1],r[2][2],inv_t[2]],
            [0,0,0,1],
        ]

    def mat_mult(a, b):
        """4x4 matrix multiplication."""
        c = [[0]*4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    c[i][j] += a[i][k] * b[k][j]
        return c

    def mat_to_xf(m):
        """Convert 4x4 back to 16-float SW format for decompose_transform."""
        return [m[0][0],m[1][0],m[2][0],
                m[0][1],m[1][1],m[2][1],
                m[0][2],m[1][2],m[2][2],
                m[0][3],m[1][3],m[2][3],
                1, 0, 0, 0]

    p = xf_to_matrix(parent_xf)
    c = xf_to_matrix(child_xf)
    rel = mat_mult(mat_inverse(p), c)
    return decompose_transform(mat_to_xf(rel))


def sanitize_name(name):
    """Make a name safe for URDF XML (no slashes, spaces, or hyphens)."""
    return name.replace("/", "_").replace(" ", "_").replace("-", "_")


def load_frames_sidecar(path):
    """Load a frames sidecar (from analyze_assembly_frames.py). Returns None
    if the path is None or missing. The sidecar's `link_world_pivots` map
    overrides mate-origin-derived pivots so rotations happen at the meshes."""
    if not path or not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def generate_urdf(data, robot_name, meshes_dir="meshes", frames_sidecar=None):
    """Generate a URDF XML string from constraint JSON data.

    Args:
        data: parsed constraint JSON (parts, joints, fixed_relationships, etc.)
        robot_name: name for the <robot> tag
        meshes_dir: relative path prefix for mesh files (used in package:// URIs)

    Returns:
        A string containing the complete URDF XML.
    """
    reclassify_unknown_joints(data)
    tree_info = analyze_kinematic_tree(data)
    root_name = tree_info["root"]
    links = tree_info["links"]
    parts_by_name = {p["name"]: p for p in data.get("parts", [])}

    # Real pivot origins from mate geometry (meters).
    # Concentric mate origins are preferred; coincident origins are fallback.
    # These are ABSOLUTE positions in the sub-assembly frame.
    pivots = extract_pivot_origins(data)

    # Fallback axes from coincident mate face normals (for joints without
    # concentric mates, e.g., wrist joints constrained by coincident+angle).
    coinc_axes = extract_coincident_axes(data)

    # Build world-position for each link's pivot by walking the chain
    # from root. Each link's URDF origin = this_pivot - parent_pivot.
    link_world_pivot = {}  # link_name → [x, y, z] meters (absolute)
    link_world_pivot[root_name] = [0, 0, 0]

    def assign_pivots(link_name):
        info = links.get(link_name, {})
        parent = info.get("parent")
        if parent and parent in link_world_pivot:
            pivot = pivots.get((parent, link_name))
            if pivot:
                link_world_pivot[link_name] = pivot
            else:
                # No mate pivot at all — use parent's pivot as best guess
                link_world_pivot[link_name] = list(link_world_pivot[parent])
        for child in info.get("children", []):
            assign_pivots(child)

    assign_pivots(root_name)

    # Apply frames-sidecar corrections. Two knobs:
    #   z_rotation_rad: rotates ALL link pivots about Z (whole-chain fix for
    #                   SW-capture-vs-GLB-export frame mismatch).
    #   link_world_pivots: per-link explicit overrides (runs after rotation).
    sidecar = load_frames_sidecar(frames_sidecar)
    if sidecar:
        theta = sidecar.get("z_rotation_rad", 0.0)
        if theta:
            c, s = math.cos(theta), math.sin(theta)
            for link_name, v in list(link_world_pivot.items()):
                link_world_pivot[link_name] = [
                    c * v[0] - s * v[1],
                    s * v[0] + c * v[1],
                    v[2],
                ]
        for link_name, pivot in sidecar.get("link_world_pivots", {}).items():
            if link_name in link_world_pivot:
                link_world_pivot[link_name] = list(pivot)

    robot = Element("robot", name=robot_name)

    def add_link_and_children(link_name, parent_link_name=None):
        info = links.get(link_name, {})
        safe_name = sanitize_name(link_name)

        # Create <link>
        link_el = SubElement(robot, "link", name=safe_name)

        # Visual geometry for each mesh file.
        # Each mesh has vertices baked at SolidWorks assembly-world positions.
        # The link is at a world position (accumulated joint origins).
        # visual <origin> = negative of link's world pivot — shifts mesh
        # vertices into the link's local frame so URDF handles all positioning.
        world_pivot = link_world_pivot.get(link_name, [0, 0, 0])
        for mesh_file in info.get("mesh_files", []):
            visual = SubElement(link_el, "visual")
            SubElement(visual, "origin",
                       xyz=f"{-world_pivot[0]:.6f} {-world_pivot[1]:.6f} {-world_pivot[2]:.6f}",
                       rpy="0 0 0")
            geom = SubElement(visual, "geometry")
            glb_name = os.path.splitext(mesh_file)[0] + ".glb"
            SubElement(geom, "mesh", filename=f"package://{meshes_dir}/{glb_name}")
            part = parts_by_name.get(link_name)
            if part and part.get("color"):
                c = part["color"]
                mat = SubElement(visual, "material", name=f"color_{safe_name}")
                SubElement(mat, "color", rgba=f"{c[0]/255:.3f} {c[1]/255:.3f} {c[2]/255:.3f} 1.0")

        # If this link has a joint to its parent, create the <joint>
        joint_data = info.get("joint")
        if joint_data and parent_link_name:
            safe_parent = sanitize_name(parent_link_name)
            joint_el = SubElement(robot, "joint",
                                  name=joint_data["id"],
                                  type=joint_data.get("type", "fixed"))

            SubElement(joint_el, "parent", link=safe_parent)
            SubElement(joint_el, "child", link=safe_name)

            # Origin: RELATIVE position = child_world_pivot - parent_world_pivot.
            # Both are in the same frame (B261 local coords), so subtraction
            # gives the correct URDF-style relative offset.
            parent_world = link_world_pivot.get(parent_link_name, [0, 0, 0])
            child_world = link_world_pivot.get(link_name, [0, 0, 0])
            xyz = [child_world[i] - parent_world[i] for i in range(3)]
            rpy = [0, 0, 0]  # home position = zero rotation at each joint

            SubElement(joint_el, "origin",
                       xyz=f"{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}",
                       rpy=f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}")

            # Axis: prefer joint's own axis, fall back to coincident mate normal.
            # Apply the sidecar's Z rotation to keep axes in the same frame as
            # the rotated pivots. Per-joint overrides (sidecar.joint_axis_overrides)
            # take precedence.
            axis = joint_data.get("axis")
            if not axis:
                axis = coinc_axes.get((parent_link_name, link_name))
            if axis and sidecar:
                theta = sidecar.get("z_rotation_rad", 0.0)
                if theta:
                    c2, s2 = math.cos(theta), math.sin(theta)
                    axis = [c2 * axis[0] - s2 * axis[1],
                            s2 * axis[0] + c2 * axis[1],
                            axis[2]]
            if sidecar:
                override = sidecar.get("joint_axis_overrides", {}).get(joint_data["id"])
                if override:
                    axis = list(override)
            if axis:
                SubElement(joint_el, "axis",
                           xyz=f"{axis[0]:.6g} {axis[1]:.6g} {axis[2]:.6g}")
            else:
                SubElement(joint_el, "axis", xyz="0 0 1")

            # Limits
            limits = joint_data.get("limits") or {}
            joint_type = joint_data.get("type", "fixed")
            if joint_type == "revolute":
                SubElement(joint_el, "limit",
                           lower=str(limits.get("lower", -math.pi)),
                           upper=str(limits.get("upper", math.pi)),
                           effort="10", velocity="3.14")
            elif joint_type == "prismatic":
                SubElement(joint_el, "limit",
                           lower=str(limits.get("lower", -1)),
                           upper=str(limits.get("upper", 1)),
                           effort="10", velocity="1.0")

        # Recurse into children
        for child_name in info.get("children", []):
            add_link_and_children(child_name, parent_link_name=link_name)

    add_link_and_children(root_name)

    # Pretty-print with XML declaration
    rough = tostring(robot, encoding="unicode")
    return parseString(f'<?xml version="1.0"?>\n{rough}').toprettyxml(indent="  ")


def main():
    parser = argparse.ArgumentParser(description="Generate URDF from SW constraint JSON")
    parser.add_argument("--input", required=True, help="Path to constraint JSON")
    parser.add_argument("--output", required=True, help="Output .urdf path")
    parser.add_argument("--name", default="robot", help="Robot name for URDF")
    parser.add_argument("--meshes", default="meshes", help="Meshes directory prefix")
    args = parser.parse_args()

    with open(args.input) as f:
        data = json.load(f)

    # Auto-detect frames sidecar: <input_stem>.frames.json next to the input.
    stem, _ = os.path.splitext(args.input)
    sidecar_path = stem + ".frames.json"
    if not os.path.exists(sidecar_path):
        sidecar_path = None

    urdf_str = generate_urdf(data, args.name, meshes_dir=args.meshes,
                             frames_sidecar=sidecar_path)
    if sidecar_path:
        print(f"Using frames sidecar: {sidecar_path}")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        f.write(urdf_str)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
