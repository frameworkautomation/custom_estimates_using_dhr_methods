"""
check_collision_free_paths.py

Offline collision-free path checker. Requires RoboDK to be running.

Usage:
    python robodk_code/check_collision_free_paths.py

Reads:  robo_dk_output/path_config.yaml
Writes: robo_dk_output/path_plan.yaml
"""

import sys
import os
import hashlib
import json
import datetime

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "path_config.yaml"
)
PLAN_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "path_plan.yaml"
)
IK_SOLUTIONS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions"
)
ROBODK_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output"
)


# ── Config loading ─────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    """Load and return path_config.yaml. Exits with message if file missing."""
    if not os.path.isfile(path):
        print(f"[ERROR] path_config.yaml not found at: {path}")
        print("        Create it from the template in robo_dk_output/path_config.yaml")
        sys.exit(1)
    with open(path) as f:
        return yaml.safe_load(f)


# ── Config hashing ─────────────────────────────────────────────────────────────

def compute_config_hashes(config: dict) -> dict:
    """
    Compute two hashes of the config:
      collision_critical — covers: collision_enable, collision_disable, cone_mesh_template
      structural         — covers: waypoints (names + joints), routing_candidates

    If collision_critical changes, all cached edges are invalid (re-run required).
    If structural changes, edges referencing changed/removed waypoints are invalid (re-run required).
    Additive changes (new groups, new waypoints not yet in routing_candidates) do not
    affect either hash and are safe to continue with.
    """
    def _hash(obj) -> str:
        return hashlib.sha256(
            json.dumps(obj, sort_keys=True).encode()
        ).hexdigest()[:16]

    collision_critical_data = {
        "collision_enable": config.get("collision_enable", []),
        "collision_disable": config.get("collision_disable", []),
        "cone_mesh_template": config.get("cone_mesh_template", ""),
    }

    structural_data = {
        "waypoints": {
            name: wp.get("joints") or wp.get("target")
            for name, wp in config.get("waypoints", {}).items()
        },
        "routing_candidates": sorted(config.get("routing_candidates", [])),
    }

    return {
        "collision_critical": _hash(collision_critical_data),
        "structural": _hash(structural_data),
    }


# ── Pathfinding ────────────────────────────────────────────────────────────────

def find_shortest_path(collision_free_edges: set, start: str, end: str):
    """
    Find the shortest path (fewest hops) from start to end using only edges
    in collision_free_edges (set of (from, to) tuples).
    Returns ordered list of node names, or None if no path exists.
    """
    import heapq
    if start == end:
        return [start]
    heap = [(0, start, [start])]
    visited = set()
    while heap:
        cost, node, path = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)
        if node == end:
            return path
        for (f, t) in collision_free_edges:
            if f == node and t not in visited:
                heapq.heappush(heap, (cost + 1, t, path + [t]))
    return None


# ── Gateway discovery ──────────────────────────────────────────────────────────

def _approach_key(cone_name: str) -> str:
    return f"{cone_name}_approach"

def _grab_key(cone_name: str) -> str:
    return f"{cone_name}_grab"

def _edge_clear(edge_cache: dict, from_node: str, to_node: str) -> bool:
    key = f"{from_node}|{to_node}"
    return edge_cache.get(key, {}).get("collision_free", False)

def _cone_ik_ok(edge_cache: dict, cone_name: str) -> bool:
    """IK is considered OK if approach→grab and grab→approach edges are both present and clear."""
    ak = _approach_key(cone_name)
    gk = _grab_key(cone_name)
    return (
        _edge_clear(edge_cache, ak, gk) and
        _edge_clear(edge_cache, gk, ak)
    )

def find_gateways(edge_cache: dict, destination_groups: dict, routing_candidates: list) -> dict:
    """
    For each destination group, find which routing_candidates qualify as gateways.

    A candidate R is a gateway for a group if, for EVERY cone in the group:
      - R → cone_approach is collision_free
      - cone_approach → R is collision_free (retract)
      - cone_approach → cone_grab is collision_free
      - cone_grab → cone_approach is collision_free

    Returns:
      {
        group_name: {
          "gateways": [candidate_name, ...],
          "cones": {
            cone_name: {"tested": True} | {"tested": False, "reason": "ik_failed"|"no_collision_free_path"}
          }
        }
      }
    """
    result = {}

    for group_name, group in destination_groups.items():
        cones = group["cones"]
        candidates = group.get("gateway_candidates", routing_candidates)

        # Determine per-cone IK status first (independent of gateway)
        cone_ik = {c: _cone_ik_ok(edge_cache, c) for c in cones}

        # Find valid gateways (must reach every cone with good IK)
        valid_gateways = []
        for candidate in candidates:
            is_gateway = all(
                cone_ik[c] and
                _edge_clear(edge_cache, candidate, _approach_key(c)) and
                _edge_clear(edge_cache, _approach_key(c), candidate)
                for c in cones
            )
            if is_gateway:
                valid_gateways.append(candidate)

        # Determine per-cone tested status
        cones_result = {}
        for cone_name in cones:
            if not cone_ik[cone_name]:
                cones_result[cone_name] = {"tested": False, "reason": "ik_failed"}
            elif not any(
                _edge_clear(edge_cache, gw, _approach_key(cone_name))
                for gw in valid_gateways
            ):
                cones_result[cone_name] = {"tested": False, "reason": "no_collision_free_path"}
            else:
                cones_result[cone_name] = {"tested": True}

        result[group_name] = {
            "gateways": valid_gateways,
            "cones": cones_result,
        }

    return result


# ── Node construction ──────────────────────────────────────────────────────────

def _resolve_waypoint_joints(rdk, robot, wp_config: dict):
    """
    Resolve a waypoint config entry to a joint list.
    Accepts:
      {"joints": [...]}           — used directly
      {"target": "TargetName"}    — RoboDK target resolved via SolveIK
    Returns None if resolution fails.
    """
    from robodk.robolink import ITEM_TYPE_TARGET, ITEM_TYPE_FRAME

    if "joints" in wp_config:
        return list(wp_config["joints"])

    target_name = wp_config.get("target")
    if target_name is None:
        print(f"[ERROR] Waypoint has neither 'joints' nor 'target'")
        return None

    target = rdk.Item(target_name, ITEM_TYPE_TARGET)
    if not target.Valid():
        print(f"[ERROR] RoboDK target not found: '{target_name}'")
        return None

    world_frame = rdk.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    pose = target.PoseAbs()
    result = robot.SolveIK(pose)
    try:
        joints = result.list()
    except AttributeError:
        joints = list(result)
    if len(joints) < 6:
        print(f"[ERROR] IK failed for target '{target_name}'")
        return None
    return joints


def build_nodes(rdk, robot, config: dict, base_ik: dict, dest_ik: dict) -> dict:
    """
    Build the complete node dict: {node_name: joint_list}.

    Sources:
      - routing_candidates from config["waypoints"] (resolved via _resolve_waypoint_joints)
      - base cone approach + grab from base_ik (pre-computed IK solutions)
      - destination cone approach + grab from dest_ik

    base_ik format (from load_latest_base_cone_ik):
      {cone_name: {"grab_ok": bool, "grab_joints": [...], "app_ok": bool, "app_joints": [...]}}

    dest_ik format (from compute_dest_ik):
      same schema as base_ik
    """
    nodes = {}

    # Routing candidate waypoints
    for name in config.get("routing_candidates", []):
        wp = config["waypoints"].get(name)
        if wp is None:
            print(f"[WARN] routing_candidate '{name}' not in waypoints — skipping")
            continue
        joints = _resolve_waypoint_joints(rdk, robot, wp)
        if joints is not None:
            nodes[name] = joints

    # Base cone nodes
    for cone_name, ik in base_ik.items():
        if ik.get("app_ok") and ik.get("grab_ok"):
            nodes[f"{cone_name}_approach"] = list(ik["app_joints"])
            nodes[f"{cone_name}_grab"] = list(ik["grab_joints"])
        else:
            print(f"[INFO] Skipping {cone_name} (IK failed)")

    # Destination cone nodes
    for cone_name, ik in dest_ik.items():
        if ik.get("app_ok") and ik.get("grab_ok"):
            nodes[f"{cone_name}_approach"] = list(ik["app_joints"])
            nodes[f"{cone_name}_grab"] = list(ik["grab_joints"])
        else:
            print(f"[INFO] Skipping {cone_name} (IK failed)")

    print(f"[INFO] Built {len(nodes)} nodes for edge testing")
    return nodes


# ── Edge testing ───────────────────────────────────────────────────────────────

def test_all_edges(rdk, robot, tool, cone_mesh, nodes: dict, config: dict) -> dict:
    """
    Test every directed pair of nodes with robot.MoveJ_Test().
    Cone mesh is attached to tool before testing (worst-case geometry).
    Returns edge_cache dict: {"{from}|{to}": {"from_joints", "to_joints", "collision_free"}}.

    collision_enable/disable pairs from config are applied before testing.
    """
    from robodk.robolink import COLLISION_ON, COLLISION_OFF, ITEM_TYPE_FRAME
    from robot_controller import Robot, PathEvaluationModel

    # Apply collision pair configuration
    for pair in config.get("collision_enable", []):
        item_a = rdk.Item(pair[0])
        item_b = rdk.Item(pair[1])
        if item_a.Valid() and item_b.Valid():
            rdk.setCollisionActivePair(COLLISION_ON, item_a, item_b)

    for pair in config.get("collision_disable", []):
        item_a = rdk.Item(pair[0])
        item_b = rdk.Item(pair[1])
        if item_a.Valid() and item_b.Valid():
            rdk.setCollisionActivePair(COLLISION_OFF, item_a, item_b)

    # Attach cone mesh to tool (worst-case geometry for all tests)
    world_frame = rdk.Item("WorldFrame", ITEM_TYPE_FRAME)
    if cone_mesh is not None and cone_mesh.Valid():
        cone_mesh.setParentStatic(tool)
        print(f"[INFO] Cone mesh '{cone_mesh.Name()}' attached to '{tool.Name()}' for collision testing")

    robot.setTool(tool)
    rdk.Render(False)

    checker = Robot(PathEvaluationModel)
    node_names = sorted(nodes.keys())
    total = len(node_names) * (len(node_names) - 1)
    tested = 0

    print(f"\nTesting {total} directed edges between {len(node_names)} nodes ...")
    for from_name in node_names:
        for to_name in node_names:
            if from_name == to_name:
                continue
            state = {
                "robot": robot,
                "from_joints": nodes[from_name],
                "to_joints": nodes[to_name],
            }
            checker.execute(state)
            edge_key = f"{from_name}|{to_name}"
            checker.edge_cache[edge_key] = {
                "from_joints": nodes[from_name],
                "to_joints": nodes[to_name],
                "collision_free": state["collision_free"],
            }
            tested += 1
            if tested % 20 == 0:
                print(f"  {tested}/{total} edges tested ...")

    rdk.Render(True)

    # Detach cone mesh back to world
    if cone_mesh is not None and cone_mesh.Valid():
        cone_mesh.setParentStatic(world_frame)

    clear = sum(1 for v in checker.edge_cache.values() if v["collision_free"])
    print(f"[INFO] Edge testing complete: {clear}/{len(checker.edge_cache)} collision-free")
    return checker.edge_cache


# ── Plan writer ────────────────────────────────────────────────────────────────

def write_plan(plan_path: str, config: dict, edge_cache: dict,
               base_cones_result: dict, dest_groups_result: dict) -> None:
    """Write path_plan.yaml. base_cones_result and dest_groups_result come from find_gateways."""
    hashes = compute_config_hashes(config)

    plan = {
        "generated": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "config_hashes": hashes,
        "edge_cache": edge_cache,
        "base_cones": base_cones_result,
        "destination_groups": dest_groups_result,
    }

    os.makedirs(os.path.dirname(plan_path), exist_ok=True)
    with open(plan_path, "w") as f:
        yaml.dump(plan, f, default_flow_style=False, sort_keys=True)
    print(f"\n[INFO] path_plan.yaml written to: {plan_path}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME

    # Import existing IK helpers from moving_a_cone.py
    from moving_a_cone import (
        load_latest_base_cone_ik,
        compute_all_offsets,
        find_base_cones,
        compute_dest_ik,
        find_destination_cones,
        ROBOT_NAME,
        TOOL_NAME,
    )

    config = load_config(CONFIG_PATH)

    # Connect to RoboDK
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK")
    except Exception as e:
        print(f"[ERROR] Cannot connect to RoboDK: {e}")
        sys.exit(1)

    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print(f"[ERROR] Robot '{ROBOT_NAME}' not found")
        sys.exit(1)

    tool_name = config.get("default_tool", TOOL_NAME)
    tool = rdk.Item(tool_name, ITEM_TYPE_TOOL)
    if not tool.Valid():
        print(f"[ERROR] Tool '{tool_name}' not found")
        sys.exit(1)

    cone_mesh_name = config.get("cone_mesh_template", "base_cone_0")
    cone_mesh = rdk.Item(cone_mesh_name)
    if not cone_mesh.Valid():
        print(f"[WARN] cone_mesh_template '{cone_mesh_name}' not found — collision checks without cone geometry")
        cone_mesh = None

    # Load or compute base cone IK
    base_ik = load_latest_base_cone_ik()
    if base_ik is None:
        base_cones = find_base_cones(rdk)
        base_ik = compute_all_offsets(rdk, robot, base_cones)

    # Load or compute dest cone IK
    dest_cones = find_destination_cones(rdk)
    dest_ik = compute_dest_ik(rdk, robot, dest_cones, tool)

    # Build nodes and test edges
    nodes = build_nodes(rdk, robot, config, base_ik, dest_ik)
    edge_cache = test_all_edges(rdk, robot, tool, cone_mesh, nodes, config)

    # Build routing candidate graph for pathfinding
    routing_candidates = config.get("routing_candidates", [])

    # Find gateways for destination groups
    dest_groups_result = find_gateways(
        edge_cache,
        config.get("destination_groups", {}),
        routing_candidates,
    )

    # Find gateways for base cones
    base_cones_result = {}
    for cone_name, ik in base_ik.items():
        ak = f"{cone_name}_approach"
        gk = f"{cone_name}_grab"
        ik_ok = (
            edge_cache.get(f"{ak}|{gk}", {}).get("collision_free", False) and
            edge_cache.get(f"{gk}|{ak}", {}).get("collision_free", False)
        )
        if not ik_ok:
            base_cones_result[cone_name] = {"tested": False, "reason": "ik_failed"}
            continue

        gateways = [
            c for c in routing_candidates
            if edge_cache.get(f"{c}|{ak}", {}).get("collision_free", False) and
               edge_cache.get(f"{ak}|{c}", {}).get("collision_free", False)
        ]
        if not gateways:
            base_cones_result[cone_name] = {"tested": False, "reason": "no_collision_free_path"}
        else:
            base_cones_result[cone_name] = {
                "tested": True,
                "approach_joints": list(nodes[ak]),
                "grab_joints": list(nodes[gk]),
                "gateways": gateways,
            }

    # Add approach/grab joints to tested dest cones
    for group_name, group_result in dest_groups_result.items():
        for cone_name, cone_result in group_result["cones"].items():
            if cone_result["tested"]:
                ak = f"{cone_name}_approach"
                gk = f"{cone_name}_grab"
                cone_result["approach_joints"] = list(nodes[ak])
                cone_result["grab_joints"] = list(nodes[gk])

    write_plan(PLAN_PATH, config, edge_cache, base_cones_result, dest_groups_result)

    # Summary
    base_ok = sum(1 for v in base_cones_result.values() if v.get("tested"))
    base_total = len(base_cones_result)
    print(f"\n[SUMMARY] Base cones:        {base_ok}/{base_total} usable")
    for g, gr in dest_groups_result.items():
        ok = sum(1 for c in gr["cones"].values() if c.get("tested"))
        tot = len(gr["cones"])
        gws = gr["gateways"]
        print(f"          {g}: {ok}/{tot} cones usable, gateways={gws}")


if __name__ == "__main__":
    main()
