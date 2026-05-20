"""
path_plan_utils.py

Pure-Python helpers for loading and validating path_plan.yaml.
No RoboDK dependency — safe to import in unit tests.
"""

import os
import sys

import yaml


def load_path_plan(plan_path: str, expected_hashes: dict | None) -> dict:
    """
    Load path_plan.yaml. Exits if:
      - file missing
      - collision_critical hash mismatch (cached results invalid)
      - structural hash mismatch (waypoint joints changed)
    Warns (continues) if only additive config changes detected.
    """
    if not os.path.isfile(plan_path):
        print(f"[ERROR] path_plan.yaml not found: {plan_path}")
        print("        Run check_collision_free_paths.py to generate it.")
        sys.exit(1)

    with open(plan_path) as f:
        plan = yaml.safe_load(f)

    if expected_hashes is not None:
        stored = plan.get("config_hashes", {})
        if stored.get("collision_critical") != expected_hashes.get("collision_critical"):
            print("[ERROR] Collision configuration has changed — cached paths are invalid.")
            print("        Re-run check_collision_free_paths.py before proceeding.")
            sys.exit(1)
        if stored.get("structural") != expected_hashes.get("structural"):
            print("[ERROR] Waypoint definitions have changed — cached edge joints are stale.")
            print("        Re-run check_collision_free_paths.py before proceeding.")
            sys.exit(1)

    return plan


def filter_tested_cones(cone_names: list, cone_plan: dict) -> list:
    """Return only cone names that are tested: true in the plan."""
    return [
        name for name in cone_names
        if cone_plan.get(name, {}).get("tested") is True
    ]


def find_cone_group(cone_name: str, destination_groups: dict):
    """
    Find which destination group contains cone_name.
    Returns (group_name, cone_data) or None if not found.
    """
    for group_name, group in destination_groups.items():
        cones = group.get("cones", {})
        if cone_name in cones:
            return group_name, cones[cone_name]
    return None


def validate_sequence(sequence_names: list, edge_cache: dict) -> list:
    """
    Check every consecutive edge in sequence_names against edge_cache.
    Returns list of problem strings (empty = all clear).
    """
    problems = []
    for i in range(len(sequence_names) - 1):
        f = sequence_names[i]
        t = sequence_names[i + 1]
        key = f"{f}|{t}"
        if key not in edge_cache:
            problems.append(f"Edge {f} → {t}: not tested. Re-run check_collision_free_paths.py.")
        elif not edge_cache[key]["collision_free"]:
            problems.append(f"Edge {f} → {t}: collision detected in simulation.")
    return problems


def build_sequence_names(
    base_cone_name: str,
    dest_cone_name: str,
    plan: dict,
    routing_candidates: list,
) -> list:
    """
    Build the ordered list of node names for a complete pick-and-place.

    Sequence:
      home → [path to base_gateway] → base_approach → base_grab → base_approach
           → [path back to base_gateway] → [path to dest_gateway]
           → dest_approach → dest_grab → dest_approach
           → [path back to dest_gateway] → [path to home]

    Uses find_shortest_path over the routing_candidates subgraph.
    Raises RuntimeError if no path exists between any required pair.
    """
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    from check_collision_free_paths import find_shortest_path

    edge_cache = plan["edge_cache"]
    base_info = plan["base_cones"][base_cone_name]
    group_result = find_cone_group(dest_cone_name, plan["destination_groups"])
    if group_result is None:
        raise RuntimeError(f"Destination cone '{dest_cone_name}' not found in any group in path_plan.yaml.")
    group_name, dest_info = group_result
    group = plan["destination_groups"][group_name]

    base_gateway = base_info["gateways"][0]
    dest_gateways = group["gateways"]

    # Build collision-free edges over routing candidates only
    candidate_edges = set()
    for f in routing_candidates:
        for t in routing_candidates:
            if f != t and edge_cache.get(f"{f}|{t}", {}).get("collision_free"):
                candidate_edges.add((f, t))

    def _path(start, end):
        p = find_shortest_path(candidate_edges, start, end)
        if p is None:
            raise RuntimeError(
                f"No tested collision-free path from '{start}' to '{end}'. "
                "Add more routing_candidates and re-run check_collision_free_paths.py."
            )
        return p

    # Find which dest gateway has a tested path from base_gateway
    dest_gateway = None
    for dg in dest_gateways:
        try:
            _path(base_gateway, dg)
            dest_gateway = dg
            break
        except RuntimeError:
            continue
    if dest_gateway is None:
        raise RuntimeError(
            f"No tested collision-free path from base gateway '{base_gateway}' "
            f"to any dest gateway {dest_gateways}. "
            "Re-run check_collision_free_paths.py with more routing_candidates."
        )

    base_ak = f"{base_cone_name}_approach"
    base_gk = f"{base_cone_name}_grab"
    dest_ak = f"{dest_cone_name}_approach"
    dest_gk = f"{dest_cone_name}_grab"

    seq = []
    seq += _path("home", base_gateway)
    seq.append(base_ak)
    seq.append(base_gk)
    seq.append(base_ak)                          # retract
    seq += _path(base_gateway, dest_gateway)[1:]  # skip base_gateway (already there)
    seq.append(dest_ak)
    seq.append(dest_gk)
    seq.append(dest_ak)                          # retract
    seq += list(reversed(_path("home", dest_gateway)))[1:]  # reverse back to home
    return seq
