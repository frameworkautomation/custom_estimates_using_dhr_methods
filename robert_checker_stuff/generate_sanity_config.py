"""Generate sanity_check_config.json for bin cone targets.

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""
import json

CONES = [
    "cone_in_bin_00_frame", "cone_in_bin_01_frame", "cone_in_bin_02_frame",
    "cone_in_bin_10_frame", "cone_in_bin_11_frame", "cone_in_bin_12_frame",
]

SUFFIXES_PICKUP = ["cone_pickup_pose", "before_pickup_offset", "post_pickup_above"]
SUFFIXES_KNOTTING = ["suction_position", "suction_offset_1", "suction_offset_2"]


def make_entry(cone, suffix):
    target_name = f"bin_{cone}_{suffix}"
    return {
        "name": target_name,
        "type": "point",
        "name_path": f"BinSanityCheckTargets/{target_name}",
        "z_axis_free": True,
        "special_track_conditions": {"type": "Locked_at_j7_0"}
    }


config = {
    "end_effectors": [
        {
            "end_effector_name": "pickup",
            "paths_and_points_to_check": [
                make_entry(c, s) for c in CONES for s in SUFFIXES_PICKUP
            ]
        },
        {
            "end_effector_name": "knotting",
            "paths_and_points_to_check": [
                make_entry(c, s) for c in CONES for s in SUFFIXES_KNOTTING
            ]
        }
    ]
}

out_path = "robert_checker_stuff/sanity_check_config.json"
with open(out_path, "w") as f:
    json.dump(config, f, indent=2)

total = sum(len(e["paths_and_points_to_check"]) for e in config["end_effectors"])
print(f"Wrote {total} entries to {out_path}")
