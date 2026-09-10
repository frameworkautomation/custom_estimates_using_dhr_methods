# Sanity Check IK Comparison Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Test the bin cone positions from `back_bin_reachability`'s station using the proven IK checker from `positioning_robert_end_effector_etc`, to determine whether failures are caused by the coupled_pivot_demo code or by the station/positions themselves.

**Architecture:** Copy `generated_from_dhr_clone.rdk` onto the working branch as `thing_to_sanity_check.rdk`. Write a config that maps the bin cone frames to the checker's expected format. Since the checker looks up `ITEM_TYPE_TARGET` but our bin items are `ITEM_TYPE_FRAME`, we add a prep script that creates targets from frames. Then run the existing checker with `z_axis_free=true`.

**Tech Stack:** Python, RoboDK API, robert_end_checker.py (existing, proven)

**Spec:** No formal spec — this is a diagnostic comparison between two branches.

## Global Constraints

- Do NOT modify `robert_end_checker.py` — it is proven working code
- The checker expects TARGET items, not FRAME items
- All bin cone frames live under `Cone_Bin_Frame/bottom_corner/cone_*`
- Child frames may be bare suffixes (e.g. `suction_position`) not prefixed
- Tools needed: `pickup`, `knotting` (must exist in the station)
- Station is 6-DOF extracted (no rail) but checker handles both — use `Locked_at_j7_0` since j7=0 in extracted station

---

### Task 1: Branch setup + copy station

**Files:**
- Copy: `robo_dk_saves/generated_from_dhr_clone.rdk` → `robo_dk_saves/thing_to_sanity_check.rdk`

- [ ] **Step 1: Create branch from positioning_robert_end_effector_etc**

```bash
git checkout positioning_robert_end_effector_etc
git checkout -b sanity_check_9_9_2026
```

- [ ] **Step 2: Copy the rdk file from back_bin_reachability**

```bash
git show back_bin_reachability:robo_dk_saves/generated_from_dhr_clone.rdk > robo_dk_saves/thing_to_sanity_check.rdk
```

- [ ] **Step 3: Commit**

```bash
git add robo_dk_saves/thing_to_sanity_check.rdk
git commit -m "add thing_to_sanity_check.rdk from back_bin_reachability for IK comparison"
```

---

### Task 2: Create prep script — targets from bin cone frames

**Files:**
- Create: `robert_checker_stuff/prep_bin_targets.py`

**Produces:** RoboDK TARGET items created from FRAME items in the station, so `robert_end_checker.py` can find them.

The script:
1. Connects to RoboDK
2. Finds `Cone_Bin_Frame/bottom_corner`
3. For each `cone_*` frame, finds child frames (suction_position, suction_offset_1, etc.)
4. Creates a TARGET at each frame's PoseAbs() with a globally unique name
5. Naming: `bin_<cone>_<suffix>` (e.g. `bin_cone_in_bin_00_frame_suction_position`)

- [ ] **Step 1: Write prep_bin_targets.py**

```python
"""Create RoboDK targets from bin cone frames for use with robert_end_checker.

Usage:
    python robert_checker_stuff/prep_bin_targets.py --robodk-ip 172.23.208.1
"""
import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_ROBOT
import argparse

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
BIN_PARENT = "Cone_Bin_Frame"
BIN_SUBFRAME = "bottom_corner"
CHILD_SUFFIXES = [
    "suction_offset_2", "suction_offset_1", "suction_position",
    "before_pickup_offset", "cone_pickup_pose", "post_pickup_above",
]
TARGET_FOLDER = "BinSanityCheckTargets"

def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")

def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None

def collect_frames(parent):
    result = {}
    try:
        for child in parent.Childs():
            if child.Type() == ITEM_TYPE_FRAME:
                result[child.Name()] = child
            result.update(collect_frames(child))
    except Exception:
        pass
    return result

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robodk-ip", default=None)
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)
    assert robot is not None

    # Clean old targets
    old_folder = RDK.Item(TARGET_FOLDER, ITEM_TYPE_FRAME)
    if old_folder.Valid():
        old_folder.Delete()

    folder = RDK.AddFrame(TARGET_FOLDER)

    # Find bin cones
    bin_frame = RDK.Item(BIN_PARENT, ITEM_TYPE_FRAME)
    assert bin_frame.Valid(), f"{BIN_PARENT} not found"

    bottom = None
    for child in bin_frame.Childs():
        if child.Name() == BIN_SUBFRAME:
            bottom = child
            break
    assert bottom is not None, f"{BIN_SUBFRAME} not found under {BIN_PARENT}"

    created = 0
    for child in bottom.Childs():
        if child.Type() != ITEM_TYPE_FRAME or not child.Name().startswith("cone_"):
            continue
        cone_name = child.Name()
        all_frames = collect_frames(child)

        for suffix in CHILD_SUFFIXES:
            frame = all_frames.get(f"{cone_name}_{suffix}") or all_frames.get(suffix)
            if frame is None:
                print(f"  [WARN] {cone_name}/{suffix} not found")
                continue

            target_name = f"bin_{cone_name}_{suffix}"
            tgt = RDK.AddTarget(target_name, folder, robot)
            tgt.setPose(frame.PoseAbs())
            created += 1
            print(f"  Created: {target_name}")

    print(f"\n[DONE] Created {created} targets in {TARGET_FOLDER}")

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add robert_checker_stuff/prep_bin_targets.py
git commit -m "add prep script to create targets from bin cone frames"
```

---

### Task 3: Create sanity check config

**Files:**
- Create: `robert_checker_stuff/sanity_check_config.json`

**Produces:** Config JSON that the existing `robert_end_checker.py` can consume, pointing at the bin cone targets with `z_axis_free=true`.

The config has entries for both `pickup` and `knotting` tools on all 6 cones x 6 suffixes = 72 points total.

- [ ] **Step 1: Write a script to generate the config**

Rather than hand-writing 72 entries, write a small generator:

```python
"""Generate sanity_check_config.json for bin cone targets."""
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

with open("robert_checker_stuff/sanity_check_config.json", "w") as f:
    json.dump(config, f, indent=2)

print(f"Wrote {sum(len(e['paths_and_points_to_check']) for e in config['end_effectors'])} entries")
```

- [ ] **Step 2: Run it to generate the config**

```bash
python robert_checker_stuff/generate_sanity_config.py
```

- [ ] **Step 3: Commit**

```bash
git add robert_checker_stuff/generate_sanity_config.py robert_checker_stuff/sanity_check_config.json
git commit -m "add sanity check config for bin cone targets (pickup + knotting)"
```

---

### Task 4: Run the comparison

- [ ] **Step 1: Load thing_to_sanity_check.rdk in RoboDK**

Open `robo_dk_saves/thing_to_sanity_check.rdk` in RoboDK.

- [ ] **Step 2: Run prep script to create targets**

```bash
python robert_checker_stuff/prep_bin_targets.py --robodk-ip 172.23.208.1
```

- [ ] **Step 3: Run the proven checker with sanity config**

```bash
python robert_checker_stuff/robert_end_checker.py --robodk-ip 172.23.208.1 --config robert_checker_stuff/sanity_check_config.json
```

- [ ] **Step 4: Compare results**

If the proven checker finds reachable solutions → the bug is in coupled_pivot_demo.py code.
If the proven checker also fails → the positions/tool setup in the station are the problem.

- [ ] **Step 5: Commit results**

```bash
git add robert_checker_stuff/ik_results.json robert_checker_stuff/reachability_report.txt
git commit -m "sanity check results — bin cone reachability via proven checker"
```
