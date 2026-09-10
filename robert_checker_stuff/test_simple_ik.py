"""
Dead simple IK test — can the robot reach ANY frame in the station?

No OptimAxes, no sweep, no seeds. Just SolveIK on a known frame.

Usage:
    python robert_checker_stuff/test_simple_ik.py --robodk-ip 172.23.208.1
"""
import sys
import math
sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import Pose_2_TxyzRxyz, eye

import argparse

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

ap = argparse.ArgumentParser()
ap.add_argument("--robodk-ip", default=None)
ap.add_argument("--frame", default=None, help="Specific frame name to test")
args = ap.parse_args()

if args.robodk_ip:
    RDK = Robolink(robodk_ip=args.robodk_ip)
else:
    RDK = Robolink()

RDK._setTimeout(60)

# Find robot
robot = None
for name in ROBOT_NAMES:
    r = RDK.Item(name, ITEM_TYPE_ROBOT)
    if r.Valid():
        robot = r
        break
assert robot is not None, f"Robot not found: {ROBOT_NAMES}"

print(f"Robot: {robot.Name()}")
print(f"Robot DOF (joints): {len(robot.Joints().list())}")
print(f"Robot base: {robot.Parent().Name()}")

robot_base_xyz = Pose_2_TxyzRxyz(robot.PoseAbs())[:3]
print(f"Robot base world pos: [{robot_base_xyz[0]:.0f}, {robot_base_xyz[1]:.0f}, {robot_base_xyz[2]:.0f}]")

# List all tools
print("\nTools in station:")
for t in RDK.ItemList(ITEM_TYPE_TOOL):
    tcp = Pose_2_TxyzRxyz(t.PoseTool())[:3]
    print(f"  {t.Name()}: TCP=[{tcp[0]:.0f}, {tcp[1]:.0f}, {tcp[2]:.0f}]")

# List frames
print("\nAll frames:")
all_frames = RDK.ItemList(ITEM_TYPE_FRAME)
for f in all_frames:
    xyz = Pose_2_TxyzRxyz(f.PoseAbs())[:3]
    dist = math.sqrt(sum((xyz[i] - robot_base_xyz[i])**2 for i in range(3)))
    print(f"  {f.Name()}: pos=[{xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f}] dist={dist:.0f}mm")

# Test IK on a specific frame or the first suction_offset_1 we find
test_frame = None
if args.frame:
    test_frame = RDK.Item(args.frame, ITEM_TYPE_FRAME)
    assert test_frame.Valid(), f"Frame '{args.frame}' not found"
else:
    for f in all_frames:
        if "suction" in f.Name().lower() or "pickup" in f.Name().lower():
            test_frame = f
            break
    if test_frame is None and len(all_frames) > 0:
        test_frame = all_frames[0]

if test_frame is None:
    print("\nNo frame to test!")
    sys.exit(1)

print(f"\n{'='*60}")
print(f"Testing IK for: {test_frame.Name()}")
target_pose = test_frame.PoseAbs()
xyz = Pose_2_TxyzRxyz(target_pose)[:3]
dist = math.sqrt(sum((xyz[i] - robot_base_xyz[i])**2 for i in range(3)))
print(f"  World pos: [{xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f}]")
print(f"  Distance from robot base: {dist:.0f}mm")
print(f"{'='*60}")

# Test 1: Raw SolveIK (no OptimAxes)
print("\n[TEST 1] robot.SolveIK(pose) — no OptimAxes, no tool")
try:
    result = robot.SolveIK(target_pose)
    joints = result.list() if hasattr(result, 'list') else list(result)
    print(f"  Result: {[f'{j:.1f}' for j in joints]}")
    if all(abs(j) < 1e-6 for j in joints):
        print(f"  WARNING: all-zero solution (IK failed silently)")
except Exception as e:
    print(f"  EXCEPTION: {e}")

# Test 2: SolveIK with each tool
for tool in RDK.ItemList(ITEM_TYPE_TOOL):
    print(f"\n[TEST 2] SolveIK with tool={tool.Name()}")
    robot.setPoseTool(tool)
    try:
        result = robot.SolveIK(target_pose)
        joints = result.list() if hasattr(result, 'list') else list(result)
        print(f"  Result: {[f'{j:.1f}' for j in joints]}")
        if all(abs(j) < 1e-6 for j in joints):
            print(f"  WARNING: all-zero solution")
    except Exception as e:
        print(f"  EXCEPTION: {e}")

# Test 3: OptimAxes + MoveJ (the approach coupled_pivot_demo uses)
print(f"\n[TEST 3] OptimAxes + MoveJ (coupled_pivot_demo approach)")

# Ensure WorldFrame
world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
if not world_frame.Valid():
    station = RDK.ActiveStation()
    world_frame = RDK.AddFrame("WorldFrame", station)
    world_frame.setPose(eye(4))
    print(f"  Created WorldFrame at identity")

robot.setPoseFrame(world_frame)
print(f"  PoseFrame: {world_frame.Name()}")

from robodk.robomath import rotz

optim = {
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1,
    "RelOn_4": 1, "RelOn_5": 1, "RelOn_6": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50,
    "RelW_4": 50, "RelW_5": 50, "RelW_6": 50,
}

seeds = [
    ("zeros",     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ("transport", [0.0, -50.0, 15.0, 0.0, -15.0, -90.0]),
    ("elbow_up",  [0.0, 30.0, -90.0, 0.0, -30.0, 0.0]),
]

STEP_DEG = 30
n_steps = int(360 / STEP_DEG)

for tool in RDK.ItemList(ITEM_TYPE_TOOL):
    robot.setPoseTool(tool)
    robot.setParam("OptimAxes", optim)
    print(f"\n  Tool: {tool.Name()}")

    # First try without rotation
    found = False
    for seed_name, seed in seeds:
        robot.setJoints(seed)
        try:
            robot.MoveJ(target_pose)
            raw = robot.Joints()
            joints = raw.list() if hasattr(raw, 'list') else list(raw)
            if not all(abs(j) < 1e-6 for j in joints):
                print(f"    theta=  0  seed={seed_name}: OK joints={[f'{j:.1f}' for j in joints]}")
                found = True
                break
        except Exception:
            pass

    if found:
        continue

    # Z-rotation sweep
    for i in range(1, n_steps):
        angle_deg = STEP_DEG * i
        angle_rad = angle_deg * math.pi / 180.0
        rotated = target_pose * rotz(angle_rad)

        for seed_name, seed in seeds:
            robot.setJoints(seed)
            try:
                robot.MoveJ(rotated)
                raw = robot.Joints()
                joints = raw.list() if hasattr(raw, 'list') else list(raw)
                if not all(abs(j) < 1e-6 for j in joints):
                    print(f"    theta={angle_deg:3.0f}  seed={seed_name}: OK joints={[f'{j:.1f}' for j in joints]}")
                    found = True
                    break
            except Exception:
                pass

        if found:
            break

    if not found:
        print(f"    FAILED all {n_steps} rotations x {len(seeds)} seeds")

print("\nDone.")
