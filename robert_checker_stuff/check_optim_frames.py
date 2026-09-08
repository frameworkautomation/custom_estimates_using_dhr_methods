"""
Interactive CLI to inspect and test optimization frames.

Lists optim frames, moves robot to cone child frames using the selected
optimization frame's j7 value.

Usage:
    python robert_checker_stuff/check_optim_frames.py --robodk-ip 172.23.208.1 --machine 4
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import eye, Pose_2_TxyzRxyz

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]


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
    raise RuntimeError(f"Robot not found")


def find_child(parent, name):
    try:
        for child in parent.Childs():
            try:
                if child.Name() == name and child.Type() == 3:
                    return child
                found = find_child(child, name)
                if found is not None:
                    return found
            except:
                continue
    except:
        pass
    return None


def collect_frames(parent):
    """Collect all frame children recursively. Returns list of (name, item)."""
    results = []
    try:
        for child in parent.Childs():
            try:
                if child.Type() == 3:
                    results.append(child)
                    results.extend(collect_frames(child))
            except:
                continue
    except:
        pass
    return results


def set_optim(robot, j7_val):
    optim = {
        "AbsOn_7": 1, "AbsJnt_7": j7_val, "AbsW_7": 100,
        "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
        "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
        "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
        "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
        "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
    }
    robot.setParam("OptimAxes", optim)
    curr = robot.Joints().list()
    if len(curr) >= 7 and curr[6] == 0.0:
        curr[6] = 0.001
        robot.setJoints(curr)


def main():
    ap = argparse.ArgumentParser(description="Interactive optimization frame tester")
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--machine", type=int, required=True)
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    robot.setPoseFrame(world_frame)

    machine_base = RDK.Item(f"Machine{args.machine}Base", ITEM_TYPE_FRAME)
    assert machine_base.Valid(), f"Machine{args.machine}Base not found"

    # Get rail base for j7 calculation (DHR uses PoseWrt rail base)
    rail_base = robot.Parent().Parent().Parent()
    print(f"[INFO] Rail base: {rail_base.Name()}")

    def frame_to_j7(frame):
        """Convert frame position to j7 value using PoseWrt rail base (DHR method)."""
        pose_wrt = frame.PoseWrt(rail_base)
        return Pose_2_TxyzRxyz(pose_wrt)[0]

    optim_folder = find_child(machine_base, "optim_frames")
    assert optim_folder is not None, \
        f"'optim_frames' not found under Machine{args.machine}Base"

    # Collect optim frames
    optim_frames = collect_frames(optim_folder)
    assert len(optim_frames) > 0, "No frames found under optim_frames"

    optim_dict = {}
    print(f"\nOptimization frames under Machine{args.machine}Base/optim_frames:")
    for i, frame in enumerate(optim_frames):
        j7 = frame_to_j7(frame)
        optim_dict[frame.Name()] = (frame, j7)
        print(f"  [{i+1}] {frame.Name():20s} j7={j7:.0f} (world X={Pose_2_TxyzRxyz(frame.PoseAbs())[0]:.0f})")

    # Also show cone frame j7 equivalents
    tp = find_child(machine_base, "top_plate_frame")
    assert tp is not None, "top_plate_frame not found"
    print(f"\nCone frame j7 equivalents:")
    for child in tp.Childs():
        try:
            if child.Type() == 3:
                j7 = frame_to_j7(child)
                print(f"  {child.Name():35s} j7={j7:.0f}")
        except:
            pass

    cone_names = []
    for child in tp.Childs():
        try:
            if child.Type() == 3:
                cone_names.append(child.Name())
        except:
            pass

    # Interactive loop
    print(f"\nCones: {cone_names}")
    print("\nCommands:")
    print("  list                          — list optim frames")
    print("  goto <optim_name>             — move j7 to optim frame position")
    print("  test <cone> <optim_name>      — MoveJ to all child frames of cone")
    print("  move <cone> <child> <optim>   — MoveJ to specific child frame")
    print("  tool <name>                   — set active tool")
    print("  home                          — move to home (all zeros)")
    print("  q                             — quit")

    while True:
        try:
            cmd = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        parts = cmd.split()

        if parts[0] == "q":
            break

        elif parts[0] == "list":
            print("\nOptim frames:")
            for name, (_, j7) in optim_dict.items():
                print(f"  {name:20s} j7={j7:.0f}")
            print(f"\nCones: {cone_names}")

        elif parts[0] == "goto" and len(parts) >= 2:
            name = parts[1]
            if name not in optim_dict:
                print(f"Unknown optim frame '{name}'. Available: {list(optim_dict.keys())}")
                continue
            _, j7 = optim_dict[name]
            print(f"Moving j7 to {j7:.0f}...")
            curr = robot.Joints().list()
            curr[6] = j7
            robot.MoveJ(curr)
            print(f"Done — j7={robot.Joints().list()[6]:.0f}")

        elif parts[0] == "tool" and len(parts) >= 2:
            tool = RDK.Item(parts[1], ITEM_TYPE_TOOL)
            if tool.Valid():
                robot.setPoseTool(tool)
                print(f"Tool set: {parts[1]}")
            else:
                print(f"Tool '{parts[1]}' not found")

        elif parts[0] == "home":
            robot.MoveJ([0, 0, 0, 0, 0, 0, 0])
            print("Home")

        elif parts[0] == "move" and len(parts) >= 4:
            cone_name, child_name, optim_name = parts[1], parts[2], parts[3]
            if optim_name not in optim_dict:
                print(f"Unknown optim '{optim_name}'. Available: {list(optim_dict.keys())}")
                continue
            cone = find_child(tp, cone_name)
            if cone is None:
                print(f"Cone '{cone_name}' not found")
                continue
            child = find_child(cone, child_name)
            if child is None:
                print(f"Child '{child_name}' not found under '{cone_name}'")
                continue

            _, j7 = optim_dict[optim_name]
            pose = child.PoseAbs()
            xyz = Pose_2_TxyzRxyz(pose)
            print(f"Target: {cone_name}/{child_name} X={xyz[0]:.0f} Y={xyz[1]:.0f} Z={xyz[2]:.0f}")
            print(f"OptimAxes j7={j7:.0f} (from {optim_name})")

            set_optim(robot, j7)
            try:
                robot.MoveJ(pose)
                print(f"OK — j7={robot.Joints().list()[6]:.0f}")
            except Exception as e:
                print(f"FAIL — {e}")

        elif parts[0] == "test" and len(parts) >= 3:
            cone_name, optim_name = parts[1], parts[2]
            if optim_name not in optim_dict:
                print(f"Unknown optim '{optim_name}'. Available: {list(optim_dict.keys())}")
                continue
            cone = find_child(tp, cone_name)
            if cone is None:
                print(f"Cone '{cone_name}' not found")
                continue

            _, j7 = optim_dict[optim_name]
            children = collect_frames(cone)
            print(f"Testing {len(children)} frames with j7={j7:.0f}...\n")
            set_optim(robot, j7)

            ok = 0
            fail = 0
            for child in children:
                pose = child.PoseAbs()
                xyz = Pose_2_TxyzRxyz(pose)
                try:
                    robot.MoveJ(pose)
                    actual = robot.Joints().list()[6]
                    print(f"  [OK]   {child.Name():20s} X={xyz[0]:.0f} j7={actual:.0f}")
                    ok += 1
                except Exception as e:
                    print(f"  [FAIL] {child.Name():20s} X={xyz[0]:.0f} — {e}")
                    fail += 1

            print(f"\n  {ok} OK, {fail} FAIL")

        else:
            print("Unknown command. Type 'q' to quit.")


if __name__ == "__main__":
    main()
