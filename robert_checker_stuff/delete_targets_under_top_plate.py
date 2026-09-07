"""
Delete all targets found recursively under top_plate_frame for a given machine.

Usage:
    python robert_checker_stuff/delete_targets_under_top_plate.py --robodk-ip 172.23.208.1 --machine 3
    python robert_checker_stuff/delete_targets_under_top_plate.py --robodk-ip 172.23.208.1 --machine all
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def delete_targets_recursive(item, count=0):
    try:
        children = item.Childs()
    except Exception:
        return count
    for child in children:
        try:
            if child.Type() == ITEM_TYPE_TARGET:
                print(f"  [DELETE] {child.Name()}")
                child.Delete()
                count += 1
            else:
                count = delete_targets_recursive(child, count)
        except Exception:
            continue
    return count


def find_frame_recursive(parent, name):
    try:
        for child in parent.Childs():
            try:
                if child.Name() == name and child.Type() == ITEM_TYPE_FRAME:
                    return child
                found = find_frame_recursive(child, name)
                if found is not None:
                    return found
            except Exception:
                continue
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description="Delete all targets under top_plate_frame")
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--machine", default="all",
                    help="Machine number (e.g. 3) or 'all' for all machines")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    if args.machine == "all":
        machines = [1, 2, 3, 4, 5, 6]
    else:
        machines = [int(args.machine)]

    total = 0
    for m in machines:
        base_name = f"Machine{m}Base"
        base = RDK.Item(base_name, ITEM_TYPE_FRAME)
        if not base.Valid():
            continue

        tp = find_frame_recursive(base, "top_plate_frame")
        if tp is None:
            print(f"[SKIP] No top_plate_frame under {base_name}")
            continue

        print(f"[INFO] Cleaning targets under {base_name}/top_plate_frame")
        count = delete_targets_recursive(tp)
        total += count
        print(f"  {count} target(s) deleted")

    print(f"\n[DONE] {total} total target(s) deleted")


if __name__ == "__main__":
    main()
