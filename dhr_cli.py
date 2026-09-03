"""
Interactive CLI client for DHR knitwear-cell simulation.

Connects to the DHR gRPC server (main.py) and sends commands.
See robert_checker_stuff/DHR_SIM_SETUP.md for full setup instructions.
"""

import sys
import os
import argparse

# Add knitwear-cell root so we can import the compiled proto modules
KNITWEAR_CELL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clones", "knitwear-cell")
sys.path.insert(0, KNITWEAR_CELL)

import grpc
import robo_pb2
import robo_pb2_grpc
from google.protobuf import empty_pb2

# ---------------------------------------------------------------------------
# Cell layout constants (from robodk.yaml)
# ---------------------------------------------------------------------------
MACHINES = list(range(1, 27))  # 1..26

LOCATIONS = {
    "machine":  {"id_range": (1, 26), "rows": 1, "slots": 1},
    "buffer":   {"id_range": (1, 1),  "rows": 1, "slots": 2},
    "rack":     {"id_range": (1, 1),  "rows": 7, "slots": 2},
    "cart":     {"id_range": (1, 1),  "rows": 3, "slots": 2},
}

LOCATION_TYPE_MAP = {
    "machine": robo_pb2.MACHINE,
    "rack":    robo_pb2.RACK,
    "cart":    robo_pb2.CART,
    "buffer":  robo_pb2.ROBOT_BUFFER,
}

TOOLS = ["GrabbingGripper", "MaintenanceGripper", "PoseCalibrationTool"]


def stream_responses(response_iter, label=""):
    """Print streaming gRPC responses until the stream ends."""
    for resp in response_iter:
        status = robo_pb2.TaskStatus.Name(resp.status)
        msg = resp.message if resp.message else ""
        print(f"  [{status}] {msg}")
    print()


def print_menu(title, options):
    """Print a numbered menu and return the list of (key, label) tuples."""
    print(f"\n{'=' * 50}")
    print(f"  {title}")
    print(f"{'=' * 50}")
    items = list(options)
    for i, (key, label) in enumerate(items, 1):
        print(f"  {i:>3}. {label}")
    print(f"    0. Back / Quit")
    print()
    return items


def pick(title, options):
    """Show a menu, get user choice. Returns the key or None for back/quit."""
    items = print_menu(title, options)
    while True:
        raw = input("  > ").strip()
        if raw == "0" or raw.lower() in ("q", "quit", "back", ""):
            return None
        try:
            idx = int(raw)
            if 1 <= idx <= len(items):
                return items[idx - 1][0]
        except ValueError:
            pass
        print(f"  Enter 1-{len(items)} or 0 to go back.")


def pick_int(prompt, lo, hi):
    """Prompt for an integer in [lo, hi]. Returns int or None."""
    while True:
        raw = input(f"  {prompt} ({lo}-{hi}, 0=back): ").strip()
        if raw == "0" or raw.lower() in ("q", "quit", "back", ""):
            return None
        try:
            val = int(raw)
            if lo <= val <= hi:
                return val
        except ValueError:
            pass
        print(f"  Enter {lo}-{hi} or 0 to go back.")


def pick_location(label):
    """Interactive location picker. Returns a robo_pb2.Location or None."""
    loc_type = pick(f"{label} — location type", [
        ("machine", "Machine (1-26)"),
        ("buffer",  "Robot Buffer (1 tray, 2 slots)"),
        ("rack",    "Rack (7 trays x 2 slots)"),
        ("cart",    "Cart (3 trays x 2 slots)"),
    ])
    if loc_type is None:
        return None

    info = LOCATIONS[loc_type]
    lo, hi = info["id_range"]

    loc_id = lo if lo == hi else pick_int("ID", lo, hi)
    if loc_id is None:
        return None

    row = 1
    if info["rows"] > 1:
        row = pick_int("Row (tray)", 1, info["rows"])
        if row is None:
            return None

    slot = 1
    if info["slots"] > 1:
        slot = pick_int("Slot", 1, info["slots"])
        if slot is None:
            return None

    return robo_pb2.Location(
        location_type=LOCATION_TYPE_MAP[loc_type],
        id=loc_id,
        row=row,
        slot=slot,
        item_type=robo_pb2.GARMENT_TRAY,
    )


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def cmd_oil_machine(stub):
    machine_id = pick_int("Machine to oil", 1, 26)
    if machine_id is None:
        return
    print(f"\n  Oiling machine {machine_id}...")
    req = robo_pb2.MaintenaceRequest(machine_id=machine_id)
    stream_responses(stub.Maintenance(req), f"Oil machine {machine_id}")


def cmd_move_bin(stub):
    print("\n  --- Source ---")
    src = pick_location("Source")
    if src is None:
        return
    print("  --- Destination ---")
    dst = pick_location("Destination")
    if dst is None:
        return
    print(f"\n  Moving bin...")
    req = robo_pb2.MoveRequest(source=src, destination=dst)
    stream_responses(stub.Move(req), "Move bin")


def cmd_change_tool(stub):
    tool = pick("Change tool to", [(t, t) for t in TOOLS])
    if tool is None:
        return
    print(f"\n  Changing tool to {tool}...")
    req = robo_pb2.ChangeToolRequest(tool_name=tool)
    stream_responses(stub.ChangeTool(req), f"Change tool → {tool}")


def cmd_open_gripper(stub):
    print("\n  Opening gripper...")
    resp = stub.OpenGripper(empty_pb2.Empty())
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_close_gripper(stub):
    print("\n  Closing gripper...")
    resp = stub.CloseGripper(empty_pb2.Empty())
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_open_door(stub):
    machine_id = pick_int("Machine", 1, 26)
    if machine_id is None:
        return
    side = pick("Side", [("front", "Front"), ("back", "Back")])
    if side is None:
        return
    side_enum = robo_pb2.FRONT if side == "front" else robo_pb2.BACK
    print(f"\n  Opening {side} door on machine {machine_id}...")
    req = robo_pb2.MachineDoorRequest(machine_id=machine_id, side=side_enum)
    resp = stub.OpenMachineDoor(req)
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_close_door(stub):
    machine_id = pick_int("Machine", 1, 26)
    if machine_id is None:
        return
    side = pick("Side", [("front", "Front"), ("back", "Back")])
    if side is None:
        return
    side_enum = robo_pb2.FRONT if side == "front" else robo_pb2.BACK
    print(f"\n  Closing {side} door on machine {machine_id}...")
    req = robo_pb2.MachineDoorRequest(machine_id=machine_id, side=side_enum)
    resp = stub.CloseMachineDoor(req)
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_start_oil_pump(stub):
    print("\n  Starting oil pump...")
    resp = stub.StartOilPump(empty_pb2.Empty())
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_stop_oil_pump(stub):
    print("\n  Stopping oil pump...")
    resp = stub.StopOilPump(empty_pb2.Empty())
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_get_state(stub):
    resp = stub.GetCurrentState(robo_pb2.GetCurrentStateRequest())
    print(f"\n  State: {resp.state}")
    print(f"  Joints: [{resp.th1:.1f}, {resp.th2:.1f}, {resp.th3:.1f}, {resp.th4:.1f}, {resp.th5:.1f}, {resp.th6:.1f}]")
    if resp.end_effector:
        ee = resp.end_effector
        print(f"  End effector: ({ee.x:.1f}, {ee.y:.1f}, {ee.z:.1f}) rot ({ee.rx:.1f}, {ee.ry:.1f}, {ee.rz:.1f})")
    print()


def cmd_acquire_safety_zone(stub):
    zone_id = pick_int("Safety zone ID", 1, 28)
    if zone_id is None:
        return
    resp = stub.AcquireSafetyZone(robo_pb2.SafetyZoneRequest(safety_zone_id=zone_id))
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


def cmd_release_safety_zone(stub):
    zone_id = pick_int("Safety zone ID", 1, 28)
    if zone_id is None:
        return
    resp = stub.ReleaseSafetyZone(robo_pb2.SafetyZoneRequest(safety_zone_id=zone_id))
    print(f"  [{robo_pb2.TaskStatus.Name(resp.status)}] {resp.message}")
    print()


# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------

MAIN_MENU = [
    ("oil",              "Oil a machine (full 32-point sequence)"),
    ("move",             "Move bin (machine/buffer/rack/cart)"),
    ("change_tool",      "Change tool"),
    ("state",            "Get current robot state"),
    ("open_gripper",     "Open gripper"),
    ("close_gripper",    "Close gripper"),
    ("open_door",        "Open machine door"),
    ("close_door",       "Close machine door"),
    ("start_oil_pump",   "Start oil pump (manual)"),
    ("stop_oil_pump",    "Stop oil pump (manual)"),
    ("acquire_zone",     "Acquire safety zone"),
    ("release_zone",     "Release safety zone"),
]

HANDLERS = {
    "oil":              cmd_oil_machine,
    "move":             cmd_move_bin,
    "change_tool":      cmd_change_tool,
    "state":            cmd_get_state,
    "open_gripper":     cmd_open_gripper,
    "close_gripper":    cmd_close_gripper,
    "open_door":        cmd_open_door,
    "close_door":       cmd_close_door,
    "start_oil_pump":   cmd_start_oil_pump,
    "stop_oil_pump":    cmd_stop_oil_pump,
    "acquire_zone":     cmd_acquire_safety_zone,
    "release_zone":     cmd_release_safety_zone,
}


def main():
    parser = argparse.ArgumentParser(description="DHR knitwear-cell simulation CLI")
    parser.add_argument("--host", default="localhost", help="gRPC server host")
    parser.add_argument("--port", type=int, default=50053, help="gRPC server port")
    args = parser.parse_args()

    addr = f"{args.host}:{args.port}"
    print(f"Connecting to DHR server at {addr}...")

    try:
        channel = grpc.insecure_channel(addr)
        stub = robo_pb2_grpc.RobotServiceStub(channel)
        # Quick connectivity check
        stub.GetCurrentState(robo_pb2.GetCurrentStateRequest())
        print("Connected!\n")
    except grpc.RpcError as e:
        print(f"[ERROR] Cannot connect to {addr}")
        print(f"        Is the DHR server running?")
        print(f"        Start it: cd clones\\knitwear-cell && set ENV_MODE=local && python main.py")
        print(f"        Error: {e}")
        sys.exit(1)

    while True:
        choice = pick("DHR Knitwear-Cell Simulation", MAIN_MENU)
        if choice is None:
            print("Bye!")
            break
        handler = HANDLERS.get(choice)
        if handler:
            try:
                handler(stub)
            except grpc.RpcError as e:
                print(f"  [gRPC ERROR] {e.code()}: {e.details()}")
                print()
            except KeyboardInterrupt:
                print("\n  Interrupted.")
                print()


if __name__ == "__main__":
    main()
