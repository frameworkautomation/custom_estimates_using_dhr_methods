"""
Place cone meshes into existing RoboDK frames with optional coloring.

Usage:
    python robert_checker_stuff/place_cones.py \
      --robodk-ip 172.23.208.1 \
      --frames "Base_Right_0,Base_Right_1,Base_Right_2" \
      --step-file my_assets/sams_simple_cone.stp \
      --colors "#FF0000,#00FF00,#0000FF"

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
This code interacts with a physical robot simulation. All changes require review.
"""

import sys
import os
import argparse
import random

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT,
)


# ── CONNECT ──────────────────────────────────────────────────────────────────

def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def wsl_to_win(path):
    """Convert /mnt/c/... to C:\\... for RoboDK."""
    if path.startswith("/mnt/"):
        drive = path[5]
        return f"{drive.upper()}:{path[6:]}".replace("/", "\\")
    return path


# ── CONE PLACEMENT ───────────────────────────────────────────────────────────

def _hex_to_rgba(hex_str):
    """Convert '#RRGGBB' or 'RRGGBB' to [R, G, B, A] with values 0-1."""
    h = hex_str.lstrip("#")
    assert len(h) == 6, f"Bad hex color: {hex_str}"
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return [r / 255.0, g / 255.0, b / 255.0, 1.0]


def _random_color(rng):
    """Generate a random saturated RGBA color."""
    # CHECK_LATER: these random colors might be too dark or too similar.
    # May want to use HSV with fixed saturation/value for better visual spread.
    return [rng.random(), rng.random(), rng.random(), 1.0]


def place_cones(RDK, frames, step_file, colors=None, seed=None):
    """Import a STEP cone mesh into each frame.

    Args:
        RDK: Robolink instance
        frames: list of RoboDK frame Items
        step_file: path to STEP file to import
        colors: optional list of hex color strings (e.g. ['#FF0000', '#00FF00']).
                Cycles through the list if fewer colors than frames.
        seed: optional int seed for random color generation (used if colors is None)

    Returns:
        list of created RoboDK cone Items
    """
    assert os.path.exists(step_file), f"STEP file not found: {step_file}"
    assert len(frames) > 0, "No frames to place cones in"

    # Parse colors
    rgba_list = None
    rng = None
    if colors:
        rgba_list = [_hex_to_rgba(c) for c in colors]
    elif seed is not None:
        rng = random.Random(seed)

    # Import STEP once, then Copy/Paste for the rest
    abs_step = os.path.abspath(step_file)
    win_step = wsl_to_win(abs_step)

    first_cone = RDK.AddFile(win_step, frames[0])
    assert first_cone.Valid(), f"Failed to import STEP file: {win_step}"
    print(f"[IMPORT] Loaded '{step_file}' into '{frames[0].Name()}'")

    cones = [first_cone]

    # CHECK_LATER: Copy/Paste may not preserve the relative pose inside the
    # frame. If cones appear offset, we may need to explicitly setPose after
    # pasting. Test with actual RoboDK to verify.
    for i, frame in enumerate(frames[1:], start=1):
        first_cone.Copy()
        pasted = frame.Paste()
        assert pasted.Valid(), f"Paste failed for frame '{frame.Name()}'"
        cones.append(pasted)
        print(f"[PASTE] Cone in '{frame.Name()}'")

    # Apply colors
    for i, cone in enumerate(cones):
        if rgba_list:
            color = rgba_list[i % len(rgba_list)]
            cone.Recolor(color)
        elif rng:
            cone.Recolor(_random_color(rng))

    return cones


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Place cone meshes into existing RoboDK frames."
    )
    parser.add_argument("--robodk-ip", default=None,
                        help="RoboDK IP (default: localhost then 172.23.208.1)")
    parser.add_argument("--frames", required=True,
                        help="Comma-separated frame names to place cones in")
    parser.add_argument("--step-file", required=True,
                        help="Path to STEP file for cone mesh")
    parser.add_argument("--colors", default=None,
                        help="Comma-separated hex colors (e.g. '#FF0000,#00FF00')")
    parser.add_argument("--color-seed", type=int, default=None,
                        help="Seed for random color generation")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    # Find frames
    frame_names = [f.strip() for f in args.frames.split(",")]
    frames = []
    for name in frame_names:
        f = RDK.Item(name, ITEM_TYPE_FRAME)
        assert f.Valid(), f"Frame not found: {name}"
        frames.append(f)

    color_list = None
    if args.colors:
        color_list = [c.strip() for c in args.colors.split(",")]

    cones = place_cones(RDK, frames, args.step_file,
                        colors=color_list, seed=args.color_seed)

    print(f"[DONE] Placed {len(cones)} cones")


if __name__ == "__main__":
    main()
