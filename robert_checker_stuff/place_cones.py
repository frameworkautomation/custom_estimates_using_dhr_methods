"""
Place cones in RoboDK by interpolating frames between two endpoint poses.

Replaces Grasshopper-based cone placement with pure Python/RoboDK.

Two main functions:
  - interpolate_frames: generate N frames (including endpoints) between two poses
  - place_cones: import a STEP mesh into each frame, with optional coloring

Usage:
    python robert_checker_stuff/place_cones.py \
      --robodk-ip 172.23.208.1 \
      --frame-start "BinStart" \
      --frame-end "BinEnd" \
      --n 6 \
      --name-prefix "Base_Right" \
      --parent "bin_0_group" \
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
from robodk.robomath import Pose_2_UR, UR_2_Pose, invH

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]


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


# ── INTERPOLATION ────────────────────────────────────────────────────────────

def interpolate_frames(RDK, pose_start, pose_end, n, name_prefix, parent):
    """Create N frames interpolated between pose_start and pose_end (inclusive).

    Args:
        RDK: Robolink instance
        pose_start: 4x4 Mat — pose of the first frame
        pose_end: 4x4 Mat — pose of the last frame
        n: total number of frames to create (including endpoints, minimum 2)
        name_prefix: frames named {name_prefix}_0 through {name_prefix}_{n-1}
        parent: RoboDK Item to parent all frames under

    Returns:
        list of created RoboDK frame Items
    """
    assert n >= 2, f"Need at least 2 frames (got {n})"
    assert parent.Valid(), f"Parent item is not valid"

    # CHECK_LATER: UR-format interpolation does linear interpolation of the
    # axis-angle rotation representation. This works well for small rotations
    # but may behave unexpectedly for large rotation differences (>180 degrees).
    # If cones end up with weird orientations, consider quaternion SLERP instead.
    pose_delta = invH(pose_start) * pose_end
    delta_ur = Pose_2_UR(pose_delta)

    frames = []
    for i in range(n):
        t = i / (n - 1)  # 0.0 for first, 1.0 for last
        rel = UR_2_Pose([c * t for c in delta_ur])
        pose_i = pose_start * rel

        name = f"{name_prefix}_{i}"
        frame = RDK.AddFrame(name, parent)
        frame.setPose(pose_i)
        frames.append(frame)
        print(f"[CREATE] Frame '{name}' (t={t:.3f})")

    return frames


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
        frames: list of RoboDK frame Items (from interpolate_frames)
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
    # CHECK_LATER: AddFile imports relative to RoboDK's working dir or absolute.
    # Using abspath to be safe. If this fails on some setups, may need to
    # convert to Windows path format for the RoboDK side.
    abs_step = os.path.abspath(step_file)
    first_cone = RDK.AddFile(abs_step, frames[0])
    assert first_cone.Valid(), f"Failed to import STEP file: {abs_step}"
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
        description="Place cones by interpolating between two RoboDK frames."
    )
    parser.add_argument("--robodk-ip", default=None,
                        help="RoboDK IP (default: localhost then 172.23.208.1)")
    parser.add_argument("--frame-start", required=True,
                        help="Name of the start frame in RoboDK")
    parser.add_argument("--frame-end", required=True,
                        help="Name of the end frame in RoboDK")
    parser.add_argument("--n", type=int, required=True,
                        help="Number of frames to create (including endpoints)")
    parser.add_argument("--name-prefix", required=True,
                        help="Prefix for frame names (e.g. 'Base_Right')")
    parser.add_argument("--parent", required=True,
                        help="Name of parent item in RoboDK to nest frames under")
    parser.add_argument("--step-file", required=True,
                        help="Path to STEP file for cone mesh")
    parser.add_argument("--colors", default=None,
                        help="Comma-separated hex colors (e.g. '#FF0000,#00FF00')")
    parser.add_argument("--color-seed", type=int, default=None,
                        help="Seed for random color generation")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    # Find start/end frames
    frame_start = RDK.Item(args.frame_start, ITEM_TYPE_FRAME)
    assert frame_start.Valid(), f"Start frame not found: {args.frame_start}"
    frame_end = RDK.Item(args.frame_end, ITEM_TYPE_FRAME)
    assert frame_end.Valid(), f"End frame not found: {args.frame_end}"

    # Find parent
    parent = RDK.Item(args.parent)
    assert parent.Valid(), f"Parent item not found: {args.parent}"

    # CHECK_LATER: using PoseAbs to get world poses. If the start/end frames
    # are nested deep in a hierarchy, these world poses are then set on frames
    # that are children of `parent`. The frames will be positioned correctly in
    # world space but their local pose relative to parent will encode the full
    # world transform. This is fine if parent is at identity, but may be
    # confusing if parent has a non-identity pose. Consider using relative
    # poses if needed.
    pose_start = frame_start.PoseAbs()
    pose_end = frame_end.PoseAbs()

    print(f"[INFO] Interpolating {args.n} frames from '{args.frame_start}' to '{args.frame_end}'")
    print(f"[INFO] Prefix: '{args.name_prefix}', parent: '{args.parent}'")

    frames = interpolate_frames(RDK, pose_start, pose_end, args.n,
                                args.name_prefix, parent)

    color_list = None
    if args.colors:
        color_list = [c.strip() for c in args.colors.split(",")]

    cones = place_cones(RDK, frames, args.step_file,
                        colors=color_list, seed=args.color_seed)

    print(f"[DONE] Placed {len(cones)} cones")


if __name__ == "__main__":
    main()
