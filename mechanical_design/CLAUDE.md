# Mechanical Design — Cone Holder Plate Modification

## Goal

The cone holders at the robot base need to be pushed further out. This means
modifying the relevant Robot-buffer plates and sending updated DXFs to Send Cut Send.

## Branch

`robot-buffer-cone-holder-plates` (branched from `collision_free_path_planning`)

## Source files (in clones/)

All files live in the `Lachezar_Robot` branch of the `robots` repo, which is already
checked out at:

```
clones/robots/Robot-buffer/
```

The files are on disk — no checkout step needed.

## DXF cut files (what you modify and send to Send Cut Send)

These are the flat-pattern laser cut profiles. All geometry is on layer `0`,
continuous linetype — no bend lines embedded. Edit these in Rhino, export as DXF.

| Plate | DXF path (relative to clones/robots/Robot-buffer/) |
|-------|-----------------------------------------------------|
| Left plate | `laser-V3/left-plate/left-plate-laser.DXF` |
| Right plate | `laser-V3/right-plate/right-plate.DXF` |
| Inner plate left | `laser-V3/inner-plate-left-robot-buffer_FRM/inner-plate-left-robot-buffer_FRM.DXF` |
| Inner plate right | `laser-V3/inner-plate-right-robot-buffer_FRM/inner-plate-right-robot-buffer_FRM.DXF` |
| Side plate left | `laser-V3/side-plate-left-robot-buffer_FRM/side-plate-left-robot-buffer_FRM.DXF` |
| Side plate right | `laser-V3/side-plate-right-robot-buffer_FRM/side-plate-right-robot-buffer_FRM.DXF` |
| End position plate left | `laser-V3/end-position-plate-left/end-position-plate-left.DXF` |
| End position plate right | `laser-V3/end-position-plate-right/end-position-plate-right.DXF` |
| T plate (flat, no bend) | `laser-V3/12.05.2026/T plate.DXF` |
| V plate (flat, no bend) | `laser-V3/12.05.2026/V plate.DXF` |
| X plate (flat, no bend) | `laser-V3/12.05.2026/X plate.DXF` |
| Connecting plate (flat) | `laser-V3/12.05.2026/connecting-plate-160x160x5-90deg.DXF` |

## Bending drawings (for reference — do NOT modify)

For plates with bends, there are separate PDFs that document the bend location,
angle, and radius. These go to the bending shop (or are uploaded alongside the DXF
to Send Cut Send's sheet metal service). The bend lines are NOT in the DXF — they
are only in these PDFs.

| Plate | Bending drawing |
|-------|----------------|
| Left plate | `laser-V3/left-plate/left-plate-bending.pdf` |
| Right plate | `laser-V3/right-plate/right-plate.pdf` |
| Inner plate left | `laser-V3/inner-plate-left-robot-buffer_FRM/inner-plate-left-robot-buffer_FRM.pdf` |
| Inner plate right | `laser-V3/inner-plate-right-robot-buffer_FRM/inner-plate-right-robot-buffer_FRM.pdf` |
| Side plate left | `laser-V3/side-plate-left-robot-buffer_FRM/side-plate-left-robot-buffer_FRM.pdf` |
| Side plate right | `laser-V3/side-plate-right-robot-buffer_FRM/side-plate-right-robot-buffer_FRM.pdf` |
| End position plate left | `laser-V3/end-position-plate-left/end-position-plate-left.pdf` |
| End position plate right | `laser-V3/end-position-plate-right/end-position-plate-right.pdf` |

## 3D reference

```
clones/robots/Robot-buffer/simulation/robot-buffer-V3_FRM.STEP
clones/robots/Robot-buffer/simulation/robot-buffer-V3_FRM-without-box.STEP
clones/robots/Robot-buffer/Robot-buffer-base.STEP
```

Rhino can open these STEP files directly. Useful for checking how the modified flat
patterns will assemble in 3D.

## Workflow

1. Open the relevant DXF flat pattern in Rhino
2. Modify 2D geometry:
   - Move mounting hole pattern further out
   - Move any associated sloped/angled cut lines
   - Keep the overall bend line position fixed (it won't be in the DXF — track it
     visually by noting which line in the flat pattern is the fold line)
3. Check that the overall blank dimensions are still correct if the bend line
   hasn't moved — if the mounting holes are on the folded leg, moving them out
   changes the leg length and therefore the flat blank size
4. Export modified DXF from Rhino (keep layer `0` as the cut profile layer)
5. Save modified DXFs into this repo under `mechanical_design/modified_dxfs/`
   before uploading to Send Cut Send

## Send Cut Send notes

- The DXF files are laser cut profiles only (no bend indicators)
- For bent plates, Send Cut Send needs to know bend angle, bend radius, and
  material thickness — this info is in the bending PDFs above
- Send Cut Send sheet metal service accepts 3D STEP if you want them to unfold
  it themselves, but the existing workflow is: flat pattern DXF + bending drawing
- Confirm with Send Cut Send whether to upload one DXF per part or combined

## Key constraint

When moving the mounting pattern, determine whether the holes are on the pre-bend
flat section or on the folded leg. If on the folded leg, moving them out increases
the leg length, which changes where the bend line sits on the flat blank — the
bending PDF will need to be updated too. If on the flat base section, the bend
line position is unaffected and only the DXF cut profile changes.
