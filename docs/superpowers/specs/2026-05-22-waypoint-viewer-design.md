# Waypoint Viewer — Design Spec

## Goal

A browser-based 3D tool for visually creating and deleting bidirectional edges between robot waypoints, with all changes written back to `all_waypoints.yaml`.

## Architecture

FastAPI Python backend + React + react-three-fiber frontend.

- FastAPI resolves `all_waypoints.yaml` via `robo_dk_output/waypoint_sources.json` (reads the `output` field), then reads and writes that file.
- React app is served as static files by FastAPI on the same port — no separate dev server in production.
- No database. YAML is the single source of truth. Frontend re-fetches after every mutation.
- Run: `uvicorn waypoint_viewer.backend.main:app` from repo root, open browser on Windows.

## File Layout

```
waypoint_viewer/
  backend/
    main.py       FastAPI app — mounts static frontend, exposes REST endpoints
    yaml_io.py    read/write all_waypoints.yaml (load, add_edge, delete_edge)
  frontend/
    src/
      App.jsx           root layout: Viewport3D left + DetailPanel right
      Viewport3D.jsx    react-three-fiber 3D scene
      DetailPanel.jsx   right panel — inspect / edge creation / edge detail
      api.js            fetch wrappers for all API calls
    package.json
  README.md
```

## Layout

Horizontal split: 3D viewport takes ~65% width on the left, detail panel takes ~35% on the right. No toolbar. No status bar.

## 3D Viewport

### Rendering

Each waypoint is rendered as:
- A **sphere** at its world-space position (x, y, z from YAML), coloured by IK solve status:
  - **Orange** — Cartesian-only, no `joints:` yet (unsolved)
  - **Green** — `ik_collision_verified: true` (joints solved, static pose is collision-free)
  - **Red** — `reachable: false` (all IK solutions had static collisions)
- An **RGB axis triad** extending from the sphere center, oriented by rx/ry/rz (ZYX Euler, same convention as the GH scripts: R = Rz * Ry * Rx)
  - Red arrow = local X axis
  - Green arrow = local Y axis
  - Blue arrow = local Z axis

Edge lines are rendered between waypoint positions, coloured by `tested` field:
- `null` (untested) → grey
- `true` (collision-free) → bright green
- `false` (collision detected) → red

### Camera

`OrbitControls` with orbit, pan, and scroll-to-zoom enabled.

### Interaction

| Action | Effect |
|---|---|
| Left-click waypoint (first) | Highlights sphere with blue (#2af) glow/ring. Fills **From** slot in panel. Panel switches to edge-creation mode. |
| Left-click waypoint (second) | Fills **To** slot. "Create Edge" button activates. |
| Left-click empty space | Deselects everything. Panel returns to idle state. |
| Left-click edge line | Highlights edge. Panel shows edge detail + Delete button. |
| Right-click waypoint | Shows full waypoint info in panel (name, source, move_type, j7, all incoming + outgoing edges with tested status). Does not affect From/To selection. |

## Detail Panel

### Idle state (nothing selected)

- Legend:
  - Waypoint spheres: orange = unsolved, green = IK verified, red = unreachable
  - Edge lines: grey = untested, bright green = collision-free, red = collision
- Counts: N waypoints, M edges
- Filter dropdown: filter 3D view by `source` (grasshopper / human) and/or `move_type` (MoveJ / MoveL). Waypoints that don't match are hidden in the 3D scene; their edges are also hidden.

### Edge-creation mode (one or two waypoints left-clicked)

```
FROM
  base_cone_grab_0
  MoveL · source: grasshopper · j7: 0.0

TO
  — click another waypoint —     ← or filled when second clicked

[ Create Edge ]                  ← active only when both slots filled
```

On "Create Edge": POST /api/edges with {from, to}. Server creates **both** A→B and B→A entries in the YAML (each with `tested: null`). Frontend re-fetches and re-renders.

### Inspect mode (right-click on waypoint)

```
base_cone_grab_0
MoveL · source: grasshopper
j7: 0.0

Outgoing edges (2)
  → base_cone_grab_0_approach   tested: null
  → transport_j7_0              tested: true

Incoming edges (1)
  ← base_cone_grab_0_approach   tested: null
```

### Edge detail mode (left-click on edge line)

```
base_cone_grab_0 → transport_j7_0
tested: true

[ Delete Edge ]
```

On "Delete Edge": DELETE /api/edges/{from}/{to}. Server removes **both** A→B and B→A from the YAML. Frontend re-fetches.

## API

All endpoints read/write `all_waypoints.yaml` (resolved via `waypoint_sources.json`).

```
GET    /api/waypoints
       Returns: { waypoints: [...], edges: [...] }

POST   /api/edges
       Body: { from: "name_a", to: "name_b" }
       Creates both A→B and B→A with tested: null.
       Returns: { created: ["name_a->name_b", "name_b->name_a"] }

DELETE /api/edges/{from}/{to}
       Removes both A→B and B→A entries.
       Returns: { deleted: ["from->to", "to->from"] }
```

Frontend re-fetches GET /api/waypoints after every POST or DELETE.

## Data Notes

- Euler angle convention: `R = Rz * Ry * Rx` (same as GH scripts). Use this to build the rotation matrix for the axis triad arrows in three.js.
- Edge directionality: A→B and B→A are always stored as separate entries in the YAML, each with their own `tested` field. This is enforced at the API level — POST always creates the pair; DELETE always removes the pair.
- YAML schema for edges:
  ```yaml
  edges:
    - from: base_cone_grab_0
      to:   transport_j7_0
      tested: null
  ```

## Out of Scope

- Editing waypoint positions or orientation
- Creating new waypoints
- Running collision checks from the UI
- Authentication or multi-user support
- Real-time sync (polling or websockets) — re-fetch on mutation is sufficient
