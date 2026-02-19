# Medieval Town Generator — Repository Review

## High-level understanding

This repository currently contains a **single Unreal Engine generator actor** (`AMedievalTownGenerator`) split across one header and one implementation file. The project is a self-contained procedural pipeline that generates a walled medieval mountain town by executing staged phases in a fixed order:

1. River waypoint generation
2. Terrain cache + terrain mesh generation (including flattening/carving)
3. River world path generation
4. Wall generation via grammar modules
5. Radiocentric road network generation
6. District-aware building lot placement
7. Mesh spawning (plaza, buildings, roads, river, bridges)
8. Forest placement
9. Mountain placement

## Core architecture

- **Primary actor**: `AMedievalTownGenerator`
- **Main entrypoint**: `GenerateTown()`
- **Data model** includes:
  - Road graph nodes/edges
  - Building lot descriptors (style, district, footprint, floors, etc.)
  - River spline/waypoint data
  - Persisted layout state (`FSavedTownLayout`) for seed-locking and replaying a generated layout

The actor exposes extensive `UPROPERTY` controls grouped by category (`General`, `Terrain`, `Buildings`, `Walls`, `Roads`, `River`, `Foliage`, `Mountains`, `Materials`) so generation behavior is tunable directly in the editor.

## Systems present in code

### 1) Terrain
- Deterministic multi-octave value noise.
- Terrain cache for repeated sampling.
- Town-center flattening blend and river channel depression.
- Flatness testing with a slope threshold for placement validity.

### 2) River
- Procedural path through town bounds from an entry direction.
- River proximity and segment crossing tests used by roads/buildings.
- River surface mesh generated from cached world path.

### 3) Roads
- Radiocentric network: radial spokes, inner/outer rings, and optional mid-block connectors.
- Roads are elevated to terrain and can be flagged as bridges when crossing river segments.
- Per-tier road width control (primary/secondary/tertiary/river path).

### 4) Walls
- Grammar-driven module sequencing (`W`, `T`, `G`, `P`, `B`).
- Wall perimeter and gate positions integrate with road routing.
- Dedicated spawn logic for sections, corner towers, and gate towers.

### 5) Buildings
- District-aware placement logic and style selection.
- Road-lining and scatter behavior with overlap rejection.
- Modular construction: foundation, floor stacks, multiple roof types, chimneys, props.

### 6) Environment dressing
- Forest ring outside wall radius with noise-based density.
- Procedural mountain peaks around the settlement perimeter.

### 7) Persistence
- Save/load support for generated layouts (`SaveCurrentLayout` / `LoadSavedLayout`) with deterministic regeneration from a locked seed + stored transforms.

## Practical observations

- The codebase is intentionally monolithic and readable by section markers (`§1` ... `§14`), which makes iteration straightforward for one-developer workflows.
- Generation order is well thought out: dependencies are respected (e.g., roads after gates/walls; buildings after roads/river).
- The project is already close to a “tool actor” pattern suitable for editor-time worldbuilding and rapid art iteration.

## Suggested evolution path (if you continue expanding)

- Split each major system into its own `.h/.cpp` helper module (terrain, roads, buildings, walls) to reduce compile overhead and improve maintainability.
- Add lightweight debug visualization toggles (district colors, rejected lot reasons, slope heatmap).
- Add optional LOD/instancing strategy for houses to improve runtime performance in large towns.
- Add export of generated graph/layout to JSON for external analysis or replay.

