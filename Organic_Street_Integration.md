# Organic Medieval Street Network Integration (UE5.3+)

## Generator v2: block growth + burgage parcels

The street/building layout pipeline was reworked for natural, historically
grounded morphology. The old design (radial attractor accretion + cursor/
scatter building placement) produced a star of spokes around the market with
free-floating buildings; v2 replaces both halves:

### Streets (`OrganicStreetGenerator.h/.cpp`)
- **Stage 2 — Primary trunks.** Terrain-aware A* routes from gates and
  bridges into an **irregular market-square ring** (5-7 nodes around the
  resolved plaza centre). Later routes **merge** into earlier ones when they
  pass within `MergeDistance`, producing trunk roads with Y-junctions instead
  of a radial star. Routes are split at river-band transitions so only the
  actual crossing is flagged `bIsBridge`.
- **Stage 3 — Block growth.** Streets sprout quasi-perpendicularly from
  existing frontages and grow step-by-step with heading noise and contour
  deflection until they (a) strike another street → T-junction (closing a
  block), (b) snap to a nearby node, or (c) stop as a dead-end lane — the
  Parish/Müller local-constraint scheme. Four passes (secondary off
  primaries, secondary off secondaries, lanes, core alleys) give irregular
  blocks, back lanes, and cul-de-sacs.
- The market square interior is a hard obstacle for routing and growth, so
  the plaza stays open and enclosed.
- Density is a field (wide/shallow market pull + 2-octave noise + bridgehead
  boosts + wall fade), not a hard core radius, so no concentric artefacts.

### Parcels & buildings (`OrganicParcelGenerator.h/.cpp`)
- Every street frontage is subdivided into **burgage plots**: narrow frontage
  (~4-9 m), several times deeper than wide, scaled by tier and local
  urbanity. (Historical reference: 1-4 perches frontage, 5-20 deep.)
- Houses sit on the frontage line (tiny setbacks in the core), gable to the
  street, with high party-wall row probability near the market → continuous
  street walls. Plot rears stay open apart from occasional outbuildings.
- An occupancy grid arbitrates space; facing streets share tight corridors
  fairly, and a relaxed **infill pass** wedges hovels/encroachments into
  leftover frontage.
- The market ring gets first claim on frontage so the square reads enclosed.
- Wealth/industry/urbanity are **continuous fields** (market + landmark
  proximity, river industry band, noise). Building kind, floors, row chance
  and setbacks all derive from them — no hard rings or angular wedges.
- `AMedievalTownGenerator::PlaceBuildings()` maps the lots onto the existing
  `FBuildingLot` / `BuildParcelBoundary()` contract, so the PCG asset nodes
  are unaffected.

## Standalone harness
`Harness/build.sh [seed]` compiles the actual generator sources against a
small UE-type shim and renders a top-down PNG — layout changes can be
iterated and visually verified without launching the editor. See
`Harness/README.md`.

## Key tunables
- `FOrganicStreetConfig::BlockSpacingCore / BlockSpacingEdge`
  (junction spacing target, default 2700/4600 cm)
- `MergeDistance` (trunk Y-merge range, default `TownRadius * 0.095`)
- `PlazaRadius` (market square ring, default `clamp(TownRadius * 0.105)`)
- `DeadEndChanceSecondary/Lane/Alley` (0.28 / 0.55 / 0.80)
- `StepHeadingNoise` (street curvature, radians/step)
- `FOrganicParcelConfig::FrontageMin/Max`, `PlotDepth*`,
  `RowHouseChanceCore/Edge`, `SetbackCore/Edge`, `RearOutbuildingChance`
- `TargetBuildingCount` caps lots at `1.5x` its value.

## Width preset (unchanged)
- `PrimaryWidthRange`: **600..1000 cm**
- `SecondaryWidthRange`: **400..700 cm**
- `LaneWidthRange`: **200..400 cm**
- `AlleyWidthRange`: **150..250 cm**
- `RiverCrossings`: **1-2** depending on town bounds

## Validation checklist
- Generate multiple seeds via `Harness/build.sh <seed>` and inspect:
  - trunk roads merge before the market (no radial star),
  - market square enclosed by large houses,
  - irregular closed blocks with built edges and open yards,
  - dead-end lanes/alleys present in the core,
  - bridges only on actual water spans, quay frontage built.
- In-engine: `bDebugRoadGraph` for tier rendering,
  `bDebugRoadWaterZones` for water corridors and bridge highlights.
