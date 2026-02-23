# Organic Medieval Street Network Integration (UE5.3+)

## Files modified
- `MedievalTownGeneratorRoads.cpp` (deterministic 4-stage street growth + terrain/river constraints + validation)
- `MedievalTownGenerator.cpp` (road ribbon UV/width improvements + irregular junction/plaza mesh caps)
- `MedievalTownGenerator.h` (debug + road morphology controls and surface tags)
- `OrganicStreetGraph.h/.cpp` (explicit graph + nearest queries + intersection tests + approximate block extraction)
- `OrganicTerrainRouting.h/.cpp` (A* terrain routing, RDP simplification, smoothing)
- `OrganicIntersectionMesher.h/.cpp` (intersection polygon and ribbon helpers)
- `OrganicStreetNetworkSettings.h/.cpp` (PCG-facing settings and preview graph builder)
- `MedievalTownGeneratorPCG.Build.cs` (PCG + Geometry dependencies)

## Integration steps
1. Place `AMedievalTownGenerator` in the level (existing actor entrypoint).
2. Enable and tune road controls:
   - `bUseOrganicStreetGrowth = true`
   - `MaxGradePrimary = 0.10`
   - `MaxGradeSecondary = 0.12`
   - `IntersectionMinSpacing = 250` (core equivalent, cm-scale project units)
   - `SecondaryLoopChance = 0.10`
   - `WidthNoiseAmplitude = 0.15`
   - `RoadOrganicWaver = 18`
3. If river is enabled, keep `bGenerateRiver=true` before road generation phase.
4. Pipeline order in generation remains:
   - Macro anchors/walls → River → Streets → Parcels → Buildings → Dressing.
5. Optional PCG scaffolding:
   - Use `UOrganicStreetNetworkSettings` as configuration source.
   - Call `BuildPreviewGraph()` for graph preview data in tools/debug contexts.

## Default medieval AAA tuning preset
- `MaxGradePrimary`: **0.10**
- `MaxGradeSecondary`: **0.12**
- `IntersectionMinSpacingCore`: **2500 cm** (25m)
- `IntersectionMinSpacingOutskirts`: **4000 cm** (40m)
- `PrimaryWidthRange`: **600..1000 cm**
- `SecondaryWidthRange`: **400..700 cm**
- `LaneWidthRange`: **200..400 cm**
- `AlleyWidthRange`: **150..250 cm**
- `PlazaChanceAt3WayCore`: **0.35**
- `LoopChanceSecondaryCore`: **0.10**
- `LoopChanceSecondaryOutskirts`: **0.03**
- `WidthNoiseAmplitude`: **0.15**
- `RiverCrossings`: **1-2** depending on town bounds

## Validation checklist
- Generate 20 seeds and inspect morphology:
  - no radial symmetry regressions,
  - no illegal water crossings,
  - stable bridge placement/approaches,
  - varied widths/intersections/plazas.
- Enable debug:
  - `bDebugRoadGraph` for role/type rendering,
  - `bDebugRoadWaterZones` for near-water forbidden corridor and bridge highlights.
- Ensure no tiny dangling edges or illegal self-intersections in inserted edges.
