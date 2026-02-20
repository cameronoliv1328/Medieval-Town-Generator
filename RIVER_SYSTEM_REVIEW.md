# River System Review (Research-Informed UE5 PCG Pass)

## Research synthesis used for this pass

This pass combines practical Unreal environment workflows with river morphology and historical city-form heuristics:

1. **River morphology / fluvial geometry**
   - Real channels are not constant-width trenches.
   - Outside bends deepen (cut bank), inside bends shallow (point bar).
   - Bed/surface/bank transitions should be smooth and continuous.

2. **Landscape modeling in real-time engines (UE procedural mesh + terrain)**
   - Ribbon water with tangent-averaged cross-sections is more stable than independent segment strips.
   - A separate submerged bed mesh improves depth readability through reflective/transparent water materials.
   - Closing side banks (surface→bed) avoids visible "open trench" artifacts.

3. **City-building patterns around rivers**
   - Settlements historically formed quay-like movement corridors parallel to water.
   - Urban river edges are often managed terraces/embankments rather than raw erosion forms.
   - Frequent pedestrian/service links from riverside path to core street graph improve integration.

## Major upgrades implemented

### A) Smooth river centerline and flow continuity
- Harmonic meander generation for waypoints.
- Catmull-Rom interpolation for centerline smoothness.
- Downstream cumulative drop so flow has coherent directional fall.

### B) Local hydraulic variation
- Width variation along alpha (`GetRiverHalfWidthAt`).
- Flow variation along alpha (`GetRiverFlowSpeedAt`).
- Closest-point query helper (`SampleRiverClosestPoint`) used by depth and proximity logic.

### C) Improved depth profile
- Center-to-edge depth blend.
- Curvature-biased outside-bend deepening.
- Edge noise modulation.
- Smooth bank falloff beyond wetted edge.

### D) Detailed surface + bed + bank geometry
- **River surface ribbon**: variable width, tangent-averaged alignment, cumulative flow-scaled UVs.
- **River bed ribbon**: narrower than surface, local depth offset with minimum below-surface guarantee.
- **Bank side closure mesh**: explicit side faces connecting bed to surface edges for robust visuals.

### E) City integration improvements

### F) Terrain built around the same river spline (single source of truth)
- Introduced a cached planar river spline (`CachedRiverPlanarPath`) built right after waypoint generation.
- Terrain carving/proximity now query this same spline via `SampleRiverClosestPoint()`.
- Water surface world path is derived from the same planar spline.

This removes river/terrain desync where terrain used one path approximation and water mesh used another.

- Added **riverfront path generation** in road graph construction:
  - bank-parallel path chains where river crosses urban footprint,
  - periodic connectors to nearest street nodes,
  - `EStreetTier::RiverPath` sizing support.
- Added a subtle **urban river terrace lift** in terrain close to riverbanks inside town radius,
  improving city-river transition and helping lots/roads read as intentionally managed edges.
- Added **floodplain flattening** around the river corridor so nearby terrain is smoother and more cohesive with the channel.
- Added an **outer-wall gorge profile** (core depression + rim shoulders) so river terrain outside
  the city reads as a real valley/canyon instead of a flat cut.

## New/important tuning controls

- `RiverGorgeDepth`
- `RiverGorgeHalfWidth`
- `RiverGorgeRimWidth`
- `RiverFloodplainWidth`
- `RiverFloodplainFlattenStrength`
- `RiverFloodplainDrop`
- `RiverSamplesPerSegment`
- `RiverVariationFrequency`
- `RiverWidthVariation`
- `RiverDownhillPerSegment`
- `RiverEdgeNoise`
- `RiverUVTilingDistance`
- `RiverBedWidthFactor`
- `RiverBedMinBelowSurface`
- `RiverFlowSpeedBase`
- `RiverFlowSpeedVariation`
- `RiverSurfaceEdgeOverlap`
- `RiverBuildingBuffer`

## Suggested tuning workflow

1. **Overall route**: `RiverWaypoints`, `RiverEntryAngleDeg`, `RiverSamplesPerSegment` (plus outside-city continuation points).
2. **Hydraulic shape**: `RiverWidth`, `RiverWidthVariation`, `RiverMaxDepth`, `RiverEdgeDepth`.
3. **Flow character**: `RiverDownhillPerSegment`, `RiverFlowSpeedBase`, `RiverFlowSpeedVariation`.
4. **Visual quality**: `RiverSurfaceEdgeOverlap`, `RiverUVTilingDistance`, bed factors.
5. **City fit**: `RiverBuildingBuffer`, `RiverExclusionRadius`, terrace effect (terrain behavior).

## Next opportunities

- Spawn quays/docks or retaining walls where riverfront path density is highest.
- Add localized wetness/sediment decals via runtime virtual texture or spline decals.
- Add bridge abutment geometry that keys off nearby riverbank normals.
- Optional WaterBodyRiver actor export for projects using UE Water plugin runtime features.
