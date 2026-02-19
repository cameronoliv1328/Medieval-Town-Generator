# River System Review and Improvements

## What was wrong

Based on observed behavior and screenshot evidence, the river had two primary issues:

1. **Visible seam / gap at water edge**
   - Water surface width exactly matched nominal carve width, so any slope interpolation, depth offset, or view-angle precision difference could expose a thin gap.
2. **Buildings too close to or over river**
   - Building rejection used only the base river exclusion radius and did not include an additional hard no-build flood margin.

## Applied changes

### 1) More realistic river cross-section

Implemented a shared depth function (`GetRiverDepthAt`) with a profile inspired by common natural channel morphology:

- **Deepest centerline (thalweg)** = `RiverMaxDepth`
- **Shallower near-water-edge shelf** = `RiverEdgeDepth`
- **Smooth bank blend** over `RiverBankFalloffWidth`

This replaced the previous constant-depth interior carve and improves realism in shallow edge transitions.

### 2) Water/riverbed seam reduction

- Added `RiverWaterSurfaceOffset` so water elevation is controlled explicitly relative to local bank height.
- Added `RiverSurfaceEdgeOverlap` and expanded rendered water half-width by this overlap.

This makes the water mesh intentionally overhang slightly beyond the nominal wet channel edge, which hides edge cracks from glancing camera angles.

### 3) Stronger no-build river safety margin

- Added `RiverBuildingBuffer` and applied it to building placement validation (`CanPlaceLot`).

This prevents lots from being accepted near the river corridor even when random placement would otherwise pass slope/spacing checks.

## Tunable parameters added

- `RiverMaxDepth`
- `RiverEdgeDepth`
- `RiverBankFalloffWidth`
- `RiverWaterSurfaceOffset`
- `RiverSurfaceEdgeOverlap`
- `RiverBuildingBuffer`

All are exposed in `UPROPERTY` under **Town | River** for iteration in-editor.

## Review notes

- Road/bridge logic was preserved, but `SegmentCrossesRiver` now uses widened effective river width for crossing tests so bridge tagging stays consistent with visual water extents.
- Terrain carve now delegates to one depth model (`GetRiverDepthAt`) to avoid divergence between systems.

## Recommended in-editor tuning order

1. Set `RiverMaxDepth` / `RiverEdgeDepth` (shape feel)
2. Set `RiverWaterSurfaceOffset` (vertical seating)
3. Increase/decrease `RiverSurfaceEdgeOverlap` until seam artifacts disappear
4. Set `RiverBuildingBuffer` to enforce desired waterfront setback

