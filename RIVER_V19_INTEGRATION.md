# River System V19 — Integration Guide

## Problem Summary
- **Gaps** between river mesh edges and terrain mesh
- **Terrain interrupting** river flow because terrain mesh resolution is too coarse to represent the carved channel
- **Shore blending** is too simple (single quad strip) causing visible seams

## Root Cause Analysis

The V18 system has three independent gap sources:

1. **Terrain mesh resolution**: With `TerrainResolution=60` and `TownRadius=18000`, the terrain cell size is ~660 units. The river channel (width ~550 units) often falls *between* terrain vertices. The terrain mesh interpolates linearly across the channel, creating a ridge where the river should be carved.

2. **Shore mesh height mismatch**: The V18 shore blend mesh samples `GetTerrainHeight` at positions that don't exactly match terrain mesh vertices. Small numerical differences create visible cracks.

3. **Single-strip shore**: One quad strip between water edge and terrain is too abrupt for a natural bank transition.

## Solution Architecture

### Fix 1: Adaptive terrain mesh density
`GenerateAdaptiveTerrainMesh()` uses a uniform grid with cell size controlled by `RiverAdaptiveTerrainCellSize` (default 100 units). Near the river corridor, terrain vertices are dense enough to represent the channel carve. The hard cap at 512×512 prevents memory blowout.

### Fix 2: Shared height function for zero-gap shore
`MTGRiver::GenerateShoreBlend()` calls `GetTerrainHeight()` at the **exact** XY position of each shore vertex. Since the terrain mesh also calls `GetTerrainHeight()` for its vertices, both meshes agree on height at any shared position. The water surface overlaps 25 units past the shore inner edge to hide sub-pixel precision issues.

### Fix 3: Multi-ring shore blend
4 concentric rings per bank (configurable via `RiverShoreBlendRings`) create a gradual transition from wet water edge to dry terrain. Vertex colors encode wetness for material-level darkening.

### Fix 4: U-shaped riverbed
5 vertices per cross-section row create a realistic submerged channel profile instead of a flat ribbon.

### Fix 5: Flow data in vertex attributes
Tangent vectors encode flow direction. Vertex colors encode flow speed, foam mask, and depth. Materials can use these for animated normals, depth-based color, and foam effects.

---

## File Changes

### New files to add:

| File | Purpose |
|------|---------|
| `MedievalTownGeneratorRiver.h` | River mesh generation namespace header |
| `MedievalTownGeneratorRiver.cpp` | River mesh generation implementation |

### Files to modify:

| File | Changes |
|------|---------|
| `MedievalTownGenerator.h` | Add new UPROPERTYs + method declarations |
| `MedievalTownGenerator.cpp` | Modify Phase2 + Phase7 + add SpawnImprovedRiverMeshes |
| `MedievalTownGeneratorTerrain.cpp` | Add GenerateAdaptiveTerrainMesh() |

---

## Step-by-Step Integration

### Step 1: Add new source files

Copy `MedievalTownGeneratorRiver.h` and `MedievalTownGeneratorRiver.cpp` into your Source folder next to the existing files.

### Step 2: Modify MedievalTownGenerator.h

**Add include** at top of file:
```cpp
#include "MedievalTownGeneratorRiver.h"
```

**Add properties** after existing River properties (around line where `RiverFoamHeightOffset` is):
```cpp
    // V19: Adaptive terrain cell size near river (smaller = denser mesh near river)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "30", UIMax = "500"))
    float RiverAdaptiveTerrainCellSize = 100.f;

    // V19: Number of shore blend rings per bank (more = smoother transition)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "2", UIMax = "8"))
    int32 RiverShoreBlendRings = 4;

    // V19: Enable improved gap-free river mesh pipeline
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bUseImprovedRiverMeshes = true;

    // V19: Extra submersion offset under water surface to prevent z-fighting
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "1", UIMax = "30"))
    float RiverTerrainSubmersionBias = 5.f;
```

**Add private method declarations** after existing river methods:
```cpp
    // ─── River V19 ───────────────────────────────────────────────────────
    void SpawnImprovedRiverMeshes();
    void GenerateAdaptiveTerrainMesh();
```

**Add friend declaration** (so MTGRiver namespace can access member data):
```cpp
    // In the public: section or just before the private: section:
    friend void MTGRiver::BuildCrossSectionSamples(
        AMedievalTownGenerator*,
        TArray<MTGRiver::FRiverCrossSectionSample>&);
```

### Step 3: Modify Phase2_SetupTerrain() in MedievalTownGenerator.cpp

Replace the existing `Phase2_SetupTerrain()` body with the version from `MedievalTownGenerator_RiverUpgrade.cpp`. This routes to `GenerateAdaptiveTerrainMesh()` when `bUseImprovedRiverMeshes` is true, and keeps the V18 code as fallback.

### Step 4: Modify Phase7_SpawnMeshes() river section

In `Phase7_SpawnMeshes()`, find the river mesh block:
```cpp
    // ── River ─────────────────────────────────────────────────────────────
    if (bGenerateRiver && CachedRiverWorldPath.Num() >= 2)
    {
        // ... ~100 lines of V18 river mesh code ...
    }
```

Replace it with:
```cpp
    // ── River ─────────────────────────────────────────────────────────────
    if (bGenerateRiver && CachedRiverWorldPath.Num() >= 2)
    {
        if (bUseImprovedRiverMeshes)
        {
            SpawnImprovedRiverMeshes();
        }
        else
        {
            // V18 fallback — keep original code here
            // (paste the original V18 river mesh code block)
        }
    }
```

### Step 5: Add SpawnImprovedRiverMeshes()

Add the `SpawnImprovedRiverMeshes()` function from `MedievalTownGenerator_RiverUpgrade.cpp` to your `MedievalTownGenerator.cpp`.

### Step 6: Add GenerateAdaptiveTerrainMesh()

Add the `GenerateAdaptiveTerrainMesh()` function from `MedievalTownGeneratorTerrain.cpp` (the new version) to your `MedievalTownGeneratorTerrain.cpp`.

### Step 7: Build.cs

No changes needed — you're already using `ProceduralMeshComponent`. Just ensure the new `.cpp` files are in your Source folder so UBT picks them up.

### Step 8: Set up materials

See `M_RiverWater_Setup.txt` for material creation instructions. At minimum:
- Your existing `WaterMaterial` should be translucent
- Your existing `GroundMaterial` is used for shore blend (works as-is)
- Optionally create a foam material with additive blending

---

## Tuning Guide

### If you still see gaps:
1. **Reduce** `RiverAdaptiveTerrainCellSize` (try 50-80). This creates more terrain vertices near the river.
2. **Increase** `RiverSurfaceEdgeOverlap` (try 40-60). This extends the water surface mesh further past the bank edge.
3. **Increase** `RiverShoreBlendWidth` (try 200-300). This makes the shore strips wider, covering more terrain.

### If performance is too heavy:
1. **Increase** `RiverAdaptiveTerrainCellSize` to 200-300.
2. **Reduce** `RiverShoreBlendRings` to 2-3.
3. **Increase** `RiverSamplesPerSegment` only if needed (fewer samples = fewer river vertices).

### If the river channel isn't deep enough in the terrain mesh:
1. The terrain mesh resolution must be fine enough to capture the channel. Reduce `RiverAdaptiveTerrainCellSize`.
2. Check that `RiverMaxDepth` is larger than `RiverWaterSurfaceOffset`.

### If the river looks flat/boring:
1. Increase `RiverWidthVariation` (0.25-0.35) for more width oscillation.
2. Increase `RiverVariationFrequency` for more meanders.
3. Set up the water material with two-layer flowing normals per the material guide.
4. Enable foam (`bGenerateRiverFoam = true`).

---

## Pipeline Order (unchanged)

```
Phase1: GenerateRiverWaypoints    → River planar path (2D spline)
Phase2: SetupTerrain              → Adaptive terrain mesh (V19: higher density near river)
Phase3: BuildRiverWorldPath       → River 3D path (Z from terrain)
Phase4: BuildWalls                → Wall perimeter + gates
Phase5: BuildRoadNetwork          → Roads (avoid river, bridges where crossing)
Phase6: PlaceBuildings            → Buildings (avoid river exclusion zone)
Phase7: SpawnMeshes               → All meshes including V19 river pipeline
Phase8: PlaceForest               → Trees (avoid river corridor)
Phase9: BuildMountains            → Mountain peaks
```

The river runs first (Phase1), terrain adapts to it (Phase2), and all downstream phases read the river exclusion data. This ordering is unchanged from V18.

---

## Acceptance Test Checklist

- [ ] No visible gaps between river edge and terrain at any camera angle
- [ ] River channel is properly carved into terrain mesh (visible depression)
- [ ] Shore strips blend smoothly from wet water edge to dry terrain
- [ ] River continues beyond town walls without interruption
- [ ] Water surface overlaps shore inner edge (no sub-pixel cracks)
- [ ] Regenerating with different seeds produces different but gap-free rivers
- [ ] `bUseImprovedRiverMeshes = false` reverts to V18 behavior
- [ ] Terrain mesh vertex count stays under 512×512 cap
- [ ] Roads, buildings, and foliage still avoid river corridor correctly
