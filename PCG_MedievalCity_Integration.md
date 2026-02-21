# Medieval PCG City Pipeline (UE5.3+)

## 1) Node order (strict dependency)
1. `PCG_MedievalCityMacro` (`UMedievalCityMacroSettings`)
   - Output pin: `Anchors`
2. `PCG_MedievalStreetNetwork` (`UMedievalStreetNetworkSettings`)
   - Input: `Anchors`
   - Output: `Streets`
3. `PCG_MedievalParcels` (`UMedievalParcelsSettings`)
   - Input: `Streets`
   - Output: `Parcels`
4. `PCG_MedievalBuildings` (`UMedievalBuildingsSettings`)
   - Input: `Parcels`
   - Output: `Buildings`

All downstream foliage/props/decals should consume `Buildings` and `Parcels` attributes.

## 2) Typical medieval defaults (market-centered)
Use `UMedievalCityGeneratorProfile` and start with:
- Seed: `1337`
- Bounds: `(-12000,-12000) -> (12000,12000)`
- TownReason: `RiverCrossing`
- WalledTown: `true`
- WallRadius: `4800`
- GateCount: `4`
- Primary/Secondary/Alley widths: `700 / 440 / 230`
- Burgage frontage/depth: `400-1000 / 3000-12000`
- Back-lane threshold: `6200`
- Core floors: `2-4`, outskirts: `1-2`

## 3) Attribute contract for spawners
### Streets output
- `StreetType` (string enum)
- `StreetWidth` (float)
- `Importance` (float)

### Parcels output
- `ParcelPolygon` (string, `x,y;x,y;...`)
- `District` (string enum)
- `Frontage` (float)
- `Depth` (float)
- `Wealth` (float 0..1)
- `MixedUse` (float 0..1)
- `HasBackLane` (bool)

### Buildings output
- `FootprintType` (string enum)
- `Floors` (int)
- `RoofType` (string)
- `PartyWall` (bool)
- `HasRearExtension` (bool)
- `FacadeTag` (string)

## 4) Debug visualization recommendation
- Use `PCG Debug` node after each stage.
- Streets: color by `StreetType`; scale by `StreetWidth`.
- Parcels: color by `District`, label with `Frontage/Depth`.
- Buildings: color by `FootprintType`, print `Floors`.

## 5) Determinism
Each stage seeds `FRandomStream` from `SeedParams.Seed` plus fixed offsets:
- Macro +0
- Streets +101
- Parcels +203
- Buildings +307

This keeps repeatable generation across runs when input bounds/profile are unchanged.

## 6) Failure handling
- Parcels reject non-simple polygons and automatically fallback to non-jittered shapes.
- Street intersections are spacing constrained to avoid over-connected grids.
- Outbuilding generation is optional and skipped when depth is too small.
