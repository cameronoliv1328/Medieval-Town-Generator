// Buildings module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorBuildings.h"
#include "MedievalTownGenerator.h"

float MTGBuildings::ComputeRoofHeight(float Width, float Depth, float PitchAngleDeg)
{
    const float HalfSpan = FMath::Min(Width, Depth) * 0.5f;
    return FMath::Tan(FMath::DegreesToRadians(PitchAngleDeg)) * HalfSpan;
}


// ─────────────────────────────────────────────────────────────────────────────
//  §7  BUILDING PLACEMENT  (district-aware + flat-area filtered)
// ─────────────────────────────────────────────────────────────────────────────

TArray<FDistrictDef> AMedievalTownGenerator::BuildDistrictDefs() const
{
    TArray<FDistrictDef> Defs;

    // ─────────────────────────────────────────────────────────────────────
    //  Compute feature angles for angular sector assignment
    //
    //  Medieval towns weren't organized in perfect concentric rings — they
    //  had angular sectors driven by geography:
    //    • Craft/industrial quarters along the river
    //    • Gate wards at each entrance (bustling commercial zones)
    //    • Administrative/military core at center
    //    • Slums on the least desirable side (downwind, away from gates)
    //    • Merchant quarters filling the best remaining sectors
    // ─────────────────────────────────────────────────────────────────────

    // River direction angle (river flows roughly along this bearing)
    float RiverAngle = 0.f;
    if (River.Waypoints.Num() >= 2)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        RiverAngle = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X));
    }

    // Gate angles
    TArray<float> GateAngles;
    for (const FRoadNode& N : RoadNodes)
    {
        if (N.bIsGate)
        {
            GateAngles.Add(FMath::RadiansToDegrees(FMath::Atan2(N.Pos.Y, N.Pos.X)));
        }
    }

    // Slums angle: opposite side from the densest gate cluster
    // (historically the least desirable, downwind side)
    float SlumsAngle = RiverAngle + 180.f;
    if (SlumsAngle > 180.f) SlumsAngle -= 360.f;

    // ── 1. Inner Ward (castle/admin core) ────────────────────────────────
    //    Large buildings, wide spacing, formal layout (low jitter).
    //    Military/administrative center with keep, church, guildhall.
    {
        FDistrictDef D;
        D.Name = TEXT("Inner Ward");
        D.Type = EDistrictType::InnerWard;
        D.InnerRadiusFraction = 0.f;
        D.OuterRadiusFraction = InnerRingRadius;
        D.Density = 0.52f;
        D.MinScale = 1.2f; D.MaxScale = 1.7f;
        D.StylePool = { EBuildingStyle::Keep, EBuildingStyle::Keep,
                        EBuildingStyle::GuildHall, EBuildingStyle::Church,
                        EBuildingStyle::TavernInn };
        D.SpacingMult = 1.4f;       // Wide spacing — prestigious buildings
        D.SetbackMult = 1.3f;       // Larger setback from road
        D.RotationJitter = 3.f;     // Very formal, aligned
        D.LShapeChance = 0.4f;      // Many L-shaped compounds
        D.CursorStartOffset = 300.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 2. Craft Quarter (river-side industrial) ─────────────────────────
    //    Warehouses, blacksmiths, yards near the river.
    //    Medium density, bigger buildings, oriented toward water.
    //    Angular wedge centered on river direction ±45°.
    {
        FDistrictDef D;
        D.Name = TEXT("Craft Quarter");
        D.Type = EDistrictType::CraftQuarter;
        D.InnerRadiusFraction = InnerRingRadius;
        D.OuterRadiusFraction = 0.92f;
        D.Density = 0.55f;
        D.MinScale = 0.9f; D.MaxScale = 1.4f;
        D.StylePool = { EBuildingStyle::Warehouse, EBuildingStyle::Warehouse,
                        EBuildingStyle::Blacksmith, EBuildingStyle::Blacksmith,
                        EBuildingStyle::Stable, EBuildingStyle::TownHouse };
        D.SpacingMult = 1.1f;       // Moderate spacing — work yards between buildings
        D.SetbackMult = 1.5f;       // Bigger setback — loading areas along road
        D.RotationJitter = 6.f;     // Somewhat organized
        D.LShapeChance = 0.35f;     // Workshops often have wings
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 400.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = RiverAngle - 45.f;
        D.MaxAngleDeg = RiverAngle + 45.f;
        Defs.Add(D);
    }

    // ── 3. Gate Wards (bustling commercial at each gate) ─────────────────
    //    Dense shops, taverns, cheap lodging near the town gates.
    //    Tight spacing, small setback, high density.
    //    Each gate gets a ±20° angular wedge in the outer ring area.
    for (int32 G = 0; G < GateAngles.Num(); G++)
    {
        FDistrictDef D;
        D.Name = FString::Printf(TEXT("Gate Ward %d"), G + 1);
        D.Type = EDistrictType::GateWard;
        D.InnerRadiusFraction = OuterRingRadius * 0.85f;
        D.OuterRadiusFraction = 0.94f;
        D.Density = 0.82f;
        D.MinScale = 0.7f; D.MaxScale = 0.95f;
        D.StylePool = { EBuildingStyle::TownHouse, EBuildingStyle::TownHouse,
                        EBuildingStyle::TavernInn, EBuildingStyle::TavernInn,
                        EBuildingStyle::Bakery, EBuildingStyle::SmallCottage,
                        EBuildingStyle::TownHouse };
        D.SpacingMult = 0.55f;      // Very tight — cramped commercial zone
        D.SetbackMult = 0.6f;       // Buildings crowd the road
        D.RotationJitter = 5.f;     // Somewhat organized along streets
        D.LShapeChance = 0.1f;      // Small plots, few wings
        D.CursorStartOffset = 150.f;
        D.MinEdgeLen = 350.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = GateAngles[G] - 20.f;
        D.MaxAngleDeg = GateAngles[G] + 20.f;
        Defs.Add(D);
    }

    // ── 4. Merchant Quarter (main commercial/residential belt) ───────────
    //    Townhouses, shops, taverns between the rings.
    //    Medium-high density, good spacing, neat orientation.
    {
        FDistrictDef D;
        D.Name = TEXT("Merchant Quarter");
        D.Type = EDistrictType::MerchantQuarter;
        D.InnerRadiusFraction = InnerRingRadius;
        D.OuterRadiusFraction = OuterRingRadius;
        D.Density = 0.75f;
        D.MinScale = 0.85f; D.MaxScale = 1.2f;
        D.StylePool = { EBuildingStyle::TownHouse, EBuildingStyle::TownHouse,
                        EBuildingStyle::TownHouse, EBuildingStyle::TavernInn,
                        EBuildingStyle::GuildHall, EBuildingStyle::Blacksmith,
                        EBuildingStyle::Bakery };
        D.SpacingMult = 0.8f;       // Fairly dense
        D.SetbackMult = 0.85f;      // Close to road — shopfronts
        D.RotationJitter = 5.f;     // Neat commercial streets
        D.LShapeChance = 0.2f;
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 5. Outer Residential (between outer ring and walls) ──────────────
    //    Modest homes, some craftsmen.
    {
        FDistrictDef D;
        D.Name = TEXT("Outer Residential");
        D.Type = EDistrictType::OuterResidential;
        D.InnerRadiusFraction = OuterRingRadius;
        D.OuterRadiusFraction = 0.88f;
        D.Density = 0.65f;
        D.MinScale = 0.75f; D.MaxScale = 1.0f;
        D.StylePool = { EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::TownHouse,
                        EBuildingStyle::Blacksmith };
        D.SpacingMult = 0.9f;
        D.SetbackMult = 1.0f;
        D.RotationJitter = 10.f;    // More organic than center
        D.LShapeChance = 0.15f;
        D.CursorStartOffset = 220.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 6. Slums (poor quarter — opposite river, least desirable) ────────
    //    Tiny, densely packed cottages with chaotic orientation.
    //    Angular wedge opposite the river ±50°.
    {
        FDistrictDef D;
        D.Name = TEXT("Slums");
        D.Type = EDistrictType::Slums;
        D.InnerRadiusFraction = OuterRingRadius * 0.7f;
        D.OuterRadiusFraction = 0.92f;
        D.Density = 0.88f;
        D.MinScale = 0.55f; D.MaxScale = 0.8f;
        D.StylePool = { EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::Stable };
        D.SpacingMult = 0.4f;       // Extremely tight — shanty-town density
        D.SetbackMult = 0.4f;       // Buildings crowd right up to paths
        D.RotationJitter = 25.f;    // Chaotic, unplanned feel
        D.LShapeChance = 0.05f;     // Too poor for extensions
        D.CursorStartOffset = 120.f;
        D.MinEdgeLen = 250.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = SlumsAngle - 50.f;
        D.MaxAngleDeg = SlumsAngle + 50.f;
        Defs.Add(D);
    }

    // ── 7. Transition Zone (stables/storage near walls) ──────────────────
    //    Wide spacing, low density, functional buildings near the walls.
    {
        FDistrictDef D;
        D.Name = TEXT("Transition Zone");
        D.Type = EDistrictType::TransitionZone;
        D.InnerRadiusFraction = 0.88f;
        D.OuterRadiusFraction = 0.94f;
        D.Density = 0.35f;
        D.MinScale = 0.7f; D.MaxScale = 1.0f;
        D.StylePool = { EBuildingStyle::Stable, EBuildingStyle::Stable,
                        EBuildingStyle::Warehouse, EBuildingStyle::SmallCottage };
        D.SpacingMult = 1.3f;       // Wide spacing — yards, pens
        D.SetbackMult = 1.2f;
        D.RotationJitter = 12.f;
        D.LShapeChance = 0.1f;
        D.CursorStartOffset = 250.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    return Defs;
}

// ── Angular-aware district lookup ───────────────────────────────────────
//
//  Checks position against all district defs, prioritizing angular wedge
//  districts (CraftQuarter, GateWard, Slums) which override the radius-only
//  fallback districts (MerchantQuarter, OuterResidential, etc).
//
//  Priority: Angular wedge match > Radius-only match > Fallback
//

static float NormalizeAngle180(float Deg)
{
    while (Deg > 180.f)  Deg -= 360.f;
    while (Deg < -180.f) Deg += 360.f;
    return Deg;
}

static bool AngleInSector(float TestDeg, float MinDeg, float MaxDeg)
{
    // Normalize all angles to -180..180
    TestDeg = NormalizeAngle180(TestDeg);
    MinDeg  = NormalizeAngle180(MinDeg);
    MaxDeg  = NormalizeAngle180(MaxDeg);

    if (MinDeg <= MaxDeg)
        return TestDeg >= MinDeg && TestDeg <= MaxDeg;
    else
        // Wraps around -180/180 boundary
        return TestDeg >= MinDeg || TestDeg <= MaxDeg;
}

EDistrictType AMedievalTownGenerator::GetDistrictAt(FVector2D Pos) const
{
    float Frac = Pos.Size() / TownRadius;
    float Angle = FMath::RadiansToDegrees(FMath::Atan2(Pos.Y, Pos.X));

    // ── Center always = InnerWard ──
    if (Frac < InnerRingRadius * 0.7f)
        return EDistrictType::InnerWard;

    // ── Near walls = TransitionZone ──
    if (Frac > 0.88f)
        return EDistrictType::TransitionZone;

    // ── Check angular wedge districts (highest priority in the mid-ring) ──
    //    These are cached in CachedDistrictDefs during PlaceBuildings.
    //    For runtime queries, re-derive from features.

    // Craft Quarter: near river
    if (River.Waypoints.Num() >= 2 && Frac > InnerRingRadius)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        float RiverAng = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X));
        if (AngleInSector(Angle, RiverAng - 45.f, RiverAng + 45.f))
            return EDistrictType::CraftQuarter;
    }

    // Gate Wards: near gates in outer ring
    if (Frac > OuterRingRadius * 0.85f)
    {
        for (const FRoadNode& N : RoadNodes)
        {
            if (!N.bIsGate) continue;
            float GA = FMath::RadiansToDegrees(FMath::Atan2(N.Pos.Y, N.Pos.X));
            if (AngleInSector(Angle, GA - 20.f, GA + 20.f))
                return EDistrictType::GateWard;
        }
    }

    // Slums: opposite river direction
    if (Frac > OuterRingRadius * 0.7f && River.Waypoints.Num() >= 2)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        float SlumsAng = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X)) + 180.f;
        if (AngleInSector(Angle, SlumsAng - 50.f, SlumsAng + 50.f))
            return EDistrictType::Slums;
    }

    // ── Radius-only fallback ──
    if (Frac < OuterRingRadius)
        return EDistrictType::MerchantQuarter;

    return EDistrictType::OuterResidential;
}

EBuildingStyle AMedievalTownGenerator::PickStyle(EDistrictType District,
                                                  const FDistrictDef& Def)
{
    if (Def.StylePool.Num() == 0) return EBuildingStyle::TownHouse;
    return Def.StylePool[Rand.RandRange(0, Def.StylePool.Num() - 1)];
}

ERoofType AMedievalTownGenerator::PickRoof(EBuildingStyle Style)
{
    switch (Style)
    {
    case EBuildingStyle::SmallCottage: return ERoofType::Thatched;
    case EBuildingStyle::TownHouse:
        return (Rand.FRand() < 0.6f) ? ERoofType::Pitched : ERoofType::Gambrel;
    case EBuildingStyle::GuildHall:
        return (Rand.FRand() < 0.5f) ? ERoofType::Hipped : ERoofType::FlatParapet;
    case EBuildingStyle::TavernInn:
        return (Rand.FRand() < 0.7f) ? ERoofType::Gambrel : ERoofType::Pitched;
    case EBuildingStyle::Church:
        return ERoofType::Pitched;
    case EBuildingStyle::Keep:
        return ERoofType::FlatParapet;
    case EBuildingStyle::Stable:
        return (Rand.FRand() < 0.5f) ? ERoofType::Gambrel : ERoofType::Thatched;
    case EBuildingStyle::Warehouse:
        return ERoofType::FlatParapet;
    default:
        return ERoofType::Pitched;
    }
}

int32 AMedievalTownGenerator::PickFloorCount(EBuildingStyle Style, EDistrictType District)
{
    // District modifies base floor counts:
    //   InnerWard:     Tall, prestigious buildings (keeps 3-4, guilds 2-3)
    //   MerchantQtr:   2-story townhouses, tall guildhalls
    //   CraftQuarter:  Single-story workshops, 2-story warehouses
    //   GateWard:      Cramped 2-3 story buildings (vertical, narrow plots)
    //   Slums:         Always 1 story (too poor for multi-story)
    //   Outer/Trans:   Mostly 1 story

    switch (Style)
    {
    case EBuildingStyle::SmallCottage:
        return 1;

    case EBuildingStyle::TownHouse:
        if (District == EDistrictType::GateWard)       return Rand.RandRange(2, 3);
        if (District == EDistrictType::MerchantQuarter) return 2;
        if (District == EDistrictType::Slums)           return 1;
        if (District == EDistrictType::InnerWard)       return 2;
        return (Rand.FRand() < 0.3f) ? 2 : 1;

    case EBuildingStyle::GuildHall:
        if (District == EDistrictType::InnerWard) return Rand.RandRange(2, 3);
        return 2;

    case EBuildingStyle::TavernInn:
        if (District == EDistrictType::GateWard) return Rand.RandRange(2, 3);
        return 2;

    case EBuildingStyle::Church:       return 1;

    case EBuildingStyle::Keep:
        return Rand.RandRange(3, 4);

    case EBuildingStyle::Stable:       return 1;

    case EBuildingStyle::Warehouse:
        if (District == EDistrictType::CraftQuarter) return Rand.RandRange(1, 2);
        return 1;

    case EBuildingStyle::Blacksmith:
        if (District == EDistrictType::CraftQuarter) return 1;
        return 1;

    case EBuildingStyle::Bakery:       return 1;
    default:                           return 1;
    }
}

FVector2D AMedievalTownGenerator::BuildingSize(EBuildingStyle Style) const
{
    switch (Style)
    {
    case EBuildingStyle::SmallCottage: return SmallCottageSize;
    case EBuildingStyle::TownHouse:    return TownHouseSize;
    case EBuildingStyle::GuildHall:    return GuildHallSize;
    case EBuildingStyle::TavernInn:    return TavernSize;
    case EBuildingStyle::Church:       return ChurchSize;
    case EBuildingStyle::Keep:         return KeepSize;
    case EBuildingStyle::Stable:       return StableSize;
    case EBuildingStyle::Warehouse:    return WarehouseSize;
    case EBuildingStyle::Blacksmith:   return SmallCottageSize * 1.1f;
    case EBuildingStyle::Bakery:       return SmallCottageSize;
    default:                           return TownHouseSize;
    }
}

bool AMedievalTownGenerator::CanPlaceLot(FVector Center, float Radius, int32 IgnoreEdgeIndex, float SpacingOverride) const
{
    FVector2D Pos2D(Center.X, Center.Y);

    // Must be inside town walls (with margin)
    if (Pos2D.Size() > TownRadius * 0.89f - Radius) return false;

    // Must not be near river
    if (IsNearRiver(Pos2D, Radius + RiverBuildingBuffer)) return false;

    // Must be on flat terrain
    FTerrainSample TS = SampleTerrain(Pos2D.X, Pos2D.Y);
    if (!TS.bIsFlat) return false;

    // Four-corner flat check (flat area detector)
    if (!IsTerrainFlat(Pos2D, Radius * 0.7f, Radius * 0.7f)) return false;

    // Must not overlap other buildings (using per-district spacing if provided)
    float Spacing = (SpacingOverride >= 0.f) ? SpacingOverride : MinBuildingSpacing;
    for (const FBuildingLot& Lot : PlacedLots)
    {
        if (!Lot.bIsPlaced) continue;
        float Dist = Dist2D(Center, Lot.Center);
        if (Dist < Radius + Lot.CollisionRadius + Spacing)
            return false;
    }

    // Must not be on a road (skip the road we're lining via IgnoreEdgeIndex)
    // Use actual spline polyline segments, not just NodeA->NodeB chord.
    const FVector ActorLoc = GetActorLocation();
    for (int32 i = 0; i < RoadEdges.Num(); i++)
    {
        if (i == IgnoreEdgeIndex) continue;
        const FRoadEdge& Edge = RoadEdges[i];
        if (!Edge.bIsGenerated) continue;

        const float RoadExclusion = Edge.Width * 0.5f + Radius + 100.f;

        if (Edge.WorldPoints.Num() >= 2)
        {
            for (int32 w = 0; w < Edge.WorldPoints.Num() - 1; w++)
            {
                const FVector2D A(Edge.WorldPoints[w].X - ActorLoc.X,
                                  Edge.WorldPoints[w].Y - ActorLoc.Y);
                const FVector2D B(Edge.WorldPoints[w + 1].X - ActorLoc.X,
                                  Edge.WorldPoints[w + 1].Y - ActorLoc.Y);
                if (CircleOverlapsSegment(Pos2D, RoadExclusion, A, B)) return false;
            }
        }
        else
        {
            const FVector2D A = RoadNodes[Edge.NodeA].Pos;
            const FVector2D B = RoadNodes[Edge.NodeB].Pos;
            if (CircleOverlapsSegment(Pos2D, RoadExclusion, A, B)) return false;
        }
    }

    return true;
}

void AMedievalTownGenerator::PlaceBuildings()
{
    TArray<FDistrictDef> Defs = BuildDistrictDefs();
    int32 Placed = 0;

    // Helper: find the best matching FDistrictDef for a position
    auto FindDef = [&](FVector2D Pos2D) -> const FDistrictDef*
    {
        EDistrictType DType = GetDistrictAt(Pos2D);
        float Frac = Pos2D.Size() / TownRadius;
        float Angle = FMath::RadiansToDegrees(FMath::Atan2(Pos2D.Y, Pos2D.X));

        // First try angular wedge defs that match position precisely
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type != DType) continue;
            if (D.bUsesAngle)
            {
                if (Frac >= D.InnerRadiusFraction && Frac <= D.OuterRadiusFraction &&
                    AngleInSector(Angle, D.MinAngleDeg, D.MaxAngleDeg))
                    return &D;
            }
        }
        // Then try radius-only defs
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type == DType && !D.bUsesAngle) return &D;
        }
        // Fallback to any matching type
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type == DType) return &D;
        }
        return nullptr;
    };

    // ═══════════════════════════════════════════════════════════════════════════
    //  Phase A: Line buildings along roads
    //
    //  Walk each road edge, placing buildings on both sides at intervals
    //  dictated by the local district's rules. Dense districts (GateWard, Slums)
    //  pack buildings tighter; formal districts (InnerWard) space them out.
    // ═══════════════════════════════════════════════════════════════════════════

    for (int32 EdgeIdx = 0; EdgeIdx < RoadEdges.Num(); EdgeIdx++)
    {
        const FRoadEdge& Edge = RoadEdges[EdgeIdx];
        if (!Edge.bIsGenerated || Edge.WorldPoints.Num() < 2) continue;

        // Compute cumulative distance along road for interpolation
        float TotalLen = 0.f;
        TArray<float> CumDist;
        CumDist.Add(0.f);
        for (int32 i = 0; i < Edge.WorldPoints.Num() - 1; i++)
        {
            TotalLen += (Edge.WorldPoints[i + 1] - Edge.WorldPoints[i]).Size();
            CumDist.Add(TotalLen);
        }

        // Sample district at road midpoint to get edge-level defaults
        FVector ActorLoc = GetActorLocation();
        FVector MidPt = Edge.WorldPoints[Edge.WorldPoints.Num() / 2];
        FVector2D MidPt2D(MidPt.X - ActorLoc.X, MidPt.Y - ActorLoc.Y);
        const FDistrictDef* EdgeDef = FindDef(MidPt2D);
        if (!EdgeDef) continue;

        // Per-district edge length threshold
        if (TotalLen < EdgeDef->MinEdgeLen) continue;

        // Place buildings on each side of the road independently
        for (int32 Side = 0; Side < 2; Side++)
        {
            float SideSign = (Side == 0) ? 1.f : -1.f;
            float Cursor = EdgeDef->CursorStartOffset;

            while (Cursor < TotalLen - EdgeDef->CursorStartOffset && Placed < TargetBuildingCount)
            {
                // ── Find point at Cursor distance along road ────────────
                FVector RoadPt = FVector::ZeroVector;
                FVector Tangent = FVector::ForwardVector;
                bool bFound = false;
                for (int32 i = 0; i < CumDist.Num() - 1; i++)
                {
                    if (CumDist[i + 1] >= Cursor)
                    {
                        float SegLen = CumDist[i + 1] - CumDist[i];
                        float LocalT = (SegLen > 1.f) ? (Cursor - CumDist[i]) / SegLen : 0.f;
                        RoadPt = FMath::Lerp(Edge.WorldPoints[i], Edge.WorldPoints[i + 1], LocalT);
                        Tangent = (Edge.WorldPoints[i + 1] - Edge.WorldPoints[i]).GetSafeNormal();
                        bFound = true;
                        break;
                    }
                }
                if (!bFound) break;

                // 2D tangent and perpendicular
                FVector2D Tan2D(Tangent.X, Tangent.Y);
                if (Tan2D.SizeSquared() < 0.01f) { Cursor += 100.f; continue; }
                Tan2D.Normalize();
                FVector2D Perp2D(-Tan2D.Y, Tan2D.X);

                // Road point in local (actor-relative) 2D
                FVector2D RoadPt2D(RoadPt.X - ActorLoc.X, RoadPt.Y - ActorLoc.Y);

                // ── Determine district at this exact road position ───────
                const FDistrictDef* LocalDef = FindDef(RoadPt2D);
                if (!LocalDef) { Cursor += 200.f; continue; }

                // ── Pick building style + footprint using district rules ─
                EBuildingStyle Style = PickStyle(LocalDef->Type, *LocalDef);
                FVector2D BaseSize = BuildingSize(Style);
                float Scale = Rand.FRandRange(LocalDef->MinScale, LocalDef->MaxScale);
                FVector2D Footprint = BaseSize * Scale;
                float Frontage = Footprint.X;
                float Depth    = Footprint.Y;

                // ── Compute candidate center (per-district setback) ──────
                float DistrictSetback = RoadBuildingSetback * LocalDef->SetbackMult;
                float Offset = Edge.Width * 0.5f + DistrictSetback + Depth * 0.5f;
                FVector2D CandPos2D = RoadPt2D + Perp2D * SideSign * Offset;

                // ── Terrain height (sample 5 points) ─────────────────────
                float HW = Footprint.X * 0.5f, HD = Footprint.Y * 0.5f;
                float H = GetTerrainHeight(CandPos2D.X, CandPos2D.Y);
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y + HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y + HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y - HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y - HD));

                FVector CandPos(CandPos2D.X, CandPos2D.Y, H);
                float CollRadius = FMath::Max(Footprint.X, Footprint.Y) * 0.6f;

                // ── Try to place (skip overlap with this road edge) ──────
                float DistrictSpacing = MinBuildingSpacing * LocalDef->SpacingMult;
                if (CanPlaceLot(CandPos, CollRadius, EdgeIdx, DistrictSpacing))
                {
                    FBuildingLot Lot;
                    Lot.Center = ActorLoc + CandPos;
                    Lot.Footprint = Footprint;
                    Lot.District = LocalDef->Type;
                    Lot.Style = Style;
                    Lot.NumFloors = PickFloorCount(Style, LocalDef->Type);
                    Lot.Roof = PickRoof(Style);
                    Lot.CollisionRadius = CollRadius;
                    Lot.bIsPlaced = true;

                    // Yaw: building front faces the road + district jitter
                    FVector2D FacingDir = -Perp2D * SideSign;
                    Lot.Yaw = FMath::RadiansToDegrees(FMath::Atan2(FacingDir.Y, FacingDir.X));
                    Lot.Yaw += Rand.FRandRange(-LocalDef->RotationJitter,
                                                LocalDef->RotationJitter);

                    // L-shape wing per district probability
                    if (Rand.FRand() < LocalDef->LShapeChance &&
                        (Style == EBuildingStyle::GuildHall || Style == EBuildingStyle::Keep ||
                         Style == EBuildingStyle::TavernInn || Style == EBuildingStyle::Warehouse))
                    {
                        Lot.bHasWing = true;
                        Lot.WingFootprint = FVector2D(Footprint.X * 0.4f, Footprint.Y * 0.6f);
                        Lot.WingYawOffset = 90.f;
                    }

                    PlacedLots.Add(Lot);
                    Placed++;

                    // Advance cursor: per-district spacing
                    Cursor += Frontage + DistrictSpacing * 0.5f;
                }
                else
                {
                    Cursor += 100.f;
                }
            }
        }
    }

    int32 RoadLinedCount = Placed;

    // ═══════════════════════════════════════════════════════════════════════════
    //  Phase B: Fill remaining space with scatter buildings
    //
    //  POSITION-FIRST approach: generate a random position inside the town,
    //  determine which district it belongs to, then apply that district's rules.
    //  This ensures uniform coverage across all areas instead of clustering
    //  in angular-sector districts.
    // ═══════════════════════════════════════════════════════════════════════════

    int32 TryLimit = (TargetBuildingCount - Placed) * 40;

    for (int32 Try = 0; Try < TryLimit && Placed < TargetBuildingCount; Try++)
    {
        // ── Generate random position inside town walls ──────────────────
        float MaxR = TownRadius * 0.88f;
        FVector2D CandPos2D = RandAnnulus(0.f, MaxR);

        // Skip plaza area — no scatter buildings in the market plaza
        if (CandPos2D.Size() < TownRadius * 0.08f) continue;

        // ── Find matching district def ──────────────────────────────────
        EDistrictType DType = GetDistrictAt(CandPos2D);
        const FDistrictDef* MatchDef = nullptr;

        float Frac = CandPos2D.Size() / TownRadius;
        float Angle = FMath::RadiansToDegrees(FMath::Atan2(CandPos2D.Y, CandPos2D.X));

        // Prefer angular wedge defs that match precisely
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type != DType) continue;
            if (D.bUsesAngle)
            {
                if (Frac >= D.InnerRadiusFraction && Frac <= D.OuterRadiusFraction &&
                    AngleInSector(Angle, D.MinAngleDeg, D.MaxAngleDeg))
                { MatchDef = &D; break; }
            }
        }
        if (!MatchDef)
        {
            for (const FDistrictDef& D : Defs)
                if (D.Type == DType && !D.bUsesAngle) { MatchDef = &D; break; }
        }
        if (!MatchDef) continue;

        // Density roll
        if (Rand.FRand() > MatchDef->Density) continue;

        EBuildingStyle Style = PickStyle(MatchDef->Type, *MatchDef);
        FVector2D BaseSize = BuildingSize(Style);
        float Scale = Rand.FRandRange(MatchDef->MinScale, MatchDef->MaxScale);
        FVector2D Footprint = BaseSize * Scale;
        float CollRadius = FMath::Max(Footprint.X, Footprint.Y) * 0.6f;

        float HW = Footprint.X * 0.5f, HD = Footprint.Y * 0.5f;
        float H = GetTerrainHeight(CandPos2D.X, CandPos2D.Y);
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y - HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y - HD));

        FVector CandPos(CandPos2D.X, CandPos2D.Y, H);
        float DistSpacing = MinBuildingSpacing * MatchDef->SpacingMult;
        if (!CanPlaceLot(CandPos, CollRadius, -1, DistSpacing)) continue;

        FBuildingLot Lot;
        Lot.Center = GetActorLocation() + CandPos;
        Lot.Footprint = Footprint;
        Lot.District = MatchDef->Type;
        Lot.Style = Style;
        Lot.NumFloors = PickFloorCount(Style, MatchDef->Type);
        Lot.Roof = PickRoof(Style);
        Lot.CollisionRadius = CollRadius;
        Lot.bIsPlaced = true;

        // Orient scatter buildings toward nearest road for coherent block layout
        float BestRoadDist = 1e9f;
        FVector2D BestRoadDir = FVector2D(1.f, 0.f);
        for (const FRoadEdge& RE : RoadEdges)
        {
            if (!RE.bIsGenerated) continue;
            FVector2D A = RoadNodes[RE.NodeA].Pos;
            FVector2D B = RoadNodes[RE.NodeB].Pos;
            FVector2D AB = B - A, AP = CandPos2D - A;
            float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                                   FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
            FVector2D Closest = A + AB * T;
            float Dist = (CandPos2D - Closest).Size();
            if (Dist < BestRoadDist)
            {
                BestRoadDist = Dist;
                BestRoadDir = (Closest - CandPos2D).GetSafeNormal();
            }
        }

        // Face toward nearest road with per-district jitter
        Lot.Yaw = FMath::RadiansToDegrees(FMath::Atan2(BestRoadDir.Y, BestRoadDir.X));
        Lot.Yaw += Rand.FRandRange(-MatchDef->RotationJitter, MatchDef->RotationJitter);

        // Per-district L-shape probability
        if (Rand.FRand() < MatchDef->LShapeChance &&
            (Style == EBuildingStyle::GuildHall || Style == EBuildingStyle::Keep ||
             Style == EBuildingStyle::TavernInn || Style == EBuildingStyle::Warehouse))
        {
            Lot.bHasWing = true;
            Lot.WingFootprint = FVector2D(Footprint.X * 0.4f, Footprint.Y * 0.6f);
            Lot.WingYawOffset = 90.f;
        }

        PlacedLots.Add(Lot);
        Placed++;
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] PlaceBuildings: %d road-lined + %d fill = %d / %d requested"),
           RoadLinedCount, Placed - RoadLinedCount, Placed, TargetBuildingCount);
}

// ─────────────────────────────────────────────────────────────────────────────
//  §8  MODULAR BUILDING MESH GENERATION
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SpawnModularBuilding(const FBuildingLot& Lot)
{
    const FVector& Center = Lot.Center;
    const float W = Lot.Footprint.X;
    const float D = Lot.Footprint.Y;
    const float Yaw = Lot.Yaw;
    const int32 Floors = Lot.NumFloors;

    float Z = Center.Z;

    // 1. Foundation
    SpawnFoundation(Center, W + 20.f, D + 20.f, FoundationHeight, Yaw);
    Z += FoundationHeight;

    // 2. Floor layers
    for (int32 F = 0; F < Floors; F++)
    {
        FVector FloorBase(Center.X, Center.Y, Z);
        bool bHasDoor = (F == 0);
        bool bHasWindows = true;
        SpawnWallFloor(FloorBase, W, D, FloorHeight, Yaw, bHasWindows, bHasDoor);
        Z += FloorHeight;
    }

    FVector RoofBase(Center.X, Center.Y, Z);

    // 3. Wing (L-shape) — spawned before main roof
    if (Lot.bHasWing)
    {
        // Offset wing to one side of main building
        float WingOffX = (W * 0.5f + Lot.WingFootprint.X * 0.5f) *
                         FMath::Cos(FMath::DegreesToRadians(Yaw));
        float WingOffY = (W * 0.5f + Lot.WingFootprint.X * 0.5f) *
                         FMath::Sin(FMath::DegreesToRadians(Yaw));
        FVector WingBase(Center.X + WingOffX, Center.Y + WingOffY, Center.Z);
        float WingH = GetTerrainHeight(WingBase.X - GetActorLocation().X,
                                       WingBase.Y - GetActorLocation().Y);
        WingBase.Z = GetActorLocation().Z + WingH;

        SpawnFoundation(WingBase, Lot.WingFootprint.X + 12.f,
                        Lot.WingFootprint.Y + 12.f, FoundationHeight, Yaw);
        float WZ = WingBase.Z + FoundationHeight;
        SpawnWallFloor(FVector(WingBase.X, WingBase.Y, WZ),
                       Lot.WingFootprint.X, Lot.WingFootprint.Y, FloorHeight, Yaw, true, false);

        FVector WingRoof(WingBase.X, WingBase.Y, WZ + FloorHeight);
        SpawnRoof_Pitched(WingRoof, Lot.WingFootprint.X, Lot.WingFootprint.Y,
                          FloorHeight * 0.5f, RoofOverhang, Yaw);
    }

    // 4. Roof (by type)
    float RoofH = (W < 500.f) ? FloorHeight * 0.55f : FloorHeight * 0.45f;
    if (Lot.Style == EBuildingStyle::Church)
        RoofH = FloorHeight * 1.2f;

    switch (Lot.Roof)
    {
    case ERoofType::Pitched:
        SpawnRoof_Pitched(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::Hipped:
        SpawnRoof_Hipped(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::Gambrel:
        SpawnRoof_Gambrel(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::FlatParapet:
        SpawnRoof_FlatParapet(RoofBase, W, D, WallThickness * 0.8f, Yaw); break;
    case ERoofType::Conical:
        SpawnRoof_Conical(RoofBase, FMath::Max(W, D) * 0.5f, RoofH * 1.2f, 12); break;
    case ERoofType::Pyramidal:
        SpawnRoof_Pyramidal(RoofBase, W, D, RoofH, Yaw); break;
    case ERoofType::Thatched:
        // Thatched uses a low pitched roof with large overhang
        SpawnRoof_Pitched(RoofBase, W, D, RoofH * 0.7f, RoofOverhang * 1.6f, Yaw); break;
    }

    // 5. Chimney (pitched & gambrel roofs get chimneys)
    if (Lot.Roof == ERoofType::Pitched || Lot.Roof == ERoofType::Gambrel ||
        Lot.Roof == ERoofType::Thatched)
    {
        int32 NumChimneys = (Lot.Style == EBuildingStyle::Blacksmith ||
                             Lot.Style == EBuildingStyle::Bakery) ? 2 : 1;
        float TotalH = FoundationHeight + Floors * FloorHeight;
        SpawnChimney(Center, TotalH, Yaw, NumChimneys);
    }

    // 6. Ground-level props (barrels, crates, etc.)
    SpawnGroundProps(Center, W, D, Yaw, Lot.Style);
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnFoundation(FVector Center,
                                                                    float W, float D,
                                                                    float Height, float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Foundation"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Extend foundation below ground to embed into terrain slopes
    // Scale embed depth with residual terrain variation inside walls
    float EmbedDepth = FMath::Max(80.f, TerrainAmplitude * (1.f - TownFlattenStrength) * 0.8f);
    float TotalH = Height + EmbedDepth;
    // Centered box: center at TotalH/2 - EmbedDepth so bottom is at -EmbedDepth, top at Height
    AddBox(V, T, N, UV, FVector(0, 0, TotalH * 0.5f - EmbedDepth), W, D, TotalH);

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnWallFloor(FVector BaseCenter,
                                                                   float W, float D,
                                                                   float FloorH, float Yaw,
                                                                   bool bAddWindows,
                                                                   bool bAddDoor)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("WallFloor"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Main floor box (open top to reduce poly count on top face)
    AddOpenTopBox(V, T, N, UV, FVector(0, 0, FloorH * 0.5f), W, D, FloorH);

    // Window insets (front and back faces)
    if (bAddWindows)
    {
        int32 NumWin = FMath::Max(1, (int32)(W / 200.f));
        float WinSpacing = W / (NumWin + 1);
        float WinW = WinSpacing * 0.3f;
        float WinH = FloorH * 0.4f;
        float WinZ = FloorH * 0.55f;
        float WinDepth = 12.f;

        // Windows are represented as slightly inset quads (not full cutouts, keeps mesh simple)
        for (int32 Wi = 0; Wi < NumWin; Wi++)
        {
            float XOff = -W * 0.5f + WinSpacing * (Wi + 1);

            // Front face window (Y = -D/2)
            FVector FWC(XOff, -D * 0.5f - WinDepth * 0.01f, WinZ);
            AddBox(V, T, N, UV, FWC, WinW, 4.f, WinH);   // Thin inset

            // Back face window
            FVector BWC(XOff, D * 0.5f + WinDepth * 0.01f, WinZ);
            AddBox(V, T, N, UV, BWC, WinW, 4.f, WinH);
        }
    }

    // Door (ground floor front face)
    if (bAddDoor)
    {
        float DoorW = FMath::Min(W * 0.18f, 120.f);
        float DoorH = FloorH * 0.7f;
        float DoorXOff = Rand.FRandRange(-W * 0.15f, W * 0.15f);

        // Door frame center height: midpoint of the door pillar
        float PillarH = DoorH * 0.9f;
        float PillarCenterZ = PillarH * 0.5f;

        FVector DoorCenter(DoorXOff, -D * 0.5f + 2.f, 0.f);
        // Door frame (two side pillars — centered around PillarCenterZ)
        AddBox(V, T, N, UV,
               DoorCenter + FVector(-DoorW * 0.5f - 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        AddBox(V, T, N, UV,
               DoorCenter + FVector(DoorW * 0.5f + 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        // Lintel — centered at top of pillars
        AddBox(V, T, N, UV,
               DoorCenter + FVector(0, 0, DoorH + 8.f),
               DoorW + 28.f, 8.f, 16.f);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, WallMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

// ── Roof variants ─────────────────────────────────────────────────────────────

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Pitched(FVector BaseCenter,
                                                                      float W, float D,
                                                                      float RoofH,
                                                                      float Overhang,
                                                                      float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofPitched"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddPitchedRoof(V, T, N, UV, FVector::ZeroVector, W, D, RoofH, Overhang);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Hipped(FVector BaseCenter,
                                                                     float W, float D,
                                                                     float RoofH,
                                                                     float Overhang,
                                                                     float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofHipped"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddHippedRoof(V, T, N, UV, FVector::ZeroVector, W, D, RoofH, Overhang);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Gambrel(FVector BaseCenter,
                                                                      float W, float D,
                                                                      float RoofH,
                                                                      float Overhang,
                                                                      float Yaw)
{
    // Gambrel = two-stage pitch: lower steeper, upper shallower
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofGambrel"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    float LowerH = RoofH * 0.55f;
    float UpperH = RoofH * 0.45f;
    float LowerW = W;
    float UpperW = W * 0.55f;

    // Lower stage (steep)
    AddPitchedRoof(V, T, N, UV, FVector(0, 0, 0), LowerW, D, LowerH, Overhang);
    // Upper stage (shallower, on top of lower ridge)
    AddPitchedRoof(V, T, N, UV, FVector(0, 0, LowerH), UpperW, D, UpperH, 0.f);

    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_FlatParapet(FVector BaseCenter,
                                                                          float W, float D,
                                                                          float ParapetH,
                                                                          float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofFlat"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Flat cap
    AddBox(V, T, N, UV, FVector(0, 0, ParapetH * 0.5f), W, D, ParapetH);

    // Parapet walls on all 4 sides
    float PH = WallHeight * 0.12f;
    float PW = WallThickness * 0.6f;
    AddBox(V, T, N, UV, FVector(0, -D * 0.5f - PW * 0.5f, ParapetH + PH * 0.5f), W + PW*2, PW, PH);
    AddBox(V, T, N, UV, FVector(0,  D * 0.5f + PW * 0.5f, ParapetH + PH * 0.5f), W + PW*2, PW, PH);
    AddBox(V, T, N, UV, FVector(-W * 0.5f - PW * 0.5f, 0, ParapetH + PH * 0.5f), PW, D, PH);
    AddBox(V, T, N, UV, FVector( W * 0.5f + PW * 0.5f, 0, ParapetH + PH * 0.5f), PW, D, PH);

    // Battlements on parapet
    for (float XBatt = -W * 0.5f + 30.f; XBatt < W * 0.5f; XBatt += 60.f)
    {
        AddBox(V, T, N, UV,
               FVector(XBatt, -D * 0.5f - PW * 0.5f, ParapetH + PH + 25.f),
               30.f, PW, 40.f);
        AddBox(V, T, N, UV,
               FVector(XBatt, D * 0.5f + PW * 0.5f, ParapetH + PH + 25.f),
               30.f, PW, 40.f);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Conical(FVector BaseCenter,
                                                                      float Radius,
                                                                      float ConeH, int32 Segs)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofConical"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddCone(V, T, N, UV, FVector::ZeroVector, Radius * 1.08f, ConeH, Segs);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Pyramidal(FVector BaseCenter,
                                                                        float W, float D,
                                                                        float PyramidH, float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofPyramid"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddPyramid(V, T, N, UV, FVector::ZeroVector, W, D, PyramidH);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnChimney(FVector Center,
                                                                float BuildingH, float Yaw,
                                                                int32 Count)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Chimney"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    float ChimneyH = BuildingH * 0.35f;
    float ChimR = 28.f;

    for (int32 i = 0; i < Count; i++)
    {
        float XOff = Count > 1 ? (i == 0 ? -70.f : 70.f) : 0.f;
        float YOff = Rand.FRandRange(-40.f, 40.f);
        FVector Base(XOff, YOff, BuildingH);
        // Stack: body cylinder + cap flare
        AddCylinder(V, T, N, UV, Base, ChimR, ChimneyH, 8, false);
        AddCylinder(V, T, N, UV, Base + FVector(0,0,ChimneyH), ChimR * 1.35f, 20.f, 8, true);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnGroundProps(FVector Center, float W,
                                                                     float D, float Yaw,
                                                                     EBuildingStyle Style)
{
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    int32 NumProps = Rand.RandRange(0, 3);
    for (int32 i = 0; i < NumProps; i++)
    {
        float PX = Rand.FRandRange(-W * 0.6f, W * 0.6f);
        float PY = Rand.FRandRange(-D * 0.6f, D * 0.6f);

        if (Style == EBuildingStyle::Blacksmith)
        {
            // Anvil-like block
            AddBox(V, T, N, UV, FVector(PX, PY, 35.f), 60.f, 40.f, 55.f);
        }
        else if (Style == EBuildingStyle::Bakery || Style == EBuildingStyle::TavernInn)
        {
            // Barrel: cylinder
            AddCylinder(V, T, N, UV, FVector(PX, PY, 0), 22.f, 55.f, 8, true);
        }
        else
        {
            // Generic crate
            float S = Rand.FRandRange(30.f, 70.f);
            AddBox(V, T, N, UV, FVector(PX, PY, S * 0.5f), S, S, S);
        }
    }

    if (V.Num() == 0) return nullptr;

    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Props"));
    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}
