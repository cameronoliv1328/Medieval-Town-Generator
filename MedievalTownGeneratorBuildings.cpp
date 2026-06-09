// Buildings module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorBuildings.h"
#include "MedievalTownGenerator.h"
#include "OrganicParcelGenerator.h"

float MTGBuildings::ComputeRoofHeight(float Width, float Depth, float PitchAngleDeg)
{
    const float HalfSpan = FMath::Min(Width, Depth) * 0.5f;
    return FMath::Tan(FMath::DegreesToRadians(PitchAngleDeg)) * HalfSpan;
}


// -----------------------------------------------------------------------------
//  ?7  BUILDING PLACEMENT  (district-aware + flat-area filtered)
// -----------------------------------------------------------------------------

TArray<FDistrictDef> AMedievalTownGenerator::BuildDistrictDefs() const
{
    TArray<FDistrictDef> Defs;

    // ---------------------------------------------------------------------
    //  Compute feature angles for angular sector assignment
    //
    //  Medieval towns weren't organized in perfect concentric rings -- they
    //  had angular sectors driven by geography:
    //    * Craft/industrial quarters along the river
    //    * Gate wards at each entrance (bustling commercial zones)
    //    * Administrative/military core at center
    //    * Slums on the least desirable side (downwind, away from gates)
    //    * Merchant quarters filling the best remaining sectors
    // ---------------------------------------------------------------------

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

    // -- 1. Inner Ward (castle/admin core) --------------------------------
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
        D.SpacingMult = 1.4f;       // Wide spacing -- prestigious buildings
        D.SetbackMult = 1.3f;       // Larger setback from road
        D.RotationJitter = 3.f;     // Very formal, aligned
        D.LShapeChance = 0.4f;      // Many L-shaped compounds
        D.CursorStartOffset = 300.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // -- 2. Craft Quarter (river-side industrial) -------------------------
    //    Warehouses, blacksmiths, yards near the river.
    //    Medium density, bigger buildings, oriented toward water.
    //    Angular wedge centered on river direction ?45?.
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
        D.SpacingMult = 1.1f;       // Moderate spacing -- work yards between buildings
        D.SetbackMult = 1.5f;       // Bigger setback -- loading areas along road
        D.RotationJitter = 6.f;     // Somewhat organized
        D.LShapeChance = 0.35f;     // Workshops often have wings
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 400.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = RiverAngle - 45.f;
        D.MaxAngleDeg = RiverAngle + 45.f;
        Defs.Add(D);
    }

    // -- 3. Gate Wards (bustling commercial at each gate) -----------------
    //    Dense shops, taverns, cheap lodging near the town gates.
    //    Tight spacing, small setback, high density.
    //    Each gate gets a ?20? angular wedge in the outer ring area.
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
        D.SpacingMult = 0.55f;      // Very tight -- cramped commercial zone
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

    // -- 4. Merchant Quarter (main commercial/residential belt) -----------
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
        D.SetbackMult = 0.85f;      // Close to road -- shopfronts
        D.RotationJitter = 5.f;     // Neat commercial streets
        D.LShapeChance = 0.2f;
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // -- 5. Outer Residential (between outer ring and walls) --------------
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

    // -- 6. Slums (poor quarter -- opposite river, least desirable) --------
    //    Tiny, densely packed cottages with chaotic orientation.
    //    Angular wedge opposite the river ?50?.
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
        D.SpacingMult = 0.4f;       // Extremely tight -- shanty-town density
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

    // -- 7. Transition Zone (stables/storage near walls) ------------------
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
        D.SpacingMult = 1.3f;       // Wide spacing -- yards, pens
        D.SetbackMult = 1.2f;
        D.RotationJitter = 12.f;
        D.LShapeChance = 0.1f;
        D.CursorStartOffset = 250.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    return Defs;
}

// -- Angular-aware district lookup ---------------------------------------
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

    // -- Center always = InnerWard --
    if (Frac < InnerRingRadius * 0.7f)
        return EDistrictType::InnerWard;

    // -- Near walls = TransitionZone --
    if (Frac > 0.88f)
        return EDistrictType::TransitionZone;

    // -- Check angular wedge districts (highest priority in the mid-ring) --
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

    // -- Radius-only fallback --
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

bool AMedievalTownGenerator::CanPlaceLot(FVector Center, float Radius, int32 IgnoreEdgeIndex, float SpacingOverride, float RiverBufferOverride) const
{
    FVector2D Pos2D(Center.X, Center.Y);

    // Must be inside town walls (with margin)
    if (Pos2D.Size() > TownRadius * 0.89f - Radius) return false;

    // Must not be near river (RiverBufferOverride < 0 = use default RiverBuildingBuffer)
    const float EffRiverBuff = (RiverBufferOverride >= 0.f) ? RiverBufferOverride : RiverBuildingBuffer;
    if (IsNearRiver(Pos2D, Radius + EffRiverBuff)) return false;

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

namespace
{
    EBuildingStyle LotKindToStyle(EOrganicLotKind Kind)
    {
        switch (Kind)
        {
        case EOrganicLotKind::Cottage:     return EBuildingStyle::SmallCottage;
        case EOrganicLotKind::TownHouse:   return EBuildingStyle::TownHouse;
        case EOrganicLotKind::GuildHall:   return EBuildingStyle::GuildHall;
        case EOrganicLotKind::Tavern:      return EBuildingStyle::TavernInn;
        case EOrganicLotKind::Warehouse:   return EBuildingStyle::Warehouse;
        case EOrganicLotKind::Blacksmith:  return EBuildingStyle::Blacksmith;
        case EOrganicLotKind::Bakery:      return EBuildingStyle::Bakery;
        case EOrganicLotKind::Stable:      return EBuildingStyle::Stable;
        case EOrganicLotKind::Outbuilding: return EBuildingStyle::Stable;
        case EOrganicLotKind::Church:      return EBuildingStyle::Church;
        case EOrganicLotKind::Keep:        return EBuildingStyle::Keep;
        }
        return EBuildingStyle::TownHouse;
    }

    // Soft district classification from the lot's continuous fields, kept for
    // the FBuildingLot/FMedievalParcel contract (PCG attributes, materials).
    EDistrictType DeriveDistrict(const FOrganicParcelLot& L, float RiverDist,
                                  float RadiusFrac)
    {
        if (RiverDist < 2400.f)                    return EDistrictType::CraftQuarter;
        if (RadiusFrac > 0.86f)                    return EDistrictType::TransitionZone;
        if (L.Wealth < 0.32f)                      return EDistrictType::Slums;
        if (L.Wealth > 0.68f && RadiusFrac < 0.4f) return EDistrictType::InnerWard;
        if (L.Wealth > 0.55f)                      return EDistrictType::MerchantQuarter;
        if (RadiusFrac > 0.62f)                    return EDistrictType::OuterResidential;
        return EDistrictType::MerchantQuarter;
    }
}

void AMedievalTownGenerator::PlaceBuildings()
{
    // -------------------------------------------------------------------------
    //  Burgage-plot placement (see OrganicParcelGenerator.h).
    //
    //  Every street frontage is subdivided into narrow, deep plots; houses sit
    //  on the frontage line, often sharing party walls, and plot interiors stay
    //  open apart from occasional rear outbuildings. Wealth/industry are
    //  continuous fields, so there are no hard district rings or wedges.
    // -------------------------------------------------------------------------

    // Rebuild a street-graph view from the road network. This includes quay
    // streets and anything else later phases appended to RoadEdges.
    FOrganicStreetGraph Streets;
    for (const FRoadNode& N : RoadNodes)
    {
        Streets.AddNode(N.Pos);
    }
    for (const FRoadEdge& E : RoadEdges)
    {
        if (!E.bIsGenerated || E.PolylinePoints.Num() < 2) continue;
        if (!RoadNodes.IsValidIndex(E.NodeA) || !RoadNodes.IsValidIndex(E.NodeB)) continue;

        EOrganicStreetType Type;
        switch (E.Tier)
        {
        case EStreetTier::Primary:   Type = EOrganicStreetType::Primary;   break;
        case EStreetTier::Secondary: Type = EOrganicStreetType::Secondary; break;
        case EStreetTier::RiverPath: Type = EOrganicStreetType::Lane;      break;
        default:
            Type = (E.Width < 260.f) ? EOrganicStreetType::Alley
                                     : EOrganicStreetType::Lane;
            break;
        }

        TArray<FVector2D> Poly = E.PolylinePoints;
        const int32 EIdx = Streets.AddEdge(E.NodeA, E.NodeB, Type, E.Width, MoveTemp(Poly));
        if (EIdx != INDEX_NONE)
        {
            Streets.Edges[EIdx].bIsBridge    = E.bIsBridge;
            Streets.Edges[EIdx].bIsPlazaEdge = E.bIsPlazaEdge;
        }
    }

    FOrganicParcelConfig PC;
    PC.TownRadius        = TownRadius;
    PC.WallFraction      = 0.90f;
    PC.MarketCenter      = CachedMarketPos;
    PC.ChurchPos         = CachedChurchPos;
    PC.KeepPos           = CachedKeepPos;
    PC.MarketPlazaRadius = CachedPlazaRadius * 0.70f;
    PC.SetbackCore       = RoadBuildingSetback * 0.5f;
    PC.SetbackEdge       = RoadBuildingSetback * 2.8f;
    PC.MaxLots           = FMath::Max(60, FMath::RoundToInt(TargetBuildingCount * 1.5f));

    FOrganicParcelTerrainQuery PT;
    PT.GetHeight   = [this](FVector2D P) { return GetTerrainHeight(P.X, P.Y); };
    PT.IsNearRiver = [this](FVector2D P, float Extra)
    {
        return bGenerateRiver ? IsNearRiver(P, Extra) : false;
    };
    PT.DistToRiver = [this](FVector2D P)
    {
        return bGenerateRiver ? DistToRiverCenter(P) : BIG_NUMBER;
    };

    FOrganicParcelGenerator ParcelGen(PC, PT, Rand);
    TArray<FOrganicParcelLot> Lots = ParcelGen.Generate(Streets);

    const FVector ActorLoc = GetActorLocation();
    int32 RowHouses = 0;

    for (const FOrganicParcelLot& L : Lots)
    {
        const float HW = L.Footprint.X * 0.5f;
        const float HD = L.Footprint.Y * 0.5f;

        // Foundation height: highest of centre + corners so the box never floats.
        float H = GetTerrainHeight(L.Center.X, L.Center.Y);
        H = FMath::Max(H, GetTerrainHeight(L.Center.X + HW, L.Center.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(L.Center.X - HW, L.Center.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(L.Center.X + HW, L.Center.Y - HD));
        H = FMath::Max(H, GetTerrainHeight(L.Center.X - HW, L.Center.Y - HD));

        const float RiverDist  = bGenerateRiver ? DistToRiverCenter(L.Center) : BIG_NUMBER;
        const float RadiusFrac = L.Center.Size() / TownRadius;

        FBuildingLot Lot;
        Lot.Center          = ActorLoc + FVector(L.Center.X, L.Center.Y, H);
        Lot.Footprint       = L.Footprint;
        Lot.Yaw             = L.YawDeg;
        Lot.Style           = LotKindToStyle(L.Kind);
        Lot.District        = DeriveDistrict(L, RiverDist, RadiusFrac);
        Lot.NumFloors       = L.Floors;
        Lot.Roof            = PickRoof(Lot.Style);
        Lot.CollisionRadius = 0.5f * FMath::Sqrt(FMath::Square(L.Footprint.X) +
                                                 FMath::Square(L.Footprint.Y));
        Lot.bIsPlaced       = true;

        // L-plan wings only on detached civic/work buildings; a wing on a
        // row house would punch through the neighbour's party wall.
        if (!L.bRowHouse && !L.bIsRearOutbuilding &&
            Rand.FRand() < LShapeProbability &&
            (Lot.Style == EBuildingStyle::GuildHall ||
             Lot.Style == EBuildingStyle::TavernInn ||
             Lot.Style == EBuildingStyle::Warehouse))
        {
            Lot.bHasWing      = true;
            Lot.WingFootprint = FVector2D(L.Footprint.X * 0.4f, L.Footprint.Y * 0.6f);
            Lot.WingYawOffset = 90.f;
        }

        if (L.bRowHouse) RowHouses++;
        PlacedLots.Add(Lot);
    }

    UE_LOG(LogTemp, Log,
           TEXT("[MTG] PlaceBuildings(Burgage): %d lots (%d row houses) from %d street edges"),
           PlacedLots.Num(), RowHouses, RoadEdges.Num());

    BuildParcelBoundary();
}

// -----------------------------------------------------------------------------
//  ?8  MODULAR BUILDING MESH GENERATION
// -----------------------------------------------------------------------------

void AMedievalTownGenerator::SpawnModularBuilding(const FBuildingLot& Lot)
{
    const FVector& Center = Lot.Center;
    const float W = Lot.Footprint.X;
    const float D = Lot.Footprint.Y;
    const float Yaw = Lot.Yaw;
    const int32 Floors = Lot.NumFloors;

    // Block-out mode: single cuboid for fast layout iteration
    if (bBlockOutMode)
    {
        const float TotalH = FoundationHeight + Floors * FloorHeight;
        UProceduralMeshComponent* BlockMesh = CreateMesh(TEXT("BlockOut"));
        TArray<FVector> BV; TArray<int32> BT; TArray<FVector> BN; TArray<FVector2D> BUV;
        AddBox(BV, BT, BN, BUV, FVector(0.f, 0.f, TotalH * 0.5f), W, D, TotalH);
        SetMeshSection(BlockMesh, 0, BV, BT, BN, BUV, WallMaterial);
        BlockMesh->SetWorldLocation(FVector(Center.X, Center.Y, Center.Z));
        BlockMesh->SetWorldRotation(FRotator(0.f, Yaw, 0.f));
        return;
    }

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

    // 3. Wing (L-shape) -- spawned before main roof
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
        // Door frame (two side pillars -- centered around PillarCenterZ)
        AddBox(V, T, N, UV,
               DoorCenter + FVector(-DoorW * 0.5f - 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        AddBox(V, T, N, UV,
               DoorCenter + FVector(DoorW * 0.5f + 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        // Lintel -- centered at top of pillars
        AddBox(V, T, N, UV,
               DoorCenter + FVector(0, 0, DoorH + 8.f),
               DoorW + 28.f, 8.f, 16.f);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, WallMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

// -- Roof variants -------------------------------------------------------------

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
