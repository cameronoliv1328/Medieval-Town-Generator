// =============================================================================
//  MedievalTownGeneratorLayout.cpp
//
//  Builds the Phase 1 → Phase 2 boundary arrays after raw generation runs.
//  These two functions read the actor's internal runtime data and produce the
//  structured FMedievalWall* and FMedievalStreet* arrays that PCG asset nodes
//  consume — no mesh spawning happens here.
//
//  Call order (managed by walls.cpp and roads.cpp respectively):
//    BuildWallBoundary()   — called at the end of GenerateWalls()
//    BuildStreetBoundary() — called at the end of ElevateRoadSplines()
// =============================================================================

#include "MedievalTownGenerator.h"

// =============================================================================
//  WALL BOUNDARY
// =============================================================================

void AMedievalTownGenerator::BuildWallBoundary()
{
    LayoutWallNodes.Empty();
    LayoutWallSegments.Empty();

    TArray<EWallModule> Modules;
    ParseGrammar(WallGrammarString, Modules);
    if (Modules.Num() == 0) return;

    const int32 N         = Modules.Num();
    const float AngleStep = 360.f / N;
    const FVector Origin  = GetActorLocation();
    const float TowerH    = WallHeight * WallTowerHeightFactor;

    // ModuleToNode[i] = index into LayoutWallNodes for module i, or -1.
    TArray<int32> ModuleToNode;
    ModuleToNode.Init(-1, N);

    // ----- Pass 1: create nodes for CornerTower and GateTower modules --------

    for (int32 i = 0; i < N; i++)
    {
        if (Modules[i] != EWallModule::CornerTower && Modules[i] != EWallModule::GateTower)
            continue;

        const float Angle     = i * AngleStep;
        const float NextAngle = (i + 1) * AngleStep;
        const float MidAngle  = (Angle + NextAngle) * 0.5f;

        FMedievalWallNode Node;
        Node.NodeIndex = LayoutWallNodes.Num();

        if (Modules[i] == EWallModule::GateTower)
        {
            // Reconstruct gate geometry identically to GenerateWalls()
            FVector StartPt  = WallPerimeterPoint(Angle);
            FVector EndPt    = WallPerimeterPoint(NextAngle);
            FVector GapStart = FMath::Lerp(StartPt, EndPt, 0.35f);
            FVector GapEnd   = FMath::Lerp(StartPt, EndPt, 0.65f);
            FVector GateCtr  = FMath::Lerp(GapStart, GapEnd, 0.5f);

            Node.WorldPos    = GateCtr;
            Node.NodeType    = EMedievalWallNodeType::GateTower;
            Node.GateWidth   = FVector2D::Distance(
                                   FVector2D(GapStart.X, GapStart.Y),
                                   FVector2D(GapEnd.X, GapEnd.Y));
            Node.TowerRadius = WallTowerRadius * 1.4f;
            Node.TowerHeight = TowerH * 1.15f;
            Node.bIsMainGate = false;  // resolved in pass 3
        }
        else  // CornerTower
        {
            Node.WorldPos    = WallPerimeterPoint(MidAngle);
            Node.NodeType    = EMedievalWallNodeType::CornerTower;
            Node.TowerRadius = WallTowerRadius;
            Node.TowerHeight = TowerH;
        }

        FVector2D NodePos2D(Node.WorldPos.X - Origin.X, Node.WorldPos.Y - Origin.Y);
        Node.OutwardFacing = NodePos2D.GetSafeNormal();

        ModuleToNode[i] = Node.NodeIndex;
        LayoutWallNodes.Add(Node);
    }

    // ----- Pass 2: create segments (one per grammar module) ------------------

    for (int32 i = 0; i < N; i++)
    {
        const float Angle     = i * AngleStep;
        const float NextAngle = (i + 1) * AngleStep;

        FVector StartPt = WallPerimeterPoint(Angle);
        FVector EndPt   = WallPerimeterPoint(NextAngle);

        FMedievalWallSegment Seg;
        Seg.SegmentIndex = i;
        Seg.WorldStart   = StartPt;
        Seg.WorldEnd     = EndPt;

        // Local 2D coords for normal / length computation
        FVector2D S2D(StartPt.X - Origin.X, StartPt.Y - Origin.Y);
        FVector2D E2D(EndPt.X   - Origin.X, EndPt.Y   - Origin.Y);
        Seg.Length = FVector2D::Distance(S2D, E2D);

        // Outward normal: CCW perpendicular of segment direction, verified against
        // the outward radial direction at the segment midpoint.
        FVector2D SegDir = (E2D - S2D).GetSafeNormal();
        FVector2D Perp(-SegDir.Y, SegDir.X);
        FVector2D Mid2D = (S2D + E2D) * 0.5f;
        if (FVector2D::DotProduct(Perp, Mid2D) < 0.f)
            Perp = -Perp;
        Seg.OutwardNormal = Perp;

        // Parapet modules are shorter and thinner (matches GenerateWalls behaviour)
        const bool bIsParapet = (Modules[i] == EWallModule::Parapet);
        Seg.WallHeight    = bIsParapet ? WallHeight * 0.55f : WallHeight;
        Seg.WallThickness = bIsParapet ? WallThickness * 0.7f : WallThickness;

        // Tower/gate module spans always have their decorative feature in the middle;
        // the wall run itself carries no battlements (the tower cap has them instead).
        Seg.bHasBattlements = (Modules[i] != EWallModule::CornerTower &&
                               Modules[i] != EWallModule::GateTower);

        // Terrain samples: one every ~300 cm so PCG can step wall sections with terrain.
        const int32 NumSamples = FMath::Max(2, FMath::RoundToInt(Seg.Length / 300.f));
        Seg.TerrainSamples.Reserve(NumSamples + 1);
        for (int32 s = 0; s <= NumSamples; s++)
        {
            const float T    = (float)s / (float)NumSamples;
            FVector2D SPos   = FMath::Lerp(S2D, E2D, T);
            const float H    = GetTerrainHeight(SPos.X, SPos.Y) + Origin.Z;
            Seg.TerrainSamples.Add(FVector(SPos.X + Origin.X, SPos.Y + Origin.Y, H));
        }

        // Connectivity: NodeIndexA = node from the previous module (sits at the module
        // boundary before this segment); NodeIndexB = node mid-span of this module.
        Seg.NodeIndexA = ModuleToNode[(i - 1 + N) % N];
        Seg.NodeIndexB = ModuleToNode[i];

        LayoutWallSegments.Add(Seg);
    }

    // ----- Pass 3: wire SegmentIndexCW/CCW and identify the main gate --------

    for (FMedievalWallNode& Node : LayoutWallNodes)
    {
        for (int32 s = 0; s < LayoutWallSegments.Num(); s++)
        {
            // The segment that leads INTO this node (NodeIndexB == this node)
            if (LayoutWallSegments[s].NodeIndexB == Node.NodeIndex)
                Node.SegmentIndexCCW = s;
            // The segment that leads AWAY FROM this node (NodeIndexA == this node)
            if (LayoutWallSegments[s].NodeIndexA == Node.NodeIndex)
                Node.SegmentIndexCW = s;
        }
    }

    // The main gate is the gate whose inward-facing direction best aligns with
    // the direction from the wall toward the market square.
    {
        const FVector2D MarketDir = CachedMarketPos.IsNearlyZero()
                                  ? FVector2D(1.f, 0.f)
                                  : CachedMarketPos.GetSafeNormal();

        float BestDot   = -2.f;
        int32 MainGateN = -1;

        for (int32 n = 0; n < LayoutWallNodes.Num(); n++)
        {
            if (LayoutWallNodes[n].NodeType != EMedievalWallNodeType::GateTower) continue;

            // Gate faces outward; market is inward — flip with negation.
            const float Dot = FVector2D::DotProduct(-LayoutWallNodes[n].OutwardFacing, MarketDir);
            if (Dot > BestDot) { BestDot = Dot; MainGateN = n; }
        }

        if (MainGateN >= 0)
            LayoutWallNodes[MainGateN].bIsMainGate = true;
    }

    UE_LOG(LogTemp, Log,
           TEXT("[MTG] BuildWallBoundary: %d nodes, %d segments"),
           LayoutWallNodes.Num(), LayoutWallSegments.Num());
}

// =============================================================================
//  STREET BOUNDARY
// =============================================================================

void AMedievalTownGenerator::BuildStreetBoundary()
{
    LayoutStreetNodes.Empty();
    LayoutStreetEdges.Empty();

    const FVector Origin = GetActorLocation();

    // ----- Nodes -------------------------------------------------------------

    LayoutStreetNodes.Reserve(RoadNodes.Num());

    for (const FRoadNode& RN : RoadNodes)
    {
        FMedievalStreetNode SN;
        SN.NodeIndex          = RN.Index;
        SN.WorldPos           = Origin + FVector(RN.Pos.X, RN.Pos.Y,
                                    GetTerrainHeight(RN.Pos.X, RN.Pos.Y) + 10.f);
        SN.ConnectedEdgeIndices = RN.ConnectedEdges;
        SN.bIsMarket            = RN.bIsMarket;
        SN.bIsGate              = RN.bIsGate;
        SN.bIsBridgeAbutment    = RN.bIsBridgeNode;

        // Classify intersection topology. Gate and Market take precedence over
        // degree-based classification because a 3-way gate reads very differently
        // from a 3-way mid-street junction.
        if (RN.bIsMarket)
            SN.IntersectionType = EMedievalIntersectionType::Market;
        else if (RN.bIsGate)
            SN.IntersectionType = EMedievalIntersectionType::Gate;
        else if (RN.ConnectedEdges.Num() <= 1)
            SN.IntersectionType = EMedievalIntersectionType::Terminus;
        else if (RN.ConnectedEdges.Num() >= 4)
            SN.IntersectionType = EMedievalIntersectionType::FourWay;
        else
            SN.IntersectionType = EMedievalIntersectionType::TJunction;

        LayoutStreetNodes.Add(SN);
    }

    // ----- Edges -------------------------------------------------------------

    LayoutStreetEdges.Reserve(RoadEdges.Num());

    for (int32 ei = 0; ei < RoadEdges.Num(); ei++)
    {
        const FRoadEdge& RE = RoadEdges[ei];
        if (!RE.bIsGenerated || RE.WorldPoints.Num() < 2) continue;

        FMedievalStreetEdge SE;
        SE.EdgeIndex     = ei;
        SE.NodeIndexA    = RE.NodeA;
        SE.NodeIndexB    = RE.NodeB;
        SE.Width         = RE.Width;
        SE.bIsBridge     = RE.bIsBridge;
        SE.WorldPolyline = RE.WorldPoints;

        SE.bIsQuayStreet = (RE.Tier == EStreetTier::RiverPath);

        // Map internal tier enum to canonical street type
        switch (RE.Tier)
        {
        case EStreetTier::Primary:   SE.StreetType = EMedievalStreetType::Primary;   break;
        case EStreetTier::Secondary: SE.StreetType = EMedievalStreetType::Secondary; break;
        case EStreetTier::Tertiary:  SE.StreetType = EMedievalStreetType::Alley;     break;
        case EStreetTier::RiverPath: SE.StreetType = EMedievalStreetType::Lane;      break;
        default:                     SE.StreetType = EMedievalStreetType::Secondary; break;
        }

        // Grade: maximum slope angle across all consecutive point pairs.
        // Used downstream to pick surface variants within a tier (e.g. steep
        // primaries keep cobbles even near the market).
        SE.GradeDeg = 0.f;
        for (int32 pi = 1; pi < RE.WorldPoints.Num(); pi++)
        {
            const float DZ  = FMath::Abs(RE.WorldPoints[pi].Z - RE.WorldPoints[pi - 1].Z);
            const float DXY = FVector2D::Distance(
                FVector2D(RE.WorldPoints[pi].X,     RE.WorldPoints[pi].Y),
                FVector2D(RE.WorldPoints[pi - 1].X, RE.WorldPoints[pi - 1].Y));
            if (DXY > 1.f)
                SE.GradeDeg = FMath::Max(SE.GradeDeg,
                                         FMath::RadiansToDegrees(FMath::Atan2(DZ, DXY)));
        }

        // Surface: bridge and quay take priority, then derive from tier + market proximity.
        if (SE.bIsBridge)
        {
            SE.Surface = EMedievalRoadSurface::BridgeDeck;
        }
        else if (SE.bIsQuayStreet)
        {
            SE.Surface = EMedievalRoadSurface::QuaySetts;
        }
        else
        {
            // A primary edge that touches the market node gets flagstone.
            const bool bNearMarket =
                (RoadNodes.IsValidIndex(RE.NodeA) && RoadNodes[RE.NodeA].bIsMarket) ||
                (RoadNodes.IsValidIndex(RE.NodeB) && RoadNodes[RE.NodeB].bIsMarket);

            switch (RE.Tier)
            {
            case EStreetTier::Primary:
                SE.Surface = (bNearMarket && SE.GradeDeg < 8.f)
                           ? EMedievalRoadSurface::MarketFlagstone
                           : EMedievalRoadSurface::PrimaryCobble;
                break;
            case EStreetTier::Secondary:
                SE.Surface = EMedievalRoadSurface::SecondaryDirt;
                break;
            default:
                SE.Surface = EMedievalRoadSurface::AlleyDirt;
                break;
            }
        }

        // Bridge span geometry: find the portion of the polyline that passes over
        // the river exclusion zone and measure it for abutment sizing.
        if (SE.bIsBridge)
        {
            const float ExclusionCheck = RiverExclusionRadius + RiverWidth * 0.5f;

            int32 SpanStart = 0;
            int32 SpanEnd   = RE.WorldPoints.Num() - 1;

            for (int32 pi = 0; pi < RE.WorldPoints.Num(); pi++)
            {
                FVector2D P2D(RE.WorldPoints[pi].X - Origin.X,
                              RE.WorldPoints[pi].Y - Origin.Y);
                if (DistToRiverCenter(P2D) < ExclusionCheck)
                {
                    SpanStart = pi;
                    break;
                }
            }
            for (int32 pi = RE.WorldPoints.Num() - 1; pi >= 0; pi--)
            {
                FVector2D P2D(RE.WorldPoints[pi].X - Origin.X,
                              RE.WorldPoints[pi].Y - Origin.Y);
                if (DistToRiverCenter(P2D) < ExclusionCheck)
                {
                    SpanEnd = pi;
                    break;
                }
            }

            SE.BridgeSpanLength   = FVector::Dist(RE.WorldPoints[SpanStart],
                                                   RE.WorldPoints[SpanEnd]);
            SE.LeftBankElevation  = RE.WorldPoints[SpanStart].Z;
            SE.RightBankElevation = RE.WorldPoints[SpanEnd].Z;
        }

        LayoutStreetEdges.Add(SE);
    }

    UE_LOG(LogTemp, Log,
           TEXT("[MTG] BuildStreetBoundary: %d nodes, %d edges (%d bridges, %d quay)"),
           LayoutStreetNodes.Num(), LayoutStreetEdges.Num(),
           LayoutStreetEdges.FilterByPredicate([](const FMedievalStreetEdge& E){ return E.bIsBridge; }).Num(),
           LayoutStreetEdges.FilterByPredicate([](const FMedievalStreetEdge& E){ return E.bIsQuayStreet; }).Num());
}

// =============================================================================
//  PARCEL BOUNDARY
// =============================================================================

namespace
{
    // EDistrictType (actor header) → EMedievalWardType (shared data header)
    static EMedievalWardType DistrictToWard(EDistrictType D)
    {
        switch (D)
        {
        case EDistrictType::InnerWard:        return EMedievalWardType::NobleWard;
        case EDistrictType::MerchantQuarter:  return EMedievalWardType::MarketWard;
        case EDistrictType::CraftQuarter:     return EMedievalWardType::CraftsWard;
        case EDistrictType::GateWard:         return EMedievalWardType::MarketWard;
        case EDistrictType::OuterResidential: return EMedievalWardType::ResidentialWard;
        case EDistrictType::Slums:            return EMedievalWardType::PoorWard;
        case EDistrictType::TransitionZone:   return EMedievalWardType::Outskirts;
        case EDistrictType::Plaza:            return EMedievalWardType::MarketWard;
        default:                              return EMedievalWardType::ResidentialWard;
        }
    }

    // EBuildingStyle → EMedievalBuildingStyle (same ordinal values; explicit for safety)
    static EMedievalBuildingStyle BuildStyleToMedieval(EBuildingStyle S)
    {
        switch (S)
        {
        case EBuildingStyle::SmallCottage: return EMedievalBuildingStyle::SmallCottage;
        case EBuildingStyle::TownHouse:    return EMedievalBuildingStyle::TownHouse;
        case EBuildingStyle::GuildHall:    return EMedievalBuildingStyle::GuildHall;
        case EBuildingStyle::TavernInn:    return EMedievalBuildingStyle::TavernInn;
        case EBuildingStyle::Church:       return EMedievalBuildingStyle::Church;
        case EBuildingStyle::Keep:         return EMedievalBuildingStyle::Keep;
        case EBuildingStyle::Stable:       return EMedievalBuildingStyle::Stable;
        case EBuildingStyle::Warehouse:    return EMedievalBuildingStyle::Warehouse;
        case EBuildingStyle::Blacksmith:   return EMedievalBuildingStyle::Blacksmith;
        case EBuildingStyle::Bakery:       return EMedievalBuildingStyle::Bakery;
        default:                           return EMedievalBuildingStyle::TownHouse;
        }
    }

    // ERoofType → EMedievalRoofType (same ordinal values; explicit for safety)
    static EMedievalRoofType RoofToMedieval(ERoofType R)
    {
        switch (R)
        {
        case ERoofType::Pitched:     return EMedievalRoofType::Pitched;
        case ERoofType::Hipped:      return EMedievalRoofType::Hipped;
        case ERoofType::Gambrel:     return EMedievalRoofType::Gambrel;
        case ERoofType::FlatParapet: return EMedievalRoofType::FlatParapet;
        case ERoofType::Conical:     return EMedievalRoofType::Conical;
        case ERoofType::Pyramidal:   return EMedievalRoofType::Pyramidal;
        case ERoofType::Thatched:    return EMedievalRoofType::Thatched;
        default:                     return EMedievalRoofType::Pitched;
        }
    }

    // Derives the footprint archetype from building programme and wing flag.
    // bIsCorner is applied separately in the second pass.
    static EMedievalFootprintType DeriveFootprint(EBuildingStyle S, bool bHasWing)
    {
        if (bHasWing) return EMedievalFootprintType::CourtyardHouse;

        switch (S)
        {
        case EBuildingStyle::SmallCottage: return EMedievalFootprintType::Hut;
        case EBuildingStyle::TownHouse:    return EMedievalFootprintType::Rowhouse;
        case EBuildingStyle::GuildHall:    return EMedievalFootprintType::HallHouse;
        case EBuildingStyle::TavernInn:    return EMedievalFootprintType::Shopfront;
        case EBuildingStyle::Church:       return EMedievalFootprintType::HallHouse;
        case EBuildingStyle::Keep:         return EMedievalFootprintType::CourtyardHouse;
        case EBuildingStyle::Stable:       return EMedievalFootprintType::Barn;
        case EBuildingStyle::Warehouse:    return EMedievalFootprintType::Barn;
        case EBuildingStyle::Blacksmith:   return EMedievalFootprintType::Workshop;
        case EBuildingStyle::Bakery:       return EMedievalFootprintType::Workshop;
        default:                           return EMedievalFootprintType::Rowhouse;
        }
    }

    // Wealth bias driven by district; combined with market proximity for WealthScore.
    static float DistrictWealthBias(EDistrictType D)
    {
        switch (D)
        {
        case EDistrictType::InnerWard:        return 0.90f;
        case EDistrictType::Plaza:            return 0.80f;
        case EDistrictType::MerchantQuarter:  return 0.75f;
        case EDistrictType::GateWard:         return 0.60f;
        case EDistrictType::CraftQuarter:     return 0.45f;
        case EDistrictType::OuterResidential: return 0.35f;
        case EDistrictType::TransitionZone:   return 0.20f;
        case EDistrictType::Slums:            return 0.10f;
        default:                              return 0.40f;
        }
    }
}

void AMedievalTownGenerator::BuildParcelBoundary()
{
    LayoutParcels.Empty();
    LayoutParcels.Reserve(PlacedLots.Num());

    const FVector Origin = GetActorLocation();

    // ----- Pass 1: populate all fields except PartyWallMask ------------------

    for (int32 LotIdx = 0; LotIdx < PlacedLots.Num(); LotIdx++)
    {
        const FBuildingLot& Lot = PlacedLots[LotIdx];
        if (!Lot.bIsPlaced) continue;

        FMedievalParcel P;
        P.LotIndex = LayoutParcels.Num();

        // ---- Geometry -------------------------------------------------------

        P.WorldCenter    = Lot.Center;
        P.FrontageWidth  = Lot.Footprint.X;   // perpendicular to StreetFacingDir
        P.LotDepth       = Lot.Footprint.Y;   // parallel to StreetFacingDir
        P.Yaw            = Lot.Yaw;
        P.GroundElevation = Lot.Center.Z;

        // Lot.Yaw is atan2(FacingDir.Y, FacingDir.X) where FacingDir points
        // from the lot toward its road (set verbatim by PlaceBuildings).
        const float YawRad = FMath::DegreesToRadians(Lot.Yaw);
        P.StreetFacingDir = FVector2D(FMath::Cos(YawRad), FMath::Sin(YawRad));

        // Polygon: 4 corners, front edge first.
        // Fwd = toward road, Rgt = 90° CW of Fwd (right side when facing road).
        const FVector2D Center2D(Lot.Center.X, Lot.Center.Y);
        const FVector2D Fwd = P.StreetFacingDir;
        const FVector2D Rgt(Fwd.Y, -Fwd.X);
        const float HalfW = P.FrontageWidth * 0.5f;
        const float HalfD = P.LotDepth      * 0.5f;

        P.LotPolygon = {
            Center2D + Fwd * HalfD - Rgt * HalfW,   // front-left
            Center2D + Fwd * HalfD + Rgt * HalfW,   // front-right
            Center2D - Fwd * HalfD + Rgt * HalfW,   // back-right
            Center2D - Fwd * HalfD - Rgt * HalfW    // back-left
        };

        // ---- Terrain --------------------------------------------------------

        // 4×4 grid sample across footprint to get max slope and foundation depth.
        const FVector2D Local2D(Lot.Center.X - Origin.X, Lot.Center.Y - Origin.Y);
        float MinH =  1e9f, MaxH = -1e9f;
        const int32 GridN = 3;
        for (int32 gx = 0; gx <= GridN; gx++)
        {
            for (int32 gy = 0; gy <= GridN; gy++)
            {
                const float fx = (float)gx / GridN - 0.5f;
                const float fy = (float)gy / GridN - 0.5f;
                const FVector2D SLocal = Local2D
                    + Fwd * (fy * Lot.Footprint.Y)
                    + Rgt * (fx * Lot.Footprint.X);
                const float SH = GetTerrainHeight(SLocal.X, SLocal.Y);
                MinH = FMath::Min(MinH, SH);
                MaxH = FMath::Max(MaxH, SH);
            }
        }
        // Foundation depth = base height + extra to level the building on sloped terrain.
        P.FoundationDepth = FoundationHeight + FMath::Max(0.f, MaxH - MinH);

        // Slope from terrain normal at lot center.
        const FTerrainSample TS = SampleTerrain(Local2D.X, Local2D.Y);
        P.GroundSlopeDeg = FMath::RadiansToDegrees(
            FMath::Acos(FMath::Clamp(TS.Normal.Z, 0.f, 1.f)));

        // ---- District & Style -----------------------------------------------

        P.Ward          = DistrictToWard(Lot.District);
        P.BuildingStyle = BuildStyleToMedieval(Lot.Style);
        P.RoofType      = RoofToMedieval(Lot.Roof);
        P.FootprintType = DeriveFootprint(Lot.Style, Lot.bHasWing);
        P.NumFloors     = Lot.NumFloors;

        // ---- Adjacency signals ----------------------------------------------

        if (bGenerateRiver && CachedRiverPlanarPath.Num() > 0)
        {
            float RDist = 0.f, HalfRW = 0.f;
            SampleRiverClosestPoint(Local2D, RDist, HalfRW, nullptr);
            P.DistToRiver = RDist;
        }

        // Wall distance: approximate via TownRadius − radial distance.
        // Accurate for the near-circular wall; good enough for material tier selection.
        P.DistToWall   = FMath::Max(0.f, TownRadius - Local2D.Size());
        P.DistToMarket = FVector2D::Distance(Local2D, CachedMarketPos);

        // ---- Wealth score ---------------------------------------------------

        // Blend market proximity (gradient across the whole town) with per-district
        // baseline so buildings of a prestige programme in a poor district still
        // read as slightly wealthier than their neighbours.
        const float MarketProx    = FMath::Clamp(1.f - P.DistToMarket / TownRadius, 0.f, 1.f);
        const float DistrictBias  = DistrictWealthBias(Lot.District);
        P.WealthScore = FMath::Clamp(MarketProx * 0.5f + DistrictBias * 0.5f, 0.f, 1.f);

        // ---- Flags ----------------------------------------------------------

        // bFacesRiver: replicate the exact condition from PlaceBuildings.
        // (Any lot within this range had its Yaw overridden to face the river.)
        P.bFacesRiver = bGenerateRiver
            && (P.DistToRiver < RiverExclusionRadius + RiverFrontageDistance);

        // bIsCornerLot: within 1.5× frontage width of any non-terminus intersection.
        const float CornerThresh = Lot.Footprint.X * 1.5f;
        for (const FMedievalStreetNode& SN : LayoutStreetNodes)
        {
            if (SN.IntersectionType == EMedievalIntersectionType::Terminus) continue;
            const FVector2D SN2D(SN.WorldPos.X - Origin.X, SN.WorldPos.Y - Origin.Y);
            if (FVector2D::Distance(Local2D, SN2D) < CornerThresh)
            {
                P.bIsCornerLot    = true;
                P.FootprintType   = EMedievalFootprintType::CornerBuilding;
                break;
            }
        }

        // ---- Wing -----------------------------------------------------------

        P.bHasWing     = Lot.bHasWing;
        P.WingSize     = Lot.WingFootprint;
        P.WingYawOffset = Lot.WingYawOffset;

        // PartyWallMask is set in Pass 2.
        P.PartyWallMask = 0;

        LayoutParcels.Add(P);
    }

    // ----- Pass 2: party wall masks ------------------------------------------
    // Two parcels share a party wall when a neighbouring lot's center lies within
    // PartyWallDist of one of the three side-face midpoints (left, right, rear).

    const float PartyWallDist = MinBuildingSpacing * 0.5f + WallThickness * 2.f;

    for (int32 i = 0; i < LayoutParcels.Num(); i++)
    {
        FMedievalParcel& Pi = LayoutParcels[i];
        const FVector2D Ci(Pi.WorldCenter.X, Pi.WorldCenter.Y);
        const FVector2D Fi = Pi.StreetFacingDir;
        const FVector2D Ri(Fi.Y, -Fi.X);

        const FVector2D LeftMid  = Ci - Ri * Pi.FrontageWidth * 0.5f;
        const FVector2D RightMid = Ci + Ri * Pi.FrontageWidth * 0.5f;
        const FVector2D RearMid  = Ci - Fi * Pi.LotDepth       * 0.5f;

        for (int32 j = 0; j < LayoutParcels.Num(); j++)
        {
            if (j == i) continue;
            const FVector2D Cj(LayoutParcels[j].WorldCenter.X, LayoutParcels[j].WorldCenter.Y);

            if (FVector2D::Distance(Cj, LeftMid)  < PartyWallDist) Pi.PartyWallMask |= 0x01;
            if (FVector2D::Distance(Cj, RightMid) < PartyWallDist) Pi.PartyWallMask |= 0x02;
            if (FVector2D::Distance(Cj, RearMid)  < PartyWallDist) Pi.PartyWallMask |= 0x04;
        }
    }

    UE_LOG(LogTemp, Log,
           TEXT("[MTG] BuildParcelBoundary: %d parcels (%d river-facing, %d corner)"),
           LayoutParcels.Num(),
           LayoutParcels.FilterByPredicate([](const FMedievalParcel& P){ return P.bFacesRiver; }).Num(),
           LayoutParcels.FilterByPredicate([](const FMedievalParcel& P){ return P.bIsCornerLot; }).Num());
}

