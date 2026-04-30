// Roads module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorRoads.h"
#include "MedievalTownGenerator.h"
#include "OrganicTerrainRouting.h"
#include "OrganicStreetGenerator.h"
#include "StreetDebugRenderer.h"
#include "DrawDebugHelpers.h"

namespace
{
    static float PolylineLength(const TArray<FVector2D>& Pts)
    {
        float L = 0.0f;
        for (int32 I = 1; I < Pts.Num(); ++I)
        {
            L += FVector2D::Distance(Pts[I - 1], Pts[I]);
        }
        return L;
    }

    static FVector2D Perp(const FVector2D& V)
    {
        return FVector2D(-V.Y, V.X);
    }

    static float Cross2D(const FVector2D& A, const FVector2D& B)
    {
        return A.X * B.Y - A.Y * B.X;
    }

    static bool SegmentIntersect2D(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D)
    {
        const FVector2D R = B - A;
        const FVector2D S = D - C;
        const float Den = Cross2D(R, S);
        if (FMath::IsNearlyZero(Den))
        {
            return false;
        }

        const FVector2D AC = C - A;
        const float T = Cross2D(AC, S) / Den;
        const float U = Cross2D(AC, R) / Den;
        return T > 0.001f && T < 0.999f && U > 0.001f && U < 0.999f;
    }
}

FVector2D MTGRoads::Perpendicular2D(const FVector2D& Dir)
{
    return FVector2D(-Dir.Y, Dir.X);
}

void AMedievalTownGenerator::ApplyOrganicGraphToTownGenerator(
    const FOrganicStreetGraph& Graph)
{
    RoadNodes.Empty();
    RoadEdges.Empty();

    for (const FOrganicStreetNode& ON : Graph.Nodes)
    {
        FRoadNode RN;
        RN.Pos        = ON.Position;
        RN.bIsGate    = ON.bIsGate;
        RN.bIsMarket  = ON.bIsMarket;
        RN.bIsLandmark= ON.bIsLandmark;
        RN.bIsBridgeNode = ON.bIsBridgeNode;
        RN.Index      = RoadNodes.Num();
        RoadNodes.Add(RN);
    }

    for (const FOrganicStreetEdge& OE : Graph.Edges)
    {
        if (OE.NodeA < 0 || OE.NodeB < 0) continue;  // tombstone
        FRoadEdge RE;
        RE.NodeA        = OE.NodeA;
        RE.NodeB        = OE.NodeB;
        RE.bIsGenerated = true;
        RE.bIsBridge    = OE.bIsBridge;
        switch (OE.StreetType)
        {
        case EOrganicStreetType::Primary:   RE.Tier = EStreetTier::Primary;   break;
        case EOrganicStreetType::Secondary: RE.Tier = EStreetTier::Secondary; break;
        default:                            RE.Tier = EStreetTier::Tertiary;  break;
        }
        RE.Width = OE.Width;
        for (const FVector2D& P : OE.Poly2D)
            RE.PolylinePoints.Add(P);
        RoadEdges.Add(RE);
    }
}

void AMedievalTownGenerator::BuildOrganicRoadNetwork()
{
    const FVector Origin = GetActorLocation();

    // -- 1. Gate positions ----------------------------------------------------
    TArray<FVector2D> Gate2D;
    if (GatePositions.Num() > 0)
    {
        for (const FVector& GP : GatePositions)
            Gate2D.Add(FVector2D(GP.X - Origin.X, GP.Y - Origin.Y));
    }
    else
    {
        const int32 NumG    = FMath::Clamp(NumGates, 2, 8);
        float BaseAngle     = Rand.FRandRange(0.f, 360.f);
        float AngleStep     = 360.f / NumG;
        for (int32 g = 0; g < NumG; g++)
        {
            float Angle = BaseAngle + g * AngleStep
                        + Rand.FRandRange(-AngleStep * 0.3f, AngleStep * 0.3f);
            float R     = TownRadius * Rand.FRandRange(0.85f, 0.95f);
            float Rad   = FMath::DegreesToRadians(Angle);
            Gate2D.Add(FVector2D(FMath::Cos(Rad) * R, FMath::Sin(Rad) * R));
        }
    }

    // -- 2. Bridge candidates from river data ---------------------------------
    TArray<FBridgeCandidate> BridgeCandidates;
    if (bGenerateRiver && CachedRiverPlanarPath.Num() >= 4)
    {
        const int32 NumSegs   = CachedRiverPlanarPath.Num() - 1;
        const int32 Stride    = FMath::Max(1, NumSegs / 12);
        for (int32 i = Stride; i < NumSegs - Stride; i += Stride)
        {
            FVector2D Pos = (CachedRiverPlanarPath[i] + CachedRiverPlanarPath[i+1]) * 0.5f;
            if (Pos.Size() > TownRadius * 0.85f) continue;

            FVector2D FlowDir = (CachedRiverPlanarPath[i+1] - CachedRiverPlanarPath[i]).GetSafeNormal();
            FVector2D CrossDir(-FlowDir.Y, FlowDir.X);

            float H_L = GetTerrainHeightNoRiver(Pos.X - CrossDir.X * RiverWidth, Pos.Y - CrossDir.Y * RiverWidth);
            float H_R = GetTerrainHeightNoRiver(Pos.X + CrossDir.X * RiverWidth, Pos.Y + CrossDir.Y * RiverWidth);
            float Curv = 0.f;
            if (i > 0 && i < NumSegs - 1)
            {
                FVector2D D0 = (CachedRiverPlanarPath[i]   - CachedRiverPlanarPath[i-1]).GetSafeNormal();
                FVector2D D1 = (CachedRiverPlanarPath[i+1] - CachedRiverPlanarPath[i]).GetSafeNormal();
                Curv = (1.f - FMath::Clamp(FVector2D::DotProduct(D0,D1), -1.f, 1.f)) * 0.5f;
            }
            float SlopePen = FMath::Abs(H_L - H_R) / (RiverWidth * 2.f);
            FBridgeCandidate BC;
            BC.Position    = Pos;
            BC.Quality     = FMath::Clamp(1.f - SlopePen * 3.f - Curv * 2.f, 0.f, 1.f);
            BC.ApproachDir = CrossDir;
            BridgeCandidates.Add(BC);
        }
    }

    // -- 3. Market center: anchor to best bridge when river spine is enabled -----
    {
        FVector2D BestBridgePos = FVector2D::ZeroVector;
        if (bRiverSpineEnabled && BridgeCandidates.Num() > 0)
        {
            int32 BestIdx = 0;
            float BestQ   = -1.f;
            for (int32 i = 0; i < BridgeCandidates.Num(); i++)
            {
                if (BridgeCandidates[i].Quality > BestQ)
                { BestQ = BridgeCandidates[i].Quality; BestIdx = i; }
            }
            BestBridgePos = BridgeCandidates[BestIdx].Position;
        }
        CachedMarketPos = BestBridgePos;
    }

    // -- 4. Church at river bend; keep commanding bridge approach ----------------
    FVector2D ChurchPos;
    FVector2D KeepPos;

    // Church: find the highest-curvature bend inside the town
    {
        float BestCurv = -1.f;
        float FallbackAngle = Rand.FRandRange(20.f, 80.f);
        ChurchPos = FVector2D(FMath::Cos(FMath::DegreesToRadians(FallbackAngle)),
                              FMath::Sin(FMath::DegreesToRadians(FallbackAngle)))
                    * TownRadius * Rand.FRandRange(0.12f, 0.25f);

        if (bGenerateRiver && CachedRiverPlanarPath.Num() >= 3)
        {
            for (int32 i = 1; i < CachedRiverPlanarPath.Num() - 1; i++)
            {
                // Skip points outside the inner town area
                if (CachedRiverPlanarPath[i].Size() > TownRadius * 0.75f) continue;

                FVector2D D0 = (CachedRiverPlanarPath[i]   - CachedRiverPlanarPath[i - 1]).GetSafeNormal();
                FVector2D D1 = (CachedRiverPlanarPath[i + 1] - CachedRiverPlanarPath[i]).GetSafeNormal();
                float Curv   = 1.f - FMath::Clamp(FVector2D::DotProduct(D0, D1), -1.f, 1.f);

                if (Curv > BestCurv)
                {
                    BestCurv = Curv;
                    // Place church on bank beside the bend, not in water
                    FVector2D BendPt  = CachedRiverPlanarPath[i];
                    FVector2D Outward = BendPt.GetSafeNormal();
                    FVector2D Cand    = BendPt + Outward * (RiverExclusionRadius + 250.f);
                    // Clamp to inner town
                    if (Cand.Size() > TownRadius * 0.55f)
                        Cand = Cand.GetSafeNormal() * TownRadius * 0.45f;
                    if (!IsNearRiver(Cand, 200.f))
                        ChurchPos = Cand;
                }
            }
        }
    }

    // Keep: commanding view of bridge approach (opposite river side from church)
    {
        float FallbackKeepAngle = FMath::RadiansToDegrees(FMath::Atan2(ChurchPos.Y, ChurchPos.X))
                                  + Rand.FRandRange(100.f, 200.f);
        KeepPos = FVector2D(FMath::Cos(FMath::DegreesToRadians(FallbackKeepAngle)),
                            FMath::Sin(FMath::DegreesToRadians(FallbackKeepAngle)))
                  * TownRadius * Rand.FRandRange(0.20f, 0.38f);

        if (bRiverSpineEnabled && CachedMarketPos != FVector2D::ZeroVector)
        {
            // Bridge crossing direction (perpendicular to river flow at bridge)
            FVector2D BridgePt = CachedMarketPos;
            // Step back from bridge along the approach direction, on the side opposite church
            FVector2D ToBridge  = BridgePt.GetSafeNormal();
            FVector2D ChurchDir = ChurchPos.GetSafeNormal();
            // If church and keep would end up on the same side, flip keep across
            float Dot = FVector2D::DotProduct(ToBridge, ChurchDir);
            FVector2D KeepDir   = (Dot > 0.f) ? -ToBridge : ToBridge;
            FVector2D Cand      = BridgePt + KeepDir * TownRadius * 0.22f;
            if (Cand.Size() > TownRadius * 0.5f)
                Cand = Cand.GetSafeNormal() * TownRadius * 0.42f;
            if (!IsNearRiver(Cand, 200.f))
                KeepPos = Cand;
        }
    }

    // -- 5. Terrain query ------------------------------------------------------
    FOrganicTerrainQuery TQ;
    TQ.GetHeight       = [this](FVector2D P) { return GetTerrainHeight(P.X, P.Y); };
    TQ.IsNearRiver     = [this](FVector2D P, float E) { return IsNearRiver(P, E); };
    TQ.BridgeSuitability = [this](FVector2D P) {
        float Dist = 0.f;
        float HalfW = RiverWidth * 0.5f;   // lvalue required — SampleRiverClosestPoint takes float&
        SampleRiverClosestPoint(P, Dist, HalfW, nullptr);
        return FMath::Clamp(1.f - Dist / (RiverWidth * 1.5f), 0.f, 1.f);
    };
    TQ.MaxGrade         = StreetGrowthData.MaxSlopeDeg / 90.f;
    TQ.SlopePenalty     = StreetGrowthData.SlopePenalty;
    TQ.WaterPenalty     = StreetGrowthData.WaterPenalty;
    TQ.ValleyPreference = StreetGrowthData.ValleyPreference;

    // -- 6. Generator config ---------------------------------------------------
    FOrganicStreetConfig Cfg;
    Cfg.TownRadius          = TownRadius;
    Cfg.MarketCenter        = CachedMarketPos;
    Cfg.PrimaryWidthMin     = PrimaryRoadWidth * 0.85f;
    Cfg.PrimaryWidthMax     = PrimaryRoadWidth * 1.15f;
    Cfg.SecondaryWidthMin   = SecondaryRoadWidth * 0.85f;
    Cfg.SecondaryWidthMax   = SecondaryRoadWidth * 1.15f;
    Cfg.LaneWidthMin        = TertiaryRoadWidth * 0.75f;
    Cfg.LaneWidthMax        = TertiaryRoadWidth * 1.25f;
    Cfg.AlleyWidthMin       = TertiaryRoadWidth * 0.40f;
    Cfg.AlleyWidthMax       = TertiaryRoadWidth * 0.70f;
    Cfg.MinSpacingCore      = StreetGrowthData.MinIntersectionSpacing * 3.8f;
    Cfg.MinSpacingOutskirts = StreetGrowthData.MinIntersectionSpacing * 6.2f;
    Cfg.CoreRadiusFraction  = InnerRingRadius;
    Cfg.SecondaryAttractors = SecondaryAttractorCount;
    Cfg.TertiaryAttractors  = FMath::RoundToInt(SecondaryAttractorCount * 1.5f);
    Cfg.MaxBridges          = bGenerateRiver
                                ? FMath::Clamp(FMath::RoundToInt(TownRadius / 12000.f) + 1, 1, 2)
                                : 0;
    Cfg.AStarCellSize       = TownRadius / 80.f;
    Cfg.RDPEpsilonPrimary   = Cfg.AStarCellSize * 0.8f;
    Cfg.RDPEpsilonSecondary = Cfg.AStarCellSize * 0.5f;
    Cfg.bDebugDraw          = bDebugDrawStreets;

    // -- 7. Generate -----------------------------------------------------------
    FOrganicStreetGenerator Generator(Cfg, TQ, Rand);
    FOrganicStreetGraph OrganicGraph = Generator.Generate(Gate2D, BridgeCandidates, ChurchPos, KeepPos);

    // -- 8. Debug draw ---------------------------------------------------------
    if (bDebugDrawStreets)
    {
        StreetDebug::FDebugSettings DS;
        DS.ActorZ = Origin.Z;
        StreetDebug::DrawGraph(GetWorld(), OrganicGraph, DS);
        StreetDebug::DrawBridgeCandidates(GetWorld(), BridgeCandidates, Origin.Z);
    }

    // -- 9. Map graph -> RoadNodes/RoadEdges -----------------------------------
    ApplyOrganicGraphToTownGenerator(OrganicGraph);
}


void AMedievalTownGenerator::ElevateRoadSplines()
{
    for (FRoadEdge& E : RoadEdges)
    {
        if (!E.bIsGenerated)
        {
            continue;
        }

        E.WorldPoints.Empty();
        TArray<FVector2D> SourcePts;
        if (E.PolylinePoints.Num() > 1)
        {
            SourcePts = E.PolylinePoints;
        }
        else
        {
            SourcePts = { RoadNodes[E.NodeA].Pos, RoadNodes[E.NodeB].Pos };
        }

        bool bTouchesRiver = false;
        for (int32 I = 1; I < SourcePts.Num(); ++I)
        {
            if (bGenerateRiver && SegmentCrossesRiver(SourcePts[I - 1], SourcePts[I]))
            {
                bTouchesRiver = true;
                break;
            }
        }
        E.bIsBridge = E.bIsBridge || bTouchesRiver;

        for (int32 Seg = 1; Seg < SourcePts.Num(); ++Seg)
        {
            const FVector2D A = SourcePts[Seg - 1];
            const FVector2D B = SourcePts[Seg];
            const int32 Subs = FMath::Max(4, RoadSplineSubdivisions + (E.Tier == EStreetTier::Primary ? 4 : 2));

            for (int32 I = 0; I <= Subs; ++I)
            {
                if (Seg > 1 && I == 0)
                {
                    continue;
                }

                const float T = (float)I / (float)Subs;
                FVector2D P = FMath::Lerp(A, B, T);

                if (!E.bIsBridge && I > 0 && I < Subs)
                {
                    const FVector2D D = (B - A).GetSafeNormal();
                    const FVector2D Side = MTGRoads::Perpendicular2D(D);
                    const float N = FMath::PerlinNoise2D(P * 0.0011f + FVector2D((float)RandomSeed * 0.005f, 0.31f));
                    P += Side * N * RoadOrganicWaver;
                }

                float H = GetTerrainHeight(P.X, P.Y) + 10.f;
                if (E.bIsBridge)
                {
                    const float Dist = DistToRiverCenter(P);
                    if (Dist < RiverWidth * 0.5f + 140.f)
                    {
                        H = GetTerrainHeightNoRiver(P.X, P.Y) + 25.f;
                    }
                }

                E.WorldPoints.Add(GetActorLocation() + FVector(P.X, P.Y, H));
            }
        }
    }
}

float AMedievalTownGenerator::RoadWidth(EStreetTier Tier) const
{
    switch (Tier)
    {
    case EStreetTier::Primary:   return PrimaryRoadWidth;
    case EStreetTier::Secondary: return SecondaryRoadWidth;
    case EStreetTier::Tertiary:  return TertiaryRoadWidth;
    case EStreetTier::RiverPath: return FMath::Max(TertiaryRoadWidth * 1.15f, 140.f);
    default:                     return SecondaryRoadWidth;
    }
}

// =============================================================================
//  QUAY STREETS  (river-parallel paths on both banks)
// =============================================================================

void AMedievalTownGenerator::BuildQuayStreets()
{
    if (CachedRiverPlanarPath.Num() < 3) return;

    const float QuayDist  = RiverExclusionRadius + QuayStreetOffset;
    const float NodeMinSep = IntersectionMinSpacing;
    const int32 Stride     = FMath::Max(2, QuayStreetSampleStep);
    const float QuayWidth  = RoadWidth(EStreetTier::RiverPath);

    // Build one quay on each bank (Side 0 = left, Side 1 = right of river flow)
    for (int32 Side = 0; Side < 2; Side++)
    {
        const float SideSign = (Side == 0) ? 1.f : -1.f;

        int32       PrevNodeIdx = -1;
        FVector2D   PrevNodePos;

        for (int32 i = Stride; i < CachedRiverPlanarPath.Num() - Stride; i += Stride)
        {
            const FVector2D& Pt = CachedRiverPlanarPath[i];

            // Tangent along river at this point
            FVector2D Tan = (CachedRiverPlanarPath[i + 1] - CachedRiverPlanarPath[i - 1]).GetSafeNormal();
            FVector2D Perp(-Tan.Y, Tan.X);  // Left-pointing perpendicular

            FVector2D QuayPos = Pt + Perp * SideSign * QuayDist;

            // Validate: inside walls with margin
            if (QuayPos.Size() > TownRadius * 0.86f) continue;
            // Validate: not in river exclusion zone
            if (IsNearRiver(QuayPos, 80.f)) continue;

            // Validate: minimum separation from all existing nodes
            bool bTooClose = false;
            for (const FRoadNode& N : RoadNodes)
            {
                if ((N.Pos - QuayPos).SizeSquared() < NodeMinSep * NodeMinSep)
                { bTooClose = true; break; }
            }
            if (bTooClose) continue;

            // Add quay node
            const int32 NewIdx = RoadNodes.Num();
            FRoadNode QuayNode(QuayPos, NewIdx);
            RoadNodes.Add(QuayNode);

            // Connect to previous quay node on the same bank
            if (PrevNodeIdx >= 0)
            {
                const float EdgeLen = (QuayPos - PrevNodePos).Size();
                // Only connect if gap is reasonable (avoid cross-river edges)
                if (EdgeLen < TownRadius * 0.35f && EdgeLen > NodeMinSep)
                {
                    const int32 EdgeIdx = RoadEdges.Num();
                    FRoadEdge QuayEdge;
                    QuayEdge.NodeA            = PrevNodeIdx;
                    QuayEdge.NodeB            = NewIdx;
                    QuayEdge.Tier             = EStreetTier::RiverPath;
                    QuayEdge.Width            = QuayWidth;
                    QuayEdge.bIsGenerated     = true;
                    QuayEdge.PolylinePoints   = { PrevNodePos, QuayPos };
                    RoadEdges.Add(QuayEdge);

                    RoadNodes[PrevNodeIdx].ConnectedEdges.Add(EdgeIdx);
                    RoadNodes[NewIdx].ConnectedEdges.Add(EdgeIdx);
                }
            }

            PrevNodeIdx = NewIdx;
            PrevNodePos = QuayPos;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] BuildQuayStreets: %d nodes, %d edges total after quay"),
           RoadNodes.Num(), RoadEdges.Num());
}

// =============================================================================
//  BRIDGE PLAZA FANS  (radial secondary streets from each bridge endpoint)
// =============================================================================

void AMedievalTownGenerator::BuildBridgePlazaFans()
{
    // Collect all unique bridge endpoint node indices
    TSet<int32> BridgeEndNodes;
    for (const FRoadEdge& E : RoadEdges)
    {
        if (E.bIsBridge)
        {
            BridgeEndNodes.Add(E.NodeA);
            BridgeEndNodes.Add(E.NodeB);
        }
    }

    if (BridgeEndNodes.Num() == 0) return;

    const float FanLen  = TownRadius * 0.07f;   // Street fan arm length
    const float MinSep  = IntersectionMinSpacing;
    const float FanW    = RoadWidth(EStreetTier::Secondary);

    for (int32 NodeIdx : BridgeEndNodes)
    {
        if (!RoadNodes.IsValidIndex(NodeIdx)) continue;
        const FRoadNode& HubNode = RoadNodes[NodeIdx];

        const int32 FanCount  = Rand.RandRange(3, 5);
        const float BaseAngle = Rand.FRandRange(0.f, 360.f);
        const float AngleStep = 360.f / FanCount;

        for (int32 f = 0; f < FanCount; f++)
        {
            const float Angle    = FMath::DegreesToRadians(BaseAngle + f * AngleStep);
            FVector2D   FanEnd   = HubNode.Pos + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * FanLen;

            // Clamp inside walls
            if (FanEnd.Size() > TownRadius * 0.84f)
                FanEnd = FanEnd.GetSafeNormal() * TownRadius * 0.84f;

            // Skip if end would be in river
            if (IsNearRiver(FanEnd, 50.f)) continue;

            // Minimum spacing from existing nodes
            bool bTooClose = false;
            for (const FRoadNode& N : RoadNodes)
            {
                if ((N.Pos - FanEnd).SizeSquared() < MinSep * MinSep)
                { bTooClose = true; break; }
            }
            if (bTooClose) continue;

            const int32 FanNodeIdx = RoadNodes.Num();
            FRoadNode   FanNode(FanEnd, FanNodeIdx);
            RoadNodes.Add(FanNode);

            const int32 FanEdgeIdx = RoadEdges.Num();
            FRoadEdge FanEdge;
            FanEdge.NodeA          = NodeIdx;
            FanEdge.NodeB          = FanNodeIdx;
            FanEdge.Tier           = EStreetTier::Secondary;
            FanEdge.Width          = FanW;
            FanEdge.bIsGenerated   = true;
            FanEdge.PolylinePoints = { HubNode.Pos, FanEnd };
            RoadEdges.Add(FanEdge);

            RoadNodes[NodeIdx].ConnectedEdges.Add(FanEdgeIdx);
            RoadNodes[FanNodeIdx].ConnectedEdges.Add(FanEdgeIdx);
        }
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] BuildBridgePlazaFans: %d bridge-end nodes fanned"),
           BridgeEndNodes.Num());
}

// =============================================================================

void AMedievalTownGenerator::BuildRoadNetwork()
{
    BuildOrganicRoadNetwork();    // <- replaces BuildRadiocentricRoads()

    if (bGenerateRiver && bRiverSpineEnabled)
        BuildQuayStreets();       // Adds quay edges on both river banks

    BuildBridgePlazaFans();       // Fans streets from each bridge endpoint

    ElevateRoadSplines();         // Elevates all edges (including new quay/fan edges)

    if (bDebugRoadGraph)
    {
        UWorld* World = GetWorld();
        if (World)
        {
            for (const FRoadNode& Node : RoadNodes)
            {
                const FVector P = GetActorLocation() + FVector(Node.Pos.X, Node.Pos.Y, GetTerrainHeight(Node.Pos.X, Node.Pos.Y) + 45.f);
                const FColor C = Node.bIsMarket ? FColor::Yellow : (Node.bIsGate ? FColor::Red : (Node.bIsBridgeNode ? FColor::Cyan : FColor::White));
                DrawDebugSphere(World, P, Node.bIsLandmark ? 100.f : 50.f, 10, C, false, 25.f, 0, 2.f);
            }

            for (const FRoadEdge& E : RoadEdges)
            {
                const FColor C = (E.Tier == EStreetTier::Primary) ? FColor(255, 180, 50) : (E.Tier == EStreetTier::Secondary ? FColor(120, 200, 255) : FColor(160, 160, 160));
                for (int32 I = 1; I < E.WorldPoints.Num(); ++I)
                {
                    DrawDebugLine(World, E.WorldPoints[I - 1], E.WorldPoints[I], E.bIsBridge ? FColor::Green : C, false, 25.f, 0, FMath::Max(2.0f, E.Width * 0.01f));
                }
            }

            if (bDebugRoadWaterZones && bGenerateRiver)
            {
                for (const FRoadNode& Node : RoadNodes)
                {
                    const bool bNearWater = DistToRiverCenter(Node.Pos) < (RiverWidth * 0.5f + 150.f);
                    if (bNearWater)
                    {
                        const FVector P = GetActorLocation() + FVector(Node.Pos.X, Node.Pos.Y, GetTerrainHeight(Node.Pos.X, Node.Pos.Y) + 100.f);
                        DrawDebugCircle(World, P, 130.f, 16, FColor::Blue, false, 25.f, 0, 2.f, FVector(1,0,0), FVector(0,1,0), false);
                    }
                }
                for (const FRoadEdge& E : RoadEdges)
                {
                    if (E.bIsBridge && E.WorldPoints.Num() > 0)
                    {
                        const FVector BP = E.WorldPoints[E.WorldPoints.Num() / 2] + FVector(0, 0, 120.f);
                        DrawDebugSphere(World, BP, 120.f, 12, FColor::Emerald, false, 25.f, 0, 4.f);
                    }
                }
            }

        }
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] BuildRoadNetwork(Organic): %d nodes, %d edges"), RoadNodes.Num(), RoadEdges.Num());
}
