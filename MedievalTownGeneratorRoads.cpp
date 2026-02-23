// Roads module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorRoads.h"
#include "MedievalTownGenerator.h"
#include "OrganicTerrainRouting.h"
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

void AMedievalTownGenerator::BuildRadiocentricRoads()
{
    RoadNodes.Empty();
    RoadEdges.Empty();

    const FVector Origin = GetActorLocation();
    const float TownR = FMath::Max(TownRadius, 2500.0f);
    const float CoreR = FMath::Clamp(CoreRadius, 1500.0f, TownR * 0.85f);
    const float MinNodeSpacing = FMath::Max(180.0f, IntersectionMinSpacing);

    // Defaults requested for medieval presets.
    const float PrimaryMaxGrade = MaxGradePrimary;
    const float SecondaryMaxGrade = MaxGradeSecondary;

    auto IsNearRiver = [&](const FVector2D& P, float Margin)
    {
        if (!bGenerateRiver || CachedRiverPlanarPath.Num() < 2)
        {
            return false;
        }
        return DistToRiverCenter(P) < (RiverWidth * 0.5f + Margin);
    };

    auto AddNode = [&](const FVector2D& Pos, bool bGate, bool bMarket, bool bBridgeNode, bool bLandmark, float Importance)
    {
        FRoadNode N;
        N.Pos = Pos;
        N.bIsGate = bGate;
        N.bIsMarket = bMarket;
        N.bIsBridgeNode = bBridgeNode;
        N.bIsLandmark = bLandmark;
        N.Importance = Importance;
        N.Index = RoadNodes.Num();
        RoadNodes.Add(N);
        return N.Index;
    };

    auto FindOrCreateNode = [&](const FVector2D& Pos, float MergeRadius, bool bGate, bool bMarket, bool bBridgeNode, bool bLandmark, float Importance)
    {
        float BestD2 = FMath::Square(MergeRadius);
        int32 BestIdx = INDEX_NONE;
        for (int32 I = 0; I < RoadNodes.Num(); ++I)
        {
            const float D2 = FVector2D::DistanceSquared(RoadNodes[I].Pos, Pos);
            if (D2 < BestD2)
            {
                BestD2 = D2;
                BestIdx = I;
            }
        }

        if (BestIdx != INDEX_NONE)
        {
            FRoadNode& N = RoadNodes[BestIdx];
            N.bIsGate |= bGate;
            N.bIsMarket |= bMarket;
            N.bIsBridgeNode |= bBridgeNode;
            N.bIsLandmark |= bLandmark;
            N.Importance = FMath::Max(N.Importance, Importance);
            return BestIdx;
        }

        return AddNode(Pos, bGate, bMarket, bBridgeNode, bLandmark, Importance);
    };

    auto WidthForTier = [&](EStreetTier Tier, const FVector2D& At)
    {
        const float CoreAlpha = FMath::Clamp(1.0f - At.Size() / FMath::Max(1.0f, CoreR), 0.0f, 1.0f);
        switch (Tier)
        {
        case EStreetTier::Primary: return FMath::Lerp(PrimaryRoadWidth * 0.9f, PrimaryRoadWidth * 1.12f, CoreAlpha);
        case EStreetTier::Secondary: return FMath::Lerp(SecondaryRoadWidth * 1.08f, SecondaryRoadWidth * 0.92f, CoreAlpha);
        case EStreetTier::Tertiary: return FMath::Lerp(TertiaryRoadWidth * 1.05f, TertiaryRoadWidth * 0.85f, CoreAlpha);
        default: return SecondaryRoadWidth;
        }
    };

    auto HasIllegalIntersection = [&](const FVector2D& A, const FVector2D& B, int32 SkipA, int32 SkipB)
    {
        for (const FRoadEdge& E : RoadEdges)
        {
            if (!RoadNodes.IsValidIndex(E.NodeA) || !RoadNodes.IsValidIndex(E.NodeB))
            {
                continue;
            }
            if (E.NodeA == SkipA || E.NodeB == SkipA || E.NodeA == SkipB || E.NodeB == SkipB)
            {
                continue;
            }

            const FVector2D C = RoadNodes[E.NodeA].Pos;
            const FVector2D D = RoadNodes[E.NodeB].Pos;
            if (SegmentIntersect2D(A, B, C, D))
            {
                return true;
            }
        }
        return false;
    };

    auto AddEdge = [&](int32 A, int32 B, EStreetTier Tier, const TArray<FVector2D>& InPolyline, bool bForceBridge)
    {
        if (A == B || !RoadNodes.IsValidIndex(A) || !RoadNodes.IsValidIndex(B))
        {
            return INDEX_NONE;
        }

        TArray<FVector2D> Polyline = InPolyline;
        if (Polyline.Num() < 2)
        {
            Polyline = { RoadNodes[A].Pos, RoadNodes[B].Pos };
        }

        if (PolylineLength(Polyline) < 140.0f)
        {
            return INDEX_NONE;
        }

        if (!bForceBridge)
        {
            for (int32 I = 1; I < Polyline.Num(); ++I)
            {
                if (IsNearRiver((Polyline[I - 1] + Polyline[I]) * 0.5f, 140.f))
                {
                    return INDEX_NONE;
                }
            }
        }

        if (HasIllegalIntersection(RoadNodes[A].Pos, RoadNodes[B].Pos, A, B))
        {
            return INDEX_NONE;
        }

        FRoadEdge E;
        E.NodeA = A;
        E.NodeB = B;
        E.Tier = Tier;
        E.bIsGenerated = true;
        E.Importance = (Tier == EStreetTier::Primary) ? 1.0f : (Tier == EStreetTier::Secondary ? 0.55f : 0.25f);
        E.Width = WidthForTier(Tier, (RoadNodes[A].Pos + RoadNodes[B].Pos) * 0.5f);
        E.TargetSpeed = (Tier == EStreetTier::Primary) ? 320.0f : (Tier == EStreetTier::Secondary ? 220.0f : 120.0f);
        E.CurvatureCost = (Polyline.Num() > 2) ? (float)Polyline.Num() * 0.08f : 0.0f;
        E.PolylinePoints = Polyline;
        E.bIsBridge = bForceBridge;
        E.SurfaceType = bForceBridge ? ERoadSurfaceType::BridgeDeck
            : (Tier == EStreetTier::Primary ? ERoadSurfaceType::MainDirtCobble
            : (Tier == EStreetTier::Secondary ? ERoadSurfaceType::MainDirtCobble : ERoadSurfaceType::AlleyDirt));

        const int32 NewEdgeIdx = RoadEdges.Num();
        RoadEdges.Add(E);
        RoadNodes[A].ConnectedEdges.Add(NewEdgeIdx);
        RoadNodes[B].ConnectedEdges.Add(NewEdgeIdx);
        return NewEdgeIdx;
    };

    auto MakeOrganicRoute = [&](const FVector2D& Start, const FVector2D& End, EStreetTier Tier, bool bAllowBridge)
    {
        FOrganicTerrainCostParams Params;
        Params.MaxGradePrimary = PrimaryMaxGrade;
        Params.MaxGradeSecondary = SecondaryMaxGrade;
        Params.SlopePenalty = 2500.0f;
        Params.WaterPenalty = 100000.0f;
        Params.ObstaclePenalty = 30000.0f;
        Params.ValleyBias = 0.15f;

        auto Height = [&](const FVector2D& P) -> float
        {
            return GetTerrainHeight(P.X, P.Y);
        };

        auto WaterForbidden = [&](const FVector2D& P) -> bool
        {
            return !bAllowBridge && IsNearRiver(P, 120.f);
        };

        TArray<FVector2D> Path = FOrganicTerrainRouter::RouteAStar(Start, End, Params, 300.0f, 14000, Height, WaterForbidden, Tier == EStreetTier::Primary);
        TArray<FVector2D> Simplified;
        FOrganicTerrainRouter::SimplifyRDP(Path, 120.f, Simplified);
        FOrganicTerrainRouter::SmoothPolyline(Simplified, 3, 0.45f);

        if (Simplified.Num() < 2)
        {
            Simplified = { Start, End };
        }

        // Medieval irregularity but deterministic by seed/noise.
        for (int32 I = 1; I < Simplified.Num() - 1; ++I)
        {
            const FVector2D Dir = (Simplified[I + 1] - Simplified[I - 1]).GetSafeNormal();
            const FVector2D Side = Perp(Dir);
            const float N = FMath::PerlinNoise2D(Simplified[I] * 0.0008f + FVector2D((float)RandomSeed * 0.011f, 2.7f));
            const float Amp = (Tier == EStreetTier::Primary) ? 130.f : 210.f;
            Simplified[I] += Side * N * Amp;
        }

        return Simplified;
    };

    // --- Stage 1: anchor construction ---
    const int32 MarketNode = AddNode(FVector2D::ZeroVector, false, true, false, true, 1.0f);
    const int32 KeepNode = AddNode(FVector2D(TownR * 0.34f, -TownR * 0.20f), false, false, false, true, 0.86f);
    const int32 ChurchNode = AddNode(FVector2D(-TownR * 0.24f, TownR * 0.19f), false, false, false, true, 0.82f);

    TArray<int32> GateNodes;
    if (GatePositions.Num() > 0)
    {
        for (const FVector& GateWorld : GatePositions)
        {
            GateNodes.Add(AddNode(FVector2D(GateWorld.X - Origin.X, GateWorld.Y - Origin.Y), true, false, false, false, 1.0f));
        }
    }
    else
    {
        const int32 NumG = FMath::Clamp(NumGates, 2, 6);
        for (int32 G = 0; G < NumG; ++G)
        {
            const float T = ((float)G + Rand.FRandRange(-0.24f, 0.24f)) / (float)NumG;
            const float A = T * TWO_PI;
            const float R = TownR * Rand.FRandRange(0.82f, 0.95f);
            GateNodes.Add(AddNode(FVector2D(FMath::Cos(A), FMath::Sin(A)) * R, true, false, false, false, 1.0f));
        }
    }

    int32 BridgeNode = INDEX_NONE;
    if (bGenerateRiver && CachedRiverPlanarPath.Num() > 6)
    {
        float BestBridgeScore = BIG_NUMBER;
        for (int32 I = 3; I < CachedRiverPlanarPath.Num() - 3; ++I)
        {
            const FVector2D P = CachedRiverPlanarPath[I];
            if (P.Size() > TownR * 0.85f)
            {
                continue;
            }

            const float H = GetTerrainHeight(P.X, P.Y);
            const float Dx = GetTerrainHeight(P.X + 250.f, P.Y) - GetTerrainHeight(P.X - 250.f, P.Y);
            const float Dy = GetTerrainHeight(P.X, P.Y + 250.f) - GetTerrainHeight(P.X, P.Y - 250.f);
            const float BankSlope = FMath::Sqrt(Dx * Dx + Dy * Dy);
            const float ToCenter = P.Size() / TownR;
            const float Score = BankSlope * 0.5f + ToCenter * 300.f + FMath::Abs(H - GetTerrainHeight(0, 0));
            if (Score < BestBridgeScore)
            {
                BestBridgeScore = Score;
                if (BridgeNode == INDEX_NONE)
                {
                    BridgeNode = AddNode(P, false, false, true, true, 0.95f);
                }
                else
                {
                    RoadNodes[BridgeNode].Pos = P;
                }
            }
        }
    }

    auto RouteAndInsert = [&](int32 NodeA, int32 NodeB, EStreetTier Tier, bool bBridgeRoute)
    {
        if (!RoadNodes.IsValidIndex(NodeA) || !RoadNodes.IsValidIndex(NodeB))
        {
            return;
        }

        TArray<FVector2D> Route = MakeOrganicRoute(RoadNodes[NodeA].Pos, RoadNodes[NodeB].Pos, Tier, bBridgeRoute);
        int32 Prev = NodeA;
        for (int32 I = 1; I < Route.Num() - 1; ++I)
        {
            const bool bCore = Route[I].Size() < CoreR;
            const float MergeRadius = bCore ? MinNodeSpacing * 0.75f : MinNodeSpacing;
            const int32 Mid = FindOrCreateNode(Route[I], MergeRadius, false, false, false, false, Tier == EStreetTier::Primary ? 0.65f : 0.35f);
            if (Mid != Prev)
            {
                TArray<FVector2D> Segment = { RoadNodes[Prev].Pos, RoadNodes[Mid].Pos };
                AddEdge(Prev, Mid, Tier, Segment, bBridgeRoute && (Prev == BridgeNode || Mid == BridgeNode));
                Prev = Mid;
            }
        }

        TArray<FVector2D> EndSegment = { RoadNodes[Prev].Pos, RoadNodes[NodeB].Pos };
        AddEdge(Prev, NodeB, Tier, EndSegment, bBridgeRoute && (Prev == BridgeNode || NodeB == BridgeNode));
    };

    // --- Stage 2: primary desire lines ---
    for (int32 Gate : GateNodes)
    {
        RouteAndInsert(Gate, MarketNode, EStreetTier::Primary, false);
    }
    RouteAndInsert(MarketNode, KeepNode, EStreetTier::Primary, false);
    RouteAndInsert(MarketNode, ChurchNode, EStreetTier::Primary, false);
    if (BridgeNode != INDEX_NONE)
    {
        RouteAndInsert(BridgeNode, MarketNode, EStreetTier::Primary, true);
    }

    // --- Stage 3: secondary accretion and sparse loops ---
    const int32 SecondarySamples = FMath::Clamp(RoadNetworkNodes * 5, 64, 240);
    for (int32 I = 0; I < SecondarySamples; ++I)
    {
        const FVector2D Candidate = RandInsideCircle(TownR * Rand.FRandRange(0.22f, 0.92f));
        const float CoreAlpha = FMath::Clamp(1.0f - Candidate.Size() / CoreR, 0.0f, 1.0f);
        const float Accept = 0.16f + CoreAlpha * 0.62f;
        if (Rand.FRand() > Accept || IsNearRiver(Candidate, 150.f))
        {
            continue;
        }

        int32 NearestNode = INDEX_NONE;
        float BestD2 = BIG_NUMBER;
        for (const FRoadNode& N : RoadNodes)
        {
            const float D2 = FVector2D::DistanceSquared(N.Pos, Candidate);
            if (D2 < BestD2)
            {
                BestD2 = D2;
                NearestNode = N.Index;
            }
        }

        if (NearestNode == INDEX_NONE || BestD2 < FMath::Square(MinNodeSpacing))
        {
            continue;
        }

        const int32 NewNode = FindOrCreateNode(Candidate, MinNodeSpacing * 0.8f, false, false, false, false, 0.4f);
        if (NewNode != NearestNode)
        {
            AddEdge(NearestNode, NewNode, EStreetTier::Secondary, { RoadNodes[NearestNode].Pos, RoadNodes[NewNode].Pos }, false);
        }

        const float LoopChance = (CoreAlpha > 0.5f) ? 0.10f : 0.03f;
        if (Rand.FRand() < LoopChance)
        {
            int32 LoopNode = INDEX_NONE;
            float LoopD2 = BIG_NUMBER;
            for (const FRoadNode& N : RoadNodes)
            {
                if (N.Index == NewNode || N.Index == NearestNode)
                {
                    continue;
                }
                const float D2 = FVector2D::DistanceSquared(N.Pos, Candidate);
                if (D2 > FMath::Square(MinNodeSpacing * 1.2f) && D2 < FMath::Square(2600.f) && D2 < LoopD2)
                {
                    LoopD2 = D2;
                    LoopNode = N.Index;
                }
            }
            if (LoopNode != INDEX_NONE)
            {
                AddEdge(NewNode, LoopNode, EStreetTier::Secondary, { RoadNodes[NewNode].Pos, RoadNodes[LoopNode].Pos }, false);
            }
        }
    }

    // --- Stage 4: tertiary lanes / alleys ---
    const int32 LaneAttempts = FMath::Clamp(RoadNetworkNodes * 4, 40, 160);
    for (int32 I = 0; I < LaneAttempts; ++I)
    {
        if (RoadNodes.Num() == 0)
        {
            break;
        }

        const int32 BaseIdx = Rand.RandRange(0, RoadNodes.Num() - 1);
        const FRoadNode& Base = RoadNodes[BaseIdx];
        const float CoreAlpha = FMath::Clamp(1.0f - Base.Pos.Size() / CoreR, 0.0f, 1.0f);
        if (CoreAlpha < 0.15f && Rand.FRand() < 0.45f)
        {
            continue;
        }

        const FVector2D Dir = (Base.Pos.IsNearlyZero() ? FVector2D(1, 0) : Base.Pos.GetSafeNormal());
        const FVector2D Offset = Perp(Dir) * Rand.FRandRange(-580.f, 580.f) + Dir * Rand.FRandRange(220.f, 900.f);
        const FVector2D Candidate = Base.Pos + Offset;

        if (Candidate.Size() > TownR * 0.98f || IsNearRiver(Candidate, 130.f))
        {
            continue;
        }

        const int32 NewNode = FindOrCreateNode(Candidate, MinNodeSpacing * 0.6f, false, false, false, false, 0.18f);
        if (NewNode != BaseIdx)
        {
            AddEdge(BaseIdx, NewNode, EStreetTier::Tertiary, { Base.Pos, RoadNodes[NewNode].Pos }, false);
        }
    }

    // Nudge perfect 4-way intersections to medieval offset junctions.
    for (FRoadNode& N : RoadNodes)
    {
        if (N.ConnectedEdges.Num() >= 4)
        {
            N.Pos += Perp(N.Pos.GetSafeNormal()) * Rand.FRandRange(60.f, 180.f);
        }
    }
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

void AMedievalTownGenerator::BuildRoadNetwork()
{
    BuildRadiocentricRoads();
    ElevateRoadSplines();

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
