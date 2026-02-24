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

void MTGRoads::ApplyOrganicGraphToTownGenerator(
    AMedievalTownGenerator* Gen,
    const FOrganicStreetGraph& Graph)
{
    Gen->RoadNodes.Empty();
    Gen->RoadEdges.Empty();

    for (const FOrganicStreetNode& ON : Graph.Nodes)
    {
        FRoadNode RN;
        RN.Pos        = ON.Position;
        RN.bIsGate    = ON.bIsGate;
        RN.bIsMarket  = ON.bIsMarket;
        RN.bIsLandmark= ON.bIsLandmark;
        RN.bIsBridgeNode = ON.bIsBridgeNode;
        RN.Index      = Gen->RoadNodes.Num();
        Gen->RoadNodes.Add(RN);
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
        // Copy 2D polyline (Z placeholder; filled by ElevateRoadSplines)
        for (const FVector2D& P : OE.Poly2D)
            RE.PolylinePoints.Add(P);
        Gen->RoadEdges.Add(RE);
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

    // -- 3. Church / keep heuristic positions ---------------------------------
    float ChurchAngle = Rand.FRandRange(20.f, 80.f);
    float ChurchR     = TownRadius * Rand.FRandRange(0.12f, 0.25f);
    FVector2D ChurchPos(FMath::Cos(FMath::DegreesToRadians(ChurchAngle)) * ChurchR,
                         FMath::Sin(FMath::DegreesToRadians(ChurchAngle)) * ChurchR);
    float KeepAngle   = ChurchAngle + Rand.FRandRange(100.f, 200.f);
    float KeepR       = TownRadius * Rand.FRandRange(0.20f, 0.38f);
    FVector2D KeepPos(FMath::Cos(FMath::DegreesToRadians(KeepAngle)) * KeepR,
                       FMath::Sin(FMath::DegreesToRadians(KeepAngle)) * KeepR);

    // -- 4. Terrain query ------------------------------------------------------
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

    // -- 5. Generator config ---------------------------------------------------
    FOrganicStreetConfig Cfg;
    Cfg.TownRadius          = TownRadius;
    Cfg.MarketCenter        = FVector2D::ZeroVector;
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

    // -- 6. Generate -----------------------------------------------------------
    FOrganicStreetGenerator Generator(Cfg, TQ, Rand);
    FOrganicStreetGraph OrganicGraph = Generator.Generate(Gate2D, BridgeCandidates, ChurchPos, KeepPos);

    // -- 7. Debug draw ---------------------------------------------------------
    if (bDebugDrawStreets)
    {
        StreetDebug::FDebugSettings DS;
        DS.ActorZ = Origin.Z;
        StreetDebug::DrawGraph(GetWorld(), OrganicGraph, DS);
        StreetDebug::DrawBridgeCandidates(GetWorld(), BridgeCandidates, Origin.Z);
    }

    // -- 8. Map graph -> RoadNodes/RoadEdges -----------------------------------
    MTGRoads::ApplyOrganicGraphToTownGenerator(this, OrganicGraph);
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
    BuildOrganicRoadNetwork();    // <- replaces BuildRadiocentricRoads()
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
