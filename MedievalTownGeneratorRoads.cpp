// Roads module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorRoads.h"
#include "MedievalTownGenerator.h"

// ─────────────────────────────────────────────────────────────────────────────
//  §5  ROAD NETWORK  (Radioconcentric: radial spokes + ring roads)
// ─────────────────────────────────────────────────────────────────────────────

FVector2D MTGRoads::Perpendicular2D(const FVector2D& Dir)
{
    return FVector2D(-Dir.Y, Dir.X);
}

//
//  Medieval towns typically had a radioconcentric layout:
//    • Central market plaza
//    • Radial roads from market to each gate
//    • 1-2 concentric ring roads connecting the radials
//    • Buildings placed in the blocks formed between roads
//
//  This function creates a structured road network:
//    1. Radial spokes: center → inner ring → outer ring → gate
//    2. Inner ring: arc segments connecting adjacent radial intersections
//    3. Outer ring: arc segments connecting adjacent radial intersections
//    4. Mid-block connectors: short streets between rings for smaller blocks

void AMedievalTownGenerator::BuildRadiocentricRoads()
{
    RoadNodes.Empty();
    RoadEdges.Empty();

    // ── 1. Get gate positions from walls (already built) ───────────────────
    //    If walls produced GatePositions, use those exact positions so roads
    //    align with wall gates. Otherwise fall back to evenly-spaced placement.

    struct FGateInfo
    {
        FVector2D Pos;
        float Angle;
        int32 NodeIndex;
    };
    TArray<FGateInfo> Gates;

    if (GatePositions.Num() > 0)
    {
        // Walls already placed — extract 2D local positions from world-space gates
        FVector Origin = GetActorLocation();
        for (const FVector& GateWorld : GatePositions)
        {
            FVector2D GateLocal(GateWorld.X - Origin.X, GateWorld.Y - Origin.Y);
            FGateInfo GI;
            GI.Pos = GateLocal;
            GI.Angle = FMath::RadiansToDegrees(FMath::Atan2(GateLocal.Y, GateLocal.X));
            if (GI.Angle < 0.f) GI.Angle += 360.f;
            GI.NodeIndex = -1;
            Gates.Add(GI);
        }
    }
    else
    {
        // Fallback: evenly-spaced gates
        const int32 NumG = FMath::Clamp(NumGates, 2, 8);
        for (int32 G = 0; G < NumG; G++)
        {
            float Angle = (float)G / NumG * 360.f;
            float Jitter = Rand.FRandRange(-8.f, 8.f);
            float Rad = FMath::DegreesToRadians(Angle + Jitter);
            FVector2D GatePos = FVector2D(FMath::Cos(Rad), FMath::Sin(Rad)) * TownRadius * 0.92f;

            FGateInfo GI;
            GI.Pos = GatePos;
            GI.Angle = Angle + Jitter;
            if (GI.Angle < 0.f) GI.Angle += 360.f;
            GI.NodeIndex = -1;
            Gates.Add(GI);
        }
    }

    const int32 NumG = Gates.Num();

    // Sort by angle for ring connectivity
    Gates.Sort([](const FGateInfo& A, const FGateInfo& B){ return A.Angle < B.Angle; });

    // ── 2. Create nodes ────────────────────────────────────────────────────

    // Helper to add a node
    auto AddNode = [this](FVector2D Pos, bool bMarket = false, bool bGate = false) -> int32
    {
        FRoadNode N;
        N.Pos = Pos;
        N.bIsMarket = bMarket;
        N.bIsGate = bGate;
        N.Index = RoadNodes.Num();
        RoadNodes.Add(N);
        return N.Index;
    };

    // Helper to add an edge
    auto AddEdge = [this](int32 A, int32 B, EStreetTier Tier) -> int32
    {
        FRoadEdge E;
        E.NodeA = A;
        E.NodeB = B;
        E.Tier = Tier;
        E.bIsGenerated = true;
        switch (Tier)
        {
        case EStreetTier::Primary:   E.Width = PrimaryRoadWidth; break;
        case EStreetTier::Secondary: E.Width = SecondaryRoadWidth; break;
        case EStreetTier::Tertiary:  E.Width = TertiaryRoadWidth; break;
        case EStreetTier::RiverPath: E.Width = FMath::Max(TertiaryRoadWidth * 1.15f, 140.f); break;
        default:                     E.Width = SecondaryRoadWidth; break;
        }
        int32 Idx = RoadEdges.Num();
        RoadEdges.Add(E);
        return Idx;
    };

    // Node 0: Market center
    int32 CenterNode = AddNode(FVector2D(0.f, 0.f), true);

    // Gate nodes — if GatePositions was empty (no walls), populate it now
    bool bNeedGatePositions = (GatePositions.Num() == 0);
    for (FGateInfo& GI : Gates)
    {
        GI.NodeIndex = AddNode(GI.Pos, false, true);
        if (bNeedGatePositions)
        {
            float H = GetTerrainHeight(GI.Pos.X, GI.Pos.Y);
            GatePositions.Add(GetActorLocation() + FVector(GI.Pos.X, GI.Pos.Y, H));
        }
    }

    // Ring intersection nodes (where each radial meets each ring)
    float Ring1R = InnerRingRadius * TownRadius;
    float Ring2R = OuterRingRadius * TownRadius;

    // For each gate, store ring1 and ring2 intersection node indices
    TArray<int32> Ring1Nodes, Ring2Nodes;

    for (const FGateInfo& GI : Gates)
    {
        FVector2D Dir = GI.Pos.GetSafeNormal();
        int32 R1Node = AddNode(Dir * Ring1R);
        int32 R2Node = AddNode(Dir * Ring2R);
        Ring1Nodes.Add(R1Node);
        Ring2Nodes.Add(R2Node);
    }

    // ── 3. Create radial edges (spokes) ────────────────────────────────────
    //    center → ring1 → ring2 → gate  for each gate

    for (int32 G = 0; G < NumG; G++)
    {
        AddEdge(CenterNode, Ring1Nodes[G], EStreetTier::Primary);
        AddEdge(Ring1Nodes[G], Ring2Nodes[G], EStreetTier::Primary);
        AddEdge(Ring2Nodes[G], Gates[G].NodeIndex, EStreetTier::Primary);
    }

    // ── 4. Create ring road arcs ───────────────────────────────────────────
    //    For each pair of adjacent gates, add arc nodes + edges along the ring circle.
    //    RingArcSubdivisions intermediate nodes per segment give smooth arcs.

    int32 ArcSubs = FMath::Clamp(RingArcSubdivisions, 1, 4);

    // Ring 1 (inner) — Secondary roads
    for (int32 G = 0; G < NumG; G++)
    {
        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);

        // Handle wrap-around (e.g. 350° to 10°)
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        // Create intermediate arc nodes
        TArray<int32> ArcChain;
        ArcChain.Add(Ring1Nodes[G]);

        for (int32 S = 1; S <= ArcSubs; S++)
        {
            float T = (float)S / (ArcSubs + 1);
            float A = FMath::Lerp(Angle1, Angle2, T);
            FVector2D ArcPt(FMath::Cos(A) * Ring1R, FMath::Sin(A) * Ring1R);

            // Skip if too close to river
            if (!IsNearRiver(ArcPt, 400.f))
                ArcChain.Add(AddNode(ArcPt));
        }
        ArcChain.Add(Ring1Nodes[Next]);

        // Connect the chain with edges
        for (int32 i = 0; i < ArcChain.Num() - 1; i++)
        {
            FVector2D PA2 = RoadNodes[ArcChain[i]].Pos;
            FVector2D PB2 = RoadNodes[ArcChain[i + 1]].Pos;
            FVector2D Mid2 = (PA2 + PB2) * 0.5f;
            if (!IsNearRiver(Mid2, 0.f))
                AddEdge(ArcChain[i], ArcChain[i + 1], EStreetTier::Secondary);
        }
    }

    // Ring 2 (outer) — Tertiary roads
    for (int32 G = 0; G < NumG; G++)
    {
        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        TArray<int32> ArcChain;
        ArcChain.Add(Ring2Nodes[G]);

        for (int32 S = 1; S <= ArcSubs; S++)
        {
            float T = (float)S / (ArcSubs + 1);
            float A = FMath::Lerp(Angle1, Angle2, T);
            FVector2D ArcPt(FMath::Cos(A) * Ring2R, FMath::Sin(A) * Ring2R);

            if (!IsNearRiver(ArcPt, 400.f))
                ArcChain.Add(AddNode(ArcPt));
        }
        ArcChain.Add(Ring2Nodes[Next]);

        for (int32 i = 0; i < ArcChain.Num() - 1; i++)
        {
            FVector2D PA2 = RoadNodes[ArcChain[i]].Pos;
            FVector2D PB2 = RoadNodes[ArcChain[i + 1]].Pos;
            FVector2D Mid2 = (PA2 + PB2) * 0.5f;
            if (!IsNearRiver(Mid2, 0.f))
                AddEdge(ArcChain[i], ArcChain[i + 1], EStreetTier::Tertiary);
        }
    }

    // ── 5. Mid-block connector streets ─────────────────────────────────────
    //    Between each pair of adjacent radials, at the midpoint angle,
    //    add a short connector road from ring1 to ring2.
    //    This creates smaller blocks for denser building placement.

    for (int32 G = 0; G < NumG; G++)
    {
        if (Rand.FRand() > MidBlockConnectorChance) continue;

        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        float MidAngle = (Angle1 + Angle2) * 0.5f;
        FVector2D Dir(FMath::Cos(MidAngle), FMath::Sin(MidAngle));

        FVector2D ConnR1 = Dir * Ring1R;
        FVector2D ConnR2 = Dir * Ring2R;

        // Skip if connector crosses river
        FVector2D ConnMid = (ConnR1 + ConnR2) * 0.5f;
        if (IsNearRiver(ConnMid, 300.f)) continue;

        int32 N1 = AddNode(ConnR1);
        int32 N2 = AddNode(ConnR2);
        AddEdge(N1, N2, EStreetTier::Tertiary);

        // Connect the inner node to nearest ring1 arc nodes
        float BestDist1a = 1e9f, BestDist1b = 1e9f;
        int32 BestNode1a = -1, BestNode1b = -1;
        float BestDist2a = 1e9f, BestDist2b = 1e9f;
        int32 BestNode2a = -1, BestNode2b = -1;

        for (int32 i = 0; i < RoadNodes.Num(); i++)
        {
            if (i == N1 || i == N2) continue;
            float NodeR = RoadNodes[i].Pos.Size();
            float D1 = (RoadNodes[i].Pos - ConnR1).Size();
            float D2 = (RoadNodes[i].Pos - ConnR2).Size();

            // Ring 1 neighbors (for N1)
            if (FMath::Abs(NodeR - Ring1R) < Ring1R * 0.15f)
            {
                if (D1 < BestDist1a) { BestDist1b = BestDist1a; BestNode1b = BestNode1a; BestDist1a = D1; BestNode1a = i; }
                else if (D1 < BestDist1b) { BestDist1b = D1; BestNode1b = i; }
            }
            // Ring 2 neighbors (for N2)
            if (FMath::Abs(NodeR - Ring2R) < Ring2R * 0.15f)
            {
                if (D2 < BestDist2a) { BestDist2b = BestDist2a; BestNode2b = BestNode2a; BestDist2a = D2; BestNode2a = i; }
                else if (D2 < BestDist2b) { BestDist2b = D2; BestNode2b = i; }
            }
        }

        // Connect to 2 nearest ring nodes on each ring (creates T-intersections)
        if (BestNode1a >= 0 && BestDist1a < Ring1R * 0.7f)
        {
            FVector2D M = (RoadNodes[BestNode1a].Pos + ConnR1) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode1a, N1, EStreetTier::Tertiary);
        }
        if (BestNode1b >= 0 && BestDist1b < Ring1R * 0.7f)
        {
            FVector2D M = (RoadNodes[BestNode1b].Pos + ConnR1) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode1b, N1, EStreetTier::Tertiary);
        }
        if (BestNode2a >= 0 && BestDist2a < Ring2R * 0.5f)
        {
            FVector2D M = (RoadNodes[BestNode2a].Pos + ConnR2) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode2a, N2, EStreetTier::Tertiary);
        }
        if (BestNode2b >= 0 && BestDist2b < Ring2R * 0.5f)
        {
            FVector2D M = (RoadNodes[BestNode2b].Pos + ConnR2) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode2b, N2, EStreetTier::Tertiary);
        }
    }

    // ── 6. Riverfront access paths (city-river integration) ──────────────────
    // Build short paths parallel to river banks where the river passes through
    // the urban footprint. This follows common historic town form: quays/pathways
    // along water with periodic connectors to street network.
    if (River.Waypoints.Num() >= 2)
    {
        TArray<int32> LeftBankNodes;
        TArray<int32> RightBankNodes;

        const int32 SamplesPerSeg = FMath::Clamp(RiverSamplesPerSegment / 2, 3, 10);
        for (int32 Seg = 0; Seg < River.Waypoints.Num() - 1; Seg++)
        {
            const FVector2D A = River.Waypoints[Seg];
            const FVector2D B = River.Waypoints[Seg + 1];
            const FVector2D Dir = (B - A).GetSafeNormal();
            const FVector2D Right = MTGRoads::Perpendicular2D(Dir);

            for (int32 S = 0; S <= SamplesPerSeg; S++)
            {
                const float T = (float)S / SamplesPerSeg;
                const FVector2D C = FMath::Lerp(A, B, T);

                float Dist = BIG_NUMBER;
                float HalfW = RiverWidth * 0.5f;
                if (!SampleRiverClosestPoint(C, Dist, HalfW, nullptr)) continue;

                // Keep waterfront paths inside city and out of the actual channel.
                if (C.Size() > TownRadius * 0.9f) continue;
                const float Offset = HalfW + RiverBuildingBuffer * 0.45f;

                const FVector2D Lp = C - Right * Offset;
                const FVector2D Rp = C + Right * Offset;

                if (Lp.Size() < TownRadius * 0.9f && !IsNearRiver(Lp, -HalfW * 0.2f))
                    LeftBankNodes.Add(AddNode(Lp));
                if (Rp.Size() < TownRadius * 0.9f && !IsNearRiver(Rp, -HalfW * 0.2f))
                    RightBankNodes.Add(AddNode(Rp));
            }
        }

        auto ConnectBankChain = [&](const TArray<int32>& Chain)
        {
            for (int32 i = 0; i < Chain.Num() - 1; i++)
            {
                const FVector2D PA2 = RoadNodes[Chain[i]].Pos;
                const FVector2D PB2 = RoadNodes[Chain[i + 1]].Pos;
                if ((PB2 - PA2).SizeSquared() < FMath::Square(TownRadius * 0.12f))
                    AddEdge(Chain[i], Chain[i + 1], EStreetTier::RiverPath);
            }

            // Connect every ~6th path node to nearest non-river road node.
            for (int32 i = 0; i < Chain.Num(); i += 6)
            {
                const FVector2D P = RoadNodes[Chain[i]].Pos;
                float BestD = BIG_NUMBER;
                int32 BestIdx = -1;

                for (int32 N = 0; N < RoadNodes.Num(); N++)
                {
                    if (N == Chain[i]) continue;
                    const FVector2D Q = RoadNodes[N].Pos;
                    const float D = (Q - P).SizeSquared();
                    if (D < BestD)
                    {
                        BestD = D;
                        BestIdx = N;
                    }
                }

                if (BestIdx >= 0 && BestD < FMath::Square(TownRadius * 0.25f))
                {
                    const FVector2D Mid = (RoadNodes[BestIdx].Pos + P) * 0.5f;
                    if (!IsNearRiver(Mid, 0.f))
                        AddEdge(Chain[i], BestIdx, EStreetTier::Tertiary);
                }
            }
        };

        ConnectBankChain(LeftBankNodes);
        ConnectBankChain(RightBankNodes);
    }
}

void AMedievalTownGenerator::ElevateRoadSplines()
{
    // For each generated edge, build a smoothed world-space spline
    // conforming to terrain height. Bridge edges use per-point detection:
    // only points actually over the river get elevated to bank height.
    for (FRoadEdge& E : RoadEdges)
    {
        if (!E.bIsGenerated) continue;
        E.WorldPoints.Empty();

        FVector2D PA = RoadNodes[E.NodeA].Pos;
        FVector2D PB = RoadNodes[E.NodeB].Pos;

        // Check if this edge crosses the river at all
        E.bIsBridge = bGenerateRiver && SegmentCrossesRiver(PA, PB);

        const int32 Subs = E.bIsBridge ? FMath::Max(RoadSplineSubdivisions * 2, 8) : FMath::Max(RoadSplineSubdivisions, 3);
        for (int32 S = 0; S <= Subs; S++)
        {
            float T = (float)S / Subs;
            FVector2D Pt = FMath::Lerp(PA, PB, T);

            // Add gentle organic waver (not at endpoints, not on bridges)
            if (S > 0 && S < Subs && RoadOrganicWaver > 0.f && !E.bIsBridge)
            {
                FVector2D Dir = (PB - PA).GetSafeNormal();
                FVector2D Perp = MTGRoads::Perpendicular2D(Dir);
                float Wave = Rand.FRandRange(-RoadOrganicWaver,
                                             RoadOrganicWaver) *
                             FMath::Sin(T * PI);
                Pt += Perp * Wave;
            }

            float H;
            if (E.bIsBridge)
            {
                // Per-point: check if THIS point is over the river
                float RDist = DistToRiverCenter(Pt);
                float HalfW = RiverWidth * 0.5f;
                float BankMargin = HalfW * 0.5f;  // Start ramp before river edge

                if (RDist < HalfW + BankMargin)
                {
                    // Over or near the river — use bank-level height + bridge raise
                    float BankH = GetTerrainHeightNoRiver(Pt.X, Pt.Y);
                    H = BankH + 25.f;
                }
                else
                {
                    // Away from river — normal terrain
                    H = GetTerrainHeight(Pt.X, Pt.Y) + 12.f;
                }
            }
            else
            {
                H = GetTerrainHeight(Pt.X, Pt.Y) + 12.f;
            }

            E.WorldPoints.Add(GetActorLocation() + FVector(Pt.X, Pt.Y, H));
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

    UE_LOG(LogTemp, Log, TEXT("[MTG] BuildRoadNetwork: %d nodes, %d edges"),
           RoadNodes.Num(), RoadEdges.Num());
}
