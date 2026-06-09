// OrganicStreetGenerator.cpp
// -----------------------------------------------------------------------------
// Organic street growth pipeline, v2 (block-growth):
//   Stage 2  Primary trunks  (gate/bridge -> market square ring, with merging)
//   Stage 3  Block growth    (streets sprout from frontages and grow until
//                             they strike another street, snap to a node,
//                             or end as a dead-end lane)
// See OrganicStreetGenerator.h for the design rationale.
// -----------------------------------------------------------------------------
#include "OrganicStreetGenerator.h"
#include "Algo/Reverse.h"

#if defined(MEDIEVAL_HARNESS)
#include <cstdio>
#endif

namespace
{
    FVector2D RotateRad(const FVector2D& V, float Rad)
    {
        const float C = FMath::Cos(Rad), S = FMath::Sin(Rad);
        return FVector2D(V.X * C - V.Y * S, V.X * S + V.Y * C);
    }

    float PolyLen(const TArray<FVector2D>& Pts)
    {
        float L = 0.f;
        for (int32 i = 0; i < Pts.Num() - 1; i++) L += (Pts[i + 1] - Pts[i]).Size();
        return L;
    }
}

// -----------------------------------------------------------------------------
//  Construction
// -----------------------------------------------------------------------------

FOrganicStreetGenerator::FOrganicStreetGenerator(
    const FOrganicStreetConfig& InConfig,
    const FOrganicTerrainQuery& InTerrain,
    FRandomStream&               InRand)
    : Config(InConfig), Terrain(InTerrain), Rand(InRand)
{}

// -----------------------------------------------------------------------------
//  Grid helpers
// -----------------------------------------------------------------------------

int32 FOrganicStreetGenerator::GridW() const
{
    return FMath::CeilToInt((Config.TownRadius * 2.2f) / Config.AStarCellSize);
}
int32 FOrganicStreetGenerator::GridH() const { return GridW(); }

FVector2D FOrganicStreetGenerator::CellToWorld(int32 X, int32 Y) const
{
    float Half = Config.TownRadius * 1.1f;
    return FVector2D(-Half + X * Config.AStarCellSize,
                     -Half + Y * Config.AStarCellSize);
}

bool FOrganicStreetGenerator::WorldToCell(FVector2D W, int32& OutX, int32& OutY) const
{
    float Half = Config.TownRadius * 1.1f;
    OutX = FMath::FloorToInt((W.X + Half) / Config.AStarCellSize);
    OutY = FMath::FloorToInt((W.Y + Half) / Config.AStarCellSize);
    return OutX >= 0 && OutX < GridW() && OutY >= 0 && OutY < GridH();
}

int32 FOrganicStreetGenerator::CellIdx(int32 X, int32 Y) const
{
    return Y * GridW() + X;
}

// -----------------------------------------------------------------------------
//  Cell cost for A*
// -----------------------------------------------------------------------------

float FOrganicStreetGenerator::CellCost(FVector2D From, FVector2D To) const
{
    FVector2D Mid = (From + To) * 0.5f;

    if (Terrain.IsNearRiver(Mid, 0.f))
        return Terrain.WaterPenalty * 4.f;

    // Keep the market square open: routes go around it, not across it.
    if ((Mid - PlazaCenter).Size() < Config.PlazaRadius * 0.9f)
        return Terrain.WaterPenalty * 3.f;

    float H0   = Terrain.GetHeight(From);
    float H1   = Terrain.GetHeight(To);
    float Dist = (To - From).Size();
    if (Dist < 0.1f) return 1.f;

    float Slope     = FMath::Abs(H1 - H0) / Dist;
    float SlopeCost = 1.f + FMath::Max(0.f, Slope - Terrain.MaxGrade)
                           * Terrain.SlopePenalty * 8.f;

    float AvgH   = (H0 + H1) * 0.5f;
    float ValleyB = 1.f + FMath::Clamp(AvgH / (Config.TownRadius * 0.05f), -0.2f, 0.2f)
                        * Terrain.ValleyPreference;

    return SlopeCost * ValleyB;
}

// -----------------------------------------------------------------------------
//  A* Pathfinder  (8-connected grid, linear-scan open list)
// -----------------------------------------------------------------------------

TArray<FVector2D> FOrganicStreetGenerator::RouteAStar(FVector2D From, FVector2D To,
                                                        float /*MaxGradeOverride*/) const
{
    const int32 GW = GridW(), GH = GridH(), Total = GW * GH;
    if (Total <= 0) return { From, To };

    int32 StartX, StartY, GoalX, GoalY;
    if (!WorldToCell(From, StartX, StartY) || !WorldToCell(To, GoalX, GoalY))
        return { From, To };

    TArray<float> G;    G.Init(TNumericLimits<float>::Max(), Total);
    TArray<float> F;    F.Init(TNumericLimits<float>::Max(), Total);
    TArray<int32> Parent; Parent.Init(-1, Total);
    TArray<bool>  Closed; Closed.Init(false, Total);
    TArray<int32> Open;   Open.Reserve(512);

    auto H = [&](int32 x, int32 y) -> float {
        return (CellToWorld(x, y) - To).Size();
    };

    const int32 SI = CellIdx(StartX, StartY);
    G[SI] = 0.f;
    F[SI] = H(StartX, StartY);
    Open.Add(SI);

    static const int32 DX[8] = { 0, 0, 1,-1, 1, 1,-1,-1 };
    static const int32 DY[8] = { 1,-1, 0, 0, 1,-1, 1,-1 };
    const float DiagMul = 1.4142f;

    while (Open.Num() > 0)
    {
        int32 BestLI = 0;
        for (int32 i = 1; i < Open.Num(); i++)
            if (F[Open[i]] < F[Open[BestLI]]) BestLI = i;
        const int32 CI = Open[BestLI];
        Open.RemoveAtSwap(BestLI);

        if (Closed[CI]) continue;
        Closed[CI] = true;

        const int32 CX = CI % GW, CY = CI / GW;
        if (CX == GoalX && CY == GoalY)
        {
            TArray<FVector2D> Path;
            int32 Idx = CI;
            while (Idx != -1)
            {
                Path.Add(CellToWorld(Idx % GW, Idx / GW));
                Idx = Parent[Idx];
            }
            Algo::Reverse(Path);
            if (Path.Num() > 0) Path[0]     = From;
            if (Path.Num() > 1) Path.Last() = To;
            return Path;
        }

        const FVector2D CWorld = CellToWorld(CX, CY);
        for (int32 d = 0; d < 8; d++)
        {
            int32 NX = CX + DX[d], NY = CY + DY[d];
            if (NX < 0 || NX >= GW || NY < 0 || NY >= GH) continue;
            const int32 NI = CellIdx(NX, NY);
            if (Closed[NI]) continue;

            const FVector2D NWorld = CellToWorld(NX, NY);
            const bool bDiag  = (DX[d] != 0 && DY[d] != 0);
            const float MoveDist = bDiag ? Config.AStarCellSize * DiagMul : Config.AStarCellSize;
            const float Cost  = CellCost(CWorld, NWorld);
            const float NewG  = G[CI] + MoveDist * Cost;

            if (NewG < G[NI])
            {
                G[NI]      = NewG;
                F[NI]      = NewG + H(NX, NY);
                Parent[NI] = CI;
                Open.AddUnique(NI);
            }
        }
    }
    return { From, To };
}

// -----------------------------------------------------------------------------
//  Polyline simplification  (Ramer-Douglas-Peucker)
// -----------------------------------------------------------------------------

static void RDP_Recurse(const TArray<FVector2D>& Pts, int32 S, int32 E,
                         float Eps, TArray<bool>& Keep)
{
    if (E <= S + 1) return;
    FVector2D AB  = Pts[E] - Pts[S];
    float ABLen   = AB.Size();
    float MaxD    = 0.f;
    int32 MaxI    = S + 1;
    for (int32 i = S + 1; i < E; i++)
    {
        float d;
        if (ABLen < 0.01f)
        {
            d = (Pts[i] - Pts[S]).Size();
        }
        else
        {
            float T = FMath::Clamp(
                FVector2D::DotProduct(Pts[i] - Pts[S], AB) / (ABLen * ABLen), 0.f, 1.f);
            d = (Pts[i] - (Pts[S] + AB * T)).Size();
        }
        if (d > MaxD) { MaxD = d; MaxI = i; }
    }
    if (MaxD > Eps)
    {
        Keep[MaxI] = true;
        RDP_Recurse(Pts, S, MaxI, Eps, Keep);
        RDP_Recurse(Pts, MaxI, E, Eps, Keep);
    }
}

TArray<FVector2D> FOrganicStreetGenerator::SimplifyRDP(
    const TArray<FVector2D>& Pts, float Epsilon) const
{
    if (Pts.Num() <= 2) return Pts;
    TArray<bool> Keep; Keep.SetNumZeroed(Pts.Num());
    Keep[0] = true; Keep.Last() = true;
    RDP_Recurse(Pts, 0, Pts.Num() - 1, Epsilon, Keep);
    TArray<FVector2D> Out;
    for (int32 i = 0; i < Pts.Num(); i++) if (Keep[i]) Out.Add(Pts[i]);
    return Out;
}

// -----------------------------------------------------------------------------
//  Chaikin smoothing  (preserves endpoints)
// -----------------------------------------------------------------------------

TArray<FVector2D> FOrganicStreetGenerator::SmoothChaikin(
    const TArray<FVector2D>& Pts, int32 Passes) const
{
    TArray<FVector2D> P = Pts;
    for (int32 Pass = 0; Pass < Passes; Pass++)
    {
        if (P.Num() < 2) break;
        TArray<FVector2D> N;
        N.Reserve(P.Num() * 2);
        N.Add(P[0]);
        for (int32 i = 0; i < P.Num() - 1; i++)
        {
            N.Add(P[i] * 0.75f + P[i+1] * 0.25f);
            N.Add(P[i] * 0.25f + P[i+1] * 0.75f);
        }
        N.Add(P.Last());
        P = MoveTemp(N);
    }
    return P;
}

// -----------------------------------------------------------------------------
//  Full routing pipeline: A* -> RDP -> Chaikin -> organic jitter
// -----------------------------------------------------------------------------

TArray<FVector2D> FOrganicStreetGenerator::RouteAndSmooth(
    FVector2D From, FVector2D To, float MaxGrade, float RDPEps) const
{
    TArray<FVector2D> Path = RouteAStar(From, To, MaxGrade);
    Path = SimplifyRDP(Path, RDPEps);
    Path = SmoothChaikin(Path, 2);

    // Organic lateral jitter on interior points only (sine-weighted, not at endpoints)
    for (int32 i = 1; i < Path.Num() - 1; i++)
    {
        FVector2D Dir  = (Path[i+1] - Path[i-1]).GetSafeNormal();
        FVector2D Perp(-Dir.Y, Dir.X);
        float Jitter   = Rand.FRandRange(-Config.AStarCellSize * 0.3f,
                                          Config.AStarCellSize * 0.3f)
                       * FMath::Sin((float)i / (Path.Num()-1) * PI);
        Path[i] += Perp * Jitter;
    }
    return Path;
}

// -----------------------------------------------------------------------------
//  Width helper
// -----------------------------------------------------------------------------

float FOrganicStreetGenerator::PickWidth(EOrganicStreetType Type) const
{
    switch (Type)
    {
    case EOrganicStreetType::Primary:   return Rand.FRandRange(Config.PrimaryWidthMin,   Config.PrimaryWidthMax);
    case EOrganicStreetType::Secondary: return Rand.FRandRange(Config.SecondaryWidthMin, Config.SecondaryWidthMax);
    case EOrganicStreetType::Lane:      return Rand.FRandRange(Config.LaneWidthMin,      Config.LaneWidthMax);
    case EOrganicStreetType::Alley:     return Rand.FRandRange(Config.AlleyWidthMin,     Config.AlleyWidthMax);
    }
    return 400.f;
}

// -----------------------------------------------------------------------------
//  Density / block-spacing fields
// -----------------------------------------------------------------------------

float FOrganicStreetGenerator::DensityAt(FVector2D Pos) const
{
    // Market pull is deliberately wide and shallow; most of the variation
    // comes from noise + bridgehead boosts, so the network does not collapse
    // into a radial pattern around a single attractor.
    float MarketInfl = FMath::Exp(-(Pos - PlazaCenter).Size() / (Config.TownRadius * 0.45f));

    float Noise = FMath::PerlinNoise2D(Pos * 0.00016f + FVector2D(3.1f, 17.9f)) * 0.24f
                + FMath::PerlinNoise2D(Pos * 0.00043f + FVector2D(41.2f, 7.7f)) * 0.12f;

    float BridgeBoost = 0.f;
    for (const FVector2D& B : BridgePoints)
        BridgeBoost = FMath::Max(BridgeBoost,
            0.30f * FMath::Exp(-(Pos - B).Size() / (Config.TownRadius * 0.16f)));

    float EdgeFade = FMath::Clamp(1.f - Pos.Size() / (Config.TownRadius * 0.96f), 0.f, 1.f);
    EdgeFade = FMath::Sqrt(EdgeFade);

    return FMath::Clamp(0.36f + 0.50f * MarketInfl + Noise + BridgeBoost, 0.04f, 1.f)
         * EdgeFade;
}

float FOrganicStreetGenerator::BlockSpacingAt(FVector2D Pos) const
{
    const float D = DensityAt(Pos);
    return FMath::Lerp(Config.BlockSpacingEdge, Config.BlockSpacingCore,
                       FMath::Clamp(D * 1.35f, 0.f, 1.f));
}

bool FOrganicStreetGenerator::IsNearJunction(const FOrganicStreetGraph& G,
                                              FVector2D Pos, float Radius) const
{
    const float R2 = Radius * Radius;
    for (const FOrganicStreetNode& N : G.Nodes)
    {
        if (N.ConnectedEdges.Num() < 2) continue;
        if ((N.Position - Pos).SizeSquared() < R2) return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
//  Bridge selection
// -----------------------------------------------------------------------------

TArray<FBridgeCandidate> FOrganicStreetGenerator::SelectBridges(
    const TArray<FBridgeCandidate>& Candidates) const
{
    if (Candidates.IsEmpty()) return {};

    TArray<FBridgeCandidate> Sorted = Candidates;
    Sorted.Sort([](const FBridgeCandidate& A, const FBridgeCandidate& B) {
        return A.Quality > B.Quality;
    });

    TArray<FBridgeCandidate> Selected;
    const float MinBridgeSep = Config.TownRadius * 0.5f;

    for (const FBridgeCandidate& C : Sorted)
    {
        bool bTooClose = false;
        for (const FBridgeCandidate& S : Selected)
            if ((S.Position - C.Position).Size() < MinBridgeSep) { bTooClose = true; break; }
        if (!bTooClose)
        {
            Selected.Add(C);
            if (Selected.Num() >= Config.MaxBridges) break;
        }
    }
    return Selected;
}

// -----------------------------------------------------------------------------
//  Market square  (irregular ring of short streets enclosing the plaza)
// -----------------------------------------------------------------------------

void FOrganicStreetGenerator::BuildMarketSquare(FOrganicStreetGraph& G)
{
    // Nudge the square off the bridgehead / river onto buildable ground.
    PlazaCenter = Config.MarketCenter;
    if (Terrain.IsNearRiver(PlazaCenter, Config.PlazaRadius))
    {
        const float Step = Config.PlazaRadius * 0.6f;
        bool bPlaced = false;
        for (int32 Ring = 1; Ring <= 6 && !bPlaced; Ring++)
        {
            for (int32 a = 0; a < 8 && !bPlaced; a++)
            {
                const float Ang = (float)a / 8.f * TWO_PI + Ring * 0.4f;
                const FVector2D Cand = Config.MarketCenter
                    + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * Step * (float)Ring;
                if (!Terrain.IsNearRiver(Cand, Config.PlazaRadius * 0.9f) &&
                    Cand.Size() < Config.TownRadius * 0.6f)
                {
                    PlazaCenter = Cand;
                    bPlaced = true;
                }
            }
        }
    }

    // Irregular convex ring: market squares grew from widened junctions, so
    // anything but a perfect circle/rectangle reads right.
    const int32 NumPts = Rand.RandRange(5, 7);
    const float BaseAngle = Rand.FRandRange(0.f, TWO_PI);
    PlazaNodes.Empty();

    for (int32 i = 0; i < NumPts; i++)
    {
        const float Ang = BaseAngle + (float)i / NumPts * TWO_PI
                        + Rand.FRandRange(-0.45f, 0.45f) / NumPts * TWO_PI;
        const float R = Config.PlazaRadius * Rand.FRandRange(0.78f, 1.05f);
        FVector2D P = PlazaCenter + FVector2D(FMath::Cos(Ang), FMath::Sin(Ang)) * R;

        const int32 N = G.AddNode(P);
        G.Nodes[N].bIsPlaza   = true;
        G.Nodes[N].Importance = 0.95f;
        PlazaNodes.Add(N);
    }
    if (PlazaNodes.Num() > 0)
        G.Nodes[PlazaNodes[0]].bIsMarket = true;

    const float RingWidth = (Config.SecondaryWidthMin + Config.SecondaryWidthMax) * 0.5f;
    for (int32 i = 0; i < PlazaNodes.Num(); i++)
    {
        const int32 A = PlazaNodes[i];
        const int32 B = PlazaNodes[(i + 1) % PlazaNodes.Num()];
        TArray<FVector2D> Poly = { G.Nodes[A].Position, G.Nodes[B].Position };
        const int32 EIdx = G.AddEdge(A, B, EOrganicStreetType::Secondary, RingWidth,
                                     MoveTemp(Poly));
        if (EIdx != INDEX_NONE)
        {
            G.Edges[EIdx].bIsPlazaEdge = true;
            G.Edges[EIdx].Surface = ESurfaceTag::PavedStone;
        }
    }
}

int32 FOrganicStreetGenerator::NearestPlazaNode(const FOrganicStreetGraph& G,
                                                 FVector2D From) const
{
    int32 Best = INDEX_NONE;
    float BestD = TNumericLimits<float>::Max();
    for (int32 N : PlazaNodes)
    {
        const float D = (G.Nodes[N].Position - From).SizeSquared();
        if (D < BestD) { BestD = D; Best = N; }
    }
    return Best;
}

// -----------------------------------------------------------------------------
//  Primary routing with trunk merging
// -----------------------------------------------------------------------------

void FOrganicStreetGenerator::AddPrimaryRoute(FOrganicStreetGraph& G,
                                               int32 FromNode, int32 EndNode,
                                               TArray<FVector2D>&& Path)
{
    // Split the route at river-band transitions so ONLY the water-crossing
    // chunks are flagged as bridges; the dry approaches stay normal streets
    // that buildings can front and other streets can junction into.
    const float W = PickWidth(EOrganicStreetType::Primary);

    TArray<int32> ChunkEnds;       // indices into Path where a chunk ends
    TArray<bool>  ChunkWet;
    bool bWet = Terrain.IsNearRiver(Path[0], 100.f);
    for (int32 i = 1; i < Path.Num(); i++)
    {
        const bool bPointWet = Terrain.IsNearRiver(Path[i], 100.f);
        if (bPointWet != bWet)
        {
            ChunkEnds.Add(i);
            ChunkWet.Add(bWet);
            bWet = bPointWet;
        }
    }
    ChunkEnds.Add(Path.Num() - 1);
    ChunkWet.Add(bWet);

    int32 PrevNode = FromNode;
    int32 Start = 0;
    for (int32 c = 0; c < ChunkEnds.Num(); c++)
    {
        const int32 End = ChunkEnds[c];
        if (End <= Start) { Start = End; continue; }

        TArray<FVector2D> Chunk;
        for (int32 i = Start; i <= End; i++) Chunk.Add(Path[i]);

        int32 NodeB;
        if (c == ChunkEnds.Num() - 1)
        {
            NodeB = EndNode;
        }
        else
        {
            NodeB = G.AddNode(Path[End]);
            G.Nodes[NodeB].Importance = 0.85f;
            if (ChunkWet[c]) G.Nodes[NodeB].bIsBridgeNode = true;
        }

        const int32 EIdx = G.AddEdge(PrevNode, NodeB, EOrganicStreetType::Primary,
                                     W, MoveTemp(Chunk));
        if (EIdx != INDEX_NONE)
        {
            G.Edges[EIdx].bIsBridge = ChunkWet[c];
            G.Edges[EIdx].Surface   = ChunkWet[c] ? ESurfaceTag::BridgeStone
                                                   : ESurfaceTag::PavedStone;
        }
        PrevNode = NodeB;
        Start = End;
    }
}

int32 FOrganicStreetGenerator::ConnectWithMerge(FOrganicStreetGraph& G,
                                                 int32 FromNode, int32 TargetNode,
                                                 bool bIsBridgeRoute)
{
    if (FromNode < 0 || TargetNode < 0 || FromNode == TargetNode) return INDEX_NONE;

    const FVector2D From = G.Nodes[FromNode].Position;
    const FVector2D To   = G.Nodes[TargetNode].Position;

    TArray<FVector2D> Path = RouteAndSmooth(From, To,
                                            Config.MaxGradePrimary,
                                            Config.RDPEpsilonPrimary);
    if (Path.Num() < 2) return INDEX_NONE;

    // Walk the path; once we are past the first stretch, merge into the first
    // existing street that comes within MergeDistance. Bridge routes never
    // merge before clearing the river.
    const float TotalLen = PolyLen(Path);
    int32 EndNode = TargetNode;
    float MergeT; FVector2D MergeP;

    float Walked = 0.f;
    for (int32 i = 1; i < Path.Num() - 1; i++)
    {
        Walked += (Path[i] - Path[i - 1]).Size();
        if (Walked < TotalLen * 0.22f) continue;
        if (bIsBridgeRoute && Terrain.IsNearRiver(Path[i], Config.MergeDistance * 0.6f))
            continue;

        const int32 HitEdge = G.FindNearestEdgePoint(Path[i], MergeT, MergeP,
                                                     Config.MergeDistance);
        if (HitEdge != INDEX_NONE && !G.Edges[HitEdge].bIsBridge &&
            G.Edges[HitEdge].StreetType == EOrganicStreetType::Primary)
        {
            const int32 MergeNode = G.SplitEdge(HitEdge, MergeT);
            if (MergeNode != INDEX_NONE)
            {
                Path.SetNum(i + 1);
                Path.Add(MergeP);
                EndNode = MergeNode;
            }
            break;
        }
    }

    AddPrimaryRoute(G, FromNode, EndNode, MoveTemp(Path));
    return EndNode;
}

// -----------------------------------------------------------------------------
//  STAGE 2 -- Primary network
// -----------------------------------------------------------------------------

void FOrganicStreetGenerator::Stage2_Primary(FOrganicStreetGraph& G,
                                              const TArray<FVector2D>& Gates,
                                              const TArray<FBridgeCandidate>& Bridges,
                                              FVector2D Church, FVector2D Keep)
{
    BuildMarketSquare(G);

    // Bridge nodes first: river crossings anchor the whole network.
    TArray<FBridgeCandidate> UsedBridges = SelectBridges(Bridges);
    BridgePoints.Empty();
    TArray<int32> BridgeNodes;
    for (const FBridgeCandidate& B : UsedBridges)
    {
        const int32 BN = G.AddNode(B.Position);
        G.Nodes[BN].bIsBridgeNode = true;
        G.Nodes[BN].Importance    = 0.9f;
        BridgeNodes.Add(BN);
        BridgePoints.Add(B.Position);
    }

    // Gate nodes
    TArray<int32> GateNodes;
    for (const FVector2D& GPos : Gates)
    {
        const int32 GN = G.AddNode(GPos);
        G.Nodes[GN].bIsGate    = true;
        G.Nodes[GN].Importance = 0.85f;
        GateNodes.Add(GN);
    }

    // Landmark nodes (church, keep) with position jitter
    auto AddLandmark = [&](FVector2D Pos, float Imp) -> int32 {
        const int32 LN = G.AddNode(Pos + FVector2D(Rand.FRandRange(-500.f, 500.f),
                                                   Rand.FRandRange(-500.f, 500.f)));
        G.Nodes[LN].bIsLandmark = true;
        G.Nodes[LN].Importance  = Imp;
        return LN;
    };
    const int32 ChurchNode = AddLandmark(Church, 0.75f);
    const int32 KeepNode   = AddLandmark(Keep,   0.80f);

    // Bridge -> market square. Routed first so the high street through the
    // square exists before the gate roads come in and merge into it.
    for (int32 BN : BridgeNodes)
        ConnectWithMerge(G, BN, NearestPlazaNode(G, G.Nodes[BN].Position), true);

    // Gates -> market square, farthest gate first (so long trunk roads form
    // early and the nearer gates merge into them organically).
    TArray<int32> SortedGates = GateNodes;
    SortedGates.Sort([&](int32 A, int32 B) {
        return (G.Nodes[A].Position - PlazaCenter).SizeSquared()
             > (G.Nodes[B].Position - PlazaCenter).SizeSquared();
    });
    for (int32 GN : SortedGates)
        ConnectWithMerge(G, GN, NearestPlazaNode(G, G.Nodes[GN].Position), false);

    // Each bridge also serves its far bank: road to the nearest gate, so the
    // crossing carries through-traffic instead of dead-ending at the water.
    for (int32 BN : BridgeNodes)
    {
        int32 NearestGate = INDEX_NONE;
        float BestD = TNumericLimits<float>::Max();
        for (int32 GN : GateNodes)
        {
            const float D = (G.Nodes[GN].Position - G.Nodes[BN].Position).SizeSquared();
            if (D < BestD) { BestD = D; NearestGate = GN; }
        }
        if (NearestGate != INDEX_NONE)
            ConnectWithMerge(G, BN, NearestGate, true);
    }

    // Landmarks hang off the network (merge happily into any primary).
    ConnectWithMerge(G, ChurchNode, NearestPlazaNode(G, G.Nodes[ChurchNode].Position), false);
    ConnectWithMerge(G, KeepNode,   NearestPlazaNode(G, G.Nodes[KeepNode].Position),   false);
}

// -----------------------------------------------------------------------------
//  STAGE 3 -- Block growth
// -----------------------------------------------------------------------------

bool FOrganicStreetGenerator::GrowStreet(FOrganicStreetGraph& G,
                                          FVector2D SeedPos, FVector2D SeedDir,
                                          int32 SeedEdge, const FGrowthPass& Pass)
{
    const float Step       = FMath::Max(Config.AStarCellSize * 1.4f, 320.f);
    const float LocalBlock = BlockSpacingAt(SeedPos);
    const float MaxLen     = LocalBlock * Pass.MaxLenMult * Rand.FRandRange(0.75f, 1.30f);
    const float GrowLimit  = Config.TownRadius * Config.GrowthLimitFraction;
    const float MaxGrade   = (Pass.GrowType == EOrganicStreetType::Secondary)
                               ? Config.MaxGradeSecondary : Config.MaxGradeLane;

    TArray<FVector2D> Poly;
    Poly.Add(SeedPos);

    FVector2D Pos = SeedPos;
    FVector2D Dir = SeedDir;
    float Length = 0.f;

    int32 EndEdge = INDEX_NONE;
    float EndT = 0.f;
    FVector2D EndPoint;
    int32 EndNode = INDEX_NONE;
    bool bConnected = false;

    while (Length < MaxLen)
    {
        // Wandering heading...
        Dir = RotateRad(Dir, Rand.FRandRange(-Config.StepHeadingNoise,
                                              Config.StepHeadingNoise));

        // ...deflected along the contour when the ground gets steep.
        if (Terrain.GetHeight)
        {
            const float H0 = Terrain.GetHeight(Pos);
            const float HA = Terrain.GetHeight(Pos + Dir * Step);
            if (FMath::Abs(HA - H0) / Step > MaxGrade * 0.6f)
            {
                const FVector2D DirL = RotateRad(Dir,  0.42f);
                const FVector2D DirR = RotateRad(Dir, -0.42f);
                const float GL = FMath::Abs(Terrain.GetHeight(Pos + DirL * Step) - H0);
                const float GR = FMath::Abs(Terrain.GetHeight(Pos + DirR * Step) - H0);
                Dir = (GL < GR) ? RotateRad(Dir, 0.21f) : RotateRad(Dir, -0.21f);
            }
        }

        const FVector2D Next = Pos + Dir * Step;

        // Hard stops: town edge, river, market square.
        if (Next.Size() > GrowLimit) break;
        if (Terrain.IsNearRiver(Next, 120.f)) break;
        if ((Next - PlazaCenter).Size() < Config.PlazaRadius * 0.9f) break;

        // Hit an existing street -> T-junction, block closed.
        float HitT; FVector2D HitP;
        const int32 HitEdge = G.RaycastEdges(Pos, Next, SeedEdge,
                                             (Length < Step * 1.5f) ? Step * 0.9f : 0.f,
                                             HitT, HitP);
        if (HitEdge != INDEX_NONE)
        {
            const float HitLen = Length + (HitP - Pos).Size();
            if (HitLen < Pass.MinLength) return false;   // pointless stub
            Poly.Add(HitP);
            EndEdge = HitEdge; EndT = HitT; EndPoint = HitP;
            bConnected = true;
            break;
        }

        // Snap to a nearby existing node (forms clean junctions).
        if (Length > Step * 1.5f)
        {
            const int32 Near = G.FindNearestNode(Next, Config.SnapDistance);
            if (Near != INDEX_NONE &&
                (G.Nodes[Near].Position - SeedPos).Size() > LocalBlock * 0.4f)
            {
                Poly.Add(G.Nodes[Near].Position);
                EndNode = Near;
                bConnected = true;
                break;
            }
        }

        Poly.Add(Next);
        Pos = Next;
        Length += Step;
    }

    if (!bConnected)
    {
        if (Length < Pass.MinLength) return false;

        // Tip rescue: if another street is close ahead, jump the gap and join
        // it -- this is what closes blocks instead of littering stubs.
        float TipT; FVector2D TipP;
        const int32 TipEdge = G.FindNearestEdgePoint(Pos + Dir * Step, TipT, TipP,
                                                     Config.SnapDistance * 2.4f);
        if (TipEdge != INDEX_NONE && !G.Edges[TipEdge].bIsBridge &&
            (TipP - SeedPos).Size() > LocalBlock * 0.4f &&
            !Terrain.IsNearRiver((Pos + TipP) * 0.5f, 60.f))
        {
            // Make sure the closing jump doesn't slice through a third street.
            float JumpT; FVector2D JumpP;
            const int32 JumpHit = G.RaycastEdges(Pos, TipP, TipEdge, Step * 0.2f,
                                                 JumpT, JumpP);
            if (JumpHit == INDEX_NONE)
            {
                Poly.Add(TipP);
                EndEdge = TipEdge; EndT = TipT; EndPoint = TipP;
                bConnected = true;
            }
        }

        // Otherwise: keep as a dead-end lane sometimes (medieval towns are
        // full of them), discard the rest.
        if (!bConnected && Rand.FRand() > Pass.DeadEndChance) return false;
    }

    // Re-attach the seed to whatever edge it currently sits on (edges may have
    // been split since the seed was sampled).
    float SeedT; FVector2D SeedSnap;
    const int32 SeedOnEdge = G.FindNearestEdgePoint(SeedPos, SeedT, SeedSnap, 600.f);
    if (SeedOnEdge == INDEX_NONE) return false;
    if (G.Edges[SeedOnEdge].bIsBridge) return false;

    const int32 StartNode = G.SplitEdge(SeedOnEdge, SeedT);
    if (StartNode == INDEX_NONE) return false;
    Poly[0] = G.Nodes[StartNode].Position;

    int32 FinalEnd = EndNode;
    if (EndEdge != INDEX_NONE)
    {
        // If the seed split shortened the polyline of the hit edge, EndT may
        // be stale; re-resolve against the current geometry.
        if (EndEdge == SeedOnEdge)
        {
            float T2; FVector2D P2;
            EndEdge = G.FindNearestEdgePoint(EndPoint, T2, P2, 600.f);
            if (EndEdge == INDEX_NONE) return false;
            EndT = T2;
        }
        FinalEnd = G.SplitEdge(EndEdge, EndT);
        if (FinalEnd == INDEX_NONE) return false;
        Poly.Last() = G.Nodes[FinalEnd].Position;
    }
    else if (EndNode == INDEX_NONE)
    {
        FinalEnd = G.AddNode(Poly.Last());   // dead-end terminus
    }

    TArray<FVector2D> Smoothed = SmoothChaikin(Poly, 1);
    G.AddEdge(StartNode, FinalEnd, Pass.GrowType,
              PickWidth(Pass.GrowType), MoveTemp(Smoothed));
    return true;
}

void FOrganicStreetGenerator::RunGrowthPass(FOrganicStreetGraph& G, const FGrowthPass& Pass)
{
    // Snapshot the edges that exist now; new streets grown during the pass
    // become seeds only in later passes (keeps growth breadth-first and even).
    struct FSeed { FVector2D Pos; FVector2D Dir; int32 Edge; };
    TArray<FSeed> Seeds;

    const int32 NumEdgesNow = G.Edges.Num();
    for (int32 EdgeIdx = 0; EdgeIdx < NumEdgesNow; EdgeIdx++)
    {
        const FOrganicStreetEdge& E = G.Edges[EdgeIdx];
        if (E.NodeA < 0 || E.Poly2D.Num() < 2 || E.bIsBridge) continue;

        const bool bAllowed =
            (E.StreetType == EOrganicStreetType::Primary   && Pass.bSeedPrimary)   ||
            (E.StreetType == EOrganicStreetType::Secondary && Pass.bSeedSecondary) ||
            (E.StreetType == EOrganicStreetType::Lane      && Pass.bSeedLane);
        if (!bAllowed) continue;
        if (E.bIsPlazaEdge && Pass.GrowType == EOrganicStreetType::Secondary) continue;

        const TArray<FVector2D>& P = E.Poly2D;
        const float Total = PolyLen(P);

        // Seeds at half-block spacing with alternating sides: a junction about
        // every block length per street side, before the density/junction
        // gates thin them out.
        bool bLeft = Rand.FRand() < 0.5f;
        float Cursor = BlockSpacingAt(P[0]) * Pass.SeedSpacingMult
                     * Rand.FRandRange(0.20f, 0.55f);
        while (Cursor < Total - 500.f)
        {
            // Locate point + tangent at Cursor.
            float Acc = 0.f;
            FVector2D SeedPos = P[0], Tangent(1.f, 0.f);
            for (int32 i = 0; i < P.Num() - 1; i++)
            {
                const float SegLen = (P[i+1] - P[i]).Size();
                if (Acc + SegLen >= Cursor || i == P.Num() - 2)
                {
                    const float T = (SegLen > 0.01f)
                        ? FMath::Clamp((Cursor - Acc) / SegLen, 0.f, 1.f) : 0.f;
                    SeedPos = FMath::Lerp(P[i], P[i+1], T);
                    Tangent = (P[i+1] - P[i]).GetSafeNormal();
                    break;
                }
                Acc += SegLen;
            }

            FSeed S;
            S.Pos  = SeedPos;
            S.Edge = EdgeIdx;
            const float Side = bLeft ? 1.f : -1.f;
            if (Rand.FRand() < 0.85f) bLeft = !bLeft;   // mostly alternate
            S.Dir = RotateRad(FVector2D(-Tangent.Y, Tangent.X) * Side,
                              Rand.FRandRange(-0.38f, 0.38f));
            Seeds.Add(S);

            Cursor += BlockSpacingAt(SeedPos) * Pass.SeedSpacingMult * 0.5f
                    * Rand.FRandRange(0.80f, 1.35f);
        }
    }

    // Shuffle so growth order is not biased along any one road.
    for (int32 i = Seeds.Num() - 1; i > 0; i--)
    {
        const int32 J = Rand.RandRange(0, i);
        const FSeed Tmp = Seeds[i]; Seeds[i] = Seeds[J]; Seeds[J] = Tmp;
    }

    int32 NumDensityReject = 0, NumJunctionReject = 0, NumGrowFail = 0, NumGrown = 0;
    for (const FSeed& S : Seeds)
    {
        // Density gate + junction spacing keep the network breathing.
        if (Rand.FRand() > DensityAt(S.Pos) * Pass.DensityGain) { NumDensityReject++; continue; }
        if (IsNearJunction(G, S.Pos, BlockSpacingAt(S.Pos) * 0.34f)) { NumJunctionReject++; continue; }

        if (GrowStreet(G, S.Pos, S.Dir, S.Edge, Pass)) NumGrown++; else NumGrowFail++;
    }
#if defined(MEDIEVAL_HARNESS)
    std::printf("  pass(type=%d): %d seeds -> %d grown, %d density-rej, %d junction-rej, %d grow-fail\n",
                (int)Pass.GrowType, Seeds.Num(), NumGrown, NumDensityReject, NumJunctionReject, NumGrowFail);
#endif
    (void)NumDensityReject; (void)NumJunctionReject; (void)NumGrowFail; (void)NumGrown;
}

void FOrganicStreetGenerator::Stage3_GrowBlocks(FOrganicStreetGraph& G)
{
    // Pass 1: secondary streets off the primary trunks.
    {
        FGrowthPass P;
        P.GrowType = EOrganicStreetType::Secondary;
        P.SeedSpacingMult = 1.0f;
        P.MaxLenMult = 3.2f;     // long enough to ladder between trunk roads
        P.MinLength = 1500.f;
        P.DeadEndChance = Config.DeadEndChanceSecondary;
        P.DensityGain = 2.2f;
        P.bSeedPrimary = true;
        RunGrowthPass(G, P);
    }
    // Pass 2: secondaries branching from secondaries (fills the quarters).
    {
        FGrowthPass P;
        P.GrowType = EOrganicStreetType::Secondary;
        P.SeedSpacingMult = 1.1f;
        P.MaxLenMult = 2.4f;
        P.MinLength = 1400.f;
        P.DeadEndChance = Config.DeadEndChanceSecondary;
        P.DensityGain = 2.3f;
        P.bSeedSecondary = true;
        RunGrowthPass(G, P);
    }
    // Pass 3: lanes subdividing the bigger blocks.
    {
        FGrowthPass P;
        P.GrowType = EOrganicStreetType::Lane;
        P.SeedSpacingMult = 0.80f;
        P.MaxLenMult = 1.1f;
        P.MinLength = 900.f;
        P.DeadEndChance = Config.DeadEndChanceLane;
        P.DensityGain = 1.05f;
        P.bSeedPrimary = true;
        P.bSeedSecondary = true;
        RunGrowthPass(G, P);
    }
    // Pass 4: alleys in the dense core; mostly dead-ends, very narrow.
    {
        FGrowthPass P;
        P.GrowType = EOrganicStreetType::Alley;
        P.SeedSpacingMult = 0.60f;
        P.MaxLenMult = 0.75f;
        P.MinLength = 600.f;
        P.DeadEndChance = Config.DeadEndChanceAlley;
        P.DensityGain = 0.70f;
        P.bSeedSecondary = true;
        P.bSeedLane = true;
        RunGrowthPass(G, P);
    }
}

// -----------------------------------------------------------------------------
//  GENERATE -- Top-level entry point
// -----------------------------------------------------------------------------

FOrganicStreetGraph FOrganicStreetGenerator::Generate(
    const TArray<FVector2D>& GatePositions,
    const TArray<FBridgeCandidate>& BridgeCandidates,
    FVector2D ChurchPos,
    FVector2D KeepPos)
{
    FOrganicStreetGraph Graph;
    Stage2_Primary   (Graph, GatePositions, BridgeCandidates, ChurchPos, KeepPos);
    Stage3_GrowBlocks(Graph);
    Graph.RemoveShortDangles(550.f);
    return Graph;
}
