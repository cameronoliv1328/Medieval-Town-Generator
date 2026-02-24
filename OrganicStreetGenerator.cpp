// OrganicStreetGenerator.cpp
// -----------------------------------------------------------------------------
// Organic street growth pipeline:
//   Stage 2  Primary routes    (anchor->anchor via A*)
//   Stage 3  Secondary streets (density-weighted attractor accretion)
//   Stage 4  Tertiary lanes    (back lanes / alleys in dense core)
// -----------------------------------------------------------------------------
#include "OrganicStreetGenerator.h"
#include "Algo/Reverse.h"

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
//  Density field  (for attractor weighting)
// -----------------------------------------------------------------------------

float FOrganicStreetGenerator::DensityAt(FVector2D Pos) const
{
    float MarketDist = (Pos - Config.MarketCenter).Size();
    float MarketInfl = FMath::Exp(-MarketDist / (Config.TownRadius * 0.35f));
    float Noise      = FMath::PerlinNoise2D(FVector2D(Pos.X, Pos.Y) * 0.00022f) * 0.3f;
    float EdgeFade   = FMath::Clamp(1.f - Pos.Size() / (Config.TownRadius * 0.90f), 0.f, 1.f);
    return FMath::Clamp(MarketInfl * 0.7f + 0.15f + Noise, 0.f, 1.f) * EdgeFade;
}

// -----------------------------------------------------------------------------
//  Intersection spacing check
// -----------------------------------------------------------------------------

bool FOrganicStreetGenerator::CheckIntersectionSpacing(
    const FOrganicStreetGraph& G, FVector2D Pos) const
{
    bool bCore = Pos.Size() < Config.TownRadius * Config.CoreRadiusFraction;
    float MinSpacing = bCore ? Config.MinSpacingCore : Config.MinSpacingOutskirts;

    for (const FOrganicStreetNode& N : G.Nodes)
    {
        if (N.ConnectedEdges.Num() < 2) continue;
        if ((N.Position - Pos).SizeSquared() < MinSpacing * MinSpacing) return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
//  Near-parallel check  (< ~22? difference = near-parallel)
// -----------------------------------------------------------------------------

bool FOrganicStreetGenerator::IsNearParallelToEdge(const FOrganicStreetGraph& G,
                                                     int32 EdgeIdx, FVector2D Dir) const
{
    if (!G.Edges.IsValidIndex(EdgeIdx) || G.Edges[EdgeIdx].Poly2D.Num() < 2) return false;
    const FOrganicStreetEdge& E = G.Edges[EdgeIdx];
    FVector2D EdgeDir = (E.Poly2D.Last() - E.Poly2D[0]).GetSafeNormal();
    return FMath::Abs(FVector2D::DotProduct(Dir, EdgeDir)) > 0.92f;
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
//  STAGE 2 -- Primary network  (anchor->anchor routes)
// -----------------------------------------------------------------------------

void FOrganicStreetGenerator::Stage2_Primary(FOrganicStreetGraph& G,
                                              const TArray<FVector2D>& Gates,
                                              const TArray<FBridgeCandidate>& Bridges,
                                              FVector2D Church, FVector2D Keep)
{
    // Market node (slight random offset for organicism)
    FVector2D MarketPos = Config.MarketCenter
        + FVector2D(Rand.FRandRange(-800.f, 800.f), Rand.FRandRange(-800.f, 800.f));
    int32 MarketNode = G.AddNode(MarketPos);
    G.Nodes[MarketNode].bIsMarket  = true;
    G.Nodes[MarketNode].Importance = 1.0f;

    // Gate nodes
    TArray<int32> GateNodes;
    for (const FVector2D& GPos : Gates)
    {
        int32 GN = G.AddNode(GPos);
        G.Nodes[GN].bIsGate    = true;
        G.Nodes[GN].Importance = 0.85f;
        GateNodes.Add(GN);
    }

    // Bridge nodes
    TArray<FBridgeCandidate> UsedBridges = SelectBridges(Bridges);
    TArray<int32> BridgeNodes;
    for (const FBridgeCandidate& B : UsedBridges)
    {
        int32 BN = G.AddNode(B.Position);
        G.Nodes[BN].bIsBridgeNode = true;
        G.Nodes[BN].Importance    = 0.9f;
        BridgeNodes.Add(BN);
    }

    // Landmark nodes (church, keep) with position jitter
    auto AddLandmark = [&](FVector2D Pos, float Imp) -> int32 {
        int32 LN = G.AddNode(Pos + FVector2D(Rand.FRandRange(-500.f, 500.f),
                                              Rand.FRandRange(-500.f, 500.f)));
        G.Nodes[LN].bIsLandmark = true;
        G.Nodes[LN].Importance  = Imp;
        return LN;
    };
    int32 ChurchNode = AddLandmark(Church, 0.75f);
    int32 KeepNode   = AddLandmark(Keep,   0.80f);

    // Helper: connect two nodes with primary route
    auto ConnectPrimary = [&](int32 A, int32 B, bool bBridge = false)
    {
        if (A < 0 || B < 0) return;
        TArray<FVector2D> Poly = RouteAndSmooth(
            G.Nodes[A].Position, G.Nodes[B].Position,
            Config.MaxGradePrimary, Config.RDPEpsilonPrimary);
        float W = PickWidth(EOrganicStreetType::Primary);
        int32 EIdx = G.AddEdge(A, B, EOrganicStreetType::Primary, W, MoveTemp(Poly));
        if (EIdx != INDEX_NONE)
        {
            G.Edges[EIdx].bIsBridge = bBridge;
            G.Edges[EIdx].Surface   = bBridge ? ESurfaceTag::BridgeStone
                                               : ESurfaceTag::PavedStone;
        }
    };

    // Gate -> Market  (T-junction preference: offset near-market waypoints)
    for (int32 GN : GateNodes)
    {
        FVector2D OffsetDir  = FVector2D(Rand.FRandRange(-1.f,1.f),
                                          Rand.FRandRange(-1.f,1.f)).GetSafeNormal();
        FVector2D NearMarket = MarketPos + OffsetDir * Config.TIntersectionOffset;
        int32 NM = G.AddNode(NearMarket);
        G.Nodes[NM].Importance = 0.9f;

        TArray<FVector2D> Poly1 = RouteAndSmooth(G.Nodes[GN].Position, NearMarket,
                                                   Config.MaxGradePrimary, Config.RDPEpsilonPrimary);
        float W = PickWidth(EOrganicStreetType::Primary);
        G.AddEdge(GN, NM, EOrganicStreetType::Primary, W, MoveTemp(Poly1));

        TArray<FVector2D> ShortPoly = { NearMarket, MarketPos };
        G.AddEdge(NM, MarketNode, EOrganicStreetType::Primary, W * 0.9f, MoveTemp(ShortPoly));
    }

    // Bridge -> Market; bridge -> nearest gate
    for (int32 i = 0; i < BridgeNodes.Num(); i++)
    {
        ConnectPrimary(BridgeNodes[i], MarketNode, false);
        if (G.Edges.Num() > 0) G.Edges.Last().bIsBridge = true;

        if (GateNodes.Num() > 0)
        {
            int32 NearestGate = -1; float BestD = 1e9f;
            for (int32 GN : GateNodes)
            {
                float D = (G.Nodes[GN].Position - G.Nodes[BridgeNodes[i]].Position).Size();
                if (D < BestD) { BestD = D; NearestGate = GN; }
            }
            if (NearestGate >= 0) ConnectPrimary(BridgeNodes[i], NearestGate);
        }
    }

    // Market -> landmarks
    ConnectPrimary(MarketNode, ChurchNode);
    ConnectPrimary(MarketNode, KeepNode);
}

// -----------------------------------------------------------------------------
//  STAGE 3 -- Secondary street accretion
// -----------------------------------------------------------------------------

bool FOrganicStreetGenerator::ConnectAttractorToGraph(
    FOrganicStreetGraph& G, FVector2D AttPos,
    EOrganicStreetType Type, float Width)
{
    float T; FVector2D Closest;
    int32 EdgeIdx = G.FindNearestEdgePoint(AttPos, T, Closest, Config.TownRadius * 0.6f);
    if (EdgeIdx == INDEX_NONE) return false;

    float ConnDist = (AttPos - Closest).Size();
    float MinDist  = (Type == EOrganicStreetType::Secondary) ? 1800.f : 800.f;
    float MaxDist  = (Type == EOrganicStreetType::Secondary) ? 8000.f : 4000.f;
    if (ConnDist < MinDist || ConnDist > MaxDist) return false;

    FVector2D ConnDir = (AttPos - Closest).GetSafeNormal();
    if (IsNearParallelToEdge(G, EdgeIdx, ConnDir)) return false;
    if (!CheckIntersectionSpacing(G, Closest)) return false;
    if (!CheckIntersectionSpacing(G, AttPos))  return false;
    if (Terrain.IsNearRiver((Closest + AttPos) * 0.5f, 0.f)) return false;
    if (G.WouldSelfIntersect(Closest, AttPos)) return false;

    float Grade = (Type == EOrganicStreetType::Secondary) ? Config.MaxGradeSecondary : Config.MaxGradeLane;
    TArray<FVector2D> Poly = RouteAndSmooth(Closest, AttPos, Grade, Config.RDPEpsilonSecondary);

    int32 ConnNode = G.SplitEdge(EdgeIdx, T);
    if (ConnNode == INDEX_NONE) return false;

    int32 EndNode = G.AddNode(AttPos);
    G.AddEdge(ConnNode, EndNode, Type, Width, MoveTemp(Poly));
    return true;
}

void FOrganicStreetGenerator::Stage3_Secondary(FOrganicStreetGraph& G)
{
    int32 Connected = 0;
    int32 MaxTries  = Config.SecondaryAttractors * 5;

    for (int32 Try = 0; Try < MaxTries && Connected < Config.SecondaryAttractors; Try++)
    {
        float R     = FMath::Sqrt(Rand.FRand()) * Config.TownRadius * 0.88f;
        float Angle = Rand.FRandRange(0.f, TWO_PI);
        FVector2D Pos(FMath::Cos(Angle) * R, FMath::Sin(Angle) * R);

        if (Rand.FRand() > DensityAt(Pos)) continue;

        EOrganicStreetType Type = EOrganicStreetType::Secondary;
        if (ConnectAttractorToGraph(G, Pos, Type, PickWidth(Type)))
        {
            // Rare loop
            bool bCore = Pos.Size() < Config.TownRadius * Config.CoreRadiusFraction;
            if (Rand.FRand() < (bCore ? Config.LoopChanceCore : Config.LoopChanceOutskirts))
            {
                FVector2D LoopOff(Rand.FRandRange(-3000.f,3000.f), Rand.FRandRange(-3000.f,3000.f));
                ConnectAttractorToGraph(G, Pos + LoopOff, Type, PickWidth(Type));
            }
            ++Connected;
        }
    }
}

// -----------------------------------------------------------------------------
//  STAGE 4 -- Tertiary lanes & alleys
// -----------------------------------------------------------------------------

void FOrganicStreetGenerator::Stage4_Tertiary(FOrganicStreetGraph& G)
{
    int32 ConnLanes = 0;
    int32 MaxTries  = Config.TertiaryAttractors * 4;

    for (int32 Try = 0; Try < MaxTries && ConnLanes < Config.TertiaryAttractors; Try++)
    {
        float R     = FMath::Sqrt(Rand.FRand()) * Config.TownRadius * 0.75f;
        float Angle = Rand.FRandRange(0.f, TWO_PI);
        FVector2D Pos(FMath::Cos(Angle) * R, FMath::Sin(Angle) * R);

        bool bCore = Pos.Size() < Config.TownRadius * Config.CoreRadiusFraction;
        EOrganicStreetType LaneType = bCore
            ? (Rand.FRand() < 0.4f ? EOrganicStreetType::Alley : EOrganicStreetType::Lane)
            : EOrganicStreetType::Lane;

        if (ConnectAttractorToGraph(G, Pos, LaneType, PickWidth(LaneType)))
            ++ConnLanes;
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
    Stage2_Primary  (Graph, GatePositions, BridgeCandidates, ChurchPos, KeepPos);
    Stage3_Secondary(Graph);
    Stage4_Tertiary (Graph);
    Graph.RemoveShortDangles(800.f);
    return Graph;
}
