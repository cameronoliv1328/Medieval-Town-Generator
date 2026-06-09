// OrganicParcelGenerator.cpp
// -----------------------------------------------------------------------------
// Burgage-plot subdivision along street frontages. See header for the design.
// -----------------------------------------------------------------------------
#include "OrganicParcelGenerator.h"

#if defined(MEDIEVAL_HARNESS)
#include <cstdio>
#endif

namespace
{
    int32 GStatDepthRej = 0, GStatQuadRej = 0, GStatSlopeRej = 0, GStatTried = 0;
    int32 GStatBlockType[8] = {0};

    float PolyLength(const TArray<FVector2D>& Pts)
    {
        float L = 0.f;
        for (int32 i = 0; i < Pts.Num() - 1; i++) L += (Pts[i + 1] - Pts[i]).Size();
        return L;
    }

    // Point + unit tangent at arc-length S along a polyline.
    void SampleAt(const TArray<FVector2D>& Pts, float S, FVector2D& OutP, FVector2D& OutTan)
    {
        OutP = Pts[0];
        OutTan = (Pts.Num() > 1 ? (Pts[1] - Pts[0]).GetSafeNormal() : FVector2D(1.f, 0.f));
        float Acc = 0.f;
        for (int32 i = 0; i < Pts.Num() - 1; i++)
        {
            const float SegLen = (Pts[i + 1] - Pts[i]).Size();
            if (Acc + SegLen >= S || i == Pts.Num() - 2)
            {
                const float T = (SegLen > 0.01f) ? FMath::Clamp((S - Acc) / SegLen, 0.f, 1.f) : 0.f;
                OutP = FMath::Lerp(Pts[i], Pts[i + 1], T);
                OutTan = (Pts[i + 1] - Pts[i]).GetSafeNormal();
                return;
            }
            Acc += SegLen;
        }
    }
}

// Harness-only diagnostic: positions of depth-rejected frontage probes.
TArray<FVector2D> GParcelDepthRejectPositions;

FOrganicParcelGenerator::FOrganicParcelGenerator(const FOrganicParcelConfig& InConfig,
                                                 const FOrganicParcelTerrainQuery& InTerrain,
                                                 FRandomStream& InRand)
    : Config(InConfig), Terrain(InTerrain), Rand(InRand)
{}

// =============================================================================
//  Continuous fields
// =============================================================================

float FOrganicParcelGenerator::WealthAt(FVector2D Pos) const
{
    const float R = Config.TownRadius;
    const float WMarket = FMath::Exp(-(Pos - Config.MarketCenter).Size() / (R * 0.40f));
    const float WChurch = FMath::Exp(-(Pos - Config.ChurchPos).Size() / (R * 0.30f));
    const float WKeep   = FMath::Exp(-(Pos - Config.KeepPos).Size()   / (R * 0.30f));
    const float Noise   = FMath::PerlinNoise2D(Pos * 0.00018f + FVector2D(31.7f, 11.3f)) * 0.30f;

    float Wealth = 0.16f + 0.60f * WMarket + 0.30f * FMath::Max(WChurch, WKeep) + Noise;

    // Working river banks drag wealth down (tanneries, smoke, flooding).
    if (Terrain.DistToRiver && Terrain.DistToRiver(Pos) < Config.QuayIndustryBand)
        Wealth -= 0.16f;

    return FMath::Clamp(Wealth, 0.f, 1.f);
}

float FOrganicParcelGenerator::UrbanityAt(FVector2D Pos) const
{
    const float R = Config.TownRadius;
    const float Core  = FMath::Exp(-(Pos - Config.MarketCenter).Size() / (R * 0.45f));
    const float Noise = FMath::PerlinNoise2D(Pos * 0.00012f + FVector2D(7.1f, 53.9f)) * 0.18f;
    const float EdgeFade = FMath::Clamp(1.f - Pos.Size() / (R * Config.WallFraction), 0.f, 1.f);
    return FMath::Clamp(Core + Noise + EdgeFade * 0.25f, 0.f, 1.f);
}

// =============================================================================
//  Occupancy grid
// =============================================================================

int32 FOrganicParcelGenerator::CellIndex(FVector2D World) const
{
    const int32 X = FMath::FloorToInt((World.X + GridHalf) / Config.GridCellSize);
    const int32 Y = FMath::FloorToInt((World.Y + GridHalf) / Config.GridCellSize);
    if (X < 0 || X >= GridN || Y < 0 || Y >= GridN) return INDEX_NONE;
    return Y * GridN + X;
}

void FOrganicParcelGenerator::StampDisc(FVector2D World, float Radius, uint8 Value, bool bOnlyFree)
{
    const float Cell = Config.GridCellSize;
    const int32 CR = FMath::CeilToInt(Radius / Cell);
    const int32 CX = FMath::FloorToInt((World.X + GridHalf) / Cell);
    const int32 CY = FMath::FloorToInt((World.Y + GridHalf) / Cell);
    for (int32 Y = CY - CR; Y <= CY + CR; Y++)
    {
        if (Y < 0 || Y >= GridN) continue;
        for (int32 X = CX - CR; X <= CX + CR; X++)
        {
            if (X < 0 || X >= GridN) continue;
            const FVector2D C(-GridHalf + (X + 0.5f) * Cell, -GridHalf + (Y + 0.5f) * Cell);
            if ((C - World).SizeSquared() > Radius * Radius) continue;
            const int32 I = Y * GridN + X;
            if (!bOnlyFree || Grid[I] == CellFree) Grid[I] = Value;
        }
    }
}

void FOrganicParcelGenerator::BuildGrid(const FOrganicStreetGraph& Graph)
{
    const float Cell = Config.GridCellSize;
    GridHalf = Config.TownRadius * 1.02f;
    GridN = FMath::CeilToInt(GridHalf * 2.f / Cell);
    Grid.Init((uint8)CellFree, GridN * GridN);

    const float WallLimit = Config.TownRadius * Config.WallFraction;

    for (int32 Y = 0; Y < GridN; Y++)
    {
        for (int32 X = 0; X < GridN; X++)
        {
            const FVector2D C(-GridHalf + (X + 0.5f) * Cell, -GridHalf + (Y + 0.5f) * Cell);
            const int32 I = Y * GridN + X;
            if (C.Size() > WallLimit)
            {
                Grid[I] = CellOutside;
            }
            else if (Terrain.IsNearRiver && Terrain.IsNearRiver(C, Config.RiverClearance))
            {
                Grid[I] = CellWater;
            }
            else if ((C - Config.MarketCenter).Size() < Config.MarketPlazaRadius)
            {
                Grid[I] = CellPlaza;
            }
        }
    }

    // Stamp carriageways (live edges only).
    for (const FOrganicStreetEdge& E : Graph.Edges)
    {
        if (E.NodeA < 0 || E.Poly2D.Num() < 2) continue;
        const float Radius = E.Width * 0.5f + Config.RoadClearance;
        const float Step = Cell * 0.6f;
        for (int32 i = 0; i < E.Poly2D.Num() - 1; i++)
        {
            const FVector2D A = E.Poly2D[i], B = E.Poly2D[i + 1];
            const float SegLen = (B - A).Size();
            const int32 N = FMath::Max(1, FMath::CeilToInt(SegLen / Step));
            for (int32 s = 0; s <= N; s++)
                StampDisc(FMath::Lerp(A, B, (float)s / N), Radius, (uint8)CellRoad, false);
        }
    }
}

bool FOrganicParcelGenerator::IsQuadFree(const FVector2D Quad[4]) const
{
    // Sample the quad interior on a lattice slightly finer than the grid.
    // The first row behind the frontage is skipped: it legitimately abuts the
    // road band and would otherwise reject every plot at grid resolution.
    const float Step = Config.GridCellSize * 0.8f;
    const FVector2D E01 = Quad[1] - Quad[0];
    const FVector2D E03 = Quad[3] - Quad[0];
    const float DepthLen = E03.Size();
    const float InsetFrac = FMath::Clamp(Config.GridCellSize * 1.2f / FMath::Max(DepthLen, 1.f), 0.f, 0.45f);
    const float UInsetFrac = FMath::Clamp(Config.GridCellSize * 0.9f / FMath::Max(E01.Size(), 1.f), 0.f, 0.40f);
    const int32 NU = FMath::Max(2, FMath::CeilToInt(E01.Size() / Step) + 1);
    const int32 NV = FMath::Max(2, FMath::CeilToInt(DepthLen / Step) + 1);
    for (int32 V = 0; V <= NV; V++)
    {
        const float TV = InsetFrac + (1.f - InsetFrac) * ((float)V / NV);
        for (int32 U = 0; U <= NU; U++)
        {
            const float TU = UInsetFrac + (1.f - 2.f * UInsetFrac) * ((float)U / NU);
            const FVector2D P = Quad[0] + E01 * TU + E03 * TV;
            const int32 I = CellIndex(P);
            if (I == INDEX_NONE || Grid[I] != CellFree) return false;
        }
    }
    return true;
}

void FOrganicParcelGenerator::ClaimQuad(const FVector2D Quad[4])
{
    const float Step = Config.GridCellSize * 0.8f;
    const FVector2D E01 = Quad[1] - Quad[0];
    const FVector2D E03 = Quad[3] - Quad[0];
    const int32 NU = FMath::Max(2, FMath::CeilToInt(E01.Size() / Step) + 1);
    const int32 NV = FMath::Max(2, FMath::CeilToInt(E03.Size() / Step) + 1);
    for (int32 V = 0; V <= NV; V++)
    {
        for (int32 U = 0; U <= NU; U++)
        {
            const FVector2D P = Quad[0] + E01 * ((float)U / NU) + E03 * ((float)V / NV);
            const int32 I = CellIndex(P);
            if (I != INDEX_NONE && Grid[I] == CellFree) Grid[I] = CellClaimed;
        }
    }
}

float FOrganicParcelGenerator::ProbeDepth(FVector2D Front, FVector2D Normal, float MaxDepth) const
{
    const float Step = Config.GridCellSize * 0.7f;
    // The band immediately behind the frontage may fall in road-stamped cells
    // at grid resolution; it is part of the plot, so start probing past it.
    float D = Config.GridCellSize * 1.2f;
    if (D >= MaxDepth) return 0.f;
    while (D + Step <= MaxDepth)
    {
        const int32 I = CellIndex(Front + Normal * (D + Step));
        if (I == INDEX_NONE || Grid[I] != CellFree)
        {
            GStatBlockType[(I == INDEX_NONE) ? 7 : FMath::Min((int32)Grid[I], 6)]++;
            break;
        }
        D += Step;
    }
    return D;
}

// =============================================================================
//  Kind / floors
// =============================================================================

EOrganicLotKind FOrganicParcelGenerator::PickKind(FVector2D Pos, float Wealth, bool bPrimaryFrontage)
{
    // Working waterfront: warehouses, smithies, dye yards.
    if (Terrain.DistToRiver && Terrain.DistToRiver(Pos) < Config.QuayIndustryBand &&
        Rand.FRand() < 0.65f)
    {
        return (Rand.FRand() < 0.65f) ? EOrganicLotKind::Warehouse : EOrganicLotKind::Blacksmith;
    }

    const float Roll = Rand.FRand();
    if (bPrimaryFrontage && Wealth > 0.62f)
    {
        if (Roll < 0.52f) return EOrganicLotKind::TownHouse;
        if (Roll < 0.66f) return EOrganicLotKind::Tavern;
        if (Roll < 0.78f) return EOrganicLotKind::GuildHall;
        if (Roll < 0.89f) return EOrganicLotKind::Bakery;
        return EOrganicLotKind::Blacksmith;
    }
    if (Wealth > 0.45f)
    {
        if (Roll < 0.56f) return EOrganicLotKind::TownHouse;
        if (Roll < 0.81f) return EOrganicLotKind::Cottage;
        if (Roll < 0.89f) return EOrganicLotKind::Bakery;
        return EOrganicLotKind::Tavern;
    }
    if (Roll < 0.68f) return EOrganicLotKind::Cottage;
    if (Roll < 0.82f) return EOrganicLotKind::TownHouse;
    if (Roll < 0.92f) return EOrganicLotKind::Stable;
    return EOrganicLotKind::Blacksmith;
}

int32 FOrganicParcelGenerator::PickFloors(EOrganicLotKind Kind, float Wealth, float Urbanity)
{
    switch (Kind)
    {
    case EOrganicLotKind::Cottage:     return 1;
    case EOrganicLotKind::Stable:      return 1;
    case EOrganicLotKind::Outbuilding: return 1;
    case EOrganicLotKind::Blacksmith:  return 1;
    case EOrganicLotKind::Bakery:      return (Wealth > 0.6f) ? 2 : 1;
    case EOrganicLotKind::Warehouse:   return Rand.RandRange(1, 2);
    case EOrganicLotKind::GuildHall:   return Rand.RandRange(2, 3);
    case EOrganicLotKind::Tavern:      return 2;
    case EOrganicLotKind::TownHouse:
    {
        // Tall narrow houses where land is scarce: core + wealth push storeys up.
        const float T = Wealth * 0.6f + Urbanity * 0.4f;
        if (T > 0.75f) return Rand.RandRange(2, 3);
        if (T > 0.45f) return Rand.RandRange(1, 2) + (Rand.FRand() < 0.3f ? 1 : 0);
        return Rand.RandRange(1, 2);
    }
    default: return 1;
    }
}

// =============================================================================
//  Frontage subdivision
// =============================================================================

void FOrganicParcelGenerator::SubdivideEdgeFrontage(const FOrganicStreetGraph& Graph,
                                                    int32 EdgeIdx, int32 Side,
                                                    bool bInfill,
                                                    TArray<FOrganicParcelLot>& OutLots)
{
    const FOrganicStreetEdge& Edge = Graph.Edges[EdgeIdx];
    const TArray<FVector2D>& Poly = Edge.Poly2D;
    const float TotalLen = PolyLength(Poly);

    const bool bPrimary = (Edge.StreetType == EOrganicStreetType::Primary);
    const bool bAlley   = (Edge.StreetType == EOrganicStreetType::Alley);

    float TierDepth = Config.PlotDepthLane;
    if (Edge.StreetType == EOrganicStreetType::Primary)        TierDepth = Config.PlotDepthPrimary;
    else if (Edge.StreetType == EOrganicStreetType::Secondary) TierDepth = Config.PlotDepthSecondary;

    // The infill pass squeezes shallow plots (hovels, stalls, encroachments)
    // into frontage the burgage pass could not use.
    if (bInfill) TierDepth *= 0.45f;

    // Junction throats are already protected by the crossing street's road
    // stamp in the grid, so only a small clearance is needed here. Many edges
    // are short junction-to-junction segments; a large clearance would strip
    // them of frontage entirely.
    const float EndClear = bAlley ? 350.f : Rand.FRandRange(110.f, 250.f);
    if (TotalLen < EndClear * 2.f + Config.FrontageMin) return;

    const float SideSign = (Side == 0) ? 1.f : -1.f;
    const float RoadHalf = Edge.Width * 0.5f + Config.RoadClearance + Config.GridCellSize * 0.9f;

    float Cursor = EndClear * Rand.FRandRange(0.8f, 1.4f);
    int32 PrevLotIdx = INDEX_NONE;     // previous row candidate on this side
    float PrevLotEnd = -1.e9f;

    while (Cursor < TotalLen - EndClear && OutLots.Num() < Config.MaxLots)
    {
        // ---- sample frontage segment ----------------------------------------
        FVector2D P0, Tan0;
        SampleAt(Poly, Cursor, P0, Tan0);

        const float Wealth   = WealthAt(P0);
        const float Urbanity = UrbanityAt(P0);

        // Narrow frontage where land is valuable, wider toward the edge.
        float Frontage = FMath::Lerp(Config.FrontageMin, Config.FrontageMax,
                                     FMath::Clamp(1.f - Urbanity * 0.85f, 0.f, 1.f));
        Frontage *= Rand.FRandRange(0.82f, 1.22f);
        if (bInfill) Frontage *= 0.72f;
        if (Cursor + Frontage > TotalLen - EndClear) break;

        FVector2D P1, Tan1;
        SampleAt(Poly, Cursor + Frontage, P1, Tan1);
        const FVector2D Tangent = ((P1 - P0).SizeSquared() > 1.f)
            ? (P1 - P0).GetSafeNormal()
            : Tan0;
        const FVector2D Normal = FVector2D(-Tangent.Y, Tangent.X) * SideSign;

        const FVector2D F0 = P0 + Normal * RoadHalf;
        const FVector2D F1 = P1 + Normal * RoadHalf;
        const FVector2D FrontMid = (F0 + F1) * 0.5f;

        // Alleys carry only occasional frontage (they're rear-access lanes).
        if (bAlley && Rand.FRand() < 0.45f)
        {
            Cursor += Frontage * Rand.FRandRange(1.2f, 2.2f);
            continue;
        }

        GStatTried++;
        // ---- probe plot depth -------------------------------------------------
        // Probe past the target so we can tell a roomy block from a tight one;
        // in tight corridors take ~60% of the free run, leaving the rest for
        // the plots facing the opposite street.
        const float DepthTarget = TierDepth * Rand.FRandRange(0.72f, 1.30f);
        const float ProbeMax = DepthTarget * 1.7f;
        // Side rays are inset by a grid cell so they don't sample the cells
        // claimed by the abutting neighbour plot (row houses share that line).
        const FVector2D SideInset = Tangent * Config.GridCellSize;
        const float D0 = ProbeDepth(F0 + SideInset, Normal, ProbeMax);
        const float DM = ProbeDepth(FrontMid, Normal, ProbeMax);
        const float D1 = ProbeDepth(F1 - SideInset, Normal, ProbeMax);
        const float Avail = FMath::Min3(D0, DM, D1);
        // Never claim the whole corridor: leave the lesser of 38% or 12m of
        // free run behind the plot so the street facing us keeps frontage.
        const float FairShare = FMath::Max(Avail * 0.62f, Avail - 1200.f);
        const float Depth = FMath::Min(DepthTarget, FairShare);

        // Core blocks were packed nearly solid: tolerate much shallower plots
        // (house + token yard) where urbanity is high.
        float MinDepthEff = Config.MinPlotDepth * FMath::Lerp(1.f, 0.55f, Urbanity);
        if (bInfill) MinDepthEff *= 0.62f;
        if (Depth < MinDepthEff)
        {
            GStatDepthRej++;
            GParcelDepthRejectPositions.Add(FrontMid);
            Cursor += FMath::Max(Frontage * 0.45f, 240.f);
            PrevLotIdx = INDEX_NONE;
            continue;
        }

        FVector2D Quad[4] = { F0, F1, F1 + Normal * Depth, F0 + Normal * Depth };
        if (!IsQuadFree(Quad))
        {
            GStatQuadRej++;
            Cursor += FMath::Max(Frontage * 0.45f, 240.f);
            PrevLotIdx = INDEX_NONE;
            continue;
        }

        // ---- terrain check ------------------------------------------------------
        if (Terrain.GetHeight)
        {
            const float HF = Terrain.GetHeight(FrontMid);
            const float HB = Terrain.GetHeight(FrontMid + Normal * Depth);
            const float HL = Terrain.GetHeight(F0);
            const float HR = Terrain.GetHeight(F1);
            if (FMath::Abs(HB - HF) / FMath::Max(Depth, 1.f) > Config.MaxSlopeGrade ||
                FMath::Abs(HR - HL) / FMath::Max(Frontage, 1.f) > Config.MaxSlopeGrade)
            {
                GStatSlopeRej++;
                Cursor += FMath::Max(Frontage * 0.6f, 300.f);
                PrevLotIdx = INDEX_NONE;
                continue;
            }
        }

        ClaimQuad(Quad);

        // ---- building on the frontage -------------------------------------------
        const float RowChance = FMath::Lerp(Config.RowHouseChanceEdge,
                                            Config.RowHouseChanceCore, Urbanity);
        const bool bRow = Rand.FRand() < RowChance;
        const bool bAdjacent = (PrevLotIdx != INDEX_NONE) &&
                               (Cursor - PrevLotEnd) < 60.f;

        const float SideGap = bRow ? Rand.FRandRange(0.f, 25.f) : Rand.FRandRange(60.f, 220.f);
        const float BWidth = FMath::Max(Frontage - SideGap, 280.f);

        const float Setback = FMath::Lerp(Config.SetbackEdge, Config.SetbackCore, Urbanity)
                            * Rand.FRandRange(0.5f, 1.5f);
        float BDepth = FMath::Clamp(Depth * Rand.FRandRange(bInfill ? 0.55f : 0.42f, bInfill ? 0.85f : 0.68f),
                                    bInfill ? 220.f : 300.f, 1750.f);
        // Tight plots: let the house swallow the yard rather than vanish.
        BDepth = FMath::Min(BDepth, Depth - Setback - FMath::Lerp(80.f, 20.f, Urbanity));
        // Keep house proportions sane (long ranges are for warehouses only).
        BDepth = FMath::Min(BDepth, BWidth * 2.4f);
        if (BDepth < (bInfill ? 190.f : 240.f))
        {
            Cursor += Frontage + 80.f;
            PrevLotIdx = INDEX_NONE;
            continue;
        }

        FOrganicParcelLot Lot;
        Lot.Center    = FrontMid + Normal * (Setback + BDepth * 0.5f);
        Lot.Footprint = FVector2D(BWidth, BDepth);
        // Local +Y points away from the street (door face is local -Y).
        Lot.FacingDir = -Normal;
        Lot.YawDeg    = FMath::RadiansToDegrees(FMath::Atan2(-Normal.X, Normal.Y));
        Lot.Wealth    = Wealth;
        Lot.bRowHouse = bRow;
        Lot.bFrontagePrimary = bPrimary;
        Lot.FrontEdgeIndex = EdgeIdx;
        Lot.Kind   = PickKind(FrontMid, Wealth, bPrimary);
        Lot.Floors = PickFloors(Lot.Kind, Wealth, Urbanity);
        Lot.PlotSize = FVector2D(Frontage, Depth);
        Lot.PlotPolygon = { Quad[0], Quad[1], Quad[2], Quad[3] };
        Lot.bIsCornerLot = (Cursor < EndClear * 1.8f) ||
                           (Cursor + Frontage > TotalLen - EndClear * 1.8f);

        // Party walls with the previous lot in a continuous row.
        if (bRow && bAdjacent && OutLots.IsValidIndex(PrevLotIdx) &&
            OutLots[PrevLotIdx].bRowHouse)
        {
            Lot.PartyWallMask |= 0x01;                 // our left wall is shared
            OutLots[PrevLotIdx].PartyWallMask |= 0x02; // their right wall is shared
        }

        const int32 NewIdx = OutLots.Add(Lot);

        // ---- rear outbuilding ------------------------------------------------------
        const float RearSpace = Depth - Setback - BDepth;
        if (!bInfill && RearSpace > 850.f && Rand.FRand() < Config.RearOutbuildingChance)
        {
            FOrganicParcelLot Barn;
            const float BW = Rand.FRandRange(180.f, FMath::Min(340.f, BWidth * 0.8f));
            const float BD = Rand.FRandRange(180.f, 320.f);
            const float LateralJitter = Rand.FRandRange(-0.5f, 0.5f) * (Frontage - BW) * 0.6f;
            Barn.Center = FrontMid
                        + Normal * (Depth - BD * 0.5f - Rand.FRandRange(60.f, 200.f))
                        + Tangent * LateralJitter;
            Barn.Footprint = FVector2D(BW, BD);
            Barn.FacingDir = -Normal;
            Barn.YawDeg = Lot.YawDeg + Rand.FRandRange(-22.f, 22.f);
            Barn.Kind = EOrganicLotKind::Outbuilding;
            Barn.Floors = 1;
            Barn.Wealth = Wealth;
            Barn.bIsRearOutbuilding = true;
            Barn.FrontEdgeIndex = EdgeIdx;
            OutLots.Add(Barn);
        }

        PrevLotIdx = NewIdx;
        PrevLotEnd = Cursor + Frontage;

        // ---- advance ------------------------------------------------------------------
        float Gap = bRow ? Rand.FRandRange(0.f, 40.f)
                         : Rand.FRandRange(50.f, 220.f) * (1.f + (1.f - Urbanity) * 0.8f);
        Cursor += Frontage + Gap;
        if (!bRow) PrevLotIdx = INDEX_NONE;
    }
}

// =============================================================================
//  Generate
// =============================================================================

TArray<FOrganicParcelLot> FOrganicParcelGenerator::Generate(const FOrganicStreetGraph& Graph)
{
    TArray<FOrganicParcelLot> Lots;
    GParcelDepthRejectPositions.Empty();
    BuildGrid(Graph);

    // Subdivide primaries first so the best frontage goes to the high street,
    // then work down the hierarchy into the remaining space.
    const EOrganicStreetType TierOrder[4] = {
        EOrganicStreetType::Primary, EOrganicStreetType::Secondary,
        EOrganicStreetType::Lane,    EOrganicStreetType::Alley };

    GStatDepthRej = GStatQuadRej = GStatSlopeRej = GStatTried = 0;
    for (const EOrganicStreetType Tier : TierOrder)
    {
        for (int32 EdgeIdx = 0; EdgeIdx < Graph.Edges.Num(); EdgeIdx++)
        {
            const FOrganicStreetEdge& E = Graph.Edges[EdgeIdx];
            if (E.NodeA < 0 || E.Poly2D.Num() < 2) continue;
            if (E.StreetType != Tier || E.bIsBridge) continue;

            // Both sides of the street, random order for variety.
            const int32 FirstSide = (Rand.FRand() < 0.5f) ? 0 : 1;
            SubdivideEdgeFrontage(Graph, EdgeIdx, FirstSide, false, Lots);
            SubdivideEdgeFrontage(Graph, EdgeIdx, 1 - FirstSide, false, Lots);

            if (Lots.Num() >= Config.MaxLots) return Lots;
        }
    }

    // Relaxed infill: hovels and encroachments wedged into whatever frontage
    // the burgage pass left over (skip alleys -- their rears stay open).
    for (int32 EdgeIdx = 0; EdgeIdx < Graph.Edges.Num(); EdgeIdx++)
    {
        const FOrganicStreetEdge& E = Graph.Edges[EdgeIdx];
        if (E.NodeA < 0 || E.Poly2D.Num() < 2 || E.bIsBridge) continue;
        if (E.StreetType == EOrganicStreetType::Alley) continue;
        SubdivideEdgeFrontage(Graph, EdgeIdx, 0, true, Lots);
        SubdivideEdgeFrontage(Graph, EdgeIdx, 1, true, Lots);
        if (Lots.Num() >= Config.MaxLots) break;
    }
#if defined(MEDIEVAL_HARNESS)
    std::printf("  parcels: %d tried, %d depth-rej, %d quad-rej, %d slope-rej\n",
                GStatTried, GStatDepthRej, GStatQuadRej, GStatSlopeRej);
    std::printf("  probe blockers: road=%d water=%d outside=%d plaza=%d claimed=%d offgrid=%d\n",
                GStatBlockType[1], GStatBlockType[2], GStatBlockType[3],
                GStatBlockType[4], GStatBlockType[5], GStatBlockType[7]);
#endif
    return Lots;
}
