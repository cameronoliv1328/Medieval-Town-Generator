// OrganicParcelGenerator.h
// -----------------------------------------------------------------------------
// Burgage-plot parcel subdivision.
//
// Replaces the old "cursor + random scatter" building placement with the
// historically grounded medieval pattern:
//   * Every street frontage is divided into narrow, deep burgage plots
//     (real-world plots were ~1-4 perches of frontage and 5-20 perches deep).
//   * Houses sit ON the frontage line (tiny setback in the core), gable to
//     the street, often sharing party walls -> continuous street walls.
//   * Plot interiors (rear yards) stay open, with occasional outbuildings,
//     which produces the classic dense-edge / hollow-block texture.
//   * Wealth, industry, and style are continuous fields (market/landmark
//     proximity + noise), not hard concentric rings or pie wedges, so
//     district transitions are gradual and organic.
//
// The class is engine-agnostic (CoreMinimal types only, no UObject), so it is
// compiled both by the UE module and the standalone harness in /Harness.
// -----------------------------------------------------------------------------
#pragma once
#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

// -- Building programme assigned to a lot ---------------------------------------
enum class EOrganicLotKind : uint8
{
    Cottage,
    TownHouse,
    GuildHall,
    Tavern,
    Warehouse,
    Blacksmith,
    Bakery,
    Stable,
    Outbuilding,
    Church,
    Keep
};

// -- One placed building + its burgage plot --------------------------------------
struct FOrganicParcelLot
{
    // Building box
    FVector2D Center = FVector2D::ZeroVector;   // local 2D centre of the building
    FVector2D Footprint = FVector2D(500.f, 400.f); // X = width along street, Y = depth
    float     YawDeg = 0.f;       // rotation; local +X runs along the street tangent
    FVector2D FacingDir = FVector2D(1.f, 0.f);  // unit vector, building front -> street

    EOrganicLotKind Kind = EOrganicLotKind::TownHouse;
    int32 Floors = 1;
    float Wealth = 0.5f;          // 0 = hovel, 1 = patrician

    bool  bRowHouse = false;      // shares party wall(s) with neighbours
    uint8 PartyWallMask = 0;      // bit0 = left shared, bit1 = right shared
    bool  bIsCornerLot = false;   // plot frontage adjoins an intersection
    bool  bFrontagePrimary = false;
    bool  bIsRearOutbuilding = false;

    // Burgage plot geometry (4 corners, front edge first, follows street side)
    TArray<FVector2D> PlotPolygon;
    FVector2D PlotSize = FVector2D(600.f, 2000.f);  // frontage x depth
    int32 FrontEdgeIndex = INDEX_NONE;              // street edge this lot fronts
};

// -- Terrain/river callbacks -------------------------------------------------------
struct FOrganicParcelTerrainQuery
{
    TFunction<float(FVector2D)>       GetHeight;
    TFunction<bool(FVector2D, float)> IsNearRiver;   // pos, extra clearance
    TFunction<float(FVector2D)>       DistToRiver;   // distance to river centreline
};

// -- Tunables ----------------------------------------------------------------------
struct FOrganicParcelConfig
{
    float TownRadius   = 18000.f;
    float WallFraction = 0.92f;          // plots must stay inside this radius fraction

    FVector2D MarketCenter = FVector2D::ZeroVector;
    FVector2D ChurchPos    = FVector2D::ZeroVector;
    FVector2D KeepPos      = FVector2D::ZeroVector;
    float MarketPlazaRadius = 1100.f;    // kept clear of plots; must be < market ring radius

    // Burgage proportions (cm). Real plots: narrow frontage, 3-8x deeper.
    float FrontageMin = 420.f;
    float FrontageMax = 860.f;
    float PlotDepthPrimary   = 2700.f;
    float PlotDepthSecondary = 2100.f;
    float PlotDepthLane      = 1500.f;
    float MinPlotDepth       = 330.f;

    // Occupancy grid
    float GridCellSize  = 130.f;
    float RoadClearance = 50.f;          // extra free band beside carriageways
    float RiverClearance = 150.f;

    // Urban texture
    float RowHouseChanceCore = 0.88f;    // party-wall rows near the market
    float RowHouseChanceEdge = 0.30f;
    float SetbackCore = 30.f;            // building front offset from plot front
    float SetbackEdge = 170.f;
    float RearOutbuildingChance = 0.45f;
    float QuayIndustryBand = 2400.f;     // river distance treated as working quay

    float MaxSlopeGrade = 0.22f;         // max dz/dxy across a plot
    int32 MaxLots = 6000;
};

// -- Generator ------------------------------------------------------------------------
class FOrganicParcelGenerator
{
public:
    FOrganicParcelGenerator(const FOrganicParcelConfig& InConfig,
                            const FOrganicParcelTerrainQuery& InTerrain,
                            FRandomStream& InRand);

    /** Subdivide every street frontage of the graph into burgage plots. */
    TArray<FOrganicParcelLot> Generate(const FOrganicStreetGraph& Graph);

    /** Continuous wealth field [0,1]; public so debug viewers can plot it. */
    float WealthAt(FVector2D Pos) const;

    /** Continuous urbanity field [0,1]: 1 = dense core, 0 = town edge. */
    float UrbanityAt(FVector2D Pos) const;

    /** Debug access to the occupancy grid (harness visualisation). */
    const TArray<uint8>& DebugGrid() const { return Grid; }
    int32 DebugGridN() const { return GridN; }
    float DebugGridHalf() const { return GridHalf; }

private:
    enum ECell : uint8
    {
        CellFree = 0,
        CellRoad,
        CellWater,
        CellOutside,
        CellPlaza,
        CellClaimed
    };

    // -- Occupancy grid ---------------------------------------------------------
    void  BuildGrid(const FOrganicStreetGraph& Graph);
    int32 CellIndex(FVector2D World) const;            // INDEX_NONE if off-grid
    bool  IsQuadFree(const FVector2D Quad[4]) const;
    void  ClaimQuad(const FVector2D Quad[4]);
    void  StampDisc(FVector2D World, float Radius, uint8 Value, bool bOnlyFree);

    /** Longest free run from Front along Normal, sampled on the grid (<= MaxDepth). */
    float ProbeDepth(FVector2D Front, FVector2D Normal, float MaxDepth) const;

    // -- Lot construction ---------------------------------------------------------
    /** bInfill = relaxed second pass: shallow hovel plots in leftover frontage. */
    void SubdivideEdgeFrontage(const FOrganicStreetGraph& Graph, int32 EdgeIdx,
                               int32 Side, bool bInfill,
                               TArray<FOrganicParcelLot>& OutLots);

    EOrganicLotKind PickKind(FVector2D Pos, float Wealth, bool bPrimaryFrontage);
    int32           PickFloors(EOrganicLotKind Kind, float Wealth, float Urbanity);

    FOrganicParcelConfig       Config;
    FOrganicParcelTerrainQuery Terrain;
    FRandomStream&             Rand;

    TArray<uint8> Grid;
    int32 GridN = 0;
    float GridHalf = 0.f;
};
