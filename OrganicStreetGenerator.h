// OrganicStreetGenerator.h
// -----------------------------------------------------------------------------
// Organic street generator, v2 (block-growth).
//
// Stage 2  Primary routes:  terrain-aware A* from gates/bridges into an
//          irregular market-square ring. Later routes MERGE into earlier ones
//          when they come close, producing trunk roads with Y-junctions
//          instead of a radial star at the market.
// Stage 3  Block growth:    streets sprout quasi-perpendicularly from
//          existing frontages and grow step-by-step with curvature noise and
//          terrain deflection until they strike another street (T-junction,
//          closing a block), snap to a nearby node, or stop as a dead-end --
//          the Parish/Mueller local-constraint scheme that real organic
//          city generators use. Repeated for secondary -> lane -> alley
//          tiers, so irregular blocks emerge instead of fishbone spurs.
// -----------------------------------------------------------------------------
#pragma once
#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"
#include "MedievalCityData.h"

// -- Terrain query interface ---------------------------------------------------
struct FOrganicTerrainQuery
{
    TFunction<float(FVector2D)>       GetHeight;          // terrain Z at local XY
    TFunction<bool(FVector2D, float)> IsNearRiver;        // true if inside river band
    TFunction<float(FVector2D)>       BridgeSuitability;  // 0=bad, 1=ideal bridge spot
    float MaxGrade         = 0.10f;
    float SlopePenalty     = 2.4f;
    float WaterPenalty     = 5.0f;
    float ValleyPreference = 0.2f;
};

// -- Bridge candidate ---------------------------------------------------------
struct FBridgeCandidate
{
    FVector2D Position;
    float     Quality     = 0.5f;
    FVector2D ApproachDir = FVector2D(1.f, 0.f);
};

// -- Generation config ---------------------------------------------------------
struct FOrganicStreetConfig
{
    float TownRadius       = 18000.f;
    FVector2D MarketCenter = FVector2D::ZeroVector;

    // Widths (cm)
    float PrimaryWidthMin   = 600.f;
    float PrimaryWidthMax   = 1000.f;
    float SecondaryWidthMin = 400.f;
    float SecondaryWidthMax = 700.f;
    float LaneWidthMin      = 200.f;
    float LaneWidthMax      = 400.f;
    float AlleyWidthMin     = 150.f;
    float AlleyWidthMax     = 250.f;

    float MaxGradePrimary   = 0.10f;
    float MaxGradeSecondary = 0.12f;
    float MaxGradeLane      = 0.15f;

    // -- Block growth ----------------------------------------------------------
    /** Target spacing between street junctions in the dense core (cm). */
    float BlockSpacingCore  = 2700.f;
    /** Target junction spacing near the walls (cm). */
    float BlockSpacingEdge  = 4600.f;
    /** New routes merge into an existing primary within this range (cm). */
    float MergeDistance     = 1700.f;
    /** Growing street tip snaps onto an existing node within this range (cm). */
    float SnapDistance      = 620.f;
    /** Streets stop growing past this fraction of TownRadius. */
    float GrowthLimitFraction = 0.93f;
    /** Per-step heading noise (radians) for growing streets. */
    float StepHeadingNoise  = 0.15f;

    /** Probability a max-length street is kept as a dead-end, by tier. */
    float DeadEndChanceSecondary = 0.28f;
    float DeadEndChanceLane      = 0.55f;
    float DeadEndChanceAlley     = 0.80f;

    // -- Market square -----------------------------------------------------------
    /** Radius of the irregular market square ring (cm). */
    float PlazaRadius = 1900.f;

    float WidthNoiseFraction  = 0.15f;

    int32 MaxBridges          = 2;

    float AStarCellSize        = 500.f;
    float RDPEpsilonPrimary    = 280.f;
    float RDPEpsilonSecondary  = 200.f;

    bool bDebugDraw = false;
};

// -- Main generator ------------------------------------------------------------
class FOrganicStreetGenerator
{
public:
    FOrganicStreetGenerator(const FOrganicStreetConfig& InConfig,
                             const FOrganicTerrainQuery& InTerrain,
                             FRandomStream& InRand);

    /** Run the full organic generator. Returns the completed graph. */
    FOrganicStreetGraph Generate(const TArray<FVector2D>& GatePositions,
                                  const TArray<FBridgeCandidate>& BridgeCandidates,
                                  FVector2D ChurchPos,
                                  FVector2D KeepPos);

    /** Expose bridge selection for external debug use */
    TArray<FBridgeCandidate> SelectBridges(
        const TArray<FBridgeCandidate>& Candidates) const;

    /** Resolved market-square centre (may be nudged off a bridgehead). */
    FVector2D GetPlazaCenter() const { return PlazaCenter; }

private:
    // -- A* --------------------------------------------------------------------
    TArray<FVector2D> RouteAStar(FVector2D From, FVector2D To,
                                  float MaxGradeOverride = -1.f) const;
    float             CellCost(FVector2D From, FVector2D To) const;

    TArray<FVector2D> SimplifyRDP(const TArray<FVector2D>& Pts, float Epsilon) const;
    TArray<FVector2D> SmoothChaikin(const TArray<FVector2D>& Pts, int32 Passes = 2) const;
    TArray<FVector2D> RouteAndSmooth(FVector2D From, FVector2D To,
                                      float MaxGrade, float RDPEps) const;

    // -- Stage 2: primary trunks + market square --------------------------------
    void  Stage2_Primary(FOrganicStreetGraph& G,
                         const TArray<FVector2D>& Gates,
                         const TArray<FBridgeCandidate>& Bridges,
                         FVector2D Church, FVector2D Keep);
    void  BuildMarketSquare(FOrganicStreetGraph& G);
    int32 NearestPlazaNode(const FOrganicStreetGraph& G, FVector2D From) const;

    /**
     * Route From toward the target node, merging into an existing edge if the
     * path passes within MergeDistance of one (creates the trunk-road Ys).
     * Returns the node the route actually terminated at.
     */
    int32 ConnectWithMerge(FOrganicStreetGraph& G, int32 FromNode, int32 TargetNode,
                           bool bIsBridgeRoute);

    /** Append a primary route, splitting it so only river crossings are bridges. */
    void  AddPrimaryRoute(FOrganicStreetGraph& G, int32 FromNode, int32 EndNode,
                          TArray<FVector2D>&& Path);

    // -- Stage 3: block growth ----------------------------------------------------
    struct FGrowthPass
    {
        EOrganicStreetType GrowType   = EOrganicStreetType::Secondary;
        float SeedSpacingMult         = 1.0f;   // x local block spacing
        float MaxLenMult              = 1.5f;   // x local block spacing
        float MinLength               = 1500.f;
        float DeadEndChance           = 0.3f;
        float DensityGain             = 1.0f;   // accept seed if rand < density*gain
        bool  bSeedPrimary            = true;
        bool  bSeedSecondary          = false;
        bool  bSeedLane               = false;
    };

    void Stage3_GrowBlocks(FOrganicStreetGraph& G);
    void RunGrowthPass(FOrganicStreetGraph& G, const FGrowthPass& Pass);
    bool GrowStreet(FOrganicStreetGraph& G, FVector2D SeedPos, FVector2D SeedDir,
                    int32 SeedEdge, const FGrowthPass& Pass);

    // -- Helpers ----------------------------------------------------------------
    float DensityAt(FVector2D Pos) const;
    float BlockSpacingAt(FVector2D Pos) const;
    float PickWidth(EOrganicStreetType Type) const;
    bool  IsNearJunction(const FOrganicStreetGraph& G, FVector2D Pos, float Radius) const;

    // Grid helpers
    int32     GridW() const;
    int32     GridH() const;
    FVector2D CellToWorld(int32 X, int32 Y) const;
    bool      WorldToCell(FVector2D W, int32& X, int32& Y) const;
    int32     CellIdx(int32 X, int32 Y) const;

    FOrganicStreetConfig Config;
    FOrganicTerrainQuery Terrain;
    FRandomStream&       Rand;

    FVector2D            PlazaCenter = FVector2D::ZeroVector;
    TArray<int32>        PlazaNodes;        // market-square ring nodes
    TArray<FVector2D>    BridgePoints;      // selected bridgeheads (density boost)
};
