// OrganicStreetGenerator.h
// -----------------------------------------------------------------------------
// Main 4-stage organic street generator.
// Replaces the symmetric BuildRadiocentricRoads() pipeline.
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

    float MinSpacingCore      = 2500.f;
    float MinSpacingOutskirts = 4000.f;
    float CoreRadiusFraction  = 0.40f;

    float PlazaChanceAt3Way   = 0.35f;
    float LoopChanceCore      = 0.10f;
    float LoopChanceOutskirts = 0.03f;

    float WidthNoiseFraction  = 0.15f;
    float TIntersectionOffset = 1100.f;

    int32 MaxBridges          = 2;
    int32 SecondaryAttractors = 60;
    int32 TertiaryAttractors  = 90;

    float AStarCellSize        = 500.f;
    float RDPEpsilonPrimary    = 400.f;
    float RDPEpsilonSecondary  = 250.f;

    bool bDebugDraw = false;
};

// -- Main generator ------------------------------------------------------------
class FOrganicStreetGenerator
{
public:
    FOrganicStreetGenerator(const FOrganicStreetConfig& InConfig,
                             const FOrganicTerrainQuery& InTerrain,
                             FRandomStream& InRand);

    /** Run the full 4-stage organic generator. Returns the completed graph. */
    FOrganicStreetGraph Generate(const TArray<FVector2D>& GatePositions,
                                  const TArray<FBridgeCandidate>& BridgeCandidates,
                                  FVector2D ChurchPos,
                                  FVector2D KeepPos);

    /** Expose bridge selection for external debug use */
    TArray<FBridgeCandidate> SelectBridges(
        const TArray<FBridgeCandidate>& Candidates) const;

private:
    // -- A* --------------------------------------------------------------------
    TArray<FVector2D> RouteAStar(FVector2D From, FVector2D To,
                                  float MaxGradeOverride = -1.f) const;
    float             CellCost(FVector2D From, FVector2D To) const;

    TArray<FVector2D> SimplifyRDP(const TArray<FVector2D>& Pts, float Epsilon) const;
    TArray<FVector2D> SmoothChaikin(const TArray<FVector2D>& Pts, int32 Passes = 2) const;
    TArray<FVector2D> RouteAndSmooth(FVector2D From, FVector2D To,
                                      float MaxGrade, float RDPEps) const;

    // -- Stages ----------------------------------------------------------------
    void Stage2_Primary  (FOrganicStreetGraph& G,
                          const TArray<FVector2D>& Gates,
                          const TArray<FBridgeCandidate>& Bridges,
                          FVector2D Church, FVector2D Keep);
    void Stage3_Secondary(FOrganicStreetGraph& G);
    void Stage4_Tertiary (FOrganicStreetGraph& G);

    // -- Helpers ---------------------------------------------------------------
    bool  ConnectAttractorToGraph(FOrganicStreetGraph& G, FVector2D Pos,
                                   EOrganicStreetType Type, float Width);
    bool  CheckIntersectionSpacing(const FOrganicStreetGraph& G, FVector2D Pos) const;
    bool  IsNearParallelToEdge(const FOrganicStreetGraph& G,
                                int32 EdgeIdx, FVector2D Dir) const;
    float DensityAt(FVector2D Pos) const;
    float PickWidth(EOrganicStreetType Type) const;

    // Grid helpers
    int32     GridW() const;
    int32     GridH() const;
    FVector2D CellToWorld(int32 X, int32 Y) const;
    bool      WorldToCell(FVector2D W, int32& X, int32& Y) const;
    int32     CellIdx(int32 X, int32 Y) const;

    FOrganicStreetConfig Config;
    FOrganicTerrainQuery Terrain;
    FRandomStream&       Rand;
};
