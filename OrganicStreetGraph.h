// OrganicStreetGraph.h
// -----------------------------------------------------------------------------
// Rich street graph used by the organic generator.
// Replaces the old FStreetNode/FStreetEdge structs with richer types that
// carry 2D poly-lines, surface tags, importance values, and node type flags.
// FRoadNode / FRoadEdge (mesh pipeline) are populated afterward via
// ApplyOrganicGraphToTownGenerator().
// -----------------------------------------------------------------------------
#pragma once
#include "CoreMinimal.h"

// -- Street tier ---------------------------------------------------------------
enum class EOrganicStreetType : uint8
{
    Primary   = 0,   // gate?market, bridge?market, market?landmark
    Secondary = 1,   // district streets, partial loops
    Lane      = 2,   // back lanes, short connectors
    Alley     = 3    // narrow service alleys, dead-ends OK
};

// -- Surface material tag ------------------------------------------------------
enum class ESurfaceTag : uint8
{
    PavedStone,
    CompactedDirt,
    DirtMud,
    BridgeStone,
    BridgeWood
};

// -- Node ----------------------------------------------------------------------
struct FOrganicStreetNode
{
    FVector2D     Position      = FVector2D::ZeroVector;
    TArray<int32> ConnectedEdges;
    float         Importance    = 0.5f;   // 0-1; drives width/dressing choices

    bool bIsGate       = false;
    bool bIsMarket     = false;
    bool bIsBridgeNode = false;
    bool bIsLandmark   = false;   // church, keep, docks ...
    bool bIsPlaza      = false;

    int32 NodeIndex    = -1;
};

// -- Edge ----------------------------------------------------------------------
struct FOrganicStreetEdge
{
    int32  NodeA = -1;
    int32  NodeB = -1;

    EOrganicStreetType StreetType = EOrganicStreetType::Secondary;
    ESurfaceTag        Surface    = ESurfaceTag::CompactedDirt;

    float Width       = 500.f;  // centreline-to-centreline (cm)
    float TargetSpeed = 1.0f;   // NPC speed multiplier

    bool bIsBridge    = false;
    bool bIsPlazaEdge = false;

    TArray<FVector2D> Poly2D;   // smoothed XY polyline; Z added by ElevateRoadSplines
};

// -- Graph ---------------------------------------------------------------------
struct FOrganicStreetGraph
{
    TArray<FOrganicStreetNode> Nodes;
    TArray<FOrganicStreetEdge> Edges;

    // -- Mutation --------------------------------------------------------------
    int32 AddNode(FVector2D Pos, bool bAnchor = false);
    int32 AddEdge(int32 A, int32 B, EOrganicStreetType Type, float Width,
                  TArray<FVector2D>&& Poly);

    /** Split an edge at parameter T (0..1), insert new node, return its index. */
    int32 SplitEdge(int32 EdgeIdx, float T);

    // -- Queries ---------------------------------------------------------------
    int32 FindNearestNode(FVector2D Pos, float MaxDist = 1e9f) const;

    /** Returns edge index; OutT is parameter along edge poly; OutClosest is world pt */
    int32 FindNearestEdgePoint(FVector2D Pos,
                                float& OutT,
                                FVector2D& OutClosest,
                                float MaxDist = 1e9f) const;

    /** 2D segment-segment intersection (shared endpoints not considered) */
    static bool SegmentsIntersect2D(FVector2D A, FVector2D B,
                                     FVector2D C, FVector2D D,
                                     FVector2D* OutPt = nullptr);

    /** Would adding straight edge A->B create a new intersection with the graph? */
    bool WouldSelfIntersect(FVector2D A, FVector2D B) const;

    // -- Validation / cleanup --------------------------------------------------
    /** Remove dangling stubs shorter than MinLength whose endpoint degree == 1 */
    void RemoveShortDangles(float MinLength);

    /** Ensure no edge references an out-of-range node; returns true if clean */
    bool Validate() const;
};
