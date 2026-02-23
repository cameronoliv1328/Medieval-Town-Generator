// OrganicIntersectionMesher.h
// ─────────────────────────────────────────────────────────────────────────────
// Builds intersection polygons and road-ribbon mesh data from the organic
// street graph. All geometry is flat (Z handled externally by ElevateRoadSplines).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

// Bug 3 fix: guard the API macro so the file compiles even when the
// module header hasn't been included first.
#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

class MEDIEVALTOWNGENERATOR_API FOrganicIntersectionMesher
{
public:
    /**
     * Build a convex-ish polygon around an intersection node.
     *
     * @param Node      The intersection node (must have >= 3 connected edges).
     * @param NodeIdx   Index of Node inside the Nodes array — used for reliable
     *                  edge-endpoint identification (Bug 5 fix).
     * @param Nodes     Full node array from FOrganicStreetGraph.
     * @param Edges     Full edge array from FOrganicStreetGraph.
     * @param OutPolygon Output polygon vertices in world XY (Z = 0).
     * @param PlazaScale Radius multiplier; > 1 widens into a plaza.
     * @return true if a valid polygon (>= 3 vertices) was produced.
     */
    static bool BuildIntersectionPolygon(
        const FOrganicStreetNode&        Node,
        int32                            NodeIdx,       // Bug 5 fix: index for identity check
        const TArray<FOrganicStreetNode>& Nodes,        // Bug 1 fix: FOrganicStreetNode
        const TArray<FOrganicStreetEdge>& Edges,        // Bug 2 fix: FOrganicStreetEdge
        TArray<FVector>&                 OutPolygon,
        float                            PlazaScale = 1.0f);

    /**
     * Build a quad-strip road ribbon mesh from one street edge.
     *
     * @param Edge          Edge to tessellate (uses Edge.Poly2D).
     * @param WidthScale    Multiplier applied on top of Edge.Width.
     * @param VertexSpacing World-units per UV tile along the road.
     * @param OutV / OutT / OutN / OutUV  Appended mesh data (not cleared).
     */
    static void BuildRoadRibbon(
        const FOrganicStreetEdge&  Edge,               // Bug 2 fix: FOrganicStreetEdge
        float                      WidthScale,
        float                      VertexSpacing,
        TArray<FVector>&           OutV,
        TArray<int32>&             OutT,
        TArray<FVector>&           OutN,
        TArray<FVector2D>&         OutUV);
};
