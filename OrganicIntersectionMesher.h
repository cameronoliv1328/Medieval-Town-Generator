#pragma once

#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

class MEDIEVALTOWNGENERATOR_API FOrganicIntersectionMesher
{
public:
    static bool BuildIntersectionPolygon(
        const FStreetNode& Node,
        const TArray<FStreetNode>& Nodes,
        const TArray<FStreetEdge>& Edges,
        TArray<FVector>& OutPolygon,
        float PlazaScale = 1.0f);

    static void BuildRoadRibbon(
        const FStreetEdge& Edge,
        float WidthScale,
        float VertexSpacing,
        TArray<FVector>& OutV,
        TArray<int32>& OutT,
        TArray<FVector>& OutN,
        TArray<FVector2D>& OutUV);
};
