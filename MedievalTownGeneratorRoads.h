#pragma once

#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

namespace MTGRoads
{
    // Utility used when adding organic offsets to road spline points.
    FVector2D Perpendicular2D(const FVector2D& Dir);

    /**
     * Convert a completed FOrganicStreetGraph into AMedievalTownGenerator::RoadNodes
     * and RoadEdges, preserving full compatibility with the existing mesh pipeline.
     * WorldPoints are populated with 2D positions (Z=0); ElevateRoadSplines() fills Z.
     */
    void ApplyOrganicGraphToTownGenerator(
        class AMedievalTownGenerator* Gen,
        const FOrganicStreetGraph& Graph);
}
