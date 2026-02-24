#pragma once

#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

// Forward declaration at global scope — must NOT be inside namespace MTGRoads
// or the compiler creates a distinct MTGRoads::AMedievalTownGenerator type
// that is incompatible with the real ::AMedievalTownGenerator.
class AMedievalTownGenerator;

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
        AMedievalTownGenerator* Gen,        // global-scope type, not MTGRoads::
        const FOrganicStreetGraph& Graph);
}
