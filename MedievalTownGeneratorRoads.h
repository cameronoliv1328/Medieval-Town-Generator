#pragma once

#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

// Forward declaration at global scope.
class AMedievalTownGenerator;

namespace MTGRoads
{
    // Utility used when adding organic offsets to road spline points.
    FVector2D Perpendicular2D(const FVector2D& Dir);

    // NOTE: ApplyOrganicGraphToTownGenerator was removed from this namespace.
    // It accesses private members (RoadNodes, RoadEdges) so it is now a
    // private member function of AMedievalTownGenerator — see the class body.
}
