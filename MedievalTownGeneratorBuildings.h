#pragma once

#include "CoreMinimal.h"

namespace MTGBuildings
{
    // Computes a roof height from the current footprint and pitch.
    float ComputeRoofHeight(float Width, float Depth, float PitchAngleDeg);
}
