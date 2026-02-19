#pragma once

#include "CoreMinimal.h"

namespace MTGTerrain
{
    // Deterministic hash/value noise helpers used by terrain sampling.
    float HashNoise(int32 X, int32 Y);
    float SmoothNoise(float X, float Y);
}
