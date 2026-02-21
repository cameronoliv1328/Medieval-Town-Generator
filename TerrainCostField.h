#pragma once

#include "CoreMinimal.h"
#include "MedievalCityData.h"

struct FTerrainCostField
{
    FMedievalTerrainQueryParams TerrainParams;
    FStreetGrowthParams StreetParams;
    FVector RiverNormal = FVector(1, 0, 0);
    FVector RiverPoint = FVector::ZeroVector;

    float Evaluate(const FVector& WorldPosition, float SlopeDeg) const;
    float RiverDistanceCost(const FVector& WorldPosition) const;
};
