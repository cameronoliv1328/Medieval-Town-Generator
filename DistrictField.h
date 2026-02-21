#pragma once

#include "CoreMinimal.h"
#include "MedievalCityData.h"

struct FDistrictField
{
    FDistrictParams Params;
    FVector MarketCenter = FVector::ZeroVector;
    FVector KeepCenter = FVector::ZeroVector;
    FVector ChurchCenter = FVector::ZeroVector;
    FVector RiverPoint = FVector::ZeroVector;
    FVector RiverNormal = FVector(1,0,0);

    EMedievalWardType SampleDistrict(const FVector& Position, float TownRadius, const FRandomStream& Rand) const;
    float WealthAt(const FVector& Position, float TownRadius) const;
    float MixedUseAt(const FVector& Position, float TownRadius) const;
};
