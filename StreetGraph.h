#pragma once

#include "CoreMinimal.h"
#include "MedievalCityData.h"
#include "StreetGraph.generated.h"

USTRUCT()
struct FStreetNode
{
    GENERATED_BODY()

    UPROPERTY()
    int32 Id = INDEX_NONE;

    UPROPERTY()
    FVector Position = FVector::ZeroVector;

    UPROPERTY()
    bool bAnchor = false;
};

USTRUCT()
struct FStreetSegment
{
    GENERATED_BODY()

    UPROPERTY()
    int32 A = INDEX_NONE;

    UPROPERTY()
    int32 B = INDEX_NONE;

    UPROPERTY()
    EMedievalStreetType Type = EMedievalStreetType::Secondary;

    UPROPERTY()
    float Width = 400.f;

    UPROPERTY()
    float Importance = 0.f;

    UPROPERTY()
    TArray<FVector> Polyline;
};

USTRUCT()
struct FStreetGraph
{
    GENERATED_BODY()

    UPROPERTY()
    TArray<FStreetNode> Nodes;

    UPROPERTY()
    TArray<FStreetSegment> Segments;

    int32 AddNode(const FVector& Position, bool bAnchor = false);
    int32 AddSegment(int32 A, int32 B, EMedievalStreetType Type, float Width, float Importance);
    bool HasNearbyIntersection(const FVector& Position, float MinSpacing) const;
};
