#pragma once

#include "CoreMinimal.h"

namespace MedievalPolygonUtils
{
    bool IsSimplePolygon(const TArray<FVector2D>& Poly);
    float PolygonArea(const TArray<FVector2D>& Poly);
    TArray<FVector2D> JitterPolygon(const TArray<FVector2D>& Poly, FRandomStream& Rand, float Amount);
    FString EncodePolygon(const TArray<FVector2D>& Poly);
    TArray<FVector2D> MakeRectangle(const FVector2D& Center, const FVector2D& Size, float YawDeg);
}
