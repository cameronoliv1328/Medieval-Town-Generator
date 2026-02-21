#include "TerrainCostField.h"

float FTerrainCostField::RiverDistanceCost(const FVector& WorldPosition) const
{
    const FVector2D P(WorldPosition.X - RiverPoint.X, WorldPosition.Y - RiverPoint.Y);
    const FVector2D N(RiverNormal.X, RiverNormal.Y);
    const float Dist = FMath::Abs(P.X * N.X + P.Y * N.Y);
    const float NearRiverFactor = FMath::Clamp(1.0f - Dist / 5000.0f, 0.0f, 1.0f);
    return NearRiverFactor * StreetParams.WaterPenalty;
}

float FTerrainCostField::Evaluate(const FVector& WorldPosition, float SlopeDeg) const
{
    const float SlopeRatio = FMath::Max(0.f, SlopeDeg / FMath::Max(1.f, TerrainParams.MaxAllowedSlopeDeg));
    const float SlopeCost = SlopeRatio * StreetParams.SlopePenalty;
    const float WaterCost = RiverDistanceCost(WorldPosition);
    const float ValleyBias = StreetParams.ValleyPreference * (WorldPosition.Z - TerrainParams.WaterLevelZ) / 1000.f;
    return 1.0f + SlopeCost + WaterCost + ValleyBias;
}
