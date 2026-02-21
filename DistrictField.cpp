#include "DistrictField.h"

EMedievalWardType FDistrictField::SampleDistrict(const FVector& Position, float TownRadius, const FRandomStream& Rand) const
{
    const float DistToMarket = FVector::Dist2D(Position, MarketCenter);
    const float Core = FMath::Clamp(1.f - DistToMarket / (TownRadius * 0.35f), 0.f, 1.f);
    const float Noise = FMath::PerlinNoise2D(FVector2D(Position.X, Position.Y) * Params.NoiseScale) * Params.NoiseAmplitude;

    const float RiverSigned = FVector2D::DotProduct(FVector2D(Position - RiverPoint), FVector2D(RiverNormal));
    const bool bRiverSide = FMath::Abs(RiverSigned) < TownRadius * 0.3f;

    if (Core + Noise > 0.8f)
    {
        return EMedievalWardType::MarketWard;
    }
    if (bRiverSide && Core < 0.8f)
    {
        return RiverSigned > 0 ? EMedievalWardType::DocksWard : EMedievalWardType::CraftsWard;
    }

    const float KeepDist = FVector::Dist2D(Position, KeepCenter);
    if (KeepDist < TownRadius * 0.25f)
    {
        return EMedievalWardType::NobleWard;
    }

    const float ChurchDist = FVector::Dist2D(Position, ChurchCenter);
    if (ChurchDist < TownRadius * 0.18f)
    {
        return EMedievalWardType::ChurchWard;
    }

    if (DistToMarket > TownRadius * 0.7f)
    {
        return EMedievalWardType::Outskirts;
    }

    return (Rand.FRand() < 0.45f) ? EMedievalWardType::PoorWard : EMedievalWardType::ResidentialWard;
}

float FDistrictField::WealthAt(const FVector& Position, float TownRadius) const
{
    const float MarketInfluence = 1.f - FVector::Dist2D(Position, MarketCenter) / (TownRadius * 0.8f);
    const float KeepInfluence = 1.f - FVector::Dist2D(Position, KeepCenter) / (TownRadius * 0.6f);
    return FMath::Clamp(0.15f + MarketInfluence * 0.55f + KeepInfluence * 0.45f, 0.f, 1.f);
}

float FDistrictField::MixedUseAt(const FVector& Position, float TownRadius) const
{
    const float MarketInfluence = 1.f - FVector::Dist2D(Position, MarketCenter) / (TownRadius * 0.5f);
    return FMath::Clamp(MarketInfluence, 0.f, 1.f);
}
