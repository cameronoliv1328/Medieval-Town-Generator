#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"
#include "MedievalParcelLayoutSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

/**
 * PCG layout reader node for town building parcel data.
 *
 * Consumes the parcel array produced by AMedievalTownGenerator::BuildParcelBoundary()
 * and emits a single "Parcels" point stream — one PCG point per burgage plot.
 *
 * Point transform:
 *   Location = WorldCenter (terrain-snapped lot centre)
 *   Rotation = Yaw aligned with the street-facing direction
 *
 * Density = WealthScore [0..1] — drives material tier selection in PCG:
 *   0.00–0.33  rough rubble / wattle-and-daub
 *   0.34–0.66  coursed fieldstone
 *   0.67–1.00  dressed ashlar
 *
 * Integer-enum attributes are stored as int32 so PCG filter nodes can compare
 * them against literal values without float precision issues.
 *
 * Full attribute list:
 *   LotIndex(int32), Ward(int32), FootprintType(int32), BuildingStyle(int32),
 *   RoofType(int32), NumFloors(int32), PartyWallMask(int32),
 *   FrontageWidth(float), LotDepth(float), FoundationDepth(float),
 *   GroundSlopeDeg(float), DistToRiver(float), DistToWall(float),
 *   DistToMarket(float), StreetFacingX(float), StreetFacingY(float),
 *   bFacesRiver(float), bIsCornerLot(float), bHasWing(float),
 *   WingSizeX(float), WingSizeY(float), WingYawOffset(float).
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalParcelLayoutSettings : public UObject
{
    GENERATED_BODY()
public:
    /** Actor reference used when the PCG component is not on the town generator. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Layout")
    TSoftObjectPtr<AActor> TownGeneratorActor;

    /**
     * When > 0, only emit parcels whose WealthScore is >= this threshold.
     * Useful for placing high-quality assets only on wealthy lots.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Layout|Filter",
              meta=(ClampMin="0.0", ClampMax="1.0"))
    float MinWealthScore = 0.f;

    /**
     * When set, only emit parcels belonging to this ward.
     * Set to 255 (or any value >= EMedievalWardType count) to disable filtering.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Layout|Filter")
    uint8 WardFilter = 255;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override
        { return FName(TEXT("PCG_MedievalParcelLayout")); }
    virtual FText GetDefaultNodeTitle() const override
        { return FText::FromString(TEXT("Medieval Parcel Layout")); }
    virtual EPCGSettingsType GetType() const override
        { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties()  const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
