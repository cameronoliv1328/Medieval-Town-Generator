#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"
#include "MedievalStreetNetworkSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalStreetNetworkSettings : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    FMedievalCitySeedParams SeedParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    FStreetGrowthParams StreetParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    int32 SecondaryAttractorCount = 60;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    bool bCreateCorePlazas = true;

    /** Organic street layout parameters */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street|Organic")
    FOrganicStreetParams OrganicParams;

    /** Minimum distance from river centreline that roads avoid crossing (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street|River")
    float RiverExclusionRadius = 800.f;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_OrganicStreetNetwork")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Organic Street Network")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Street Network")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
