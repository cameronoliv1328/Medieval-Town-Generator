#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGCompat.h"
#include "MedievalCityData.h"
#include "MedievalStreetNetworkSettings.generated.h"

UCLASS(BlueprintType, ClassGroup=(Procedural))
class UMedievalStreetNetworkSettings : public MEDIEVAL_PCG_SETTINGS_PARENT
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

#if MEDIEVAL_HAS_PCG
    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalStreetNetwork")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Street Network")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
