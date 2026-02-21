#pragma once

#include "CoreMinimal.h"
#include "PCGSettings.h"
#include "MedievalCityData.h"
#include "MedievalBuildingsSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalBuildingsSettings : public UPCGSettings
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Buildings")
    FMedievalCitySeedParams SeedParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Buildings")
    FBuildingFootprintParams BuildingParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Buildings")
    FParcelParams ParcelParams;

    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalBuildings")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Buildings")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
};
