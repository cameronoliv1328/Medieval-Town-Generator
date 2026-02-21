#pragma once

#include "CoreMinimal.h"
#include "PCGSettings.h"
#include "MedievalCityData.h"
#include "MedievalParcelsSettings.generated.h"

UCLASS(BlueprintType, ClassGroup=(Procedural))
class UMedievalParcelsSettings : public UPCGSettings
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    FMedievalCitySeedParams SeedParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    FParcelParams ParcelParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    FDistrictParams DistrictParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float CoreRadiusFactor = 0.38f;

    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalParcels")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Parcels")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
};
