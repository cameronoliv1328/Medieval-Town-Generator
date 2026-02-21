#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGCompat.h"
#include "MedievalCityData.h"
#include "MedievalCityMacroSettings.generated.h"

UCLASS(BlueprintType, ClassGroup=(Procedural))
class UMedievalCityMacroSettings : public MEDIEVAL_PCG_SETTINGS_PARENT
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="City")
    FMedievalCitySeedParams SeedParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="City")
    FMedievalCityArchetypeParams ArchetypeParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="City")
    FDistrictParams DistrictParams;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
    bool bDebugDraw = true;

#if MEDIEVAL_HAS_PCG
    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalCityMacro")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval City Macro")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
