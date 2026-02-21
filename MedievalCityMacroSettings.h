#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"
#include "MedievalCityMacroSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalCityMacroSettings : public UObject
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

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalCityMacro")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval City Macro")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
