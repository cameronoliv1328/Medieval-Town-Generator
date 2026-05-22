#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"
#include "MedievalStreetLayoutSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

/**
 * PCG layout reader node for town street and bridge data.
 *
 * Consumes the street graph produced by AMedievalTownGenerator::BuildStreetBoundary()
 * and emits two output point streams:
 *
 *   StreetNodes — one PCG point per intersection / gate / market node.
 *                 Attributes: IntersectionType(int32), bIsMarket, bIsGate,
 *                 bIsBridgeAbutment (all float 0/1).
 *
 *   StreetEdges — one PCG point per WorldPolyline vertex within each road edge.
 *                 Attributes: EdgeIndex(int32), StreetType(int32), Surface(int32),
 *                 Width, GradeDeg, bIsBridge, bIsQuayStreet (float 0/1),
 *                 VertexIndex(int32), VertexCount(int32).
 *
 * PCG steps StreetEdge points in VertexIndex order to place road surface meshes
 * and bridge deck segments.
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalStreetLayoutSettings : public UObject
{
    GENERATED_BODY()
public:
    /** Actor reference used when the PCG component is not on the town generator. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Layout")
    TSoftObjectPtr<AActor> TownGeneratorActor;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override
        { return FName(TEXT("PCG_MedievalStreetLayout")); }
    virtual FText GetDefaultNodeTitle() const override
        { return FText::FromString(TEXT("Medieval Street Layout")); }
    virtual EPCGSettingsType GetType() const override
        { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties()  const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
