#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"
#include "MedievalWallLayoutSettings.generated.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

/**
 * PCG layout reader node for town wall data.
 *
 * Place this node in a PCG graph to consume the wall and gate geometry produced
 * by AMedievalTownGenerator::BuildWallBoundary().  Two output pins are emitted:
 *
 *   WallNodes    — one PCG point per tower or gate node on the wall perimeter.
 *                  Attributes: NodeType(int32), TowerRadius, TowerHeight,
 *                  GateWidth, bIsMainGate, OutwardFacingX/Y, SegmentIndexCW/CCW.
 *
 *   WallSegments — one PCG point per terrain-following sample within each wall
 *                  segment.  Attributes: SegmentIndex(int32), WallHeight,
 *                  WallThickness, bHasBattlements, OutwardNormalX/Y, Length.
 *
 * Actor resolution order:
 *   1. Context->SourceComponent->GetOwner() cast to AMedievalTownGenerator.
 *   2. TownGeneratorActor property (set manually in PCG graph editor if the PCG
 *      component lives on a different actor).
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalWallLayoutSettings : public UObject
{
    GENERATED_BODY()
public:
    /** Actor reference used when the PCG component is not on the town generator. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Layout")
    TSoftObjectPtr<AActor> TownGeneratorActor;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override
        { return FName(TEXT("PCG_MedievalWallLayout")); }
    virtual FText GetDefaultNodeTitle() const override
        { return FText::FromString(TEXT("Medieval Wall Layout")); }
    virtual EPCGSettingsType GetType() const override
        { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties()  const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
