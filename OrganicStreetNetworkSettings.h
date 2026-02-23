#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "OrganicTerrainRouting.h"
#include "OrganicStreetNetworkSettings.generated.h"

// NOTE: Do NOT include OrganicStreetGraph.h here.
// FOrganicStreetGraph is a plain C++ struct — UHT will error if it sees it
// inside a UCLASS header. Include OrganicStreetGraph.h only in the .cpp.

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  UOrganicStreetNetworkSettings
//  Data asset / PCG settings for the organic street network generator.
//  All UHT-visible properties use USTRUCT or primitive types only.
// ─────────────────────────────────────────────────────────────────────────────
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UOrganicStreetNetworkSettings : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    int32 Seed = 1337;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    FBox TownBounds = FBox(FVector(-12000, -12000, -500), FVector(12000, 12000, 3000));

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float IntersectionMinSpacingCore = 2500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float IntersectionMinSpacingOutskirts = 4000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    FVector2D PrimaryWidthRange = FVector2D(600.0f, 1000.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    FVector2D SecondaryWidthRange = FVector2D(400.0f, 700.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    FVector2D LaneWidthRange = FVector2D(200.0f, 400.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    FVector2D AlleyWidthRange = FVector2D(150.0f, 250.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float PlazaChanceAt3WayCore = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float LoopChanceSecondaryCore = 0.10f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float LoopChanceSecondaryOutskirts = 0.03f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    float WidthNoiseAmplitude = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Morphology")
    int32 RiverCrossings = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets|Routing")
    FOrganicTerrainCostParams CostParams;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const { return FName(TEXT("PCG_OrganicStreetNetwork")); }
    virtual FText GetDefaultNodeTitle() const { return FText::FromString(TEXT("Organic Street Network")); }
#endif
};

// ─────────────────────────────────────────────────────────────────────────────
//  OrganicStreetSettingsUtils
//  Free functions that use FOrganicStreetGraph (plain C++ type).
//  Declared here, defined in .cpp — kept OUTSIDE the UCLASS so UHT never
//  attempts to reflect FOrganicStreetGraph.
// ─────────────────────────────────────────────────────────────────────────────
struct FOrganicStreetGraph;   // forward-declare; include OrganicStreetGraph.h in callers

namespace OrganicStreetSettingsUtils
{
    /** Build a preview street graph from a settings object. */
    FOrganicStreetGraph BuildPreviewGraph(const UOrganicStreetNetworkSettings* Settings);
}
