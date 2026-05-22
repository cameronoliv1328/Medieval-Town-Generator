#pragma once

#include "CoreMinimal.h"
#include "MedievalPCGToggle.h"
#include "MedievalCityData.h"

#ifndef MEDIEVALTOWNGENERATOR_API
#define MEDIEVALTOWNGENERATOR_API
#endif

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGSettings.h"
#endif

#include "MedievalBuildingSpawnerSettings.generated.h"

/**
 * PCG node: Medieval Building Spawner
 *
 * Input:  none — reads LayoutParcels directly from AMedievalTownGenerator.
 *
 * Output pins (wire each to its own PCGStaticMeshSpawner with the appropriate mesh list):
 *
 *   Landmarks   — Church, Keep; grand unique meshes.
 *
 *   Residential — SmallCottage, TownHouse, GuildHall, TavernInn, Bakery;
 *                 mid-density street-front buildings.
 *
 *   Service     — Stable, Warehouse, Blacksmith; working / yard buildings.
 *
 * Every output point:
 *   Location  = Parcel.WorldCenter (terrain-snapped)
 *   Rotation  = FRotator(0, Parcel.Yaw, 0)  (+X aligns with street frontage)
 *   Scale.X   = FrontageWidth / BuildingUnitFrontage
 *   Scale.Y   = LotDepth      / BuildingUnitDepth
 *   Scale.Z   = NumFloors * FloorHeight / BuildingUnitHeight
 *   Density   = WealthScore  [0=poorest, 1=richest] → material tier selection
 *
 * All 20 parcel attributes are written as typed PCG metadata so PCGStaticMeshSpawner
 * can use attribute-based mesh variant selection (BuildingStyle, RoofType, Ward…).
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalBuildingSpawnerSettings
#if MEDIEVAL_ENABLE_PCG_NODES
    : public UPCGSettings
#else
    : public UObject
#endif
{
    GENERATED_BODY()

public:
    /** Set this when the PCG component is not on the AMedievalTownGenerator actor. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner")
    TSoftObjectPtr<AActor> TownGeneratorActor;

    /** Mesh footprint width in cm (the X extent of one unit building mesh). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BuildingUnitFrontage = 600.f;

    /** Mesh footprint depth in cm (the Y extent of one unit building mesh). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BuildingUnitDepth = 2000.f;

    /** Height per floor in cm.  Scale.Z = NumFloors * FloorHeight / BuildingUnitHeight. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float FloorHeight = 300.f;

    /** One-storey mesh height in cm — a single-floor parcel gets Scale.Z = 1.0. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BuildingUnitHeight = 300.f;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override
        { return FName(TEXT("PCG_MedievalBuildingSpawner")); }
    virtual FText GetDefaultNodeTitle() const override
        { return FText::FromString(TEXT("Medieval Building Spawner")); }
    virtual EPCGSettingsType GetType() const override
        { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties()  const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
