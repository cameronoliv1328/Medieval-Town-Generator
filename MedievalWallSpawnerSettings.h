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

#include "MedievalWallSpawnerSettings.generated.h"

/**
 * PCG node: Medieval Wall Spawner
 *
 * Input:  none — reads wall layout directly from AMedievalTownGenerator.
 *
 * Output pins:
 *   WallSections  — one point per terrain sub-segment within each wall segment.
 *                   Transform scale encodes (subLen / WallSectionMeshUnitLength,
 *                   thickness / WallMeshUnitThickness, height / WallMeshUnitHeight).
 *
 *   Battlements   — one point per terrain sub-segment for battlement-enabled segments,
 *                   elevated to the wall top (Z += WallHeight).
 *                   Scale encodes (subLen / BattlementMeshUnitLength,
 *                   thickness / WallMeshUnitThickness, 1).
 *
 *   CornerTowers  — one point per CornerTower node.
 *                   Scale encodes (radius / CornerTowerMeshUnitRadius,
 *                   radius / CornerTowerMeshUnitRadius,
 *                   height / CornerTowerMeshUnitHeight).
 *
 *   GateTowers    — two points per GateTower node (left and right flanking towers),
 *                   offset along the wall tangent by TowerRadius * 1.8.
 *                   Scale encodes (radius / GateTowerMeshUnitRadius,
 *                   radius / GateTowerMeshUnitRadius,
 *                   height / GateTowerMeshUnitHeight).
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalWallSpawnerSettings
#if MEDIEVAL_ENABLE_PCG_NODES
    : public UPCGSettings
#else
    : public UObject
#endif
{
    GENERATED_BODY()

public:

    /** The AMedievalTownGenerator actor to read wall layout from. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner")
    TSoftObjectPtr<AActor> TownGeneratorActor;

    // -----------------------------------------------------------------------
    // Mesh unit dimensions — match your imported mesh tile dimensions exactly.
    // -----------------------------------------------------------------------

    /** Length of one wall-section mesh tile (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float WallSectionMeshUnitLength = 400.f;

    /** Height of one wall-section mesh tile (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float WallMeshUnitHeight = 900.f;

    /** Wall thickness of one wall-section mesh tile (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float WallMeshUnitThickness = 120.f;

    /** Base radius of the corner tower mesh (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float CornerTowerMeshUnitRadius = 220.f;

    /** Full height of the corner tower mesh (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float CornerTowerMeshUnitHeight = 1260.f;

    /** Base radius of the gate tower mesh (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float GateTowerMeshUnitRadius = 308.f;

    /** Full height of the gate tower mesh (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float GateTowerMeshUnitHeight = 1449.f;

    /** Length of one crenellation (battlement) strip tile (cm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BattlementMeshUnitLength = 400.f;

#if MEDIEVAL_ENABLE_PCG_NODES
    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalWallSpawner")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Wall Spawner")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:
    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;
#endif
};
