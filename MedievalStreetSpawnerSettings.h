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

#include "MedievalStreetSpawnerSettings.generated.h"

/**
 * PCG node: Medieval Street Spawner
 *
 * Reads LayoutStreetEdges + LayoutStreetNodes from AMedievalTownGenerator and emits
 * four output point streams:
 *
 *   RoadTiles      — one point per sub-segment of non-bridge edges.
 *                    Transform scale = (subLen/tileLen, width/tileWidth, 1).
 *                    Surface int32 attribute drives mesh selection.
 *
 *   BridgeDecks    — one point per bridge sub-segment.
 *                    Transform scale = (subLen/deckLen, width/deckWidth, 1).
 *
 *   BridgeRails    — TWO points per bridge sub-segment (left + right sides),
 *                    each offset laterally by Width * 0.5 from the centreline.
 *                    Transform scale = (subLen/railLen, 1, 1).
 *
 *   Intersections  — one point per StreetNode.
 *                    Density encodes intersection type:
 *                      0.0 = Terminus, 0.25 = TJunction, 0.5 = FourWay,
 *                      0.75 = Gate, 1.0 = Market.
 */
UCLASS(BlueprintType, ClassGroup=(Procedural))
class MEDIEVALTOWNGENERATOR_API UMedievalStreetSpawnerSettings
#if MEDIEVAL_ENABLE_PCG_NODES
    : public UPCGSettings
#else
    : public UObject
#endif
{
    GENERATED_BODY()

public:

    // -------------------------------------------------------------------------
    // Spawner target
    // -------------------------------------------------------------------------

    /** The actor that owns AMedievalTownGenerator. Used when this node is not
     *  placed on the generator actor itself. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner")
    TSoftObjectPtr<AActor> TownGeneratorActor;

    // -------------------------------------------------------------------------
    // Mesh unit dimensions — must match the actual static-mesh extents
    // -------------------------------------------------------------------------

    /** Length (X) of one road-tile mesh in centimetres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float RoadTileMeshUnitLength = 400.f;

    /** Width (Y) of one road-tile mesh in centimetres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float RoadTileMeshUnitWidth = 420.f;

    /** Length (X) of one bridge-deck mesh in centimetres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BridgeDeckMeshUnitLength = 400.f;

    /** Width (Y) of one bridge-deck mesh in centimetres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BridgeDeckMeshUnitWidth = 420.f;

    /** Length (X) of one bridge-rail mesh in centimetres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spawner|MeshUnits")
    float BridgeRailMeshUnitLength = 400.f;

#if MEDIEVAL_ENABLE_PCG_NODES

    // -------------------------------------------------------------------------
    // IPCGSettings interface
    // -------------------------------------------------------------------------

    virtual FName GetDefaultNodeName() const override { return FName(TEXT("PCG_MedievalStreetSpawner")); }
    virtual FText GetDefaultNodeTitle() const override { return FText::FromString(TEXT("Medieval Street Spawner")); }
    virtual EPCGSettingsType GetType() const override { return EPCGSettingsType::Spatial; }

protected:

    virtual TArray<FPCGPinProperties> InputPinProperties() const override;
    virtual TArray<FPCGPinProperties> OutputPinProperties() const override;
    virtual FPCGElementPtr CreateElement() const override;

#endif // MEDIEVAL_ENABLE_PCG_NODES
};
