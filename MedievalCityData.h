#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "MedievalCityData.generated.h"

UENUM(BlueprintType)
enum class EMedievalTownReason : uint8
{
    RiverCrossing,
    Harbor,
    CastleTown,
    MonasteryTown,
    TradeJunction
};

UENUM(BlueprintType)
enum class EMedievalWardType : uint8
{
    MarketWard,
    CraftsWard,
    DocksWard,
    NobleWard,
    PoorWard,
    ChurchWard,
    ResidentialWard,
    Outskirts
};

UENUM(BlueprintType)
enum class EMedievalStreetType : uint8
{
    Primary,
    Secondary,
    Alley,
    Lane,
    WallRoad
};

UENUM(BlueprintType)
enum class EMedievalFootprintType : uint8
{
    Rowhouse,
    Shopfront,
    CourtyardHouse,
    HallHouse,
    Hut,
    Workshop,
    Barn,
    CornerBuilding
};

USTRUCT(BlueprintType)
struct FMedievalTerrainQueryParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Terrain")
    float SampleSpacing = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Terrain")
    float MaxAllowedSlopeDeg = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Terrain")
    float WaterLevelZ = 0.0f;
};

USTRUCT(BlueprintType)
struct FMedievalCitySeedParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Seed")
    int32 Seed = 1337;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Seed")
    FBox TownBounds = FBox(FVector(-12000, -12000, -500), FVector(12000, 12000, 3000));

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Seed")
    FMedievalTerrainQueryParams TerrainQuery;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Seed")
    float RiverInfluence = 0.6f;
};

USTRUCT(BlueprintType)
struct FMedievalCityArchetypeParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    EMedievalTownReason TownReason = EMedievalTownReason::RiverCrossing;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    bool bWalledTown = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype", meta=(ClampMin="1000", ClampMax="25000"))
    float WallRadius = 4800.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype", meta=(ClampMin="2", ClampMax="6"))
    int32 GateCount = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float GateBiasAlongMainRoads = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    FVector2D MarketSquareSizeRange = FVector2D(1600.f, 2800.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float MarketNearCenterBias = 0.85f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float ChurchWeight = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float KeepWeight = 0.85f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float GuildhallWeight = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float InnWeight = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Archetype")
    float DocksOrMillsWeight = 0.8f;
};

USTRUCT(BlueprintType)
struct FStreetGrowthParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float PrimaryRoadWidth = 700.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float SecondaryWidth = 440.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float AlleyWidth = 230.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float MaxCurvatureDegPer100m = 18.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float MaxSlopeDeg = 14.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float MinIntersectionSpacing = 650.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float DesireLineStrength = 1.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float SlopePenalty = 2.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float WaterPenalty = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Street")
    float ValleyPreference = 0.2f;
};

USTRUCT(BlueprintType)
struct FParcelParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    FVector2D BurgageFrontageRange = FVector2D(400.f, 1000.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    FVector2D BurgageDepthRange = FVector2D(3000.f, 12000.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float CourtyardParcelChanceRich = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float SetbackCore = 40.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float SetbackOutskirts = 190.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float BackLaneSpacing = 6200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Parcels")
    float BackLaneCreationProbability = 0.6f;
};

USTRUCT(BlueprintType)
struct FBuildingFootprintParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    FVector2D CoreFloorsRange = FVector2D(2.f, 4.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    FVector2D OuterFloorsRange = FVector2D(1.f, 2.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    float PartyWallChanceCore = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    float RearExtensionChance = 0.45f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    float YardReserveRatio = 0.28f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Building")
    float ServiceStripReserve = 180.f;
};

USTRUCT(BlueprintType)
struct FDistrictParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float MarketWeight = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float CraftsWeight = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float NobleWeight = 0.55f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float PoorWeight = 0.65f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float RiverAdjacencyBias = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float NoiseScale = 0.00045f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="District")
    float NoiseAmplitude = 0.35f;
};

UCLASS(BlueprintType)
class UMedievalCityGeneratorProfile : public UDataAsset
{
    GENERATED_BODY()
public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FMedievalCitySeedParams Seed;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FMedievalCityArchetypeParams Archetype;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FStreetGrowthParams Streets;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FParcelParams Parcels;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FBuildingFootprintParams Buildings;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="City")
    FDistrictParams Districts;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Organic street generation parameters
//  Exposed as a USTRUCT so they can be edited in Details and stored in profiles.
// ─────────────────────────────────────────────────────────────────────────────
USTRUCT(BlueprintType)
struct FOrganicStreetParams
{
    GENERATED_BODY()

    /** Minimum distance between intersections in the dense core (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float MinIntersectionSpacing = 2500.f;

    /** Probability of widening a 3-way junction into a small plaza */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float PlazaChanceAt3Way = 0.35f;

    /** Probability of adding a loop road in the dense core */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float LoopChanceCore = 0.10f;

    /** Probability of adding a loop road in the outskirts */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float LoopChanceOutskirts = 0.03f;

    /** Offset applied to near-market waypoints to prefer T-junctions over 4-ways (cm) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float TIntersectionOffset = 1100.f;

    /** A* grid cell size in cm. Smaller = tighter terrain following, slower generation */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float AStarCellSize = 500.f;

    /** Fraction of TownRadius treated as the "dense core" for spacing/density purposes */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float CoreRadiusFraction = 0.40f;

    /** ±fraction applied as width noise per edge segment */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float WidthNoiseFraction = 0.15f;
};
