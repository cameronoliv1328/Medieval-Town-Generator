// =============================================================================
// MedievalTownGenerator.h  --  VERSION 18
// =============================================================================
// Procedural Medieval Town Generator for Unreal Engine 5.5+
//
// NEW IN v12:
//   [1]  District Zoning        -- 4 districts: InnerWard, Merchant, Residential, Transition
//   [2]  Voronoi Road Network   -- organic gate-to-market roads via Delaunay/MST
//   [3]  Modular Building       -- Foundation + tiled floors + multi-roof variants
//   [4]  Shape Grammar Walls    -- grammar-string driven battlement / tower / gate placement
//   [5]  Flat Area Detector     -- slope check rejects lots on uneven ground
//   [6]  Spline-Height Roads    -- roads conform to terrain elevation
//   [7]  Noise Foliage Density  -- Perlin density map for forest ring
//   [8]  Water Body River       -- WaterBody-compatible river path + mesh
//   [9]  Save / Load Layout     -- lock a seed and re-spawn without regeneration
//   [10] Additional Improvements -- L-shape buildings, chimneys, ground props
// =============================================================================

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SplineComponent.h"
#include "MedievalCityData.h"
#include "MedievalTownGeneratorRiver.h"
#include "MedievalTownGenerator.generated.h"

// -----------------------------------------------------------------------------
//  ENUMERATIONS
// -----------------------------------------------------------------------------

/** Town district type -- controls which buildings appear and at what density */
UENUM(BlueprintType)
enum class EDistrictType : uint8
{
    InnerWard        UMETA(DisplayName = "Inner Ward (Castle/Admin)"),
    MerchantQuarter  UMETA(DisplayName = "Merchant Quarter"),
    CraftQuarter     UMETA(DisplayName = "Craft Quarter (River-side)"),
    GateWard         UMETA(DisplayName = "Gate Ward (Commercial)"),
    OuterResidential UMETA(DisplayName = "Outer Residential"),
    Slums            UMETA(DisplayName = "Slums / Poor Quarter"),
    TransitionZone   UMETA(DisplayName = "Transition (Stables/Storage)"),
    Plaza            UMETA(DisplayName = "Market Plaza")
};

/** Building style -- drives floor count, roof type, and proportions */
UENUM(BlueprintType)
enum class EBuildingStyle : uint8
{
    SmallCottage  UMETA(DisplayName = "Small Cottage"),
    TownHouse     UMETA(DisplayName = "Town House"),
    GuildHall     UMETA(DisplayName = "Guild Hall"),
    TavernInn     UMETA(DisplayName = "Tavern / Inn"),
    Church        UMETA(DisplayName = "Church / Chapel"),
    Keep          UMETA(DisplayName = "Keep / Donjon"),
    Stable        UMETA(DisplayName = "Stable"),
    Warehouse     UMETA(DisplayName = "Warehouse"),
    Blacksmith    UMETA(DisplayName = "Blacksmith"),
    Bakery        UMETA(DisplayName = "Bakery")
};

/** Roof geometry variant for modular building tops */
UENUM(BlueprintType)
enum class ERoofType : uint8
{
    Pitched     UMETA(DisplayName = "Gabled / Pitched"),
    Hipped      UMETA(DisplayName = "Hipped"),
    Gambrel     UMETA(DisplayName = "Gambrel (Barn)"),
    FlatParapet UMETA(DisplayName = "Flat with Parapet"),
    Conical     UMETA(DisplayName = "Conical (Round Tower)"),
    Pyramidal   UMETA(DisplayName = "Pyramidal (Square Tower)"),
    Thatched    UMETA(DisplayName = "Thatched (Low Pitch)")
};

/** Road tier -- controls width and material tint */
UENUM(BlueprintType)
enum class EStreetTier : uint8
{
    Primary    UMETA(DisplayName = "Primary Road (Gate to Market)"),
    Secondary  UMETA(DisplayName = "Secondary Road (Ring)"),
    Tertiary   UMETA(DisplayName = "Tertiary Road (Alley)"),
    RiverPath  UMETA(DisplayName = "River-side Path")
};


UENUM(BlueprintType)
enum class ERoadSurfaceType : uint8
{
    MarketStone UMETA(DisplayName = "Market Stone"),
    MainDirtCobble UMETA(DisplayName = "Main Dirt/Cobble"),
    AlleyDirt UMETA(DisplayName = "Alley Dirt"),
    BridgeDeck UMETA(DisplayName = "Bridge Deck")
};

/** Shape grammar wall module types */
UENUM(BlueprintType)
enum class EWallModule : uint8
{
    WallSection   UMETA(DisplayName = "Standard Wall Section"),
    CornerTower   UMETA(DisplayName = "Corner Round Tower"),
    GateTower     UMETA(DisplayName = "Gate Twin Towers"),
    Parapet       UMETA(DisplayName = "Crenellated Parapet"),
    Buttress      UMETA(DisplayName = "Flying Buttress")
};

// -----------------------------------------------------------------------------
//  CORE DATA STRUCTS
// -----------------------------------------------------------------------------

/** A single node in the Voronoi/Delaunay road network graph */
USTRUCT(BlueprintType)
struct FRoadNode
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector2D Pos;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsGate = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsMarket = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsBridgeNode = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    bool bIsLandmark = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float Importance = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<int32> ConnectedEdges;

    int32 Index = -1;

    FRoadNode() {}
    FRoadNode(FVector2D InPos, int32 InIdx) : Pos(InPos), Index(InIdx) {}
};

/** A directed edge between two road nodes; stores smoothed world-space spline pts */
USTRUCT(BlueprintType)
struct FRoadEdge
{
    GENERATED_BODY()

    int32 NodeA = -1;
    int32 NodeB = -1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    EStreetTier Tier = EStreetTier::Secondary;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float Width = 280.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float TargetSpeed = 2.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float CurvatureCost = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float Importance = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    TArray<FVector2D> PolylinePoints;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    ERoadSurfaceType SurfaceType = ERoadSurfaceType::MainDirtCobble;

    // World-space points (terrain height already baked in)
    TArray<FVector> WorldPoints;

    UPROPERTY(Transient)
    TObjectPtr<USplineComponent> GeneratedSplineRef = nullptr;

    bool bIsGenerated = false;
    bool bIsBridge = false;      // True if this edge crosses the river
};

/** Describes a single placed building lot */
USTRUCT(BlueprintType)
struct FBuildingLot
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector Center = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    FVector2D Footprint = FVector2D(500.f, 400.f);   // Width x Depth

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    float Yaw = 0.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    EDistrictType District = EDistrictType::OuterResidential;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    EBuildingStyle Style = EBuildingStyle::TownHouse;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    int32 NumFloors = 1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
    ERoofType Roof = ERoofType::Pitched;

    /** Half-diagonal used for collision checks */
    float CollisionRadius = 350.f;

    /** Whether building has an L-shaped or T-shaped extension wing */
    bool bHasWing = false;
    FVector2D WingFootprint = FVector2D(250.f, 200.f);
    float WingYawOffset = 90.f;

    bool bIsPlaced = false;
};

/** District definition -- each district has unique placement rules */
struct FDistrictDef
{
    FString Name;
    EDistrictType Type;
    float InnerRadiusFraction = 0.f;    // Fraction of TownRadius for inner boundary
    float OuterRadiusFraction = 1.f;    // Fraction of TownRadius for outer boundary
    float Density = 0.6f;               // Probability to accept a candidate lot
    float MinScale = 0.8f;
    float MaxScale = 1.3f;
    TArray<EBuildingStyle> StylePool;   // Weighted pool -- repeated = higher chance

    // -- Per-district placement rules (multipliers of global values) --
    float SpacingMult = 1.0f;           // Multiplier for MinBuildingSpacing
    float SetbackMult = 1.0f;           // Multiplier for RoadBuildingSetback
    float RotationJitter = 8.f;         // Degrees of random rotation jitter
    float LShapeChance = 0.25f;         // Probability of L-shape wing
    float CursorStartOffset = 250.f;    // How far from intersection to start placing
    float MinEdgeLen = 600.f;           // Skip road edges shorter than this

    // -- Angular sector bounds (set by BuildDistrictDefs) --
    float MinAngleDeg = -180.f;         // Angular sector start (-180 to 180)
    float MaxAngleDeg = 180.f;          // Angular sector end
    bool bUsesAngle = false;            // Whether this district is an angular wedge
};

/** Resolved terrain sample (height + normal) */
struct FTerrainSample
{
    float Height = 0.f;
    FVector Normal = FVector::UpVector;
    bool bIsFlat = true;   // True if slope < threshold
};

/** River path (world-space 2D waypoints; Z comes from terrain) */
struct FRiverPath
{
    TArray<FVector2D> Waypoints;
    float Width = 550.f;
    float ExclusionRadius = 800.f;    // Buildings must stay this far away
};

/** Result of saving the current generated layout */
USTRUCT(BlueprintType)
struct FSavedTransform
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Location = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRotator Rotation = FRotator::ZeroRotator;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 StyleIndex = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 DistrictIndex = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 NumFloors = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 RoofTypeIndex = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector2D Footprint = FVector2D(500.f, 400.f);
};

USTRUCT(BlueprintType)
struct FSavedTownLayout
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 LockedSeed = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FSavedTransform> Buildings;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FVector> RiverPoints;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FVector> WallPoints;

    bool bIsValid = false;
};

// -----------------------------------------------------------------------------
//  MAIN CLASS
// -----------------------------------------------------------------------------

UCLASS()
class AMedievalTownGenerator : public AActor
{
    GENERATED_BODY()

public:
    AMedievalTownGenerator();

protected:
    virtual void BeginPlay() override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

public:
    // ===== GENERAL =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | General",
        meta = (UIMin = "0", UIMax = "99999"))
    int32 RandomSeed = 12345;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | General",
        meta = (UIMin = "6000", UIMax = "60000", ClampMin = "6000", ClampMax = "60000"))
    float TownRadius = 18000.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | General")
    bool bGenerateOnBeginPlay = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | General")
    bool bAutoRegenerateInEditor = false;

    // ===== TERRAIN =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "0", UIMax = "3000"))
    float TerrainAmplitude = 800.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "0.00005", UIMax = "0.001"))
    float TerrainFrequency = 0.00015f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "1", UIMax = "6"))
    int32 TerrainOctaves = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "10", UIMax = "200"))
    int32 TerrainResolution = 60;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "0", UIMax = "30"))
    float MaxSlopeForBuilding = 25.f;    // Degrees -- flat area detector threshold

    // How aggressively terrain inside the walls is flattened (0 = no flatten, 1 = pancake flat)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "0.0", UIMax = "1.0"))
    float TownFlattenStrength = 0.85f;

    // Width of the flatten-to-wild transition zone (fraction of TownRadius outside the wall)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Terrain",
        meta = (UIMin = "0.05", UIMax = "0.5"))
    float TownFlattenTransition = 0.25f;

    // ===== BUILDINGS =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings",
        meta = (UIMin = "20", UIMax = "500"))
    int32 TargetBuildingCount = 180;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings",
        meta = (UIMin = "100", UIMax = "800"))
    float MinBuildingSpacing = 200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings",
        meta = (UIMin = "0", UIMax = "20"))
    float BuildingRotationJitter = 8.f;  // Degrees of random rotation around 90-deg snapping

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings",
        meta = (UIMin = "0.0", UIMax = "1.0"))
    float LShapeProbability = 0.25f;     // Chance a larger building has a wing

    // Per-style base size (W x D in cm)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D SmallCottageSize = FVector2D(400.f, 350.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D TownHouseSize = FVector2D(550.f, 450.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D GuildHallSize = FVector2D(900.f, 700.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D TavernSize = FVector2D(750.f, 600.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D ChurchSize = FVector2D(500.f, 950.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D KeepSize = FVector2D(950.f, 950.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D StableSize = FVector2D(900.f, 450.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Sizes")
    FVector2D WarehouseSize = FVector2D(800.f, 550.f);

    // Per-floor height
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Heights",
        meta = (UIMin = "200", UIMax = "600"))
    float FloorHeight = 350.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Heights",
        meta = (UIMin = "50", UIMax = "200"))
    float FoundationHeight = 80.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Heights",
        meta = (UIMin = "0", UIMax = "60"))
    float RoofPitchAngle = 42.f;        // Degrees from horizontal

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Buildings | Heights",
        meta = (UIMin = "20", UIMax = "120"))
    float RoofOverhang = 40.f;

    // ===== WALLS =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls")
    bool bGenerateWalls = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "6", UIMax = "32"))
    int32 WallSegments = 18;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "200", UIMax = "2500"))
    float WallHeight = 900.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "50", UIMax = "400"))
    float WallThickness = 120.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "4", UIMax = "8"))
    int32 NumGates = 4;

    /** Grammar string for wall module sequence.
     *  Tokens: W=WallSection  T=CornerTower  G=GateTower  P=Parapet  B=Buttress
     *  Example: "T W W W G W W W T W W W W W T W W W G W W W T W W W W W W" */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls")
    FString WallGrammarString = TEXT("T W W G W W T W W W G W W T W W G W W T W W W G W W");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "0.7", UIMax = "1.5"))
    float WallTowerRadiusFactor = 1.15f;   // Towers placed slightly outside wall perimeter

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "100", UIMax = "600"))
    float WallTowerRadius = 220.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "0.3", UIMax = "1.0"))
    float WallTowerHeightFactor = 1.4f;   // Towers are taller than wall

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "50", UIMax = "200"))
    float BattlementWidth = 80.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Walls",
        meta = (UIMin = "60", UIMax = "250"))
    float BattlementHeight = 120.f;

    // ===== ROADS =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "8", UIMax = "60"))
    int32 RoadNetworkNodes = 24;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "200", UIMax = "700"))
    float PrimaryRoadWidth = 420.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "120", UIMax = "400"))
    float SecondaryRoadWidth = 280.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "60", UIMax = "200"))
    float TertiaryRoadWidth = 160.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "2", UIMax = "12"))
    int32 RoadSplineSubdivisions = 5;    // Control points between nodes for terrain-following

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0", UIMax = "50"))
    float RoadOrganicWaver = 18.f;       // Randomness added to spline points (cm)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads")
    bool bUseOrganicStreetGrowth = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "1000", UIMax = "12000"))
    float CoreRadius = 4200.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "100", UIMax = "1000"))
    float IntersectionMinSpacing = 250.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0", UIMax = "1"))
    float SecondaryLoopChance = 0.10f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0", UIMax = "1"))
    float PlazaChanceAtConvergence = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads")
    bool bDebugRoadGraph = false;

    /** Draw organic street graph in editor: yellow=primary, blue=secondary, green=lane */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads | Organic")
    bool bDebugDrawStreets = false;

    /** Number of secondary street attractor samples per generation pass (default 60) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads | Organic",
        meta = (UIMin = "20", UIMax = "150"))
    int32 SecondaryAttractorCount = 60;

    /** Terrain routing weights used by the organic A* pathfinder */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads | Organic")
    FStreetGrowthParams StreetGrowthData;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads")
    bool bDebugRoadWaterZones = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads", meta = (UIMin = "0.05", UIMax = "0.2"))
    float MaxGradePrimary = 0.10f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads", meta = (UIMin = "0.06", UIMax = "0.25"))
    float MaxGradeSecondary = 0.12f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads", meta = (UIMin = "0.10", UIMax = "0.20"))
    float WidthNoiseAmplitude = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "20", UIMax = "300"))
    float RoadBuildingSetback = 60.f;    // Gap between road edge and building front

    // Ring road radii (fraction of TownRadius)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0.2", UIMax = "0.5"))
    float InnerRingRadius = 0.33f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0.45", UIMax = "0.8"))
    float OuterRingRadius = 0.63f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "1", UIMax = "4"))
    int32 RingArcSubdivisions = 2;          // Intermediate nodes per ring arc (smoothness)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Roads",
        meta = (UIMin = "0.0", UIMax = "1.0"))
    float MidBlockConnectorChance = 0.65f;   // Chance to add connector streets between rings

    // ===== RIVER =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bGenerateRiver = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "300", UIMax = "1500"))
    float RiverWidth = 550.f;

    // How deep the river bed is below the local bank height at the thalweg (centerline)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "60", UIMax = "500"))
    float RiverMaxDepth = 180.f;

    // Shallow depth at the water edge for a more realistic sloped bed profile
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "20", UIMax = "250"))
    float RiverEdgeDepth = 55.f;

    // Distance from the water edge to blend back up to natural terrain
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "40", UIMax = "800"))
    float RiverBankFalloffWidth = 120.f;

    // Water plane is kept slightly below bank height so it sits "inside" the channel
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "80"))
    float RiverWaterSurfaceOffset = 10.f;

    // Slightly extend river surface beyond nominal width to hide edge seams at glancing angles
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "120"))
    float RiverSurfaceEdgeOverlap = 25.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "500", UIMax = "2000"))
    float RiverExclusionRadius = 450.f;

    // Extra no-build margin beyond the river exclusion radius
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "800"))
    float RiverBuildingBuffer = 140.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "3", UIMax = "10"))
    int32 RiverWaypoints = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "90"))
    float RiverEntryAngleDeg = 30.f;     // Direction the river enters from
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "4", UIMax = "24"))
    int32 RiverSamplesPerSegment = 10;

    // Meander octave used for width and bank variation along river length
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.2", UIMax = "3.0"))
    float RiverVariationFrequency = 1.15f;

    // Relative width variation (+/- range as fraction of RiverWidth)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.0", UIMax = "0.45"))
    float RiverWidthVariation = 0.22f;

    // Per-segment downhill drop in cm used to keep believable flow direction
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "1", UIMax = "120"))
    float RiverDownhillPerSegment = 18.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.0", UIMax = "0.35"))
    float RiverEdgeNoise = 0.08f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "100", UIMax = "4000"))
    float RiverUVTilingDistance = 900.f;

    // River bed mesh is rendered narrower than surface for natural submerged banks
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.45", UIMax = "1.0"))
    float RiverBedWidthFactor = 0.82f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "40", UIMax = "500"))
    float RiverBedMinBelowSurface = 70.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.1", UIMax = "4.0"))
    float RiverFlowSpeedBase = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.0", UIMax = "1.5"))
    float RiverFlowSpeedVariation = 0.35f;
    // Extra gorge carving outside walls to blend river into surrounding mountains.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "1200"))
    float RiverGorgeDepth = 320.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "100", UIMax = "2000"))
    float RiverGorgeHalfWidth = 650.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "40", UIMax = "800"))
    float RiverGorgeRimWidth = 220.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "50", UIMax = "2000"))
    float RiverFloodplainWidth = 420.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0.0", UIMax = "1.0"))
    float RiverFloodplainFlattenStrength = 0.55f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "240"))
    float RiverFloodplainDrop = 28.f;


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "20", UIMax = "600"))
    float RiverShoreBlendWidth = 140.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bAdaptiveRiverSampling = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "8"))
    int32 RiverCurvatureSubdivisionBoost = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "4"))
    int32 RiverPlanarSmoothPasses = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bGenerateRiverFoam = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "6", UIMax = "260"))
    float RiverFoamWidth = 36.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "0", UIMax = "20"))
    float RiverFoamHeightOffset = 2.5f;

    // V19 improved river/terrain integration controls
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "30", UIMax = "500"))
    float RiverAdaptiveTerrainCellSize = 100.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "2", UIMax = "8"))
    int32 RiverShoreBlendRings = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bUseImprovedRiverMeshes = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "1", UIMax = "30"))
    float RiverTerrainSubmersionBias = 5.f;


    // ===== FOLIAGE =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "0", UIMax = "2000"))
    int32 ForestTreeCount = 400;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "1.0", UIMax = "3.0"))
    float ForestRingInnerFraction = 1.02f;   // Inner edge of forest (fraction of TownRadius)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "1.1", UIMax = "5.0"))
    float ForestRingOuterFraction = 2.5f;    // Outer edge of forest

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "0.0", UIMax = "1.0"))
    float ForestDensityFrequency = 0.0003f;  // Perlin frequency for tree clustering

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "200", UIMax = "1200"))
    float TreeHeightMin = 400.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "400", UIMax = "2500"))
    float TreeHeightMax = 900.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "80", UIMax = "400"))
    float TreeCrownRadiusMin = 120.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Foliage",
        meta = (UIMin = "150", UIMax = "700"))
    float TreeCrownRadiusMax = 280.f;

    // ===== MOUNTAINS =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Mountains",
        meta = (UIMin = "3", UIMax = "12"))
    int32 MountainCount = 6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Mountains")
    FVector MountainScaleMin = FVector(1200.f, 800.f, 1400.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Mountains")
    FVector MountainScaleMax = FVector(3500.f, 2000.f, 4200.f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Mountains",
        meta = (UIMin = "2.0", UIMax = "6.0"))
    float MountainRingFraction = 3.5f;       // How far out (multiples of TownRadius)

    // ===== MATERIALS =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* WallMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* RoofMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* StoneMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* GroundMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* RoadMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* TreeMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* WaterMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Materials")
    UMaterialInterface* RiverFoamMaterial;

    // ===== SAVE / LOAD =====

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Save Layout")
    bool bUseSavedLayout = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | Save Layout")
    FSavedTownLayout SavedLayout;

    // ===== PUBLIC FUNCTIONS =====

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Town")
    void GenerateTown();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Town")
    void ClearTown();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Town")
    void RegenerateWithNewSeed(int32 NewSeed);

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Town | Save Layout")
    void SaveCurrentLayout();

    UFUNCTION(BlueprintCallable, CallInEditor, Category = "Town | Save Layout")
    void LoadSavedLayout();

    UFUNCTION(BlueprintCallable, Category = "Town")
    TArray<FVector> GetRiverWorldPath() const { return CachedRiverWorldPath; }

    // River helper accessors used by MTGRiver utility namespace.
    UFUNCTION(BlueprintPure, Category = "Town|River")
    float QueryTerrainHeight(float X, float Y) const { return GetTerrainHeight(X, Y); }

    UFUNCTION(BlueprintPure, Category = "Town|River")
    float QueryTerrainHeightNoRiver(float X, float Y) const { return GetTerrainHeightNoRiver(X, Y); }

    UFUNCTION(BlueprintPure, Category = "Town|River")
    float QueryRiverDepthAt(FVector2D Pos) const { return GetRiverDepthAt(Pos); }

    UFUNCTION(BlueprintPure, Category = "Town|River")
    float QueryRiverHalfWidthAt(float Alpha) const { return GetRiverHalfWidthAt(Alpha); }

    UFUNCTION(BlueprintPure, Category = "Town|River")
    float QueryRiverFlowSpeedAt(float Alpha) const { return GetRiverFlowSpeedAt(Alpha); }

private:
    // --- Internal runtime data ------------------------------------------------
    FRandomStream Rand;

    TArray<UProceduralMeshComponent*> GeneratedMeshes;
    TArray<FBuildingLot>              PlacedLots;
    TArray<FRoadNode>                 RoadNodes;
    TArray<FRoadEdge>                 RoadEdges;
    FRiverPath                        River;
    TArray<FVector>                   WallPerimeter;   // World-space wall ring
    TArray<FVector>                   GatePositions;
    TArray<FVector>                   CachedRiverWorldPath;
    TArray<FVector2D>                 CachedRiverPlanarPath;

    // --- Terrain cache -------------------------------------------------------
    TArray<float> TerrainHeightCache;
    int32         TerrainCacheRes = 0;

    // --- Main pipeline -------------------------------------------------------
    void Phase1_GenerateRiverWaypoints();
    void Phase2_SetupTerrain();
    void Phase3_BuildRiverWorldPath();
    void Phase4_BuildWalls();
    void Phase5_BuildRoadNetwork();
    void Phase6_PlaceBuildings();
    void Phase7_SpawnMeshes();
    void Phase8_PlaceForest();
    void Phase9_BuildMountains();

    // --- Terrain -------------------------------------------------------------
    float SampleNoise(float X, float Y, int32 Octaves, float Freq, float Amp,
                      float Persistence, float Lacunarity) const;
    float GetTerrainHeight(float X, float Y) const;
    float GetTerrainHeightNoRiver(float X, float Y) const; // Bank height without river carve
    FTerrainSample SampleTerrain(float X, float Y) const;
    bool IsTerrainFlat(FVector2D Center, float HalfW, float HalfD) const;
    void BuildTerrainCache();

    // --- River ---------------------------------------------------------------
    void  GenerateRiverWaypoints();
    void  BuildRiverPlanarPath();
    void  BuildRiverWorldPath();
    float GetRiverDepthAt(FVector2D Pos) const;
    float GetRiverHalfWidthAt(float RiverAlpha01) const;
    float GetRiverFlowSpeedAt(float RiverAlpha01) const;
    bool  SampleRiverClosestPoint(FVector2D Pos, float& OutDist, float& OutHalfW,
                                  float* OutAlpha = nullptr) const;
    bool  IsNearRiver(FVector2D Pos, float ExtraRadius = 0.f) const;
    float DistToRiverCenter(FVector2D Pos) const;  // Min dist to river centerline
    bool  SegmentCrossesRiver(FVector2D A, FVector2D B) const;
    void  SpawnBridgeMesh(const FRoadEdge& Edge);
    void  SpawnImprovedRiverMeshes();
    void  GenerateAdaptiveTerrainMesh();

    // --- Road network --------------------------------------------------------
    void BuildRoadNetwork();
    void BuildOrganicRoadNetwork();  // replaces BuildRadiocentricRoads()
    void ApplyOrganicGraphToTownGenerator(const struct FOrganicStreetGraph& Graph); // private member — needs RoadNodes/RoadEdges
    void ElevateRoadSplines();
    float RoadWidth(EStreetTier Tier) const;

    // --- Wall shape grammar --------------------------------------------------
    void  GenerateWalls();
    void  ParseGrammar(const FString& Grammar, TArray<EWallModule>& OutModules);
    void  SpawnWallSection(FVector Start, FVector End, float Height, float Thickness,
                           bool bBattlements);
    void  SpawnCornerTower(FVector Center, float Radius, float Height);
    void  SpawnGateTower(FVector Center, FVector Direction, float TowerRadius, float Height);
    FVector WallPerimeterPoint(float AngleDeg) const;

    // --- Building placement --------------------------------------------------
    void PlaceBuildings();
    TArray<FDistrictDef> BuildDistrictDefs() const;
    EDistrictType  GetDistrictAt(FVector2D Pos) const;
    EBuildingStyle PickStyle(EDistrictType District, const FDistrictDef& Def);
    ERoofType      PickRoof(EBuildingStyle Style);
    int32          PickFloorCount(EBuildingStyle Style, EDistrictType District);
    FVector2D      BuildingSize(EBuildingStyle Style) const;
    bool           CanPlaceLot(FVector Center, float Radius, int32 IgnoreEdgeIndex = -1, float SpacingOverride = -1.f) const;

    // --- Modular building mesh generation ------------------------------------
    void SpawnModularBuilding(const FBuildingLot& Lot);
    UProceduralMeshComponent* SpawnFoundation(FVector Center, float W, float D,
                                               float Height, float Yaw);
    UProceduralMeshComponent* SpawnWallFloor(FVector BaseCenter, float W, float D,
                                              float FloorH, float Yaw, bool bAddWindows,
                                              bool bAddDoor);
    UProceduralMeshComponent* SpawnRoof_Pitched(FVector BaseCenter, float W, float D,
                                                 float RoofH, float Overhang, float Yaw);
    UProceduralMeshComponent* SpawnRoof_Hipped(FVector BaseCenter, float W, float D,
                                                float RoofH, float Overhang, float Yaw);
    UProceduralMeshComponent* SpawnRoof_Gambrel(FVector BaseCenter, float W, float D,
                                                 float RoofH, float Overhang, float Yaw);
    UProceduralMeshComponent* SpawnRoof_FlatParapet(FVector BaseCenter, float W, float D,
                                                     float ParapetH, float Yaw);
    UProceduralMeshComponent* SpawnRoof_Conical(FVector BaseCenter, float Radius,
                                                 float ConeH, int32 Segs);
    UProceduralMeshComponent* SpawnRoof_Pyramidal(FVector BaseCenter, float W, float D,
                                                   float PyramidH, float Yaw);
    UProceduralMeshComponent* SpawnChimney(FVector BaseCenter, float BuildingH,
                                            float Yaw, int32 Count);
    UProceduralMeshComponent* SpawnGroundProps(FVector Center, float W, float D, float Yaw,
                                                EBuildingStyle Style);

    // --- Road mesh -----------------------------------------------------------
    void SpawnRoadMesh(const FRoadEdge& Edge);

    // --- Foliage -------------------------------------------------------------
    void PlaceForest();
    UProceduralMeshComponent* SpawnTree(FVector Location, float Height, float CrownRadius,
                                         int32 CrownTiers = 3);
    float ForestDensityAt(float X, float Y) const;

    // --- Mountains -----------------------------------------------------------
    void SpawnMountains();
    UProceduralMeshComponent* SpawnMountainPeak(FVector Location, FVector Scale);

    // --- Geometry primitives -------------------------------------------------
    UProceduralMeshComponent* CreateMesh(const FString& Name);
    void SetMeshSection(UProceduralMeshComponent* Mesh, int32 Section,
                        TArray<FVector>& V, TArray<int32>& T,
                        TArray<FVector>& N, TArray<FVector2D>& UV,
                        UMaterialInterface* Mat = nullptr);
    void ApplyMaterial(UProceduralMeshComponent* Mesh, UMaterialInterface* Mat);

    // Box: solid 6-sided box
    void AddBox(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                TArray<FVector2D>& UV, FVector Center, float W, float D, float H);
    // Box with open top face
    void AddOpenTopBox(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                       TArray<FVector2D>& UV, FVector Center, float W, float D, float H);
    // Pitched (gabled) roof ridge
    void AddPitchedRoof(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                        TArray<FVector2D>& UV, FVector Base, float W, float D,
                        float H, float Overhang);
    // Hipped roof (4 triangular faces meeting at ridge)
    void AddHippedRoof(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                       TArray<FVector2D>& UV, FVector Base, float W, float D,
                       float H, float Overhang);
    // Cylinder
    void AddCylinder(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                     TArray<FVector2D>& UV, FVector Base, float Radius,
                     float Height, int32 Segments, bool bCap = true);
    // Cone
    void AddCone(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                 TArray<FVector2D>& UV, FVector Base, float Radius,
                 float Height, int32 Segments);
    // Quad (single planar face)
    void AddQuad(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                 TArray<FVector2D>& UV, FVector P0, FVector P1, FVector P2, FVector P3);
    // Pyramid
    void AddPyramid(TArray<FVector>& V, TArray<int32>& T, TArray<FVector>& N,
                    TArray<FVector2D>& UV, FVector Base, float W, float D, float H);

    // --- Math helpers --------------------------------------------------------
    FVector2D RandInsideCircle(float Radius);
    FVector2D RandAnnulus(float InnerR, float OuterR);
    bool CircleOverlapsSegment(FVector2D Center, float R, FVector2D A, FVector2D B) const;
    float Dist2D(FVector A, FVector B) const;
    FVector RotateAroundZ(FVector V, FVector Center, float AngleDeg) const;
};
