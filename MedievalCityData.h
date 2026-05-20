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

// =============================================================================
//  CANONICAL BUILDING & ROOF ENUMS
//  Defined here (not in the actor header) so PCG nodes can reference them
//  without pulling in the full AMedievalTownGenerator translation unit.
// =============================================================================

UENUM(BlueprintType)
enum class EMedievalBuildingStyle : uint8
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

UENUM(BlueprintType)
enum class EMedievalRoofType : uint8
{
    Pitched     UMETA(DisplayName = "Gabled / Pitched"),
    Hipped      UMETA(DisplayName = "Hipped"),
    Gambrel     UMETA(DisplayName = "Gambrel (Barn)"),
    FlatParapet UMETA(DisplayName = "Flat with Parapet"),
    Conical     UMETA(DisplayName = "Conical (Round Tower)"),
    Pyramidal   UMETA(DisplayName = "Pyramidal (Square Tower)"),
    Thatched    UMETA(DisplayName = "Thatched (Low Pitch)")
};

// =============================================================================
//  FMedievalParcel  --  Phase 1 → Phase 2 boundary struct
//
//  Produced by the layout agents (terrain, walls, roads, district planner).
//  Consumed by PCG asset nodes (buildings, walls, roads, bridges).
//  All geometry is world-space; Z is baked terrain elevation at lot center.
//
//  Party wall bit mask (PartyWallMask):
//    Bit 0 (0x01) = left side shared  → skip left wall mesh
//    Bit 1 (0x02) = right side shared → skip right wall mesh
//    Bit 2 (0x04) = rear side shared  → skip rear wall mesh
// =============================================================================

USTRUCT(BlueprintType)
struct FMedievalParcel
{
    GENERATED_BODY()

    // ---- Identity -----------------------------------------------------------

    /** Stable unique index within the current generation; used to seed PCG per-lot RNG. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Identity")
    int32 LotIndex = -1;

    // ---- Geometry -----------------------------------------------------------

    /** World-space lot center; Z = terrain height at this point. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    FVector WorldCenter = FVector::ZeroVector;

    /** Width of the lot along its street frontage (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    float FrontageWidth = 600.f;

    /** Depth of the lot perpendicular to the street (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    float LotDepth = 2000.f;

    /** Z-rotation of the lot so that +X aligns with the street facing direction (degrees). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    float Yaw = 0.f;

    /** Unit vector (world 2D) pointing from the lot center toward its street frontage. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    FVector2D StreetFacingDir = FVector2D(1.f, 0.f);

    /** Full lot boundary — 4 corners in world-space 2D, CCW winding, front edge first. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Geometry")
    TArray<FVector2D> LotPolygon;

    // ---- Terrain ------------------------------------------------------------

    /** Terrain Z at WorldCenter (same as WorldCenter.Z; stored separately for PCG attribute access). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Terrain")
    float GroundElevation = 0.f;

    /** Extra foundation height needed to level the building on a sloped lot (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Terrain")
    float FoundationDepth = 80.f;

    /** Average terrain slope across the lot footprint (degrees). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Terrain")
    float GroundSlopeDeg = 0.f;

    // ---- District & Style ---------------------------------------------------

    /** District zone this lot falls within; drives style pool and density rules. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    EMedievalWardType Ward = EMedievalWardType::ResidentialWard;

    /** Footprint archetype; determines how the parcel is subdivided and massed. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    EMedievalFootprintType FootprintType = EMedievalFootprintType::Rowhouse;

    /** Specific building programme placed on this lot. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    EMedievalBuildingStyle BuildingStyle = EMedievalBuildingStyle::TownHouse;

    /** Roof geometry variant for the main building volume. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    EMedievalRoofType RoofType = EMedievalRoofType::Pitched;

    /** Number of storeys above ground floor. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    int32 NumFloors = 2;

    /**
     * Normalised wealth score for this lot [0 = poorest, 1 = richest].
     * PCG uses this to select material tier: 0–0.33 = rough rubble,
     * 0.34–0.66 = coursed stone, 0.67–1.0 = dressed ashlar.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Style")
    float WealthScore = 0.5f;

    // ---- Adjacency signals --------------------------------------------------

    /** Distance from WorldCenter to the nearest river centreline point (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Adjacency")
    float DistToRiver = 1e9f;

    /** Distance from WorldCenter to the nearest wall segment (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Adjacency")
    float DistToWall = 1e9f;

    /** Distance from WorldCenter to the market square centre (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Adjacency")
    float DistToMarket = 1e9f;

    // ---- Flags --------------------------------------------------------------

    /** Building entrance faces the river rather than its street frontage. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Flags")
    bool bFacesRiver = false;

    /** Lot sits at a street intersection; PCG should wrap the facade on two faces. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Flags")
    bool bIsCornerLot = false;

    /** Bit 0=left, 1=right, 2=rear: set bit means that side is a shared party wall — skip mesh. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Flags")
    uint8 PartyWallMask = 0;

    // ---- Wing (L/T-plan buildings) ------------------------------------------

    /** True if the main volume has an attached wing forming an L or T plan. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Wing")
    bool bHasWing = false;

    /** Width × Depth of the wing volume (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Wing")
    FVector2D WingSize = FVector2D(250.f, 200.f);

    /** Yaw offset of the wing relative to the main body (typically 90° for L-plan). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Parcel|Wing")
    float WingYawOffset = 90.f;
};

// =============================================================================
//  WALL BOUNDARY STRUCTS  --  Phase 1 → Phase 2 contract for wall & gate data
//
//  The generator produces two parallel arrays:
//    TArray<FMedievalWallNode>    WallNodes    -- towers, gates, termini
//    TArray<FMedievalWallSegment> WallSegments -- straight runs between nodes
//
//  PCG asset nodes consume these to place:
//    - Wall section meshes      (FMedievalWallSegment, stepped along TerrainSamples)
//    - Crenellation strip       (FMedievalWallSegment, bHasBattlements)
//    - Corner tower meshes      (FMedievalWallNode, NodeType == CornerTower)
//    - Gate arch + twin towers  (FMedievalWallNode, NodeType == GateTower)
// =============================================================================

UENUM(BlueprintType)
enum class EMedievalWallNodeType : uint8
{
    CornerTower  UMETA(DisplayName = "Corner Tower"),   // Direction change; cylindrical tower
    GateTower    UMETA(DisplayName = "Gate Tower"),     // Town gate; twin towers + arch
    Terminus     UMETA(DisplayName = "Terminus")        // Wall ends here (river/cliff edge)
};

USTRUCT(BlueprintType)
struct FMedievalWallNode
{
    GENERATED_BODY()

    // ---- Identity -----------------------------------------------------------

    /** Stable index into the WallNodes array for this generation. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Identity")
    int32 NodeIndex = -1;

    // ---- Position & orientation ---------------------------------------------

    /** World-space position; Z = terrain height at this point. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    FVector WorldPos = FVector::ZeroVector;

    /** What occupies this node on the wall perimeter. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    EMedievalWallNodeType NodeType = EMedievalWallNodeType::CornerTower;

    /** Unit vector pointing away from the town centre (for orienting tower and gate meshes). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    FVector2D OutwardFacing = FVector2D(1.f, 0.f);

    // ---- Tower geometry (valid for CornerTower and GateTower) ---------------

    /** Outer radius of the cylindrical tower (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Tower")
    float TowerRadius = 220.f;

    /** Total height of the tower above ground (cm); typically WallHeight * TowerHeightFactor. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Tower")
    float TowerHeight = 1260.f;

    // ---- Gate geometry (valid for GateTower only) ---------------------------

    /** Clear width of the gate archway (cm); sets which gate mesh variant to select. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Gate")
    float GateWidth = 420.f;

    /** True for the gate that faces the primary approach road; PCG may use a grander arch. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Gate")
    bool bIsMainGate = false;

    // ---- Connectivity -------------------------------------------------------

    /** Index of the wall segment on the clockwise side of this node (-1 if Terminus). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Connectivity")
    int32 SegmentIndexCW = -1;

    /** Index of the wall segment on the counter-clockwise side of this node (-1 if Terminus). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Connectivity")
    int32 SegmentIndexCCW = -1;
};

USTRUCT(BlueprintType)
struct FMedievalWallSegment
{
    GENERATED_BODY()

    // ---- Identity -----------------------------------------------------------

    /** Stable index into the WallSegments array for this generation. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Identity")
    int32 SegmentIndex = -1;

    /** Index of the node at the start of this segment (clockwise winding). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Connectivity")
    int32 NodeIndexA = -1;

    /** Index of the node at the end of this segment. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Connectivity")
    int32 NodeIndexB = -1;

    // ---- Geometry -----------------------------------------------------------

    /** World-space start point (= WallNodes[NodeIndexA].WorldPos). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    FVector WorldStart = FVector::ZeroVector;

    /** World-space end point (= WallNodes[NodeIndexB].WorldPos). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    FVector WorldEnd = FVector::ZeroVector;

    /** Straight-line distance from WorldStart to WorldEnd (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    float Length = 0.f;

    /** Unit vector perpendicular to the segment, pointing outside the town perimeter. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    FVector2D OutwardNormal = FVector2D(1.f, 0.f);

    // ---- Wall dimensions ----------------------------------------------------

    /** Wall height above terrain at the segment midpoint (cm). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    float WallHeight = 900.f;

    /** Wall thickness (cm); constant across the segment. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    float WallThickness = 120.f;

    /** Whether the wall walk has a crenellated parapet on this segment. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    bool bHasBattlements = true;

    // ---- Terrain following --------------------------------------------------

    /**
     * World-space 3D sample points evenly distributed along the segment.
     * Z = terrain height at each point. PCG subdivides the segment into
     * wall section meshes at these points so the wall steps with the terrain
     * rather than floating or burying itself.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wall|Geometry")
    TArray<FVector> TerrainSamples;
};

// =============================================================================
//  ROAD BOUNDARY STRUCTS  --  Phase 1 → Phase 2 contract for street & bridge data
//
//  The generator produces two parallel arrays:
//    TArray<FMedievalStreetNode> StreetNodes -- intersections, gates, market, bridge ends
//    TArray<FMedievalStreetEdge> StreetEdges -- road segments between nodes
//
//  PCG asset nodes consume these to place:
//    - Road surface meshes    (FMedievalStreetEdge, splined along WorldPolyline)
//    - Intersection cap tiles (FMedievalStreetNode, classified by IntersectionType)
//    - Bridge deck + abutments(FMedievalStreetEdge, bIsBridge == true)
//    - Quay surface setts     (FMedievalStreetEdge, bIsQuayStreet == true)
//
//  EMedievalRoadSurface drives material selection at the PCG node boundary.
//  Grade drives surface variant within a tier: steep primaries stay cobbled,
//  not the smoother flagstone used on flat market approaches.
// =============================================================================

UENUM(BlueprintType)
enum class EMedievalIntersectionType : uint8
{
    Terminus    UMETA(DisplayName = "Terminus / Dead End"),
    TJunction   UMETA(DisplayName = "T Junction"),
    FourWay     UMETA(DisplayName = "Four-Way"),
    Gate        UMETA(DisplayName = "Town Gate"),
    Market      UMETA(DisplayName = "Market Square / Plaza")
};

UENUM(BlueprintType)
enum class EMedievalRoadSurface : uint8
{
    MarketFlagstone UMETA(DisplayName = "Market Flagstone"),   // Plaza / market square
    PrimaryCobble   UMETA(DisplayName = "Primary Cobble"),     // Main roads, gate approaches
    SecondaryDirt   UMETA(DisplayName = "Secondary Dirt"),     // Secondary roads, packed earth
    AlleyDirt       UMETA(DisplayName = "Alley Dirt"),         // Narrow lanes, loose earth
    BridgeDeck      UMETA(DisplayName = "Bridge Deck"),        // Timber or stone bridge surface
    QuaySetts       UMETA(DisplayName = "Quay Setts")          // Waterfront granite sett paving
};

USTRUCT(BlueprintType)
struct FMedievalStreetNode
{
    GENERATED_BODY()

    // ---- Identity -----------------------------------------------------------

    /** Stable index into the StreetNodes array for this generation. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Identity")
    int32 NodeIndex = -1;

    // ---- Position -----------------------------------------------------------

    /** World-space position; Z = terrain height at this intersection. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Geometry")
    FVector WorldPos = FVector::ZeroVector;

    // ---- Classification -----------------------------------------------------

    /** Intersection topology; drives which cap tile or plaza mesh PCG selects. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Type")
    EMedievalIntersectionType IntersectionType = EMedievalIntersectionType::TJunction;

    // ---- Connectivity -------------------------------------------------------

    /** Indices of all edges that meet at this node (into the StreetEdges array). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Connectivity")
    TArray<int32> ConnectedEdgeIndices;

    // ---- Semantic flags -----------------------------------------------------

    /** This node is the market square anchor; PCG places paving and stall props here. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Flags")
    bool bIsMarket = false;

    /** This node coincides with a town gate; road surface transitions to PrimaryCobble. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Flags")
    bool bIsGate = false;

    /** This node is a bridge abutment; PCG places approach ramp and abutment cap here. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Flags")
    bool bIsBridgeAbutment = false;
};

USTRUCT(BlueprintType)
struct FMedievalStreetEdge
{
    GENERATED_BODY()

    // ---- Identity -----------------------------------------------------------

    /** Stable index into the StreetEdges array for this generation. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Identity")
    int32 EdgeIndex = -1;

    /** Index of the node at the start of this edge (into the StreetNodes array). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Connectivity")
    int32 NodeIndexA = -1;

    /** Index of the node at the end of this edge. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Connectivity")
    int32 NodeIndexB = -1;

    // ---- Road classification ------------------------------------------------

    /** Hierarchy tier; drives width, surface, and prop density defaults. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Type")
    EMedievalStreetType StreetType = EMedievalStreetType::Secondary;

    /**
     * Surface material for this edge.
     * Derived from StreetType + GradeDeg: steep primaries → PrimaryCobble even
     * near the market; flat alleys → AlleyDirt; quay roads → QuaySetts regardless of tier.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Type")
    EMedievalRoadSurface Surface = EMedievalRoadSurface::SecondaryDirt;

    // ---- Dimensions ---------------------------------------------------------

    /** Carriageway width at the midpoint of the edge (cm); may vary slightly end-to-end. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Geometry")
    float Width = 280.f;

    /** Maximum slope angle along the edge (degrees); used for surface variant selection. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Geometry")
    float GradeDeg = 0.f;

    // ---- Geometry -----------------------------------------------------------

    /**
     * Terrain-following 3D polyline: all points have terrain Z already baked in.
     * PCG lofts the road surface mesh along this spline.
     * Minimum 2 points (straight edge); more points for curved or terrain-hugging roads.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Geometry")
    TArray<FVector> WorldPolyline;

    // ---- Bridge data (valid only when bIsBridge == true) --------------------

    /** True if this edge crosses the river and requires a bridge mesh. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Bridge")
    bool bIsBridge = false;

    /**
     * Clear span between inner abutment faces (cm).
     * PCG uses this to select a bridge deck mesh of the right length,
     * or to tile multiple deck sections if the span exceeds one mesh.
     */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Bridge")
    float BridgeSpanLength = 0.f;

    /** Terrain Z at the left abutment; PCG sizes the left pier/abutment block to this depth. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Bridge")
    float LeftBankElevation = 0.f;

    /** Terrain Z at the right abutment; PCG sizes the right pier/abutment block to this depth. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Bridge")
    float RightBankElevation = 0.f;

    // ---- Special flags ------------------------------------------------------

    /** Edge runs parallel to the river bank; surface defaults to QuaySetts. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Street|Flags")
    bool bIsQuayStreet = false;
};

// -----------------------------------------------------------------------------
//  Organic street generation parameters
//  Exposed as a USTRUCT so they can be edited in Details and stored in profiles.
// -----------------------------------------------------------------------------
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

    /** ?fraction applied as width noise per edge segment */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Organic Streets")
    float WidthNoiseFraction = 0.15f;
};
