// =============================================================================
// MedievalTownGenerator.cpp  —  VERSION 18
// =============================================================================
// Full implementation of every system described in MedievalTownGenerator.h
//
// Section map:
//   §1  Constructor / lifecycle
//   §2  Main pipeline  (GenerateTown / ClearTown / phases 1–9)
//   §3  Terrain  (Perlin noise, height cache, slope detection)
//   §4  River path generation (waypoints + world path)
//   §5  Road network  (radioconcentric: center→gate radials + ring roads + connectors)
//   §6  Shape grammar walls
//   §7  Building placement  (road-lining + scatter fill, district-aware)
//   §8  Modular building meshes  (foundation + floors + 6 roof variants + props)
//   §9  Road mesh rendering
//   §10 Forest (Perlin-density noise ring)
//   §11 Mountains
//   §12 Save / Load layout
//   §13 Geometry primitives  (Box, Cylinder, Cone, Pitched/Hipped/Gambrel/Pyramid roofs)
//   §14 Math helpers
// =============================================================================

#include "MedievalTownGenerator.h"
#include "KismetProceduralMeshLibrary.h"
#include "Engine/World.h"

// ─────────────────────────────────────────────────────────────────────────────
//  §1  CONSTRUCTOR / LIFECYCLE
// ─────────────────────────────────────────────────────────────────────────────

AMedievalTownGenerator::AMedievalTownGenerator()
{
    PrimaryActorTick.bCanEverTick = false;
    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
}

void AMedievalTownGenerator::BeginPlay()
{
    Super::BeginPlay();
    if (bGenerateOnBeginPlay)
        GenerateTown();
}

#if WITH_EDITOR
void AMedievalTownGenerator::PostEditChangeProperty(FPropertyChangedEvent& Evt)
{
    Super::PostEditChangeProperty(Evt);
    if (bAutoRegenerateInEditor && Evt.Property)
        GenerateTown();
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  §2  MAIN PIPELINE
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::GenerateTown()
{
    ClearTown();
    Rand.Initialize(RandomSeed);
    UE_LOG(LogTemp, Log, TEXT("[MTG] GenerateTown seed=%d radius=%.0f"), RandomSeed, TownRadius);

    if (bUseSavedLayout && SavedLayout.bIsValid)
    {
        LoadSavedLayout();
        return;
    }

    // ── Pipeline order: River → Terrain → Walls → Roads → Buildings → Meshes ──
    Phase1_GenerateRiverWaypoints();  // River waypoints (drives terrain carving)
    Phase2_SetupTerrain();            // Terrain mesh (carves river channel)
    Phase3_BuildRiverWorldPath();     // River world-space path
    Phase4_BuildWalls();              // Walls + gates (must precede roads)
    Phase5_BuildRoadNetwork();        // Roads route to wall gates; bridges over river
    Phase6_PlaceBuildings();          // Buildings avoid roads + river
    Phase7_SpawnMeshes();             // Plaza, buildings, roads, bridges, river meshes
    Phase8_PlaceForest();
    Phase9_BuildMountains();

    UE_LOG(LogTemp, Log, TEXT("[MTG] Generation complete. Lots=%d Roads=%d"),
           PlacedLots.Num(), RoadEdges.Num());
}

void AMedievalTownGenerator::ClearTown()
{
    for (UProceduralMeshComponent* M : GeneratedMeshes)
        if (IsValid(M)) M->DestroyComponent();
    GeneratedMeshes.Empty();
    PlacedLots.Empty();
    RoadNodes.Empty();
    RoadEdges.Empty();
    WallPerimeter.Empty();
    GatePositions.Empty();
    CachedRiverWorldPath.Empty();
    River.Waypoints.Empty();
    TerrainHeightCache.Empty();
    TerrainCacheRes = 0;
}

void AMedievalTownGenerator::RegenerateWithNewSeed(int32 NewSeed)
{
    RandomSeed = NewSeed;
    GenerateTown();
}

void AMedievalTownGenerator::Phase1_GenerateRiverWaypoints()
{
    if (!bGenerateRiver) return;
    GenerateRiverWaypoints();
}

void AMedievalTownGenerator::Phase2_SetupTerrain()
{
    BuildTerrainCache();
    // Spawn terrain mesh
    const FString Name = TEXT("Terrain");
    UProceduralMeshComponent* Mesh = CreateMesh(Name);

    const int32 Res = TerrainResolution;
    // Terrain must cover the full forest ring so trees have ground beneath them
    const float Ext = TownRadius * FMath::Max(1.1f, ForestRingOuterFraction + 0.1f);
    const float Step = (Ext * 2.f) / Res;
    const FVector Origin = GetActorLocation();

    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Pre-compute vertex grid with terrain-gradient normals for smooth shading
    // Grid of (Res+1) x (Res+1) vertices
    const int32 VPerRow = Res + 1;
    V.Reserve(VPerRow * VPerRow);
    N.Reserve(VPerRow * VPerRow);
    UV.Reserve(VPerRow * VPerRow);

    for (int32 Row = 0; Row <= Res; Row++)
    {
        for (int32 Col = 0; Col <= Res; Col++)
        {
            float X = -Ext + Col * Step;
            float Y = -Ext + Row * Step;
            float H = GetTerrainHeight(X, Y);

            V.Add(FVector(X, Y, H));
            UV.Add(FVector2D((float)Col / Res, (float)Row / Res));

            // Per-vertex normal from terrain gradient (smooth shading)
            const float Delta = Step * 0.5f;
            float Hx = GetTerrainHeight(X + Delta, Y) - GetTerrainHeight(X - Delta, Y);
            float Hy = GetTerrainHeight(X, Y + Delta) - GetTerrainHeight(X, Y - Delta);
            FVector VNorm(-Hx / (2.f * Delta), -Hy / (2.f * Delta), 1.f);
            N.Add(VNorm.GetSafeNormal());
        }
    }

    // Generate triangle indices with correct winding (CCW from above = front face)
    T.Reserve(Res * Res * 6);
    for (int32 Row = 0; Row < Res; Row++)
    {
        for (int32 Col = 0; Col < Res; Col++)
        {
            int32 BL = Row * VPerRow + Col;
            int32 BR = BL + 1;
            int32 TL = BL + VPerRow;
            int32 TR = TL + 1;

            // Two triangles per quad — CW from above = front face in UE5 left-handed
            T.Add(BL); T.Add(TL); T.Add(BR);
            T.Add(BR); T.Add(TL); T.Add(TR);
        }
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, GroundMaterial);
}

void AMedievalTownGenerator::Phase3_BuildRiverWorldPath()
{
    if (!bGenerateRiver) return;
    BuildRiverWorldPath();
}

void AMedievalTownGenerator::Phase4_BuildWalls()
{
    if (!bGenerateWalls) return;
    GenerateWalls();
}

void AMedievalTownGenerator::Phase5_BuildRoadNetwork()
{
    BuildRoadNetwork();
}

void AMedievalTownGenerator::Phase6_PlaceBuildings()
{
    PlaceBuildings();
}

void AMedievalTownGenerator::Phase7_SpawnMeshes()
{
    // ── Market Plaza & Well ───────────────────────────────────────────────────
    {
        float PlazaR = TownRadius * 0.1f;
        float PlazaH = GetTerrainHeight(0.f, 0.f);
        FVector PlazaCenter = GetActorLocation() + FVector(0.f, 0.f, PlazaH + 2.f);

        UProceduralMeshComponent* PlazaMesh = CreateMesh(TEXT("Plaza"));
        TArray<FVector> PV; TArray<int32> PT; TArray<FVector> PN; TArray<FVector2D> PUV;
        const int32 PlazaSegs = 24;
        PV.Add(FVector::ZeroVector); PN.Add(FVector::UpVector); PUV.Add(FVector2D(0.5f,0.5f));
        for (int32 S = 0; S <= PlazaSegs; S++)
        {
            float Ang = (float)S / PlazaSegs * TWO_PI;
            float PX = FMath::Cos(Ang) * PlazaR;
            float PY = FMath::Sin(Ang) * PlazaR;
            PV.Add(FVector(PX, PY, 0.f));
            PN.Add(FVector::UpVector);
            PUV.Add(FVector2D(FMath::Cos(Ang)*0.5f+0.5f, FMath::Sin(Ang)*0.5f+0.5f));
            if (S > 0 && S <= PlazaSegs)
            {
                int32 Next = (S % PlazaSegs) + 1;
                PT.Add(0); PT.Add(Next); PT.Add(S);   // CW from above = front face in UE5
            }
        }
        SetMeshSection(PlazaMesh, 0, PV, PT, PN, PUV, StoneMaterial);
        PlazaMesh->SetWorldLocation(PlazaCenter);

        // Central well
        UProceduralMeshComponent* WellMesh = CreateMesh(TEXT("Well"));
        TArray<FVector> WV; TArray<int32> WT; TArray<FVector> WN; TArray<FVector2D> WUV;
        float WellR = PlazaR * 0.06f;
        float WellH = PlazaR * 0.12f;
        AddCylinder(WV, WT, WN, WUV, FVector::ZeroVector, WellR, WellH, 10, true);
        AddCone(WV, WT, WN, WUV, FVector(0,0,WellH), WellR*1.2f, WellH*0.5f, 10);
        SetMeshSection(WellMesh, 0, WV, WT, WN, WUV, StoneMaterial);
        WellMesh->SetWorldLocation(PlazaCenter);
    }

    // ── Buildings ─────────────────────────────────────────────────────────────
    for (const FBuildingLot& Lot : PlacedLots)
        if (Lot.bIsPlaced)
            SpawnModularBuilding(Lot);

    // ── Road & Bridge meshes ─────────────────────────────────────────────────
    for (const FRoadEdge& Edge : RoadEdges)
    {
        if (!Edge.bIsGenerated || Edge.WorldPoints.Num() < 2) continue;
        // Always render road surface for the full length
        SpawnRoadMesh(Edge);
        // For bridge edges, add railings only over the river portion
        if (Edge.bIsBridge)
            SpawnBridgeMesh(Edge);
    }

    // ── River ─────────────────────────────────────────────────────────────────
    if (bGenerateRiver && CachedRiverWorldPath.Num() >= 2)
    {
        UProceduralMeshComponent* RiverMesh = CreateMesh(TEXT("River"));
        TArray<FVector> RV; TArray<int32> RT; TArray<FVector> RN; TArray<FVector2D> RUV;
        const float HalfW = RiverWidth * 0.5f;

        for (int32 i = 0; i < CachedRiverWorldPath.Num() - 1; i++)
        {
            FVector P0 = CachedRiverWorldPath[i];
            FVector P1 = CachedRiverWorldPath[i + 1];
            FVector Dir = (P1 - P0); Dir.Z = 0.f;
            float Len = Dir.Size2D();
            if (Len < 1.f) continue;
            Dir /= Len;
            FVector Right(Dir.Y, -Dir.X, 0.f);

            FVector L0 = P0 - Right*HalfW; L0.Z = P0.Z;
            FVector R0 = P0 + Right*HalfW; R0.Z = P0.Z;
            FVector L1 = P1 - Right*HalfW; L1.Z = P1.Z;
            FVector R1 = P1 + Right*HalfW; R1.Z = P1.Z;

            int32 Base = RV.Num();
            RV.Add(L0); RV.Add(R0); RV.Add(L1); RV.Add(R1);
            RUV.Add(FVector2D(0,(float)i)); RUV.Add(FVector2D(1,(float)i));
            RUV.Add(FVector2D(0,(float)(i+1))); RUV.Add(FVector2D(1,(float)(i+1)));
            RN.Add(FVector::UpVector); RN.Add(FVector::UpVector);
            RN.Add(FVector::UpVector); RN.Add(FVector::UpVector);
            // CW from above = front face visible from +Z in UE5 (river surface)
            RT.Add(Base); RT.Add(Base+2); RT.Add(Base+1);
            RT.Add(Base+1); RT.Add(Base+2); RT.Add(Base+3);
        }
        if (RV.Num() > 0)
        {
            SetMeshSection(RiverMesh, 0, RV, RT, RN, RUV, WaterMaterial);
            RiverMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        }
    }
}

void AMedievalTownGenerator::Phase8_PlaceForest()
{
    PlaceForest();
}

void AMedievalTownGenerator::Phase9_BuildMountains()
{
    SpawnMountains();
}

// ─────────────────────────────────────────────────────────────────────────────
//  §3  TERRAIN
// ─────────────────────────────────────────────────────────────────────────────

// Deterministic hash-based noise (no external dependency)
static float HashNoise(int32 X, int32 Y)
{
    int32 N = X + Y * 57;
    N = (N << 13) ^ N;
    return 1.f - ((N * (N * N * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.f;
}

static float SmoothNoise(float X, float Y)
{
    int32 IX = (int32)FMath::FloorToInt(X);
    int32 IY = (int32)FMath::FloorToInt(Y);
    float FX = X - IX, FY = Y - IY;
    float UX = FX * FX * (3.f - 2.f * FX);
    float UY = FY * FY * (3.f - 2.f * FY);

    float V00 = HashNoise(IX,   IY);
    float V10 = HashNoise(IX+1, IY);
    float V01 = HashNoise(IX,   IY+1);
    float V11 = HashNoise(IX+1, IY+1);

    return FMath::Lerp(FMath::Lerp(V00, V10, UX),
                       FMath::Lerp(V01, V11, UX), UY);
}

float AMedievalTownGenerator::SampleNoise(float X, float Y, int32 Octaves,
                                           float Freq, float Amp,
                                           float Persistence, float Lacunarity) const
{
    float Value = 0.f, A = Amp, F = Freq;
    for (int32 O = 0; O < Octaves; O++)
    {
        Value += SmoothNoise(X * F, Y * F) * A;
        A *= Persistence;
        F *= Lacunarity;
    }
    return Value;
}

float AMedievalTownGenerator::GetTerrainHeight(float X, float Y) const
{
    // 1. Base terrain noise (full amplitude)
    float H = SampleNoise(X, Y, TerrainOctaves, TerrainFrequency,
                           TerrainAmplitude, 0.5f, 2.f);

    // 2. Flatten terrain inside the walled area for gentle hills,
    //    smooth transition to full wild terrain outside the walls.
    float D = FMath::Sqrt(X * X + Y * Y);
    float WallR = TownRadius;
    float TransitionR = WallR * (1.f + TownFlattenTransition);

    if (D < TransitionR)
    {
        // How much to flatten: inside walls = TownFlattenStrength, at transition edge = 0
        float FlattenAlpha;
        if (D < WallR)
        {
            // Fully inside town — apply full flatten strength
            FlattenAlpha = TownFlattenStrength;
        }
        else
        {
            // Transition zone — smoothly interpolate from flattened to wild
            float T = (D - WallR) / (TransitionR - WallR);
            T = T * T * (3.f - 2.f * T);  // Smoothstep for natural blend
            FlattenAlpha = TownFlattenStrength * (1.f - T);
        }

        // Flatten toward 0 (base level) — keeping (1-alpha) fraction for gentle hills
        H = FMath::Lerp(H, 0.f, FlattenAlpha);
    }

    // 3. Extra flatten for center (market plaza area) — on top of town flatten
    float PlazaR = TownRadius * 0.12f;
    if (D < PlazaR)
    {
        float FlattenT = 1.f - (D / PlazaR);
        FlattenT = FlattenT * FlattenT;
        H = FMath::Lerp(H, 0.f, FlattenT);
    }

    // 4. Carve river channel with steep banks and flat riverbed
    if (River.Waypoints.Num() >= 2)
    {
        FVector2D Pos(X, Y);
        float MinRiverDist = 1e9f;
        for (int32 i = 0; i < River.Waypoints.Num() - 1; i++)
        {
            FVector2D A = River.Waypoints[i], B = River.Waypoints[i + 1];
            FVector2D AB = B - A, AP = Pos - A;
            float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                                   FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
            MinRiverDist = FMath::Min(MinRiverDist, (Pos - (A + AB * T)).Size());
        }
        float HalfRiver = RiverWidth * 0.5f;
        float BankWidth = HalfRiver * 0.35f;   // Steep transition zone
        float ChannelDepth = 160.f;             // Deep channel

        if (MinRiverDist < HalfRiver + BankWidth)
        {
            if (MinRiverDist < HalfRiver)
            {
                // Inside river channel — flat riverbed
                H -= ChannelDepth;
            }
            else
            {
                // Bank transition zone — steep falloff using smoothstep
                float BankT = (MinRiverDist - HalfRiver) / BankWidth;
                BankT = BankT * BankT * (3.f - 2.f * BankT);  // Smoothstep
                H -= ChannelDepth * (1.f - BankT);
            }
        }
    }

    return H;
}

float AMedievalTownGenerator::GetTerrainHeightNoRiver(float X, float Y) const
{
    // Same as GetTerrainHeight but WITHOUT the river channel carve.
    // Used to find the bank-level height for placing the river water surface.
    float H = SampleNoise(X, Y, TerrainOctaves, TerrainFrequency,
                           TerrainAmplitude, 0.5f, 2.f);

    float D = FMath::Sqrt(X * X + Y * Y);
    float WallR = TownRadius;
    float TransitionR = WallR * (1.f + TownFlattenTransition);

    if (D < TransitionR)
    {
        float FlattenAlpha;
        if (D < WallR)
            FlattenAlpha = TownFlattenStrength;
        else
        {
            float T = (D - WallR) / (TransitionR - WallR);
            T = T * T * (3.f - 2.f * T);
            FlattenAlpha = TownFlattenStrength * (1.f - T);
        }
        H = FMath::Lerp(H, 0.f, FlattenAlpha);
    }

    float PlazaR = TownRadius * 0.12f;
    if (D < PlazaR)
    {
        float FlattenT = 1.f - (D / PlazaR);
        FlattenT = FlattenT * FlattenT;
        H = FMath::Lerp(H, 0.f, FlattenT);
    }

    // NO river carve — returns bank-level height
    return H;
}

FTerrainSample AMedievalTownGenerator::SampleTerrain(float X, float Y) const
{
    FTerrainSample Out;
    Out.Height = GetTerrainHeight(X, Y);

    // Compute finite-difference normal
    // Use a larger delta to average out high-frequency octaves and get representative slope
    const float Delta = 200.f;
    float Hx = GetTerrainHeight(X + Delta, Y) - GetTerrainHeight(X - Delta, Y);
    float Hy = GetTerrainHeight(X, Y + Delta) - GetTerrainHeight(X, Y - Delta);
    Out.Normal = FVector(-Hx / (2.f * Delta), -Hy / (2.f * Delta), 1.f).GetSafeNormal();

    float SlopeDeg = FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Out.Normal.Z, -1.f, 1.f)));
    Out.bIsFlat = SlopeDeg < MaxSlopeForBuilding;

    return Out;
}

bool AMedievalTownGenerator::IsTerrainFlat(FVector2D Center, float HalfW, float HalfD) const
{
    // Sample 4 corners + center — all must be flat
    static const FVector2D Offsets[] = {
        FVector2D(1, 1), FVector2D(-1, 1), FVector2D(1, -1), FVector2D(-1, -1), FVector2D(0, 0)
    };
    for (const FVector2D& Off : Offsets)
    {
        FTerrainSample S = SampleTerrain(Center.X + Off.X * HalfW,
                                          Center.Y + Off.Y * HalfD);
        if (!S.bIsFlat) return false;
    }
    return true;
}

void AMedievalTownGenerator::BuildTerrainCache()
{
    // Pre-bake terrain for faster IsTerrainFlat queries
    // (we don't actually need this if sampling directly, but good for LOD meshes)
    TerrainCacheRes = TerrainResolution;
}

// ─────────────────────────────────────────────────────────────────────────────
//  §4  RIVER PATH
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::GenerateRiverWaypoints()
{
    River.Width = RiverWidth;
    River.ExclusionRadius = RiverExclusionRadius;
    River.Waypoints.Empty();

    // River enters from one side, exits the other, curving through town
    float EntryAngle = FMath::DegreesToRadians(RiverEntryAngleDeg +
                        Rand.FRandRange(-20.f, 20.f));
    float ExitAngle = EntryAngle + FMath::DegreesToRadians(Rand.FRandRange(140.f, 200.f));

    FVector2D Entry = FVector2D(FMath::Cos(EntryAngle), FMath::Sin(EntryAngle)) *
                      TownRadius * 1.15f;
    FVector2D Exit  = FVector2D(FMath::Cos(ExitAngle),  FMath::Sin(ExitAngle))  *
                      TownRadius * 1.15f;

    River.Waypoints.Add(Entry);

    // Add N intermediate points that meander, biased away from center
    const int32 Extra = FMath::Max(RiverWaypoints - 2, 0);
    FVector2D Prev = Entry;
    for (int32 i = 0; i < Extra; i++)
    {
        float T = (float)(i + 1) / (Extra + 1);
        FVector2D BasePt = FMath::Lerp(Entry, Exit, T);

        // Perpendicular meander
        FVector2D Dir = (Exit - Entry).GetSafeNormal();
        FVector2D Perp(-Dir.Y, Dir.X);
        float Meander = Rand.FRandRange(-TownRadius * 0.25f, TownRadius * 0.25f);
        BasePt += Perp * Meander;

        // Keep outside market area
        float DistFromCenter = BasePt.Size();
        if (DistFromCenter < TownRadius * 0.25f)
        {
            BasePt = BasePt.GetSafeNormal() * TownRadius * 0.3f;
        }

        River.Waypoints.Add(BasePt);
    }
    River.Waypoints.Add(Exit);
}

void AMedievalTownGenerator::BuildRiverWorldPath()
{
    // Build world-space path for the river water surface.
    // CRITICAL: Use GetTerrainHeightNoRiver() (bank height) so the water
    // sits at the terrain surface level, NOT at the bottom of the carved channel.
    // The river mesh will then be offset slightly below ground for the "water in channel" look.
    CachedRiverWorldPath.Empty();
    const int32 SamplesPerSegment = 8;
    for (int32 i = 0; i < River.Waypoints.Num() - 1; i++)
    {
        FVector2D A = River.Waypoints[i], B = River.Waypoints[i + 1];
        for (int32 s = 0; s < SamplesPerSegment; s++)
        {
            float T = (float)s / SamplesPerSegment;
            FVector2D Pt = FMath::Lerp(A, B, T);
            float BankH = GetTerrainHeightNoRiver(Pt.X, Pt.Y);
            // Water surface sits 55 units below the bank edge (in the 160-unit channel)
            CachedRiverWorldPath.Add(GetActorLocation() + FVector(Pt.X, Pt.Y, BankH - 55.f));
        }
    }
    // Add final point
    FVector2D Last = River.Waypoints.Last();
    float BankH = GetTerrainHeightNoRiver(Last.X, Last.Y);
    CachedRiverWorldPath.Add(GetActorLocation() + FVector(Last.X, Last.Y, BankH - 55.f));
}

bool AMedievalTownGenerator::IsNearRiver(FVector2D Pos, float ExtraRadius) const
{
    const float R = River.ExclusionRadius + ExtraRadius;
    for (int32 i = 0; i < River.Waypoints.Num() - 1; i++)
    {
        FVector2D A = River.Waypoints[i], B = River.Waypoints[i + 1];
        FVector2D AB = B - A, AP = Pos - A;
        float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                               FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
        FVector2D Closest = A + AB * T;
        if ((Pos - Closest).Size() < R) return true;
    }
    return false;
}

float AMedievalTownGenerator::DistToRiverCenter(FVector2D Pos) const
{
    float MinD = 1e9f;
    for (int32 i = 0; i < River.Waypoints.Num() - 1; i++)
    {
        FVector2D A = River.Waypoints[i], B = River.Waypoints[i + 1];
        FVector2D AB = B - A, AP = Pos - A;
        float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                               FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
        MinD = FMath::Min(MinD, (Pos - (A + AB * T)).Size());
    }
    return MinD;
}

bool AMedievalTownGenerator::SegmentCrossesRiver(FVector2D SA, FVector2D SB) const
{
    if (River.Waypoints.Num() < 2) return false;
    // Sample along the segment; if any sample is within river width, it crosses
    const float HalfW = RiverWidth * 0.5f;
    const int32 Samples = 10;
    for (int32 S = 0; S <= Samples; S++)
    {
        float T = (float)S / Samples;
        FVector2D Pt = FMath::Lerp(SA, SB, T);
        if (DistToRiverCenter(Pt) < HalfW * 1.2f)
            return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  §5  ROAD NETWORK  (Radioconcentric: radial spokes + ring roads)
// ─────────────────────────────────────────────────────────────────────────────
//
//  Medieval towns typically had a radioconcentric layout:
//    • Central market plaza
//    • Radial roads from market to each gate
//    • 1-2 concentric ring roads connecting the radials
//    • Buildings placed in the blocks formed between roads
//
//  This function creates a structured road network:
//    1. Radial spokes: center → inner ring → outer ring → gate
//    2. Inner ring: arc segments connecting adjacent radial intersections
//    3. Outer ring: arc segments connecting adjacent radial intersections
//    4. Mid-block connectors: short streets between rings for smaller blocks

void AMedievalTownGenerator::BuildRadiocentricRoads()
{
    RoadNodes.Empty();
    RoadEdges.Empty();

    // ── 1. Get gate positions from walls (already built) ───────────────────
    //    If walls produced GatePositions, use those exact positions so roads
    //    align with wall gates. Otherwise fall back to evenly-spaced placement.

    struct FGateInfo
    {
        FVector2D Pos;
        float Angle;
        int32 NodeIndex;
    };
    TArray<FGateInfo> Gates;

    if (GatePositions.Num() > 0)
    {
        // Walls already placed — extract 2D local positions from world-space gates
        FVector Origin = GetActorLocation();
        for (const FVector& GateWorld : GatePositions)
        {
            FVector2D GateLocal(GateWorld.X - Origin.X, GateWorld.Y - Origin.Y);
            FGateInfo GI;
            GI.Pos = GateLocal;
            GI.Angle = FMath::RadiansToDegrees(FMath::Atan2(GateLocal.Y, GateLocal.X));
            if (GI.Angle < 0.f) GI.Angle += 360.f;
            GI.NodeIndex = -1;
            Gates.Add(GI);
        }
    }
    else
    {
        // Fallback: evenly-spaced gates
        const int32 NumG = FMath::Clamp(NumGates, 2, 8);
        for (int32 G = 0; G < NumG; G++)
        {
            float Angle = (float)G / NumG * 360.f;
            float Jitter = Rand.FRandRange(-8.f, 8.f);
            float Rad = FMath::DegreesToRadians(Angle + Jitter);
            FVector2D GatePos = FVector2D(FMath::Cos(Rad), FMath::Sin(Rad)) * TownRadius * 0.92f;

            FGateInfo GI;
            GI.Pos = GatePos;
            GI.Angle = Angle + Jitter;
            if (GI.Angle < 0.f) GI.Angle += 360.f;
            GI.NodeIndex = -1;
            Gates.Add(GI);
        }
    }

    const int32 NumG = Gates.Num();

    // Sort by angle for ring connectivity
    Gates.Sort([](const FGateInfo& A, const FGateInfo& B){ return A.Angle < B.Angle; });

    // ── 2. Create nodes ────────────────────────────────────────────────────

    // Helper to add a node
    auto AddNode = [this](FVector2D Pos, bool bMarket = false, bool bGate = false) -> int32
    {
        FRoadNode N;
        N.Pos = Pos;
        N.bIsMarket = bMarket;
        N.bIsGate = bGate;
        N.Index = RoadNodes.Num();
        RoadNodes.Add(N);
        return N.Index;
    };

    // Helper to add an edge
    auto AddEdge = [this](int32 A, int32 B, EStreetTier Tier) -> int32
    {
        FRoadEdge E;
        E.NodeA = A;
        E.NodeB = B;
        E.Tier = Tier;
        E.bIsGenerated = true;
        switch (Tier)
        {
        case EStreetTier::Primary:   E.Width = PrimaryRoadWidth; break;
        case EStreetTier::Secondary: E.Width = SecondaryRoadWidth; break;
        case EStreetTier::Tertiary:  E.Width = TertiaryRoadWidth; break;
        default:                     E.Width = SecondaryRoadWidth; break;
        }
        int32 Idx = RoadEdges.Num();
        RoadEdges.Add(E);
        return Idx;
    };

    // Node 0: Market center
    int32 CenterNode = AddNode(FVector2D(0.f, 0.f), true);

    // Gate nodes — if GatePositions was empty (no walls), populate it now
    bool bNeedGatePositions = (GatePositions.Num() == 0);
    for (FGateInfo& GI : Gates)
    {
        GI.NodeIndex = AddNode(GI.Pos, false, true);
        if (bNeedGatePositions)
        {
            float H = GetTerrainHeight(GI.Pos.X, GI.Pos.Y);
            GatePositions.Add(GetActorLocation() + FVector(GI.Pos.X, GI.Pos.Y, H));
        }
    }

    // Ring intersection nodes (where each radial meets each ring)
    float Ring1R = InnerRingRadius * TownRadius;
    float Ring2R = OuterRingRadius * TownRadius;

    // For each gate, store ring1 and ring2 intersection node indices
    TArray<int32> Ring1Nodes, Ring2Nodes;

    for (const FGateInfo& GI : Gates)
    {
        FVector2D Dir = GI.Pos.GetSafeNormal();
        int32 R1Node = AddNode(Dir * Ring1R);
        int32 R2Node = AddNode(Dir * Ring2R);
        Ring1Nodes.Add(R1Node);
        Ring2Nodes.Add(R2Node);
    }

    // ── 3. Create radial edges (spokes) ────────────────────────────────────
    //    center → ring1 → ring2 → gate  for each gate

    for (int32 G = 0; G < NumG; G++)
    {
        AddEdge(CenterNode, Ring1Nodes[G], EStreetTier::Primary);
        AddEdge(Ring1Nodes[G], Ring2Nodes[G], EStreetTier::Primary);
        AddEdge(Ring2Nodes[G], Gates[G].NodeIndex, EStreetTier::Primary);
    }

    // ── 4. Create ring road arcs ───────────────────────────────────────────
    //    For each pair of adjacent gates, add arc nodes + edges along the ring circle.
    //    RingArcSubdivisions intermediate nodes per segment give smooth arcs.

    int32 ArcSubs = FMath::Clamp(RingArcSubdivisions, 1, 4);

    // Ring 1 (inner) — Secondary roads
    for (int32 G = 0; G < NumG; G++)
    {
        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);

        // Handle wrap-around (e.g. 350° to 10°)
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        // Create intermediate arc nodes
        TArray<int32> ArcChain;
        ArcChain.Add(Ring1Nodes[G]);

        for (int32 S = 1; S <= ArcSubs; S++)
        {
            float T = (float)S / (ArcSubs + 1);
            float A = FMath::Lerp(Angle1, Angle2, T);
            FVector2D ArcPt(FMath::Cos(A) * Ring1R, FMath::Sin(A) * Ring1R);

            // Skip if too close to river
            if (!IsNearRiver(ArcPt, 400.f))
                ArcChain.Add(AddNode(ArcPt));
        }
        ArcChain.Add(Ring1Nodes[Next]);

        // Connect the chain with edges
        for (int32 i = 0; i < ArcChain.Num() - 1; i++)
        {
            FVector2D PA2 = RoadNodes[ArcChain[i]].Pos;
            FVector2D PB2 = RoadNodes[ArcChain[i + 1]].Pos;
            FVector2D Mid2 = (PA2 + PB2) * 0.5f;
            if (!IsNearRiver(Mid2, 0.f))
                AddEdge(ArcChain[i], ArcChain[i + 1], EStreetTier::Secondary);
        }
    }

    // Ring 2 (outer) — Tertiary roads
    for (int32 G = 0; G < NumG; G++)
    {
        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        TArray<int32> ArcChain;
        ArcChain.Add(Ring2Nodes[G]);

        for (int32 S = 1; S <= ArcSubs; S++)
        {
            float T = (float)S / (ArcSubs + 1);
            float A = FMath::Lerp(Angle1, Angle2, T);
            FVector2D ArcPt(FMath::Cos(A) * Ring2R, FMath::Sin(A) * Ring2R);

            if (!IsNearRiver(ArcPt, 400.f))
                ArcChain.Add(AddNode(ArcPt));
        }
        ArcChain.Add(Ring2Nodes[Next]);

        for (int32 i = 0; i < ArcChain.Num() - 1; i++)
        {
            FVector2D PA2 = RoadNodes[ArcChain[i]].Pos;
            FVector2D PB2 = RoadNodes[ArcChain[i + 1]].Pos;
            FVector2D Mid2 = (PA2 + PB2) * 0.5f;
            if (!IsNearRiver(Mid2, 0.f))
                AddEdge(ArcChain[i], ArcChain[i + 1], EStreetTier::Tertiary);
        }
    }

    // ── 5. Mid-block connector streets ─────────────────────────────────────
    //    Between each pair of adjacent radials, at the midpoint angle,
    //    add a short connector road from ring1 to ring2.
    //    This creates smaller blocks for denser building placement.

    for (int32 G = 0; G < NumG; G++)
    {
        if (Rand.FRand() > MidBlockConnectorChance) continue;

        int32 Next = (G + 1) % NumG;
        float Angle1 = FMath::DegreesToRadians(Gates[G].Angle);
        float Angle2 = FMath::DegreesToRadians(Gates[Next].Angle);
        if (Angle2 <= Angle1) Angle2 += 2.f * PI;

        float MidAngle = (Angle1 + Angle2) * 0.5f;
        FVector2D Dir(FMath::Cos(MidAngle), FMath::Sin(MidAngle));

        FVector2D ConnR1 = Dir * Ring1R;
        FVector2D ConnR2 = Dir * Ring2R;

        // Skip if connector crosses river
        FVector2D ConnMid = (ConnR1 + ConnR2) * 0.5f;
        if (IsNearRiver(ConnMid, 300.f)) continue;

        int32 N1 = AddNode(ConnR1);
        int32 N2 = AddNode(ConnR2);
        AddEdge(N1, N2, EStreetTier::Tertiary);

        // Connect the inner node to nearest ring1 arc nodes
        float BestDist1a = 1e9f, BestDist1b = 1e9f;
        int32 BestNode1a = -1, BestNode1b = -1;
        float BestDist2a = 1e9f, BestDist2b = 1e9f;
        int32 BestNode2a = -1, BestNode2b = -1;

        for (int32 i = 0; i < RoadNodes.Num(); i++)
        {
            if (i == N1 || i == N2) continue;
            float NodeR = RoadNodes[i].Pos.Size();
            float D1 = (RoadNodes[i].Pos - ConnR1).Size();
            float D2 = (RoadNodes[i].Pos - ConnR2).Size();

            // Ring 1 neighbors (for N1)
            if (FMath::Abs(NodeR - Ring1R) < Ring1R * 0.15f)
            {
                if (D1 < BestDist1a) { BestDist1b = BestDist1a; BestNode1b = BestNode1a; BestDist1a = D1; BestNode1a = i; }
                else if (D1 < BestDist1b) { BestDist1b = D1; BestNode1b = i; }
            }
            // Ring 2 neighbors (for N2)
            if (FMath::Abs(NodeR - Ring2R) < Ring2R * 0.15f)
            {
                if (D2 < BestDist2a) { BestDist2b = BestDist2a; BestNode2b = BestNode2a; BestDist2a = D2; BestNode2a = i; }
                else if (D2 < BestDist2b) { BestDist2b = D2; BestNode2b = i; }
            }
        }

        // Connect to 2 nearest ring nodes on each ring (creates T-intersections)
        if (BestNode1a >= 0 && BestDist1a < Ring1R * 0.7f)
        {
            FVector2D M = (RoadNodes[BestNode1a].Pos + ConnR1) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode1a, N1, EStreetTier::Tertiary);
        }
        if (BestNode1b >= 0 && BestDist1b < Ring1R * 0.7f)
        {
            FVector2D M = (RoadNodes[BestNode1b].Pos + ConnR1) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode1b, N1, EStreetTier::Tertiary);
        }
        if (BestNode2a >= 0 && BestDist2a < Ring2R * 0.5f)
        {
            FVector2D M = (RoadNodes[BestNode2a].Pos + ConnR2) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode2a, N2, EStreetTier::Tertiary);
        }
        if (BestNode2b >= 0 && BestDist2b < Ring2R * 0.5f)
        {
            FVector2D M = (RoadNodes[BestNode2b].Pos + ConnR2) * 0.5f;
            if (!IsNearRiver(M, 0.f)) AddEdge(BestNode2b, N2, EStreetTier::Tertiary);
        }
    }
}

void AMedievalTownGenerator::ElevateRoadSplines()
{
    // For each generated edge, build a smoothed world-space spline
    // conforming to terrain height. Bridge edges use per-point detection:
    // only points actually over the river get elevated to bank height.
    for (FRoadEdge& E : RoadEdges)
    {
        if (!E.bIsGenerated) continue;
        E.WorldPoints.Empty();

        FVector2D PA = RoadNodes[E.NodeA].Pos;
        FVector2D PB = RoadNodes[E.NodeB].Pos;

        // Check if this edge crosses the river at all
        E.bIsBridge = bGenerateRiver && SegmentCrossesRiver(PA, PB);

        const int32 Subs = E.bIsBridge ? FMath::Max(RoadSplineSubdivisions * 2, 8) : FMath::Max(RoadSplineSubdivisions, 3);
        for (int32 S = 0; S <= Subs; S++)
        {
            float T = (float)S / Subs;
            FVector2D Pt = FMath::Lerp(PA, PB, T);

            // Add gentle organic waver (not at endpoints, not on bridges)
            if (S > 0 && S < Subs && RoadOrganicWaver > 0.f && !E.bIsBridge)
            {
                FVector2D Dir = (PB - PA).GetSafeNormal();
                FVector2D Perp(-Dir.Y, Dir.X);
                float Wave = Rand.FRandRange(-RoadOrganicWaver,
                                             RoadOrganicWaver) *
                             FMath::Sin(T * PI);
                Pt += Perp * Wave;
            }

            float H;
            if (E.bIsBridge)
            {
                // Per-point: check if THIS point is over the river
                float RDist = DistToRiverCenter(Pt);
                float HalfW = RiverWidth * 0.5f;
                float BankMargin = HalfW * 0.5f;  // Start ramp before river edge

                if (RDist < HalfW + BankMargin)
                {
                    // Over or near the river — use bank-level height + bridge raise
                    float BankH = GetTerrainHeightNoRiver(Pt.X, Pt.Y);
                    H = BankH + 25.f;
                }
                else
                {
                    // Away from river — normal terrain
                    H = GetTerrainHeight(Pt.X, Pt.Y) + 12.f;
                }
            }
            else
            {
                H = GetTerrainHeight(Pt.X, Pt.Y) + 12.f;
            }

            E.WorldPoints.Add(GetActorLocation() + FVector(Pt.X, Pt.Y, H));
        }
    }
}

float AMedievalTownGenerator::RoadWidth(EStreetTier Tier) const
{
    switch (Tier)
    {
    case EStreetTier::Primary:   return PrimaryRoadWidth;
    case EStreetTier::Secondary: return SecondaryRoadWidth;
    case EStreetTier::Tertiary:  return TertiaryRoadWidth;
    default:                     return SecondaryRoadWidth;
    }
}

void AMedievalTownGenerator::BuildRoadNetwork()
{
    BuildRadiocentricRoads();
    ElevateRoadSplines();

    UE_LOG(LogTemp, Log, TEXT("[MTG] BuildRoadNetwork: %d nodes, %d edges"),
           RoadNodes.Num(), RoadEdges.Num());
}

// ─────────────────────────────────────────────────────────────────────────────
//  §6  SHAPE GRAMMAR WALLS
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::ParseGrammar(const FString& Grammar, TArray<EWallModule>& Out)
{
    Out.Empty();
    TArray<FString> Tokens;
    Grammar.ParseIntoArrayWS(Tokens);
    for (const FString& Token : Tokens)
    {
        if (Token == TEXT("W")) Out.Add(EWallModule::WallSection);
        else if (Token == TEXT("T")) Out.Add(EWallModule::CornerTower);
        else if (Token == TEXT("G")) Out.Add(EWallModule::GateTower);
        else if (Token == TEXT("P")) Out.Add(EWallModule::Parapet);
        else if (Token == TEXT("B")) Out.Add(EWallModule::Buttress);
        else Out.Add(EWallModule::WallSection);  // default
    }
}

FVector AMedievalTownGenerator::WallPerimeterPoint(float AngleDeg) const
{
    float Rad = FMath::DegreesToRadians(AngleDeg);
    FVector2D P2D = FVector2D(FMath::Cos(Rad), FMath::Sin(Rad)) * TownRadius;
    float H = GetTerrainHeight(P2D.X, P2D.Y);
    return GetActorLocation() + FVector(P2D.X, P2D.Y, H);
}

void AMedievalTownGenerator::GenerateWalls()
{
    TArray<EWallModule> Modules;
    ParseGrammar(WallGrammarString, Modules);

    if (Modules.Num() == 0) return;

    const int32 TotalModules = Modules.Num();
    const float AngleStep = 360.f / TotalModules;

    WallPerimeter.Empty();
    GatePositions.Empty();

    for (int32 i = 0; i < TotalModules; i++)
    {
        float Angle     = i * AngleStep;
        float NextAngle = (i + 1) * AngleStep;

        FVector StartPt = WallPerimeterPoint(Angle);
        FVector EndPt   = WallPerimeterPoint(NextAngle);

        WallPerimeter.Add(StartPt);

        float MidAngle = (Angle + NextAngle) * 0.5f;
        FVector MidPt = WallPerimeterPoint(MidAngle);
        float TowerH = WallHeight * WallTowerHeightFactor;

        switch (Modules[i])
        {
        case EWallModule::WallSection:
            SpawnWallSection(StartPt, EndPt, WallHeight, WallThickness, true);
            break;

        case EWallModule::CornerTower:
            SpawnWallSection(StartPt, EndPt, WallHeight, WallThickness, false);
            SpawnCornerTower(MidPt, WallTowerRadius, TowerH);
            break;

        case EWallModule::GateTower:
        {
            // Split into two half-walls with gap in middle for gate
            FVector GapStart = FMath::Lerp(StartPt, EndPt, 0.35f);
            FVector GapEnd   = FMath::Lerp(StartPt, EndPt, 0.65f);
            SpawnWallSection(StartPt, GapStart, WallHeight, WallThickness, true);
            SpawnWallSection(GapEnd,  EndPt,    WallHeight, WallThickness, true);

            // Gate archway mesh (box with opening)
            FVector GateCenter = FMath::Lerp(GapStart, GapEnd, 0.5f);
            FVector GateDir = (GapEnd - GapStart).GetSafeNormal();
            SpawnGateTower(GateCenter, GateDir, WallTowerRadius * 1.4f, TowerH * 1.15f);

            GatePositions.Add(GateCenter);
            break;
        }

        case EWallModule::Parapet:
        {
            // Low parapet wall with frequent crenels
            SpawnWallSection(StartPt, EndPt, WallHeight * 0.55f, WallThickness * 0.7f, true);
            break;
        }

        case EWallModule::Buttress:
        {
            SpawnWallSection(StartPt, EndPt, WallHeight, WallThickness, true);
            // Add a buttress halfway
            FVector ButtressPos = FMath::Lerp(StartPt, EndPt, 0.5f);
            FVector WallDir = (EndPt - StartPt).GetSafeNormal();
            FVector InwardDir = FVector::CrossProduct(WallDir, FVector::UpVector).GetSafeNormal();
            ButtressPos += InwardDir * WallThickness * 2.f;

            UProceduralMeshComponent* BM = CreateMesh(TEXT("Buttress"));
            TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
            // Build in LOCAL space (centered at origin), then set world location
            AddBox(V, T, N, UV, FVector(0, 0, WallHeight * 0.45f),
                   WallThickness * 2.f, WallThickness * 2.f, WallHeight * 0.9f);
            SetMeshSection(BM, 0, V, T, N, UV, StoneMaterial);
            BM->SetWorldLocation(ButtressPos);
            break;
        }
        }
    }
}

void AMedievalTownGenerator::SpawnWallSection(FVector Start, FVector End,
                                               float Height, float Thickness,
                                               bool bBattlements)
{
    FVector Dir = (End - Start);
    float Length = Dir.Size();
    if (Length < 1.f) return;
    Dir /= Length;

    // Sample terrain along segment to find LOWEST point — place wall base there
    // so the wall never floats. Use the lowest Z to ensure wall covers all terrain.
    float MinZ = FMath::Min(Start.Z, End.Z);
    const int32 TerrainSamples = FMath::Max(4, (int32)(Length / 400.f));
    for (int32 S = 1; S < TerrainSamples; S++)
    {
        float T = (float)S / TerrainSamples;
        FVector Pt = FMath::Lerp(Start, End, T);
        FVector2D Local2D(Pt.X - GetActorLocation().X, Pt.Y - GetActorLocation().Y);
        float SampleH = GetTerrainHeight(Local2D.X, Local2D.Y) + GetActorLocation().Z;
        MinZ = FMath::Min(MinZ, SampleH);
    }

    // Mid XY, but use the lowest terrain Z minus a small embed amount
    FVector Mid = (Start + End) * 0.5f;
    Mid.Z = MinZ - 50.f;  // Embed 50 units below lowest point so wall never floats

    // Wall section rotation: X-axis aligns with wall direction
    FRotator WallRot = Dir.Rotation();

    UProceduralMeshComponent* WallMesh = CreateMesh(TEXT("WallSeg"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Build in LOCAL space: centered at origin, extending along X axis
    // Wall bottom is at -Height*0.5, top at +Height*0.5 (centered AddBox)
    // Add extra height to cover terrain variation
    float MaxZ = FMath::Max(Start.Z, End.Z);
    float TerrainVariation = MaxZ - MinZ;
    float TotalHeight = Height + TerrainVariation + 100.f;  // Extra to ensure coverage

    FVector LocalCenter(0.f, 0.f, TotalHeight * 0.5f);
    AddBox(V, T, N, UV, LocalCenter, Length, Thickness, TotalHeight);

    if (bBattlements)
    {
        // Crenellations in local space along X axis — on top of extended wall
        const float BW = BattlementWidth;
        const float BH = BattlementHeight;
        const float Gap = BW;
        float LX = -Length * 0.5f + BW * 0.5f;

        while (LX + BW * 0.5f < Length * 0.5f)
        {
            FVector BattLocal(LX, 0.f, TotalHeight + BH * 0.5f);
            AddBox(V, T, N, UV, BattLocal, BW, Thickness * 0.9f, BH);
            LX += BW + Gap;
        }
    }

    SetMeshSection(WallMesh, 0, V, T, N, UV, StoneMaterial);

    // Position at midpoint with correct world rotation
    WallMesh->SetWorldLocation(Mid);
    WallMesh->SetWorldRotation(WallRot);
}

void AMedievalTownGenerator::SpawnCornerTower(FVector Center, float Radius, float Height)
{
    UProceduralMeshComponent* TM = CreateMesh(TEXT("CTower"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Build in LOCAL space (origin = tower base center)
    // Extend below ground to embed into terrain and avoid gaps
    float EmbedDepth = 80.f;
    FVector Local = FVector(0, 0, -EmbedDepth);
    float TotalH = Height + EmbedDepth;
    AddCylinder(V, T, N, UV, Local, Radius, TotalH, 12, true);
    // Conical cap at top of cylinder
    AddCone(V, T, N, UV, Local + FVector(0, 0, TotalH), Radius * 1.1f, Radius * 0.8f, 12);
    // Battlements around top rim
    const int32 BCnt = 8;
    for (int32 i = 0; i < BCnt; i++)
    {
        float Ang = (float)i / BCnt * TWO_PI;
        FVector BP = Local + FVector(FMath::Cos(Ang), FMath::Sin(Ang), 0.f) *
                     (Radius - BattlementWidth * 0.5f) +
                     FVector(0, 0, TotalH + BattlementHeight * 0.5f);
        AddBox(V, T, N, UV, BP, BattlementWidth, BattlementWidth, BattlementHeight);
    }
    SetMeshSection(TM, 0, V, T, N, UV, StoneMaterial);
    TM->SetWorldLocation(Center);
}

void AMedievalTownGenerator::SpawnGateTower(FVector Center, FVector Direction,
                                             float TowerRadius, float Height)
{
    // Direction = wall tangent (along the wall).
    // Twin towers flank the gate opening = offset along the wall tangent (Direction).
    // The passage goes through the wall = perpendicular to Direction.
    float TwinOffset = TowerRadius * 1.8f;

    for (int32 Side = -1; Side <= 1; Side += 2)
    {
        FVector TowerCenter = Center + Direction * TwinOffset * Side;
        UProceduralMeshComponent* TM = CreateMesh(TEXT("GateTower"));
        TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
        // Build in local space, embed below ground
        float EmbedDepth = 80.f;
        FVector Local(0, 0, -EmbedDepth);
        float TotalH = Height + EmbedDepth;
        AddCylinder(V, T, N, UV, Local, TowerRadius, TotalH, 10, true);
        AddCone(V, T, N, UV, Local + FVector(0, 0, TotalH),
                TowerRadius * 1.1f, TowerRadius * 0.75f, 10);
        SetMeshSection(TM, 0, V, T, N, UV, StoneMaterial);
        TM->SetWorldLocation(TowerCenter);
    }

    // Gate arch connector — build in local space, set world location at Center
    // After rotation by Direction.Rotation(): local X = wall tangent, local Y = through-wall
    // Pillars flank the opening along X (wall tangent), lintel spans along X
    UProceduralMeshComponent* ArchMesh = CreateMesh(TEXT("GateArch"));
    TArray<FVector> V2; TArray<int32> T2; TArray<FVector> N2; TArray<FVector2D> UV2;
    float GateW = TowerRadius * 2.f;
    float PillarH = WallHeight * 0.4f;
    // Two side pillars (flanking the opening along local X = wall tangent)
    AddBox(V2, T2, N2, UV2, FVector(GateW + WallThickness*0.75f, 0, PillarH * 0.5f),
           WallThickness * 1.5f, WallThickness * 1.5f, PillarH);
    AddBox(V2, T2, N2, UV2, FVector(-(GateW + WallThickness*0.75f), 0, PillarH * 0.5f),
           WallThickness * 1.5f, WallThickness * 1.5f, PillarH);
    // Lintel connecting the two pillars (spans along local X)
    AddBox(V2, T2, N2, UV2, FVector(0, 0, PillarH + WallThickness * 0.5f),
           GateW * 2.f + WallThickness * 3.f, WallThickness, WallThickness);
    SetMeshSection(ArchMesh, 0, V2, T2, N2, UV2, StoneMaterial);
    ArchMesh->SetWorldLocation(Center);
    ArchMesh->SetWorldRotation(Direction.Rotation());
}

// ─────────────────────────────────────────────────────────────────────────────
//  §7  BUILDING PLACEMENT  (district-aware + flat-area filtered)
// ─────────────────────────────────────────────────────────────────────────────

TArray<FDistrictDef> AMedievalTownGenerator::BuildDistrictDefs() const
{
    TArray<FDistrictDef> Defs;

    // ─────────────────────────────────────────────────────────────────────
    //  Compute feature angles for angular sector assignment
    //
    //  Medieval towns weren't organized in perfect concentric rings — they
    //  had angular sectors driven by geography:
    //    • Craft/industrial quarters along the river
    //    • Gate wards at each entrance (bustling commercial zones)
    //    • Administrative/military core at center
    //    • Slums on the least desirable side (downwind, away from gates)
    //    • Merchant quarters filling the best remaining sectors
    // ─────────────────────────────────────────────────────────────────────

    // River direction angle (river flows roughly along this bearing)
    float RiverAngle = 0.f;
    if (River.Waypoints.Num() >= 2)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        RiverAngle = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X));
    }

    // Gate angles
    TArray<float> GateAngles;
    for (const FRoadNode& N : RoadNodes)
    {
        if (N.bIsGate)
        {
            GateAngles.Add(FMath::RadiansToDegrees(FMath::Atan2(N.Pos.Y, N.Pos.X)));
        }
    }

    // Slums angle: opposite side from the densest gate cluster
    // (historically the least desirable, downwind side)
    float SlumsAngle = RiverAngle + 180.f;
    if (SlumsAngle > 180.f) SlumsAngle -= 360.f;

    // ── 1. Inner Ward (castle/admin core) ────────────────────────────────
    //    Large buildings, wide spacing, formal layout (low jitter).
    //    Military/administrative center with keep, church, guildhall.
    {
        FDistrictDef D;
        D.Name = TEXT("Inner Ward");
        D.Type = EDistrictType::InnerWard;
        D.InnerRadiusFraction = 0.f;
        D.OuterRadiusFraction = InnerRingRadius;
        D.Density = 0.52f;
        D.MinScale = 1.2f; D.MaxScale = 1.7f;
        D.StylePool = { EBuildingStyle::Keep, EBuildingStyle::Keep,
                        EBuildingStyle::GuildHall, EBuildingStyle::Church,
                        EBuildingStyle::TavernInn };
        D.SpacingMult = 1.4f;       // Wide spacing — prestigious buildings
        D.SetbackMult = 1.3f;       // Larger setback from road
        D.RotationJitter = 3.f;     // Very formal, aligned
        D.LShapeChance = 0.4f;      // Many L-shaped compounds
        D.CursorStartOffset = 300.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 2. Craft Quarter (river-side industrial) ─────────────────────────
    //    Warehouses, blacksmiths, yards near the river.
    //    Medium density, bigger buildings, oriented toward water.
    //    Angular wedge centered on river direction ±45°.
    {
        FDistrictDef D;
        D.Name = TEXT("Craft Quarter");
        D.Type = EDistrictType::CraftQuarter;
        D.InnerRadiusFraction = InnerRingRadius;
        D.OuterRadiusFraction = 0.92f;
        D.Density = 0.55f;
        D.MinScale = 0.9f; D.MaxScale = 1.4f;
        D.StylePool = { EBuildingStyle::Warehouse, EBuildingStyle::Warehouse,
                        EBuildingStyle::Blacksmith, EBuildingStyle::Blacksmith,
                        EBuildingStyle::Stable, EBuildingStyle::TownHouse };
        D.SpacingMult = 1.1f;       // Moderate spacing — work yards between buildings
        D.SetbackMult = 1.5f;       // Bigger setback — loading areas along road
        D.RotationJitter = 6.f;     // Somewhat organized
        D.LShapeChance = 0.35f;     // Workshops often have wings
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 400.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = RiverAngle - 45.f;
        D.MaxAngleDeg = RiverAngle + 45.f;
        Defs.Add(D);
    }

    // ── 3. Gate Wards (bustling commercial at each gate) ─────────────────
    //    Dense shops, taverns, cheap lodging near the town gates.
    //    Tight spacing, small setback, high density.
    //    Each gate gets a ±20° angular wedge in the outer ring area.
    for (int32 G = 0; G < GateAngles.Num(); G++)
    {
        FDistrictDef D;
        D.Name = FString::Printf(TEXT("Gate Ward %d"), G + 1);
        D.Type = EDistrictType::GateWard;
        D.InnerRadiusFraction = OuterRingRadius * 0.85f;
        D.OuterRadiusFraction = 0.94f;
        D.Density = 0.82f;
        D.MinScale = 0.7f; D.MaxScale = 0.95f;
        D.StylePool = { EBuildingStyle::TownHouse, EBuildingStyle::TownHouse,
                        EBuildingStyle::TavernInn, EBuildingStyle::TavernInn,
                        EBuildingStyle::Bakery, EBuildingStyle::SmallCottage,
                        EBuildingStyle::TownHouse };
        D.SpacingMult = 0.55f;      // Very tight — cramped commercial zone
        D.SetbackMult = 0.6f;       // Buildings crowd the road
        D.RotationJitter = 5.f;     // Somewhat organized along streets
        D.LShapeChance = 0.1f;      // Small plots, few wings
        D.CursorStartOffset = 150.f;
        D.MinEdgeLen = 350.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = GateAngles[G] - 20.f;
        D.MaxAngleDeg = GateAngles[G] + 20.f;
        Defs.Add(D);
    }

    // ── 4. Merchant Quarter (main commercial/residential belt) ───────────
    //    Townhouses, shops, taverns between the rings.
    //    Medium-high density, good spacing, neat orientation.
    {
        FDistrictDef D;
        D.Name = TEXT("Merchant Quarter");
        D.Type = EDistrictType::MerchantQuarter;
        D.InnerRadiusFraction = InnerRingRadius;
        D.OuterRadiusFraction = OuterRingRadius;
        D.Density = 0.75f;
        D.MinScale = 0.85f; D.MaxScale = 1.2f;
        D.StylePool = { EBuildingStyle::TownHouse, EBuildingStyle::TownHouse,
                        EBuildingStyle::TownHouse, EBuildingStyle::TavernInn,
                        EBuildingStyle::GuildHall, EBuildingStyle::Blacksmith,
                        EBuildingStyle::Bakery };
        D.SpacingMult = 0.8f;       // Fairly dense
        D.SetbackMult = 0.85f;      // Close to road — shopfronts
        D.RotationJitter = 5.f;     // Neat commercial streets
        D.LShapeChance = 0.2f;
        D.CursorStartOffset = 200.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 5. Outer Residential (between outer ring and walls) ──────────────
    //    Modest homes, some craftsmen.
    {
        FDistrictDef D;
        D.Name = TEXT("Outer Residential");
        D.Type = EDistrictType::OuterResidential;
        D.InnerRadiusFraction = OuterRingRadius;
        D.OuterRadiusFraction = 0.88f;
        D.Density = 0.65f;
        D.MinScale = 0.75f; D.MaxScale = 1.0f;
        D.StylePool = { EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::TownHouse,
                        EBuildingStyle::Blacksmith };
        D.SpacingMult = 0.9f;
        D.SetbackMult = 1.0f;
        D.RotationJitter = 10.f;    // More organic than center
        D.LShapeChance = 0.15f;
        D.CursorStartOffset = 220.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    // ── 6. Slums (poor quarter — opposite river, least desirable) ────────
    //    Tiny, densely packed cottages with chaotic orientation.
    //    Angular wedge opposite the river ±50°.
    {
        FDistrictDef D;
        D.Name = TEXT("Slums");
        D.Type = EDistrictType::Slums;
        D.InnerRadiusFraction = OuterRingRadius * 0.7f;
        D.OuterRadiusFraction = 0.92f;
        D.Density = 0.88f;
        D.MinScale = 0.55f; D.MaxScale = 0.8f;
        D.StylePool = { EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::SmallCottage,
                        EBuildingStyle::SmallCottage, EBuildingStyle::Stable };
        D.SpacingMult = 0.4f;       // Extremely tight — shanty-town density
        D.SetbackMult = 0.4f;       // Buildings crowd right up to paths
        D.RotationJitter = 25.f;    // Chaotic, unplanned feel
        D.LShapeChance = 0.05f;     // Too poor for extensions
        D.CursorStartOffset = 120.f;
        D.MinEdgeLen = 250.f;
        D.bUsesAngle = true;
        D.MinAngleDeg = SlumsAngle - 50.f;
        D.MaxAngleDeg = SlumsAngle + 50.f;
        Defs.Add(D);
    }

    // ── 7. Transition Zone (stables/storage near walls) ──────────────────
    //    Wide spacing, low density, functional buildings near the walls.
    {
        FDistrictDef D;
        D.Name = TEXT("Transition Zone");
        D.Type = EDistrictType::TransitionZone;
        D.InnerRadiusFraction = 0.88f;
        D.OuterRadiusFraction = 0.94f;
        D.Density = 0.35f;
        D.MinScale = 0.7f; D.MaxScale = 1.0f;
        D.StylePool = { EBuildingStyle::Stable, EBuildingStyle::Stable,
                        EBuildingStyle::Warehouse, EBuildingStyle::SmallCottage };
        D.SpacingMult = 1.3f;       // Wide spacing — yards, pens
        D.SetbackMult = 1.2f;
        D.RotationJitter = 12.f;
        D.LShapeChance = 0.1f;
        D.CursorStartOffset = 250.f;
        D.MinEdgeLen = 500.f;
        Defs.Add(D);
    }

    return Defs;
}

// ── Angular-aware district lookup ───────────────────────────────────────
//
//  Checks position against all district defs, prioritizing angular wedge
//  districts (CraftQuarter, GateWard, Slums) which override the radius-only
//  fallback districts (MerchantQuarter, OuterResidential, etc).
//
//  Priority: Angular wedge match > Radius-only match > Fallback
//

static float NormalizeAngle180(float Deg)
{
    while (Deg > 180.f)  Deg -= 360.f;
    while (Deg < -180.f) Deg += 360.f;
    return Deg;
}

static bool AngleInSector(float TestDeg, float MinDeg, float MaxDeg)
{
    // Normalize all angles to -180..180
    TestDeg = NormalizeAngle180(TestDeg);
    MinDeg  = NormalizeAngle180(MinDeg);
    MaxDeg  = NormalizeAngle180(MaxDeg);

    if (MinDeg <= MaxDeg)
        return TestDeg >= MinDeg && TestDeg <= MaxDeg;
    else
        // Wraps around -180/180 boundary
        return TestDeg >= MinDeg || TestDeg <= MaxDeg;
}

EDistrictType AMedievalTownGenerator::GetDistrictAt(FVector2D Pos) const
{
    float Frac = Pos.Size() / TownRadius;
    float Angle = FMath::RadiansToDegrees(FMath::Atan2(Pos.Y, Pos.X));

    // ── Center always = InnerWard ──
    if (Frac < InnerRingRadius * 0.7f)
        return EDistrictType::InnerWard;

    // ── Near walls = TransitionZone ──
    if (Frac > 0.88f)
        return EDistrictType::TransitionZone;

    // ── Check angular wedge districts (highest priority in the mid-ring) ──
    //    These are cached in CachedDistrictDefs during PlaceBuildings.
    //    For runtime queries, re-derive from features.

    // Craft Quarter: near river
    if (River.Waypoints.Num() >= 2 && Frac > InnerRingRadius)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        float RiverAng = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X));
        if (AngleInSector(Angle, RiverAng - 45.f, RiverAng + 45.f))
            return EDistrictType::CraftQuarter;
    }

    // Gate Wards: near gates in outer ring
    if (Frac > OuterRingRadius * 0.85f)
    {
        for (const FRoadNode& N : RoadNodes)
        {
            if (!N.bIsGate) continue;
            float GA = FMath::RadiansToDegrees(FMath::Atan2(N.Pos.Y, N.Pos.X));
            if (AngleInSector(Angle, GA - 20.f, GA + 20.f))
                return EDistrictType::GateWard;
        }
    }

    // Slums: opposite river direction
    if (Frac > OuterRingRadius * 0.7f && River.Waypoints.Num() >= 2)
    {
        FVector2D RD = River.Waypoints.Last() - River.Waypoints[0];
        float SlumsAng = FMath::RadiansToDegrees(FMath::Atan2(RD.Y, RD.X)) + 180.f;
        if (AngleInSector(Angle, SlumsAng - 50.f, SlumsAng + 50.f))
            return EDistrictType::Slums;
    }

    // ── Radius-only fallback ──
    if (Frac < OuterRingRadius)
        return EDistrictType::MerchantQuarter;

    return EDistrictType::OuterResidential;
}

EBuildingStyle AMedievalTownGenerator::PickStyle(EDistrictType District,
                                                  const FDistrictDef& Def) const
{
    if (Def.StylePool.Num() == 0) return EBuildingStyle::TownHouse;
    return Def.StylePool[Rand.RandRange(0, Def.StylePool.Num() - 1)];
}

ERoofType AMedievalTownGenerator::PickRoof(EBuildingStyle Style) const
{
    switch (Style)
    {
    case EBuildingStyle::SmallCottage: return ERoofType::Thatched;
    case EBuildingStyle::TownHouse:
        return (Rand.FRand() < 0.6f) ? ERoofType::Pitched : ERoofType::Gambrel;
    case EBuildingStyle::GuildHall:
        return (Rand.FRand() < 0.5f) ? ERoofType::Hipped : ERoofType::FlatParapet;
    case EBuildingStyle::TavernInn:
        return (Rand.FRand() < 0.7f) ? ERoofType::Gambrel : ERoofType::Pitched;
    case EBuildingStyle::Church:
        return ERoofType::Pitched;
    case EBuildingStyle::Keep:
        return ERoofType::FlatParapet;
    case EBuildingStyle::Stable:
        return (Rand.FRand() < 0.5f) ? ERoofType::Gambrel : ERoofType::Thatched;
    case EBuildingStyle::Warehouse:
        return ERoofType::FlatParapet;
    default:
        return ERoofType::Pitched;
    }
}

int32 AMedievalTownGenerator::PickFloorCount(EBuildingStyle Style, EDistrictType District) const
{
    // District modifies base floor counts:
    //   InnerWard:     Tall, prestigious buildings (keeps 3-4, guilds 2-3)
    //   MerchantQtr:   2-story townhouses, tall guildhalls
    //   CraftQuarter:  Single-story workshops, 2-story warehouses
    //   GateWard:      Cramped 2-3 story buildings (vertical, narrow plots)
    //   Slums:         Always 1 story (too poor for multi-story)
    //   Outer/Trans:   Mostly 1 story

    switch (Style)
    {
    case EBuildingStyle::SmallCottage:
        return 1;

    case EBuildingStyle::TownHouse:
        if (District == EDistrictType::GateWard)       return Rand.RandRange(2, 3);
        if (District == EDistrictType::MerchantQuarter) return 2;
        if (District == EDistrictType::Slums)           return 1;
        if (District == EDistrictType::InnerWard)       return 2;
        return (Rand.FRand() < 0.3f) ? 2 : 1;

    case EBuildingStyle::GuildHall:
        if (District == EDistrictType::InnerWard) return Rand.RandRange(2, 3);
        return 2;

    case EBuildingStyle::TavernInn:
        if (District == EDistrictType::GateWard) return Rand.RandRange(2, 3);
        return 2;

    case EBuildingStyle::Church:       return 1;

    case EBuildingStyle::Keep:
        return Rand.RandRange(3, 4);

    case EBuildingStyle::Stable:       return 1;

    case EBuildingStyle::Warehouse:
        if (District == EDistrictType::CraftQuarter) return Rand.RandRange(1, 2);
        return 1;

    case EBuildingStyle::Blacksmith:
        if (District == EDistrictType::CraftQuarter) return 1;
        return 1;

    case EBuildingStyle::Bakery:       return 1;
    default:                           return 1;
    }
}

FVector2D AMedievalTownGenerator::BuildingSize(EBuildingStyle Style) const
{
    switch (Style)
    {
    case EBuildingStyle::SmallCottage: return SmallCottageSize;
    case EBuildingStyle::TownHouse:    return TownHouseSize;
    case EBuildingStyle::GuildHall:    return GuildHallSize;
    case EBuildingStyle::TavernInn:    return TavernSize;
    case EBuildingStyle::Church:       return ChurchSize;
    case EBuildingStyle::Keep:         return KeepSize;
    case EBuildingStyle::Stable:       return StableSize;
    case EBuildingStyle::Warehouse:    return WarehouseSize;
    case EBuildingStyle::Blacksmith:   return SmallCottageSize * 1.1f;
    case EBuildingStyle::Bakery:       return SmallCottageSize;
    default:                           return TownHouseSize;
    }
}

bool AMedievalTownGenerator::CanPlaceLot(FVector Center, float Radius, int32 IgnoreEdgeIndex, float SpacingOverride) const
{
    FVector2D Pos2D(Center.X, Center.Y);

    // Must be inside town walls (with margin)
    if (Pos2D.Size() > TownRadius * 0.89f - Radius) return false;

    // Must not be near river
    if (IsNearRiver(Pos2D, Radius)) return false;

    // Must be on flat terrain
    FTerrainSample TS = SampleTerrain(Pos2D.X, Pos2D.Y);
    if (!TS.bIsFlat) return false;

    // Four-corner flat check (flat area detector)
    if (!IsTerrainFlat(Pos2D, Radius * 0.7f, Radius * 0.7f)) return false;

    // Must not overlap other buildings (using per-district spacing if provided)
    float Spacing = (SpacingOverride >= 0.f) ? SpacingOverride : MinBuildingSpacing;
    for (const FBuildingLot& Lot : PlacedLots)
    {
        if (!Lot.bIsPlaced) continue;
        float Dist = Dist2D(Center, Lot.Center);
        if (Dist < Radius + Lot.CollisionRadius + Spacing)
            return false;
    }

    // Must not be on a road (skip the road we're lining via IgnoreEdgeIndex)
    for (int32 i = 0; i < RoadEdges.Num(); i++)
    {
        if (i == IgnoreEdgeIndex) continue;
        const FRoadEdge& Edge = RoadEdges[i];
        if (!Edge.bIsGenerated) continue;
        FVector2D A = RoadNodes[Edge.NodeA].Pos;
        FVector2D B = RoadNodes[Edge.NodeB].Pos;
        float RoadExclusion = Edge.Width * 0.5f + Radius + 100.f;
        if (CircleOverlapsSegment(Pos2D, RoadExclusion, A, B)) return false;
    }

    return true;
}

void AMedievalTownGenerator::PlaceBuildings()
{
    TArray<FDistrictDef> Defs = BuildDistrictDefs();
    int32 Placed = 0;

    // Helper: find the best matching FDistrictDef for a position
    auto FindDef = [&](FVector2D Pos2D) -> const FDistrictDef*
    {
        EDistrictType DType = GetDistrictAt(Pos2D);
        float Frac = Pos2D.Size() / TownRadius;
        float Angle = FMath::RadiansToDegrees(FMath::Atan2(Pos2D.Y, Pos2D.X));

        // First try angular wedge defs that match position precisely
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type != DType) continue;
            if (D.bUsesAngle)
            {
                if (Frac >= D.InnerRadiusFraction && Frac <= D.OuterRadiusFraction &&
                    AngleInSector(Angle, D.MinAngleDeg, D.MaxAngleDeg))
                    return &D;
            }
        }
        // Then try radius-only defs
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type == DType && !D.bUsesAngle) return &D;
        }
        // Fallback to any matching type
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type == DType) return &D;
        }
        return nullptr;
    };

    // ═══════════════════════════════════════════════════════════════════════════
    //  Phase A: Line buildings along roads
    //
    //  Walk each road edge, placing buildings on both sides at intervals
    //  dictated by the local district's rules. Dense districts (GateWard, Slums)
    //  pack buildings tighter; formal districts (InnerWard) space them out.
    // ═══════════════════════════════════════════════════════════════════════════

    for (int32 EdgeIdx = 0; EdgeIdx < RoadEdges.Num(); EdgeIdx++)
    {
        const FRoadEdge& Edge = RoadEdges[EdgeIdx];
        if (!Edge.bIsGenerated || Edge.WorldPoints.Num() < 2) continue;

        // Compute cumulative distance along road for interpolation
        float TotalLen = 0.f;
        TArray<float> CumDist;
        CumDist.Add(0.f);
        for (int32 i = 0; i < Edge.WorldPoints.Num() - 1; i++)
        {
            TotalLen += (Edge.WorldPoints[i + 1] - Edge.WorldPoints[i]).Size();
            CumDist.Add(TotalLen);
        }

        // Sample district at road midpoint to get edge-level defaults
        FVector ActorLoc = GetActorLocation();
        FVector MidPt = Edge.WorldPoints[Edge.WorldPoints.Num() / 2];
        FVector2D MidPt2D(MidPt.X - ActorLoc.X, MidPt.Y - ActorLoc.Y);
        const FDistrictDef* EdgeDef = FindDef(MidPt2D);
        if (!EdgeDef) continue;

        // Per-district edge length threshold
        if (TotalLen < EdgeDef->MinEdgeLen) continue;

        // Place buildings on each side of the road independently
        for (int32 Side = 0; Side < 2; Side++)
        {
            float SideSign = (Side == 0) ? 1.f : -1.f;
            float Cursor = EdgeDef->CursorStartOffset;

            while (Cursor < TotalLen - EdgeDef->CursorStartOffset && Placed < TargetBuildingCount)
            {
                // ── Find point at Cursor distance along road ────────────
                FVector RoadPt = FVector::ZeroVector;
                FVector Tangent = FVector::ForwardVector;
                bool bFound = false;
                for (int32 i = 0; i < CumDist.Num() - 1; i++)
                {
                    if (CumDist[i + 1] >= Cursor)
                    {
                        float SegLen = CumDist[i + 1] - CumDist[i];
                        float LocalT = (SegLen > 1.f) ? (Cursor - CumDist[i]) / SegLen : 0.f;
                        RoadPt = FMath::Lerp(Edge.WorldPoints[i], Edge.WorldPoints[i + 1], LocalT);
                        Tangent = (Edge.WorldPoints[i + 1] - Edge.WorldPoints[i]).GetSafeNormal();
                        bFound = true;
                        break;
                    }
                }
                if (!bFound) break;

                // 2D tangent and perpendicular
                FVector2D Tan2D(Tangent.X, Tangent.Y);
                if (Tan2D.SizeSquared() < 0.01f) { Cursor += 100.f; continue; }
                Tan2D.Normalize();
                FVector2D Perp2D(-Tan2D.Y, Tan2D.X);

                // Road point in local (actor-relative) 2D
                FVector2D RoadPt2D(RoadPt.X - ActorLoc.X, RoadPt.Y - ActorLoc.Y);

                // ── Determine district at this exact road position ───────
                const FDistrictDef* LocalDef = FindDef(RoadPt2D);
                if (!LocalDef) { Cursor += 200.f; continue; }

                // ── Pick building style + footprint using district rules ─
                EBuildingStyle Style = PickStyle(LocalDef->Type, *LocalDef);
                FVector2D BaseSize = BuildingSize(Style);
                float Scale = Rand.FRandRange(LocalDef->MinScale, LocalDef->MaxScale);
                FVector2D Footprint = BaseSize * Scale;
                float Frontage = Footprint.X;
                float Depth    = Footprint.Y;

                // ── Compute candidate center (per-district setback) ──────
                float DistrictSetback = RoadBuildingSetback * LocalDef->SetbackMult;
                float Offset = Edge.Width * 0.5f + DistrictSetback + Depth * 0.5f;
                FVector2D CandPos2D = RoadPt2D + Perp2D * SideSign * Offset;

                // ── Terrain height (sample 5 points) ─────────────────────
                float HW = Footprint.X * 0.5f, HD = Footprint.Y * 0.5f;
                float H = GetTerrainHeight(CandPos2D.X, CandPos2D.Y);
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y + HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y + HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y - HD));
                H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y - HD));

                FVector CandPos(CandPos2D.X, CandPos2D.Y, H);
                float CollRadius = FMath::Max(Footprint.X, Footprint.Y) * 0.6f;

                // ── Try to place (skip overlap with this road edge) ──────
                float DistrictSpacing = MinBuildingSpacing * LocalDef->SpacingMult;
                if (CanPlaceLot(CandPos, CollRadius, EdgeIdx, DistrictSpacing))
                {
                    FBuildingLot Lot;
                    Lot.Center = ActorLoc + CandPos;
                    Lot.Footprint = Footprint;
                    Lot.District = LocalDef->Type;
                    Lot.Style = Style;
                    Lot.NumFloors = PickFloorCount(Style, LocalDef->Type);
                    Lot.Roof = PickRoof(Style);
                    Lot.CollisionRadius = CollRadius;
                    Lot.bIsPlaced = true;

                    // Yaw: building front faces the road + district jitter
                    FVector2D FacingDir = -Perp2D * SideSign;
                    Lot.Yaw = FMath::RadiansToDegrees(FMath::Atan2(FacingDir.Y, FacingDir.X));
                    Lot.Yaw += Rand.FRandRange(-LocalDef->RotationJitter,
                                                LocalDef->RotationJitter);

                    // L-shape wing per district probability
                    if (Rand.FRand() < LocalDef->LShapeChance &&
                        (Style == EBuildingStyle::GuildHall || Style == EBuildingStyle::Keep ||
                         Style == EBuildingStyle::TavernInn || Style == EBuildingStyle::Warehouse))
                    {
                        Lot.bHasWing = true;
                        Lot.WingFootprint = FVector2D(Footprint.X * 0.4f, Footprint.Y * 0.6f);
                        Lot.WingYawOffset = 90.f;
                    }

                    PlacedLots.Add(Lot);
                    Placed++;

                    // Advance cursor: per-district spacing
                    Cursor += Frontage + DistrictSpacing * 0.5f;
                }
                else
                {
                    Cursor += 100.f;
                }
            }
        }
    }

    int32 RoadLinedCount = Placed;

    // ═══════════════════════════════════════════════════════════════════════════
    //  Phase B: Fill remaining space with scatter buildings
    //
    //  POSITION-FIRST approach: generate a random position inside the town,
    //  determine which district it belongs to, then apply that district's rules.
    //  This ensures uniform coverage across all areas instead of clustering
    //  in angular-sector districts.
    // ═══════════════════════════════════════════════════════════════════════════

    int32 TryLimit = (TargetBuildingCount - Placed) * 40;

    for (int32 Try = 0; Try < TryLimit && Placed < TargetBuildingCount; Try++)
    {
        // ── Generate random position inside town walls ──────────────────
        float MaxR = TownRadius * 0.88f;
        FVector2D CandPos2D = RandAnnulus(0.f, MaxR);

        // Skip plaza area — no scatter buildings in the market plaza
        if (CandPos2D.Size() < TownRadius * 0.08f) continue;

        // ── Find matching district def ──────────────────────────────────
        EDistrictType DType = GetDistrictAt(CandPos2D);
        const FDistrictDef* MatchDef = nullptr;

        float Frac = CandPos2D.Size() / TownRadius;
        float Angle = FMath::RadiansToDegrees(FMath::Atan2(CandPos2D.Y, CandPos2D.X));

        // Prefer angular wedge defs that match precisely
        for (const FDistrictDef& D : Defs)
        {
            if (D.Type != DType) continue;
            if (D.bUsesAngle)
            {
                if (Frac >= D.InnerRadiusFraction && Frac <= D.OuterRadiusFraction &&
                    AngleInSector(Angle, D.MinAngleDeg, D.MaxAngleDeg))
                { MatchDef = &D; break; }
            }
        }
        if (!MatchDef)
        {
            for (const FDistrictDef& D : Defs)
                if (D.Type == DType && !D.bUsesAngle) { MatchDef = &D; break; }
        }
        if (!MatchDef) continue;

        // Density roll
        if (Rand.FRand() > MatchDef->Density) continue;

        EBuildingStyle Style = PickStyle(MatchDef->Type, *MatchDef);
        FVector2D BaseSize = BuildingSize(Style);
        float Scale = Rand.FRandRange(MatchDef->MinScale, MatchDef->MaxScale);
        FVector2D Footprint = BaseSize * Scale;
        float CollRadius = FMath::Max(Footprint.X, Footprint.Y) * 0.6f;

        float HW = Footprint.X * 0.5f, HD = Footprint.Y * 0.5f;
        float H = GetTerrainHeight(CandPos2D.X, CandPos2D.Y);
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y + HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X + HW, CandPos2D.Y - HD));
        H = FMath::Max(H, GetTerrainHeight(CandPos2D.X - HW, CandPos2D.Y - HD));

        FVector CandPos(CandPos2D.X, CandPos2D.Y, H);
        float DistSpacing = MinBuildingSpacing * MatchDef->SpacingMult;
        if (!CanPlaceLot(CandPos, CollRadius, -1, DistSpacing)) continue;

        FBuildingLot Lot;
        Lot.Center = GetActorLocation() + CandPos;
        Lot.Footprint = Footprint;
        Lot.District = MatchDef->Type;
        Lot.Style = Style;
        Lot.NumFloors = PickFloorCount(Style, MatchDef->Type);
        Lot.Roof = PickRoof(Style);
        Lot.CollisionRadius = CollRadius;
        Lot.bIsPlaced = true;

        // Orient scatter buildings toward nearest road for coherent block layout
        float BestRoadDist = 1e9f;
        FVector2D BestRoadDir = FVector2D(1.f, 0.f);
        for (const FRoadEdge& RE : RoadEdges)
        {
            if (!RE.bIsGenerated) continue;
            FVector2D A = RoadNodes[RE.NodeA].Pos;
            FVector2D B = RoadNodes[RE.NodeB].Pos;
            FVector2D AB = B - A, AP = CandPos2D - A;
            float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                                   FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
            FVector2D Closest = A + AB * T;
            float Dist = (CandPos2D - Closest).Size();
            if (Dist < BestRoadDist)
            {
                BestRoadDist = Dist;
                BestRoadDir = (Closest - CandPos2D).GetSafeNormal();
            }
        }

        // Face toward nearest road with per-district jitter
        Lot.Yaw = FMath::RadiansToDegrees(FMath::Atan2(BestRoadDir.Y, BestRoadDir.X));
        Lot.Yaw += Rand.FRandRange(-MatchDef->RotationJitter, MatchDef->RotationJitter);

        // Per-district L-shape probability
        if (Rand.FRand() < MatchDef->LShapeChance &&
            (Style == EBuildingStyle::GuildHall || Style == EBuildingStyle::Keep ||
             Style == EBuildingStyle::TavernInn || Style == EBuildingStyle::Warehouse))
        {
            Lot.bHasWing = true;
            Lot.WingFootprint = FVector2D(Footprint.X * 0.4f, Footprint.Y * 0.6f);
            Lot.WingYawOffset = 90.f;
        }

        PlacedLots.Add(Lot);
        Placed++;
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] PlaceBuildings: %d road-lined + %d fill = %d / %d requested"),
           RoadLinedCount, Placed - RoadLinedCount, Placed, TargetBuildingCount);
}

// ─────────────────────────────────────────────────────────────────────────────
//  §8  MODULAR BUILDING MESH GENERATION
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SpawnModularBuilding(const FBuildingLot& Lot)
{
    const FVector& Center = Lot.Center;
    const float W = Lot.Footprint.X;
    const float D = Lot.Footprint.Y;
    const float Yaw = Lot.Yaw;
    const int32 Floors = Lot.NumFloors;

    float Z = Center.Z;

    // 1. Foundation
    SpawnFoundation(Center, W + 20.f, D + 20.f, FoundationHeight, Yaw);
    Z += FoundationHeight;

    // 2. Floor layers
    for (int32 F = 0; F < Floors; F++)
    {
        FVector FloorBase(Center.X, Center.Y, Z);
        bool bHasDoor = (F == 0);
        bool bHasWindows = true;
        SpawnWallFloor(FloorBase, W, D, FloorHeight, Yaw, bHasWindows, bHasDoor);
        Z += FloorHeight;
    }

    FVector RoofBase(Center.X, Center.Y, Z);

    // 3. Wing (L-shape) — spawned before main roof
    if (Lot.bHasWing)
    {
        // Offset wing to one side of main building
        float WingOffX = (W * 0.5f + Lot.WingFootprint.X * 0.5f) *
                         FMath::Cos(FMath::DegreesToRadians(Yaw));
        float WingOffY = (W * 0.5f + Lot.WingFootprint.X * 0.5f) *
                         FMath::Sin(FMath::DegreesToRadians(Yaw));
        FVector WingBase(Center.X + WingOffX, Center.Y + WingOffY, Center.Z);
        float WingH = GetTerrainHeight(WingBase.X - GetActorLocation().X,
                                       WingBase.Y - GetActorLocation().Y);
        WingBase.Z = GetActorLocation().Z + WingH;

        SpawnFoundation(WingBase, Lot.WingFootprint.X + 12.f,
                        Lot.WingFootprint.Y + 12.f, FoundationHeight, Yaw);
        float WZ = WingBase.Z + FoundationHeight;
        SpawnWallFloor(FVector(WingBase.X, WingBase.Y, WZ),
                       Lot.WingFootprint.X, Lot.WingFootprint.Y, FloorHeight, Yaw, true, false);

        FVector WingRoof(WingBase.X, WingBase.Y, WZ + FloorHeight);
        SpawnRoof_Pitched(WingRoof, Lot.WingFootprint.X, Lot.WingFootprint.Y,
                          FloorHeight * 0.5f, RoofOverhang, Yaw);
    }

    // 4. Roof (by type)
    float RoofH = (W < 500.f) ? FloorHeight * 0.55f : FloorHeight * 0.45f;
    if (Lot.Style == EBuildingStyle::Church)
        RoofH = FloorHeight * 1.2f;

    switch (Lot.Roof)
    {
    case ERoofType::Pitched:
        SpawnRoof_Pitched(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::Hipped:
        SpawnRoof_Hipped(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::Gambrel:
        SpawnRoof_Gambrel(RoofBase, W, D, RoofH, RoofOverhang, Yaw); break;
    case ERoofType::FlatParapet:
        SpawnRoof_FlatParapet(RoofBase, W, D, WallThickness * 0.8f, Yaw); break;
    case ERoofType::Conical:
        SpawnRoof_Conical(RoofBase, FMath::Max(W, D) * 0.5f, RoofH * 1.2f, 12); break;
    case ERoofType::Pyramidal:
        SpawnRoof_Pyramidal(RoofBase, W, D, RoofH, Yaw); break;
    case ERoofType::Thatched:
        // Thatched uses a low pitched roof with large overhang
        SpawnRoof_Pitched(RoofBase, W, D, RoofH * 0.7f, RoofOverhang * 1.6f, Yaw); break;
    }

    // 5. Chimney (pitched & gambrel roofs get chimneys)
    if (Lot.Roof == ERoofType::Pitched || Lot.Roof == ERoofType::Gambrel ||
        Lot.Roof == ERoofType::Thatched)
    {
        int32 NumChimneys = (Lot.Style == EBuildingStyle::Blacksmith ||
                             Lot.Style == EBuildingStyle::Bakery) ? 2 : 1;
        float TotalH = FoundationHeight + Floors * FloorHeight;
        SpawnChimney(Center, TotalH, Yaw, NumChimneys);
    }

    // 6. Ground-level props (barrels, crates, etc.)
    SpawnGroundProps(Center, W, D, Yaw, Lot.Style);
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnFoundation(FVector Center,
                                                                    float W, float D,
                                                                    float Height, float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Foundation"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Extend foundation below ground to embed into terrain slopes
    // Scale embed depth with residual terrain variation inside walls
    float EmbedDepth = FMath::Max(80.f, TerrainAmplitude * (1.f - TownFlattenStrength) * 0.8f);
    float TotalH = Height + EmbedDepth;
    // Centered box: center at TotalH/2 - EmbedDepth so bottom is at -EmbedDepth, top at Height
    AddBox(V, T, N, UV, FVector(0, 0, TotalH * 0.5f - EmbedDepth), W, D, TotalH);

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnWallFloor(FVector BaseCenter,
                                                                   float W, float D,
                                                                   float FloorH, float Yaw,
                                                                   bool bAddWindows,
                                                                   bool bAddDoor)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("WallFloor"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Main floor box (open top to reduce poly count on top face)
    AddOpenTopBox(V, T, N, UV, FVector(0, 0, FloorH * 0.5f), W, D, FloorH);

    // Window insets (front and back faces)
    if (bAddWindows)
    {
        int32 NumWin = FMath::Max(1, (int32)(W / 200.f));
        float WinSpacing = W / (NumWin + 1);
        float WinW = WinSpacing * 0.3f;
        float WinH = FloorH * 0.4f;
        float WinZ = FloorH * 0.55f;
        float WinDepth = 12.f;

        // Windows are represented as slightly inset quads (not full cutouts, keeps mesh simple)
        for (int32 Wi = 0; Wi < NumWin; Wi++)
        {
            float XOff = -W * 0.5f + WinSpacing * (Wi + 1);

            // Front face window (Y = -D/2)
            FVector FWC(XOff, -D * 0.5f - WinDepth * 0.01f, WinZ);
            AddBox(V, T, N, UV, FWC, WinW, 4.f, WinH);   // Thin inset

            // Back face window
            FVector BWC(XOff, D * 0.5f + WinDepth * 0.01f, WinZ);
            AddBox(V, T, N, UV, BWC, WinW, 4.f, WinH);
        }
    }

    // Door (ground floor front face)
    if (bAddDoor)
    {
        float DoorW = FMath::Min(W * 0.18f, 120.f);
        float DoorH = FloorH * 0.7f;
        float DoorXOff = Rand.FRandRange(-W * 0.15f, W * 0.15f);

        // Door frame center height: midpoint of the door pillar
        float PillarH = DoorH * 0.9f;
        float PillarCenterZ = PillarH * 0.5f;

        FVector DoorCenter(DoorXOff, -D * 0.5f + 2.f, 0.f);
        // Door frame (two side pillars — centered around PillarCenterZ)
        AddBox(V, T, N, UV,
               DoorCenter + FVector(-DoorW * 0.5f - 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        AddBox(V, T, N, UV,
               DoorCenter + FVector(DoorW * 0.5f + 8.f, 0, PillarCenterZ),
               12.f, 8.f, PillarH);
        // Lintel — centered at top of pillars
        AddBox(V, T, N, UV,
               DoorCenter + FVector(0, 0, DoorH + 8.f),
               DoorW + 28.f, 8.f, 16.f);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, WallMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

// ── Roof variants ─────────────────────────────────────────────────────────────

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Pitched(FVector BaseCenter,
                                                                      float W, float D,
                                                                      float RoofH,
                                                                      float Overhang,
                                                                      float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofPitched"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddPitchedRoof(V, T, N, UV, FVector::ZeroVector, W, D, RoofH, Overhang);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Hipped(FVector BaseCenter,
                                                                     float W, float D,
                                                                     float RoofH,
                                                                     float Overhang,
                                                                     float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofHipped"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddHippedRoof(V, T, N, UV, FVector::ZeroVector, W, D, RoofH, Overhang);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Gambrel(FVector BaseCenter,
                                                                      float W, float D,
                                                                      float RoofH,
                                                                      float Overhang,
                                                                      float Yaw)
{
    // Gambrel = two-stage pitch: lower steeper, upper shallower
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofGambrel"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    float LowerH = RoofH * 0.55f;
    float UpperH = RoofH * 0.45f;
    float LowerW = W;
    float UpperW = W * 0.55f;

    // Lower stage (steep)
    AddPitchedRoof(V, T, N, UV, FVector(0, 0, 0), LowerW, D, LowerH, Overhang);
    // Upper stage (shallower, on top of lower ridge)
    AddPitchedRoof(V, T, N, UV, FVector(0, 0, LowerH), UpperW, D, UpperH, 0.f);

    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_FlatParapet(FVector BaseCenter,
                                                                          float W, float D,
                                                                          float ParapetH,
                                                                          float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofFlat"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Flat cap
    AddBox(V, T, N, UV, FVector(0, 0, ParapetH * 0.5f), W, D, ParapetH);

    // Parapet walls on all 4 sides
    float PH = WallHeight * 0.12f;
    float PW = WallThickness * 0.6f;
    AddBox(V, T, N, UV, FVector(0, -D * 0.5f - PW * 0.5f, ParapetH + PH * 0.5f), W + PW*2, PW, PH);
    AddBox(V, T, N, UV, FVector(0,  D * 0.5f + PW * 0.5f, ParapetH + PH * 0.5f), W + PW*2, PW, PH);
    AddBox(V, T, N, UV, FVector(-W * 0.5f - PW * 0.5f, 0, ParapetH + PH * 0.5f), PW, D, PH);
    AddBox(V, T, N, UV, FVector( W * 0.5f + PW * 0.5f, 0, ParapetH + PH * 0.5f), PW, D, PH);

    // Battlements on parapet
    for (float XBatt = -W * 0.5f + 30.f; XBatt < W * 0.5f; XBatt += 60.f)
    {
        AddBox(V, T, N, UV,
               FVector(XBatt, -D * 0.5f - PW * 0.5f, ParapetH + PH + 25.f),
               30.f, PW, 40.f);
        AddBox(V, T, N, UV,
               FVector(XBatt, D * 0.5f + PW * 0.5f, ParapetH + PH + 25.f),
               30.f, PW, 40.f);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Conical(FVector BaseCenter,
                                                                      float Radius,
                                                                      float ConeH, int32 Segs)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofConical"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddCone(V, T, N, UV, FVector::ZeroVector, Radius * 1.08f, ConeH, Segs);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnRoof_Pyramidal(FVector BaseCenter,
                                                                        float W, float D,
                                                                        float PyramidH, float Yaw)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("RoofPyramid"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
    AddPyramid(V, T, N, UV, FVector::ZeroVector, W, D, PyramidH);
    SetMeshSection(Mesh, 0, V, T, N, UV, RoofMaterial);
    Mesh->SetWorldLocation(BaseCenter);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnChimney(FVector Center,
                                                                float BuildingH, float Yaw,
                                                                int32 Count)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Chimney"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    float ChimneyH = BuildingH * 0.35f;
    float ChimR = 28.f;

    for (int32 i = 0; i < Count; i++)
    {
        float XOff = Count > 1 ? (i == 0 ? -70.f : 70.f) : 0.f;
        float YOff = Rand.FRandRange(-40.f, 40.f);
        FVector Base(XOff, YOff, BuildingH);
        // Stack: body cylinder + cap flare
        AddCylinder(V, T, N, UV, Base, ChimR, ChimneyH, 8, false);
        AddCylinder(V, T, N, UV, Base + FVector(0,0,ChimneyH), ChimR * 1.35f, 20.f, 8, true);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnGroundProps(FVector Center, float W,
                                                                     float D, float Yaw,
                                                                     EBuildingStyle Style)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Props"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    int32 NumProps = Rand.RandRange(0, 3);
    for (int32 i = 0; i < NumProps; i++)
    {
        float PX = Rand.FRandRange(-W * 0.6f, W * 0.6f);
        float PY = Rand.FRandRange(-D * 0.6f, D * 0.6f);

        if (Style == EBuildingStyle::Blacksmith)
        {
            // Anvil-like block
            AddBox(V, T, N, UV, FVector(PX, PY, 35.f), 60.f, 40.f, 55.f);
        }
        else if (Style == EBuildingStyle::Bakery || Style == EBuildingStyle::TavernInn)
        {
            // Barrel: cylinder
            AddCylinder(V, T, N, UV, FVector(PX, PY, 0), 22.f, 55.f, 8, true);
        }
        else
        {
            // Generic crate
            float S = Rand.FRandRange(30.f, 70.f);
            AddBox(V, T, N, UV, FVector(PX, PY, S * 0.5f), S, S, S);
        }
    }

    if (V.Num() == 0) { Mesh->DestroyComponent(); return nullptr; }

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Center);
    Mesh->SetWorldRotation(FRotator(0, Yaw, 0));
    return Mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
//  §9  ROAD MESH RENDERING
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SpawnRoadMesh(const FRoadEdge& Edge)
{
    if (Edge.WorldPoints.Num() < 2) return;

    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Road"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    const float HalfW = Edge.Width * 0.5f;

    for (int32 i = 0; i < Edge.WorldPoints.Num() - 1; i++)
    {
        FVector P0 = Edge.WorldPoints[i];
        FVector P1 = Edge.WorldPoints[i + 1];
        FVector Dir = (P1 - P0);
        Dir.Z = 0.f;
        float Len = Dir.Size();
        if (Len < 1.f) continue;
        Dir /= Len;

        FVector Right(Dir.Y, -Dir.X, 0.f);

        // Raise road above terrain to prevent Z-fighting
        FVector L0 = P0 - Right * HalfW;
        FVector R0 = P0 + Right * HalfW;
        FVector L1 = P1 - Right * HalfW;
        FVector R1 = P1 + Right * HalfW;

        int32 Base = V.Num();
        V.Add(L0); V.Add(R0); V.Add(L1); V.Add(R1);

        float U0 = (float)i / (Edge.WorldPoints.Num()-1);
        float U1 = (float)(i+1) / (Edge.WorldPoints.Num()-1);
        UV.Add(FVector2D(0, U0)); UV.Add(FVector2D(1, U0));
        UV.Add(FVector2D(0, U1)); UV.Add(FVector2D(1, U1));

        FVector FaceN = FVector::UpVector;
        N.Add(FaceN); N.Add(FaceN); N.Add(FaceN); N.Add(FaceN);

        // CW from above = front face visible from +Z in UE5 (road surface)
        T.Add(Base); T.Add(Base+2); T.Add(Base+1);
        T.Add(Base+1); T.Add(Base+2); T.Add(Base+3);
    }

    // Use RoadMaterial if set, otherwise fall back to StoneMaterial
    UMaterialInterface* RoadMat = RoadMaterial ? RoadMaterial : StoneMaterial;
    SetMeshSection(Mesh, 0, V, T, N, UV, RoadMat);

    // Disable collision on roads — they're flat overlays and shouldn't block movement
    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void AMedievalTownGenerator::SpawnBridgeMesh(const FRoadEdge& Edge)
{
    // Only adds RAILINGS for the segments actually over the river.
    // The road deck is already rendered by SpawnRoadMesh for the full edge.
    if (Edge.WorldPoints.Num() < 2) return;

    UProceduralMeshComponent* RailMesh = CreateMesh(TEXT("Bridge_Rails"));
    TArray<FVector> RV; TArray<int32> RT; TArray<FVector> RN; TArray<FVector2D> RUV;

    const float HalfW = Edge.Width * 0.5f;
    const float RailH = 100.f;      // Railing height
    const float RailThick = 40.f;   // Railing thickness
    const float HalfRiver = RiverWidth * 0.5f;

    FVector Origin = GetActorLocation();

    for (int32 i = 0; i < Edge.WorldPoints.Num() - 1; i++)
    {
        FVector P0 = Edge.WorldPoints[i];
        FVector P1 = Edge.WorldPoints[i + 1];

        // Check if the midpoint of THIS segment is over the river
        FVector Mid = (P0 + P1) * 0.5f;
        FVector2D Mid2D(Mid.X - Origin.X, Mid.Y - Origin.Y);
        float RDist = DistToRiverCenter(Mid2D);
        if (RDist > HalfRiver * 1.3f) continue;  // Not over river — skip

        FVector Dir = (P1 - P0); Dir.Z = 0.f;
        float Len = Dir.Size2D();
        if (Len < 1.f) continue;
        Dir /= Len;
        FVector Right(Dir.Y, -Dir.X, 0.f);

        // Left railing
        FVector LL0 = P0 - Right * (HalfW + RailThick);
        FVector LR0 = P0 - Right * HalfW;
        FVector LL1 = P1 - Right * (HalfW + RailThick);
        FVector LR1 = P1 - Right * HalfW;

        int32 B = RV.Num();
        RV.Add(LL0); RV.Add(LR0); RV.Add(LL1); RV.Add(LR1);
        RV.Add(LL0 + FVector(0,0,RailH)); RV.Add(LR0 + FVector(0,0,RailH));
        RV.Add(LL1 + FVector(0,0,RailH)); RV.Add(LR1 + FVector(0,0,RailH));
        for (int32 v = 0; v < 8; v++) { RN.Add(FVector::UpVector); RUV.Add(FVector2D(0,0)); }

        // Outer face
        RT.Add(B+0); RT.Add(B+2); RT.Add(B+4);
        RT.Add(B+4); RT.Add(B+2); RT.Add(B+6);
        // Inner face
        RT.Add(B+1); RT.Add(B+5); RT.Add(B+3);
        RT.Add(B+5); RT.Add(B+7); RT.Add(B+3);
        // Top face
        RT.Add(B+4); RT.Add(B+6); RT.Add(B+5);
        RT.Add(B+5); RT.Add(B+6); RT.Add(B+7);

        // Right railing
        FVector RL0 = P0 + Right * HalfW;
        FVector RR0 = P0 + Right * (HalfW + RailThick);
        FVector RL1 = P1 + Right * HalfW;
        FVector RR1 = P1 + Right * (HalfW + RailThick);

        B = RV.Num();
        RV.Add(RL0); RV.Add(RR0); RV.Add(RL1); RV.Add(RR1);
        RV.Add(RL0 + FVector(0,0,RailH)); RV.Add(RR0 + FVector(0,0,RailH));
        RV.Add(RL1 + FVector(0,0,RailH)); RV.Add(RR1 + FVector(0,0,RailH));
        for (int32 v = 0; v < 8; v++) { RN.Add(FVector::UpVector); RUV.Add(FVector2D(0,0)); }

        RT.Add(B+0); RT.Add(B+4); RT.Add(B+2);
        RT.Add(B+4); RT.Add(B+6); RT.Add(B+2);
        RT.Add(B+1); RT.Add(B+3); RT.Add(B+5);
        RT.Add(B+5); RT.Add(B+3); RT.Add(B+7);
        RT.Add(B+4); RT.Add(B+5); RT.Add(B+6);
        RT.Add(B+5); RT.Add(B+7); RT.Add(B+6);
    }

    if (RV.Num() > 0)
    {
        SetMeshSection(RailMesh, 0, RV, RT, RN, RUV, StoneMaterial);
        UE_LOG(LogTemp, Log, TEXT("[MTG] Bridge railings: %d verts over river"), RV.Num());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  §10  FOREST  (Perlin density ring)
// ─────────────────────────────────────────────────────────────────────────────

float AMedievalTownGenerator::ForestDensityAt(float X, float Y) const
{
    // Clustered density using noise
    return (SampleNoise(X, Y, 3, ForestDensityFrequency, 1.f, 0.5f, 2.f) + 1.f) * 0.5f;
}

void AMedievalTownGenerator::PlaceForest()
{
    float InnerR = TownRadius * ForestRingInnerFraction;
    float OuterR = TownRadius * ForestRingOuterFraction;

    int32 Placed = 0;
    int32 Tries = 0;
    const int32 MaxTries = ForestTreeCount * 15;
    const float MinTreeSep = (TreeCrownRadiusMax + 20.f) * 1.5f;

    TArray<FVector2D> PlacedTrees;

    while (Placed < ForestTreeCount && Tries < MaxTries)
    {
        Tries++;
        FVector2D Pos = RandAnnulus(InnerR, OuterR);

        // Noise density gating
        float Density = ForestDensityAt(Pos.X, Pos.Y);
        if (Rand.FRand() > Density) continue;

        // Separation check
        bool bTooClose = false;
        for (const FVector2D& PT : PlacedTrees)
        {
            if ((Pos - PT).Size() < MinTreeSep) { bTooClose = true; break; }
        }
        if (bTooClose) continue;

        float H = GetTerrainHeight(Pos.X, Pos.Y);
        FVector Loc = GetActorLocation() + FVector(Pos.X, Pos.Y, H);

        float TreeH = Rand.FRandRange(TreeHeightMin, TreeHeightMax);
        float CrownR = Rand.FRandRange(TreeCrownRadiusMin, TreeCrownRadiusMax);
        int32 CrownTiers = Rand.RandRange(2, 4);

        SpawnTree(Loc, TreeH, CrownR, CrownTiers);
        PlacedTrees.Add(Pos);
        Placed++;
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] Forest: %d trees placed"), Placed);
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnTree(FVector Location,
                                                              float TreeH, float CrownRadius,
                                                              int32 CrownTiers)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Tree"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Trunk
    float TrunkH = TreeH * 0.35f;
    float TrunkR = CrownRadius * 0.12f;
    AddCylinder(V, T, N, UV, FVector::ZeroVector, TrunkR, TrunkH, 6, false);

    // Crown tiers (stacked cones)
    float TierStep = (TreeH - TrunkH) / CrownTiers;
    for (int32 Tier = 0; Tier < CrownTiers; Tier++)
    {
        float TierZ = TrunkH + Tier * TierStep * 0.6f;
        float TierR = CrownRadius * (1.f - (float)Tier / CrownTiers) * 0.85f;
        float TierH = TierStep * 1.1f;
        AddCone(V, T, N, UV, FVector(0, 0, TierZ), TierR, TierH, 8);
    }

    SetMeshSection(Mesh, 0, V, T, N, UV, TreeMaterial);
    Mesh->SetWorldLocation(Location);

    // Slight random rotation for variety
    float RandomYaw = Rand.FRandRange(0.f, 360.f);
    Mesh->SetWorldRotation(FRotator(0, RandomYaw, 0));

    return Mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
//  §11  MOUNTAINS
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SpawnMountains()
{
    for (int32 i = 0; i < MountainCount; i++)
    {
        float Angle = (float)i / MountainCount * 360.f + Rand.FRandRange(-20.f, 20.f);
        float Rad = FMath::DegreesToRadians(Angle);
        float Dist = TownRadius * Rand.FRandRange(MountainRingFraction * 0.75f,
                                                   MountainRingFraction);

        FVector2D Pos2D(FMath::Cos(Rad) * Dist, FMath::Sin(Rad) * Dist);
        float H = GetTerrainHeight(Pos2D.X, Pos2D.Y);
        FVector Loc = GetActorLocation() + FVector(Pos2D.X, Pos2D.Y, H);

        FVector Scale;
        Scale.X = Rand.FRandRange(MountainScaleMin.X, MountainScaleMax.X);
        Scale.Y = Rand.FRandRange(MountainScaleMin.Y, MountainScaleMax.Y);
        Scale.Z = Rand.FRandRange(MountainScaleMin.Z, MountainScaleMax.Z);

        SpawnMountainPeak(Loc, Scale);
    }
}

UProceduralMeshComponent* AMedievalTownGenerator::SpawnMountainPeak(FVector Location,
                                                                      FVector Scale)
{
    UProceduralMeshComponent* Mesh = CreateMesh(TEXT("Mountain"));
    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;

    // Main peak cone
    AddCone(V, T, N, UV, FVector::ZeroVector, Scale.X, Scale.Z, 12);

    // Snowcap
    // Snowcap: starts at 72% of peak height, cone radius = Scale.X * (1 - 0.72) = 28% of base
    float SnowStart = Scale.Z * 0.72f;
    float SnowR     = Scale.X * 0.28f;   // Correct: linear interpolation of cone radius at SnowStart
    AddCone(V, T, N, UV, FVector(0, 0, SnowStart), SnowR, Scale.Z * 0.28f, 12);

    SetMeshSection(Mesh, 0, V, T, N, UV, StoneMaterial);
    Mesh->SetWorldLocation(Location);

    float RandomYaw = Rand.FRandRange(0.f, 360.f);
    Mesh->SetWorldRotation(FRotator(0, RandomYaw, 0));
    return Mesh;
}

// ─────────────────────────────────────────────────────────────────────────────
//  §12  SAVE / LOAD LAYOUT
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SaveCurrentLayout()
{
    SavedLayout = FSavedTownLayout();
    SavedLayout.LockedSeed = RandomSeed;
    SavedLayout.bIsValid = true;

    for (const FBuildingLot& Lot : PlacedLots)
    {
        if (!Lot.bIsPlaced) continue;

        FSavedTransform ST;
        ST.Location = Lot.Center;
        ST.Rotation = FRotator(0, Lot.Yaw, 0);
        ST.StyleIndex = (int32)Lot.Style;
        ST.DistrictIndex = (int32)Lot.District;
        ST.NumFloors = Lot.NumFloors;
        ST.RoofTypeIndex = (int32)Lot.Roof;
        ST.Footprint = Lot.Footprint;
        SavedLayout.Buildings.Add(ST);
    }

    for (const FVector& WP : CachedRiverWorldPath)
        SavedLayout.RiverPoints.Add(WP);

    for (const FVector& WP : WallPerimeter)
        SavedLayout.WallPoints.Add(WP);

    UE_LOG(LogTemp, Log, TEXT("[MTG] SaveLayout: %d buildings, %d river pts, %d wall pts"),
           SavedLayout.Buildings.Num(), SavedLayout.RiverPoints.Num(),
           SavedLayout.WallPoints.Num());
}

void AMedievalTownGenerator::LoadSavedLayout()
{
    if (!SavedLayout.bIsValid)
    {
        UE_LOG(LogTemp, Warning, TEXT("[MTG] LoadSavedLayout: no valid saved layout"));
        return;
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] Loading saved layout (seed=%d)"), SavedLayout.LockedSeed);

    // Rebuild river path from saved points
    CachedRiverWorldPath = SavedLayout.RiverPoints;
    WallPerimeter = SavedLayout.WallPoints;

    // Rebuild terrain cache (needed for height queries)
    BuildTerrainCache();

    // Re-create building meshes from saved transforms
    for (const FSavedTransform& ST : SavedLayout.Buildings)
    {
        FBuildingLot Lot;
        Lot.Center = ST.Location;
        Lot.Yaw = ST.Rotation.Yaw;
        Lot.Style = (EBuildingStyle)ST.StyleIndex;
        Lot.District = (EDistrictType)ST.DistrictIndex;
        Lot.NumFloors = ST.NumFloors;
        Lot.Roof = (ERoofType)ST.RoofTypeIndex;
        Lot.Footprint = ST.Footprint;
        Lot.bIsPlaced = true;

        PlacedLots.Add(Lot);
        SpawnModularBuilding(Lot);
    }

    UE_LOG(LogTemp, Log, TEXT("[MTG] LoadSavedLayout complete: %d buildings"),
           PlacedLots.Num());
}

// ─────────────────────────────────────────────────────────────────────────────
//  §13  GEOMETRY PRIMITIVES
// ─────────────────────────────────────────────────────────────────────────────

UProceduralMeshComponent* AMedievalTownGenerator::CreateMesh(const FString& Name)
{
    // Generate unique name to avoid UE naming collisions
    FString UniqueName = FString::Printf(TEXT("%s_%d"), *Name, GeneratedMeshes.Num());
    UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, *UniqueName);
    Mesh->SetupAttachment(GetRootComponent());
    Mesh->RegisterComponent();
    GeneratedMeshes.Add(Mesh);
    return Mesh;
}

void AMedievalTownGenerator::SetMeshSection(UProceduralMeshComponent* Mesh, int32 Section,
                                             TArray<FVector>& V, TArray<int32>& T,
                                             TArray<FVector>& N, TArray<FVector2D>& UV,
                                             UMaterialInterface* Mat)
{
    if (!IsValid(Mesh) || V.Num() == 0) return;

    TArray<FColor> Colors;
    TArray<FProcMeshTangent> Tangents;
    Mesh->CreateMeshSection(Section, V, T, N, UV, Colors, Tangents, true);

    if (Mat)
        Mesh->SetMaterial(Section, Mat);
}

void AMedievalTownGenerator::ApplyMaterial(UProceduralMeshComponent* Mesh,
                                            UMaterialInterface* Mat)
{
    if (IsValid(Mesh) && Mat)
        Mesh->SetMaterial(0, Mat);
}

// ── Box (6-faced solid) — CENTERED around Center ─────────────────────────────
void AMedievalTownGenerator::AddBox(TArray<FVector>& V, TArray<int32>& T,
                                     TArray<FVector>& N, TArray<FVector2D>& UV,
                                     FVector Center, float W, float D, float H)
{
    float HW = W * 0.5f, HD = D * 0.5f, HH = H * 0.5f;
    FVector Corners[8] = {
        Center + FVector(-HW, -HD, -HH),
        Center + FVector( HW, -HD, -HH),
        Center + FVector( HW,  HD, -HH),
        Center + FVector(-HW,  HD, -HH),
        Center + FVector(-HW, -HD,  HH),
        Center + FVector( HW, -HD,  HH),
        Center + FVector( HW,  HD,  HH),
        Center + FVector(-HW,  HD,  HH),
    };

    // face indices: [verts], normal
    struct FaceData { int32 VI[4]; FVector Norm; };
    FaceData Faces[6] = {
        {{0,1,5,4}, FVector(0,-1,0)},  // Front  (-Y)
        {{2,3,7,6}, FVector(0, 1,0)},  // Back   (+Y)
        {{1,2,6,5}, FVector(1, 0,0)},  // Right  (+X)
        {{3,0,4,7}, FVector(-1,0,0)},  // Left   (-X)
        {{4,5,6,7}, FVector(0, 0,1)},  // Top    (+Z)
        {{1,0,3,2}, FVector(0, 0,-1)}, // Bottom (-Z)
    };

    for (const FaceData& F : Faces)
    {
        int32 Base = V.Num();
        for (int32 vi = 0; vi < 4; vi++)
        {
            V.Add(Corners[F.VI[vi]]);
            N.Add(F.Norm);
        }
        UV.Add(FVector2D(0,0)); UV.Add(FVector2D(1,0));
        UV.Add(FVector2D(1,1)); UV.Add(FVector2D(0,1));

        T.Add(Base); T.Add(Base+2); T.Add(Base+1);
        T.Add(Base); T.Add(Base+3); T.Add(Base+2);
    }
}

// ── Box without top face (saves polys on interior floor tops) — CENTERED ─────
void AMedievalTownGenerator::AddOpenTopBox(TArray<FVector>& V, TArray<int32>& T,
                                            TArray<FVector>& N, TArray<FVector2D>& UV,
                                            FVector Center, float W, float D, float H)
{
    float HW = W * 0.5f, HD = D * 0.5f, HH = H * 0.5f;
    FVector Corners[8] = {
        Center + FVector(-HW, -HD, -HH),
        Center + FVector( HW, -HD, -HH),
        Center + FVector( HW,  HD, -HH),
        Center + FVector(-HW,  HD, -HH),
        Center + FVector(-HW, -HD,  HH),
        Center + FVector( HW, -HD,  HH),
        Center + FVector( HW,  HD,  HH),
        Center + FVector(-HW,  HD,  HH),
    };

    struct FaceData { int32 VI[4]; FVector Norm; };
    FaceData Faces[5] = {
        {{0,1,5,4}, FVector(0,-1,0)},
        {{2,3,7,6}, FVector(0, 1,0)},
        {{1,2,6,5}, FVector(1, 0,0)},
        {{3,0,4,7}, FVector(-1,0,0)},
        {{1,0,3,2}, FVector(0, 0,-1)},  // Bottom only
    };

    for (const FaceData& F : Faces)
    {
        int32 Base = V.Num();
        for (int32 vi = 0; vi < 4; vi++)
        { V.Add(Corners[F.VI[vi]]); N.Add(F.Norm); }
        UV.Add(FVector2D(0,0)); UV.Add(FVector2D(1,0));
        UV.Add(FVector2D(1,1)); UV.Add(FVector2D(0,1));
        T.Add(Base); T.Add(Base+2); T.Add(Base+1);
        T.Add(Base); T.Add(Base+3); T.Add(Base+2);
    }
}

// ── Pitched (gabled) roof ─────────────────────────────────────────────────────
void AMedievalTownGenerator::AddPitchedRoof(TArray<FVector>& V, TArray<int32>& T,
                                             TArray<FVector>& N, TArray<FVector2D>& UV,
                                             FVector Base, float W, float D,
                                             float RoofH, float Overhang)
{
    float HW = W * 0.5f + Overhang;
    float HD = D * 0.5f + Overhang;
    float Ridge = RoofH;

    // Ridge runs along X axis (long axis)
    FVector RidgeL(-HW + Overhang, 0.f, Ridge);
    FVector RidgeR( HW - Overhang, 0.f, Ridge);

    FVector BFL(-HW, -HD, 0); FVector BFR( HW, -HD, 0);
    FVector BBL(-HW,  HD, 0); FVector BBR( HW,  HD, 0);

    // Front slope  (Base added to ALL corners for consistent Gambrel stacking)
    AddQuad(V, T, N, UV, Base+BFL, Base+BFR, Base+RidgeR, Base+RidgeL);
    // Back slope
    AddQuad(V, T, N, UV, Base+BBR, Base+BBL, Base+RidgeL, Base+RidgeR);
    // Left gable triangle
    {
        int32 B = V.Num();
        V.Add(Base+BFL); V.Add(Base+BBL); V.Add(Base+RidgeL);
        FVector Norm = FVector::CrossProduct((Base+BBL)-(Base+BFL),(Base+RidgeL)-(Base+BFL)).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D(0,0)); UV.Add(FVector2D(1,0)); UV.Add(FVector2D(0.5f,1));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
    // Right gable triangle
    {
        int32 B = V.Num();
        V.Add(Base+BFR); V.Add(Base+RidgeR); V.Add(Base+BBR);
        FVector Norm = FVector::CrossProduct((Base+RidgeR)-(Base+BFR),(Base+BBR)-(Base+BFR)).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D(0,0)); UV.Add(FVector2D(0.5f,1)); UV.Add(FVector2D(1,0));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
}

// ── Hipped roof ───────────────────────────────────────────────────────────────
void AMedievalTownGenerator::AddHippedRoof(TArray<FVector>& V, TArray<int32>& T,
                                            TArray<FVector>& N, TArray<FVector2D>& UV,
                                            FVector Base, float W, float D,
                                            float RoofH, float Overhang)
{
    float HW = W * 0.5f + Overhang;
    float HD = D * 0.5f + Overhang;

    // Ridge: shortened, runs along X, centered
    float RidgeLen = FMath::Max(0.f, W - D) * 0.45f;
    FVector RL(-RidgeLen, 0, RoofH), RR(RidgeLen, 0, RoofH);

    FVector BFL(-HW, -HD, 0), BFR(HW, -HD, 0);
    FVector BBL(-HW,  HD, 0), BBR(HW,  HD, 0);

    // Front face
    AddQuad(V, T, N, UV, Base+BFL, Base+BFR, Base+RR, Base+RL);
    // Back face
    AddQuad(V, T, N, UV, Base+BBR, Base+BBL, Base+RL, Base+RR);
    // Left face
    {
        int32 B = V.Num();
        V.Add(Base+BFL); V.Add(Base+BBL); V.Add(Base+RL);
        FVector Norm = FVector::CrossProduct((Base+BBL)-(Base+BFL),(Base+RL)-(Base+BFL)).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D(0,0));UV.Add(FVector2D(1,0));UV.Add(FVector2D(0.5f,1));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
    // Right face
    {
        int32 B = V.Num();
        V.Add(Base+BFR); V.Add(Base+RR); V.Add(Base+BBR);
        FVector Norm = FVector::CrossProduct((Base+RR)-(Base+BFR),(Base+BBR)-(Base+BFR)).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D(0,0));UV.Add(FVector2D(0.5f,1));UV.Add(FVector2D(1,0));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
}

// ── Pyramid ──────────────────────────────────────────────────────────────────
void AMedievalTownGenerator::AddPyramid(TArray<FVector>& V, TArray<int32>& T,
                                         TArray<FVector>& N, TArray<FVector2D>& UV,
                                         FVector Base, float W, float D, float H)
{
    float HW = W * 0.5f, HD = D * 0.5f;
    FVector Apex = Base + FVector(0, 0, H);
    FVector Pts[4] = {
        Base + FVector(-HW,-HD,0), Base + FVector(HW,-HD,0),
        Base + FVector(HW, HD,0), Base + FVector(-HW,HD,0)
    };

    // 4 triangular faces
    for (int32 i = 0; i < 4; i++)
    {
        int32 j = (i + 1) % 4;
        int32 B = V.Num();
        V.Add(Pts[i]); V.Add(Pts[j]); V.Add(Apex);
        FVector Norm = FVector::CrossProduct(Pts[j]-Pts[i], Apex-Pts[i]).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D(0,0)); UV.Add(FVector2D(1,0)); UV.Add(FVector2D(0.5f,1));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
    // Base quad
    AddQuad(V, T, N, UV, Pts[3], Pts[2], Pts[1], Pts[0]);
}

// ── Cylinder ─────────────────────────────────────────────────────────────────
void AMedievalTownGenerator::AddCylinder(TArray<FVector>& V, TArray<int32>& T,
                                          TArray<FVector>& N, TArray<FVector2D>& UV,
                                          FVector Base, float Radius, float Height,
                                          int32 Segments, bool bCap)
{
    float AngleStep = TWO_PI / Segments;

    // Side faces
    for (int32 S = 0; S < Segments; S++)
    {
        float A0 = S * AngleStep, A1 = (S + 1) * AngleStep;
        float Cos0 = FMath::Cos(A0), Sin0 = FMath::Sin(A0);
        float Cos1 = FMath::Cos(A1), Sin1 = FMath::Sin(A1);

        FVector B0 = Base + FVector(Cos0*Radius, Sin0*Radius, 0);
        FVector B1 = Base + FVector(Cos1*Radius, Sin1*Radius, 0);
        FVector T0 = B0 + FVector(0, 0, Height);
        FVector T1 = B1 + FVector(0, 0, Height);

        FVector Norm = FVector(Cos0+Cos1, Sin0+Sin1, 0).GetSafeNormal();

        int32 Bx = V.Num();
        V.Add(B0); V.Add(B1); V.Add(T1); V.Add(T0);
        N.Add(Norm); N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D((float)S/Segments, 0));
        UV.Add(FVector2D((float)(S+1)/Segments, 0));
        UV.Add(FVector2D((float)(S+1)/Segments, 1));
        UV.Add(FVector2D((float)S/Segments, 1));
        T.Add(Bx); T.Add(Bx+2); T.Add(Bx+1);
        T.Add(Bx); T.Add(Bx+3); T.Add(Bx+2);
    }

    if (bCap)
    {
        // Top cap (fan)
        FVector TopCenter = Base + FVector(0, 0, Height);
        int32 CenterIdx = V.Num();
        V.Add(TopCenter); N.Add(FVector::UpVector); UV.Add(FVector2D(0.5f, 0.5f));

        for (int32 S = 0; S < Segments; S++)
        {
            float A0 = S * AngleStep, A1 = (S + 1) * AngleStep;
            int32 B = V.Num();
            V.Add(TopCenter + FVector(FMath::Cos(A0)*Radius, FMath::Sin(A0)*Radius, 0));
            V.Add(TopCenter + FVector(FMath::Cos(A1)*Radius, FMath::Sin(A1)*Radius, 0));
            N.Add(FVector::UpVector); N.Add(FVector::UpVector);
            UV.Add(FVector2D(FMath::Cos(A0)*0.5f+0.5f, FMath::Sin(A0)*0.5f+0.5f));
            UV.Add(FVector2D(FMath::Cos(A1)*0.5f+0.5f, FMath::Sin(A1)*0.5f+0.5f));
            // CW from above = front face visible from +Z in UE5 left-handed system
            T.Add(CenterIdx); T.Add(B+1); T.Add(B);
        }
    }
}

// ── Cone ─────────────────────────────────────────────────────────────────────
void AMedievalTownGenerator::AddCone(TArray<FVector>& V, TArray<int32>& T,
                                      TArray<FVector>& N, TArray<FVector2D>& UV,
                                      FVector Base, float Radius, float Height, int32 Segs)
{
    FVector Apex = Base + FVector(0, 0, Height);
    float AngleStep = TWO_PI / Segs;

    for (int32 S = 0; S < Segs; S++)
    {
        float A0 = S * AngleStep, A1 = (S + 1) * AngleStep;
        FVector P0 = Base + FVector(FMath::Cos(A0)*Radius, FMath::Sin(A0)*Radius, 0);
        FVector P1 = Base + FVector(FMath::Cos(A1)*Radius, FMath::Sin(A1)*Radius, 0);

        int32 B = V.Num();
        V.Add(P0); V.Add(P1); V.Add(Apex);
        FVector Norm = FVector::CrossProduct(P1-P0, Apex-P0).GetSafeNormal();
        N.Add(Norm); N.Add(Norm); N.Add(Norm);
        UV.Add(FVector2D((float)S/Segs, 0));
        UV.Add(FVector2D((float)(S+1)/Segs, 0));
        UV.Add(FVector2D((float)S/Segs+0.5f/Segs, 1));
        T.Add(B); T.Add(B+2); T.Add(B+1);
    }
}

// ── Quad (single planar face — 2 tris) ───────────────────────────────────────
void AMedievalTownGenerator::AddQuad(TArray<FVector>& V, TArray<int32>& T,
                                      TArray<FVector>& N, TArray<FVector2D>& UV,
                                      FVector P0, FVector P1, FVector P2, FVector P3)
{
    FVector Norm = FVector::CrossProduct(P1-P0, P3-P0).GetSafeNormal();
    int32 Base = V.Num();
    V.Add(P0); V.Add(P1); V.Add(P2); V.Add(P3);
    N.Add(Norm); N.Add(Norm); N.Add(Norm); N.Add(Norm);
    UV.Add(FVector2D(0,0)); UV.Add(FVector2D(1,0));
    UV.Add(FVector2D(1,1)); UV.Add(FVector2D(0,1));
    T.Add(Base); T.Add(Base+2); T.Add(Base+1);
    T.Add(Base); T.Add(Base+3); T.Add(Base+2);
}

// ─────────────────────────────────────────────────────────────────────────────
//  §14  MATH HELPERS
// ─────────────────────────────────────────────────────────────────────────────

FVector2D AMedievalTownGenerator::RandInsideCircle(float Radius)
{
    float Angle = Rand.FRandRange(0.f, TWO_PI);
    float R = FMath::Sqrt(Rand.FRand()) * Radius;   // Sqrt for uniform distribution
    return FVector2D(FMath::Cos(Angle) * R, FMath::Sin(Angle) * R);
}

FVector2D AMedievalTownGenerator::RandAnnulus(float InnerR, float OuterR)
{
    float Angle = Rand.FRandRange(0.f, TWO_PI);
    float R = FMath::Sqrt(Rand.FRandRange(InnerR * InnerR, OuterR * OuterR));
    return FVector2D(FMath::Cos(Angle) * R, FMath::Sin(Angle) * R);
}

bool AMedievalTownGenerator::CircleOverlapsSegment(FVector2D Center, float R,
                                                    FVector2D A, FVector2D B) const
{
    FVector2D AB = B - A, AC = Center - A;
    float T = FMath::Clamp(FVector2D::DotProduct(AC, AB) /
                           FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
    return (Center - (A + AB * T)).SizeSquared() < R * R;
}

float AMedievalTownGenerator::Dist2D(FVector A, FVector B) const
{
    return FMath::Sqrt(FMath::Square(A.X - B.X) + FMath::Square(A.Y - B.Y));
}

FVector AMedievalTownGenerator::RotateAroundZ(FVector V, FVector Center, float AngleDeg) const
{
    float Rad = FMath::DegreesToRadians(AngleDeg);
    float Cos = FMath::Cos(Rad), Sin = FMath::Sin(Rad);
    FVector Offset = V - Center;
    return Center + FVector(Offset.X * Cos - Offset.Y * Sin,
                             Offset.X * Sin + Offset.Y * Cos,
                             Offset.Z);
}
