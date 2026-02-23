// =============================================================================
// MedievalTownGenerator.cpp  —  VERSION 18
// =============================================================================
// Core orchestration + shared systems for AMedievalTownGenerator.
// Terrain/Roads/Walls/Buildings are split into dedicated helper translation units.
//
// Section map:
//   §1  Constructor / lifecycle
//   §2  Main pipeline  (GenerateTown / ClearTown / phases 1–9)
//   §3  Terrain  (moved to MedievalTownGeneratorTerrain.cpp)
//   §4  River path generation (waypoints + world path)
//   §5  Road network  (moved to MedievalTownGeneratorRoads.cpp)
//   §6  Shape grammar walls (moved to MedievalTownGeneratorWalls.cpp)
//   §7-8 Building systems (moved to MedievalTownGeneratorBuildings.cpp)
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
    CachedRiverPlanarPath.Empty();
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
    BuildRiverPlanarPath();
}

void AMedievalTownGenerator::Phase2_SetupTerrain()
{
    BuildTerrainCache();

    if (bUseImprovedRiverMeshes && bGenerateRiver)
    {
        GenerateAdaptiveTerrainMesh();
        return;
    }

    // Spawn terrain mesh
    const FString Name = TEXT("Terrain");
    UProceduralMeshComponent* Mesh = CreateMesh(Name);

    const int32 Res = TerrainResolution;
    const float Ext = TownRadius * FMath::Max(1.1f, ForestRingOuterFraction + 0.1f);
    const float Step = (Ext * 2.f) / Res;

    TArray<FVector> V; TArray<int32> T; TArray<FVector> N; TArray<FVector2D> UV;
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

            const float Delta = Step * 0.5f;
            float Hx = GetTerrainHeight(X + Delta, Y) - GetTerrainHeight(X - Delta, Y);
            float Hy = GetTerrainHeight(X, Y + Delta) - GetTerrainHeight(X, Y - Delta);
            N.Add(FVector(-Hx / (2.f * Delta), -Hy / (2.f * Delta), 1.f).GetSafeNormal());
        }
    }

    T.Reserve(Res * Res * 6);
    for (int32 Row = 0; Row < Res; Row++)
    {
        for (int32 Col = 0; Col < Res; Col++)
        {
            int32 BL = Row * VPerRow + Col;
            int32 BR = BL + 1;
            int32 TL = BL + VPerRow;
            int32 TR = TL + 1;
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
        if (bUseImprovedRiverMeshes)
        {
            SpawnImprovedRiverMeshes();
        }
        else
        {
            const int32 NumPts = CachedRiverWorldPath.Num();
            TArray<FVector> SurfaceV; TArray<int32> SurfaceT; TArray<FVector> SurfaceN; TArray<FVector2D> SurfaceUV;
            TArray<FVector> BedV; TArray<int32> BedT; TArray<FVector> BedN; TArray<FVector2D> BedUV;
            TArray<FVector> ShoreV; TArray<int32> ShoreT; TArray<FVector> ShoreN; TArray<FVector2D> ShoreUV;
            TArray<FVector> FoamV; TArray<int32> FoamT; TArray<FVector> FoamN; TArray<FVector2D> FoamUV;

            for (int32 i = 0; i < NumPts; i++)
            {
                FVector P = CachedRiverWorldPath[i] - GetActorLocation();
                FVector Tan = (i == NumPts - 1)
                    ? (CachedRiverWorldPath[i] - CachedRiverWorldPath[i - 1])
                    : (CachedRiverWorldPath[i + 1] - CachedRiverWorldPath[i]);
                Tan.Z = 0.f; Tan.Normalize();
                FVector Right = FVector(Tan.Y, -Tan.X, 0.f);

                const float Alpha = (NumPts > 1) ? (float)i / (float)(NumPts - 1) : 0.f;
                const float HalfW = GetRiverHalfWidthAt(Alpha);
                const float SurfHalfW = HalfW + RiverSurfaceEdgeOverlap;
                const float Depth = GetRiverDepthAt(FVector2D(P.X, P.Y));

                FVector L = P - Right * SurfHalfW;
                FVector R = P + Right * SurfHalfW;
                FVector BL = P - Right * HalfW; BL.Z -= Depth;
                FVector BR = P + Right * HalfW; BR.Z -= Depth;

                SurfaceV.Add(L); SurfaceV.Add(R);
                BedV.Add(BL); BedV.Add(BR);
                SurfaceN.Add(FVector::UpVector); SurfaceN.Add(FVector::UpVector);
                BedN.Add(FVector::UpVector); BedN.Add(FVector::UpVector);
                SurfaceUV.Add(FVector2D(0.f, i)); SurfaceUV.Add(FVector2D(1.f, i));
                BedUV.Add(FVector2D(0.f, i)); BedUV.Add(FVector2D(1.f, i));

                const float ShoreHalfW = SurfHalfW + RiverShoreBlendWidth;
                FVector SL = P - Right * ShoreHalfW;
                FVector SR = P + Right * ShoreHalfW;
                SL.Z = GetTerrainHeight(SL.X, SL.Y);
                SR.Z = GetTerrainHeight(SR.X, SR.Y);
                ShoreV.Add(L); ShoreV.Add(SL); ShoreV.Add(R); ShoreV.Add(SR);
                ShoreN.Add(FVector::UpVector); ShoreN.Add(FVector::UpVector);
                ShoreN.Add(FVector::UpVector); ShoreN.Add(FVector::UpVector);
                ShoreUV.Add(FVector2D(0.f, i)); ShoreUV.Add(FVector2D(1.f, i));
                ShoreUV.Add(FVector2D(0.f, i)); ShoreUV.Add(FVector2D(1.f, i));

                if (bGenerateRiverFoam)
                {
                    const float FoamHalfW = SurfHalfW + RiverFoamWidth;
                    FVector FL0 = P - Right * SurfHalfW;
                    FVector FL1 = P - Right * FoamHalfW;
                    FVector FR0 = P + Right * SurfHalfW;
                    FVector FR1 = P + Right * FoamHalfW;
                    FL0.Z += RiverFoamHeightOffset; FL1.Z += RiverFoamHeightOffset;
                    FR0.Z += RiverFoamHeightOffset; FR1.Z += RiverFoamHeightOffset;
                    FoamV.Add(FL0); FoamV.Add(FL1); FoamV.Add(FR0); FoamV.Add(FR1);
                    FoamN.Add(FVector::UpVector); FoamN.Add(FVector::UpVector);
                    FoamN.Add(FVector::UpVector); FoamN.Add(FVector::UpVector);
                    FoamUV.Add(FVector2D(0.f, i)); FoamUV.Add(FVector2D(1.f, i));
                    FoamUV.Add(FVector2D(0.f, i)); FoamUV.Add(FVector2D(1.f, i));
                }
            }

            auto AddRibbonTris = [&](int32 VertsPerRow, TArray<int32>& Tris, int32 Rows)
            {
                for (int32 i = 0; i < Rows - 1; i++)
                {
                    int32 R0 = i * VertsPerRow;
                    int32 R1 = (i + 1) * VertsPerRow;
                    for (int32 c = 0; c < VertsPerRow - 1; c++)
                    {
                        int32 BL = R0 + c, BR = R0 + c + 1, TL = R1 + c, TR = R1 + c + 1;
                        Tris.Add(BL); Tris.Add(TL); Tris.Add(BR);
                        Tris.Add(BR); Tris.Add(TL); Tris.Add(TR);
                    }
                }
            };

            AddRibbonTris(2, SurfaceT, NumPts);
            AddRibbonTris(2, BedT, NumPts);
            AddRibbonTris(2, ShoreT, NumPts * 2);

            UProceduralMeshComponent* WaterMesh = CreateMesh(TEXT("River"));
            SetMeshSection(WaterMesh, 0, SurfaceV, SurfaceT, SurfaceN, SurfaceUV, WaterMaterial ? WaterMaterial : GroundMaterial);
            WaterMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            WaterMesh->SetTranslucentSortPriority(100);

            UProceduralMeshComponent* BedMesh = CreateMesh(TEXT("RiverBed"));
            SetMeshSection(BedMesh, 0, BedV, BedT, BedN, BedUV, GroundMaterial);
            BedMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            UProceduralMeshComponent* ShoreMesh = CreateMesh(TEXT("RiverShore"));
            SetMeshSection(ShoreMesh, 0, ShoreV, ShoreT, ShoreN, ShoreUV, GroundMaterial);
            ShoreMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            if (bGenerateRiverFoam && FoamV.Num() > 3)
            {
                AddRibbonTris(2, FoamT, NumPts * 2);
                UProceduralMeshComponent* FoamMesh = CreateMesh(TEXT("RiverFoam"));
                SetMeshSection(FoamMesh, 0, FoamV, FoamT, FoamN, FoamUV, RiverFoamMaterial ? RiverFoamMaterial : WaterMaterial);
                FoamMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                FoamMesh->SetTranslucentSortPriority(110);
            }
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
//  §4  RIVER PATH
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::GenerateRiverWaypoints()
{
    River.Width = RiverWidth;
    River.ExclusionRadius = RiverExclusionRadius;
    River.Waypoints.Empty();

    // Anchor river outside the wall ring, with controlled meander toward opposite side.
    const float EntryAngle = FMath::DegreesToRadians(RiverEntryAngleDeg + Rand.FRandRange(-12.f, 12.f));
    const float ExitAngle = EntryAngle + FMath::DegreesToRadians(Rand.FRandRange(150.f, 185.f));

    const float InnerExtent = TownRadius * 1.18f;
    const float OuterExtent = TownRadius * 2.05f; // keep river visible beyond city walls

    const FVector2D EntryDir(FMath::Cos(EntryAngle), FMath::Sin(EntryAngle));
    const FVector2D ExitDir(FMath::Cos(ExitAngle), FMath::Sin(ExitAngle));

    const FVector2D OuterEntry = EntryDir * OuterExtent;
    const FVector2D Entry = EntryDir * InnerExtent;
    const FVector2D Exit  = ExitDir * InnerExtent;
    const FVector2D OuterExit = ExitDir * OuterExtent;

    // Add outside-city continuation points first/last so river persists past walls.
    River.Waypoints.Add(OuterEntry + FVector2D(Rand.FRandRange(-TownRadius * 0.03f, TownRadius * 0.03f),
                                                Rand.FRandRange(-TownRadius * 0.03f, TownRadius * 0.03f)));
    River.Waypoints.Add(Entry);

    const int32 Extra = FMath::Max(RiverWaypoints - 2, 0);
    const FVector2D MainDir = (Exit - Entry).GetSafeNormal();
    const FVector2D Perp(-MainDir.Y, MainDir.X);

    for (int32 i = 0; i < Extra; i++)
    {
        const float T = (float)(i + 1) / (float)(Extra + 1);
        FVector2D Pt = FMath::Lerp(Entry, Exit, T);

        // Multi-frequency meander creates more natural bends than pure random offsets.
        const float HarmA = FMath::Sin((T * RiverVariationFrequency + 0.11f) * TWO_PI);
        const float HarmB = 0.55f * FMath::Sin((T * (RiverVariationFrequency * 2.2f) + 0.43f) * TWO_PI);
        const float MeanderNorm = HarmA + HarmB;

        const float MeanderAmp = TownRadius * 0.24f;
        Pt += Perp * MeanderNorm * MeanderAmp;

        // Keep center plaza mostly clear.
        const float MinR = TownRadius * 0.28f;
        if (Pt.Size() < MinR)
        {
            Pt = Pt.GetSafeNormal() * MinR;
        }

        River.Waypoints.Add(Pt);
    }

    River.Waypoints.Add(Exit);
    River.Waypoints.Add(OuterExit + FVector2D(Rand.FRandRange(-TownRadius * 0.03f, TownRadius * 0.03f),
                                               Rand.FRandRange(-TownRadius * 0.03f, TownRadius * 0.03f)));
}


void AMedievalTownGenerator::BuildRiverPlanarPath()
{
    CachedRiverPlanarPath.Empty();
    if (River.Waypoints.Num() < 2) return;

    const int32 BaseSamplesPerSegment = FMath::Max(4, RiverSamplesPerSegment);

    auto Catmull = [](const FVector2D& P0, const FVector2D& P1,
                      const FVector2D& P2, const FVector2D& P3, float T)
    {
        const float T2 = T * T;
        const float T3 = T2 * T;
        return 0.5f * ((2.f * P1) +
                       (-P0 + P2) * T +
                       (2.f*P0 - 5.f*P1 + 4.f*P2 - P3) * T2 +
                       (-P0 + 3.f*P1 - 3.f*P2 + P3) * T3);
    };

    for (int32 i = 0; i < River.Waypoints.Num() - 1; i++)
    {
        const FVector2D P0 = River.Waypoints[FMath::Max(0, i - 1)];
        const FVector2D P1 = River.Waypoints[i];
        const FVector2D P2 = River.Waypoints[i + 1];
        const FVector2D P3 = River.Waypoints[FMath::Min(River.Waypoints.Num() - 1, i + 2)];

        int32 LocalSamples = BaseSamplesPerSegment;
        if (bAdaptiveRiverSampling)
        {
            const FVector2D D0 = (P1 - P0).GetSafeNormal();
            const FVector2D D1 = (P2 - P1).GetSafeNormal();
            const float Curv = 1.f - FMath::Clamp(FVector2D::DotProduct(D0, D1), -1.f, 1.f);
            LocalSamples += FMath::RoundToInt(Curv * RiverCurvatureSubdivisionBoost * 3.f);
        }
        LocalSamples = FMath::Clamp(LocalSamples, 4, 32);

        for (int32 sIdx = 0; sIdx < LocalSamples; sIdx++)
        {
            const float T = (float)sIdx / (float)LocalSamples;
            CachedRiverPlanarPath.Add(Catmull(P0, P1, P2, P3, T));
        }
    }

    CachedRiverPlanarPath.Add(River.Waypoints.Last());

    // Light smoothing pass for AAA-like spline coherence while preserving endpoints.
    for (int32 Pass = 0; Pass < RiverPlanarSmoothPasses; Pass++)
    {
        if (CachedRiverPlanarPath.Num() < 3) break;
        TArray<FVector2D> Smoothed = CachedRiverPlanarPath;
        for (int32 i = 1; i < CachedRiverPlanarPath.Num() - 1; i++)
        {
            Smoothed[i] = (CachedRiverPlanarPath[i - 1] + CachedRiverPlanarPath[i] * 2.f + CachedRiverPlanarPath[i + 1]) * 0.25f;
        }
        CachedRiverPlanarPath = MoveTemp(Smoothed);
    }
}


void AMedievalTownGenerator::BuildRiverWorldPath()
{
    // Build world-space river path directly from cached planar spline so
    // terrain carving, proximity tests, and water mesh use the same XY path.
    CachedRiverWorldPath.Empty();
    if (CachedRiverPlanarPath.Num() < 2)
        BuildRiverPlanarPath();
    if (CachedRiverPlanarPath.Num() < 2) return;

    float CumulativeDrop = 0.f;
    for (int32 i = 0; i < CachedRiverPlanarPath.Num(); i++)
    {
        const FVector2D Pt = CachedRiverPlanarPath[i];

        if (i > 0)
        {
            const float SegLen = (CachedRiverPlanarPath[i] - CachedRiverPlanarPath[i - 1]).Size();
            CumulativeDrop += (RiverDownhillPerSegment / 100.f) * (SegLen / FMath::Max(10.f, TownRadius * 0.01f));
        }

        const float BankH = GetTerrainHeightNoRiver(Pt.X, Pt.Y);
        const float WaterZ = BankH - RiverWaterSurfaceOffset - CumulativeDrop;
        CachedRiverWorldPath.Add(GetActorLocation() + FVector(Pt.X, Pt.Y, WaterZ));
    }
}


float AMedievalTownGenerator::GetRiverHalfWidthAt(float RiverAlpha01) const
{
    const float A = FMath::Clamp(RiverAlpha01, 0.f, 1.f);

    // Hydraulic-geometry-inspired variation: wider in low-energy reaches,
    // narrower in higher-energy reaches + small pseudo-noise wobble.
    const float Harmonic = FMath::Sin(A * TWO_PI * RiverVariationFrequency);
    const float Secondary = FMath::Sin((A * 3.1f + 0.37f) * TWO_PI) * 0.35f;
    float WidthScale = 1.f + RiverWidthVariation * (Harmonic + Secondary);
    WidthScale = FMath::Clamp(WidthScale, 0.65f, 1.45f);

    return RiverWidth * 0.5f * WidthScale;
}

float AMedievalTownGenerator::GetRiverFlowSpeedAt(float RiverAlpha01) const
{
    const float A = FMath::Clamp(RiverAlpha01, 0.f, 1.f);
    const float Osc = FMath::Sin((A * RiverVariationFrequency + 0.21f) * TWO_PI);
    const float Local = RiverFlowSpeedBase * (1.f + Osc * RiverFlowSpeedVariation);
    return FMath::Max(0.05f, Local);
}

bool AMedievalTownGenerator::SampleRiverClosestPoint(FVector2D Pos, float& OutDist, float& OutHalfW,
                                                      float* OutAlpha) const
{
    const TArray<FVector2D>& RiverPath2D = (CachedRiverPlanarPath.Num() >= 2) ? CachedRiverPlanarPath : River.Waypoints;
    if (RiverPath2D.Num() < 2) return false;

    float BestDist = BIG_NUMBER;
    float BestAlpha = 0.f;

    const float NumSegInv = 1.f / FMath::Max(1, RiverPath2D.Num() - 1);

    for (int32 i = 0; i < RiverPath2D.Num() - 1; i++)
    {
        const FVector2D A = RiverPath2D[i];
        const FVector2D B = RiverPath2D[i + 1];
        const FVector2D AB = B - A;
        const FVector2D AP = Pos - A;
        const float T = FMath::Clamp(FVector2D::DotProduct(AP, AB) /
                                     FMath::Max(AB.SizeSquared(), 1.f), 0.f, 1.f);
        const FVector2D C = A + AB * T;
        const float D = (Pos - C).Size();

        if (D < BestDist)
        {
            BestDist = D;
            BestAlpha = (i + T) * NumSegInv;
        }
    }

    OutDist = BestDist;
    OutHalfW = GetRiverHalfWidthAt(BestAlpha);
    if (OutAlpha) *OutAlpha = BestAlpha;
    return true;
}


float AMedievalTownGenerator::GetRiverDepthAt(FVector2D Pos) const
{
    float Dist = 0.f;
    float HalfRiver = RiverWidth * 0.5f;
    float Alpha = 0.f;
    if (!SampleRiverClosestPoint(Pos, Dist, HalfRiver, &Alpha)) return 0.f;

    const float BankWidth = FMath::Max(1.f, RiverBankFalloffWidth);

    // Meander asymmetry: outside bends are deeper than inside bends.
    const float Curv = FMath::Sin(Alpha * TWO_PI * RiverVariationFrequency + 0.7f);
    const float OutsideBendFactor = 1.f + Curv * 0.22f;
    const float LocalMaxDepth = RiverMaxDepth * OutsideBendFactor;

    if (Dist <= HalfRiver)
    {
        const float T = FMath::Clamp(Dist / FMath::Max(HalfRiver, 1.f), 0.f, 1.f);
        const float EdgeNoise = 1.f + RiverEdgeNoise * FMath::Sin((Alpha * 8.0f + T * 5.0f) * TWO_PI);
        return FMath::Lerp(LocalMaxDepth, RiverEdgeDepth, T * T) * EdgeNoise;
    }

    if (Dist < HalfRiver + BankWidth)
    {
        float BankT = (Dist - HalfRiver) / BankWidth;
        BankT = FMath::Clamp(BankT, 0.f, 1.f);
        BankT = BankT * BankT * (3.f - 2.f * BankT);
        return RiverEdgeDepth * (1.f - BankT);
    }

    return 0.f;
}

bool AMedievalTownGenerator::IsNearRiver(FVector2D Pos, float ExtraRadius) const
{
    float Dist = 0.f;
    float HalfW = RiverWidth * 0.5f;
    if (!SampleRiverClosestPoint(Pos, Dist, HalfW, nullptr)) return false;

    const float R = FMath::Max(River.ExclusionRadius, HalfW) + ExtraRadius;
    return Dist < R;
}

float AMedievalTownGenerator::DistToRiverCenter(FVector2D Pos) const
{
    float Dist = BIG_NUMBER;
    float HalfW = RiverWidth * 0.5f;
    SampleRiverClosestPoint(Pos, Dist, HalfW, nullptr);
    return Dist;
}

bool AMedievalTownGenerator::SegmentCrossesRiver(FVector2D SA, FVector2D SB) const
{
    if (River.Waypoints.Num() < 2) return false;

    const int32 Samples = 14;
    for (int32 S = 0; S <= Samples; S++)
    {
        const float T = (float)S / Samples;
        const FVector2D Pt = FMath::Lerp(SA, SB, T);

        float Dist = 0.f;
        float HalfW = RiverWidth * 0.5f;
        if (!SampleRiverClosestPoint(Pt, Dist, HalfW, nullptr)) continue;

        if (Dist < (HalfW + RiverSurfaceEdgeOverlap) * 1.15f)
            return true;
    }
    return false;
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

    auto AddFace = [&](const FVector& A, const FVector& B, const FVector& C, const FVector& D,
                       const FVector& FaceNormal)
    {
        int32 Base = RV.Num();
        RV.Add(A); RV.Add(B); RV.Add(C); RV.Add(D);
        RN.Add(FaceNormal); RN.Add(FaceNormal); RN.Add(FaceNormal); RN.Add(FaceNormal);
        RUV.Add(FVector2D(0,0)); RUV.Add(FVector2D(1,0)); RUV.Add(FVector2D(0,1)); RUV.Add(FVector2D(1,1));
        RT.Add(Base + 0); RT.Add(Base + 2); RT.Add(Base + 1);
        RT.Add(Base + 1); RT.Add(Base + 2); RT.Add(Base + 3);
    };

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

        const FVector LL0Top = LL0 + FVector(0,0,RailH);
        const FVector LR0Top = LR0 + FVector(0,0,RailH);
        const FVector LL1Top = LL1 + FVector(0,0,RailH);
        const FVector LR1Top = LR1 + FVector(0,0,RailH);

        // Left railing faces: outer, inner, top
        AddFace(LL0, LL1, LL0Top, LL1Top, -Right);
        AddFace(LR0, LR0Top, LR1, LR1Top, Right);
        AddFace(LL0Top, LL1Top, LR0Top, LR1Top, FVector::UpVector);

        // Right railing
        FVector RL0 = P0 + Right * HalfW;
        FVector RR0 = P0 + Right * (HalfW + RailThick);
        FVector RL1 = P1 + Right * HalfW;
        FVector RR1 = P1 + Right * (HalfW + RailThick);

        const FVector RL0Top = RL0 + FVector(0,0,RailH);
        const FVector RR0Top = RR0 + FVector(0,0,RailH);
        const FVector RL1Top = RL1 + FVector(0,0,RailH);
        const FVector RR1Top = RR1 + FVector(0,0,RailH);

        AddFace(RR0, RR0Top, RR1, RR1Top, Right);
        AddFace(RL0, RL1, RL0Top, RL1Top, -Right);
        AddFace(RL0Top, RL1Top, RR0Top, RR1Top, FVector::UpVector);
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
    CachedRiverPlanarPath.Empty();
    const FVector Origin = GetActorLocation();
    for (const FVector& P : CachedRiverWorldPath)
        CachedRiverPlanarPath.Add(FVector2D(P.X - Origin.X, P.Y - Origin.Y));
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
