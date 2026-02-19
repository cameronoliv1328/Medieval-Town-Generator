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
        const float HalfW = RiverWidth * 0.5f + RiverSurfaceEdgeOverlap;

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
    // Use local bank height (terrain without carve), then offset slightly downward
    // so the water sits inside the carved channel and avoids visible seam gaps.
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
            // Water surface sits slightly below the local bank height
            CachedRiverWorldPath.Add(GetActorLocation() + FVector(Pt.X, Pt.Y, BankH - RiverWaterSurfaceOffset));
        }
    }
    // Add final point
    FVector2D Last = River.Waypoints.Last();
    float BankH = GetTerrainHeightNoRiver(Last.X, Last.Y);
    CachedRiverWorldPath.Add(GetActorLocation() + FVector(Last.X, Last.Y, BankH - RiverWaterSurfaceOffset));
}

float AMedievalTownGenerator::GetRiverDepthAt(FVector2D Pos) const
{
    if (River.Waypoints.Num() < 2) return 0.f;

    const float D = DistToRiverCenter(Pos);
    const float HalfRiver = RiverWidth * 0.5f;
    const float BankWidth = FMath::Max(1.f, RiverBankFalloffWidth);

    if (D <= HalfRiver)
    {
        const float T = FMath::Clamp(D / FMath::Max(HalfRiver, 1.f), 0.f, 1.f);
        return FMath::Lerp(RiverMaxDepth, RiverEdgeDepth, T * T);
    }

    if (D < HalfRiver + BankWidth)
    {
        float BankT = (D - HalfRiver) / BankWidth;
        BankT = FMath::Clamp(BankT, 0.f, 1.f);
        BankT = BankT * BankT * (3.f - 2.f * BankT);
        return RiverEdgeDepth * (1.f - BankT);
    }

    return 0.f;
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
    const float HalfW = RiverWidth * 0.5f + RiverSurfaceEdgeOverlap;
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
