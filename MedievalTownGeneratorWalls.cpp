// Walls module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGenerator.h"
#include "MedievalTownGeneratorWalls.h"

// ─────────────────────────────────────────────────────────────────────────────
//  §6  SHAPE GRAMMAR WALLS
// ─────────────────────────────────────────────────────────────────────────────

EWallModule MTGWalls::ModuleFromToken(const FString& Token)
{
    if (Token == TEXT("W")) return EWallModule::WallSection;
    if (Token == TEXT("T")) return EWallModule::CornerTower;
    if (Token == TEXT("G")) return EWallModule::GateTower;
    if (Token == TEXT("P")) return EWallModule::Parapet;
    if (Token == TEXT("B")) return EWallModule::Buttress;
    return EWallModule::WallSection;
}

void AMedievalTownGenerator::ParseGrammar(const FString& Grammar, TArray<EWallModule>& Out)
{
    Out.Empty();
    TArray<FString> Tokens;
    Grammar.ParseIntoArrayWS(Tokens);
    for (const FString& Token : Tokens)
    {
        Out.Add(MTGWalls::ModuleFromToken(Token));
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

