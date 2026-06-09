// HarnessMain.cxx
// -----------------------------------------------------------------------------
// Standalone layout harness: compiles the engine-agnostic generator sources
// against the UE shim (Harness/Shim) and renders a top-down image of the
// resulting street network, parcels, and buildings so layout changes can be
// iterated on and visually verified without launching Unreal.
//
// Build + run:   Harness/build.sh [seed] [outprefix]
//
// The terrain, river, gate, and landmark setup below intentionally mirrors
// AMedievalTownGenerator::BuildOrganicRoadNetwork() so the harness exercises
// the same code paths the in-engine generator does.
// -----------------------------------------------------------------------------
#include "CoreMinimal.h"

#include "../OrganicStreetGraph.cpp"
#include "../OrganicStreetGenerator.cpp"
#include "../OrganicParcelGenerator.cpp"

#include <cstdio>
#include <cstring>
#include <string>

// =============================================================================
//  Synthetic world (mirrors the UE generator's terrain + river behaviour)
// =============================================================================

struct FHarnessWorld
{
    float TownRadius = 18000.f;
    float RiverWidth = 550.f;
    float RiverExclusion = 800.f;
    float TerrainAmplitude = 800.f;
    float TerrainFrequency = 0.00015f;
    float TownFlattenStrength = 0.85f;
    int32 Seed = 1337;

    TArray<FVector2D> RiverPath;

    void BuildRiver(FRandomStream& Rand)
    {
        // Meandering river crossing the town, like BuildRiverPlanarPath():
        // straight chord through the town with sinus + noise lateral offsets.
        RiverPath.Empty();
        const float Angle = Rand.FRandRange(0.f, TWO_PI);
        const FVector2D Dir(FMath::Cos(Angle), FMath::Sin(Angle));
        const FVector2D Perp(-Dir.Y, Dir.X);
        const FVector2D Offset = Perp * Rand.FRandRange(-0.25f, 0.25f) * TownRadius;
        const int32 N = 64;
        for (int32 i = 0; i <= N; i++)
        {
            const float T = (float)i / (float)N;
            const float Along = (T - 0.5f) * TownRadius * 2.6f;
            float Lateral = FMath::Sin(T * PI * 2.2f + Rand.GetCurrentSeed() * 0.001f) * TownRadius * 0.10f;
            Lateral += FMath::PerlinNoise2D(FVector2D(T * 7.3f, (float)Seed * 0.13f)) * TownRadius * 0.06f;
            RiverPath.Add(Dir * Along + Perp * Lateral + Offset);
        }
    }

    float DistToRiver(FVector2D P) const
    {
        float Best = BIG_NUMBER;
        for (int32 i = 0; i < RiverPath.Num() - 1; i++)
        {
            const FVector2D A = RiverPath[i], B = RiverPath[i + 1];
            const FVector2D AB = B - A;
            const float L2 = AB.SizeSquared();
            const float T = (L2 > 0.f) ? FMath::Clamp(FVector2D::DotProduct(P - A, AB) / L2, 0.f, 1.f) : 0.f;
            const float D = (P - (A + AB * T)).Size();
            if (D < Best) Best = D;
        }
        return Best;
    }

    bool IsNearRiver(FVector2D P, float Extra) const
    {
        return DistToRiver(P) < RiverExclusion + Extra;
    }

    float GetHeightNoRiver(FVector2D P) const
    {
        float H = FMath::PerlinNoise2D(P * TerrainFrequency + FVector2D((float)Seed * 0.01f, 0.f)) * TerrainAmplitude;
        H += FMath::PerlinNoise2D(P * TerrainFrequency * 3.1f + FVector2D(11.7f, (float)Seed * 0.02f)) * TerrainAmplitude * 0.35f;
        // Flatten inside town like TownFlattenStrength does.
        const float R = P.Size();
        const float Flatten = FMath::Clamp(1.f - R / TownRadius, 0.f, 1.f) * TownFlattenStrength;
        return H * (1.f - Flatten);
    }

    float GetHeight(FVector2D P) const
    {
        float H = GetHeightNoRiver(P);
        const float DR = DistToRiver(P);
        if (DR < RiverWidth)
            H -= (1.f - DR / RiverWidth) * 220.f;  // river channel carve
        return H;
    }
};

// =============================================================================
//  Simple RGB framebuffer renderer
// =============================================================================

struct FCanvas
{
    int32 W = 1600, H = 1600;
    float WorldHalf = 20000.f;   // world units mapped to half the image
    std::vector<uint8> Px;

    void Init(int32 InW, int32 InH, float InWorldHalf)
    {
        W = InW; H = InH; WorldHalf = InWorldHalf;
        Px.assign((size_t)W * H * 3, 24);
    }
    FVector2D WorldToPx(FVector2D P) const
    {
        return FVector2D((P.X / WorldHalf * 0.5f + 0.5f) * W,
                         (P.Y / WorldHalf * 0.5f + 0.5f) * H);
    }
    float PxPerWorld() const { return (float)W / (WorldHalf * 2.f); }

    void Put(int32 X, int32 Y, uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        if (X < 0 || X >= W || Y < 0 || Y >= H) return;
        size_t I = ((size_t)Y * W + X) * 3;
        Px[I]     = (uint8)(Px[I]     * (1.f - Alpha) + R * Alpha);
        Px[I + 1] = (uint8)(Px[I + 1] * (1.f - Alpha) + G * Alpha);
        Px[I + 2] = (uint8)(Px[I + 2] * (1.f - Alpha) + B * Alpha);
    }

    void Disc(FVector2D World, float WorldRad, uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        const FVector2D C = WorldToPx(World);
        const float PR = FMath::Max(1.f, WorldRad * PxPerWorld());
        for (int32 Y = (int32)(C.Y - PR) - 1; Y <= (int32)(C.Y + PR) + 1; Y++)
            for (int32 X = (int32)(C.X - PR) - 1; X <= (int32)(C.X + PR) + 1; X++)
                if (FMath::Square(X + 0.5f - C.X) + FMath::Square(Y + 0.5f - C.Y) <= PR * PR)
                    Put(X, Y, R, G, B, Alpha);
    }

    void Segment(FVector2D A, FVector2D B, float WorldWidth, uint8 R, uint8 G, uint8 Bl, float Alpha = 1.f)
    {
        const FVector2D PA = WorldToPx(A), PB = WorldToPx(B);
        const float HalfW = FMath::Max(0.5f, WorldWidth * PxPerWorld() * 0.5f);
        const int32 MinX = (int32)FMath::Min(PA.X, PB.X) - (int32)HalfW - 1;
        const int32 MaxX = (int32)FMath::Max(PA.X, PB.X) + (int32)HalfW + 1;
        const int32 MinY = (int32)FMath::Min(PA.Y, PB.Y) - (int32)HalfW - 1;
        const int32 MaxY = (int32)FMath::Max(PA.Y, PB.Y) + (int32)HalfW + 1;
        const FVector2D AB = PB - PA;
        const float L2 = FMath::Max(AB.SizeSquared(), 1.e-4f);
        for (int32 Y = MinY; Y <= MaxY; Y++)
            for (int32 X = MinX; X <= MaxX; X++)
            {
                const FVector2D P((float)X + 0.5f, (float)Y + 0.5f);
                const float T = FMath::Clamp(FVector2D::DotProduct(P - PA, AB) / L2, 0.f, 1.f);
                const float D = (P - (PA + AB * T)).Size();
                if (D <= HalfW) Put(X, Y, R, G, Bl, Alpha);
            }
    }

    void Polyline(const TArray<FVector2D>& Pts, float WorldWidth, uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        for (int32 i = 0; i < Pts.Num() - 1; i++)
            Segment(Pts[i], Pts[i + 1], WorldWidth, R, G, B, Alpha);
    }

    // Filled convex quad (worlds-space corners, any winding)
    void Quad(const FVector2D Corner[4], uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        FVector2D P[4];
        float MinX = 1e9f, MaxX = -1e9f, MinY = 1e9f, MaxY = -1e9f;
        for (int32 i = 0; i < 4; i++)
        {
            P[i] = WorldToPx(Corner[i]);
            MinX = FMath::Min(MinX, P[i].X); MaxX = FMath::Max(MaxX, P[i].X);
            MinY = FMath::Min(MinY, P[i].Y); MaxY = FMath::Max(MaxY, P[i].Y);
        }
        // Determine winding from signed area
        float Area = 0.f;
        for (int32 i = 0; i < 4; i++)
        {
            const FVector2D& A = P[i];
            const FVector2D& B = P[(i + 1) & 3];
            Area += A.X * B.Y - B.X * A.Y;
        }
        const float Sign = (Area >= 0.f) ? 1.f : -1.f;
        for (int32 Y = (int32)MinY - 1; Y <= (int32)MaxY + 1; Y++)
            for (int32 X = (int32)MinX - 1; X <= (int32)MaxX + 1; X++)
            {
                const FVector2D Q((float)X + 0.5f, (float)Y + 0.5f);
                bool bIn = true;
                for (int32 i = 0; i < 4 && bIn; i++)
                {
                    const FVector2D& A = P[i];
                    const FVector2D& B = P[(i + 1) & 3];
                    if (Sign * ((B.X - A.X) * (Q.Y - A.Y) - (B.Y - A.Y) * (Q.X - A.X)) < 0.f)
                        bIn = false;
                }
                if (bIn) Put(X, Y, R, G, B, Alpha);
            }
    }

    void QuadOutline(const FVector2D Corner[4], float WorldWidth, uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        for (int32 i = 0; i < 4; i++)
            Segment(Corner[i], Corner[(i + 1) & 3], WorldWidth, R, G, B, Alpha);
    }

    void Circle(FVector2D World, float WorldRad, float WorldWidth, uint8 R, uint8 G, uint8 B, float Alpha = 1.f)
    {
        const int32 Steps = 160;
        for (int32 i = 0; i < Steps; i++)
        {
            const float A0 = (float)i / Steps * TWO_PI;
            const float A1 = (float)(i + 1) / Steps * TWO_PI;
            Segment(World + FVector2D(FMath::Cos(A0), FMath::Sin(A0)) * WorldRad,
                    World + FVector2D(FMath::Cos(A1), FMath::Sin(A1)) * WorldRad,
                    WorldWidth, R, G, B, Alpha);
        }
    }

    bool SavePPM(const char* Path) const
    {
        FILE* F = std::fopen(Path, "wb");
        if (!F) return false;
        std::fprintf(F, "P6\n%d %d\n255\n", W, H);
        std::fwrite(Px.data(), 1, Px.size(), F);
        std::fclose(F);
        return true;
    }
};

// =============================================================================
//  Scene setup mirroring BuildOrganicRoadNetwork()
// =============================================================================

int main(int argc, char** argv)
{
    const int32 Seed = (argc > 1) ? std::atoi(argv[1]) : 1337;
    const std::string OutPrefix = (argc > 2) ? argv[2] : "/tmp/town";

    FHarnessWorld World;
    World.Seed = Seed;
    FRandomStream Rand(Seed);
    World.BuildRiver(Rand);

    const float TownRadius = World.TownRadius;

    // -- Gates (mirrors fallback path in BuildOrganicRoadNetwork) --------------
    TArray<FVector2D> Gates;
    {
        const int32 NumG = 4;
        float BaseAngle = Rand.FRandRange(0.f, 360.f);
        float AngleStep = 360.f / NumG;
        for (int32 g = 0; g < NumG; g++)
        {
            float Angle = BaseAngle + g * AngleStep + Rand.FRandRange(-AngleStep * 0.3f, AngleStep * 0.3f);
            float R = TownRadius * Rand.FRandRange(0.85f, 0.95f);
            float RadA = FMath::DegreesToRadians(Angle);
            FVector2D GP(FMath::Cos(RadA) * R, FMath::Sin(RadA) * R);
            // Don't drop a gate into the river
            if (World.IsNearRiver(GP, 200.f)) { Angle += AngleStep * 0.45f; RadA = FMath::DegreesToRadians(Angle); GP = FVector2D(FMath::Cos(RadA) * R, FMath::Sin(RadA) * R); }
            Gates.Add(GP);
        }
    }

    // -- Bridge candidates ------------------------------------------------------
    TArray<FBridgeCandidate> BridgeCandidates;
    {
        const int32 NumSegs = World.RiverPath.Num() - 1;
        const int32 Stride = FMath::Max(1, NumSegs / 12);
        for (int32 i = Stride; i < NumSegs - Stride; i += Stride)
        {
            FVector2D Pos = (World.RiverPath[i] + World.RiverPath[i + 1]) * 0.5f;
            if (Pos.Size() > TownRadius * 0.85f) continue;
            FVector2D Flow = (World.RiverPath[i + 1] - World.RiverPath[i]).GetSafeNormal();
            FVector2D CrossDir(-Flow.Y, Flow.X);
            float HL = World.GetHeightNoRiver(Pos - CrossDir * World.RiverWidth);
            float HR = World.GetHeightNoRiver(Pos + CrossDir * World.RiverWidth);
            float Curv = 0.f;
            if (i > 0 && i < NumSegs - 1)
            {
                FVector2D D0 = (World.RiverPath[i] - World.RiverPath[i - 1]).GetSafeNormal();
                FVector2D D1 = (World.RiverPath[i + 1] - World.RiverPath[i]).GetSafeNormal();
                Curv = (1.f - FMath::Clamp(FVector2D::DotProduct(D0, D1), -1.f, 1.f)) * 0.5f;
            }
            float SlopePen = FMath::Abs(HL - HR) / (World.RiverWidth * 2.f);
            FBridgeCandidate BC;
            BC.Position = Pos;
            BC.Quality = FMath::Clamp(1.f - SlopePen * 3.f - Curv * 2.f, 0.f, 1.f);
            // Prefer central crossings (stand-in for market-anchor preference)
            BC.Quality *= FMath::Clamp(1.f - Pos.Size() / (TownRadius * 1.4f), 0.f, 1.f) + 0.5f;
            BC.ApproachDir = CrossDir;
            BridgeCandidates.Add(BC);
        }
    }

    // -- Market = best bridge (mirrors CachedMarketPos logic) --------------------
    FVector2D MarketPos = FVector2D::ZeroVector;
    {
        float BestQ = -1.f;
        for (const FBridgeCandidate& C : BridgeCandidates)
            if (C.Quality > BestQ) { BestQ = C.Quality; MarketPos = C.Position; }
    }

    // -- Church at river bend; keep opposite (simplified mirror) -----------------
    FVector2D ChurchPos = FVector2D(TownRadius * 0.2f, TownRadius * 0.1f);
    {
        float BestCurv = -1.f;
        for (int32 i = 1; i < World.RiverPath.Num() - 1; i++)
        {
            if (World.RiverPath[i].Size() > TownRadius * 0.75f) continue;
            FVector2D D0 = (World.RiverPath[i] - World.RiverPath[i - 1]).GetSafeNormal();
            FVector2D D1 = (World.RiverPath[i + 1] - World.RiverPath[i]).GetSafeNormal();
            float Curv = 1.f - FMath::Clamp(FVector2D::DotProduct(D0, D1), -1.f, 1.f);
            if (Curv > BestCurv)
            {
                BestCurv = Curv;
                FVector2D Bend = World.RiverPath[i];
                FVector2D Out = Bend.GetSafeNormal();
                FVector2D Cand = Bend + Out * (World.RiverExclusion + 250.f);
                if (Cand.Size() > TownRadius * 0.55f) Cand = Cand.GetSafeNormal() * TownRadius * 0.45f;
                if (!World.IsNearRiver(Cand, 200.f)) ChurchPos = Cand;
            }
        }
    }
    FVector2D KeepPos;
    {
        FVector2D ToBridge = MarketPos.GetSafeNormal();
        FVector2D ChurchDir = ChurchPos.GetSafeNormal();
        FVector2D KeepDir = (FVector2D::DotProduct(ToBridge, ChurchDir) > 0.f) ? -ToBridge : ToBridge;
        KeepPos = MarketPos + KeepDir * TownRadius * 0.22f;
        if (KeepPos.Size() > TownRadius * 0.5f) KeepPos = KeepPos.GetSafeNormal() * TownRadius * 0.42f;
    }

    // -- Terrain query + config (mirrors steps 5-6) ------------------------------
    FOrganicTerrainQuery TQ;
    TQ.GetHeight = [&World](FVector2D P) { return World.GetHeight(P); };
    TQ.IsNearRiver = [&World](FVector2D P, float E) { return World.IsNearRiver(P, E); };
    TQ.BridgeSuitability = [&World](FVector2D P) {
        return FMath::Clamp(1.f - World.DistToRiver(P) / (World.RiverWidth * 1.5f), 0.f, 1.f);
    };
    TQ.MaxGrade = 14.f / 90.f;
    TQ.SlopePenalty = 2.4f;
    TQ.WaterPenalty = 5.0f;
    TQ.ValleyPreference = 0.2f;

    FOrganicStreetConfig Cfg;
    Cfg.TownRadius = TownRadius;
    Cfg.MarketCenter = MarketPos;
    Cfg.PrimaryWidthMin = 700.f * 0.85f;  Cfg.PrimaryWidthMax = 700.f * 1.15f;
    Cfg.SecondaryWidthMin = 440.f * 0.85f; Cfg.SecondaryWidthMax = 440.f * 1.15f;
    Cfg.LaneWidthMin = 280.f * 0.75f;     Cfg.LaneWidthMax = 280.f * 1.25f;
    Cfg.AlleyWidthMin = 280.f * 0.40f;    Cfg.AlleyWidthMax = 280.f * 0.70f;
    Cfg.MaxBridges = FMath::Clamp(FMath::RoundToInt(TownRadius / 12000.f) + 1, 1, 2);
    Cfg.AStarCellSize = TownRadius / 80.f;
    Cfg.RDPEpsilonPrimary = Cfg.AStarCellSize * 0.55f;
    Cfg.RDPEpsilonSecondary = Cfg.AStarCellSize * 0.4f;

    // -- Generate streets ---------------------------------------------------------
    FOrganicStreetGenerator Generator(Cfg, TQ, Rand);
    FOrganicStreetGraph Graph = Generator.Generate(Gates, BridgeCandidates, ChurchPos, KeepPos);

    int32 LiveEdges = 0;
    for (const FOrganicStreetEdge& E : Graph.Edges) if (E.NodeA >= 0) LiveEdges++;
    std::printf("streets: %d nodes, %d edges\n", Graph.Nodes.Num(), LiveEdges);

    // -- Generate parcels -----------------------------------------------------------
    FOrganicParcelConfig PCfg;
    PCfg.TownRadius = TownRadius;
    PCfg.MarketCenter = Generator.GetPlazaCenter();
    // Exclusion disc must sit INSIDE the market ring so houses can enclose it.
    PCfg.MarketPlazaRadius = Cfg.PlazaRadius * 0.70f;
    PCfg.ChurchPos = ChurchPos;
    PCfg.KeepPos = KeepPos;

    FOrganicParcelTerrainQuery PTQ;
    PTQ.GetHeight = TQ.GetHeight;
    PTQ.IsNearRiver = TQ.IsNearRiver;
    PTQ.DistToRiver = [&World](FVector2D P) { return World.DistToRiver(P); };

    FOrganicParcelGenerator ParcelGen(PCfg, PTQ, Rand);
    TArray<FOrganicParcelLot> Lots = ParcelGen.Generate(Graph);
    std::printf("parcels: %d building lots\n", Lots.Num());

    // ============================== RENDER ========================================
    FCanvas Canvas;
    Canvas.Init(2200, 2200, TownRadius * 1.12f);

    // Terrain shading
    for (int32 Y = 0; Y < Canvas.H; Y++)
        for (int32 X = 0; X < Canvas.W; X++)
        {
            const FVector2D P((((float)X + 0.5f) / Canvas.W - 0.5f) * 2.f * Canvas.WorldHalf,
                              (((float)Y + 0.5f) / Canvas.H - 0.5f) * 2.f * Canvas.WorldHalf);
            const float Hgt = World.GetHeightNoRiver(P);
            const float Shade = FMath::Clamp(0.5f + Hgt / (World.TerrainAmplitude * 2.5f), 0.f, 1.f);
            uint8 G = (uint8)(78 + Shade * 70);
            uint8 R = (uint8)(58 + Shade * 60);
            uint8 B = (uint8)(40 + Shade * 48);
            Canvas.Put(X, Y, R, G, B, 1.f);
        }

    // River
    Canvas.Polyline(World.RiverPath, World.RiverWidth * 2.f, 38, 76, 122, 0.85f);
    Canvas.Polyline(World.RiverPath, World.RiverWidth, 52, 105, 165, 1.f);

    // Wall ring
    Canvas.Circle(FVector2D::ZeroVector, TownRadius * 0.95f, 220.f, 96, 92, 86, 0.9f);

    // Parcels (plots first, light outline)
    for (const FOrganicParcelLot& L : Lots)
    {
        if (L.PlotPolygon.Num() == 4)
        {
            FVector2D C[4] = { L.PlotPolygon[0], L.PlotPolygon[1], L.PlotPolygon[2], L.PlotPolygon[3] };
            Canvas.QuadOutline(C, 30.f, 168, 158, 128, 0.35f);
        }
    }

    // Streets by tier (under buildings)
    for (const FOrganicStreetEdge& E : Graph.Edges)
    {
        if (E.NodeA < 0 || E.Poly2D.Num() < 2) continue;
        uint8 R = 150, G = 140, B = 120;
        switch (E.StreetType)
        {
        case EOrganicStreetType::Primary:   R = 214; G = 190; B = 142; break;
        case EOrganicStreetType::Secondary: R = 188; G = 170; B = 132; break;
        case EOrganicStreetType::Lane:      R = 166; G = 152; B = 122; break;
        case EOrganicStreetType::Alley:     R = 142; G = 132; B = 110; break;
        }
        if (E.bIsBridge) { R = 230; G = 226; B = 214; }
        Canvas.Polyline(E.Poly2D, FMath::Max(E.Width, 180.f), R, G, B, 1.f);
    }

    // Buildings (filled rects coloured by kind)
    for (const FOrganicParcelLot& L : Lots)
    {
        const float Yaw = FMath::DegreesToRadians(L.YawDeg);
        const FVector2D AX(FMath::Cos(Yaw), FMath::Sin(Yaw));            // local +X
        const FVector2D AY(-FMath::Sin(Yaw), FMath::Cos(Yaw));           // local +Y
        const FVector2D HX = AX * (L.Footprint.X * 0.5f);
        const FVector2D HY = AY * (L.Footprint.Y * 0.5f);
        FVector2D C[4] = { L.Center - HX - HY, L.Center + HX - HY,
                           L.Center + HX + HY, L.Center - HX + HY };
        uint8 R = 172, G = 96, B = 64;        // default: townhouse brick
        switch (L.Kind)
        {
        case EOrganicLotKind::Cottage:     R = 150; G = 110; B = 72;  break;
        case EOrganicLotKind::TownHouse:   R = 176; G = 98;  B = 62;  break;
        case EOrganicLotKind::GuildHall:   R = 196; G = 140; B = 90;  break;
        case EOrganicLotKind::Tavern:      R = 188; G = 120; B = 70;  break;
        case EOrganicLotKind::Warehouse:   R = 130; G = 112; B = 96;  break;
        case EOrganicLotKind::Blacksmith:  R = 110; G = 96;  B = 92;  break;
        case EOrganicLotKind::Bakery:      R = 198; G = 150; B = 96;  break;
        case EOrganicLotKind::Stable:      R = 136; G = 118; B = 80;  break;
        case EOrganicLotKind::Outbuilding: R = 128; G = 112; B = 78;  break;
        case EOrganicLotKind::Church:      R = 214; G = 206; B = 188; break;
        case EOrganicLotKind::Keep:        R = 168; G = 164; B = 158; break;
        }
        Canvas.Quad(C, R, G, B, 1.f);
        Canvas.QuadOutline(C, 22.f, (uint8)(R / 2), (uint8)(G / 2), (uint8)(B / 2), 0.8f);
    }

    // Occupancy grid overlay (debug): road=tan, water=blue, plaza=yellow,
    // claimed=green tint, outside=dark
    const bool bDebugOverlay = (argc > 3 && std::strcmp(argv[3], "debug") == 0);
    if (bDebugOverlay)
    {
        const TArray<uint8>& Gr = ParcelGen.DebugGrid();
        const int32 N = ParcelGen.DebugGridN();
        const float GH = ParcelGen.DebugGridHalf();
        const float CellSz = GH * 2.f / N;
        for (int32 cy = 0; cy < N; cy++)
            for (int32 cx = 0; cx < N; cx++)
            {
                const uint8 C = Gr[cy * N + cx];
                if (C == 0) continue;
                const FVector2D P(-GH + (cx + 0.5f) * CellSz, -GH + (cy + 0.5f) * CellSz);
                uint8 R=0,Gc=0,B=0; float A=0.25f;
                switch (C) {
                case 1: R=255; Gc=230; B=140; break;   // road
                case 2: R=60;  Gc=80;  B=255; break;   // water
                case 3: continue;                       // outside
                case 4: R=255; Gc=255; B=0;   break;   // plaza
                case 5: R=0;   Gc=255; B=120; break;   // claimed
                }
                Canvas.Disc(P, CellSz*0.5f, R, Gc, B, A);
            }
    }

    // Depth-reject diagnostics
    extern TArray<FVector2D> GParcelDepthRejectPositions;
    if (bDebugOverlay)
        for (const FVector2D& P : GParcelDepthRejectPositions)
            Canvas.Disc(P, 90.f, 255, 0, 255, 0.9f);

    // Landmarks + gates + market
    std::printf("plaza center: %.0f %.0f\n", Generator.GetPlazaCenter().X, Generator.GetPlazaCenter().Y);
    Canvas.Disc(Generator.GetPlazaCenter(), 220.f, 250, 220, 90, 0.9f);
    Canvas.Disc(ChurchPos, 260.f, 240, 240, 240, 0.9f);
    Canvas.Disc(KeepPos, 280.f, 120, 120, 130, 0.9f);
    for (const FVector2D& G : Gates) Canvas.Disc(G, 260.f, 210, 60, 50, 0.95f);

    const std::string Out = OutPrefix + ".ppm";
    if (!Canvas.SavePPM(Out.c_str())) { std::fprintf(stderr, "failed to write %s\n", Out.c_str()); return 1; }
    std::printf("wrote %s\n", Out.c_str());
    return 0;
}
