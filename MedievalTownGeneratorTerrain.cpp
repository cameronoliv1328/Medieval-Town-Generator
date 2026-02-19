// Terrain module extracted from MedievalTownGenerator.cpp
#include "MedievalTownGeneratorTerrain.h"
#include "MedievalTownGenerator.h"

// ─────────────────────────────────────────────────────────────────────────────
//  §3  TERRAIN
// ─────────────────────────────────────────────────────────────────────────────

// Deterministic hash-based noise (no external dependency)
float MTGTerrain::HashNoise(int32 X, int32 Y)
{
    int32 N = X + Y * 57;
    N = (N << 13) ^ N;
    return 1.f - ((N * (N * N * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.f;
}

float MTGTerrain::SmoothNoise(float X, float Y)
{
    int32 IX = (int32)FMath::FloorToInt(X);
    int32 IY = (int32)FMath::FloorToInt(Y);
    float FX = X - IX, FY = Y - IY;
    float UX = FX * FX * (3.f - 2.f * FX);
    float UY = FY * FY * (3.f - 2.f * FY);

    float V00 = MTGTerrain::HashNoise(IX,   IY);
    float V10 = MTGTerrain::HashNoise(IX+1, IY);
    float V01 = MTGTerrain::HashNoise(IX,   IY+1);
    float V11 = MTGTerrain::HashNoise(IX+1, IY+1);

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
        Value += MTGTerrain::SmoothNoise(X * F, Y * F) * A;
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

    // 4. Carve river channel with a smooth thalweg + bank profile
    if (River.Waypoints.Num() >= 2)
    {
        FVector2D Pos(X, Y);
        // River carve depth from shared river cross-section helper
        H -= GetRiverDepthAt(Pos);
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
