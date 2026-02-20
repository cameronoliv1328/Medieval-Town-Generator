#pragma once

#include "CoreMinimal.h"

// Forward declarations
class AMedievalTownGenerator;
class UProceduralMeshComponent;

/**
 * River mesh generation utilities.
 *
 * Key design decisions for gap-free rendering:
 *   1. River surface extends slightly PAST nominal width (overlap skirt).
 *   2. Shore blend strips sample terrain heights at EXACT vertex XY positions
 *      so there is zero vertical mismatch with terrain mesh.
 *   3. Multiple blend rings (inner wet → outer dry) provide smooth transition.
 *   4. Vertex colors encode: R=flow speed, G=foam mask, B=bank blend, A=depth.
 *   5. UV2 (packed into tangent channel) encodes flow direction for material.
 */
namespace MTGRiver
{
    /** Per-sample data computed along the river centerline. */
    struct FRiverCrossSectionSample
    {
        FVector  WorldPos;       // Center point in world space
        FVector  Tangent;        // Normalized flow direction (Z=0)
        FVector  Right;          // Perpendicular to tangent (Z=0)
        float    HalfWidth;      // Half width at this sample
        float    Depth;          // Max depth at centerline
        float    FlowSpeed;      // Relative flow speed
        float    CumulativeV;    // Cumulative V coordinate for UV tiling
        float    Alpha;          // 0..1 parameter along river length
        float    BankHeightL;    // Terrain height at left bank edge (no river carve)
        float    BankHeightR;    // Terrain height at right bank edge (no river carve)
    };

    /**
     * Build cross-section samples from the cached river world path.
     * This is the single source of truth used by all river meshes.
     */
    void BuildCrossSectionSamples(
        AMedievalTownGenerator* Gen,
        TArray<FRiverCrossSectionSample>& OutSamples);

    /**
     * Generate the water surface ribbon mesh.
     * Extends slightly past nominal width to hide edge precision issues.
     * Vertex colors encode flow data for the water material.
     */
    void GenerateWaterSurface(
        AMedievalTownGenerator* Gen,
        const TArray<FRiverCrossSectionSample>& Samples,
        TArray<FVector>& OutV, TArray<int32>& OutT,
        TArray<FVector>& OutN, TArray<FVector2D>& OutUV,
        TArray<FColor>& OutColors,
        TArray<FProcMeshTangent>& OutTangents);

    /**
     * Generate the riverbed mesh (submerged below water surface).
     * Uses a U-shaped cross section for realistic bed profile.
     */
    void GenerateRiverbed(
        AMedievalTownGenerator* Gen,
        const TArray<FRiverCrossSectionSample>& Samples,
        TArray<FVector>& OutV, TArray<int32>& OutT,
        TArray<FVector>& OutN, TArray<FVector2D>& OutUV,
        TArray<FColor>& OutColors);

    /**
     * Generate multi-ring shore blend strips that stitch water edge to terrain.
     * Each ring samples terrain height at its exact XY position — NO GAPS.
     *
     * Rings from inside out:
     *   Ring 0: Water edge → wet bank (darkened/wet terrain)
     *   Ring 1: Wet bank → dry bank (transition)
     *   Ring 2: Dry bank → untouched terrain (feather out)
     *
     * Both left and right banks are generated.
     */
    void GenerateShoreBlend(
        AMedievalTownGenerator* Gen,
        const TArray<FRiverCrossSectionSample>& Samples,
        TArray<FVector>& OutV, TArray<int32>& OutT,
        TArray<FVector>& OutN, TArray<FVector2D>& OutUV,
        TArray<FColor>& OutColors);

    /**
     * Generate foam strips along water edges.
     * Foam is wider where flow speed is higher or curvature increases.
     */
    void GenerateFoamStrips(
        AMedievalTownGenerator* Gen,
        const TArray<FRiverCrossSectionSample>& Samples,
        TArray<FVector>& OutV, TArray<int32>& OutT,
        TArray<FVector>& OutN, TArray<FVector2D>& OutUV,
        TArray<FColor>& OutColors);

    /**
     * Compute curvature at a sample index (used for adaptive foam/depth).
     * Returns 0..1 where 1 = maximum curvature (sharp bend).
     */
    float ComputeCurvature(
        const TArray<FRiverCrossSectionSample>& Samples,
        int32 Index);
}
