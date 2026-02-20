// =============================================================================
// MedievalTownGenerator_RiverUpgrade.h
// =============================================================================
//
// This file documents ALL additions to MedievalTownGenerator.h for VERSION 19.
// Apply these changes to your existing header.
//
// Changes are organized by section (where to add them in the existing header).
// =============================================================================

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  ADD TO: #include section (top of .h file)
// ─────────────────────────────────────────────────────────────────────────────
// #include "MedievalTownGeneratorRiver.h"   // Add this include


// ─────────────────────────────────────────────────────────────────────────────
//  ADD TO: UPROPERTY section under "Town | River" category
//  (after existing river properties)
// ─────────────────────────────────────────────────────────────────────────────

/*

    // ===== RIVER (NEW V19 — Adaptive terrain + shore blend) =====

    // Cell size for terrain mesh near river corridor.
    // Smaller = more terrain vertices near river = better channel representation.
    // Range: 30-500. Typical: 80-150 for AAA quality.
    // If this produces too many vertices (>512x512), it caps automatically.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "30", UIMax = "500"))
    float RiverAdaptiveTerrainCellSize = 100.f;

    // Number of concentric shore blend rings per bank side.
    // More rings = smoother shore transition but more triangles.
    // 3-5 is typical for production quality.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "2", UIMax = "8"))
    int32 RiverShoreBlendRings = 4;

    // When true, uses the new gap-free river mesh generation pipeline.
    // Disable to revert to V18 behavior for comparison.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River")
    bool bUseImprovedRiverMeshes = true;

    // Extra submersion offset: terrain directly under water is pushed down
    // by this amount to guarantee NO z-fighting between terrain and water.
    // Only affects vertices within the water surface footprint.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Town | River",
        meta = (UIMin = "1", UIMax = "30"))
    float RiverTerrainSubmersionBias = 5.f;

*/


// ─────────────────────────────────────────────────────────────────────────────
//  ADD TO: Private method declarations (after existing river methods)
// ─────────────────────────────────────────────────────────────────────────────

/*

    // ─── River (V19 improved mesh pipeline) ──────────────────────────────
    void SpawnImprovedRiverMeshes();
    void GenerateAdaptiveTerrainMesh();  // Replaces terrain part of Phase2

    // Make GetRiverHalfWidthAt and GetRiverFlowSpeedAt accessible to
    // the MTGRiver namespace (they're already public in the const interface,
    // but the river helper needs them). If they're currently private,
    // move them to public or add friend:
    friend void MTGRiver::BuildCrossSectionSamples(
        AMedievalTownGenerator*, TArray<MTGRiver::FRiverCrossSectionSample>&);

*/


// ─────────────────────────────────────────────────────────────────────────────
//  MODIFY: GetRiverWorldPath() — make non-const accessible for river helper
//  OR keep const and use the friend declaration above.
//  The easiest approach: just keep it const (already is).
// ─────────────────────────────────────────────────────────────────────────────
