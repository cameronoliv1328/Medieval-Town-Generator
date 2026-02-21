// =============================================================================
// MedievalTownGenerator_RiverUpgrade.cpp
// =============================================================================
//
// This file contains the CHANGED/NEW functions for VERSION 19.
// Apply these changes to your existing MedievalTownGenerator.cpp.
//
// Summary of changes:
//   1. Phase2_SetupTerrain calls GenerateAdaptiveTerrainMesh() instead of
//      inline terrain mesh code (when bUseImprovedRiverMeshes is true).
//   2. Phase7_SpawnMeshes calls SpawnImprovedRiverMeshes() instead of
//      inline river mesh code (when bUseImprovedRiverMeshes is true).
//   3. New SpawnImprovedRiverMeshes() uses the MTGRiver:: helpers.
//
// =============================================================================

#include "MedievalTownGenerator.h"
#include "MedievalTownGeneratorRiver.h"

// ─────────────────────────────────────────────────────────────────────────────
//  REPLACE: Phase2_SetupTerrain()
//  Changes: Use adaptive terrain mesh when improved river is enabled.
// ─────────────────────────────────────────────────────────────────────────────

// Removed duplicate Phase2_SetupTerrain implementation (kept primary version in MedievalTownGenerator.cpp).


// ─────────────────────────────────────────────────────────────────────────────
//  REPLACE: River section in Phase7_SpawnMeshes()
//
//  The key change: replace the inline river mesh code with a call to
//  SpawnImprovedRiverMeshes(). Keep the rest of Phase7 unchanged.
// ─────────────────────────────────────────────────────────────────────────────

// In Phase7_SpawnMeshes(), replace the entire block:
//
//   // ── River ────────────────────────────────────────────
//   if (bGenerateRiver && CachedRiverWorldPath.Num() >= 2)
//   {
//       ... (all the V18 river mesh code) ...
//   }
//
// With:
//
//   // ── River ────────────────────────────────────────────
//   if (bGenerateRiver && CachedRiverWorldPath.Num() >= 2)
//   {
//       if (bUseImprovedRiverMeshes)
//       {
//           SpawnImprovedRiverMeshes();
//       }
//       else
//       {
//           // ... keep V18 inline code as fallback ...
//       }
//   }


// ─────────────────────────────────────────────────────────────────────────────
//  NEW: SpawnImprovedRiverMeshes()
//
//  This is the new V19 river mesh pipeline. It uses MTGRiver:: helpers
//  to generate four mesh layers from shared cross-section data.
// ─────────────────────────────────────────────────────────────────────────────

void AMedievalTownGenerator::SpawnImprovedRiverMeshes()
{
    // ── Step 1: Build cross-section samples (shared by all meshes) ────────
    TArray<MTGRiver::FRiverCrossSectionSample> Samples;
    MTGRiver::BuildCrossSectionSamples(this, Samples);

    if (Samples.Num() < 2)
    {
        UE_LOG(LogTemp, Warning, TEXT("[MTG] River: too few samples (%d)"), Samples.Num());
        return;
    }

    // ── Step 2: Shore blend strips (MUST be rendered first/underneath) ────
    //
    //  These strips stitch the water edge to the terrain surface.
    //  They sample GetTerrainHeight at exact vertex positions → zero gaps.
    //  The ground material is applied so they blend seamlessly with terrain.
    {
        TArray<FVector> V; TArray<int32> T; TArray<FVector> N;
        TArray<FVector2D> UV; TArray<FColor> Colors;
        MTGRiver::GenerateShoreBlend(this, Samples, V, T, N, UV, Colors);

        if (V.Num() > 3)
        {
            UProceduralMeshComponent* ShoreMesh = CreateMesh(TEXT("RiverShore"));
            TArray<FProcMeshTangent> Tangents;  // Empty — not needed for opaque shore
            ShoreMesh->CreateMeshSection(0, V, T, N, UV, Colors, Tangents, true);

            // Use ground material with vertex color for wetness blending
            // If you have a dedicated shore material, use that instead.
            UMaterialInterface* ShoreMat = GroundMaterial ? GroundMaterial : StoneMaterial;
            ShoreMesh->SetMaterial(0, ShoreMat);
            ShoreMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            UE_LOG(LogTemp, Log, TEXT("[MTG] River shore: %d verts, %d tris"),
                   V.Num(), T.Num() / 3);
        }
    }

    // ── Step 3: Riverbed (opaque, submerged) ──────────────────────────────
    {
        TArray<FVector> V; TArray<int32> T; TArray<FVector> N;
        TArray<FVector2D> UV; TArray<FColor> Colors;
        MTGRiver::GenerateRiverbed(this, Samples, V, T, N, UV, Colors);

        if (V.Num() > 3)
        {
            UProceduralMeshComponent* BedMesh = CreateMesh(TEXT("RiverBed"));
            TArray<FProcMeshTangent> Tangents;
            BedMesh->CreateMeshSection(0, V, T, N, UV, Colors, Tangents, true);

            UMaterialInterface* BedMat = GroundMaterial ? GroundMaterial : StoneMaterial;
            BedMesh->SetMaterial(0, BedMat);
            BedMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            UE_LOG(LogTemp, Log, TEXT("[MTG] River bed: %d verts"), V.Num());
        }
    }

    // ── Step 4: Water surface (translucent, on top) ───────────────────────
    {
        TArray<FVector> V; TArray<int32> T; TArray<FVector> N;
        TArray<FVector2D> UV; TArray<FColor> Colors;
        TArray<FProcMeshTangent> Tangents;
        MTGRiver::GenerateWaterSurface(this, Samples, V, T, N, UV, Colors, Tangents);

        if (V.Num() > 3)
        {
            UProceduralMeshComponent* WaterMesh = CreateMesh(TEXT("River"));
            WaterMesh->CreateMeshSection(0, V, T, N, UV, Colors, Tangents, true);

            if (WaterMaterial)
                WaterMesh->SetMaterial(0, WaterMaterial);
            WaterMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            // Translucent sort priority: render after opaque terrain/bed
            WaterMesh->SetTranslucentSortPriority(100);

            UE_LOG(LogTemp, Log, TEXT("[MTG] River surface: %d verts"), V.Num());
        }
    }

    // ── Step 5: Foam strips (additive, on top of water) ───────────────────
    if (bGenerateRiverFoam)
    {
        TArray<FVector> V; TArray<int32> T; TArray<FVector> N;
        TArray<FVector2D> UV; TArray<FColor> Colors;
        MTGRiver::GenerateFoamStrips(this, Samples, V, T, N, UV, Colors);

        if (V.Num() > 3)
        {
            UProceduralMeshComponent* FoamMesh = CreateMesh(TEXT("RiverFoam"));
            TArray<FProcMeshTangent> Tangents;
            FoamMesh->CreateMeshSection(0, V, T, N, UV, Colors, Tangents, true);

            UMaterialInterface* FoamMat = RiverFoamMaterial ? RiverFoamMaterial : WaterMaterial;
            FoamMesh->SetMaterial(0, FoamMat);
            FoamMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            FoamMesh->SetTranslucentSortPriority(110);

            UE_LOG(LogTemp, Log, TEXT("[MTG] River foam: %d verts"), V.Num());
        }
    }
}
