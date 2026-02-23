// StreetDebugRenderer.h
// ─────────────────────────────────────────────────────────────────────────────
// Debug visualization helpers for the organic street graph.
// Call from BeginPlay or editor-only paths; zero overhead in shipping builds.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once
#include "CoreMinimal.h"
#include "OrganicStreetGraph.h"

class UWorld;
struct FBridgeCandidate;

namespace StreetDebug
{
    struct FDebugSettings
    {
        bool  bShowNodes      = true;
        bool  bShowEdges      = true;
        bool  bShowImportance = true;
        bool  bShowBridges    = true;
        float NodeRadius      = 80.f;
        float LifeTime        = 60.f;
        float ActorZ          = 0.f;
    };

    /** Draw the full street graph: colour-coded edges + typed node spheres */
    void DrawGraph(UWorld* World, const FOrganicStreetGraph& Graph,
                   const FDebugSettings& Settings);

    /** Draw bridge candidates as cyan spheres with approach direction arrows */
    void DrawBridgeCandidates(UWorld* World,
                               const TArray<FBridgeCandidate>& Candidates,
                               float ActorZ, float LifeTime = 60.f);

    /** Draw river exclusion zone as translucent blue band along river path */
    void DrawForbiddenRiverZone(UWorld* World, const TArray<FVector2D>& RiverPath,
                                 float HalfWidth, float ActorZ, float LifeTime = 60.f);
}
