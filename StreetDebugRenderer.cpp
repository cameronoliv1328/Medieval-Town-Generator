// StreetDebugRenderer.cpp
#include "StreetDebugRenderer.h"
#include "OrganicStreetGenerator.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

// -- Edge colour by tier -------------------------------------------------------
static FColor EdgeColor(EOrganicStreetType T)
{
    switch (T)
    {
    case EOrganicStreetType::Primary:   return FColor(255, 200,  60);  // yellow
    case EOrganicStreetType::Secondary: return FColor(160, 210, 255);  // blue
    case EOrganicStreetType::Lane:      return FColor(180, 255, 180);  // green
    case EOrganicStreetType::Alley:     return FColor(200, 180, 140);  // tan
    }
    return FColor::White;
}

void StreetDebug::DrawGraph(UWorld* World, const FOrganicStreetGraph& Graph,
                             const FDebugSettings& S)
{
    if (!World) return;
    const float Z = S.ActorZ + 120.f;

    if (S.bShowEdges)
    {
        for (const FOrganicStreetEdge& E : Graph.Edges)
        {
            if (E.NodeA < 0) continue;
            FColor Col  = EdgeColor(E.StreetType);
            if (E.bIsBridge) Col = FColor::Cyan;
            float Thick = (E.StreetType == EOrganicStreetType::Primary) ? 4.f : 2.f;
            for (int32 i = 0; i < E.Poly2D.Num() - 1; i++)
            {
                DrawDebugLine(World,
                    FVector(E.Poly2D[i].X,   E.Poly2D[i].Y,   Z),
                    FVector(E.Poly2D[i+1].X, E.Poly2D[i+1].Y, Z),
                    Col, false, S.LifeTime, 0, Thick);
            }
        }
    }

    if (S.bShowNodes)
    {
        for (const FOrganicStreetNode& N : Graph.Nodes)
        {
            FColor Col = FColor::White;
            if (N.bIsMarket)          Col = FColor::Yellow;
            else if (N.bIsGate)       Col = FColor::Orange;
            else if (N.bIsBridgeNode) Col = FColor::Cyan;
            else if (N.bIsLandmark)   Col = FColor::Purple;
            else if (N.bIsPlaza)      Col = FColor::Green;

            float R = S.NodeRadius;
            if (N.bIsMarket || N.bIsGate) R *= 2.f;

            DrawDebugSphere(World,
                FVector(N.Position.X, N.Position.Y, Z),
                R, 6, Col, false, S.LifeTime);

            if (S.bShowImportance)
            {
                DrawDebugLine(World,
                    FVector(N.Position.X, N.Position.Y, Z),
                    FVector(N.Position.X, N.Position.Y, Z + N.Importance * 800.f),
                    FColor::White, false, S.LifeTime, 0, 1.f);
            }
        }
    }
}

void StreetDebug::DrawBridgeCandidates(UWorld* World,
                                        const TArray<FBridgeCandidate>& Candidates,
                                        float ActorZ, float LifeTime)
{
    if (!World) return;
    for (const FBridgeCandidate& B : Candidates)
    {
        const float Z = ActorZ + 200.f;
        DrawDebugSphere(World, FVector(B.Position.X, B.Position.Y, Z),
                        300.f, 8, FColor::Cyan, false, LifeTime);
        const FVector Dir3(B.ApproachDir.X, B.ApproachDir.Y, 0.f);
        DrawDebugDirectionalArrow(World,
            FVector(B.Position.X, B.Position.Y, Z),
            FVector(B.Position.X, B.Position.Y, Z) + Dir3 * 800.f,
            200.f, FColor::Cyan, false, LifeTime, 0, 2.f);
    }
}

void StreetDebug::DrawForbiddenRiverZone(UWorld* World,
                                          const TArray<FVector2D>& RiverPath,
                                          float HalfWidth, float ActorZ, float LifeTime)
{
    if (!World || RiverPath.Num() < 2) return;
    const float Z = ActorZ + 50.f;
    for (int32 i = 0; i < RiverPath.Num() - 1; i++)
    {
        FVector2D A = RiverPath[i], B = RiverPath[i+1];
        FVector2D Dir  = (B - A).GetSafeNormal();
        FVector2D Perp(-Dir.Y, Dir.X);
        FVector AL(A.X - Perp.X * HalfWidth, A.Y - Perp.Y * HalfWidth, Z);
        FVector AR(A.X + Perp.X * HalfWidth, A.Y + Perp.Y * HalfWidth, Z);
        FVector BL(B.X - Perp.X * HalfWidth, B.Y - Perp.Y * HalfWidth, Z);
        FVector BR(B.X + Perp.X * HalfWidth, B.Y + Perp.Y * HalfWidth, Z);
        DrawDebugLine(World, AL, BL, FColor(0, 80, 255, 120), false, LifeTime, 0, 1.f);
        DrawDebugLine(World, AR, BR, FColor(0, 80, 255, 120), false, LifeTime, 0, 1.f);
    }
}
