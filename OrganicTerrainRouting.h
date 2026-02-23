#pragma once

#include "CoreMinimal.h"
#include "Algo/Reverse.h"
#include "OrganicTerrainRouting.generated.h"

USTRUCT(BlueprintType)
struct FOrganicTerrainCostParams
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float SlopePenalty = 2500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float WaterPenalty = 100000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ObstaclePenalty = 30000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ValleyBias = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxGradePrimary = 0.10f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxGradeSecondary = 0.12f;
};

class FOrganicTerrainRouter
{
public:
    template<typename HeightFunc, typename WaterTestFunc>
    static TArray<FVector2D> RouteAStar(
        const FVector2D& Start,
        const FVector2D& End,
        const FOrganicTerrainCostParams& Params,
        float CellSize,
        int32 MaxIterations,
        HeightFunc&& HeightSampler,
        WaterTestFunc&& WaterForbidden,
        bool bPrimaryRoad)
    {
        struct FNode
        {
            FIntPoint P;
            float G = BIG_NUMBER;
            float F = BIG_NUMBER;
            FIntPoint Parent = FIntPoint(MAX_int32, MAX_int32);
            bool bClosed = false;
        };

        auto ToGrid = [CellSize](const FVector2D& V) -> FIntPoint
        {
            return FIntPoint(FMath::RoundToInt(V.X / CellSize), FMath::RoundToInt(V.Y / CellSize));
        };
        auto ToWorld = [CellSize](const FIntPoint& P) -> FVector2D
        {
            return FVector2D((float)P.X * CellSize, (float)P.Y * CellSize);
        };

        const FIntPoint StartP = ToGrid(Start);
        const FIntPoint EndP = ToGrid(End);

        TMap<FIntPoint, FNode> Nodes;
        TArray<FIntPoint> Open;
        auto Heuristic = [&](const FIntPoint& A)
        {
            return FVector2D::Distance(ToWorld(A), ToWorld(EndP));
        };

        FNode& S = Nodes.FindOrAdd(StartP);
        S.P = StartP;
        S.G = 0.0f;
        S.F = Heuristic(StartP);
        Open.Add(StartP);

        const float MaxGrade = bPrimaryRoad ? Params.MaxGradePrimary : Params.MaxGradeSecondary;
        const FIntPoint Dirs[8] = {
            FIntPoint(1,0),FIntPoint(-1,0),FIntPoint(0,1),FIntPoint(0,-1),
            FIntPoint(1,1),FIntPoint(1,-1),FIntPoint(-1,1),FIntPoint(-1,-1)
        };

        for (int32 Iter = 0; Iter < MaxIterations && Open.Num() > 0; ++Iter)
        {
            int32 BestOpen = 0;
            float BestF = BIG_NUMBER;
            for (int32 I = 0; I < Open.Num(); ++I)
            {
                const float F = Nodes[Open[I]].F;
                if (F < BestF)
                {
                    BestF = F;
                    BestOpen = I;
                }
            }

            const FIntPoint CurrentP = Open[BestOpen];
            Open.RemoveAtSwap(BestOpen);
            FNode& Current = Nodes[CurrentP];
            Current.bClosed = true;
            if (CurrentP == EndP)
            {
                break;
            }

            const FVector2D CurW = ToWorld(CurrentP);
            const float CurH = HeightSampler(CurW);

            for (const FIntPoint& D : Dirs)
            {
                const FIntPoint NP = CurrentP + D;
                FVector2D NW = ToWorld(NP);

                if (WaterForbidden(NW))
                {
                    continue;
                }

                FNode& N = Nodes.FindOrAdd(NP);
                N.P = NP;
                if (N.bClosed)
                {
                    continue;
                }

                const float NH = HeightSampler(NW);
                const float Dist = FVector2D::Distance(CurW, NW);
                const float Grade = FMath::Abs(NH - CurH) / FMath::Max(1.0f, Dist);
                const float SlopeCost = (Grade > MaxGrade) ? Params.SlopePenalty * (Grade / MaxGrade) : Params.SlopePenalty * Grade;
                const float ValleyCost = Params.ValleyBias * NH;
                const float GNew = Current.G + Dist + SlopeCost + ValleyCost;

                if (GNew < N.G)
                {
                    N.G = GNew;
                    N.F = GNew + Heuristic(NP);
                    N.Parent = CurrentP;
                    Open.AddUnique(NP);
                }
            }
        }

        TArray<FVector2D> Path;
        if (!Nodes.Contains(EndP))
        {
            Path = { Start, End };
            return Path;
        }

        FIntPoint P = EndP;
        int32 Guard = 0;
        while (Guard++ < 65536)
        {
            Path.Add(ToWorld(P));
            if (P == StartP)
            {
                break;
            }
            const FNode* Node = Nodes.Find(P);
            if (!Node || Node->Parent == FIntPoint(MAX_int32, MAX_int32))
            {
                break;
            }
            P = Node->Parent;
        }
        Algo::Reverse(Path);
        Path[0] = Start;
        Path.Last() = End;
        return Path;
    }

    static void SimplifyRDP(const TArray<FVector2D>& In, float Epsilon, TArray<FVector2D>& Out);
    static void SmoothPolyline(TArray<FVector2D>& InOut, int32 Iterations, float Alpha);
};
