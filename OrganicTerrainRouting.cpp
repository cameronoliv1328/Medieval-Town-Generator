#include "OrganicTerrainRouting.h"

namespace
{
    static float PointLineDistance(const FVector2D& P, const FVector2D& A, const FVector2D& B)
    {
        const FVector2D AB = B - A;
        const float Len2 = AB.SizeSquared();
        if (Len2 < KINDA_SMALL_NUMBER)
        {
            return FVector2D::Distance(P, A);
        }
        const float T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / Len2, 0.0f, 1.0f);
        const FVector2D Closest = A + AB * T;
        return FVector2D::Distance(P, Closest);
    }

    static void RDPImpl(const TArray<FVector2D>& In, int32 Start, int32 End, float Eps, TArray<int32>& Keep)
    {
        float MaxDist = 0.0f;
        int32 MaxIdx = INDEX_NONE;
        for (int32 I = Start + 1; I < End; ++I)
        {
            const float D = PointLineDistance(In[I], In[Start], In[End]);
            if (D > MaxDist)
            {
                MaxDist = D;
                MaxIdx = I;
            }
        }

        if (MaxIdx != INDEX_NONE && MaxDist > Eps)
        {
            RDPImpl(In, Start, MaxIdx, Eps, Keep);
            Keep.Add(MaxIdx);
            RDPImpl(In, MaxIdx, End, Eps, Keep);
        }
    }
}

void FOrganicTerrainRouter::SimplifyRDP(const TArray<FVector2D>& In, float Epsilon, TArray<FVector2D>& Out)
{
    if (In.Num() <= 2)
    {
        Out = In;
        return;
    }

    TArray<int32> Keep;
    Keep.Add(0);
    RDPImpl(In, 0, In.Num() - 1, Epsilon, Keep);
    Keep.Add(In.Num() - 1);
    Keep.Sort();

    Out.Reset(Keep.Num());
    for (int32 Idx : Keep)
    {
        Out.Add(In[Idx]);
    }
}

void FOrganicTerrainRouter::SmoothPolyline(TArray<FVector2D>& InOut, int32 Iterations, float Alpha)
{
    if (InOut.Num() <= 2)
    {
        return;
    }

    const float A = FMath::Clamp(Alpha, 0.01f, 0.95f);
    for (int32 Iter = 0; Iter < Iterations; ++Iter)
    {
        TArray<FVector2D> Copy = InOut;
        for (int32 I = 1; I < InOut.Num() - 1; ++I)
        {
            const FVector2D Avg = (Copy[I - 1] + Copy[I + 1]) * 0.5f;
            InOut[I] = FMath::Lerp(Copy[I], Avg, A);
        }
    }
}
