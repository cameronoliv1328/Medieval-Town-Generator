#include "PolygonUtils.h"

namespace MedievalPolygonUtils
{
    static bool SegmentsIntersect(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D)
    {
        auto CCW = [](const FVector2D& P1, const FVector2D& P2, const FVector2D& P3)
        {
            return (P3.Y - P1.Y) * (P2.X - P1.X) > (P2.Y - P1.Y) * (P3.X - P1.X);
        };
        return CCW(A, C, D) != CCW(B, C, D) && CCW(A, B, C) != CCW(A, B, D);
    }

    bool IsSimplePolygon(const TArray<FVector2D>& Poly)
    {
        if (Poly.Num() < 3) return false;
        for (int32 I = 0; I < Poly.Num(); ++I)
        {
            const int32 INext = (I + 1) % Poly.Num();
            for (int32 J = I + 1; J < Poly.Num(); ++J)
            {
                const int32 JNext = (J + 1) % Poly.Num();
                if (I == J || I == JNext || INext == J) continue;
                if (SegmentsIntersect(Poly[I], Poly[INext], Poly[J], Poly[JNext]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    float PolygonArea(const TArray<FVector2D>& Poly)
    {
        float Area = 0.f;
        for (int32 I = 0; I < Poly.Num(); ++I)
        {
            const FVector2D A = Poly[I];
            const FVector2D B = Poly[(I + 1) % Poly.Num()];
            Area += A.X * B.Y - B.X * A.Y;
        }
        return FMath::Abs(Area * 0.5f);
    }

    TArray<FVector2D> JitterPolygon(const TArray<FVector2D>& Poly, FRandomStream& Rand, float Amount)
    {
        TArray<FVector2D> Out = Poly;
        for (FVector2D& V : Out)
        {
            V += FVector2D(Rand.FRandRange(-Amount, Amount), Rand.FRandRange(-Amount, Amount));
        }
        if (!IsSimplePolygon(Out))
        {
            return Poly;
        }
        return Out;
    }

    FString EncodePolygon(const TArray<FVector2D>& Poly)
    {
        FString Encoded;
        for (int32 I = 0; I < Poly.Num(); ++I)
        {
            Encoded += FString::Printf(TEXT("%.1f,%.1f"), Poly[I].X, Poly[I].Y);
            if (I + 1 < Poly.Num())
            {
                Encoded += TEXT(";");
            }
        }
        return Encoded;
    }

    TArray<FVector2D> MakeRectangle(const FVector2D& Center, const FVector2D& Size, float YawDeg)
    {
        const float Rad = FMath::DegreesToRadians(YawDeg);
        const FVector2D X(FMath::Cos(Rad), FMath::Sin(Rad));
        const FVector2D Y(-X.Y, X.X);
        const FVector2D HX = X * (Size.X * 0.5f);
        const FVector2D HY = Y * (Size.Y * 0.5f);
        return { Center - HX - HY, Center + HX - HY, Center + HX + HY, Center - HX + HY };
    }
}
