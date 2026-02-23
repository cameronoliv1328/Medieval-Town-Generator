#include "OrganicIntersectionMesher.h"

bool FOrganicIntersectionMesher::BuildIntersectionPolygon(
    const FStreetNode& Node,
    const TArray<FStreetNode>& Nodes,
    const TArray<FStreetEdge>& Edges,
    TArray<FVector>& OutPolygon,
    float PlazaScale)
{
    OutPolygon.Reset();
    if (Node.ConnectedEdges.Num() < 3)
    {
        return false;
    }

    struct FCorner
    {
        float Angle = 0.0f;
        FVector Pos = FVector::ZeroVector;
    };

    TArray<FCorner> Corners;
    for (int32 EdgeIdx : Node.ConnectedEdges)
    {
        if (!Edges.IsValidIndex(EdgeIdx)) continue;
        const FStreetEdge& E = Edges[EdgeIdx];
        if (!Nodes.IsValidIndex(E.A) || !Nodes.IsValidIndex(E.B)) continue;

        const bool bNodeAtA = (E.A != INDEX_NONE && Nodes[E.A].Position.Equals(Node.Position, 1.0f));
        const FVector Other = bNodeAtA ? Nodes[E.B].Position : Nodes[E.A].Position;
        FVector Dir = (Other - Node.Position).GetSafeNormal2D();
        if (Dir.IsNearlyZero())
        {
            continue;
        }

        const FVector Side(-Dir.Y, Dir.X, 0.0f);
        const float R = FMath::Max(80.0f, E.Width * 0.75f * PlazaScale);
        const float A = FMath::Atan2(Dir.Y, Dir.X);

        FCorner Left;
        Left.Angle = A - 0.15f;
        Left.Pos = Node.Position + (Dir * R + Side * (E.Width * 0.45f));
        Corners.Add(Left);

        FCorner Right;
        Right.Angle = A + 0.15f;
        Right.Pos = Node.Position + (Dir * R - Side * (E.Width * 0.45f));
        Corners.Add(Right);
    }

    if (Corners.Num() < 3)
    {
        return false;
    }

    Corners.Sort([](const FCorner& A, const FCorner& B)
    {
        return A.Angle < B.Angle;
    });

    for (const FCorner& C : Corners)
    {
        OutPolygon.Add(FVector(C.Pos.X, C.Pos.Y, Node.Position.Z));
    }

    return OutPolygon.Num() >= 3;
}

void FOrganicIntersectionMesher::BuildRoadRibbon(
    const FStreetEdge& Edge,
    float WidthScale,
    float VertexSpacing,
    TArray<FVector>& OutV,
    TArray<int32>& OutT,
    TArray<FVector>& OutN,
    TArray<FVector2D>& OutUV)
{
    if (Edge.PolylinePoints.Num() < 2)
    {
        return;
    }

    float RunDist = 0.0f;
    const int32 Base = OutV.Num();

    for (int32 I = 0; I < Edge.PolylinePoints.Num(); ++I)
    {
        const FVector P = Edge.PolylinePoints[I];
        const FVector Prev = (I > 0) ? Edge.PolylinePoints[I - 1] : P;
        const FVector Next = (I + 1 < Edge.PolylinePoints.Num()) ? Edge.PolylinePoints[I + 1] : P;

        const FVector Dir = (Next - Prev).GetSafeNormal2D();
        const FVector Side(-Dir.Y, Dir.X, 0.0f);
        if (I > 0)
        {
            RunDist += FVector::Dist2D(Prev, P);
        }

        const float N = FMath::PerlinNoise1D(RunDist * 0.001f + 1.37f);
        const float W = Edge.Width * FMath::Lerp(0.9f, 1.1f, (N + 1.0f) * 0.5f) * WidthScale;
        const float HalfW = W * 0.5f;

        OutV.Add(P - Side * HalfW);
        OutV.Add(P + Side * HalfW);
        OutN.Add(FVector::UpVector);
        OutN.Add(FVector::UpVector);
        OutUV.Add(FVector2D(0.0f, RunDist / FMath::Max(50.0f, VertexSpacing)));
        OutUV.Add(FVector2D(1.0f, RunDist / FMath::Max(50.0f, VertexSpacing)));

        if (I > 0)
        {
            const int32 BL = Base + (I - 1) * 2;
            const int32 BR = BL + 1;
            const int32 TL = Base + I * 2;
            const int32 TR = TL + 1;
            OutT.Add(BL); OutT.Add(TL); OutT.Add(BR);
            OutT.Add(BR); OutT.Add(TL); OutT.Add(TR);
        }
    }
}
