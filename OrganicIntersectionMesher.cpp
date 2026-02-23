// OrganicIntersectionMesher.cpp
#include "OrganicIntersectionMesher.h"

// -----------------------------------------------------------------------------
//  BuildIntersectionPolygon
// -----------------------------------------------------------------------------

bool FOrganicIntersectionMesher::BuildIntersectionPolygon(
    const FOrganicStreetNode&         Node,       // Bug 1 fix: FOrganicStreetNode
    int32                             NodeIdx,    // Bug 5 fix: explicit index
    const TArray<FOrganicStreetNode>& Nodes,      // Bug 1 fix
    const TArray<FOrganicStreetEdge>& Edges,      // Bug 2 fix
    TArray<FVector>&                  OutPolygon,
    float                             PlazaScale)
{
    OutPolygon.Reset();
    if (Node.ConnectedEdges.Num() < 3)
    {
        return false;
    }

    // Bug 6 fix: Node.Position is FVector2D -- pre-convert once for 3D operations.
    const FVector NodePos3D(Node.Position.X, Node.Position.Y, 0.f);

    struct FCorner
    {
        float   Angle = 0.0f;
        FVector Pos   = FVector::ZeroVector;
    };

    TArray<FCorner> Corners;
    for (int32 EdgeIdx : Node.ConnectedEdges)
    {
        if (!Edges.IsValidIndex(EdgeIdx)) continue;
        const FOrganicStreetEdge& E = Edges[EdgeIdx];  // Bug 2 fix

        // Bug 4 fix: use NodeA / NodeB (was E.A / E.B)
        if (!Nodes.IsValidIndex(E.NodeA) || !Nodes.IsValidIndex(E.NodeB)) continue;

        // Bug 5 fix: compare by index -- not by position (fragile, breaks on coincident nodes,
        //            and Position is now FVector2D so FVector::Equals() wouldn't compile).
        const bool    bNodeAtA  = (E.NodeA == NodeIdx);
        const int32   OtherIdx  = bNodeAtA ? E.NodeB : E.NodeA;

        // Bug 6 fix: Nodes[x].Position is FVector2D -- compute direction in 2D then lift to 3D.
        const FVector2D OtherPos2D = Nodes[OtherIdx].Position;
        const FVector2D DirXY      = (OtherPos2D - Node.Position).GetSafeNormal(); // Bug 6+9 fix
        if (DirXY.IsNearlyZero()) continue;

        const FVector Dir(DirXY.X, DirXY.Y, 0.f);
        const FVector Side(-Dir.Y, Dir.X, 0.f);
        const float   R = FMath::Max(80.f, E.Width * 0.75f * PlazaScale);
        const float   A = FMath::Atan2(Dir.Y, Dir.X);

        FCorner Left;
        Left.Angle = A - 0.15f;
        Left.Pos   = NodePos3D + (Dir * R + Side * (E.Width * 0.45f));  // Bug 6 fix
        Corners.Add(Left);

        FCorner Right;
        Right.Angle = A + 0.15f;
        Right.Pos   = NodePos3D + (Dir * R - Side * (E.Width * 0.45f)); // Bug 6 fix
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
        // Bug 7 fix: Node.Position is FVector2D (no .Z) -- use 0.f as floor height.
        OutPolygon.Add(FVector(C.Pos.X, C.Pos.Y, 0.f));
    }

    return OutPolygon.Num() >= 3;
}

// -----------------------------------------------------------------------------
//  BuildRoadRibbon
// -----------------------------------------------------------------------------

void FOrganicIntersectionMesher::BuildRoadRibbon(
    const FOrganicStreetEdge& Edge,   // Bug 2 fix: FOrganicStreetEdge
    float                     WidthScale,
    float                     VertexSpacing,
    TArray<FVector>&          OutV,
    TArray<int32>&            OutT,
    TArray<FVector>&          OutN,
    TArray<FVector2D>&        OutUV)
{
    // Bug 8 fix: field is now Poly2D (TArray<FVector2D>), not PolylinePoints (TArray<FVector>)
    if (Edge.Poly2D.Num() < 2)
    {
        return;
    }

    float        RunDist = 0.0f;
    const int32  Base    = OutV.Num();

    for (int32 I = 0; I < Edge.Poly2D.Num(); ++I)
    {
        // Bug 8 fix: Poly2D elements are FVector2D -- fetch in 2D, lift to FVector for mesh.
        const FVector2D P2D    = Edge.Poly2D[I];
        const FVector2D Prev2D = (I > 0)                        ? Edge.Poly2D[I - 1] : P2D;
        const FVector2D Next2D = (I + 1 < Edge.Poly2D.Num())    ? Edge.Poly2D[I + 1] : P2D;

        // Bug 9 fix: arithmetic on FVector2D -- use GetSafeNormal() not GetSafeNormal2D().
        const FVector2D DirXY = (Next2D - Prev2D).GetSafeNormal();
        const FVector   Dir(DirXY.X, DirXY.Y, 0.f);
        const FVector   Side(-Dir.Y, Dir.X, 0.f);

        // Bug 10 fix: FVector2D::Distance(), not FVector::Dist2D().
        if (I > 0)
        {
            RunDist += FVector2D::Distance(Prev2D, P2D);
        }

        const float N      = FMath::PerlinNoise1D(RunDist * 0.001f + 1.37f);
        const float W      = Edge.Width * FMath::Lerp(0.9f, 1.1f, (N + 1.0f) * 0.5f) * WidthScale;
        const float HalfW  = W * 0.5f;

        // Lift 2D polyline point to 3D (Z filled later by ElevateRoadSplines).
        const FVector P(P2D.X, P2D.Y, 0.f);

        OutV.Add(P - Side * HalfW);
        OutV.Add(P + Side * HalfW);
        OutN.Add(FVector::UpVector);
        OutN.Add(FVector::UpVector);

        const float UVV = RunDist / FMath::Max(50.0f, VertexSpacing);
        OutUV.Add(FVector2D(0.f, UVV));
        OutUV.Add(FVector2D(1.f, UVV));

        if (I > 0)
        {
            const int32 BL = Base + (I - 1) * 2;
            const int32 BR = BL + 1;
            const int32 TL = Base + I * 2;
            const int32 TR = TL + 1;
            // CCW winding (view from above / +Z normal)
            OutT.Add(BL); OutT.Add(TL); OutT.Add(BR);
            OutT.Add(BR); OutT.Add(TL); OutT.Add(TR);
        }
    }
}
