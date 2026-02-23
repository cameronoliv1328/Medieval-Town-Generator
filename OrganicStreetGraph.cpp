#include "OrganicStreetGraph.h"

FOrganicStreetSpatialHash::FOrganicStreetSpatialHash(float InCell)
    : Cell(FMath::Max(100.0f, InCell))
{
}

FIntPoint FOrganicStreetSpatialHash::ToCell(const FVector2D& P) const
{
    return FIntPoint(FMath::FloorToInt(P.X / Cell), FMath::FloorToInt(P.Y / Cell));
}

void FOrganicStreetSpatialHash::Insert(const FVector2D& P, int32 Idx)
{
    Buckets.FindOrAdd(ToCell(P)).Add(Idx);
}

void FOrganicStreetSpatialHash::Query(const FVector2D& P, float Radius, TArray<int32>& OutIndices) const
{
    const int32 Rings = FMath::CeilToInt(FMath::Max(1.0f, Radius) / Cell);
    const FIntPoint C = ToCell(P);
    for (int32 Y = -Rings; Y <= Rings; ++Y)
    {
        for (int32 X = -Rings; X <= Rings; ++X)
        {
            if (const TArray<int32>* Found = Buckets.Find(FIntPoint(C.X + X, C.Y + Y)))
            {
                OutIndices.Append(*Found);
            }
        }
    }
}

namespace
{
    static float Cross2D(const FVector2D& A, const FVector2D& B)
    {
        return A.X * B.Y - A.Y * B.X;
    }
}

int32 OrganicStreetGraphUtils::FindNearestNode(const FOrganicStreetGraph& Graph, const FVector& Position, float* OutDistance)
{
    float BestD2 = TNumericLimits<float>::Max();
    int32 Best = INDEX_NONE;
    for (int32 I = 0; I < Graph.Nodes.Num(); ++I)
    {
        const float D2 = FVector::DistSquared2D(Position, Graph.Nodes[I].Position);
        if (D2 < BestD2)
        {
            BestD2 = D2;
            Best = I;
        }
    }

    if (OutDistance)
    {
        *OutDistance = (Best == INDEX_NONE) ? BIG_NUMBER : FMath::Sqrt(BestD2);
    }
    return Best;
}

bool OrganicStreetGraphUtils::SegmentIntersection2D(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D)
{
    const FVector2D R = B - A;
    const FVector2D S = D - C;
    const float Den = Cross2D(R, S);
    if (FMath::IsNearlyZero(Den))
    {
        return false;
    }

    const FVector2D CA = C - A;
    const float T = Cross2D(CA, S) / Den;
    const float U = Cross2D(CA, R) / Den;
    return T > 0.001f && T < 0.999f && U > 0.001f && U < 0.999f;
}

bool OrganicStreetGraphUtils::EdgeIntersectsExisting(const FOrganicStreetGraph& Graph, const FVector2D& A, const FVector2D& B, int32 IgnoreNodeA, int32 IgnoreNodeB)
{
    for (const FStreetEdge& E : Graph.Edges)
    {
        if (E.A == IgnoreNodeA || E.B == IgnoreNodeA || E.A == IgnoreNodeB || E.B == IgnoreNodeB)
        {
            continue;
        }

        if (!Graph.Nodes.IsValidIndex(E.A) || !Graph.Nodes.IsValidIndex(E.B))
        {
            continue;
        }

        const FVector2D C(Graph.Nodes[E.A].Position.X, Graph.Nodes[E.A].Position.Y);
        const FVector2D D(Graph.Nodes[E.B].Position.X, Graph.Nodes[E.B].Position.Y);
        if (SegmentIntersection2D(A, B, C, D))
        {
            return true;
        }
    }
    return false;
}

void OrganicStreetGraphUtils::BuildNodeAdjacency(const FOrganicStreetGraph& Graph, TArray<TArray<int32>>& OutAdjacency)
{
    OutAdjacency.SetNum(Graph.Nodes.Num());
    for (const FStreetEdge& Edge : Graph.Edges)
    {
        if (!Graph.Nodes.IsValidIndex(Edge.A) || !Graph.Nodes.IsValidIndex(Edge.B))
        {
            continue;
        }
        OutAdjacency[Edge.A].AddUnique(Edge.B);
        OutAdjacency[Edge.B].AddUnique(Edge.A);
    }
}

void OrganicStreetGraphUtils::ExtractApproxBlocks(const FOrganicStreetGraph& Graph, TArray<TArray<int32>>& OutBlocks, int32 MaxBlockEdges)
{
    OutBlocks.Reset();
    TArray<TArray<int32>> Adjacency;
    BuildNodeAdjacency(Graph, Adjacency);

    // Lightweight cycle probe for triangles/quads/pentagons used for debug/block hints.
    for (int32 Start = 0; Start < Graph.Nodes.Num(); ++Start)
    {
        for (int32 N1 : Adjacency[Start])
        {
            if (N1 <= Start) continue;
            for (int32 N2 : Adjacency[N1])
            {
                if (N2 == Start || N2 <= Start) continue;

                TArray<int32> Path = { Start, N1, N2 };
                int32 Current = N2;

                for (int32 Step = 0; Step < MaxBlockEdges - 2; ++Step)
                {
                    bool bClosed = false;
                    for (int32 Next : Adjacency[Current])
                    {
                        if (Next == Start && Path.Num() >= 3)
                        {
                            OutBlocks.Add(Path);
                            bClosed = true;
                            break;
                        }
                    }
                    if (bClosed) break;

                    int32 Candidate = INDEX_NONE;
                    for (int32 Next : Adjacency[Current])
                    {
                        if (!Path.Contains(Next))
                        {
                            Candidate = Next;
                            break;
                        }
                    }
                    if (Candidate == INDEX_NONE) break;

                    Path.Add(Candidate);
                    Current = Candidate;
                }
            }
        }
    }
}
