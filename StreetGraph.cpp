#include "StreetGraph.h"

int32 FStreetGraph::AddNode(const FVector& Position, bool bAnchor)
{
    FStreetNode& Node = Nodes.AddDefaulted_GetRef();
    Node.Id = Nodes.Num() - 1;
    Node.Position = Position;
    Node.bAnchor = bAnchor;
    return Node.Id;
}

int32 FStreetGraph::AddSegment(int32 A, int32 B, EMedievalStreetType Type, float Width, float Importance)
{
    if (!Nodes.IsValidIndex(A) || !Nodes.IsValidIndex(B) || A == B)
    {
        return INDEX_NONE;
    }
    FStreetSegment& Segment = Segments.AddDefaulted_GetRef();
    Segment.A = A;
    Segment.B = B;
    Segment.Type = Type;
    Segment.Width = Width;
    Segment.Importance = Importance;
    Segment.Polyline = { Nodes[A].Position, Nodes[B].Position };
    return Segments.Num() - 1;
}

bool FStreetGraph::HasNearbyIntersection(const FVector& Position, float MinSpacing) const
{
    for (const FStreetNode& Node : Nodes)
    {
        if (FVector::DistSquared2D(Node.Position, Position) < FMath::Square(MinSpacing))
        {
            return true;
        }
    }
    return false;
}
