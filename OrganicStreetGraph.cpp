// OrganicStreetGraph.cpp
#include "OrganicStreetGraph.h"

int32 FOrganicStreetGraph::AddNode(FVector2D Pos, bool)
{
    FOrganicStreetNode& N = Nodes.AddDefaulted_GetRef();
    N.Position  = Pos;
    N.NodeIndex = Nodes.Num() - 1;
    return N.NodeIndex;
}

int32 FOrganicStreetGraph::AddEdge(int32 A, int32 B, EOrganicStreetType Type,
                                    float Width, TArray<FVector2D>&& Poly)
{
    if (!Nodes.IsValidIndex(A) || !Nodes.IsValidIndex(B) || A == B) return INDEX_NONE;
    FOrganicStreetEdge& E = Edges.AddDefaulted_GetRef();
    E.NodeA = A; E.NodeB = B; E.StreetType = Type; E.Width = Width;
    E.Poly2D = MoveTemp(Poly);
    const int32 Idx = Edges.Num() - 1;
    Nodes[A].ConnectedEdges.Add(Idx);
    Nodes[B].ConnectedEdges.Add(Idx);
    return Idx;
}

int32 FOrganicStreetGraph::SplitEdge(int32 EdgeIdx, float T)
{
    if (!Edges.IsValidIndex(EdgeIdx)) return INDEX_NONE;
    FOrganicStreetEdge OldEdge = Edges[EdgeIdx];
    const TArray<FVector2D>& Poly = OldEdge.Poly2D;
    float TotalLen = 0.f;
    TArray<float> Cum; Cum.Add(0.f);
    for (int32 i = 0; i < Poly.Num()-1; i++) { TotalLen += (Poly[i+1]-Poly[i]).Size(); Cum.Add(TotalLen); }
    float TargetLen = T * TotalLen;
    FVector2D SplitPos = Poly.Last(); int32 SplitSeg = FMath::Max(0, Poly.Num()-2);
    for (int32 i = 0; i < Poly.Num()-1; i++)
    {
        if (Cum[i+1] >= TargetLen)
        {
            float St = (TotalLen>0.f) ? (TargetLen-Cum[i])/FMath::Max(Cum[i+1]-Cum[i],0.01f) : 0.f;
            SplitPos = FMath::Lerp(Poly[i],Poly[i+1],St); SplitSeg = i; break;
        }
    }
    TArray<FVector2D> PolyA, PolyB;
    for (int32 i=0;i<=SplitSeg;i++) PolyA.Add(Poly[i]); PolyA.Add(SplitPos);
    PolyB.Add(SplitPos); for (int32 i=SplitSeg+1;i<Poly.Num();i++) PolyB.Add(Poly[i]);
    Nodes[OldEdge.NodeA].ConnectedEdges.Remove(EdgeIdx);
    Nodes[OldEdge.NodeB].ConnectedEdges.Remove(EdgeIdx);
    int32 NN = AddNode(SplitPos);
    Nodes[NN].Importance = FMath::Lerp(Nodes[OldEdge.NodeA].Importance, Nodes[OldEdge.NodeB].Importance, T);
    Edges[EdgeIdx] = {}; Edges[EdgeIdx].NodeA=OldEdge.NodeA; Edges[EdgeIdx].NodeB=NN;
    Edges[EdgeIdx].StreetType=OldEdge.StreetType; Edges[EdgeIdx].Width=OldEdge.Width;
    Edges[EdgeIdx].Poly2D=MoveTemp(PolyA);
    Nodes[OldEdge.NodeA].ConnectedEdges.Add(EdgeIdx); Nodes[NN].ConnectedEdges.Add(EdgeIdx);
    AddEdge(NN, OldEdge.NodeB, OldEdge.StreetType, OldEdge.Width, MoveTemp(PolyB));
    return NN;
}

int32 FOrganicStreetGraph::FindNearestNode(FVector2D Pos, float MaxDist) const
{
    int32 Best=-1; float BestD=MaxDist*MaxDist;
    for (int32 i=0;i<Nodes.Num();i++) { float D=(Nodes[i].Position-Pos).SizeSquared(); if(D<BestD){BestD=D;Best=i;} }
    return Best;
}

int32 FOrganicStreetGraph::FindNearestEdgePoint(FVector2D Pos, float& OutT,
                                                  FVector2D& OutClosest, float MaxDist) const
{
    int32 Best=-1; float BestD=MaxDist*MaxDist; OutT=0.f; OutClosest=Pos;
    for (int32 ei=0;ei<Edges.Num();ei++)
    {
        const TArray<FVector2D>& P=Edges[ei].Poly2D;
        for (int32 si=0;si<P.Num()-1;si++)
        {
            FVector2D AB=P[si+1]-P[si], AP=Pos-P[si];
            float Len2=AB.SizeSquared();
            float T=Len2>0.f?FMath::Clamp(FVector2D::DotProduct(AP,AB)/Len2,0.f,1.f):0.f;
            FVector2D C=P[si]+AB*T; float D2=(Pos-C).SizeSquared();
            if (D2<BestD) {
                BestD=D2; Best=ei; OutClosest=C;
                float TLen=0.f; for(int32 k=0;k<P.Num()-1;k++) TLen+=(P[k+1]-P[k]).Size();
                float PLen=0.f; for(int32 k=0;k<si;k++) PLen+=(P[k+1]-P[k]).Size();
                OutT=TLen>0.f?(PLen+T*AB.Size())/TLen:0.f;
            }
        }
    }
    return Best;
}

bool FOrganicStreetGraph::SegmentsIntersect2D(FVector2D A, FVector2D B,
                                               FVector2D C, FVector2D D, FVector2D* OutPt)
{
    FVector2D r=B-A, s=D-C; float rs=r.X*s.Y-r.Y*s.X;
    if (FMath::Abs(rs)<1e-6f) return false;
    FVector2D AC=C-A;
    float t=(AC.X*s.Y-AC.Y*s.X)/rs, u=(AC.X*r.Y-AC.Y*r.X)/rs;
    if (t>0.001f&&t<0.999f&&u>0.001f&&u<0.999f) { if(OutPt)*OutPt=A+r*t; return true; }
    return false;
}

bool FOrganicStreetGraph::WouldSelfIntersect(FVector2D A, FVector2D B) const
{
    for (const FOrganicStreetEdge& E:Edges)
        for (int32 i=0;i<E.Poly2D.Num()-1;i++)
            if (SegmentsIntersect2D(A,B,E.Poly2D[i],E.Poly2D[i+1])) return true;
    return false;
}

void FOrganicStreetGraph::RemoveShortDangles(float MinLength)
{
    bool bChanged=true;
    while (bChanged) {
        bChanged=false;
        for (int32 ni=0;ni<Nodes.Num();ni++) {
            if (Nodes[ni].ConnectedEdges.Num()!=1) continue;
            int32 EIdx=Nodes[ni].ConnectedEdges[0];
            if (!Edges.IsValidIndex(EIdx)) continue;
            const FOrganicStreetEdge& E=Edges[EIdx];
            if (E.StreetType==EOrganicStreetType::Alley) continue;
            float Len=0.f; for(int32 k=0;k<E.Poly2D.Num()-1;k++) Len+=(E.Poly2D[k+1]-E.Poly2D[k]).Size();
            if (Len<MinLength) {
                int32 Other=(E.NodeA==ni)?E.NodeB:E.NodeA;
                Nodes[ni].ConnectedEdges.Empty();
                Nodes[Other].ConnectedEdges.Remove(EIdx);
                Edges[EIdx]={}; bChanged=true;
            }
        }
    }
}

bool FOrganicStreetGraph::Validate() const
{
    for (const FOrganicStreetEdge& E:Edges)
        if (E.NodeA!=-1||E.NodeB!=-1)
            if (!Nodes.IsValidIndex(E.NodeA)||!Nodes.IsValidIndex(E.NodeB)) return false;
    return true;
}
