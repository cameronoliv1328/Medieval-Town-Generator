#include "OrganicStreetNetworkSettings.h"
#include "OrganicTerrainRouting.h"

FOrganicStreetGraph UOrganicStreetNetworkSettings::BuildPreviewGraph() const
{
    FOrganicStreetGraph Graph;
    FRandomStream Rand(Seed);

    const FVector C = TownBounds.GetCenter();
    const float Radius = TownBounds.GetExtent().Size2D() * 0.45f;

    auto AddNode = [&](const FVector2D& P, float Importance, bool bGate, bool bMarket, bool bBridge, bool bLandmark)
    {
        FStreetNode N;
        N.Position = FVector(P.X, P.Y, 0.0f);
        N.Importance = Importance;
        N.bIsGate = bGate;
        N.bIsMarket = bMarket;
        N.bIsBridgeNode = bBridge;
        N.bIsLandmark = bLandmark;
        Graph.Nodes.Add(N);
        return Graph.Nodes.Num() - 1;
    };

    auto AddEdge = [&](int32 A, int32 B, EOrganicStreetType Type, float W)
    {
        FStreetEdge E;
        E.A = A;
        E.B = B;
        E.StreetType = Type;
        E.Width = W;
        E.TargetSpeed = (Type == EOrganicStreetType::Primary) ? 3.0f : (Type == EOrganicStreetType::Secondary ? 2.2f : 1.2f);
        E.CurvatureCost = Rand.FRandRange(0.0f, 0.3f);
        E.PolylinePoints.Add(Graph.Nodes[A].Position);
        E.PolylinePoints.Add(Graph.Nodes[B].Position);
        Graph.Edges.Add(E);

        const int32 Idx = Graph.Edges.Num() - 1;
        Graph.Nodes[A].ConnectedEdges.Add(Idx);
        Graph.Nodes[B].ConnectedEdges.Add(Idx);
    };

    const FVector2D Market(C.X, C.Y);
    const int32 MarketNode = AddNode(Market, 1.0f, false, true, false, true);
    const int32 KeepNode = AddNode(Market + FVector2D(Radius * 0.3f, -Radius * 0.2f), 0.9f, false, false, false, true);
    const int32 ChurchNode = AddNode(Market + FVector2D(-Radius * 0.24f, Radius * 0.19f), 0.85f, false, false, false, true);

    TArray<int32> Gates;
    const int32 GateCount = 4;
    for (int32 G = 0; G < GateCount; ++G)
    {
        const float A = ((float)G + Rand.FRandRange(-0.3f, 0.3f)) / GateCount * TWO_PI;
        const FVector2D GP = Market + FVector2D(FMath::Cos(A), FMath::Sin(A)) * Radius;
        Gates.Add(AddNode(GP, 1.0f, true, false, false, false));
    }

    for (int32 Gate : Gates)
    {
        AddEdge(Gate, MarketNode, EOrganicStreetType::Primary, Rand.FRandRange(PrimaryWidthRange.X, PrimaryWidthRange.Y));
    }
    AddEdge(MarketNode, KeepNode, EOrganicStreetType::Primary, Rand.FRandRange(PrimaryWidthRange.X, PrimaryWidthRange.Y));
    AddEdge(MarketNode, ChurchNode, EOrganicStreetType::Primary, Rand.FRandRange(PrimaryWidthRange.X, PrimaryWidthRange.Y));

    for (int32 I = 0; I < 24; ++I)
    {
        const FVector2D P = Market + FVector2D(Rand.FRandRange(-Radius, Radius), Rand.FRandRange(-Radius, Radius));
        const int32 N = AddNode(P, 0.35f, false, false, false, false);
        const int32 Near = OrganicStreetGraphUtils::FindNearestNode(Graph, FVector(P.X, P.Y, 0.0f));
        if (Near != INDEX_NONE && Near != N)
        {
            AddEdge(N, Near, EOrganicStreetType::Secondary, Rand.FRandRange(SecondaryWidthRange.X, SecondaryWidthRange.Y));
        }
    }

    return Graph;
}

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"
#include "PCGSettings.h"

class FOrganicStreetNetworkElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UOrganicStreetNetworkSettings* Settings = Context ? Cast<UOrganicStreetNetworkSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context)
        {
            return true;
        }

        const FOrganicStreetGraph Graph = Settings->BuildPreviewGraph();
        UPCGPointData* OutNodes = NewObject<UPCGPointData>();
        for (const FStreetNode& N : Graph.Nodes)
        {
            FPCGPoint Pt;
            Pt.Transform = FTransform(N.Position);
            Pt.Density = FMath::Clamp(N.Importance, 0.05f, 1.0f);
            Pt.Seed = Settings->Seed;
            OutNodes->GetMutablePoints().Add(Pt);
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Pin = TEXT("StreetNodes");
        Context->OutputData.TaggedData.Last().Data = OutNodes;
        return true;
    }
};
#endif
