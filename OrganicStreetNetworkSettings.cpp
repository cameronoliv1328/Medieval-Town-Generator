#include "OrganicStreetNetworkSettings.h"
#include "OrganicStreetGraph.h"   // plain C++ -- safe in .cpp, NOT in .h

FOrganicStreetGraph OrganicStreetSettingsUtils::BuildPreviewGraph(
    const UOrganicStreetNetworkSettings* Settings)
{
    check(Settings);
    FOrganicStreetGraph Graph;
    FRandomStream Rand(Settings->Seed);

    const FVector   C      = Settings->TownBounds.GetCenter();
    const float     Radius = Settings->TownBounds.GetExtent().Size2D() * 0.45f;
    const FVector2D Market(C.X, C.Y);

    // -- Add anchor nodes -----------------------------------------------------
    auto MakeAnchor = [&](FVector2D Pos, float Imp,
                          bool bGate, bool bMkt, bool bBridge, bool bLandmark) -> int32
    {
        int32 Idx = Graph.AddNode(Pos);
        Graph.Nodes[Idx].Importance    = Imp;
        Graph.Nodes[Idx].bIsGate       = bGate;
        Graph.Nodes[Idx].bIsMarket     = bMkt;
        Graph.Nodes[Idx].bIsBridgeNode = bBridge;
        Graph.Nodes[Idx].bIsLandmark   = bLandmark;
        return Idx;
    };

    const int32 MarketNode = MakeAnchor(Market,                                             1.00f, false, true,  false, true);
    const int32 KeepNode   = MakeAnchor(Market + FVector2D( Radius*0.30f, -Radius*0.20f),  0.90f, false, false, false, true);
    const int32 ChurchNode = MakeAnchor(Market + FVector2D(-Radius*0.24f,  Radius*0.19f),  0.85f, false, false, false, true);

    // -- Gates -----------------------------------------------------------------
    TArray<int32> Gates;
    const int32 GateCount = 4;
    for (int32 G = 0; G < GateCount; ++G)
    {
        float Angle = ((float)G + Rand.FRandRange(-0.3f, 0.3f)) / GateCount * TWO_PI;
        FVector2D GP = Market + FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)) * Radius;
        Gates.Add(MakeAnchor(GP, 1.0f, true, false, false, false));
    }

    // -- Primary edges ---------------------------------------------------------
    auto AddPrimary = [&](int32 A, int32 B)
    {
        float W = Rand.FRandRange(Settings->PrimaryWidthRange.X, Settings->PrimaryWidthRange.Y);
        TArray<FVector2D> Poly = { Graph.Nodes[A].Position, Graph.Nodes[B].Position };
        Graph.AddEdge(A, B, EOrganicStreetType::Primary, W, MoveTemp(Poly));
    };

    for (int32 Gate : Gates) AddPrimary(Gate, MarketNode);
    AddPrimary(MarketNode, KeepNode);
    AddPrimary(MarketNode, ChurchNode);

    // -- Secondary nodes -------------------------------------------------------
    for (int32 I = 0; I < 24; ++I)
    {
        FVector2D Pos = Market + FVector2D(Rand.FRandRange(-Radius, Radius),
                                           Rand.FRandRange(-Radius, Radius));
        int32 NewNode = Graph.AddNode(Pos);
        Graph.Nodes[NewNode].Importance = 0.35f;

        int32 Nearest = Graph.FindNearestNode(Pos, Radius * 2.f);
        if (Nearest != INDEX_NONE && Nearest != NewNode)
        {
            float W = Rand.FRandRange(Settings->SecondaryWidthRange.X,
                                       Settings->SecondaryWidthRange.Y);
            TArray<FVector2D> Poly = { Pos, Graph.Nodes[Nearest].Position };
            Graph.AddEdge(NewNode, Nearest, EOrganicStreetType::Secondary, W, MoveTemp(Poly));
        }
    }

    return Graph;
}

// -----------------------------------------------------------------------------
//  PCG node implementation
// -----------------------------------------------------------------------------
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
        const UOrganicStreetNetworkSettings* Settings =
            Context ? Cast<UOrganicStreetNetworkSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context) return true;

        // Safe: FOrganicStreetGraph used only in .cpp, never in .h
        const FOrganicStreetGraph Graph =
            OrganicStreetSettingsUtils::BuildPreviewGraph(Settings);

        UPCGPointData* OutNodes = NewObject<UPCGPointData>();
        for (const FOrganicStreetNode& N : Graph.Nodes)
        {
            FPCGPoint Pt;
            Pt.Transform = FTransform(FVector(N.Position.X, N.Position.Y, 0.f));
            Pt.Density   = FMath::Clamp(N.Importance, 0.05f, 1.0f);
            Pt.Seed      = Settings->Seed;
            OutNodes->GetMutablePoints().Add(Pt);
        }

        FPCGTaggedData& Out = Context->OutputData.TaggedData.Emplace_GetRef();
        Out.Data = OutNodes;
        Out.Tags.Add(FName("StreetNodes"));
        return true;
    }
};
#endif
