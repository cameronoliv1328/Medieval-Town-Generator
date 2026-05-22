#include "MedievalStreetLayoutSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

static AMedievalTownGenerator* ResolveStreetGen(
    FPCGContext* Context, const TSoftObjectPtr<AActor>& Fallback)
{
    if (Context && Context->SourceComponent)
    {
        if (AMedievalTownGenerator* G =
                Cast<AMedievalTownGenerator>(Context->SourceComponent->GetOwner()))
            return G;
    }
    return Cast<AMedievalTownGenerator>(Fallback.Get());
}

// ---------------------------------------------------------------------------
class FMedievalStreetLayoutElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalStreetLayoutSettings* Settings =
            Context ? Cast<UMedievalStreetLayoutSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context) return true;

        AMedievalTownGenerator* Gen =
            ResolveStreetGen(Context, Settings->TownGeneratorActor);
        if (!Gen) return true;

        // ---- StreetNodes output ---------------------------------------------
        // One PCG point per intersection, gate, market, or bridge abutment.
        UPCGPointData* OutNodes = NewObject<UPCGPointData>();
        UPCGMetadata* NMeta = OutNodes->Metadata;

        auto* IntTypeAttr  = NMeta->FindOrCreateAttribute<int32>(TEXT("IntersectionType"),  0);
        auto* IsMarketAttr = NMeta->FindOrCreateAttribute<float>(TEXT("bIsMarket"),         0.f);
        auto* IsGateAttr   = NMeta->FindOrCreateAttribute<float>(TEXT("bIsGate"),           0.f);
        auto* IsBridgeAttr = NMeta->FindOrCreateAttribute<float>(TEXT("bIsBridgeAbutment"), 0.f);
        auto* DegreeAttr   = NMeta->FindOrCreateAttribute<int32>(TEXT("Degree"),            0);

        for (const FMedievalStreetNode& Node : Gen->GetLayoutStreetNodes())
        {
            // Density encodes semantic priority for PCG prop scattering.
            float D = Node.bIsMarket ? 1.f : (Node.bIsGate ? 0.75f : 0.5f);

            FPCGPoint& Pt = OutNodes->GetMutablePoints().AddDefaulted_GetRef();
            Pt.Transform  = FTransform(Node.WorldPos);
            Pt.Seed       = Node.NodeIndex;
            Pt.Density    = D;
            NMeta->InitializeOnSet(Pt.MetadataEntry);

            IntTypeAttr ->SetValue(Pt.MetadataEntry, (int32)Node.IntersectionType);
            IsMarketAttr->SetValue(Pt.MetadataEntry, Node.bIsMarket         ? 1.f : 0.f);
            IsGateAttr  ->SetValue(Pt.MetadataEntry, Node.bIsGate           ? 1.f : 0.f);
            IsBridgeAttr->SetValue(Pt.MetadataEntry, Node.bIsBridgeAbutment ? 1.f : 0.f);
            DegreeAttr  ->SetValue(Pt.MetadataEntry, Node.ConnectedEdgeIndices.Num());
        }

        // ---- StreetEdges output ---------------------------------------------
        // One PCG point per WorldPolyline vertex; VertexIndex/Count let PCG
        // reconstruct each edge as an ordered polyline for mesh placement.
        UPCGPointData* OutEdges = NewObject<UPCGPointData>();
        UPCGMetadata* EMeta = OutEdges->Metadata;

        auto* EdgeIdxAttr  = EMeta->FindOrCreateAttribute<int32>(TEXT("EdgeIndex"),     -1);
        auto* NodeAAttr    = EMeta->FindOrCreateAttribute<int32>(TEXT("NodeIndexA"),    -1);
        auto* NodeBAttr    = EMeta->FindOrCreateAttribute<int32>(TEXT("NodeIndexB"),    -1);
        auto* StreetTAttr  = EMeta->FindOrCreateAttribute<int32>(TEXT("StreetType"),     0);
        auto* SurfaceAttr  = EMeta->FindOrCreateAttribute<int32>(TEXT("Surface"),        0);
        auto* WidthAttr    = EMeta->FindOrCreateAttribute<float>(TEXT("Width"),         0.f);
        auto* GradeAttr    = EMeta->FindOrCreateAttribute<float>(TEXT("GradeDeg"),      0.f);
        auto* BridgeAttr   = EMeta->FindOrCreateAttribute<float>(TEXT("bIsBridge"),     0.f);
        auto* QuayAttr     = EMeta->FindOrCreateAttribute<float>(TEXT("bIsQuayStreet"), 0.f);
        auto* VtxIdxAttr   = EMeta->FindOrCreateAttribute<int32>(TEXT("VertexIndex"),    0);
        auto* VtxCntAttr   = EMeta->FindOrCreateAttribute<int32>(TEXT("VertexCount"),    0);

        for (const FMedievalStreetEdge& Edge : Gen->GetLayoutStreetEdges())
        {
            if (Edge.WorldPolyline.IsEmpty()) continue;

            const int32 VtxCount = Edge.WorldPolyline.Num();
            float D = Edge.bIsBridge ? 1.f : (float)Edge.StreetType / 4.f;

            for (int32 Vi = 0; Vi < VtxCount; ++Vi)
            {
                const FVector& Pos = Edge.WorldPolyline[Vi];

                // Rotation aligned with the edge direction at this vertex.
                FVector FwdDir = FVector::ZeroVector;
                if (Vi + 1 < VtxCount)
                    FwdDir = (Edge.WorldPolyline[Vi + 1] - Pos).GetSafeNormal();
                else if (Vi > 0)
                    FwdDir = (Pos - Edge.WorldPolyline[Vi - 1]).GetSafeNormal();

                FRotator EdgeRot = FwdDir.IsNearlyZero()
                    ? FRotator::ZeroRotator
                    : FwdDir.Rotation();

                FPCGPoint& Pt = OutEdges->GetMutablePoints().AddDefaulted_GetRef();
                Pt.Transform  = FTransform(EdgeRot, Pos);
                Pt.Seed       = Edge.EdgeIndex * 10000 + Vi;
                Pt.Density    = D;
                EMeta->InitializeOnSet(Pt.MetadataEntry);

                EdgeIdxAttr->SetValue(Pt.MetadataEntry, Edge.EdgeIndex);
                NodeAAttr  ->SetValue(Pt.MetadataEntry, Edge.NodeIndexA);
                NodeBAttr  ->SetValue(Pt.MetadataEntry, Edge.NodeIndexB);
                StreetTAttr->SetValue(Pt.MetadataEntry, (int32)Edge.StreetType);
                SurfaceAttr->SetValue(Pt.MetadataEntry, (int32)Edge.Surface);
                WidthAttr  ->SetValue(Pt.MetadataEntry, Edge.Width);
                GradeAttr  ->SetValue(Pt.MetadataEntry, Edge.GradeDeg);
                BridgeAttr ->SetValue(Pt.MetadataEntry, Edge.bIsBridge     ? 1.f : 0.f);
                QuayAttr   ->SetValue(Pt.MetadataEntry, Edge.bIsQuayStreet ? 1.f : 0.f);
                VtxIdxAttr ->SetValue(Pt.MetadataEntry, Vi);
                VtxCntAttr ->SetValue(Pt.MetadataEntry, VtxCount);
            }
        }

        FPCGTaggedData& NodesOut  = Context->OutputData.TaggedData.Emplace_GetRef();
        NodesOut.Data             = OutNodes;
        NodesOut.Tags.Add(FName("StreetNodes"));

        FPCGTaggedData& EdgesOut  = Context->OutputData.TaggedData.Emplace_GetRef();
        EdgesOut.Data             = OutEdges;
        EdgesOut.Tags.Add(FName("StreetEdges"));

        return true;
    }
};

// ---------------------------------------------------------------------------
TArray<FPCGPinProperties> UMedievalStreetLayoutSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalStreetLayoutSettings::OutputPinProperties() const
{
    return {
        FPCGPinProperties(FName(TEXT("StreetNodes")), EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("StreetEdges")), EPCGDataType::Point),
    };
}

FPCGElementPtr UMedievalStreetLayoutSettings::CreateElement() const
{
    return MakeShared<FMedievalStreetLayoutElement>();
}
#endif
