#include "MedievalStreetSpawnerSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

// ---------------------------------------------------------------------------
// Actor resolver
// ---------------------------------------------------------------------------

static AMedievalTownGenerator* ResolveStreetGen(FPCGContext* Context, const TSoftObjectPtr<AActor>& Fallback)
{
    if (Context && Context->SourceComponent)
        if (AMedievalTownGenerator* G = Cast<AMedievalTownGenerator>(Context->SourceComponent->GetOwner()))
            return G;
    return Cast<AMedievalTownGenerator>(Fallback.Get());
}

// ---------------------------------------------------------------------------
// Element
// ---------------------------------------------------------------------------

class FMedievalStreetSpawnerElement : public IPCGElement
{
protected:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalStreetSpawnerSettings* Settings =
            Context ? Cast<UMedievalStreetSpawnerSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context)
            return true;

        AMedievalTownGenerator* Gen = ResolveStreetGen(Context, Settings->TownGeneratorActor);
        if (!Gen)
            return true;

        const TArray<FMedievalStreetEdge>& Edges = Gen->GetLayoutStreetEdges();
        const TArray<FMedievalStreetNode>& Nodes  = Gen->GetLayoutStreetNodes();

        // -----------------------------------------------------------------------
        // RoadTiles
        // -----------------------------------------------------------------------
        UPCGPointData* OutTiles = NewObject<UPCGPointData>();
        {
            UPCGMetadata* Meta = OutTiles->Metadata;
            auto* AttrEdgeIndex  = Meta->FindOrCreateAttribute<int32>(TEXT("EdgeIndex"),  0);
            auto* AttrSurface    = Meta->FindOrCreateAttribute<int32>(TEXT("Surface"),    0);
            auto* AttrStreetType = Meta->FindOrCreateAttribute<int32>(TEXT("StreetType"), 0);
            auto* AttrWidth      = Meta->FindOrCreateAttribute<float>(TEXT("Width"),      0.f);
            auto* AttrGradeDeg   = Meta->FindOrCreateAttribute<float>(TEXT("GradeDeg"),  0.f);
            auto* AttrSubLength  = Meta->FindOrCreateAttribute<float>(TEXT("SubLength"), 0.f);

            TArray<FPCGPoint>& Points = OutTiles->GetMutablePoints();

            for (const FMedievalStreetEdge& Edge : Edges)
            {
                if (Edge.bIsBridge || Edge.WorldPolyline.Num() < 2)
                    continue;

                const TArray<FVector>& Poly = Edge.WorldPolyline;
                for (int32 i = 0; i < Poly.Num() - 1; ++i)
                {
                    const FVector A = Poly[i];
                    const FVector B = Poly[i + 1];
                    const FVector Dir    = (B - A).GetSafeNormal();
                    const float   SubLen = FVector::Dist(A, B);
                    const FVector Mid    = (A + B) * 0.5f;

                    const FRotator Rot   = Dir.IsNearlyZero() ? FRotator::ZeroRotator : Dir.Rotation();
                    const FVector  Scale = FVector(
                        SubLen / Settings->RoadTileMeshUnitLength,
                        Edge.Width / Settings->RoadTileMeshUnitWidth,
                        1.f);

                    const float Density = (float)Edge.StreetType / 4.f;
                    const int32 Seed    = Edge.EdgeIndex * 10000 + i;

                    FPCGPoint& Pt = Points.AddDefaulted_GetRef();
                    Pt.Transform  = FTransform(Rot.Quaternion(), Mid, Scale);
                    Pt.Seed       = Seed;
                    Pt.Density    = Density;
                    Meta->InitializeOnSet(Pt.MetadataEntry);

                    AttrEdgeIndex ->SetValue(Pt.MetadataEntry, Edge.EdgeIndex);
                    AttrSurface   ->SetValue(Pt.MetadataEntry, (int32)Edge.Surface);
                    AttrStreetType->SetValue(Pt.MetadataEntry, (int32)Edge.StreetType);
                    AttrWidth     ->SetValue(Pt.MetadataEntry, Edge.Width);
                    AttrGradeDeg  ->SetValue(Pt.MetadataEntry, Edge.GradeDeg);
                    AttrSubLength ->SetValue(Pt.MetadataEntry, SubLen);
                }
            }
        }

        // -----------------------------------------------------------------------
        // BridgeDecks
        // -----------------------------------------------------------------------
        UPCGPointData* OutDecks = NewObject<UPCGPointData>();
        {
            UPCGMetadata* Meta = OutDecks->Metadata;
            auto* AttrEdgeIndex       = Meta->FindOrCreateAttribute<int32>(TEXT("EdgeIndex"),        0);
            auto* AttrWidth           = Meta->FindOrCreateAttribute<float>(TEXT("Width"),            0.f);
            auto* AttrBridgeSpanLen   = Meta->FindOrCreateAttribute<float>(TEXT("BridgeSpanLength"), 0.f);
            auto* AttrSubLength       = Meta->FindOrCreateAttribute<float>(TEXT("SubLength"),        0.f);

            TArray<FPCGPoint>& Points = OutDecks->GetMutablePoints();

            for (const FMedievalStreetEdge& Edge : Edges)
            {
                if (!Edge.bIsBridge || Edge.WorldPolyline.Num() < 2)
                    continue;

                const TArray<FVector>& Poly = Edge.WorldPolyline;
                for (int32 i = 0; i < Poly.Num() - 1; ++i)
                {
                    const FVector A = Poly[i];
                    const FVector B = Poly[i + 1];
                    const FVector Dir    = (B - A).GetSafeNormal();
                    const float   SubLen = FVector::Dist(A, B);
                    const FVector Mid    = (A + B) * 0.5f;

                    const FRotator Rot   = Dir.IsNearlyZero() ? FRotator::ZeroRotator : Dir.Rotation();
                    const FVector  Scale = FVector(
                        SubLen / Settings->BridgeDeckMeshUnitLength,
                        Edge.Width / Settings->BridgeDeckMeshUnitWidth,
                        1.f);

                    FPCGPoint& Pt = Points.AddDefaulted_GetRef();
                    Pt.Transform  = FTransform(Rot.Quaternion(), Mid, Scale);
                    Pt.Seed       = Edge.EdgeIndex * 10000 + i;
                    Pt.Density    = 1.f;
                    Meta->InitializeOnSet(Pt.MetadataEntry);

                    AttrEdgeIndex    ->SetValue(Pt.MetadataEntry, Edge.EdgeIndex);
                    AttrWidth        ->SetValue(Pt.MetadataEntry, Edge.Width);
                    AttrBridgeSpanLen->SetValue(Pt.MetadataEntry, Edge.BridgeSpanLength);
                    AttrSubLength    ->SetValue(Pt.MetadataEntry, SubLen);
                }
            }
        }

        // -----------------------------------------------------------------------
        // BridgeRails
        // -----------------------------------------------------------------------
        UPCGPointData* OutRails = NewObject<UPCGPointData>();
        {
            UPCGMetadata* Meta = OutRails->Metadata;
            auto* AttrEdgeIndex = Meta->FindOrCreateAttribute<int32>(TEXT("EdgeIndex"), 0);
            auto* AttrSide      = Meta->FindOrCreateAttribute<int32>(TEXT("Side"),      0);

            TArray<FPCGPoint>& Points = OutRails->GetMutablePoints();

            for (const FMedievalStreetEdge& Edge : Edges)
            {
                if (!Edge.bIsBridge || Edge.WorldPolyline.Num() < 2)
                    continue;

                const TArray<FVector>& Poly = Edge.WorldPolyline;
                for (int32 i = 0; i < Poly.Num() - 1; ++i)
                {
                    const FVector A = Poly[i];
                    const FVector B = Poly[i + 1];
                    const FVector Dir     = (B - A).GetSafeNormal();
                    const float   SubLen  = FVector::Dist(A, B);
                    const FVector Mid     = (A + B) * 0.5f;
                    const FRotator Rot    = Dir.IsNearlyZero() ? FRotator::ZeroRotator : Dir.Rotation();
                    const FVector Lateral = FVector::CrossProduct(Dir, FVector::UpVector).GetSafeNormal();

                    const FVector Scale = FVector(SubLen / Settings->BridgeRailMeshUnitLength, 1.f, 1.f);

                    static const int32 Sides[2] = { -1, 1 };
                    for (int32 SideIdx = 0; SideIdx < 2; ++SideIdx)
                    {
                        const int32   Side    = Sides[SideIdx];
                        const FVector RailPos = Mid + Lateral * (float)Side * Edge.Width * 0.5f;

                        FPCGPoint& Pt = Points.AddDefaulted_GetRef();
                        Pt.Transform  = FTransform(Rot.Quaternion(), RailPos, Scale);
                        Pt.Seed       = Edge.EdgeIndex * 20000 + i * 2 + (Side == -1 ? 0 : 1);
                        Pt.Density    = 1.f;
                        Meta->InitializeOnSet(Pt.MetadataEntry);

                        AttrEdgeIndex->SetValue(Pt.MetadataEntry, Edge.EdgeIndex);
                        AttrSide     ->SetValue(Pt.MetadataEntry, Side);
                    }
                }
            }
        }

        // -----------------------------------------------------------------------
        // Intersections
        // -----------------------------------------------------------------------
        UPCGPointData* OutIntersections = NewObject<UPCGPointData>();
        {
            UPCGMetadata* Meta = OutIntersections->Metadata;
            auto* AttrNodeIndex       = Meta->FindOrCreateAttribute<int32>(TEXT("NodeIndex"),        0);
            auto* AttrIntersectType   = Meta->FindOrCreateAttribute<int32>(TEXT("IntersectionType"), 0);
            auto* AttrIsMarket        = Meta->FindOrCreateAttribute<float>(TEXT("bIsMarket"),        0.f);
            auto* AttrIsGate          = Meta->FindOrCreateAttribute<float>(TEXT("bIsGate"),          0.f);
            auto* AttrIsBridge        = Meta->FindOrCreateAttribute<float>(TEXT("bIsBridge"),        0.f);
            auto* AttrDegree          = Meta->FindOrCreateAttribute<int32>(TEXT("Degree"),           0);

            TArray<FPCGPoint>& Points = OutIntersections->GetMutablePoints();

            for (const FMedievalStreetNode& Node : Nodes)
            {
                float Density = (float)Node.IntersectionType / 4.f;
                if      (Node.bIsMarket) Density = 1.f;
                else if (Node.bIsGate)   Density = 0.75f;

                FPCGPoint& Pt = Points.AddDefaulted_GetRef();
                Pt.Transform  = FTransform(FQuat::Identity, Node.WorldPos, FVector::OneVector);
                Pt.Seed       = Node.NodeIndex;
                Pt.Density    = Density;
                Meta->InitializeOnSet(Pt.MetadataEntry);

                AttrNodeIndex    ->SetValue(Pt.MetadataEntry, Node.NodeIndex);
                AttrIntersectType->SetValue(Pt.MetadataEntry, (int32)Node.IntersectionType);
                AttrIsMarket     ->SetValue(Pt.MetadataEntry, Node.bIsMarket     ? 1.f : 0.f);
                AttrIsGate       ->SetValue(Pt.MetadataEntry, Node.bIsGate       ? 1.f : 0.f);
                AttrIsBridge     ->SetValue(Pt.MetadataEntry, Node.bIsBridgeAbutment ? 1.f : 0.f);
                AttrDegree       ->SetValue(Pt.MetadataEntry, Node.ConnectedEdgeIndices.Num());
            }
        }

        // -----------------------------------------------------------------------
        // Emit outputs
        // -----------------------------------------------------------------------
        TArray<FPCGTaggedData>& Outputs = Context->OutputData.TaggedData;

        FPCGTaggedData& TilesOut = Outputs.Emplace_GetRef();
        TilesOut.Data = OutTiles;
        TilesOut.Tags.Add(FName(TEXT("RoadTiles")));

        FPCGTaggedData& DecksOut = Outputs.Emplace_GetRef();
        DecksOut.Data = OutDecks;
        DecksOut.Tags.Add(FName(TEXT("BridgeDecks")));

        FPCGTaggedData& RailsOut = Outputs.Emplace_GetRef();
        RailsOut.Data = OutRails;
        RailsOut.Tags.Add(FName(TEXT("BridgeRails")));

        FPCGTaggedData& IntersectOut = Outputs.Emplace_GetRef();
        IntersectOut.Data = OutIntersections;
        IntersectOut.Tags.Add(FName(TEXT("Intersections")));

        return true;
    }
};

// ---------------------------------------------------------------------------
// UMedievalStreetSpawnerSettings — PCGSettings interface
// ---------------------------------------------------------------------------

TArray<FPCGPinProperties> UMedievalStreetSpawnerSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalStreetSpawnerSettings::OutputPinProperties() const
{
    TArray<FPCGPinProperties> Pins;
    Pins.Emplace(TEXT("RoadTiles"),      EPCGDataType::Point);
    Pins.Emplace(TEXT("BridgeDecks"),    EPCGDataType::Point);
    Pins.Emplace(TEXT("BridgeRails"),    EPCGDataType::Point);
    Pins.Emplace(TEXT("Intersections"),  EPCGDataType::Point);
    return Pins;
}

FPCGElementPtr UMedievalStreetSpawnerSettings::CreateElement() const
{
    return MakeShared<FMedievalStreetSpawnerElement>();
}

#endif // MEDIEVAL_ENABLE_PCG_NODES
