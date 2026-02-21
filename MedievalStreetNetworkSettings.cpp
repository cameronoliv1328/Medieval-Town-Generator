#include "MedievalStreetNetworkSettings.h"
#include "StreetGraph.h"
#include "TerrainCostField.h"
#include "PCGContext.h"
#include "PCGPointData.h"

class FMedievalStreetNetworkElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalStreetNetworkSettings* Settings = Context ? Cast<UMedievalStreetNetworkSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context || Context->InputData.TaggedData.IsEmpty()) return true;

        const UPCGPointData* InAnchors = Cast<UPCGPointData>(Context->InputData.TaggedData[0].Data);
        if (!InAnchors) return true;

        FRandomStream Rand(Settings->SeedParams.Seed + 101);
        FStreetGraph Graph;
        TArray<int32> GateNodeIds;
        int32 MarketNodeId = INDEX_NONE;

        for (const FPCGPoint& Point : InAnchors->GetPoints())
        {
            const int32 NodeId = Graph.AddNode(Point.Transform.GetLocation(), true);
            const FString Type = InAnchors->Metadata->GetConstTypedAttribute<FString>(TEXT("AnchorType"))->GetValue(Point.MetadataEntry);
            if (Type == TEXT("Gate")) GateNodeIds.Add(NodeId);
            else if (Type == TEXT("Market")) MarketNodeId = NodeId;
        }

        if (MarketNodeId == INDEX_NONE) return true;

        for (const int32 GateNodeId : GateNodeIds)
        {
            Graph.AddSegment(GateNodeId, MarketNodeId, EMedievalStreetType::Primary, Settings->StreetParams.PrimaryRoadWidth, 1.0f);
        }

        const FVector Center = Settings->SeedParams.TownBounds.GetCenter();
        const float Radius = Settings->SeedParams.TownBounds.GetExtent().Size2D() * 0.4f;
        for (int32 I = 0; I < Settings->SecondaryAttractorCount; ++I)
        {
            const float A = Rand.FRandRange(0.f, 2.f * PI);
            const float R = Radius * FMath::Sqrt(Rand.FRand());
            const FVector Candidate = Center + FVector(FMath::Cos(A) * R, FMath::Sin(A) * R, 0);
            if (Graph.HasNearbyIntersection(Candidate, Settings->StreetParams.MinIntersectionSpacing))
            {
                continue;
            }

            int32 Closest = INDEX_NONE;
            float MinD2 = TNumericLimits<float>::Max();
            for (const FStreetNode& Node : Graph.Nodes)
            {
                const float D2 = FVector::DistSquared2D(Node.Position, Candidate);
                if (D2 < MinD2)
                {
                    MinD2 = D2;
                    Closest = Node.Id;
                }
            }

            if (Closest != INDEX_NONE)
            {
                const int32 N = Graph.AddNode(Candidate, false);
                Graph.AddSegment(Closest, N, Rand.FRand() < 0.3f ? EMedievalStreetType::Alley : EMedievalStreetType::Secondary,
                                 Rand.FRand() < 0.3f ? Settings->StreetParams.AlleyWidth : Settings->StreetParams.SecondaryWidth,
                                 Rand.FRandRange(0.2f, 0.8f));
            }
        }

        UPCGPointData* OutStreets = NewObject<UPCGPointData>();
        TArray<FPCGPoint>& OutPoints = OutStreets->GetMutablePoints();
        for (const FStreetSegment& Segment : Graph.Segments)
        {
            const FVector A = Graph.Nodes[Segment.A].Position;
            const FVector B = Graph.Nodes[Segment.B].Position;
            FPCGPoint& P = OutPoints.AddDefaulted_GetRef();
            P.Transform = FTransform((A + B) * 0.5f);
            P.BoundsMin = FVector::ZeroVector;
            P.BoundsMax = FVector(FVector::Dist2D(A, B), Segment.Width * 0.5f, 150.f);
            P.Seed = Rand.RandRange(1, MAX_int32);
            P.MetadataEntry = OutStreets->MutableMetadata()->AddEntry();
            OutStreets->MutableMetadata()->SetStringValue(P.MetadataEntry, TEXT("StreetType"), StaticEnum<EMedievalStreetType>()->GetNameStringByValue((int64)Segment.Type));
            OutStreets->MutableMetadata()->SetFloatValue(P.MetadataEntry, TEXT("StreetWidth"), Segment.Width);
            OutStreets->MutableMetadata()->SetFloatValue(P.MetadataEntry, TEXT("Importance"), Segment.Importance);
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutStreets;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::InputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Anchors")), EPCGDataType::Point) };
}

TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::OutputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Streets")), EPCGDataType::Point) };
}

FPCGElementPtr UMedievalStreetNetworkSettings::CreateElement() const
{
    return MakeShared<FMedievalStreetNetworkElement>();
}
