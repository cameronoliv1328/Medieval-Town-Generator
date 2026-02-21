#include "MedievalStreetNetworkSettings.h"
#include "StreetGraph.h"
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

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
        int32 MarketNode = INDEX_NONE;

        for (const FPCGPoint& Point : InAnchors->GetPoints())
        {
            const int32 Id = Graph.AddNode(Point.Transform.GetLocation(), true);
            if (MarketNode == INDEX_NONE) { MarketNode = Id; }
        }
        if (MarketNode == INDEX_NONE) return true;

        for (int32 I = 0; I < Graph.Nodes.Num(); ++I)
        {
            if (I != MarketNode)
            {
                Graph.AddSegment(I, MarketNode, EMedievalStreetType::Primary, Settings->StreetParams.PrimaryRoadWidth, 1.0f);
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
            P.Density = Segment.Importance;
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
