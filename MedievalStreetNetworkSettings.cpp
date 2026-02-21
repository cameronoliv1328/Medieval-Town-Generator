#include "MedievalStreetNetworkSettings.h"
#include "MedievalPCGToggle.h"

#if MEDIEVAL_ENABLE_PCG_NODES
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
        UPCGPointData* OutStreets = NewObject<UPCGPointData>();
        TArray<FPCGPoint>& OutPoints = OutStreets->GetMutablePoints();
        for (const FPCGPoint& Point : InAnchors->GetPoints())
        {
            FPCGPoint& P = OutPoints.AddDefaulted_GetRef();
            P.Transform = Point.Transform;
            P.Seed = Rand.RandRange(1, MAX_int32);
            P.Density = 1.0;
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutStreets;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::InputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Anchors")), EPCGDataType::Point) }; }
TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::OutputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Streets")), EPCGDataType::Point) }; }
FPCGElementPtr UMedievalStreetNetworkSettings::CreateElement() const { return MakeShared<FMedievalStreetNetworkElement>(); }
#endif
