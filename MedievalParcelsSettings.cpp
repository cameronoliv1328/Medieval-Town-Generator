#include "MedievalParcelsSettings.h"
#include "MedievalPCGToggle.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

class FMedievalParcelsElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalParcelsSettings* Settings = Context ? Cast<UMedievalParcelsSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context || Context->InputData.TaggedData.IsEmpty()) return true;

        const UPCGPointData* Streets = Cast<UPCGPointData>(Context->InputData.TaggedData[0].Data);
        if (!Streets) return true;

        FRandomStream Rand(Settings->SeedParams.Seed + 203);
        UPCGPointData* OutParcels = NewObject<UPCGPointData>();
        for (const FPCGPoint& StreetPoint : Streets->GetPoints())
        {
            FPCGPoint P = StreetPoint;
            P.Transform.AddToTranslation(FVector(Rand.FRandRange(-200, 200), Rand.FRandRange(-200, 200), 0));
            OutParcels->GetMutablePoints().Add(P);
        }
        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutParcels;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalParcelsSettings::InputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Streets")), EPCGDataType::Point) }; }
TArray<FPCGPinProperties> UMedievalParcelsSettings::OutputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Parcels")), EPCGDataType::Point) }; }
FPCGElementPtr UMedievalParcelsSettings::CreateElement() const { return MakeShared<FMedievalParcelsElement>(); }
#endif
