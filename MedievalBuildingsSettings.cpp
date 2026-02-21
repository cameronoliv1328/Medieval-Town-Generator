#include "MedievalBuildingsSettings.h"
#include "MedievalPCGToggle.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

class FMedievalBuildingsElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalBuildingsSettings* Settings = Context ? Cast<UMedievalBuildingsSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context || Context->InputData.TaggedData.IsEmpty()) return true;

        const UPCGPointData* Parcels = Cast<UPCGPointData>(Context->InputData.TaggedData[0].Data);
        if (!Parcels) return true;

        UPCGPointData* OutBuildings = NewObject<UPCGPointData>();
        OutBuildings->GetMutablePoints() = Parcels->GetPoints();
        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutBuildings;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalBuildingsSettings::InputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Parcels")), EPCGDataType::Point) }; }
TArray<FPCGPinProperties> UMedievalBuildingsSettings::OutputPinProperties() const { return { FPCGPinProperties(FName(TEXT("Buildings")), EPCGDataType::Point) }; }
FPCGElementPtr UMedievalBuildingsSettings::CreateElement() const { return MakeShared<FMedievalBuildingsElement>(); }
#endif
