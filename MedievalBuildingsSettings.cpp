#include "MedievalBuildingsSettings.h"
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

        FRandomStream Rand(Settings->SeedParams.Seed + 307);
        UPCGPointData* OutBuildings = NewObject<UPCGPointData>();

        for (const FPCGPoint& ParcelPoint : Parcels->GetPoints())
        {
            const FVector Pos = ParcelPoint.Transform.GetLocation();
            FPCGPoint B;
            B.Transform = FTransform(FRotator(0, Rand.FRandRange(-9.f, 9.f), 0), Pos + FVector(Rand.FRandRange(-40, 40), Rand.FRandRange(-40, 40), 0));
            B.BoundsMin = FVector(-220, -300, 0);
            B.BoundsMax = FVector(220, 300, 1200);
            B.Density = 1.0f;
            B.Seed = Rand.RandRange(1, MAX_int32);
            OutBuildings->GetMutablePoints().Add(B);
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutBuildings;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalBuildingsSettings::InputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Parcels")), EPCGDataType::Point) };
}

TArray<FPCGPinProperties> UMedievalBuildingsSettings::OutputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Buildings")), EPCGDataType::Point) };
}

FPCGElementPtr UMedievalBuildingsSettings::CreateElement() const
{
    return MakeShared<FMedievalBuildingsElement>();
}
