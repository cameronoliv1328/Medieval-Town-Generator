#include "MedievalParcelsSettings.h"
#include "PolygonUtils.h"
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
            const FVector StreetPos = StreetPoint.Transform.GetLocation();
            const int32 ParcelCount = Rand.RandRange(3, 8);
            for (int32 I = 0; I < ParcelCount; ++I)
            {
                const float Frontage = Rand.FRandRange(Settings->ParcelParams.BurgageFrontageRange.X, Settings->ParcelParams.BurgageFrontageRange.Y);
                const float Depth = Rand.FRandRange(Settings->ParcelParams.BurgageDepthRange.X * 0.65f, Settings->ParcelParams.BurgageDepthRange.Y);
                FPCGPoint P;
                P.Transform = FTransform(StreetPos + FVector(Rand.FRandRange(-700.f, 700.f), Rand.FRandRange(-700.f, 700.f), 0));
                P.BoundsMin = FVector(-Frontage * 0.5f, -Depth * 0.5f, 0);
                P.BoundsMax = FVector(Frontage * 0.5f, Depth * 0.5f, 200.f);
                P.Seed = Rand.RandRange(1, MAX_int32);
                P.Density = 1.0f;
                OutParcels->GetMutablePoints().Add(P);
            }
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Data = OutParcels;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalParcelsSettings::InputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Streets")), EPCGDataType::Point) };
}

TArray<FPCGPinProperties> UMedievalParcelsSettings::OutputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Parcels")), EPCGDataType::Point) };
}

FPCGElementPtr UMedievalParcelsSettings::CreateElement() const
{
    return MakeShared<FMedievalParcelsElement>();
}
