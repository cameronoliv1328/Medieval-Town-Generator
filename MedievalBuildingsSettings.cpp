#include "MedievalBuildingsSettings.h"
#include "MedievalPCGCompat.h"
#if MEDIEVAL_HAS_PCG
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"
#endif
#include "PolygonUtils.h"

#if MEDIEVAL_HAS_PCG

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
            const float Frontage = Parcels->Metadata->GetConstTypedAttribute<float>(TEXT("Frontage"))->GetValue(ParcelPoint.MetadataEntry);
            const float Depth = Parcels->Metadata->GetConstTypedAttribute<float>(TEXT("Depth"))->GetValue(ParcelPoint.MetadataEntry);
            const FString District = Parcels->Metadata->GetConstTypedAttribute<FString>(TEXT("District"))->GetValue(ParcelPoint.MetadataEntry);
            const float Wealth = Parcels->Metadata->GetConstTypedAttribute<float>(TEXT("Wealth"))->GetValue(ParcelPoint.MetadataEntry);
            const float MixedUse = Parcels->Metadata->GetConstTypedAttribute<float>(TEXT("MixedUse"))->GetValue(ParcelPoint.MetadataEntry);

            EMedievalFootprintType Type = EMedievalFootprintType::Rowhouse;
            if (District.Contains(TEXT("NobleWard")) && Wealth > 0.65f)
            {
                Type = EMedievalFootprintType::CourtyardHouse;
            }
            else if (District.Contains(TEXT("PoorWard")))
            {
                Type = EMedievalFootprintType::Hut;
            }
            else if (MixedUse > 0.65f)
            {
                Type = EMedievalFootprintType::Shopfront;
            }

            const float Setback = District.Contains(TEXT("Outskirts")) ? Settings->ParcelParams.SetbackOutskirts : Settings->ParcelParams.SetbackCore;
            const FVector Pos = ParcelPoint.Transform.GetLocation();
            const float Width = FMath::Max(Frontage - 100.f, 180.f);
            const float BuildDepth = FMath::Max(Depth * (1.f - Settings->BuildingParams.YardReserveRatio) - Setback, 220.f);

            FPCGPoint B;
            B.Transform = FTransform(FRotator(0, Rand.FRandRange(-9.f, 9.f), 0), Pos + FVector(Rand.FRandRange(-40, 40), Rand.FRandRange(-40, 40), 0));
            B.BoundsMin = FVector(-Width * 0.5f, -BuildDepth * 0.5f, 0);
            B.BoundsMax = FVector(Width * 0.5f, BuildDepth * 0.5f, 1200.f);
            B.Density = 1.0f;
            B.Seed = Rand.RandRange(1, MAX_int32);
            B.MetadataEntry = OutBuildings->MutableMetadata()->AddEntry();

            const int32 Floors = District.Contains(TEXT("MarketWard"))
                ? Rand.RandRange((int32)Settings->BuildingParams.CoreFloorsRange.X, (int32)Settings->BuildingParams.CoreFloorsRange.Y)
                : Rand.RandRange((int32)Settings->BuildingParams.OuterFloorsRange.X, (int32)Settings->BuildingParams.OuterFloorsRange.Y);

            const FString Roof = (Type == EMedievalFootprintType::CourtyardHouse) ? TEXT("Hip") : (Rand.FRand() < 0.65f ? TEXT("Gable") : TEXT("HalfHip"));
            OutBuildings->MutableMetadata()->SetStringValue(B.MetadataEntry, TEXT("FootprintType"), StaticEnum<EMedievalFootprintType>()->GetNameStringByValue((int64)Type));
            OutBuildings->MutableMetadata()->SetInt32Value(B.MetadataEntry, TEXT("Floors"), Floors);
            OutBuildings->MutableMetadata()->SetStringValue(B.MetadataEntry, TEXT("RoofType"), Roof);
            OutBuildings->MutableMetadata()->SetBoolValue(B.MetadataEntry, TEXT("PartyWall"), Rand.FRand() < Settings->BuildingParams.PartyWallChanceCore && District.Contains(TEXT("MarketWard")));
            OutBuildings->MutableMetadata()->SetBoolValue(B.MetadataEntry, TEXT("HasRearExtension"), Rand.FRand() < Settings->BuildingParams.RearExtensionChance);
            OutBuildings->MutableMetadata()->SetStringValue(B.MetadataEntry, TEXT("FacadeTag"), MixedUse > 0.55f ? TEXT("Shopfront") : TEXT("Residential"));

            OutBuildings->GetMutablePoints().Add(B);

            if (Depth > 3200.f && Rand.FRand() < 0.4f)
            {
                FPCGPoint Outbuilding = B;
                Outbuilding.Transform.SetLocation(Pos + FVector(0, BuildDepth * 0.35f, 0));
                Outbuilding.BoundsMin = FVector(-180, -220, 0);
                Outbuilding.BoundsMax = FVector(180, 220, 500);
                Outbuilding.MetadataEntry = OutBuildings->MutableMetadata()->AddEntry();
                OutBuildings->MutableMetadata()->SetStringValue(Outbuilding.MetadataEntry, TEXT("FootprintType"), TEXT("Workshop"));
                OutBuildings->MutableMetadata()->SetInt32Value(Outbuilding.MetadataEntry, TEXT("Floors"), 1);
                OutBuildings->MutableMetadata()->SetStringValue(Outbuilding.MetadataEntry, TEXT("RoofType"), TEXT("Gable"));
                OutBuildings->MutableMetadata()->SetStringValue(Outbuilding.MetadataEntry, TEXT("FacadeTag"), TEXT("Service"));
                OutBuildings->GetMutablePoints().Add(Outbuilding);
            }
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

#endif

#if !MEDIEVAL_HAS_PCG
// PCG plugin/module not available in this build target; class remains data-only for compilation safety.
#endif
