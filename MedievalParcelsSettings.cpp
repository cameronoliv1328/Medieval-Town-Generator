#include "MedievalParcelsSettings.h"
#include "DistrictField.h"
#include "PolygonUtils.h"
#include "PCGContext.h"
#include "PCGPointData.h"

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
        const FVector Center = Settings->SeedParams.TownBounds.GetCenter();
        const float Radius = Settings->SeedParams.TownBounds.GetExtent().Size2D() * 0.4f;

        FDistrictField DistrictField;
        DistrictField.Params = Settings->DistrictParams;
        DistrictField.MarketCenter = Center;
        DistrictField.KeepCenter = Center + FVector(Radius * 0.45f, -Radius * 0.2f, 0);
        DistrictField.ChurchCenter = Center + FVector(-Radius * 0.12f, Radius * 0.1f, 0);

        UPCGPointData* OutParcels = NewObject<UPCGPointData>();
        TArray<FPCGPoint>& OutPoints = OutParcels->GetMutablePoints();

        for (const FPCGPoint& StreetPoint : Streets->GetPoints())
        {
            const FVector StreetPos = StreetPoint.Transform.GetLocation();
            const bool bCore = FVector::Dist2D(StreetPos, Center) < Radius * Settings->CoreRadiusFactor;
            const int32 ParcelCount = bCore ? Rand.RandRange(5, 10) : Rand.RandRange(2, 6);

            for (int32 I = 0; I < ParcelCount; ++I)
            {
                const float Frontage = Rand.FRandRange(Settings->ParcelParams.BurgageFrontageRange.X, Settings->ParcelParams.BurgageFrontageRange.Y);
                const float Depth = bCore
                    ? Rand.FRandRange(Settings->ParcelParams.BurgageDepthRange.X * 0.65f, Settings->ParcelParams.BurgageDepthRange.Y)
                    : Rand.FRandRange(1500.f, Settings->ParcelParams.BurgageDepthRange.Y * 0.45f);

                const FVector2D ParcelCenter(StreetPos.X + Rand.FRandRange(-700.f, 700.f), StreetPos.Y + Rand.FRandRange(-700.f, 700.f));
                const float Yaw = Rand.FRandRange(0.f, 180.f);
                TArray<FVector2D> Poly = MedievalPolygonUtils::MakeRectangle(ParcelCenter, FVector2D(Frontage, Depth), Yaw);

                if (!bCore)
                {
                    Poly = MedievalPolygonUtils::JitterPolygon(Poly, Rand, 80.f);
                }

                if (!MedievalPolygonUtils::IsSimplePolygon(Poly))
                {
                    continue;
                }

                const EMedievalDistrictType District = DistrictField.SampleDistrict(FVector(ParcelCenter, 0), Radius, Rand);
                FPCGPoint& ParcelPoint = OutPoints.AddDefaulted_GetRef();
                ParcelPoint.Transform = FTransform(FVector(ParcelCenter, 0));
                ParcelPoint.BoundsMin = FVector(-Frontage * 0.5f, -Depth * 0.5f, 0);
                ParcelPoint.BoundsMax = FVector(Frontage * 0.5f, Depth * 0.5f, 200.f);
                ParcelPoint.Seed = Rand.RandRange(1, MAX_int32);
                ParcelPoint.MetadataEntry = OutParcels->MutableMetadata()->AddEntry();

                const float Wealth = DistrictField.WealthAt(FVector(ParcelCenter, 0), Radius);
                const float Mixed = DistrictField.MixedUseAt(FVector(ParcelCenter, 0), Radius);
                OutParcels->MutableMetadata()->SetStringValue(ParcelPoint.MetadataEntry, TEXT("ParcelPolygon"), MedievalPolygonUtils::EncodePolygon(Poly));
                OutParcels->MutableMetadata()->SetStringValue(ParcelPoint.MetadataEntry, TEXT("District"), StaticEnum<EMedievalDistrictType>()->GetNameStringByValue((int64)District));
                OutParcels->MutableMetadata()->SetFloatValue(ParcelPoint.MetadataEntry, TEXT("Frontage"), Frontage);
                OutParcels->MutableMetadata()->SetFloatValue(ParcelPoint.MetadataEntry, TEXT("Depth"), Depth);
                OutParcels->MutableMetadata()->SetFloatValue(ParcelPoint.MetadataEntry, TEXT("Wealth"), Wealth);
                OutParcels->MutableMetadata()->SetFloatValue(ParcelPoint.MetadataEntry, TEXT("MixedUse"), Mixed);
                OutParcels->MutableMetadata()->SetBoolValue(ParcelPoint.MetadataEntry, TEXT("HasBackLane"), bCore && Depth > Settings->ParcelParams.BackLaneSpacing && Rand.FRand() < Settings->ParcelParams.BackLaneCreationProbability);
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
