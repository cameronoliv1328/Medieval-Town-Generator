#include "MedievalCityMacroSettings.h"
#include "MedievalPCGCompat.h"
#if MEDIEVAL_HAS_PCG
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"
#endif
#include "DistrictField.h"

#if MEDIEVAL_HAS_PCG

class FMedievalCityMacroElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalCityMacroSettings* Settings = Context ? Cast<UMedievalCityMacroSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context) return true;

        FRandomStream Rand(Settings->SeedParams.Seed);
        const FVector Center = Settings->SeedParams.TownBounds.GetCenter();
        const float Radius = Settings->ArchetypeParams.bWalledTown ? Settings->ArchetypeParams.WallRadius : Settings->SeedParams.TownBounds.GetExtent().Size2D() * 0.35f;

        UPCGPointData* AnchorData = NewObject<UPCGPointData>();
        AnchorData->InitializeFromData(nullptr);
        TArray<FPCGPoint>& Points = AnchorData->GetMutablePoints();

        auto AddAnchor = [&](const FVector& Pos, const FString& Type)
        {
            FPCGPoint& P = Points.AddDefaulted_GetRef();
            P.Transform = FTransform(Pos);
            P.Seed = Rand.RandRange(1, MAX_int32);
            P.Density = 1.0;
            P.MetadataEntry = AnchorData->MutableMetadata()->AddEntry();
            AnchorData->MutableMetadata()->SetStringValue(P.MetadataEntry, TEXT("AnchorType"), Type);
        };

        const float MarketSize = Rand.FRandRange(Settings->ArchetypeParams.MarketSquareSizeRange.X, Settings->ArchetypeParams.MarketSquareSizeRange.Y);
        const FVector Market = Center + FVector(Rand.FRandRange(-Radius * 0.1f, Radius * 0.1f), Rand.FRandRange(-Radius * 0.1f, Radius * 0.1f), 0);
        const FVector Keep = Center + FVector(Radius * 0.55f, -Radius * 0.25f, 0);
        const FVector Church = Market + FVector(-Radius * 0.2f, Radius * 0.12f, 0);

        AddAnchor(Center, TEXT("TownCenter"));
        AddAnchor(Market, TEXT("Market"));
        AddAnchor(Keep, TEXT("Keep"));
        AddAnchor(Church, TEXT("Church"));

        for (int32 GateIndex = 0; GateIndex < Settings->ArchetypeParams.GateCount; ++GateIndex)
        {
            const float T = (2 * PI * GateIndex) / FMath::Max(1, Settings->ArchetypeParams.GateCount);
            const FVector Gate = Center + FVector(FMath::Cos(T), FMath::Sin(T), 0) * Radius;
            AddAnchor(Gate, TEXT("Gate"));
        }

        Context->OutputData.TaggedData.Emplace_GetRef().Data = AnchorData;
        return true;
    }
};

TArray<FPCGPinProperties> UMedievalCityMacroSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalCityMacroSettings::OutputPinProperties() const
{
    TArray<FPCGPinProperties> Pins;
    Pins.Emplace(FName(TEXT("Anchors")), EPCGDataType::Point);
    return Pins;
}

FPCGElementPtr UMedievalCityMacroSettings::CreateElement() const
{
    return MakeShared<FMedievalCityMacroElement>();
}

#endif

#if !MEDIEVAL_HAS_PCG
// PCG plugin/module not available in this build target; class remains data-only for compilation safety.
#endif
