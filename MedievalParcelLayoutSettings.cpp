#include "MedievalParcelLayoutSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

static AMedievalTownGenerator* ResolveParcelGen(
    FPCGContext* Context, const TSoftObjectPtr<AActor>& Fallback)
{
    if (Context && Context->SourceComponent)
    {
        if (AMedievalTownGenerator* G =
                Cast<AMedievalTownGenerator>(Context->SourceComponent->GetOwner()))
            return G;
    }
    return Cast<AMedievalTownGenerator>(Fallback.Get());
}

// ---------------------------------------------------------------------------
class FMedievalParcelLayoutElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalParcelLayoutSettings* Settings =
            Context ? Cast<UMedievalParcelLayoutSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context) return true;

        AMedievalTownGenerator* Gen =
            ResolveParcelGen(Context, Settings->TownGeneratorActor);
        if (!Gen) return true;

        const float MinWealth  = Settings->MinWealthScore;
        const bool  bFilterWard = (Settings->WardFilter < 255);
        const uint8 WardFilter  = Settings->WardFilter;

        UPCGPointData* OutParcels = NewObject<UPCGPointData>();
        UPCGMetadata* Meta = OutParcels->Metadata;

        // ---- Create all typed attribute channels ----------------------------
        // Identity
        auto* LotIdxAttr     = Meta->FindOrCreateAttribute<int32>(TEXT("LotIndex"),        -1);
        // District / style
        auto* WardAttr        = Meta->FindOrCreateAttribute<int32>(TEXT("Ward"),             0);
        auto* FootprintAttr   = Meta->FindOrCreateAttribute<int32>(TEXT("FootprintType"),    0);
        auto* StyleAttr       = Meta->FindOrCreateAttribute<int32>(TEXT("BuildingStyle"),    0);
        auto* RoofAttr        = Meta->FindOrCreateAttribute<int32>(TEXT("RoofType"),         0);
        auto* FloorsAttr      = Meta->FindOrCreateAttribute<int32>(TEXT("NumFloors"),        1);
        // Geometry
        auto* FrontageAttr    = Meta->FindOrCreateAttribute<float>(TEXT("FrontageWidth"),   0.f);
        auto* DepthAttr       = Meta->FindOrCreateAttribute<float>(TEXT("LotDepth"),        0.f);
        auto* FacingXAttr     = Meta->FindOrCreateAttribute<float>(TEXT("StreetFacingX"),   0.f);
        auto* FacingYAttr     = Meta->FindOrCreateAttribute<float>(TEXT("StreetFacingY"),   0.f);
        // Terrain
        auto* FoundDepthAttr  = Meta->FindOrCreateAttribute<float>(TEXT("FoundationDepth"), 0.f);
        auto* SlopeAttr       = Meta->FindOrCreateAttribute<float>(TEXT("GroundSlopeDeg"),  0.f);
        // Distances
        auto* DistRiverAttr   = Meta->FindOrCreateAttribute<float>(TEXT("DistToRiver"),     0.f);
        auto* DistWallAttr    = Meta->FindOrCreateAttribute<float>(TEXT("DistToWall"),      0.f);
        auto* DistMarketAttr  = Meta->FindOrCreateAttribute<float>(TEXT("DistToMarket"),    0.f);
        // Flags
        auto* FacesRiverAttr  = Meta->FindOrCreateAttribute<float>(TEXT("bFacesRiver"),     0.f);
        auto* CornerAttr      = Meta->FindOrCreateAttribute<float>(TEXT("bIsCornerLot"),    0.f);
        auto* PartyWallAttr   = Meta->FindOrCreateAttribute<int32>(TEXT("PartyWallMask"),   0);
        // Wing
        auto* HasWingAttr     = Meta->FindOrCreateAttribute<float>(TEXT("bHasWing"),        0.f);
        auto* WingSizeXAttr   = Meta->FindOrCreateAttribute<float>(TEXT("WingSizeX"),       0.f);
        auto* WingSizeYAttr   = Meta->FindOrCreateAttribute<float>(TEXT("WingSizeY"),       0.f);
        auto* WingYawAttr     = Meta->FindOrCreateAttribute<float>(TEXT("WingYawOffset"),   0.f);

        // ---- Emit one point per parcel --------------------------------------
        for (const FMedievalParcel& Parcel : Gen->GetLayoutParcels())
        {
            // Apply optional filters before allocating point storage.
            if (Parcel.WealthScore < MinWealth) continue;
            if (bFilterWard && (uint8)Parcel.Ward != WardFilter) continue;

            // Lot yaw aligns +X with the street-facing direction.
            FRotator LotRot(0.f, Parcel.Yaw, 0.f);

            FPCGPoint& Pt = OutParcels->GetMutablePoints().AddDefaulted_GetRef();
            Pt.Transform  = FTransform(LotRot, Parcel.WorldCenter);
            Pt.Seed       = Parcel.LotIndex;
            Pt.Density    = Parcel.WealthScore;   // 0 = poorest, 1 = richest
            Meta->InitializeOnSet(Pt.MetadataEntry);

            LotIdxAttr   ->SetValue(Pt.MetadataEntry, Parcel.LotIndex);
            WardAttr     ->SetValue(Pt.MetadataEntry, (int32)Parcel.Ward);
            FootprintAttr->SetValue(Pt.MetadataEntry, (int32)Parcel.FootprintType);
            StyleAttr    ->SetValue(Pt.MetadataEntry, (int32)Parcel.BuildingStyle);
            RoofAttr     ->SetValue(Pt.MetadataEntry, (int32)Parcel.RoofType);
            FloorsAttr   ->SetValue(Pt.MetadataEntry, Parcel.NumFloors);

            FrontageAttr  ->SetValue(Pt.MetadataEntry, Parcel.FrontageWidth);
            DepthAttr     ->SetValue(Pt.MetadataEntry, Parcel.LotDepth);
            FacingXAttr   ->SetValue(Pt.MetadataEntry, Parcel.StreetFacingDir.X);
            FacingYAttr   ->SetValue(Pt.MetadataEntry, Parcel.StreetFacingDir.Y);

            FoundDepthAttr->SetValue(Pt.MetadataEntry, Parcel.FoundationDepth);
            SlopeAttr     ->SetValue(Pt.MetadataEntry, Parcel.GroundSlopeDeg);

            DistRiverAttr ->SetValue(Pt.MetadataEntry, Parcel.DistToRiver);
            DistWallAttr  ->SetValue(Pt.MetadataEntry, Parcel.DistToWall);
            DistMarketAttr->SetValue(Pt.MetadataEntry, Parcel.DistToMarket);

            FacesRiverAttr->SetValue(Pt.MetadataEntry, Parcel.bFacesRiver   ? 1.f : 0.f);
            CornerAttr    ->SetValue(Pt.MetadataEntry, Parcel.bIsCornerLot  ? 1.f : 0.f);
            PartyWallAttr ->SetValue(Pt.MetadataEntry, (int32)Parcel.PartyWallMask);

            HasWingAttr   ->SetValue(Pt.MetadataEntry, Parcel.bHasWing      ? 1.f : 0.f);
            WingSizeXAttr ->SetValue(Pt.MetadataEntry, Parcel.WingSize.X);
            WingSizeYAttr ->SetValue(Pt.MetadataEntry, Parcel.WingSize.Y);
            WingYawAttr   ->SetValue(Pt.MetadataEntry, Parcel.WingYawOffset);
        }

        FPCGTaggedData& Out = Context->OutputData.TaggedData.Emplace_GetRef();
        Out.Data            = OutParcels;
        Out.Tags.Add(FName("Parcels"));

        return true;
    }
};

// ---------------------------------------------------------------------------
TArray<FPCGPinProperties> UMedievalParcelLayoutSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalParcelLayoutSettings::OutputPinProperties() const
{
    return {
        FPCGPinProperties(FName(TEXT("Parcels")), EPCGDataType::Point),
    };
}

FPCGElementPtr UMedievalParcelLayoutSettings::CreateElement() const
{
    return MakeShared<FMedievalParcelLayoutElement>();
}
#endif
