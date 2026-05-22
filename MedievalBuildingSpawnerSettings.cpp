// MedievalBuildingSpawnerSettings.cpp

#include "MedievalBuildingSpawnerSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES

#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

// ---------------------------------------------------------------------------
// Helper: resolve the town generator from context or soft pointer fallback
// ---------------------------------------------------------------------------

static AMedievalTownGenerator* ResolveBuildingSpawnerGen(
    FPCGContext* Context,
    const TSoftObjectPtr<AActor>& Fallback)
{
    if (Context && Context->SourceComponent)
    {
        if (AMedievalTownGenerator* G =
                Cast<AMedievalTownGenerator>(Context->SourceComponent->GetOwner()))
        {
            return G;
        }
    }
    return Cast<AMedievalTownGenerator>(Fallback.Get());
}

// ---------------------------------------------------------------------------
// Attribute struct — one instance per UPCGPointData output
// ---------------------------------------------------------------------------

struct FBuildingAttrs
{
    // int32 attributes
    FPCGMetadataAttribute<int32>* LotIndex        = nullptr;
    FPCGMetadataAttribute<int32>* BuildingStyle   = nullptr;
    FPCGMetadataAttribute<int32>* RoofType        = nullptr;
    FPCGMetadataAttribute<int32>* Ward            = nullptr;
    FPCGMetadataAttribute<int32>* FootprintType   = nullptr;
    FPCGMetadataAttribute<int32>* NumFloors       = nullptr;
    FPCGMetadataAttribute<int32>* PartyWallMask   = nullptr;

    // float attributes
    FPCGMetadataAttribute<float>* FrontageWidth   = nullptr;
    FPCGMetadataAttribute<float>* LotDepth        = nullptr;
    FPCGMetadataAttribute<float>* FoundationDepth = nullptr;
    FPCGMetadataAttribute<float>* GroundSlopeDeg  = nullptr;
    FPCGMetadataAttribute<float>* DistToMarket    = nullptr;
    FPCGMetadataAttribute<float>* DistToWall      = nullptr;
    FPCGMetadataAttribute<float>* DistToRiver     = nullptr;
    FPCGMetadataAttribute<float>* bFacesRiver     = nullptr;
    FPCGMetadataAttribute<float>* bIsCornerLot    = nullptr;
    FPCGMetadataAttribute<float>* bHasWing        = nullptr;
    FPCGMetadataAttribute<float>* WingSizeX       = nullptr;
    FPCGMetadataAttribute<float>* WingSizeY       = nullptr;
    FPCGMetadataAttribute<float>* WingYawOffset   = nullptr;
};

static FBuildingAttrs MakeBuildingAttrs(UPCGMetadata* M)
{
    FBuildingAttrs A;

    // int32
    A.LotIndex      = M->FindOrCreateAttribute<int32>(TEXT("LotIndex"),      -1,    false, true);
    A.BuildingStyle = M->FindOrCreateAttribute<int32>(TEXT("BuildingStyle"),  0,     false, true);
    A.RoofType      = M->FindOrCreateAttribute<int32>(TEXT("RoofType"),       0,     false, true);
    A.Ward          = M->FindOrCreateAttribute<int32>(TEXT("Ward"),           0,     false, true);
    A.FootprintType = M->FindOrCreateAttribute<int32>(TEXT("FootprintType"),  0,     false, true);
    A.NumFloors     = M->FindOrCreateAttribute<int32>(TEXT("NumFloors"),      1,     false, true);
    A.PartyWallMask = M->FindOrCreateAttribute<int32>(TEXT("PartyWallMask"),  0,     false, true);

    // float
    A.FrontageWidth   = M->FindOrCreateAttribute<float>(TEXT("FrontageWidth"),   0.f, false, true);
    A.LotDepth        = M->FindOrCreateAttribute<float>(TEXT("LotDepth"),        0.f, false, true);
    A.FoundationDepth = M->FindOrCreateAttribute<float>(TEXT("FoundationDepth"), 0.f, false, true);
    A.GroundSlopeDeg  = M->FindOrCreateAttribute<float>(TEXT("GroundSlopeDeg"),  0.f, false, true);
    A.DistToMarket    = M->FindOrCreateAttribute<float>(TEXT("DistToMarket"),    0.f, false, true);
    A.DistToWall      = M->FindOrCreateAttribute<float>(TEXT("DistToWall"),      0.f, false, true);
    A.DistToRiver     = M->FindOrCreateAttribute<float>(TEXT("DistToRiver"),     0.f, false, true);
    A.bFacesRiver     = M->FindOrCreateAttribute<float>(TEXT("bFacesRiver"),     0.f, false, true);
    A.bIsCornerLot    = M->FindOrCreateAttribute<float>(TEXT("bIsCornerLot"),    0.f, false, true);
    A.bHasWing        = M->FindOrCreateAttribute<float>(TEXT("bHasWing"),        0.f, false, true);
    A.WingSizeX       = M->FindOrCreateAttribute<float>(TEXT("WingSizeX"),       0.f, false, true);
    A.WingSizeY       = M->FindOrCreateAttribute<float>(TEXT("WingSizeY"),       0.f, false, true);
    A.WingYawOffset   = M->FindOrCreateAttribute<float>(TEXT("WingYawOffset"),   0.f, false, true);

    return A;
}

// ---------------------------------------------------------------------------
// Element
// ---------------------------------------------------------------------------

class FMedievalBuildingSpawnerElement : public IPCGElement
{
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalBuildingSpawnerSettings* Settings =
            Context ? Cast<UMedievalBuildingSpawnerSettings>(Context->GetInputSettings()) : nullptr;

        if (!Settings || !Context)
        {
            return true;
        }

        AMedievalTownGenerator* Gen =
            ResolveBuildingSpawnerGen(Context, Settings->TownGeneratorActor);

        if (!Gen)
        {
            return true;
        }

        const TArray<FMedievalParcel>& Parcels = Gen->GetLayoutParcels();

        // -----------------------------------------------------------------
        // Create the three output point data objects
        // -----------------------------------------------------------------

        UPCGPointData* OutLandmarks   = NewObject<UPCGPointData>();
        UPCGPointData* OutResidential = NewObject<UPCGPointData>();
        UPCGPointData* OutService     = NewObject<UPCGPointData>();

        // -----------------------------------------------------------------
        // Create metadata attributes for each output (three separate sets)
        // -----------------------------------------------------------------

        FBuildingAttrs LAttr = MakeBuildingAttrs(OutLandmarks->Metadata);
        FBuildingAttrs RAttr = MakeBuildingAttrs(OutResidential->Metadata);
        FBuildingAttrs SAttr = MakeBuildingAttrs(OutService->Metadata);

        // -----------------------------------------------------------------
        // Classifier lambda: 0=Landmark, 1=Residential, 2=Service
        // -----------------------------------------------------------------

        auto Classify = [](EMedievalBuildingStyle S) -> int32
        {
            switch (S)
            {
                case EMedievalBuildingStyle::Church:
                case EMedievalBuildingStyle::Keep:
                    return 0; // Landmark

                case EMedievalBuildingStyle::Stable:
                case EMedievalBuildingStyle::Warehouse:
                case EMedievalBuildingStyle::Blacksmith:
                    return 2; // Service

                default:
                    return 1; // Residential (SmallCottage, TownHouse, GuildHall, TavernInn, Bakery)
            }
        };

        // -----------------------------------------------------------------
        // Iterate parcels
        // -----------------------------------------------------------------

        for (const FMedievalParcel& Parcel : Parcels)
        {
            const int32 Tier = Classify(Parcel.BuildingStyle);

            UPCGPointData* Out  = (Tier == 0) ? OutLandmarks
                                : (Tier == 2) ? OutService
                                :               OutResidential;

            UPCGMetadata*  Meta = Out->Metadata;

            FBuildingAttrs& Attr = (Tier == 0) ? LAttr
                                 : (Tier == 2) ? SAttr
                                 :               RAttr;

            // Compute transform components
            FRotator LotRot(0.f, Parcel.Yaw, 0.f);

            const float TotalHeight = Parcel.NumFloors * Settings->FloorHeight;
            FVector Scale(
                Parcel.FrontageWidth / Settings->BuildingUnitFrontage,
                Parcel.LotDepth      / Settings->BuildingUnitDepth,
                TotalHeight          / Settings->BuildingUnitHeight
            );

            // Add point
            FPCGPoint& Pt = Out->GetMutablePoints().AddDefaulted_GetRef();
            Pt.Transform  = FTransform(LotRot.Quaternion(), Parcel.WorldCenter, Scale);
            Pt.Seed       = Parcel.LotIndex;
            Pt.Density    = Parcel.WealthScore;

            Meta->InitializeOnSet(Pt.MetadataEntry);

            // Set int32 metadata
            Attr.LotIndex->SetValue(     Pt.MetadataEntry, Parcel.LotIndex);
            Attr.BuildingStyle->SetValue(Pt.MetadataEntry, static_cast<int32>(Parcel.BuildingStyle));
            Attr.RoofType->SetValue(     Pt.MetadataEntry, static_cast<int32>(Parcel.RoofType));
            Attr.Ward->SetValue(         Pt.MetadataEntry, static_cast<int32>(Parcel.Ward));
            Attr.FootprintType->SetValue(Pt.MetadataEntry, static_cast<int32>(Parcel.FootprintType));
            Attr.NumFloors->SetValue(    Pt.MetadataEntry, Parcel.NumFloors);
            Attr.PartyWallMask->SetValue(Pt.MetadataEntry, static_cast<int32>(Parcel.PartyWallMask));

            // Set float metadata
            Attr.FrontageWidth->SetValue(  Pt.MetadataEntry, Parcel.FrontageWidth);
            Attr.LotDepth->SetValue(       Pt.MetadataEntry, Parcel.LotDepth);
            Attr.FoundationDepth->SetValue(Pt.MetadataEntry, Parcel.FoundationDepth);
            Attr.GroundSlopeDeg->SetValue( Pt.MetadataEntry, Parcel.GroundSlopeDeg);
            Attr.DistToMarket->SetValue(   Pt.MetadataEntry, Parcel.DistToMarket);
            Attr.DistToWall->SetValue(     Pt.MetadataEntry, Parcel.DistToWall);
            Attr.DistToRiver->SetValue(    Pt.MetadataEntry, Parcel.DistToRiver);
            Attr.bFacesRiver->SetValue(    Pt.MetadataEntry, Parcel.bFacesRiver  ? 1.f : 0.f);
            Attr.bIsCornerLot->SetValue(   Pt.MetadataEntry, Parcel.bIsCornerLot ? 1.f : 0.f);
            Attr.bHasWing->SetValue(       Pt.MetadataEntry, Parcel.bHasWing     ? 1.f : 0.f);
            Attr.WingSizeX->SetValue(      Pt.MetadataEntry, Parcel.WingSize.X);
            Attr.WingSizeY->SetValue(      Pt.MetadataEntry, Parcel.WingSize.Y);
            Attr.WingYawOffset->SetValue(  Pt.MetadataEntry, Parcel.WingYawOffset);
        }

        // -----------------------------------------------------------------
        // Write tagged outputs
        // -----------------------------------------------------------------

        FPCGTaggedData& LandmarksOut   = Context->OutputData.TaggedData.Emplace_GetRef();
        LandmarksOut.Data              = OutLandmarks;
        LandmarksOut.Tags.Add(FName(TEXT("Landmarks")));

        FPCGTaggedData& ResidentialOut = Context->OutputData.TaggedData.Emplace_GetRef();
        ResidentialOut.Data            = OutResidential;
        ResidentialOut.Tags.Add(FName(TEXT("Residential")));

        FPCGTaggedData& ServiceOut     = Context->OutputData.TaggedData.Emplace_GetRef();
        ServiceOut.Data                = OutService;
        ServiceOut.Tags.Add(FName(TEXT("Service")));

        return true;
    }
};

// ---------------------------------------------------------------------------
// UMedievalBuildingSpawnerSettings — UPCGSettings interface
// ---------------------------------------------------------------------------

TArray<FPCGPinProperties> UMedievalBuildingSpawnerSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalBuildingSpawnerSettings::OutputPinProperties() const
{
    TArray<FPCGPinProperties> Pins;

    Pins.Emplace(FName(TEXT("Landmarks")),
                 EPCGDataType::Point,
                 /*bAllowMultipleConnections=*/true,
                 /*bAllowMultipleData=*/false);

    Pins.Emplace(FName(TEXT("Residential")),
                 EPCGDataType::Point,
                 /*bAllowMultipleConnections=*/true,
                 /*bAllowMultipleData=*/false);

    Pins.Emplace(FName(TEXT("Service")),
                 EPCGDataType::Point,
                 /*bAllowMultipleConnections=*/true,
                 /*bAllowMultipleData=*/false);

    return Pins;
}

FPCGElementPtr UMedievalBuildingSpawnerSettings::CreateElement() const
{
    return MakeShared<FMedievalBuildingSpawnerElement>();
}

#endif // MEDIEVAL_ENABLE_PCG_NODES
