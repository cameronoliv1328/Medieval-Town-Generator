#include "MedievalWallSpawnerSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

static AMedievalTownGenerator* ResolveWallGen(FPCGContext* Context, const TSoftObjectPtr<AActor>& Fallback)
{
    if (Context && Context->SourceComponent)
        if (AMedievalTownGenerator* G = Cast<AMedievalTownGenerator>(Context->SourceComponent->GetOwner()))
            return G;
    return Cast<AMedievalTownGenerator>(Fallback.Get());
}

class FMedievalWallSpawnerElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalWallSpawnerSettings* Settings = Context
            ? Cast<UMedievalWallSpawnerSettings>(Context->GetInputSettings())
            : nullptr;
        if (!Settings || !Context)
            return true;

        AMedievalTownGenerator* Gen = ResolveWallGen(Context, Settings->TownGeneratorActor);
        if (!Gen)
            return true;

        const TArray<FMedievalWallSegment>& Segments = Gen->GetLayoutWallSegments();
        const TArray<FMedievalWallNode>&    Nodes    = Gen->GetLayoutWallNodes();

        // --------------------------------------------------------------------
        // WallSections
        // --------------------------------------------------------------------
        UPCGPointData* OutSections = NewObject<UPCGPointData>();
        UPCGMetadata*  MetaSect    = OutSections->Metadata;

        auto* SegIdxAttr    = MetaSect->FindOrCreateAttribute<int32>(TEXT("SegmentIndex"),  0);
        auto* WallHAttr     = MetaSect->FindOrCreateAttribute<float>(TEXT("WallHeight"),     0.f);
        auto* WallTAttr     = MetaSect->FindOrCreateAttribute<float>(TEXT("WallThickness"),  0.f);
        auto* SubLenAttr    = MetaSect->FindOrCreateAttribute<float>(TEXT("SubLength"),      0.f);

        for (const FMedievalWallSegment& Seg : Segments)
        {
            if (Seg.TerrainSamples.Num() < 2)
                continue;

            for (int32 i = 0; i < Seg.TerrainSamples.Num() - 1; ++i)
            {
                const FVector A      = Seg.TerrainSamples[i];
                const FVector B      = Seg.TerrainSamples[i + 1];
                const FVector Dir    = (B - A).GetSafeNormal();
                const float   SubLen = FVector::Dist(A, B);
                const FVector Mid    = (A + B) * 0.5f;
                const FRotator Rot   = Dir.Rotation();

                const FVector Scale(
                    SubLen               / Settings->WallSectionMeshUnitLength,
                    Seg.WallThickness    / Settings->WallMeshUnitThickness,
                    Seg.WallHeight       / Settings->WallMeshUnitHeight
                );

                FPCGPoint& Pt = OutSections->GetMutablePoints().AddDefaulted_GetRef();
                Pt.Transform  = FTransform(Rot.Quaternion(), Mid, Scale);
                Pt.Density    = Seg.bHasBattlements ? 1.f : 0.f;
                Pt.Seed       = Seg.SegmentIndex * 1000 + i;

                MetaSect->InitializeOnSet(Pt.MetadataEntry);
                SegIdxAttr->SetValue(Pt.MetadataEntry, Seg.SegmentIndex);
                WallHAttr ->SetValue(Pt.MetadataEntry, Seg.WallHeight);
                WallTAttr ->SetValue(Pt.MetadataEntry, Seg.WallThickness);
                SubLenAttr->SetValue(Pt.MetadataEntry, SubLen);
            }
        }

        // --------------------------------------------------------------------
        // Battlements
        // --------------------------------------------------------------------
        UPCGPointData* OutBattles = NewObject<UPCGPointData>();
        UPCGMetadata*  MetaBatt   = OutBattles->Metadata;

        auto* BSegIdxAttr  = MetaBatt->FindOrCreateAttribute<int32>(TEXT("SegmentIndex"), 0);
        auto* BSubLenAttr  = MetaBatt->FindOrCreateAttribute<float>(TEXT("SubLength"),    0.f);

        for (const FMedievalWallSegment& Seg : Segments)
        {
            if (!Seg.bHasBattlements)
                continue;
            if (Seg.TerrainSamples.Num() < 2)
                continue;

            for (int32 i = 0; i < Seg.TerrainSamples.Num() - 1; ++i)
            {
                const FVector A        = Seg.TerrainSamples[i];
                const FVector B        = Seg.TerrainSamples[i + 1];
                const FVector Dir      = (B - A).GetSafeNormal();
                const float   SubLen   = FVector::Dist(A, B);
                const FVector Mid      = (A + B) * 0.5f;
                const FVector BattMid  = Mid + FVector(0.f, 0.f, Seg.WallHeight);
                const FRotator Rot     = Dir.Rotation();

                const FVector Scale(
                    SubLen            / Settings->BattlementMeshUnitLength,
                    Seg.WallThickness / Settings->WallMeshUnitThickness,
                    1.f
                );

                FPCGPoint& Pt = OutBattles->GetMutablePoints().AddDefaulted_GetRef();
                Pt.Transform  = FTransform(Rot.Quaternion(), BattMid, Scale);
                Pt.Density    = 1.f;
                Pt.Seed       = Seg.SegmentIndex * 1000 + i;

                MetaBatt->InitializeOnSet(Pt.MetadataEntry);
                BSegIdxAttr->SetValue(Pt.MetadataEntry, Seg.SegmentIndex);
                BSubLenAttr->SetValue(Pt.MetadataEntry, SubLen);
            }
        }

        // --------------------------------------------------------------------
        // CornerTowers
        // --------------------------------------------------------------------
        UPCGPointData* OutCorners = NewObject<UPCGPointData>();
        UPCGMetadata*  MetaCorner = OutCorners->Metadata;

        auto* CNIdxAttr  = MetaCorner->FindOrCreateAttribute<int32>(TEXT("NodeIndex"),   0);
        auto* CTRadAttr  = MetaCorner->FindOrCreateAttribute<float>(TEXT("TowerRadius"), 0.f);
        auto* CTHgtAttr  = MetaCorner->FindOrCreateAttribute<float>(TEXT("TowerHeight"), 0.f);

        for (const FMedievalWallNode& Node : Nodes)
        {
            if (Node.NodeType != EMedievalWallNodeType::CornerTower)
                continue;

            const float    OutwardYaw = FMath::RadiansToDegrees(
                FMath::Atan2(Node.OutwardFacing.Y, Node.OutwardFacing.X));
            const FRotator Rot(0.f, OutwardYaw, 0.f);

            const FVector Scale(
                Node.TowerRadius / Settings->CornerTowerMeshUnitRadius,
                Node.TowerRadius / Settings->CornerTowerMeshUnitRadius,
                Node.TowerHeight / Settings->CornerTowerMeshUnitHeight
            );

            FPCGPoint& Pt = OutCorners->GetMutablePoints().AddDefaulted_GetRef();
            Pt.Transform  = FTransform(Rot.Quaternion(), Node.WorldPos, Scale);
            Pt.Density    = 0.75f;
            Pt.Seed       = Node.NodeIndex;

            MetaCorner->InitializeOnSet(Pt.MetadataEntry);
            CNIdxAttr->SetValue(Pt.MetadataEntry, Node.NodeIndex);
            CTRadAttr->SetValue(Pt.MetadataEntry, Node.TowerRadius);
            CTHgtAttr->SetValue(Pt.MetadataEntry, Node.TowerHeight);
        }

        // --------------------------------------------------------------------
        // GateTowers
        // --------------------------------------------------------------------
        UPCGPointData* OutGates = NewObject<UPCGPointData>();
        UPCGMetadata*  MetaGate = OutGates->Metadata;

        auto* GNIdxAttr    = MetaGate->FindOrCreateAttribute<int32>(TEXT("NodeIndex"),   0);
        auto* GWidthAttr   = MetaGate->FindOrCreateAttribute<float>(TEXT("GateWidth"),   0.f);
        auto* GTRadAttr    = MetaGate->FindOrCreateAttribute<float>(TEXT("TowerRadius"), 0.f);
        auto* GTHgtAttr    = MetaGate->FindOrCreateAttribute<float>(TEXT("TowerHeight"), 0.f);
        auto* GMainAttr    = MetaGate->FindOrCreateAttribute<float>(TEXT("bIsMainGate"), 0.f);
        auto* GSideAttr    = MetaGate->FindOrCreateAttribute<int32>(TEXT("Side"),        0);

        for (const FMedievalWallNode& Node : Nodes)
        {
            if (Node.NodeType != EMedievalWallNodeType::GateTower)
                continue;

            const FVector OutwardFacing3D(Node.OutwardFacing.X, Node.OutwardFacing.Y, 0.f);
            const FVector WallTangent(-Node.OutwardFacing.Y, Node.OutwardFacing.X, 0.f);
            const float   TwinOffset  = Node.TowerRadius * 1.8f;

            const float    OutwardYaw = FMath::RadiansToDegrees(
                FMath::Atan2(Node.OutwardFacing.Y, Node.OutwardFacing.X));
            const FRotator Rot(0.f, OutwardYaw, 0.f);

            const FVector Scale(
                Node.TowerRadius / Settings->GateTowerMeshUnitRadius,
                Node.TowerRadius / Settings->GateTowerMeshUnitRadius,
                Node.TowerHeight / Settings->GateTowerMeshUnitHeight
            );

            for (int32 Side : {-1, 1})
            {
                const FVector TowerCenter = Node.WorldPos + WallTangent * (TwinOffset * (float)Side);

                FPCGPoint& Pt = OutGates->GetMutablePoints().AddDefaulted_GetRef();
                Pt.Transform  = FTransform(Rot.Quaternion(), TowerCenter, Scale);
                Pt.Density    = Node.bIsMainGate ? 1.f : 0.5f;
                Pt.Seed       = Node.NodeIndex * 2 + (Side == -1 ? 0 : 1);

                MetaGate->InitializeOnSet(Pt.MetadataEntry);
                GNIdxAttr ->SetValue(Pt.MetadataEntry, Node.NodeIndex);
                GWidthAttr->SetValue(Pt.MetadataEntry, Node.GateWidth);
                GTRadAttr ->SetValue(Pt.MetadataEntry, Node.TowerRadius);
                GTHgtAttr ->SetValue(Pt.MetadataEntry, Node.TowerHeight);
                GMainAttr ->SetValue(Pt.MetadataEntry, Node.bIsMainGate ? 1.f : 0.f);
                GSideAttr ->SetValue(Pt.MetadataEntry, Side);
            }
        }

        // --------------------------------------------------------------------
        // Emit tagged outputs
        // --------------------------------------------------------------------
        {
            FPCGTaggedData& SectOut = Context->OutputData.TaggedData.Emplace_GetRef();
            SectOut.Data = OutSections;
            SectOut.Tags.Add(FName(TEXT("WallSections")));
        }
        {
            FPCGTaggedData& BattOut = Context->OutputData.TaggedData.Emplace_GetRef();
            BattOut.Data = OutBattles;
            BattOut.Tags.Add(FName(TEXT("Battlements")));
        }
        {
            FPCGTaggedData& CornerOut = Context->OutputData.TaggedData.Emplace_GetRef();
            CornerOut.Data = OutCorners;
            CornerOut.Tags.Add(FName(TEXT("CornerTowers")));
        }
        {
            FPCGTaggedData& GateOut = Context->OutputData.TaggedData.Emplace_GetRef();
            GateOut.Data = OutGates;
            GateOut.Tags.Add(FName(TEXT("GateTowers")));
        }

        return true;
    }
};

TArray<FPCGPinProperties> UMedievalWallSpawnerSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalWallSpawnerSettings::OutputPinProperties() const
{
    return {
        FPCGPinProperties(FName(TEXT("WallSections")),  EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("Battlements")),   EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("CornerTowers")),  EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("GateTowers")),    EPCGDataType::Point),
    };
}

FPCGElementPtr UMedievalWallSpawnerSettings::CreateElement() const
{
    return MakeShared<FMedievalWallSpawnerElement>();
}

#endif
