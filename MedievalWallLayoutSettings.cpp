#include "MedievalWallLayoutSettings.h"
#include "MedievalPCGToggle.h"
#include "MedievalTownGenerator.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

// ---------------------------------------------------------------------------
//  Helper: resolve AMedievalTownGenerator from PCG context
// ---------------------------------------------------------------------------
static AMedievalTownGenerator* ResolveTownGenerator(
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
//  Wall layout PCG element
// ---------------------------------------------------------------------------
class FMedievalWallLayoutElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalWallLayoutSettings* Settings =
            Context ? Cast<UMedievalWallLayoutSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context) return true;

        AMedievalTownGenerator* Gen =
            ResolveTownGenerator(Context, Settings->TownGeneratorActor);
        if (!Gen) return true;

        // ---- WallNodes output -----------------------------------------------
        // One PCG point per tower / gate node on the perimeter.
        UPCGPointData* OutNodes = NewObject<UPCGPointData>();
        UPCGMetadata* NMeta = OutNodes->Metadata;

        auto* NodeTypeAttr  = NMeta->FindOrCreateAttribute<int32>(TEXT("NodeType"),         0);
        auto* TowerRadAttr  = NMeta->FindOrCreateAttribute<float>(TEXT("TowerRadius"),      0.f);
        auto* TowerHAttr    = NMeta->FindOrCreateAttribute<float>(TEXT("TowerHeight"),      0.f);
        auto* GateWAttr     = NMeta->FindOrCreateAttribute<float>(TEXT("GateWidth"),        0.f);
        auto* MainGateAttr  = NMeta->FindOrCreateAttribute<float>(TEXT("bIsMainGate"),      0.f);
        auto* OutFacXAttr   = NMeta->FindOrCreateAttribute<float>(TEXT("OutwardFacingX"),   0.f);
        auto* OutFacYAttr   = NMeta->FindOrCreateAttribute<float>(TEXT("OutwardFacingY"),   0.f);
        auto* SegCWAttr     = NMeta->FindOrCreateAttribute<int32>(TEXT("SegmentIndexCW"),  -1);
        auto* SegCCWAttr    = NMeta->FindOrCreateAttribute<int32>(TEXT("SegmentIndexCCW"), -1);

        for (const FMedievalWallNode& Node : Gen->GetLayoutWallNodes())
        {
            // Rotation: yaw derived from OutwardFacing so meshes face away from town.
            float OutwardYaw = FMath::RadiansToDegrees(
                FMath::Atan2(Node.OutwardFacing.Y, Node.OutwardFacing.X));
            FRotator NodeRot(0.f, OutwardYaw, 0.f);

            FPCGPoint& Pt = OutNodes->GetMutablePoints().AddDefaulted_GetRef();
            Pt.Transform  = FTransform(NodeRot, Node.WorldPos);
            Pt.Seed       = Node.NodeIndex;
            Pt.Density    = Node.bIsMainGate ? 1.f : 0.5f;
            NMeta->InitializeOnSet(Pt.MetadataEntry);

            NodeTypeAttr->SetValue(Pt.MetadataEntry, (int32)Node.NodeType);
            TowerRadAttr->SetValue(Pt.MetadataEntry, Node.TowerRadius);
            TowerHAttr  ->SetValue(Pt.MetadataEntry, Node.TowerHeight);
            GateWAttr   ->SetValue(Pt.MetadataEntry, Node.GateWidth);
            MainGateAttr->SetValue(Pt.MetadataEntry, Node.bIsMainGate ? 1.f : 0.f);
            OutFacXAttr ->SetValue(Pt.MetadataEntry, Node.OutwardFacing.X);
            OutFacYAttr ->SetValue(Pt.MetadataEntry, Node.OutwardFacing.Y);
            SegCWAttr   ->SetValue(Pt.MetadataEntry, Node.SegmentIndexCW);
            SegCCWAttr  ->SetValue(Pt.MetadataEntry, Node.SegmentIndexCCW);
        }

        // ---- WallSegments output --------------------------------------------
        // One PCG point per terrain-following sample within each segment.
        // PCG steps between consecutive same-SegmentIndex points to place
        // wall section meshes that hug the ground.
        UPCGPointData* OutSegs = NewObject<UPCGPointData>();
        UPCGMetadata* SMeta = OutSegs->Metadata;

        auto* SegIdxAttr  = SMeta->FindOrCreateAttribute<int32>(TEXT("SegmentIndex"),    -1);
        auto* NodeAAttr   = SMeta->FindOrCreateAttribute<int32>(TEXT("NodeIndexA"),       -1);
        auto* NodeBAttr   = SMeta->FindOrCreateAttribute<int32>(TEXT("NodeIndexB"),       -1);
        auto* WallHAttr   = SMeta->FindOrCreateAttribute<float>(TEXT("WallHeight"),       0.f);
        auto* WallTAttr   = SMeta->FindOrCreateAttribute<float>(TEXT("WallThickness"),    0.f);
        auto* BattleAttr  = SMeta->FindOrCreateAttribute<float>(TEXT("bHasBattlements"),  0.f);
        auto* NormXAttr   = SMeta->FindOrCreateAttribute<float>(TEXT("OutwardNormalX"),   0.f);
        auto* NormYAttr   = SMeta->FindOrCreateAttribute<float>(TEXT("OutwardNormalY"),   0.f);
        auto* LenAttr     = SMeta->FindOrCreateAttribute<float>(TEXT("Length"),           0.f);
        auto* SampleIdxAttr = SMeta->FindOrCreateAttribute<int32>(TEXT("SampleIndex"),    0);
        auto* SampleCntAttr = SMeta->FindOrCreateAttribute<int32>(TEXT("SampleCount"),    0);

        // Wall tangent direction for rotation: perpendicular to outward normal (CCW 90°).
        for (const FMedievalWallSegment& Seg : Gen->GetLayoutWallSegments())
        {
            if (Seg.TerrainSamples.IsEmpty()) continue;

            // Wall runs perpendicular to the outward normal.
            FVector2D WallDir2D(-Seg.OutwardNormal.Y, Seg.OutwardNormal.X);
            float WallYaw = FMath::RadiansToDegrees(
                FMath::Atan2(WallDir2D.Y, WallDir2D.X));
            FRotator SegRot(0.f, WallYaw, 0.f);

            const int32 SampleCount = Seg.TerrainSamples.Num();
            for (int32 Si = 0; Si < SampleCount; ++Si)
            {
                FPCGPoint& Pt = OutSegs->GetMutablePoints().AddDefaulted_GetRef();
                Pt.Transform  = FTransform(SegRot, Seg.TerrainSamples[Si]);
                Pt.Seed       = Seg.SegmentIndex * 1000 + Si;
                Pt.Density    = Seg.bHasBattlements ? 1.f : 0.f;
                SMeta->InitializeOnSet(Pt.MetadataEntry);

                SegIdxAttr  ->SetValue(Pt.MetadataEntry, Seg.SegmentIndex);
                NodeAAttr   ->SetValue(Pt.MetadataEntry, Seg.NodeIndexA);
                NodeBAttr   ->SetValue(Pt.MetadataEntry, Seg.NodeIndexB);
                WallHAttr   ->SetValue(Pt.MetadataEntry, Seg.WallHeight);
                WallTAttr   ->SetValue(Pt.MetadataEntry, Seg.WallThickness);
                BattleAttr  ->SetValue(Pt.MetadataEntry, Seg.bHasBattlements ? 1.f : 0.f);
                NormXAttr   ->SetValue(Pt.MetadataEntry, Seg.OutwardNormal.X);
                NormYAttr   ->SetValue(Pt.MetadataEntry, Seg.OutwardNormal.Y);
                LenAttr     ->SetValue(Pt.MetadataEntry, Seg.Length);
                SampleIdxAttr->SetValue(Pt.MetadataEntry, Si);
                SampleCntAttr->SetValue(Pt.MetadataEntry, SampleCount);
            }
        }

        FPCGTaggedData& NodesOut  = Context->OutputData.TaggedData.Emplace_GetRef();
        NodesOut.Data             = OutNodes;
        NodesOut.Tags.Add(FName("WallNodes"));

        FPCGTaggedData& SegsOut   = Context->OutputData.TaggedData.Emplace_GetRef();
        SegsOut.Data              = OutSegs;
        SegsOut.Tags.Add(FName("WallSegments"));

        return true;
    }
};

// ---------------------------------------------------------------------------
TArray<FPCGPinProperties> UMedievalWallLayoutSettings::InputPinProperties() const
{
    return {};
}

TArray<FPCGPinProperties> UMedievalWallLayoutSettings::OutputPinProperties() const
{
    return {
        FPCGPinProperties(FName(TEXT("WallNodes")),    EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("WallSegments")), EPCGDataType::Point),
    };
}

FPCGElementPtr UMedievalWallLayoutSettings::CreateElement() const
{
    return MakeShared<FMedievalWallLayoutElement>();
}
#endif
