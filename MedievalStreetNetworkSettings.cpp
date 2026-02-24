#include "MedievalStreetNetworkSettings.h"
#include "MedievalPCGToggle.h"

#if MEDIEVAL_ENABLE_PCG_NODES
#include "OrganicStreetGenerator.h"
#include "PCGContext.h"
#include "PCGPointData.h"
#include "PCGPin.h"

class FPCG_OrganicStreetNetworkElement : public IPCGElement
{
public:
    virtual bool ExecuteInternal(FPCGContext* Context) const override
    {
        const UMedievalStreetNetworkSettings* Settings =
            Context ? Cast<UMedievalStreetNetworkSettings>(Context->GetInputSettings()) : nullptr;
        if (!Settings || !Context || Context->InputData.TaggedData.IsEmpty()) return true;

        const UPCGPointData* InAnchors =
            Cast<UPCGPointData>(Context->InputData.TaggedData[0].Data);
        if (!InAnchors) return true;

        FRandomStream Rand(Settings->SeedParams.Seed + 101);
        const FVector  Center     = Settings->SeedParams.TownBounds.GetCenter();
        const float    TownRadius = Settings->SeedParams.TownBounds.GetExtent().Size2D() * 0.5f;

        // Collect gate positions from outer anchors
        TArray<FVector2D> Gates;
        for (const FPCGPoint& P : InAnchors->GetPoints())
        {
            FVector Local = P.Transform.GetLocation() - Center;
            if (FVector2D(Local.X, Local.Y).Size() > TownRadius * 0.7f)
                Gates.Add(FVector2D(Local.X, Local.Y));
        }

        // Flat terrain (PCG context has no landscape access here)
        FOrganicTerrainQuery TQ;
        TQ.GetHeight         = [](FVector2D)        { return 0.f; };
        TQ.IsNearRiver       = [](FVector2D, float) { return false; };
        TQ.BridgeSuitability = [](FVector2D)        { return 0.f; };
        TQ.MaxGrade          = Settings->StreetParams.MaxSlopeDeg / 90.f;
        TQ.SlopePenalty      = Settings->StreetParams.SlopePenalty;
        TQ.WaterPenalty      = Settings->StreetParams.WaterPenalty;
        TQ.ValleyPreference  = Settings->StreetParams.ValleyPreference;

        FOrganicStreetConfig Cfg;
        Cfg.TownRadius           = TownRadius;
        Cfg.SecondaryAttractors  = Settings->SecondaryAttractorCount;
        Cfg.AStarCellSize        = TownRadius / 80.f;
        Cfg.MinSpacingCore       = Settings->OrganicParams.MinIntersectionSpacing;
        Cfg.MinSpacingOutskirts  = Settings->OrganicParams.MinIntersectionSpacing * 1.6f;
        Cfg.CoreRadiusFraction   = Settings->OrganicParams.CoreRadiusFraction;
        Cfg.PlazaChanceAt3Way    = Settings->OrganicParams.PlazaChanceAt3Way;
        Cfg.LoopChanceCore       = Settings->OrganicParams.LoopChanceCore;
        Cfg.LoopChanceOutskirts  = Settings->OrganicParams.LoopChanceOutskirts;
        Cfg.TIntersectionOffset  = Settings->OrganicParams.TIntersectionOffset;

        // Church / keep heuristic
        FVector2D ChurchPos(TownRadius * 0.18f,  TownRadius * 0.10f);
        FVector2D KeepPos  (-TownRadius * 0.22f, TownRadius * 0.14f);

        FOrganicStreetGenerator Gen(Cfg, TQ, Rand);
        FOrganicStreetGraph Graph = Gen.Generate(Gates, {}, ChurchPos, KeepPos);

        // -- Emit Streets (one PCG point per street node) --------------------
        UPCGPointData* OutStreets = NewObject<UPCGPointData>();
        for (const FOrganicStreetNode& N : Graph.Nodes)
        {
            FPCGPoint& P = OutStreets->GetMutablePoints().AddDefaulted_GetRef();
            P.Transform  = FTransform(FVector(N.Position.X + Center.X,
                                               N.Position.Y + Center.Y,
                                               Center.Z));
            P.Seed       = Rand.RandRange(1, MAX_int32);
            P.Density    = N.Importance;
        }

        // -- Emit RoadCorridor (mid-points of all edges for building exclusion)
        UPCGPointData* OutCorridor = NewObject<UPCGPointData>();
        for (const FOrganicStreetEdge& E : Graph.Edges)
        {
            if (E.NodeA < 0 || E.Poly2D.IsEmpty()) continue;
            for (const FVector2D& P2 : E.Poly2D)
            {
                FPCGPoint& P = OutCorridor->GetMutablePoints().AddDefaulted_GetRef();
                P.Transform  = FTransform(FVector(P2.X + Center.X, P2.Y + Center.Y, Center.Z));
                P.Density    = static_cast<float>(E.StreetType) / 3.f;
            }
        }

        FPCGTaggedData& StreetsOut    = Context->OutputData.TaggedData.Emplace_GetRef();
        StreetsOut.Data               = OutStreets;
        StreetsOut.Tags.Add(FName("Streets"));

        FPCGTaggedData& CorridorOut   = Context->OutputData.TaggedData.Emplace_GetRef();
        CorridorOut.Data              = OutCorridor;
        CorridorOut.Tags.Add(FName("RoadCorridor"));

        return true;
    }
};

TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::InputPinProperties() const
{
    return { FPCGPinProperties(FName(TEXT("Anchors")), EPCGDataType::Point) };
}

TArray<FPCGPinProperties> UMedievalStreetNetworkSettings::OutputPinProperties() const
{
    return {
        FPCGPinProperties(FName(TEXT("Streets")),      EPCGDataType::Point),
        FPCGPinProperties(FName(TEXT("RoadCorridor")), EPCGDataType::Point)
    };
}

FPCGElementPtr UMedievalStreetNetworkSettings::CreateElement() const
{
    return MakeShared<FPCG_OrganicStreetNetworkElement>();
}
#endif
