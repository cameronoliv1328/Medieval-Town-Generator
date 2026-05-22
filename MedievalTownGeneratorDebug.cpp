// =============================================================================
//  MedievalTownGeneratorDebug.cpp
//
//  DebugDrawLayout() — visualises the three Phase 1 boundary arrays as
//  coloured lines and shapes in the editor / in-game viewport so that layout
//  correctness can be verified before real PCG asset nodes are wired up.
//
//  Colour convention (matches the header doc comment):
//    Walls    — white lines (sections), cyan (CornerTower), gold (GateTower)
//    Streets  — white=Primary, yellow=Secondary, grey=Alley/Lane, orange=Bridge,
//               teal=Quay; spheres: green=Market, red=Gate, cyan=BridgeAbutment
//    Parcels  — one box per lot; ward colour:
//                 MarketWard=yellow   CraftsWard=orange   DocksWard=cyan
//                 NobleWard=purple    PoorWard=dark-grey  ChurchWard=white
//                 ResidentialWard=green  Outskirts=brown
// =============================================================================

#include "MedievalTownGenerator.h"
#include "DrawDebugHelpers.h"

// ---------------------------------------------------------------------------
//  Local helpers
// ---------------------------------------------------------------------------

namespace MTGDebug
{
    // Returns a colour for each ward type.
    static FColor WardColor(EMedievalWardType Ward)
    {
        switch (Ward)
        {
        case EMedievalWardType::MarketWard:      return FColor(255, 220,   0);  // yellow
        case EMedievalWardType::CraftsWard:      return FColor(255, 140,   0);  // orange
        case EMedievalWardType::DocksWard:       return FColor(  0, 200, 220);  // teal
        case EMedievalWardType::NobleWard:       return FColor(180,   0, 220);  // purple
        case EMedievalWardType::PoorWard:        return FColor( 90,  90,  90);  // dark grey
        case EMedievalWardType::ChurchWard:      return FColor(220, 220, 255);  // pale blue
        case EMedievalWardType::ResidentialWard: return FColor( 60, 180,  60);  // green
        default:                                 return FColor( 90,  60,  30);  // brown
        }
    }

    // Returns a colour for each road surface type.
    static FColor SurfaceColor(EMedievalRoadSurface S)
    {
        switch (S)
        {
        case EMedievalRoadSurface::MarketFlagstone: return FColor(255, 220,   0);  // gold
        case EMedievalRoadSurface::PrimaryCobble:   return FColor(255, 255, 255);  // white
        case EMedievalRoadSurface::SecondaryDirt:   return FColor(200, 200, 120);  // straw
        case EMedievalRoadSurface::AlleyDirt:       return FColor(140, 140, 140);  // grey
        case EMedievalRoadSurface::BridgeDeck:      return FColor(255, 140,  30);  // orange
        case EMedievalRoadSurface::QuaySetts:       return FColor(  0, 220, 200);  // teal
        default:                                    return FColor(200, 200, 200);
        }
    }
}

// ---------------------------------------------------------------------------
//  DebugDrawLayout
// ---------------------------------------------------------------------------

void AMedievalTownGenerator::DebugDrawLayout()
{
    UWorld* World = GetWorld();
    if (!World) return;

    const float  Dur    = DebugDrawDuration;
    const FVector Origin = GetActorLocation();

    // =========================================================================
    //  WALLS
    // =========================================================================

    // --- Segments: white polyline at base height, dashed top line at WallHeight
    for (const FMedievalWallSegment& Seg : LayoutWallSegments)
    {
        const int32 N = Seg.TerrainSamples.Num();
        for (int32 i = 0; i < N - 1; ++i)
        {
            const FVector& A = Seg.TerrainSamples[i];
            const FVector& B = Seg.TerrainSamples[i + 1];

            // Base line
            DrawDebugLine(World, A, B, FColor::White, false, Dur, 0, 8.f);

            // Top line (battle-walk elevation)
            FVector ATop = A + FVector(0, 0, Seg.WallHeight);
            FVector BTop = B + FVector(0, 0, Seg.WallHeight);
            FColor TopCol = Seg.bHasBattlements ? FColor(200, 200, 255) : FColor(160, 160, 160);
            DrawDebugLine(World, ATop, BTop, TopCol, false, Dur, 0, 4.f);

            // Vertical tick at each sample to show wall height
            DrawDebugLine(World, A, ATop, FColor(100, 100, 180), false, Dur, 0, 2.f);
        }
    }

    // --- Nodes: spheres sized to tower radius
    for (const FMedievalWallNode& Node : LayoutWallNodes)
    {
        FColor Col;
        switch (Node.NodeType)
        {
        case EMedievalWallNodeType::CornerTower: Col = FColor::Cyan;   break;
        case EMedievalWallNodeType::GateTower:   Col = FColor(255, 180, 0); break;  // gold
        default:                                 Col = FColor::White;  break;
        }

        DrawDebugSphere(World, Node.WorldPos, Node.TowerRadius * 0.6f, 10, Col, false, Dur, 0, 4.f);

        // Outward-facing arrow
        FVector ArrowEnd = Node.WorldPos + FVector(Node.OutwardFacing.X, Node.OutwardFacing.Y, 0.f) * Node.TowerRadius * 1.5f;
        DrawDebugLine(World, Node.WorldPos, ArrowEnd, Col, false, Dur, 0, 6.f);

        // Extra ring for main gate
        if (Node.bIsMainGate)
            DrawDebugSphere(World, Node.WorldPos, Node.TowerRadius * 1.1f, 12,
                            FColor::Yellow, false, Dur, 0, 6.f);

        // Label
        DrawDebugString(World, Node.WorldPos + FVector(0, 0, Node.TowerHeight + 50.f),
                        Node.NodeType == EMedievalWallNodeType::GateTower
                            ? (Node.bIsMainGate ? TEXT("MAIN GATE") : TEXT("Gate"))
                            : TEXT("Tower"),
                        nullptr, Col, Dur);
    }

    // =========================================================================
    //  STREETS
    // =========================================================================

    // --- Edges: polylines coloured by surface type
    for (const FMedievalStreetEdge& Edge : LayoutStreetEdges)
    {
        if (Edge.WorldPolyline.Num() < 2) continue;

        FColor LineCol = MTGDebug::SurfaceColor(Edge.Surface);
        const float Thickness = Edge.bIsBridge ? 10.f : (Edge.StreetType == EMedievalStreetType::Primary ? 6.f : 3.f);

        for (int32 i = 0; i < Edge.WorldPolyline.Num() - 1; ++i)
        {
            DrawDebugLine(World,
                Edge.WorldPolyline[i] + FVector(0, 0, 20.f),
                Edge.WorldPolyline[i + 1] + FVector(0, 0, 20.f),
                LineCol, false, Dur, 0, Thickness);
        }

        // Width indicator at midpoint
        if (Edge.WorldPolyline.Num() >= 2)
        {
            int32 Mid = Edge.WorldPolyline.Num() / 2;
            FVector MidPt = Edge.WorldPolyline[Mid] + FVector(0, 0, 20.f);
            FVector Dir = (Edge.WorldPolyline[Mid] - Edge.WorldPolyline[Mid - 1]).GetSafeNormal();
            FVector Lat = FVector::CrossProduct(Dir, FVector::UpVector).GetSafeNormal();
            DrawDebugLine(World, MidPt - Lat * Edge.Width * 0.5f,
                                 MidPt + Lat * Edge.Width * 0.5f,
                                 LineCol, false, Dur, 0, 3.f);
        }
    }

    // --- Nodes: spheres coloured by semantic type
    for (const FMedievalStreetNode& Node : LayoutStreetNodes)
    {
        FColor Col;
        float  Radius = 60.f;

        if (Node.bIsMarket)
        {
            Col    = FColor::Yellow;
            Radius = 220.f;
        }
        else if (Node.bIsGate)
        {
            Col    = FColor::Red;
            Radius = 140.f;
        }
        else if (Node.bIsBridgeAbutment)
        {
            Col    = FColor::Orange;
            Radius = 100.f;
        }
        else
        {
            switch (Node.IntersectionType)
            {
            case EMedievalIntersectionType::Terminus:  Col = FColor(100, 100, 100); Radius = 40.f;  break;
            case EMedievalIntersectionType::TJunction: Col = FColor(160, 200, 255); Radius = 60.f;  break;
            case EMedievalIntersectionType::FourWay:   Col = FColor(255, 255, 255); Radius = 80.f;  break;
            default:                                   Col = FColor(200, 200, 200); Radius = 50.f;  break;
            }
        }

        DrawDebugSphere(World, Node.WorldPos + FVector(0, 0, 30.f), Radius, 8, Col, false, Dur, 0, 4.f);

        if (Node.bIsMarket)
            DrawDebugString(World, Node.WorldPos + FVector(0, 0, 350.f),
                            TEXT("MARKET"), nullptr, FColor::Yellow, Dur);
    }

    // =========================================================================
    //  PARCELS
    // =========================================================================

    for (const FMedievalParcel& Parcel : LayoutParcels)
    {
        FColor BoxCol = MTGDebug::WardColor(Parcel.Ward);

        // Box representing lot footprint (centered at WorldCenter, oriented by Yaw)
        FVector HalfExt(Parcel.FrontageWidth * 0.45f, Parcel.LotDepth * 0.45f, 40.f);
        FRotator LotRot(0.f, Parcel.Yaw, 0.f);
        DrawDebugBox(World,
                     Parcel.WorldCenter + FVector(0, 0, 25.f),
                     HalfExt, LotRot.Quaternion(),
                     BoxCol, false, Dur, 0, 3.f);

        // Street-facing arrow (short line from front edge toward road)
        FVector FrontCenter = Parcel.WorldCenter
            + FVector(Parcel.StreetFacingDir.X, Parcel.StreetFacingDir.Y, 0.f)
              * Parcel.FrontageWidth * 0.5f;
        FVector ArrowTip = FrontCenter
            + FVector(Parcel.StreetFacingDir.X, Parcel.StreetFacingDir.Y, 0.f) * 120.f;
        DrawDebugLine(World,
                      FrontCenter + FVector(0, 0, 30.f),
                      ArrowTip    + FVector(0, 0, 30.f),
                      BoxCol, false, Dur, 0, 4.f);

        // Special flag markers
        if (Parcel.bIsCornerLot)
            DrawDebugSphere(World, Parcel.WorldCenter + FVector(0, 0, 60.f),
                            50.f, 6, FColor::White, false, Dur, 0, 2.f);

        if (Parcel.bFacesRiver)
            DrawDebugSphere(World, Parcel.WorldCenter + FVector(0, 0, 60.f),
                            50.f, 6, FColor::Cyan, false, Dur, 0, 2.f);
    }

    // =========================================================================
    //  SUMMARY LOG
    // =========================================================================

    UE_LOG(LogTemp, Log,
        TEXT("[MTGDebug] DebugDrawLayout: %d wall segs, %d wall nodes | "
             "%d street edges, %d street nodes | %d parcels"),
        LayoutWallSegments.Num(), LayoutWallNodes.Num(),
        LayoutStreetEdges.Num(),  LayoutStreetNodes.Num(),
        LayoutParcels.Num());
}
