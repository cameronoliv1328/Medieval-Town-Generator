#pragma once

#include "CoreMinimal.h"
#include "Components/SplineComponent.h"
#include "OrganicStreetGraph.generated.h"

UENUM(BlueprintType)
enum class EOrganicStreetType : uint8
{
    Primary,
    Secondary,
    Lane,
    Alley
};

USTRUCT(BlueprintType)
struct FStreetNode
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Position = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<int32> ConnectedEdges;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Importance = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsGate = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsMarket = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsBridgeNode = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsLandmark = false;
};

USTRUCT(BlueprintType)
struct FStreetEdge
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 A = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int32 B = INDEX_NONE;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    EOrganicStreetType StreetType = EOrganicStreetType::Secondary;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Width = 4.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float TargetSpeed = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float CurvatureCost = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    bool bIsBridge = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FVector> PolylinePoints;

    UPROPERTY(Transient)
    TObjectPtr<USplineComponent> GeneratedSplineRef = nullptr;
};

USTRUCT(BlueprintType)
struct FOrganicStreetGraph
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FStreetNode> Nodes;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    TArray<FStreetEdge> Edges;
};

class FOrganicStreetSpatialHash
{
public:
    explicit FOrganicStreetSpatialHash(float InCell = 1000.0f);

    void Insert(const FVector2D& P, int32 Idx);
    void Query(const FVector2D& P, float Radius, TArray<int32>& OutIndices) const;

private:
    FIntPoint ToCell(const FVector2D& P) const;

private:
    float Cell = 1000.0f;
    TMap<FIntPoint, TArray<int32>> Buckets;
};

namespace OrganicStreetGraphUtils
{
    int32 FindNearestNode(const FOrganicStreetGraph& Graph, const FVector& Position, float* OutDistance = nullptr);
    bool SegmentIntersection2D(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D);
    bool EdgeIntersectsExisting(const FOrganicStreetGraph& Graph, const FVector2D& A, const FVector2D& B, int32 IgnoreNodeA = INDEX_NONE, int32 IgnoreNodeB = INDEX_NONE);
    void BuildNodeAdjacency(const FOrganicStreetGraph& Graph, TArray<TArray<int32>>& OutAdjacency);
    void ExtractApproxBlocks(const FOrganicStreetGraph& Graph, TArray<TArray<int32>>& OutBlocks, int32 MaxBlockEdges = 6);
}
