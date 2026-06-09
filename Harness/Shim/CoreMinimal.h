// CoreMinimal.h  (HARNESS SHIM)
// -----------------------------------------------------------------------------
// Minimal stand-in for Unreal's CoreMinimal.h so the engine-agnostic generator
// sources (OrganicStreetGraph, OrganicStreetGenerator, OrganicParcelGenerator)
// can be compiled in the standalone layout harness (Harness/HarnessMain.cxx).
//
// This file is NEVER seen by Unreal Build Tool: Harness/Shim is not on the
// module include path, so in-engine builds resolve CoreMinimal.h normally.
// Only the harness compiles with -IHarness/Shim, which makes this shim win.
// -----------------------------------------------------------------------------
#pragma once

#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <utility>

#define MEDIEVAL_HARNESS 1

// -- Fundamental typedefs -------------------------------------------------------
typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;
typedef int64_t  int64;
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#define INDEX_NONE (-1)

#ifndef PI
#define PI (3.14159265358979323846f)
#endif
#ifndef TWO_PI
#define TWO_PI (6.28318530717958647692f)
#endif
#ifndef HALF_PI
#define HALF_PI (1.57079632679489661923f)
#endif
#ifndef BIG_NUMBER
#define BIG_NUMBER (3.4e+38f)
#endif
#ifndef SMALL_NUMBER
#define SMALL_NUMBER (1.e-8f)
#endif
#ifndef KINDA_SMALL_NUMBER
#define KINDA_SMALL_NUMBER (1.e-4f)
#endif

#define TEXT(x) x
#define UE_LOG(...) ((void)0)
#define check(expr) ((void)0)
#define ensure(expr) (!!(expr))

// -- UHT reflection macro no-ops -------------------------------------------------
#define UENUM(...)
#define USTRUCT(...)
#define UCLASS(...)
#define UINTERFACE(...)
#define UFUNCTION(...)
#define UPROPERTY(...)
#define UMETA(...)
#define GENERATED_BODY()
#define GENERATED_USTRUCT_BODY()

#define MoveTemp(x) std::move(x)
#define Forward std::forward

template <typename T> using TFunction = std::function<T>;

// -- TNumericLimits ---------------------------------------------------------------
template <typename T>
struct TNumericLimits
{
    static T Max()    { return std::numeric_limits<T>::max(); }
    static T Min()    { return std::numeric_limits<T>::min(); }
    static T Lowest() { return std::numeric_limits<T>::lowest(); }
};

// -- TArray ----------------------------------------------------------------------
template <typename T>
class TArray
{
public:
    std::vector<T> Data;

    TArray() {}
    TArray(std::initializer_list<T> Init) : Data(Init) {}

    int32 Num() const { return (int32)Data.size(); }
    bool  IsEmpty() const { return Data.empty(); }
    bool  IsValidIndex(int32 I) const { return I >= 0 && I < Num(); }

    decltype(auto) operator[](int32 I)       { return Data[(size_t)I]; }
    decltype(auto) operator[](int32 I) const { return Data[(size_t)I]; }

    decltype(auto) Last()       { return Data.back(); }
    decltype(auto) Last() const { return Data.back(); }

    int32 Add(const T& V) { Data.push_back(V); return Num() - 1; }
    int32 Add(T&& V)      { Data.push_back(std::move(V)); return Num() - 1; }
    int32 AddUnique(const T& V)
    {
        for (int32 I = 0; I < Num(); ++I) if (Data[(size_t)I] == V) return I;
        return Add(V);
    }
    T& AddDefaulted_GetRef() { Data.emplace_back(); return Data.back(); }
    int32 AddDefaulted(int32 Count = 1) { Data.resize(Data.size() + (size_t)Count); return Num() - Count; }

    template <typename... ArgsType>
    int32 Emplace(ArgsType&&... Args) { Data.emplace_back(std::forward<ArgsType>(Args)...); return Num() - 1; }

    void Append(const TArray<T>& Other) { Data.insert(Data.end(), Other.Data.begin(), Other.Data.end()); }

    void Reserve(int32 N) { Data.reserve((size_t)N); }
    void SetNum(int32 N)  { Data.resize((size_t)N); }
    void SetNumZeroed(int32 N) { Data.assign((size_t)N, T()); }
    void Init(const T& V, int32 N) { Data.assign((size_t)N, V); }
    void Empty(int32 Slack = 0) { Data.clear(); if (Slack > 0) Data.reserve((size_t)Slack); }
    void Reset(int32 Slack = 0) { Empty(Slack); }

    void RemoveAt(int32 I) { Data.erase(Data.begin() + I); }
    void RemoveAtSwap(int32 I)
    {
        Data[(size_t)I] = std::move(Data.back());
        Data.pop_back();
    }
    int32 Remove(const T& V)
    {
        int32 Removed = 0;
        for (int32 I = Num() - 1; I >= 0; --I)
            if (Data[(size_t)I] == V) { RemoveAt(I); ++Removed; }
        return Removed;
    }
    bool Contains(const T& V) const
    {
        return std::find(Data.begin(), Data.end(), V) != Data.end();
    }
    int32 Find(const T& V) const
    {
        auto It = std::find(Data.begin(), Data.end(), V);
        return It == Data.end() ? INDEX_NONE : (int32)(It - Data.begin());
    }

    template <typename Pred>
    void Sort(Pred P) { std::stable_sort(Data.begin(), Data.end(), P); }
    void Sort()       { std::stable_sort(Data.begin(), Data.end()); }

    auto begin()       { return Data.begin(); }
    auto end()         { return Data.end(); }
    auto begin() const { return Data.begin(); }
    auto end()   const { return Data.end(); }
};

// -- TSet --------------------------------------------------------------------------
template <typename T>
class TSet
{
public:
    std::set<T> Data;
    int32 Num() const { return (int32)Data.size(); }
    void  Add(const T& V) { Data.insert(V); }
    bool  Contains(const T& V) const { return Data.count(V) > 0; }
    void  Empty() { Data.clear(); }
    auto begin()       { return Data.begin(); }
    auto end()         { return Data.end(); }
    auto begin() const { return Data.begin(); }
    auto end()   const { return Data.end(); }
};

// -- FVector2D ----------------------------------------------------------------------
struct FVector2D
{
    float X = 0.f;
    float Y = 0.f;

    FVector2D() {}
    FVector2D(float InX, float InY) : X(InX), Y(InY) {}

    static const FVector2D ZeroVector;
    static const FVector2D UnitVector;

    FVector2D operator+(const FVector2D& V) const { return FVector2D(X + V.X, Y + V.Y); }
    FVector2D operator-(const FVector2D& V) const { return FVector2D(X - V.X, Y - V.Y); }
    FVector2D operator*(float S) const { return FVector2D(X * S, Y * S); }
    FVector2D operator/(float S) const { return FVector2D(X / S, Y / S); }
    FVector2D operator*(const FVector2D& V) const { return FVector2D(X * V.X, Y * V.Y); }
    FVector2D operator-() const { return FVector2D(-X, -Y); }
    FVector2D& operator+=(const FVector2D& V) { X += V.X; Y += V.Y; return *this; }
    FVector2D& operator-=(const FVector2D& V) { X -= V.X; Y -= V.Y; return *this; }
    FVector2D& operator*=(float S) { X *= S; Y *= S; return *this; }
    bool operator==(const FVector2D& V) const { return X == V.X && Y == V.Y; }
    bool operator!=(const FVector2D& V) const { return !(*this == V); }

    float Size() const { return std::sqrt(X * X + Y * Y); }
    float SizeSquared() const { return X * X + Y * Y; }

    FVector2D GetSafeNormal(float Tolerance = 1.e-8f) const
    {
        const float S = Size();
        return (S > Tolerance) ? FVector2D(X / S, Y / S) : FVector2D(0.f, 0.f);
    }
    bool Normalize(float Tolerance = 1.e-8f)
    {
        const float S = Size();
        if (S > Tolerance) { X /= S; Y /= S; return true; }
        X = 0.f; Y = 0.f; return false;
    }
    bool IsNearlyZero(float Tolerance = 1.e-4f) const
    {
        return std::fabs(X) <= Tolerance && std::fabs(Y) <= Tolerance;
    }

    static float DotProduct(const FVector2D& A, const FVector2D& B) { return A.X * B.X + A.Y * B.Y; }
    static float CrossProduct(const FVector2D& A, const FVector2D& B) { return A.X * B.Y - A.Y * B.X; }
    static float Distance(const FVector2D& A, const FVector2D& B) { return (B - A).Size(); }
    static float DistSquared(const FVector2D& A, const FVector2D& B) { return (B - A).SizeSquared(); }
};

inline FVector2D operator*(float S, const FVector2D& V) { return V * S; }

// -- FVector --------------------------------------------------------------------------
struct FVector
{
    float X = 0.f, Y = 0.f, Z = 0.f;

    FVector() {}
    FVector(float InX, float InY, float InZ) : X(InX), Y(InY), Z(InZ) {}
    explicit FVector(float V) : X(V), Y(V), Z(V) {}

    static const FVector ZeroVector;
    static const FVector UpVector;
    static const FVector ForwardVector;

    FVector operator+(const FVector& V) const { return FVector(X + V.X, Y + V.Y, Z + V.Z); }
    FVector operator-(const FVector& V) const { return FVector(X - V.X, Y - V.Y, Z - V.Z); }
    FVector operator*(float S) const { return FVector(X * S, Y * S, Z * S); }
    float Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
};

// -- FBox -----------------------------------------------------------------------------
struct FBox
{
    FVector Min, Max;
    FBox() {}
    FBox(const FVector& InMin, const FVector& InMax) : Min(InMin), Max(InMax) {}
};

// -- FString (token support only) -------------------------------------------------------
struct FString
{
    std::string S;
    FString() {}
    FString(const char* In) : S(In) {}
    static FString Printf(const char* Fmt, ...) { return FString(Fmt); }
};

// -- FMath -------------------------------------------------------------------------------
struct FMath
{
    template <typename T> static T Abs(T A) { return A < (T)0 ? -A : A; }
    template <typename T> static T Max(T A, T B) { return A > B ? A : B; }
    template <typename T> static T Min(T A, T B) { return A < B ? A : B; }
    template <typename T> static T Max3(T A, T B, T C) { return Max(Max(A, B), C); }
    template <typename T> static T Min3(T A, T B, T C) { return Min(Min(A, B), C); }
    template <typename T> static T Clamp(T V, T Lo, T Hi) { return V < Lo ? Lo : (V > Hi ? Hi : V); }
    template <typename T> static T Square(T V) { return V * V; }

    template <typename T, typename U>
    static T Lerp(const T& A, const T& B, U Alpha) { return (T)(A + (B - A) * Alpha); }

    static float Sqrt(float V) { return std::sqrt(V); }
    static float Pow(float A, float B) { return std::pow(A, B); }
    static float Exp(float V) { return std::exp(V); }
    static float Loge(float V) { return std::log(V); }
    static float Sin(float V) { return std::sin(V); }
    static float Cos(float V) { return std::cos(V); }
    static float Tan(float V) { return std::tan(V); }
    static float Asin(float V) { return std::asin(Clamp(V, -1.f, 1.f)); }
    static float Acos(float V) { return std::acos(Clamp(V, -1.f, 1.f)); }
    static float Atan2(float Y, float X) { return std::atan2(Y, X); }
    static float Fmod(float A, float B) { return std::fmod(A, B); }

    static int32 FloorToInt(float V) { return (int32)std::floor(V); }
    static int32 CeilToInt(float V) { return (int32)std::ceil(V); }
    static int32 RoundToInt(float V) { return (int32)std::lround(V); }

    static float DegreesToRadians(float Deg) { return Deg * (PI / 180.f); }
    static float RadiansToDegrees(float Rad) { return Rad * (180.f / PI); }

    static bool IsNearlyZero(float V, float Tolerance = 1.e-8f) { return Abs(V) <= Tolerance; }
    static bool IsNearlyEqual(float A, float B, float Tolerance = 1.e-8f) { return Abs(A - B) <= Tolerance; }

    static void SinCos(float* OutSin, float* OutCos, float V) { *OutSin = std::sin(V); *OutCos = std::cos(V); }

    // Classic Perlin gradient noise, output approximately in [-1, 1].
    static float PerlinNoise2D(const FVector2D& Location)
    {
        static const uint8 Perm[256] = {
            151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,
            8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
            35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,
            134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,
            55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,
            18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,
            250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,
            189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,
            172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,
            228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,
            107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,150,254,
            138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180 };

        auto Hash = [](int32 X, int32 Y) -> uint8
        {
            return Perm[(uint8)(Perm[(uint8)(X & 255)] + (Y & 255))];
        };
        auto Grad = [](uint8 H, float DX, float DY) -> float
        {
            switch (H & 7)
            {
            case 0: return  DX + DY;
            case 1: return  DX - DY;
            case 2: return -DX + DY;
            case 3: return -DX - DY;
            case 4: return  DX;
            case 5: return -DX;
            case 6: return  DY;
            default: return -DY;
            }
        };
        auto Fade = [](float T) { return T * T * T * (T * (T * 6.f - 15.f) + 10.f); };

        const int32 X0 = FloorToInt(Location.X);
        const int32 Y0 = FloorToInt(Location.Y);
        const float FX = Location.X - (float)X0;
        const float FY = Location.Y - (float)Y0;
        const float U = Fade(FX);
        const float V = Fade(FY);

        const float N00 = Grad(Hash(X0, Y0),         FX,       FY);
        const float N10 = Grad(Hash(X0 + 1, Y0),     FX - 1.f, FY);
        const float N01 = Grad(Hash(X0, Y0 + 1),     FX,       FY - 1.f);
        const float N11 = Grad(Hash(X0 + 1, Y0 + 1), FX - 1.f, FY - 1.f);

        const float NX0 = N00 + U * (N10 - N00);
        const float NX1 = N01 + U * (N11 - N01);
        return (NX0 + V * (NX1 - NX0)) * 1.4142f;
    }
};

// -- FRandomStream -----------------------------------------------------------------------
struct FRandomStream
{
    mutable uint32 State = 0;

    FRandomStream() {}
    explicit FRandomStream(int32 Seed) { Initialize(Seed); }

    void Initialize(int32 Seed) { State = (uint32)Seed * 2654435761u + 1013904223u; }

    float FRand() const
    {
        State = State * 1664525u + 1013904223u;
        // Take the high bits for better statistical quality.
        return (float)((State >> 8) & 0xFFFFFF) / 16777216.f;
    }
    float FRandRange(float InMin, float InMax) const { return InMin + (InMax - InMin) * FRand(); }
    int32 RandRange(int32 InMin, int32 InMax) const
    {
        const int32 Range = InMax - InMin + 1;
        return InMin + (Range > 0 ? (int32)(FRand() * (float)Range) % Range : 0);
    }
    int32 GetCurrentSeed() const { return (int32)State; }
};

// -- Algo --------------------------------------------------------------------------------
namespace Algo
{
    template <typename ContainerType>
    void Reverse(ContainerType& Container)
    {
        std::reverse(Container.begin(), Container.end());
    }
}

// -- Static member definitions (C++17 inline) -----------------------------------------------
inline const FVector2D FVector2D::ZeroVector(0.f, 0.f);
inline const FVector2D FVector2D::UnitVector(1.f, 1.f);
inline const FVector FVector::ZeroVector(0.f, 0.f, 0.f);
inline const FVector FVector::UpVector(0.f, 0.f, 1.f);
inline const FVector FVector::ForwardVector(1.f, 0.f, 0.f);

// -- UObject stand-ins ---------------------------------------------------------------------
class UObject {};
