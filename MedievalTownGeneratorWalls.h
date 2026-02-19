#pragma once

#include "CoreMinimal.h"

enum class EWallModule : uint8;

namespace MTGWalls
{
    // Converts grammar token text to a wall module enum.
    EWallModule ModuleFromToken(const FString& Token);
}
