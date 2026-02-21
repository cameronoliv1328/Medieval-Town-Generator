#pragma once

#include "CoreMinimal.h"

#if __has_include("PCGSettings.h")
    #include "PCGSettings.h"
    #define MEDIEVAL_HAS_PCG 1
    #define MEDIEVAL_PCG_SETTINGS_PARENT UPCGSettings
#else
    #include "UObject/Object.h"
    #define MEDIEVAL_HAS_PCG 0
    #define MEDIEVAL_PCG_SETTINGS_PARENT UObject
#endif
