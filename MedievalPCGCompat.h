#pragma once

#include "CoreMinimal.h"

#if __has_include("PCGSettings.h")
    #include "PCGSettings.h"
    #define MEDIEVAL_HAS_PCG 1
#else
    #include "UObject/Object.h"
    #define MEDIEVAL_HAS_PCG 0
#endif

#if MEDIEVAL_HAS_PCG
using FMedievalPCGSettingsBase = UPCGSettings;
#else
using FMedievalPCGSettingsBase = UObject;
#endif
