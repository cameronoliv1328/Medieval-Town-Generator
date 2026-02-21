#pragma once

// Default OFF to avoid UHT parent-class failures when the PCG plugin/module
// is not enabled in the host project's target/module.
#ifndef MEDIEVAL_ENABLE_PCG_NODES
#define MEDIEVAL_ENABLE_PCG_NODES 0
#endif

#if MEDIEVAL_ENABLE_PCG_NODES
#include "PCGSettings.h"
#endif
