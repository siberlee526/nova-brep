#pragma once

#include <stddef.h>
#include "Nova.h"
#include "def.h"


typedef void* NvHeapHandle;
NVPAL_PORT NvHeapHandle NvHeapCreate(Nova::UInt32 flags);
NVPAL_PORT void NvHeapDestroy(NvHeapHandle heap);
NVPAL_PORT void* NvHeapAlloc(NvHeapHandle heap, size_t size);
NVPAL_PORT void* acTryHeapAlloc(NvHeapHandle heap, size_t size);
NVPAL_PORT void NvHeapFree(NvHeapHandle heap, void* p);
NVPAL_PORT void* NvHeapReAlloc(NvHeapHandle heap, void* p, size_t size);
NVPAL_PORT size_t NvHeapSize(NvHeapHandle heap, const void* p);
NVPAL_PORT bool NvHeapValidate(NvHeapHandle heap, const void* p);

NVPAL_PORT void* acAllocAligned(size_t alignment, size_t size);
NVPAL_PORT void acFreeAligned(void* p);
NVPAL_PORT size_t acMsizeAligned(void* p, size_t alignment);
