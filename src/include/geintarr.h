#ifndef NV_GEINTARR_H
#define NV_GEINTARR_H

#ifndef unix
#include <stdlib.h>
#endif
#include "Nova.h"
#include "assert.h"

#include "nvarray.h"
typedef NvArray<int> NvGeIntArray;
typedef NvArray<Nova::IntPtr> NvGeIntPtrArray;

#if GE_LOCATED_NEW
GE_DLLEXPIMPORT
NvGe::metaTypeIndex NvGeGetMetaTypeIndex(NvGeIntArray* pT);
#endif
#endif
