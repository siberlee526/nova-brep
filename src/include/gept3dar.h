#ifndef NV_GEPT3DAR_H
#define NV_GEPT3DAR_H

#include "Nova.h"
#include "assert.h"
#include "gepnt3d.h"

#include "nvarray.h"
typedef NvArray<NvGePoint3d> NvGePoint3dArray;

#if GE_LOCATED_NEW
GE_DLLEXPIMPORT
NvGe::metaTypeIndex NvGeGetMetaTypeIndex(NvGePoint3dArray* pT);
#endif

#endif
