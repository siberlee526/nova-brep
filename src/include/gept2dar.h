#ifndef NV_GEPT2DAR_H
#define NV_GEPT2DAR_H

#include "Nova.h"
#include "assert.h"
#include "gepnt2d.h"

#include "nvarray.h"
typedef NvArray<NvGePoint2d> NvGePoint2dArray;

#if GE_LOCATED_NEW
GE_DLLEXPIMPORT
NvGe::metaTypeIndex NvGeGetMetaTypeIndex(NvGePoint2dArray* pT);
#endif

#endif
