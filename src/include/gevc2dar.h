#ifndef NV_GEVC2DAR_H
#define NV_GEVC2DAR_H

#include "Nova.h"
#include "assert.h"
#include "gevec2d.h"

#include "nvarray.h"
typedef NvArray<NvGeVector2d> NvGeVector2dArray;

#if GE_LOCATED_NEW
GE_DLLEXPIMPORT
NvGe::metaTypeIndex NvGeGetMetaTypeIndex(NvGeVector2dArray* pT);
#endif

#endif
