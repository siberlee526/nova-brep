#ifndef NV_GEVC3DAR_H
#define NV_GEVC3DAR_H

#include "Nova.h"
#include "assert.h"
#include "gevec3d.h"

#include "nvarray.h"
typedef NvArray<NvGeVector3d> NvGeVector3dArray;

#if GE_LOCATED_NEW
GE_DLLEXPIMPORT
NvGe::metaTypeIndex NvGeGetMetaTypeIndex(NvGeVector3dArray* pT);
#endif

#endif
