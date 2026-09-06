#ifndef NV_GEGBL_H
#define NV_GEGBL_H

#include <stdlib.h>
#include "gedll.h"
#include "Nova.h"
#include "getol.h"

#include "gegblge.h"
#pragma pack (push, 8)

class NvGeVector3d;

struct
NvGeContext
{
    // System wide default tolerance.
    //
    GE_DLLDATAEXIMP static NvGeTol     gTol;

    GE_DLLDATAEXIMP static void (*gErrorFunc)();

#ifndef GELIB2D
    // Function to calculate a vector which is orthogonal to the given vector.
    //
    GE_DLLDATAEXIMP static void (*gOrthoVector)(const NvGeVector3d&,NvGeVector3d&);
#endif

    GE_DLLDATAEXIMP static void* (*gAllocMem)(size_t);
    GE_DLLDATAEXIMP static void  (*gFreeMem)(void*);

#ifdef GE_LOCATED_NEW
    GE_DLLDATAEXIMP static void* (*gAllocMemNear) (size_t, NvGe::metaTypeIndex, const void* );
    GE_DLLDATAEXIMP static void* (*gAllocMemNearVector) (size_t, NvGe::metaTypeIndex, unsigned int, const void* );
    GE_DLLDATAEXIMP static void (*gSetExternalStore) (const void* );
#endif
#ifndef NDEBUG
    GE_DLLDATAEXIMP static void (*gAssertFunc)(const NCHAR *condition, const NCHAR *filename,
                                    int lineNumber, const NCHAR *status);
#endif
};


#pragma pack (pop)
#endif // NV_GEGBL_H
