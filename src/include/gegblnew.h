#ifndef NV_GEGBLNEW_H
#define NV_GEGBLNEW_H

#ifdef GE_LOCATED_NEW

#include "gegblge.h"
#include "gegetmti.h"

GE_DLLEXPIMPORT
void* operator new ( size_t, enum NvGe::metaTypeIndex, const void* );

GE_DLLEXPIMPORT
void* operator new ( size_t, enum NvGe::metaTypeIndex, unsigned int, const void* );

#define GENEWLOC( T, ptr)  new (NvGeGetMetaTypeIndex<T >(), (ptr)) T
#define GENEWLOCVEC( T, count, ptr) new (NvGeGetMetaTypeIndex<T >(),(count),(ptr)) T [ (count) ]
#else   //#ifdef GE_LOCATED_NEW
#define GENEWLOC( T, ptr)  new T
#define GENEWLOCVEC( T, count, ptr) new T [ (count) ]
#endif


#endif  // NV_GEGBLNEW_H
