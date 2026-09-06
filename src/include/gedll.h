#ifndef NV_GEDLL_H
#define NV_GEDLL_H

#include "Nova.h"
#if defined(_MSC_VER)
#pragma warning(disable:4251)
#pragma warning(disable:4273)
#pragma warning(disable:4275)
#endif

#ifdef  NVGE_INTERNAL
#define GE_DLLEXPIMPORT ADESK_EXPORT
#define GE_DLLDATAEXIMP __declspec(dllexport)
#else
//don't use __declspec(dllimport) so that we can use the .objs with both static and dynamc linking
#define GE_DLLEXPIMPORT
//data must use __declspec(dllimport) but it causes no name mangling issues
#define GE_DLLDATAEXIMP __declspec(dllimport)
#endif

#ifdef  ACGX_INTERNAL
#define GX_DLLEXPIMPORT __declspec(dllexport)
#else
//don't use __declspec(dllimport) so that we can use the .objs with both static and dynamc linking
#define GX_DLLEXPIMPORT
#endif


#endif // NV_GEDLL_H
