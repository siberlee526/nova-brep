#pragma once

#define NV_UNICODE 1

#if defined(__cplusplus) && defined(_MSC_VER) && !defined(_NATIVE_WCHAR_T_DEFINED)
#error Please use native wchar_t type (/Zc:wchar_t)
#endif

typedef wchar_t NCHAR;

#define _NVRX_T(x)      L ## x
#define NVRX_T(x)      _NVRX_T(x)

