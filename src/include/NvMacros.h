#ifndef NvMacros_HeaderFile
#define NvMacros_HeaderFile

//! Export macro for NvBREP library symbols.
//! NVBREP_DLL_EXPORTS is defined privately by CMake when building the library itself.
#if defined(_WIN32)
  #if defined(NVBREP_DLL_EXPORTS)
    #define NV_EXPORT __declspec(dllexport)
  #else
    #define NV_EXPORT __declspec(dllimport)
  #endif
#else
  #define NV_EXPORT
#endif

#endif // NvMacros_HeaderFile
