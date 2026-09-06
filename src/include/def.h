#pragma once

#ifndef NVPAL_DEF_H
#define NVPAL_DEF_H

#if defined(_MSC_VER) 
    #ifdef  NVPAL_API
        #define   NVPAL_PORT _declspec(dllexport)
    #else
//don't use __declspec(dllimport) so that we can use the .objs with both static and dynamc linking
        #define   NVPAL_PORT
    #endif
#elif defined(__clang__)
    #ifdef  NVPAL_API
        #define   NVPAL_PORT __attribute__ ((visibility ("default")))
    #else
        #define   NVPAL_PORT
    #endif
#else
    #error Visual C++ or Clang compiler is required.
#endif

#if defined(NVPAL_API) || !defined(_NOVA_CROSS_PLATFORM_) || defined(NVPAL_TEST)
#define NV_NON_CROSS_PLATFORM_API
#endif



#endif
