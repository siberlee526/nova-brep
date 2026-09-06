#pragma once

#ifndef ACPAL_DEF_H
#define ACPAL_DEF_H

#if defined(_MSC_VER) 
    #ifdef  ACPAL_API
        #define   ACPAL_PORT _declspec(dllexport)
    #else
//don't use __declspec(dllimport) so that we can use the .objs with both static and dynamc linking
        #define   ACPAL_PORT
    #endif
#elif defined(__clang__)
    #ifdef  ACPAL_API
        #define   ACPAL_PORT __attribute__ ((visibility ("default")))
    #else
        #define   ACPAL_PORT
    #endif
#else
    #error Visual C++ or Clang compiler is required.
#endif

#if defined(ACPAL_API) || !defined(_ADESK_CROSS_PLATFORM_) || defined(ACPAL_TEST)
#define AC_NON_CROSS_PLATFORM_API
#endif



#endif
