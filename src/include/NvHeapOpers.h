#pragma once
#include "Nova.h"
#include "nvheapmanager.h"

class NvHeapOperators {
    public:

#undef new
#undef delete
        static void* operator new(size_t size) {
            void *p = ::acHeapAlloc(nullptr,size);
            if (p)
                return p;
            ADESK_UNREACHABLE;
        }

        static void* operator new[](size_t size) {
            void *p = ::acHeapAlloc(nullptr, size);
            if (p)
                return p;
            ADESK_UNREACHABLE;
        }

        static void* operator new(size_t size, const char *,
                                  int)
        {
            void * p = ::acHeapAlloc(nullptr, size);
            if (p)
                return p;
            ADESK_UNREACHABLE;
        }

        static void* operator new[](size_t size,
                                    const char *, int )
        {
            void *p = ::acHeapAlloc(nullptr, size);
            if (p)
                return p;
            ADESK_UNREACHABLE;
        }

        static void operator delete(void *p) {   
            if(p != NULL)
                ::acHeapFree(nullptr, p);
        }

        static void operator delete[](void *p) {   
            if(p != NULL)
                ::acHeapFree(nullptr, p);
        }

        // Unicode: leaving pFName as char for now
        static void operator delete(void *p, const char *,
                                    int )
        {
            if (p != NULL)
                ::acHeapFree(nullptr, p);
        }

        static void operator delete[](void *p,
                                      const char *, int )
        {
            if (p != NULL)
                ::acHeapFree(nullptr, p );
        }

};  // NvHeapOperators
