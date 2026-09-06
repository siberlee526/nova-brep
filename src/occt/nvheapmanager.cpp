// nvheapmanager.cpp - implementation of the stack-heap allocator hooks.
//
// The ARX stack heap pools allocations near a parent object to improve
// locality; this port routes to the CRT heap (malloc/realloc/free), which
// keeps the API contract (pointer stability within a scope) with plain
// global allocation semantics.

#include <nvheapmanager.h>

#include <cstdlib>

//=================================================================================================

void* acStackHeapAlloc (size_t theSize, const void* theParent)
{
  (void)theParent; // locality hint only; the CRT heap ignores it
  return theSize != 0 ? std::malloc (theSize) : nullptr;
}

//=================================================================================================

void* acStackHeapRealloc (void* thePtr, size_t theSize)
{
  if (theSize == 0)
  {
    std::free (thePtr);
    return nullptr;
  }
  return std::realloc (thePtr, theSize);
}

//=================================================================================================

void acStackHeapFree (void* thePtr)
{
  std::free (thePtr);
}
