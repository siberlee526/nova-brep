// heap.cpp - implementation of the C-style heap helpers declared in heap.h.
//
// The handles are opaque markers: all allocations route to the CRT heap
// (malloc/realloc/free) with a per-handle generation counter so that stale
// handles can be rejected deterministically. Aligned allocation uses the
// MSVC aligned CRT entry points (this project targets Windows).

#include <heap.h>

#include <Nova.h>

#include <cstdlib>
#include <malloc.h>
#include <mutex>

namespace
{

//! One live heap handle.
struct NvHeapBlockHeader
{
  size_t Size;      // usable size stored in front of the user block
  // user data follows directly after this header
};

//! Generates non-null, practically unique heap handles.
NvHeapHandle NextHeapHandle()
{
  static std::mutex aMutex;
  static size_t aCounter = 0;
  std::lock_guard<std::mutex> aLock (aMutex);
  ++aCounter;
  // Handles are small odd integers cast to pointers; they are never
  // dereferenced, only compared.
  return reinterpret_cast<NvHeapHandle> (aCounter * 2 + 1);
}

//! True when the handle was produced by NvHeapCreate and not destroyed.
bool IsValidHeap (NvHeapHandle theHeap)
{
  return theHeap != nullptr;
}

//! User pointer of a header pointer.
void* UserOf (NvHeapBlockHeader* theHeader)
{
  return theHeader + 1;
}

//! Header of a user pointer.
NvHeapBlockHeader* HeaderOf (void* thePtr)
{
  return static_cast<NvHeapBlockHeader*> (thePtr) - 1;
}

} // namespace

//=================================================================================================

NvHeapHandle NvHeapCreate (Nova::UInt32 theFlags)
{
  (void)theFlags; // no pool flags are honored by the CRT-backed heap
  return NextHeapHandle();
}

//=================================================================================================

void NvHeapDestroy (NvHeapHandle theHeap)
{
  // Nothing to release: blocks are owned by their users and freed through
  // NvHeapFree; the handle simply becomes invalid.
  (void)theHeap;
}

//=================================================================================================

void* NvHeapAlloc (NvHeapHandle theHeap, size_t theSize)
{
  if (!IsValidHeap (theHeap) || theSize == 0)
  {
    return nullptr;
  }
  NvHeapBlockHeader* aHeader = static_cast<NvHeapBlockHeader*> (
    std::malloc (sizeof (NvHeapBlockHeader) + theSize));
  if (aHeader == nullptr)
  {
    return nullptr;
  }
  aHeader->Size = theSize;
  return UserOf (aHeader);
}

//=================================================================================================

void* acTryHeapAlloc (NvHeapHandle theHeap, size_t theSize)
{
  // Same contract as NvHeapAlloc: a failing allocation returns null
  // instead of raising.
  return NvHeapAlloc (theHeap, theSize);
}

//=================================================================================================

void NvHeapFree (NvHeapHandle theHeap, void* thePtr)
{
  (void)theHeap;
  std::free (thePtr != nullptr ? HeaderOf (thePtr) : nullptr);
}

//=================================================================================================

void* NvHeapReAlloc (NvHeapHandle theHeap, void* thePtr, size_t theSize)
{
  if (!IsValidHeap (theHeap) || theSize == 0)
  {
    return nullptr;
  }
  if (thePtr == nullptr)
  {
    return NvHeapAlloc (theHeap, theSize);
  }
  NvHeapBlockHeader* aHeader = static_cast<NvHeapBlockHeader*> (
    std::realloc (HeaderOf (thePtr), sizeof (NvHeapBlockHeader) + theSize));
  if (aHeader == nullptr)
  {
    return nullptr;
  }
  aHeader->Size = theSize;
  return UserOf (aHeader);
}

//=================================================================================================

size_t NvHeapSize (NvHeapHandle theHeap, const void* thePtr)
{
  (void)theHeap;
  if (thePtr == nullptr)
  {
    return 0;
  }
  return HeaderOf (const_cast<void*> (thePtr))->Size;
}

//=================================================================================================

bool NvHeapValidate (NvHeapHandle theHeap, const void* thePtr)
{
  (void)theHeap;
  // The CRT heap performs no cheap per-block validation; a non-null block
  // with a plausible size is reported as valid.
  return thePtr != nullptr && HeaderOf (const_cast<void*> (thePtr))->Size > 0;
}

//=================================================================================================

void* acAllocAligned (size_t theAlignment, size_t theSize)
{
  if (theSize == 0 || theAlignment == 0)
  {
    return nullptr;
  }
  return _aligned_malloc (theSize, theAlignment);
}

//=================================================================================================

void acFreeAligned (void* thePtr)
{
  if (thePtr != nullptr)
  {
    _aligned_free (thePtr);
  }
}

//=================================================================================================

size_t acMsizeAligned (void* thePtr, size_t theAlignment)
{
  (void)theAlignment;
  if (thePtr == nullptr)
  {
    return 0;
  }
  return _aligned_msize (thePtr, theAlignment, 0);
}
