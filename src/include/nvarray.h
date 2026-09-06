#ifndef NV_NVARRAY_H
#define NV_NVARRAY_H

#include "c11_Annex_K.h"
#include <memory>
#include <stdlib.h>
#include <algorithm>
#include <type_traits>
#include "Nova.h"

#ifdef ASSERT
#define NV_ARRAY_ASSERT ASSERT
#elif defined assert
#define NV_ARRAY_ASSERT assert
#elif defined _ASSERTE
#define NV_ARRAY_ASSERT _ASSERTE
#else
#define NV_ARRAY_ASSERT(T)
#endif

#pragma pack (push, 8)
#pragma push_macro("new")
#pragma push_macro("delete")
#undef new
#undef delete

#define NVARRAY_GROWTH_THRESHOLD 0x10000

// Helper function for asserting that copy/move params are valid.
// Shouldn't generate any code in production builds
template <class T> void NvArrayValidateParams(bool bSameBuffer,
                                              T* pDest, int nBufLen,
                                              const T * pSource, int nCount);

// If the contained class T can be safely copied by the memcpy operator you
// should use the NvArrayMemCopyReallocator template with the array.
// If the class you intend to contain requires the use of operator=() for
// the copying during reallocation you should use NvArrayObjectCopyReallocator
//
// The default copy behaviour is based on the is_trivial trait of the type T
//

// This reallocator assumes that we can just do a memcpy (or memmove) for all
// copying and moving operations
template <class T> class NvArrayMemCopyReallocator
{
public:
    static void copyItems(T* pDest, int nBufLen, const T * pSource, int nCount)
    {
        NvArrayValidateParams<T>(false, pDest, nBufLen, pSource, nCount);
        if (nCount > 0)
           memcpy_s(pDest, nBufLen * sizeof(T), pSource, nCount * sizeof(T));
    }
    static void moveItems(T* pDest, int nBufLen, T * pSource, int nCount,
                          bool bSameBuffer)
    {
        NvArrayValidateParams<T>(bSameBuffer, pDest, nBufLen, pSource, nCount);
        if (nCount > 0) {
            // If bSameBuffer is true, then we are moving elements left or right in a buffer,
            // and so we have to do that in a safe way without clobbering source data.
            if (bSameBuffer)
                memmove_s(pDest, nBufLen * sizeof(T), pSource, nCount * sizeof(T));
            else
                memcpy_s(pDest, nBufLen * sizeof(T), pSource, nCount * sizeof(T));
        }
    }
};

// This reallocator copies and moves actual T objects, so assgt operators are called.
template <class T> class NvArrayObjectCopyReallocator
{
public:
    // Copy from source to existing items, using copy assignment operator
    static void copyItems(T* pDest, int nBufLen, const T * pSource, int nCount)
    {
        NvArrayValidateParams<T>(false, pDest, nBufLen, pSource, nCount);
        while (nCount--) {
            *pDest = *pSource;
            pDest++;
            pSource++;
        }
    }

    // Move from source to initialized items, using move assignment operator
    static void moveItems(T* pDest, int nBufLen, T * pSource, int nCount,
                          bool bSameBuffer)
    {
        NvArrayValidateParams<T>(bSameBuffer, pDest, nBufLen, pSource, nCount);
        while (nCount--) {
            *pDest = std::move(*pSource);
            pDest++;
            pSource++;
        }
    }
};

// define allocator type for passing as default arg to the NvArray template
template<typename T, bool>
struct NvArrayItemCopierSelector;

template<typename T>
struct NvArrayItemCopierSelector<T, false>
{
    typedef NvArrayObjectCopyReallocator<T> allocator;
};

template<typename T>
struct NvArrayItemCopierSelector<T, true>
{
    typedef NvArrayMemCopyReallocator<T> allocator;
};

template <typename T, typename R = typename NvArrayItemCopierSelector<T, std::is_trivial<T>::value>::allocator  > class NvArray
{

public:
    // ctor is explicit to disallow uses like:  NvArray<int> arr = 1;
    //
    explicit NvArray(int initPhysicalLength = 0, int initGrowLength = 8);
    NvArray(const NvArray<T,R>&);
    NvArray(NvArray<T,R>&&);
    ~NvArray();

    typedef T Type;
    typedef R Allocator;
    // Maximum theoretical number of elements in this array
    constexpr static int maxLength() { return 0x7fffffff / sizeof(T); }
  
    // Useful for validating that an NvArray uses the efficient copy method.
    // E.g.: static_assert(NvArray<MyType>::eUsesMemCopy, "NvArray<MyType> uses slow copy!");
    enum {eUsesMemCopy = std::is_same<R, NvArrayMemCopyReallocator<T> >::value};

    // Assignment and == operators.
    //
    NvArray<T,R>&         operator =  (const NvArray<T,R>&);
    NvArray<T,R>&         operator =  (NvArray<T,R>&&);
    bool                operator == (const NvArray<T,R>&) const;

    // Indexing into the array.
    //
    T&                  operator [] (int);
    const T &           operator [] (int) const;

    // More access to array-elements.
    //
    const T &             at          (int index) const;
          T &             at          (int index);
    NvArray<T,R>&         setAt       (int index, const T& value);
    NvArray<T,R>&         setAll      (const T& value);
    T&                  first       ();
    const T &           first       () const;
    T&                  last        ();
    const T &           last        () const;

    // Adding array-elements with copy semantics.
    //
    // Return the index of the new last element
    int                   append      (const T& value);

    NvArray<T,R>&         append      (const NvArray<T,R>& array);

    // Adding array-elements with move semantics.
    //
    int                   append      (T&& value);
    int                   appendMove  (T& value);
    NvArray<T,R>&         appendMove  (NvArray<T,R>& array);

    // Append the value to the array n times.  E.g.:  arr.appendRep(v1, 7);
    NvArray<T,R>&         appendRep   (const T& value, int nCount);

    // Variadic method allows appending an arbitrary list of values
    // E.g.:  arr.appendList(v1, v2, v3);
    template<typename... TV> NvArray<T,R> & appendList (const TV & ... args)
    {
        const auto nNumArgs = sizeof...(TV);    // how many vals in the list
        // Make enough room for the vals
        const int nOldLen = this->mLogicalLen;
        this->setLogicalLength(mLogicalLen + nNumArgs);
        // Use private helper to assign vals to new slots
        return this->assignHelper(nOldLen, args...);
    }

    NvArray<T,R>&         insertAt    (int index, T&& value);
    NvArray<T,R>&         insertAt    (int index, const T& value);
    NvArray<T,R>&         insertAtMove(int index, T& value);

    // Removing array-elements.
    //
    NvArray<T,R>&         removeAt    (int index);
    bool                  remove      (const T& value, int start = 0);
    NvArray<T,R>&         removeFirst ();
    NvArray<T,R>&         removeLast  ();
    NvArray<T,R>&         removeAll   ();
    NvArray<T,R>&         removeSubArray (int startIndex, int endIndex);

    // Query about array-elements.
    //
    bool                contains    (const T& value, int start = 0) const;
    bool                find        (const T& value, int& foundAt,
                                     int start = 0) const;
    int                 find        (const T& value) const;
    int                 findFrom    (const T& value, int start) const;

    // Array length.
    //
    int                 length      () const;           // Same as logical length.
    bool                isEmpty     () const;
    int                 logicalLength() const;          // number of existing items
    NvArray<T,R>&       setLogicalLength(int);
    NvArray<T,R>&       setLogicalLength(int, const T& value);  // init new cells to value
    int                 physicalLength() const;         // current capacity in items
    NvArray<T,R>&       setPhysicalLength(int);

    // Automatic resizing.
    // Number of items by which to increase capacity
    //
    int                 growLength  () const;
    NvArray<T,R>&         setGrowLength(int);

    // Utility.
    //
    NvArray<T,R>&         reverse     ();
    NvArray<T,R>&         swap        (int i1, int i2);

    // Treat as simple array of T.
    //
    const T*            asArrayPtr  () const;
    T*                  asArrayPtr  ();

    // begin() and end() methods return iterators which allow things like
    // range based for loops, std::sort, std::for_each etc to use NvArrays
    // E.g.: for (const auto & elt : arr) sum += elt; 
    //
    T * begin() { return mpArray; }
    T * end() { return mpArray + mLogicalLen; }

    const T * begin() const { return mpArray; }
    const T * end() const { return mpArray + mLogicalLen; }

private:
    // Helper variadic for assigning a value list to a range of cells
    NvArray<T,R> & assignHelper (int)
    {
        // Gets called if no args are passed to appendList()
        return *this;
    }
    NvArray<T,R> & assignHelper (int nIndex, const T & value)
    {
        this->setAt(nIndex, value);
        return *this;
    }
    template<typename... TV> NvArray<T,R> & assignHelper (int nIndex, const T & value,
                                                          const TV & ... args)
    {
        this->assignHelper(nIndex, value);
        return this->assignHelper(nIndex + 1, args...);
    }
protected:
    T*                  mpArray {nullptr};
    int                 mPhysicalLen {0};   // Actual buffer length.
    int                 mLogicalLen {0};    // Number of items in the array.
    int                 mGrowLen {0};       // Buffer grows by this value.

    void                insertSpace(int nIndex);
    void                copyOtherIntoThis(const NvArray<T,R>& otherArray);
    void                moveOtherIntoThis(NvArray<T,R>& otherArray);
    bool                isValid     (int) const;
};

#pragma pack (pop)

#ifdef GE_LOCATED_NEW
#error NvArray.h doesn't expect GE_LOCATED_NEW!
#endif

#pragma pack (push, 8)

// Inline methods.

template <class T, class R> inline bool
NvArray<T,R>::contains(const T& value, int start) const
{ return this->findFrom(value, start) != -1; }

template <class T, class R> inline int
NvArray<T,R>::length() const
{ return mLogicalLen; }

template <class T, class R> inline bool
NvArray<T,R>::isEmpty() const
{ return mLogicalLen == 0; }

template <class T, class R> inline int
NvArray<T,R>::logicalLength() const
{ return mLogicalLen; }

template <class T, class R> inline int
NvArray<T,R>::physicalLength() const
{ return mPhysicalLen; }

template <class T, class R> inline int
NvArray<T,R>::growLength() const
{ return mGrowLen; }

template <class T, class R> inline const T*
NvArray<T,R>::asArrayPtr() const
{ return mpArray; }

template <class T, class R> inline T*
NvArray<T,R>::asArrayPtr()
{ return mpArray; }

template <class T, class R> inline bool
NvArray<T,R>::isValid(int i) const
{ return i >= 0 && i < mLogicalLen; }

template <class T, class R> inline T&
NvArray<T,R>::operator [] (int i)
{ NV_ARRAY_ASSERT(this->isValid(i)); return mpArray[i]; }

template <class T, class R> inline const T&
NvArray<T,R>::operator [] (int i) const
{ NV_ARRAY_ASSERT(this->isValid(i)); return mpArray[i]; }

template <class T, class R> inline T&
NvArray<T,R>::at(int i)
{ NV_ARRAY_ASSERT(this->isValid(i)); return mpArray[i]; }

template <class T, class R> inline const T&
NvArray<T,R>::at(int i) const
{ NV_ARRAY_ASSERT(this->isValid(i)); return mpArray[i]; }

template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::setAt(int i, const T& value)
{ NV_ARRAY_ASSERT(this->isValid(i)); mpArray[i] = value; return *this; }

template <class T, class R> inline T&
NvArray<T,R>::first()
{ NV_ARRAY_ASSERT(!this->isEmpty()); return mpArray[0]; }

template <class T, class R> inline const T&
NvArray<T,R>::first() const
{ NV_ARRAY_ASSERT(!this->isEmpty()); return mpArray[0]; }

template <class T, class R> inline T&
NvArray<T,R>::last()
{ NV_ARRAY_ASSERT(!this->isEmpty()); return mpArray[mLogicalLen-1]; }

template <class T, class R> inline const T&
NvArray<T,R>::last() const
{ NV_ARRAY_ASSERT(!this->isEmpty()); return mpArray[mLogicalLen-1]; }

template <class T, class R> inline int
NvArray<T,R>::append(const T& value)
{ insertAt(mLogicalLen, value); return mLogicalLen-1; }

template <class T, class R> inline NvArray<T,R> &
NvArray<T,R>::appendRep(const T& value, int nCount)
{
    NV_ARRAY_ASSERT(nCount > 0);
    if (nCount > 0)
        this->setLogicalLength(mLogicalLen + nCount, value);
    return *this;
}

template <class T, class R> inline int NvArray<T,R>::append(T && value)
{
    return this->appendMove(value);
}

template <class T, class R> inline int
NvArray<T,R>::appendMove(T& value)
{
    this->insertAtMove(mLogicalLen, value);
    return this->mLogicalLen - 1;
}

template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::removeFirst()
{ NV_ARRAY_ASSERT(!isEmpty()); return removeAt(0); }

template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::removeLast()
{ 
    NV_ARRAY_ASSERT(!isEmpty());
    if (!isEmpty())
        mLogicalLen--;
    return *this;
}

template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::removeAll()
{
    this->setLogicalLength(0);
    return *this;
}

template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::setGrowLength(int glen)
{
    NV_ARRAY_ASSERT(glen > 0);
    NV_ARRAY_ASSERT(glen <= maxLength());
    mGrowLen = glen;
    return *this;
}

template < class T, class R > inline
NvArray< T, R > ::NvArray(int physicalLength, int growLength) : mGrowLen(growLength)
{
    // Replacing is_pod with is_trivial. is_trivial should be a superset of is_pod
    static_assert(std::is_trivial<T>::value || !std::is_pod<T>::value, "is_pod but not is_trivial?");
    NV_ARRAY_ASSERT(mGrowLen > 0);
    NV_ARRAY_ASSERT(mGrowLen <= maxLength());
    NV_ARRAY_ASSERT(physicalLength >= 0);
    NV_ARRAY_ASSERT(physicalLength <= maxLength());
    if (physicalLength > 0)
        this->setPhysicalLength(physicalLength);
}

// Copy ctor. Similar to copy assignment operator.
//
template <class T, class R> inline
NvArray<T,R>::NvArray(const NvArray<T,R>& src) : mGrowLen(src.mGrowLen)
{
    this->copyOtherIntoThis(src);
}

// Move ctor
template <class T, class R> inline
NvArray<T,R>::NvArray(NvArray<T,R>&& src) : mGrowLen(src.mGrowLen)
{
    this->moveOtherIntoThis(src);
}

// Dtor
template <class T, class R> inline
NvArray<T,R>::~NvArray()
{
    if (mPhysicalLen > 0)
        this->setPhysicalLength(0); // frees up buffer
}

// Copy assignment operator.Similar to copy ctor
// The grow length of this array is not affected by this operation.
//
template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::operator = (const NvArray<T,R>& src)
{
    if (this != &src)
        this->copyOtherIntoThis(src);
                    return *this;
            }

// Move assignment operator
template <class T, class R> inline NvArray<T,R>&
NvArray<T,R>::operator = (NvArray<T,R>&& src)
{
    if (this != &src) {
        this->setPhysicalLength(0);     // destruct this one
        this->moveOtherIntoThis(src);
    }
    return *this;
}


// The equal to operator.  The equal to operator compares
// the data in two arrays.  If the logical length of the
// two arrays are the same and the corresponding entries of
// the two arrays are equal, true is returned. Otherwise,
// false is returned.
//
template <class T, class R> bool
NvArray<T,R>::operator == (const NvArray<T,R>& cpr) const
{
    if (mLogicalLen == cpr.mLogicalLen) {
        for (int i = 0; i < mLogicalLen; i++)
            if (mpArray[i] != cpr.mpArray[i])
                return false;
        return true;
    }
    return false;
}

// Sets all the elements within the logical-length of the array,
// (that is, elements 0..length()-1), to `value'.
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::setAll(const T& value)
{
    for (int i = 0; i < mLogicalLen; i++) {
        mpArray[i] = value;
    }
    return *this;
}

// Appends the `otherArray' to the end of this array.  The logical length of
// this array will increase by the logical length of the `otherArray'.
// Special case: appending to self, where otherArray == this. That works,
// because otherArray.mpArray gets updated by setPhysicalLength.
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::append(const NvArray<T,R>& otherArray)
{
    const int nOrigLogLen = this->mLogicalLen;
    // Save other array's original logical length in case we are appending to
    // ourselves. Then grow our logical (and physical, if necessary) length
    const int nOrigOtherLogLen = otherArray.mLogicalLen;
    this->setLogicalLength(nOrigLogLen + nOrigOtherLogLen);

    R::copyItems(mpArray + nOrigLogLen, mLogicalLen - nOrigLogLen,
                 otherArray.mpArray, nOrigOtherLogLen);
    
        return *this;
}

// Helper meethod called from copy ctor and copy asst oper
// Doesn't mess with this->mGrowLen
// this->mpArray is re-used, if it's big enough
template <class T, class R> inline void
NvArray<T,R>::copyOtherIntoThis(const NvArray<T,R>& otherArray)
{
    NV_ARRAY_ASSERT(this != &otherArray);
    // Create or grow our buffer if necessary to hold other array's contents
    this->setLogicalLength(otherArray.mLogicalLen);
    // It's okay to call copyItems with zero count
    R::copyItems(mpArray, mPhysicalLen, otherArray.mpArray, mLogicalLen);
}

// Helper meethod called from move ctor, move asst oper and appendMove
// Doesn't mess with this->mGrowLen
// Assumes this->mpArray has been destroyed (or not created yet)
template <class T, class R> inline void
NvArray<T,R>::moveOtherIntoThis(NvArray<T,R>& otherArray)
{
    NV_ARRAY_ASSERT(this != &otherArray);
    NV_ARRAY_ASSERT(this->mpArray == nullptr);  // make sure no leaks
    this->mpArray = otherArray.mpArray;
    this->mLogicalLen = otherArray.mLogicalLen;
    this->mPhysicalLen = otherArray.mPhysicalLen;
    otherArray.mpArray = nullptr;
    otherArray.mLogicalLen = 0;
    otherArray.mPhysicalLen = 0;
    otherArray.mGrowLen = 8;    // back to the default? (doesn't matter)
}

template <class T, class R> NvArray<T,R>&
NvArray<T,R>::appendMove(NvArray<T,R>& otherArray)
{
    // Can't move into ourselves!
    NV_ARRAY_ASSERT(this != &otherArray);
    if (this != &otherArray) {
        if (this->mLogicalLen == 0) {
            // Special case - if this one is empty, then it can simply take
            // over the entire buffer of the other one.
            this->setPhysicalLength(0);
            this->moveOtherIntoThis(otherArray);
        }
        else {
            const int nOrigLogLen = this->mLogicalLen;
            // Grow our logical (and physical, if necessary) length
            this->setLogicalLength(nOrigLogLen + otherArray.mLogicalLen);
    
            R::moveItems(mpArray + nOrigLogLen, mLogicalLen - nOrigLogLen,
                 otherArray.mpArray, otherArray.mLogicalLen, /*bSameBuffer*/false);
        }
    }
    return *this;
}

// Inserts `value' at `index'.  The value formerly at `index'
// gets moved to `index+1',  `index+1 gets moved to `index+2' and so on.
// Note that insertAt(length(), value) is equivalent to append(value).
// The logical length of the array will increase by one.  If the physical
// length is not long enough it will increase by the grow length (with the
// usual caveat about insufficient memory).
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::insertAt(int index, const T& value)
{
    NV_ARRAY_ASSERT(index >= 0);
    NV_ARRAY_ASSERT(index <= mLogicalLen);
    NV_ARRAY_ASSERT(mLogicalLen <= mPhysicalLen);
    if (index < 0 || index > mLogicalLen)
        return *this;   // error
    if (index == mLogicalLen && mLogicalLen < mPhysicalLen) {
        mpArray[index] = value; // it's safe to immediately assign it
        mLogicalLen++;
    }
    else {
        // make a copy in case value is coming from this array!
        // Must be non-const so we can move out of it
        T tmp(value);
        this->insertSpace(index);
        mpArray[index] = std::move(tmp);
    }
        return *this;
}

template <class T, class R> NvArray<T,R>&
NvArray<T,R>::insertAt(int index, T && value)   // move semantics
{
    return this->insertAtMove(index, value);
}

template <class T, class R> NvArray<T,R>&
NvArray<T,R>::insertAtMove(int index, T& value)
{
    NV_ARRAY_ASSERT(index >= 0);
    NV_ARRAY_ASSERT(index <= mLogicalLen);
    NV_ARRAY_ASSERT(mLogicalLen <= mPhysicalLen);
    if (index < 0 || index > mLogicalLen)
        return *this;   // error
    // If appending, and we don't need to grow the buffer, then it's easy
    if (index == mLogicalLen && mLogicalLen < mPhysicalLen) {
        mpArray[index] = std::move(value);
        mLogicalLen++;
    }
    else {
        // tmp is non-const, so we can move out of it
        T tmp(std::move(value));        // save the value, in case it's in our buffer
        this->insertSpace(index);
        mpArray[index] = std::move(tmp);
    }
    return *this;
    }

// helper for the insertAt() and insertAtMove() methods.
// called when we need to slide items up to make a hole, or when we want to
// append and the buffer is already maxed out
template <class T, class R> void NvArray<T,R>::insertSpace(int nIndex)
{
    // Grow logical (and maybe physical) buffer
    this->setLogicalLength(mLogicalLen + 1);

    if (nIndex < mLogicalLen - 1) {     // if not inserting at end
        NV_ARRAY_ASSERT(mLogicalLen >= 0);
        // Note: we don't call moveItems() here, because we're sliding items up.
        // moveItems() assumes we're sliding down, such as during remove.
        // Here, we need to start moving from the end and work our way down
        T* p = mpArray + mLogicalLen - 1;       // start at last item in the array
        T* const pSpace = mpArray + nIndex;     // points to the new space
        NV_ARRAY_ASSERT(p >= pSpace);
        do {
            // slide the items up to make a hole at nIndex
            // note: we use move semantics even when called from normal insertAt
            *p = std::move(*(p-1));     // slide the items up
        } while (--p != pSpace);
    }
}

// Removes the element at `index'.  The logical length will
// decrease by one.  `index' MUST BE within bounds.
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::removeAt(int index)
{
    NV_ARRAY_ASSERT(isValid(index));
    NV_ARRAY_ASSERT(mLogicalLen <= mPhysicalLen);
    NV_ARRAY_ASSERT(!isEmpty());
    if (isEmpty() || !isValid(index))
        return *this;

    // Shift array elements to the left if needed.
    //
    if (index < mLogicalLen - 1) {
        R::moveItems(mpArray + index, mPhysicalLen - index,
                     mpArray + index + 1, mLogicalLen - 1 - index,
                     /*bSameBuffer*/true);
    }
    mLogicalLen--;
    return *this;
}

// Removes all elements starting with 'startIndex' and ending with 'endIndex'
// The logical length will decrease by number of removed elements.
// Both `startIndex' and 'endIndex' MUST BE within bounds.
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::removeSubArray(int startIndex, int endIndex)
{
    NV_ARRAY_ASSERT(isValid(startIndex));
    NV_ARRAY_ASSERT(startIndex <= endIndex);

    if ( endIndex >= mLogicalLen - 1) {
        mLogicalLen = startIndex;       // deleting to the right end
        return *this;
    }

    // We didn't delete the right end, so shift remaining elements down
    //
    const int kNumToRemove = endIndex + 1 - startIndex;
    const int kNumToShift = mLogicalLen - 1 - endIndex;
    NV_ARRAY_ASSERT(kNumToShift >= 1);
    R::moveItems(mpArray + startIndex, mPhysicalLen - startIndex,
                 mpArray + endIndex + 1, kNumToShift,
                 /*bSameBuffer*/true);
    mLogicalLen -= kNumToRemove;
    return *this;
}

// Returns true if and only if the array contains `value' from
// index `start' onwards.  Returns, in `index', the first location
// that contains `value'.  The search begins at position `start'.
// `start' is supplied with a default value of `0', i.e., the
// beginning of the array.
//
template <class T, class R> bool
NvArray<T,R>::find(const T& value, int& index, int start) const
{
    const int nFoundAt = this->findFrom(value, start);
    if (nFoundAt == -1)
        return false;
    index = nFoundAt;
    return true;
}

template <class T, class R> int
NvArray<T,R>::find(const T& value) const
{
    return this->findFrom(value, 0);   // search from the beginning
}

template <class T, class R> int
NvArray<T,R>::findFrom(const T& value, int start) const
{
    NV_ARRAY_ASSERT(start >= 0);
    if (start < 0)
        return -1;
    for (int i = start; i < this->mLogicalLen; i++) {
        if (mpArray[i] == value)
            return i;
    }
    return -1;
}

// Allows you to set the logical length of the array.
// If you try to set the logical length to be greater than
// the physical length, then the array is grown to a
// reasonable size (thus increasing both the logical length
// AND the physical length).
// Also, the physical length will grow in growth length
// steps.
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::setLogicalLength(int n)
{
    NV_ARRAY_ASSERT(n >= 0);
    if (n < 0)
        n = 0;  // avoid going negative
    NV_ARRAY_ASSERT(n <= maxLength());
    if (n > mPhysicalLen) {

        const int growth = (mPhysicalLen * sizeof(T)) < NVARRAY_GROWTH_THRESHOLD ?
            mPhysicalLen : NVARRAY_GROWTH_THRESHOLD / sizeof(T);

        int minSize = mPhysicalLen + std::max<int>(growth, mGrowLen);
        if ( n > minSize)
            minSize = n;
        setPhysicalLength(minSize);
    }
    mLogicalLen = n;
    NV_ARRAY_ASSERT(mLogicalLen <= mPhysicalLen);
    return *this;
}

template <class T, class R> NvArray<T,R>&
NvArray<T,R>::setLogicalLength(int n, const T& value)
{
    const int nOldLen = this->mLogicalLen;
    this->setLogicalLength(n);
    for (int i = nOldLen; i < this->mLogicalLen; i++)
        this->mpArray[i] = value;
    return *this;
}

// Grows or shrinks the physical buffer by allocating a new buffer (if new size
// is not zero) and deleting the old buffer.
// Reduces logical length if it was previously > the new physical length.
// Uses move semantics to move old cells to corresponding new cells
// Uses placement new to initialize new cells above previous buffer length
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::setPhysicalLength(int n)
{
    NV_ARRAY_ASSERT(mPhysicalLen >= mLogicalLen);
    NV_ARRAY_ASSERT((mPhysicalLen == 0) == (mpArray == nullptr));
    NV_ARRAY_ASSERT(n >= 0);
    NV_ARRAY_ASSERT(n <= maxLength());
    if (n == mPhysicalLen || n < 0)
        return *this;   // nothing to do

    T* pOldArray = mpArray;
    const int nOldLen = mPhysicalLen;

    mPhysicalLen = n;   // could be growing or shrinking
    mpArray = nullptr;

    if (mPhysicalLen < mLogicalLen)
        mLogicalLen = mPhysicalLen;     // shrinking the array

    if (mPhysicalLen != 0) {
        // Allocate the new physical memory buffer.
        // This can cause an exception or return null, depending on set_new_handler 
        mpArray = static_cast<T *>(::operator new(sizeof(T) * mPhysicalLen));
        NV_ARRAY_ASSERT(mpArray != nullptr);
        if (mpArray == nullptr) {
            // If allocation failed, then set array to empty
            // Should we restore the previous buffer ptr and length values instead?
            mPhysicalLen = 0;
            mLogicalLen = 0;
        }
        else {
            // First do a placement new to initialize the new array items to
            // default value.
            // This should not generate any code if T doesn't have a default ctor
            // Note we don't say new(&mpArray[i]) because T may have an & operator
            // such as if it's a smart ptr class. See tfs bug 68838
            T *pNewBuf = mpArray;
            for (int i = 0; i < mPhysicalLen; i++, pNewBuf++)
                ::new(pNewBuf) T;       // placement new: calls default ctor

            // Now move the old values from the old buf to the new buf
            R::moveItems(mpArray, mPhysicalLen, pOldArray, mLogicalLen,
                         /*bSameBuffer*/false);   // move items (if any) to new buf
        }
    }

    // This for loop should not generate any code if T doesn't have a dtor
    for (int i = 0; i < nOldLen; i++)
        (pOldArray + i)->~T();   // placement delete: call the dtor

    // now free the raw memory
    ::operator delete(static_cast<void *>(pOldArray));

    return *this;
}

// Reverses the order of the array.  That is if you have two
// arrays, `a' and `b', then if you assign `a = b' then call
// `a.reverse()' then a[0] == b[n], a[1] == b[n-1],... a[n] == b[0].
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::reverse()
{
    for (int i = 0; i < mLogicalLen/2; i++) {
        // tmp is non-const, so we can move out of it
        T tmp = std::move(mpArray[i]);
        mpArray[i] = std::move(mpArray[mLogicalLen - 1 - i]);
        mpArray[mLogicalLen - 1 - i] = std::move(tmp);
    }
    return *this;
}

// Swaps the elements in `i1' and `i2'.
//
template <class T, class R> NvArray<T,R>&
NvArray<T,R>::swap(int i1, int i2)
{
    NV_ARRAY_ASSERT(isValid(i1));
    NV_ARRAY_ASSERT(isValid(i2));

    if (i1 == i2) return *this;

    // tmp is non-const, so we can move out of it
    T tmp = std::move(mpArray[i1]);
    mpArray[i1] = std::move(mpArray[i2]);
    mpArray[i2] = std::move(tmp);
    return *this;
}

// Returns true if and only if `value' was removed from the array from
// position `start' onwards.  Only the first occurrence of `value'
// is removed.  Calling this function is equivalent to doing a "find(),
// then "removeAt()".
//
template <class T, class R> bool
NvArray<T,R>::remove(const T& value, int start)
{
    const int i = this->findFrom(value, start);
    if (i == -1)
        return false;
    this->removeAt(i);
    return true;
}

template <class T> void NvArrayValidateParams(bool bSameBuffer,
                                              T* pDest, int nBufLen,
                                              const T * pSource, int nCount)
{
    NOVA_UNREFED_PARAM(pDest);
    NOVA_UNREFED_PARAM(nBufLen);
    NOVA_UNREFED_PARAM(pSource);
    NOVA_UNREFED_PARAM(nCount);
    NV_ARRAY_ASSERT(nCount >= 0);
    NV_ARRAY_ASSERT(nCount <= nBufLen);
    NV_ARRAY_ASSERT(nCount <= NvArray<T>::maxLength());
    if (bSameBuffer) {
        // if moving within same buffer, we expect we're moving items "down", as
        // with a remove.
        NV_ARRAY_ASSERT(pSource > pDest);
    }
    else {
        // otherwise there should be no overlap ever.
        NV_ARRAY_ASSERT(pSource >= pDest + nBufLen || (pDest >= pSource + nCount));
    }
}

#include "nvarrayhelper.h"

#ifdef _Nv_String_h_
typedef
NvArray< NvString, NvArrayObjectCopyReallocator< NvString > > AcStringArray;
#endif


#pragma pop_macro("new")
#pragma pop_macro("delete")
#pragma pack (pop)
#endif
