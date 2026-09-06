// NvString.cpp - implementation of the NvString wide-character string class.
//
// Storage model (dictated by the frozen NvString.h):
//   NvString has the single data member `wchar_t* m_wsz`, which is never null:
//   it always points to a null-terminated wide-character buffer. Each buffer is
//   preceded by a NvStringBlock control header (use count, capacity, loan
//   state, lazily built utf-8 cache). Copies share the buffer and bump the use
//   count; every mutating operation detaches first (copy-on-write), so clients
//   can never observe sharing. This matches the header notes: buffers "may be
//   shared by multiple NvString instances" and the destructor "decrements use
//   count and frees memory if the count goes to zero".
//
// Documented behavior decisions:
//   - Out-of-range indices throw NvException("NvString::method(): index out
//     of range") where the header leaves the behavior open (setAt). Where an
//     ARX-style clamp is conventional (mid/substr/left/right/insert/
//     deleteAtIndex/tokenize/reserve), inputs are clamped instead.
//   - Null C-string arguments are treated as empty strings.
//   - NvDbHandle is only forward-declared in this build ("class NvDbHandle;"),
//     so the handle constructor and assign() cannot format a handle value;
//     they throw NvException. asNvDbHandle() cannot be compiled at all (an
//     incomplete type cannot be returned by value) and stays undefined.
//   - loadString() and the resource constructor cannot load platform resource
//     strings in this build; loadString() reports false, the constructor
//     produces an empty string.
//   - The class is not thread-safe: sharing is single-thread copy-on-write.

#include <NvString.h>
#include <Nova.h>
#include <NvAChar.h>

#include <NvException.h>

#include <cassert>
#include <cerrno>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <cwctype>
#include <limits>
#include <new>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{

//! Control header placed immediately in front of the wide-character buffer.
//! `data` is the first wide character; the real array is longer than one
//! element and always null-terminated. Standard layout so offsetof() applies.
struct NvStringBlock
{
  unsigned refCount;    //!< number of NvString instances sharing this buffer
  unsigned capacity;    //!< logical characters the buffer can hold, terminator excluded
  int openLength;       //!< >= 0 while a getBuffer() loan is outstanding, -1 otherwise
  char* utf8;           //!< lazily built utf-8 cache (owned), nullptr until first utf8Ptr()
  wchar_t data[1];      //!< wide characters; the actual array is longer
};

constexpr size_t kBlockDataOffset = offsetof(NvStringBlock, data);

//! Control block of the buffer that `theWsz` points into.
NvStringBlock* BlockOf(const wchar_t* theWsz)
{
  char* aRaw = reinterpret_cast<char*>(const_cast<wchar_t*>(theWsz));
  return reinterpret_cast<NvStringBlock*>(aRaw - kBlockDataOffset);
}

//! Drops a utf-8 cache that no longer matches the buffer content.
void DropUtf8Cache(NvStringBlock* theBlock)
{
  if (theBlock->utf8 != nullptr)
  {
    delete[] theBlock->utf8;
    theBlock->utf8 = nullptr;
  }
}

//! Allocates a fresh exclusively owned buffer with the given capacity.
//! The content is empty (only the terminator at position 0 is written).
wchar_t* AllocBlock(unsigned theCapacity)
{
  constexpr size_t kMaxChars =
    (static_cast<size_t>(-1) - kBlockDataOffset - sizeof(wchar_t)) / sizeof(wchar_t);
  if (static_cast<size_t>(theCapacity) > kMaxChars)
  {
    throw std::bad_alloc();
  }
  const size_t aBytes = kBlockDataOffset + (static_cast<size_t>(theCapacity) + 1) * sizeof(wchar_t);
  unsigned char* aRaw = new unsigned char[aBytes];
  NvStringBlock* aBlock = reinterpret_cast<NvStringBlock*>(aRaw);
  aBlock->refCount = 1;
  aBlock->capacity = theCapacity;
  aBlock->openLength = -1;
  aBlock->utf8 = nullptr;
  aBlock->data[0] = L'\0';
  return aBlock->data;
}

//! Allocates a fresh buffer of the given capacity holding a copy of
//! `theCharCount` characters from `theSrc` (which may be null when the count
//! is zero). A null terminator is always written after the copy.
wchar_t* NewBlockFrom(const wchar_t* theSrc, unsigned theCharCount, unsigned theCapacity)
{
  wchar_t* aWsz = AllocBlock(theCapacity);
  if (theCharCount > 0)
  {
    std::wmemcpy(aWsz, theSrc, theCharCount);
  }
  aWsz[theCharCount] = L'\0';
  return aWsz;
}

//! Same as above with an exact-fit capacity.
wchar_t* NewBlockFrom(const wchar_t* theSrc, unsigned theCharCount)
{
  return NewBlockFrom(theSrc, theCharCount, theCharCount);
}

//! Length of a null-terminated wide string as unsigned.
unsigned WszLength(const wchar_t* theWsz)
{
  const size_t aLen = std::wcslen(theWsz);
  assert(aLen <= static_cast<size_t>(UINT_MAX));
  return static_cast<unsigned>(aLen);
}

//! Releases ownership of the current buffer, freeing it when the use count
//! drops to zero. Leaves the pointer null.
void ReleaseStorage(wchar_t*& theWsz)
{
  NvStringBlock* aBlock = BlockOf(theWsz);
  if (--aBlock->refCount == 0)
  {
    DropUtf8Cache(aBlock);
    delete[] reinterpret_cast<unsigned char*>(aBlock);
  }
  theWsz = nullptr;
}

//! Replaces the current buffer with a shared reference to `theSrc`'s buffer.
void ShareStorage(wchar_t*& theWsz, const wchar_t* theSrc)
{
  ++BlockOf(theSrc)->refCount;
  ReleaseStorage(theWsz);
  theWsz = const_cast<wchar_t*>(theSrc);
}

//! Replaces the current buffer with a private copy of `theCharCount`
//! characters from `theSrc`. Copying happens before the release, so
//! assigning from this string's own buffer is safe.
void CopyStorage(wchar_t*& theWsz, const wchar_t* theSrc, unsigned theCharCount)
{
  wchar_t* aNew = NewBlockFrom(theSrc, theCharCount);
  ReleaseStorage(theWsz);
  theWsz = aNew;
}

//! Guarantees exclusive ownership of the buffer (copy-on-write detach) and
//! drops the utf-8 cache, since the caller is about to modify the content.
void MakeUnique(wchar_t*& theWsz)
{
  NvStringBlock* aBlock = BlockOf(theWsz);
  if (aBlock->refCount == 1)
  {
    DropUtf8Cache(aBlock);
    return;
  }
  const unsigned aLen = WszLength(theWsz);
  wchar_t* aCopy = NewBlockFrom(theWsz, aLen, aBlock->capacity);
  ReleaseStorage(theWsz);
  theWsz = aCopy;
}

//! Guarantees that the buffer can hold `theCharCount` logical characters and
//! is exclusively owned. Grows by doubling; capacity is preserved when a
//! shared buffer merely needs to be detached.
void EnsureCapacity(wchar_t*& theWsz, unsigned theCharCount)
{
  NvStringBlock* aBlock = BlockOf(theWsz);
  if (aBlock->refCount == 1 && aBlock->capacity >= theCharCount)
  {
    DropUtf8Cache(aBlock);
    return;
  }
  const unsigned aLen = WszLength(theWsz);
  unsigned aNewCapacity = aBlock->capacity;
  if (aNewCapacity < theCharCount)
  {
    constexpr unsigned kHalfMax = (std::numeric_limits<unsigned>::max)() / 2u;
    aNewCapacity = (aNewCapacity < kHalfMax)
      ? aNewCapacity + aNewCapacity / 2u + 16u
      : theCharCount;
    if (aNewCapacity < theCharCount)
    {
      aNewCapacity = theCharCount;
    }
  }
  wchar_t* aNew = AllocBlock(aNewCapacity);
  std::wmemcpy(aNew, theWsz, static_cast<size_t>(aLen) + 1);
  ReleaseStorage(theWsz);
  theWsz = aNew;
}

//! Appends `theSrcLen` characters from `theSrc`, tolerating a source that
//! lives inside this string's own buffer.
void AppendChars(wchar_t*& theWsz, const wchar_t* theSrc, unsigned theSrcLen)
{
  if (theSrcLen == 0)
  {
    return;
  }
  const wchar_t* aSrc = theSrc;
  std::vector<wchar_t> aTemp;
  if (BlockOf(theSrc) == BlockOf(theWsz))
  {
    aTemp.assign(theSrc, theSrc + theSrcLen);
    aSrc = aTemp.data();
  }
  const unsigned aLen = WszLength(theWsz);
  EnsureCapacity(theWsz, aLen + theSrcLen);
  std::wmemcpy(theWsz + aLen, aSrc, theSrcLen);
  theWsz[aLen + theSrcLen] = L'\0';
}

//! The whitespace set used wherever the header says "search for whitespace".
//! Fixed ASCII set so behavior does not depend on the C locale.
bool IsSpaceChar(wchar_t theChar)
{
  switch (theChar)
  {
    case L' ':
    case L'\t':
    case L'\n':
    case L'\v':
    case L'\f':
    case L'\r':
      return true;
    default:
      return false;
  }
}

//! True when the string holds only whitespace (or is empty).
bool IsBlank(const wchar_t* theStr)
{
  const wchar_t* aPtr = theStr;
  while (IsSpaceChar(*aPtr))
  {
    ++aPtr;
  }
  return *aPtr == L'\0';
}

//! Sign of a raw comparison result, normalized to -1 / 0 / 1 as the header
//! documents for the compare family.
int NormalizedSign(int theComparison)
{
  return (theComparison > 0) - (theComparison < 0);
}

//! Binary wide-string comparison normalized to -1 / 0 / 1.
int NormalizedCompare(const wchar_t* theLeft, const wchar_t* theRight)
{
  return NormalizedSign(std::wcscmp(theLeft, theRight));
}

//! Digit value of a wide character in the given base, -1 when invalid.
int DigitVal(wchar_t theChar, unsigned theBase)
{
  int aValue = -1;
  if (theChar >= L'0' && theChar <= L'9')
  {
    aValue = theChar - L'0';
  }
  else if (theChar >= L'a' && theChar <= L'f')
  {
    aValue = theChar - L'a' + 10;
  }
  else if (theChar >= L'A' && theChar <= L'F')
  {
    aValue = theChar - L'A' + 10;
  }
  if (aValue < 0 || static_cast<unsigned>(aValue) >= theBase)
  {
    return -1;
  }
  return aValue;
}

//! Result of scanning a string as an unsigned magnitude.
struct NvParseOutcome
{
  bool ok;                    //!< syntax is valid (digits found, no trailing garbage)
  bool negative;              //!< a leading '-' was consumed
  bool overflow;              //!< magnitude does not fit in 64 bits
  unsigned long long magnitude;
};

//! Scans the string as an unsigned magnitude in the given base. Surrounding
//! whitespace is skipped; any other trailing garbage invalidates the result.
NvParseOutcome ParseDigits(const wchar_t* theStr, unsigned theBase, bool theAllowSign)
{
  NvParseOutcome aResult;
  aResult.ok = false;
  aResult.negative = false;
  aResult.overflow = false;
  aResult.magnitude = 0;

  const wchar_t* aPtr = theStr;
  while (IsSpaceChar(*aPtr))
  {
    ++aPtr;
  }
  if (theAllowSign)
  {
    if (*aPtr == L'+')
    {
      ++aPtr;
    }
    else if (*aPtr == L'-')
    {
      aResult.negative = true;
      ++aPtr;
    }
  }
  if (theBase == 16 && aPtr[0] == L'0' && (aPtr[1] == L'x' || aPtr[1] == L'X')
      && DigitVal(aPtr[2], 16) >= 0)
  {
    aPtr += 2;
  }

  int aDigitCount = 0;
  for (; ; ++aPtr)
  {
    const int aDigit = DigitVal(*aPtr, theBase);
    if (aDigit < 0)
    {
      break;
    }
    ++aDigitCount;
    if (aResult.magnitude > (ULLONG_MAX - static_cast<unsigned long long>(aDigit)) / theBase)
    {
      aResult.overflow = true;
      aResult.magnitude = ULLONG_MAX; // saturated; the flag marks it invalid
    }
    else
    {
      aResult.magnitude = aResult.magnitude * theBase + static_cast<unsigned long long>(aDigit);
    }
  }
  while (IsSpaceChar(*aPtr))
  {
    ++aPtr;
  }
  aResult.ok = (aDigitCount > 0) && (*aPtr == L'\0');
  return aResult;
}

//! Applies the kParseAssert / kParseExcept error policy. Throws per the
//! kParseExcept contract (documented as an int exception).
void HandleParseError(int theFlags)
{
  if ((theFlags & NvString::kParseAssert) != 0)
  {
    assert(!"NvString: number parsing failed"); // debug-build pop, per kParseAssert
  }
  if ((theFlags & NvString::kParseExcept) != 0)
  {
    throw 1;
  }
}

//! Error return value selected by the kParseZero / kParseMinus1 flags.
template <typename T>
T ParseErrorValue(int theFlags)
{
  if ((theFlags & NvString::kParseMinus1) != 0)
  {
    if constexpr (std::is_signed<T>::value)
    {
      return static_cast<T>(-1);
    }
    else
    {
      return (std::numeric_limits<T>::max)();
    }
  }
  return static_cast<T>(0);
}

//! Shared implementation of the asDeci()/asHex() family: parse the string as
//! decimal or hex and apply the kParse* error and range policy of `theFlags`.
template <typename T>
T ParseAs(const wchar_t* theStr, bool theHex, bool theAllowSign, int theFlags)
{
  if (IsBlank(theStr))
  {
    // An empty string parses as zero unless kParseNoEmpty is set.
    if ((theFlags & NvString::kParseNoEmpty) == 0)
    {
      return static_cast<T>(0);
    }
    HandleParseError(theFlags);
    return ParseErrorValue<T>(theFlags);
  }

  const NvParseOutcome aOutcome = ParseDigits(theStr, theHex ? 16u : 10u, theAllowSign);
  if (!aOutcome.ok)
  {
    HandleParseError(theFlags);
    return ParseErrorValue<T>(theFlags);
  }

  if constexpr (std::is_signed<T>::value)
  {
    constexpr unsigned long long kNegLimit =
      (sizeof(T) >= 8) ? 0x8000000000000000ULL : 0x80000000ULL;
    constexpr unsigned long long kPosLimit =
      (sizeof(T) >= 8) ? 0x7FFFFFFFFFFFFFFFULL : 0x7FFFFFFFULL;
    if (aOutcome.overflow
        || aOutcome.magnitude > (aOutcome.negative ? kNegLimit : kPosLimit))
    {
      HandleParseError(theFlags);
      return ParseErrorValue<T>(theFlags);
    }
    // magnitude - 1 keeps the negation in range at exactly 2^63.
    const int64_t aValue = aOutcome.negative
      ? -static_cast<int64_t>(aOutcome.magnitude - 1) - 1
      : static_cast<int64_t>(aOutcome.magnitude);
    return static_cast<T>(aValue);
  }
  else
  {
    constexpr unsigned long long kLimit = (sizeof(T) >= 8) ? ULLONG_MAX : 0xFFFFFFFFULL;
    if (aOutcome.overflow || aOutcome.magnitude > kLimit)
    {
      HandleParseError(theFlags);
      return ParseErrorValue<T>(theFlags);
    }
    return static_cast<T>(aOutcome.magnitude);
  }
}

//! Reads the next code point from a wide string, combining utf-16 surrogate
//! pairs when wchar_t is 16 bits. Advances `theIndex`; returns 0 at the end.
unsigned int NextCodePoint(const wchar_t* theStr, unsigned& theIndex)
{
  const unsigned int aFirst = static_cast<unsigned int>(theStr[theIndex]);
  if (aFirst == 0)
  {
    return 0;
  }
  ++theIndex;
  if constexpr (sizeof(wchar_t) == 2)
  {
    if (aFirst >= 0xD800u && aFirst <= 0xDBFFu)
    {
      const unsigned int aSecond = static_cast<unsigned int>(theStr[theIndex]);
      if (aSecond >= 0xDC00u && aSecond <= 0xDFFFu)
      {
        ++theIndex;
        return 0x10000u + ((aFirst - 0xD800u) << 10) + (aSecond - 0xDC00u);
      }
    }
  }
  return aFirst;
}

//! Encodes a wide string as utf-8 into a freshly allocated, null-terminated
//! char array (caller owns it). Lone surrogates and values beyond Unicode are
//! preserved bit-exactly so that encode(decode(x)) is always the identity.
char* EncodeUtf8(const wchar_t* theWsz)
{
  std::vector<char> aBytes;
  aBytes.reserve(static_cast<size_t>(WszLength(theWsz)) + 1);
  unsigned anIndex = 0;
  for (;;)
  {
    const unsigned int aCodePoint = NextCodePoint(theWsz, anIndex);
    if (aCodePoint == 0)
    {
      break;
    }
    if (aCodePoint < 0x80u)
    {
      aBytes.push_back(static_cast<char>(aCodePoint));
    }
    else if (aCodePoint < 0x800u)
    {
      aBytes.push_back(static_cast<char>(0xC0u | (aCodePoint >> 6)));
      aBytes.push_back(static_cast<char>(0x80u | (aCodePoint & 0x3Fu)));
    }
    else if (aCodePoint < 0x10000u)
    {
      aBytes.push_back(static_cast<char>(0xE0u | (aCodePoint >> 12)));
      aBytes.push_back(static_cast<char>(0x80u | ((aCodePoint >> 6) & 0x3Fu)));
      aBytes.push_back(static_cast<char>(0x80u | (aCodePoint & 0x3Fu)));
    }
    else
    {
      aBytes.push_back(static_cast<char>(0xF0u | (aCodePoint >> 18)));
      aBytes.push_back(static_cast<char>(0x80u | ((aCodePoint >> 12) & 0x3Fu)));
      aBytes.push_back(static_cast<char>(0x80u | ((aCodePoint >> 6) & 0x3Fu)));
      aBytes.push_back(static_cast<char>(0x80u | (aCodePoint & 0x3Fu)));
    }
  }
  aBytes.push_back('\0');

  char* aUtf8 = new char[aBytes.size()];
  std::memcpy(aUtf8, aBytes.data(), aBytes.size());
  return aUtf8;
}

//! Decodes `theByteCount` bytes of utf-8 into a null-terminated wide buffer.
//! Invalid sequences are passed through byte-transparently (each invalid lead
//! byte becomes one wide character) so arbitrary binary data survives; a
//! sequence truncated by `theByteCount` is handled the same way.
std::vector<wchar_t> DecodeUtf8(const char* theUtf8, size_t theByteCount)
{
  std::vector<wchar_t> aWide;
  if (theUtf8 == nullptr)
  {
    aWide.push_back(L'\0');
    return aWide;
  }
  size_t anIndex = 0;
  while (anIndex < theByteCount)
  {
    const unsigned char aLead = static_cast<unsigned char>(theUtf8[anIndex]);
    if (aLead < 0x80u)
    {
      aWide.push_back(static_cast<wchar_t>(aLead));
      ++anIndex;
      continue;
    }

    int aSequenceLen = 0;
    unsigned int aCodePoint = 0;
    if ((aLead & 0xE0u) == 0xC0u)
    {
      aSequenceLen = 2;
      aCodePoint = aLead & 0x1Fu;
    }
    else if ((aLead & 0xF0u) == 0xE0u)
    {
      aSequenceLen = 3;
      aCodePoint = aLead & 0x0Fu;
    }
    else if ((aLead & 0xF8u) == 0xF0u)
    {
      aSequenceLen = 4;
      aCodePoint = aLead & 0x07u;
    }

    bool aValid = (aSequenceLen > 0)
      && (anIndex + static_cast<size_t>(aSequenceLen) <= theByteCount);
    if (aValid)
    {
      for (int aContinuation = 1; aContinuation < aSequenceLen; ++aContinuation)
      {
        const unsigned char aByte =
          static_cast<unsigned char>(theUtf8[anIndex + static_cast<size_t>(aContinuation)]);
        if ((aByte & 0xC0u) != 0x80u)
        {
          aValid = false;
          break;
        }
        aCodePoint = (aCodePoint << 6) | (aByte & 0x3Fu);
      }
    }
    if (aValid)
    {
      // Reject overlong encodings (C0/C1 leads, 2-byte ascii, etc.).
      const unsigned int aMinimum = (aSequenceLen == 2) ? 0x80u
                                  : (aSequenceLen == 3) ? 0x800u
                                  : 0x10000u;
      if (aCodePoint < aMinimum)
      {
        aValid = false;
      }
    }
    if (!aValid)
    {
      aWide.push_back(static_cast<wchar_t>(aLead));
      ++anIndex;
      continue;
    }

    if constexpr (sizeof(wchar_t) == 2)
    {
      if (aCodePoint > 0xFFFFu)
      {
        // Valid Unicode fits a surrogate pair; anything above is clamped.
        if (aCodePoint > 0x10FFFFu)
        {
          aCodePoint = 0x10FFFFu;
        }
        aCodePoint -= 0x10000u;
        aWide.push_back(static_cast<wchar_t>(0xD800u + (aCodePoint >> 10)));
        aWide.push_back(static_cast<wchar_t>(0xDC00u + (aCodePoint & 0x3FFu)));
      }
      else
      {
        aWide.push_back(static_cast<wchar_t>(aCodePoint));
      }
    }
    else
    {
      aWide.push_back(static_cast<wchar_t>(aCodePoint));
    }
    anIndex += static_cast<size_t>(aSequenceLen);
  }
  aWide.push_back(L'\0');
  return aWide;
}

//! Lowercased copy of a wide string, null-terminated.
std::vector<wchar_t> FoldLower(const wchar_t* theStr)
{
  std::vector<wchar_t> aFolded;
  for (const wchar_t* aPtr = theStr; *aPtr != L'\0'; ++aPtr)
  {
    aFolded.push_back(static_cast<wchar_t>(std::towlower(static_cast<wint_t>(*aPtr))));
  }
  aFolded.push_back(L'\0');
  return aFolded;
}

//! Throws unless the encoding is the only one this build supports.
void CheckUtf8Encoding(NvString::Encoding theEncoding, const char* theMethodName)
{
  if (theEncoding != NvString::Utf8)
  {
    const std::string aMessage = std::string(theMethodName) + ": only Utf8 encoding is supported";
    throw NvException(aMessage.c_str());
  }
}

} // namespace

//=================================================================================================

NvString::NvString()
  : m_wsz(AllocBlock(0))
{
}

//=================================================================================================

NvString::NvString(wchar_t theChar)
  : m_wsz(AllocBlock(1))
{
  this->m_wsz[0] = theChar;
  this->m_wsz[1] = L'\0';
}

//=================================================================================================

NvString::NvString(const char* thePsz, Encoding theEncoding)
  : m_wsz(nullptr)
{
  CheckUtf8Encoding(theEncoding, "NvString::NvString()");
  if (thePsz == nullptr)
  {
    this->m_wsz = AllocBlock(0);
    return;
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  this->m_wsz = NewBlockFrom(aWide.data(), static_cast<unsigned>(aWide.size()) - 1);
}

//=================================================================================================

NvString::NvString(const char* thePsz, Encoding theEncoding, unsigned int theByteCount)
  : m_wsz(nullptr)
{
  CheckUtf8Encoding(theEncoding, "NvString::NvString()");
  if (thePsz == nullptr || theByteCount == 0)
  {
    this->m_wsz = AllocBlock(0);
    return;
  }
  // Per the header, a byte count that splits the last utf-8 sequence is
  // undefined; the decoder falls back to per-byte transparency instead.
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, theByteCount);
  this->m_wsz = NewBlockFrom(aWide.data(), static_cast<unsigned>(aWide.size()) - 1);
}

//=================================================================================================

NvString::NvString(const wchar_t* thePwsz)
  : m_wsz(nullptr)
{
  const unsigned aLen = (thePwsz == nullptr) ? 0u : WszLength(thePwsz);
  this->m_wsz = NewBlockFrom(thePwsz, aLen);
}

//=================================================================================================

NvString::NvString(const wchar_t* thePwsz, unsigned int theCount)
  : m_wsz(nullptr)
{
  // The count is honored verbatim (CString-style): the source must provide at
  // least `theCount` characters, which may include embedded nulls.
  const unsigned aCount = (thePwsz == nullptr) ? 0u : theCount;
  this->m_wsz = NewBlockFrom(thePwsz, aCount);
}

//=================================================================================================

NvString::NvString(const NvString& theOther)
  : m_wsz(theOther.m_wsz)
{
  ++BlockOf(this->m_wsz)->refCount;
}

//=================================================================================================

NvString::NvString(NvString&& theOther)
  : m_wsz(theOther.m_wsz)
{
  // A moved-from string stays a valid empty string (m_wsz is never null).
  theOther.m_wsz = AllocBlock(0);
}

//=================================================================================================

NvString::NvString(eFormat theCtorFlags, unsigned theArg)
  : m_wsz(nullptr)
{
  wchar_t aBuffer[32];
  constexpr size_t kBufferChars = sizeof(aBuffer) / sizeof(aBuffer[0]);
  int aWritten = 0;
  switch (theCtorFlags)
  {
    case kSigned:
      aWritten = std::swprintf(aBuffer, kBufferChars, L"%d", static_cast<int>(theArg));
      break;
    case kHex:
      aWritten = std::swprintf(aBuffer, kBufferChars, L"%x", theArg);
      break;
    case kUnSigned:
    default:
      aWritten = std::swprintf(aBuffer, kBufferChars, L"%u", theArg);
      break;
  }
  this->m_wsz = (aWritten > 0)
    ? NewBlockFrom(aBuffer, static_cast<unsigned>(aWritten))
    : AllocBlock(0);
}

//=================================================================================================

NvString::NvString(NCHAR theChar, unsigned theRepeatTimes)
  : m_wsz(AllocBlock(theRepeatTimes))
{
  for (unsigned anIndex = 0; anIndex < theRepeatTimes; ++anIndex)
  {
    this->m_wsz[anIndex] = theChar;
  }
  this->m_wsz[theRepeatTimes] = L'\0';
}

//=================================================================================================

NvString::NvString(const NvDbHandle& theHandle)
  : m_wsz(nullptr)
{
  // NvDbHandle has no definition in this build, so its value cannot be
  // formatted; refuse loudly instead of producing a wrong string.
  (void)theHandle;
  throw NvException("NvString::NvString(): NvDbHandle is not defined in this build");
}

//=================================================================================================

NvString::NvString(const NvRxResourceInstance& theResourceInstance, unsigned int theResourceId)
  : m_wsz(AllocBlock(0))
{
  // Platform resource strings are not available in this build; produce an
  // empty string (loadString() reports availability through its return).
  (void)theResourceInstance;
  (void)theResourceId;
}

//=================================================================================================

NvString::~NvString()
{
  ReleaseStorage(this->m_wsz);
}

//=================================================================================================

const char* NvString::utf8Ptr() const
{
  // The only const method allowed to modify the object: it fills the lazily
  // built utf-8 cache in the control block. Wide pointers stay valid.
  NvStringBlock* aBlock = BlockOf(this->m_wsz);
  if (aBlock->utf8 == nullptr)
  {
    aBlock->utf8 = EncodeUtf8(this->m_wsz);
  }
  return aBlock->utf8;
}

//=================================================================================================

unsigned NvString::length() const
{
  return WszLength(this->m_wsz);
}

//=================================================================================================

unsigned NvString::capacity() const
{
  return BlockOf(this->m_wsz)->capacity;
}

//=================================================================================================

bool NvString::reserve(unsigned theCapacity)
{
  const unsigned aNeeded = (theCapacity > 0) ? theCapacity - 1 : 0;
  const unsigned aLen = WszLength(this->m_wsz);
  // Live content must always fit, so a reservation below the current length
  // cannot truncate; it is honored down to the current length only.
  const unsigned aFloor = (aNeeded > aLen) ? aNeeded : aLen;

  NvStringBlock* aBlock = BlockOf(this->m_wsz);
  if (aBlock->capacity == aFloor)
  {
    return false;
  }
  if (aBlock->refCount == 1)
  {
    wchar_t* aNew = AllocBlock(aFloor);
    std::wmemcpy(aNew, this->m_wsz, static_cast<size_t>(aLen) + 1);
    ReleaseStorage(this->m_wsz);
    this->m_wsz = aNew;
  }
  else
  {
    // Never shrink a buffer other instances are still sharing.
    wchar_t* aNew = NewBlockFrom(this->m_wsz, aLen, aFloor);
    ReleaseStorage(this->m_wsz);
    this->m_wsz = aNew;
  }
  return true;
}

//=================================================================================================

bool NvString::isAscii() const
{
  // Per the header, codes 0x00..0x1f are control characters and fail the test.
  for (const wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    const unsigned int aCode = static_cast<unsigned int>(*aPtr);
    if (aCode < 0x20u || aCode > 0x7Fu)
    {
      return false;
    }
  }
  return true;
}

//=================================================================================================

bool NvString::is7Bit() const
{
  // Characters reached before the terminator are >= 0x01 by construction.
  for (const wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    if (static_cast<unsigned int>(*aPtr) > 0x7Fu)
    {
      return false;
    }
  }
  return true;
}

//=================================================================================================

int NvString::asDeci(int theFlags) const
{
  return ParseAs<int>(this->m_wsz, false, true, theFlags);
}

//=================================================================================================

int NvString::asHex(int theFlags) const
{
  return ParseAs<int>(this->m_wsz, true, true, theFlags);
}

//=================================================================================================

unsigned int NvString::asUDeci(int theFlags) const
{
  return ParseAs<unsigned int>(this->m_wsz, false, false, theFlags);
}

//=================================================================================================

unsigned int NvString::asUHex(int theFlags) const
{
  return ParseAs<unsigned int>(this->m_wsz, true, false, theFlags);
}

//=================================================================================================

int64_t NvString::asDeci64(int theFlags) const
{
  return ParseAs<int64_t>(this->m_wsz, false, true, theFlags);
}

//=================================================================================================

int64_t NvString::asHex64(int theFlags) const
{
  return ParseAs<int64_t>(this->m_wsz, true, true, theFlags);
}

//=================================================================================================

Nova::UInt64 NvString::asUDeci64(int theFlags) const
{
  return ParseAs<Nova::UInt64>(this->m_wsz, false, false, theFlags);
}

//=================================================================================================

Nova::UInt64 NvString::asUHex64(int theFlags) const
{
  return ParseAs<Nova::UInt64>(this->m_wsz, true, false, theFlags);
}

// asNvDbHandle is intentionally not defined: the NvDbHandle return type is
// only forward-declared in NvString.h and has no definition in this build, so
// a function returning it by value cannot be compiled.

//=================================================================================================

int NvString::find(const NCHAR* thePsz, int theStartPos) const
{
  if (thePsz == nullptr)
  {
    return -1;
  }
  const int aStart = (theStartPos < 0) ? 0 : theStartPos;
  const unsigned aLen = WszLength(this->m_wsz);
  if (aStart > static_cast<int>(aLen))
  {
    return -1;
  }
  const wchar_t* aFound = std::wcsstr(this->m_wsz + aStart, thePsz);
  if (aFound == nullptr)
  {
    return -1;
  }
  return static_cast<int>(aFound - this->m_wsz);
}

//=================================================================================================

int NvString::findOneOf(const NCHAR* thePsz, int theStartPos) const
{
  const int aStart = (theStartPos < 0) ? 0 : theStartPos;
  const unsigned aLen = WszLength(this->m_wsz);
  if (aStart >= static_cast<int>(aLen))
  {
    return -1;
  }
  // A null group searches for whitespace, per the header remarks.
  for (unsigned anIndex = static_cast<unsigned>(aStart); anIndex < aLen; ++anIndex)
  {
    const wchar_t aChar = this->m_wsz[anIndex];
    const bool aMatch = (thePsz == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePsz, aChar) != nullptr);
    if (aMatch)
    {
      return static_cast<int>(anIndex);
    }
  }
  return -1;
}

//=================================================================================================

int NvString::findNoneOf(const NCHAR* thePsz, int theStartPos) const
{
  const int aStart = (theStartPos < 0) ? 0 : theStartPos;
  const unsigned aLen = WszLength(this->m_wsz);
  if (aStart >= static_cast<int>(aLen))
  {
    return -1;
  }
  // A null group searches for the first non-whitespace character.
  for (unsigned anIndex = static_cast<unsigned>(aStart); anIndex < aLen; ++anIndex)
  {
    const wchar_t aChar = this->m_wsz[anIndex];
    const bool aMatch = (thePsz == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePsz, aChar) != nullptr);
    if (!aMatch)
    {
      return static_cast<int>(anIndex);
    }
  }
  return -1;
}

//=================================================================================================

int NvString::findLast(const NCHAR* thePsz, int theEndPos) const
{
  if (thePsz == nullptr)
  {
    return -1;
  }
  const unsigned aLen = WszLength(this->m_wsz);
  const unsigned aPatternLen = WszLength(thePsz);
  if (aPatternLen == 0)
  {
    // Empty needle: reported at the effective end position.
    if (theEndPos < 0 || theEndPos > static_cast<int>(aLen))
    {
      return static_cast<int>(aLen);
    }
    return theEndPos;
  }
  // The match start may not exceed theEndPos (or the end of the string when
  // theEndPos keeps its default -1).
  int aLimit = (theEndPos < 0) ? static_cast<int>(aLen) : theEndPos;
  const int aMaxStart = static_cast<int>(aLen - aPatternLen);
  if (aLimit > aMaxStart)
  {
    aLimit = aMaxStart;
  }
  for (int aStart = aLimit; aStart >= 0; --aStart)
  {
    if (std::wmemcmp(this->m_wsz + aStart, thePsz, aPatternLen) == 0)
    {
      return aStart;
    }
  }
  return -1;
}

//=================================================================================================

int NvString::findLastOneOf(const NCHAR* thePsz, int theEndPos) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  int aStart = static_cast<int>(aLen) - 1;
  if (theEndPos >= 0 && theEndPos < aStart)
  {
    aStart = theEndPos;
  }
  // A null group searches for whitespace, per the header remarks.
  for (; aStart >= 0; --aStart)
  {
    const wchar_t aChar = this->m_wsz[aStart];
    const bool aMatch = (thePsz == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePsz, aChar) != nullptr);
    if (aMatch)
    {
      return aStart;
    }
  }
  return -1;
}

//=================================================================================================

int NvString::findLastNoneOf(const NCHAR* thePsz, int theEndPos) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  int aStart = static_cast<int>(aLen) - 1;
  if (theEndPos >= 0 && theEndPos < aStart)
  {
    aStart = theEndPos;
  }
  // A null group searches for the last non-whitespace character.
  for (; aStart >= 0; --aStart)
  {
    const wchar_t aChar = this->m_wsz[aStart];
    const bool aMatch = (thePsz == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePsz, aChar) != nullptr);
    if (!aMatch)
    {
      return aStart;
    }
  }
  return -1;
}

//=================================================================================================

NvString NvString::substr(int theStart, int theNumChars) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  const int aStart = (theStart < 0) ? 0 : theStart; // clamped, ARX-style
  if (aStart >= static_cast<int>(aLen))
  {
    return NvString();
  }
  const unsigned aStartU = static_cast<unsigned>(aStart);
  // Any negative count means "the rest of the string", per the header.
  unsigned aCount = (theNumChars < 0) ? (aLen - aStartU) : static_cast<unsigned>(theNumChars);
  if (aCount > aLen - aStartU)
  {
    aCount = aLen - aStartU;
  }
  NvString aResult;
  CopyStorage(aResult.m_wsz, this->m_wsz + aStartU, aCount);
  return aResult;
}

//=================================================================================================

NvString NvString::substrRev(int theNumChars) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  if (theNumChars <= 0)
  {
    return NvString();
  }
  unsigned aCount = static_cast<unsigned>(theNumChars);
  if (aCount > aLen)
  {
    aCount = aLen;
  }
  NvString aResult;
  CopyStorage(aResult.m_wsz, this->m_wsz + (aLen - aCount), aCount);
  return aResult;
}

//=================================================================================================

NvString& NvString::assign(const char* thePsz, Encoding theEncoding)
{
  CheckUtf8Encoding(theEncoding, "NvString::assign()");
  if (thePsz == nullptr)
  {
    return this->setEmpty();
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  CopyStorage(this->m_wsz, aWide.data(), static_cast<unsigned>(aWide.size()) - 1);
  return *this;
}

//=================================================================================================

NvString& NvString::assign(const wchar_t* thePwsz)
{
  if (thePwsz == nullptr)
  {
    return this->setEmpty();
  }
  CopyStorage(this->m_wsz, thePwsz, WszLength(thePwsz));
  return *this;
}

//=================================================================================================

NvString& NvString::assign(const NvString& theOther)
{
  if (this != &theOther)
  {
    ShareStorage(this->m_wsz, theOther.m_wsz);
  }
  return *this;
}

//=================================================================================================

NvString& NvString::assign(const NvDbHandle& theHandle)
{
  // See the constructor comment: no NvDbHandle definition in this build.
  (void)theHandle;
  throw NvException("NvString::assign(): NvDbHandle is not defined in this build");
}

//=================================================================================================

NvString& NvString::operator = (NvString&& theOther)
{
  if (this != &theOther)
  {
    ReleaseStorage(this->m_wsz);
    this->m_wsz = theOther.m_wsz;
    theOther.m_wsz = AllocBlock(0);
  }
  return *this;
}

//=================================================================================================

NvString& NvString::setEmpty()
{
  NvStringBlock* aBlock = BlockOf(this->m_wsz);
  if (aBlock->refCount == 1)
  {
    DropUtf8Cache(aBlock);
    aBlock->openLength = -1;
    this->m_wsz[0] = L'\0';
  }
  else
  {
    // Keep other instances untouched; take a fresh empty buffer instead.
    ReleaseStorage(this->m_wsz);
    this->m_wsz = AllocBlock(0);
  }
  return *this;
}

//=================================================================================================

bool NvString::loadString(const NvRxResourceInstance& theResourceInstance, unsigned theResourceId)
{
  // Platform resource loading is not available in this build.
  (void)theResourceInstance;
  (void)theResourceId;
  return false;
}

//=================================================================================================

NvString& NvString::format(const NCHAR* theFormat, ...)
{
  va_list aArgs;
  va_start(aArgs, theFormat);
  this->formatV(theFormat, aArgs);
  va_end(aArgs);
  return *this;
}

//=================================================================================================

NvString& NvString::formatV(const NCHAR* theFormat, va_list theArgs)
{
  if (theFormat == nullptr)
  {
    return this->setEmpty();
  }

  wchar_t aStackBuffer[1024];
  constexpr unsigned kStackChars = sizeof(aStackBuffer) / sizeof(aStackBuffer[0]);

  int aWritten = -1;
  {
    va_list aArgsCopy;
    va_copy(aArgsCopy, theArgs);
    aWritten = std::vswprintf(aStackBuffer, kStackChars, theFormat, aArgsCopy);
    va_end(aArgsCopy);
  }

  if (aWritten >= 0 && static_cast<unsigned>(aWritten) < kStackChars)
  {
    CopyStorage(this->m_wsz, aStackBuffer, static_cast<unsigned>(aWritten));
    return *this;
  }

  // Conformant runtimes report the required length, others a negative value
  // on truncation; retry with bigger buffers in both cases.
  if (aWritten >= 0)
  {
    std::vector<wchar_t> aBuffer(static_cast<unsigned>(aWritten) + 1);
    va_list aArgsCopy;
    va_copy(aArgsCopy, theArgs);
    aWritten = std::vswprintf(aBuffer.data(), aBuffer.size(), theFormat, aArgsCopy);
    va_end(aArgsCopy);
    if (aWritten >= 0)
    {
      CopyStorage(this->m_wsz, aBuffer.data(), static_cast<unsigned>(aWritten));
      return *this;
    }
  }
  else
  {
    for (unsigned aSize = 4096u; aSize <= 1048576u; aSize *= 4u)
    {
      std::vector<wchar_t> aBuffer(aSize);
      va_list aArgsCopy;
      va_copy(aArgsCopy, theArgs);
      const int aRetry = std::vswprintf(aBuffer.data(), aSize, theFormat, aArgsCopy);
      va_end(aArgsCopy);
      if (aRetry >= 0 && static_cast<unsigned>(aRetry) < aSize)
      {
        CopyStorage(this->m_wsz, aBuffer.data(), static_cast<unsigned>(aRetry));
        return *this;
      }
    }
  }

  // Formatting failed on every attempt; leave the string empty.
  return this->setEmpty();
}

//=================================================================================================

NvString& NvString::appendFormat(const NCHAR* theFormat, ...)
{
  va_list aArgs;
  va_start(aArgs, theFormat);
  NvString aTail;
  aTail.formatV(theFormat, aArgs);
  va_end(aArgs);
  return this->append(aTail);
}

//=================================================================================================

NvString& NvString::append(const char* thePsz, Encoding theEncoding)
{
  CheckUtf8Encoding(theEncoding, "NvString::append()");
  if (thePsz == nullptr)
  {
    return *this;
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  AppendChars(this->m_wsz, aWide.data(), static_cast<unsigned>(aWide.size()) - 1);
  return *this;
}

//=================================================================================================

NvString& NvString::append(const wchar_t* thePwsz)
{
  if (thePwsz == nullptr)
  {
    return *this;
  }
  AppendChars(this->m_wsz, thePwsz, WszLength(thePwsz));
  return *this;
}

//=================================================================================================

NvString& NvString::append(const NvString& theOther)
{
  AppendChars(this->m_wsz, theOther.m_wsz, WszLength(theOther.m_wsz));
  return *this;
}

//=================================================================================================

NvString NvString::concat(const char* thePsz, Encoding theEncoding) const
{
  NvString aResult(*this);
  aResult.append(thePsz, theEncoding);
  return aResult;
}

//=================================================================================================

NvString NvString::concat(const wchar_t* thePwsz) const
{
  NvString aResult(*this);
  aResult.append(thePwsz);
  return aResult;
}

//=================================================================================================

NvString NvString::concat(const NvString& theOther) const
{
  NvString aResult(*this);
  aResult.append(theOther);
  return aResult;
}

//=================================================================================================

NvString NvString::precat(const char* thePsz, Encoding theEncoding) const
{
  CheckUtf8Encoding(theEncoding, "NvString::precat()");
  if (thePsz == nullptr)
  {
    return NvString(*this);
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  return this->precat(aWide.data());
}

//=================================================================================================

NvString NvString::precat(const wchar_t* thePwsz) const
{
  const unsigned aPrefixLen = (thePwsz == nullptr) ? 0u : WszLength(thePwsz);
  const unsigned aLen = WszLength(this->m_wsz);
  NvString aResult;
  aResult.m_wsz = AllocBlock(aPrefixLen + aLen);
  if (aPrefixLen > 0)
  {
    std::wmemcpy(aResult.m_wsz, thePwsz, aPrefixLen);
  }
  std::wmemcpy(aResult.m_wsz + aPrefixLen, this->m_wsz, static_cast<size_t>(aLen) + 1);
  return aResult;
}

//=================================================================================================

int NvString::compare(const char* thePsz, Encoding theEncoding) const
{
  CheckUtf8Encoding(theEncoding, "NvString::compare()");
  if (thePsz == nullptr)
  {
    return NormalizedCompare(this->m_wsz, L"");
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  return NormalizedCompare(this->m_wsz, aWide.data());
}

//=================================================================================================

int NvString::compare(const wchar_t* thePwsz) const
{
  if (thePwsz == nullptr)
  {
    return NormalizedCompare(this->m_wsz, L"");
  }
  return NormalizedCompare(this->m_wsz, thePwsz);
}

//=================================================================================================

int NvString::collate(const wchar_t* thePwsz) const
{
  if (thePwsz == nullptr)
  {
    thePwsz = L"";
  }
  errno = 0;
  const int aComparison = std::wcscoll(this->m_wsz, thePwsz);
  if (errno != 0)
  {
    // Collation failed for this locale; fall back to the binary order.
    return NormalizedCompare(this->m_wsz, thePwsz);
  }
  return NormalizedSign(aComparison);
}

//=================================================================================================

int NvString::compareNoCase(const char* thePsz, Encoding theEncoding) const
{
  CheckUtf8Encoding(theEncoding, "NvString::compareNoCase()");
  if (thePsz == nullptr)
  {
    return this->compareNoCase(L"");
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  return this->compareNoCase(aWide.data());
}

//=================================================================================================

int NvString::compareNoCase(const wchar_t* thePwsz) const
{
  if (thePwsz == nullptr)
  {
    thePwsz = L"";
  }
  const wchar_t* aLeft = this->m_wsz;
  const wchar_t* aRight = thePwsz;
  while (*aLeft != L'\0' && *aRight != L'\0')
  {
    const wint_t aLeftChar = std::towlower(static_cast<wint_t>(*aLeft));
    const wint_t aRightChar = std::towlower(static_cast<wint_t>(*aRight));
    if (aLeftChar != aRightChar)
    {
      return (aLeftChar < aRightChar) ? -1 : 1;
    }
    ++aLeft;
    ++aRight;
  }
  if (*aLeft != L'\0')
  {
    return 1;
  }
  if (*aRight != L'\0')
  {
    return -1;
  }
  return 0;
}

//=================================================================================================

int NvString::collateNoCase(const wchar_t* thePsz) const
{
  const std::vector<wchar_t> aLeft = FoldLower(this->m_wsz);
  const std::vector<wchar_t> aRight = FoldLower((thePsz == nullptr) ? L"" : thePsz);
  errno = 0;
  int aComparison = std::wcscoll(aLeft.data(), aRight.data());
  if (errno != 0)
  {
    aComparison = std::wcscmp(aLeft.data(), aRight.data());
  }
  return NormalizedSign(aComparison);
}

//=================================================================================================

int NvString::match(const char* thePsz, Encoding theEncoding) const
{
  CheckUtf8Encoding(theEncoding, "NvString::match()");
  if (thePsz == nullptr)
  {
    return 0;
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  return this->match(aWide.data());
}

//=================================================================================================

int NvString::match(const wchar_t* thePwsz) const
{
  if (thePwsz == nullptr)
  {
    return 0;
  }
  int aCount = 0;
  const wchar_t* aLeft = this->m_wsz;
  const wchar_t* aRight = thePwsz;
  while (*aLeft != L'\0' && *aRight != L'\0' && *aLeft == *aRight)
  {
    ++aLeft;
    ++aRight;
    ++aCount;
  }
  return aCount;
}

//=================================================================================================

int NvString::match(const NvString& theOther) const
{
  return this->match(theOther.m_wsz);
}

//=================================================================================================

int NvString::matchNoCase(const char* thePsz, Encoding theEncoding) const
{
  CheckUtf8Encoding(theEncoding, "NvString::matchNoCase()");
  if (thePsz == nullptr)
  {
    return 0;
  }
  const std::vector<wchar_t> aWide = DecodeUtf8(thePsz, std::strlen(thePsz));
  return this->matchNoCase(aWide.data());
}

//=================================================================================================

int NvString::matchNoCase(const wchar_t* thePwsz) const
{
  if (thePwsz == nullptr)
  {
    return 0;
  }
  int aCount = 0;
  const wchar_t* aLeft = this->m_wsz;
  const wchar_t* aRight = thePwsz;
  while (*aLeft != L'\0' && *aRight != L'\0')
  {
    const wint_t aLeftChar = std::towlower(static_cast<wint_t>(*aLeft));
    const wint_t aRightChar = std::towlower(static_cast<wint_t>(*aRight));
    if (aLeftChar != aRightChar)
    {
      break;
    }
    ++aLeft;
    ++aRight;
    ++aCount;
  }
  return aCount;
}

//=================================================================================================

int NvString::matchNoCase(const NvString& theOther) const
{
  return this->matchNoCase(theOther.m_wsz);
}

//=================================================================================================

NvString& NvString::makeUpper()
{
  MakeUnique(this->m_wsz);
  for (wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    *aPtr = static_cast<wchar_t>(std::towupper(static_cast<wint_t>(*aPtr)));
  }
  return *this;
}

//=================================================================================================

NvString& NvString::makeLower()
{
  MakeUnique(this->m_wsz);
  for (wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    *aPtr = static_cast<wchar_t>(std::towlower(static_cast<wint_t>(*aPtr)));
  }
  return *this;
}

//=================================================================================================

NvString& NvString::makeReverse()
{
  MakeUnique(this->m_wsz);
  const unsigned aLen = WszLength(this->m_wsz);
  for (unsigned anIndex = 0; anIndex < aLen / 2; ++anIndex)
  {
    std::swap(this->m_wsz[anIndex], this->m_wsz[aLen - 1 - anIndex]);
  }
  return *this;
}

//=================================================================================================

NvString& NvString::trimLeft(const wchar_t* thePwszChars)
{
  const unsigned aLen = WszLength(this->m_wsz);
  unsigned aStart = 0;
  while (aStart < aLen)
  {
    const wchar_t aChar = this->m_wsz[aStart];
    const bool aMatch = (thePwszChars == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePwszChars, aChar) != nullptr);
    if (!aMatch)
    {
      break;
    }
    ++aStart;
  }
  if (aStart > 0)
  {
    MakeUnique(this->m_wsz);
    std::wmemmove(this->m_wsz, this->m_wsz + aStart, static_cast<size_t>(aLen - aStart) + 1);
  }
  return *this;
}

//=================================================================================================

NvString& NvString::trimRight(const wchar_t* thePwszChars)
{
  const unsigned aLen = WszLength(this->m_wsz);
  unsigned aEnd = aLen;
  while (aEnd > 0)
  {
    const wchar_t aChar = this->m_wsz[aEnd - 1];
    const bool aMatch = (thePwszChars == nullptr)
      ? IsSpaceChar(aChar)
      : (std::wcschr(thePwszChars, aChar) != nullptr);
    if (!aMatch)
    {
      break;
    }
    --aEnd;
  }
  if (aEnd < aLen)
  {
    MakeUnique(this->m_wsz);
    this->m_wsz[aEnd] = L'\0';
  }
  return *this;
}

//=================================================================================================

int NvString::remove(wchar_t theChar)
{
  // A zero character designates whitespace, per the header remarks.
  const bool aWhitespace = (theChar == L'\0');
  const unsigned aLen = WszLength(this->m_wsz);
  int aRemoved = 0;
  for (const wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    const bool aMatch = aWhitespace ? IsSpaceChar(*aPtr) : (*aPtr == theChar);
    if (aMatch)
    {
      ++aRemoved;
    }
  }
  if (aRemoved == 0)
  {
    return 0;
  }
  MakeUnique(this->m_wsz);
  unsigned aWrite = 0;
  for (unsigned anIndex = 0; anIndex < aLen; ++anIndex)
  {
    const wchar_t aChar = this->m_wsz[anIndex];
    const bool aMatch = aWhitespace ? IsSpaceChar(aChar) : (aChar == theChar);
    if (!aMatch)
    {
      this->m_wsz[aWrite] = aChar;
      ++aWrite;
    }
  }
  this->m_wsz[aWrite] = L'\0';
  return aRemoved;
}

//=================================================================================================

NvString NvString::spanExcluding(const wchar_t* thePwszChars) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  unsigned aCount = 0;
  while (aCount < aLen)
  {
    const bool aMatch = (thePwszChars == nullptr)
      ? false
      : (std::wcschr(thePwszChars, this->m_wsz[aCount]) != nullptr);
    if (aMatch)
    {
      break;
    }
    ++aCount;
  }
  NvString aResult;
  CopyStorage(aResult.m_wsz, this->m_wsz, aCount);
  return aResult;
}

//=================================================================================================

int NvString::replace(const wchar_t* thePwszOld, const wchar_t* thePwszNew)
{
  if (thePwszOld == nullptr || thePwszOld[0] == L'\0')
  {
    return 0;
  }
  const unsigned aOldLen = WszLength(thePwszOld);
  const unsigned aNewLen = (thePwszNew == nullptr) ? 0u : WszLength(thePwszNew);
  const unsigned aLen = WszLength(this->m_wsz);

  unsigned long long aCount = 0;
  for (const wchar_t* aScan = this->m_wsz;;)
  {
    const wchar_t* aFound = std::wcsstr(aScan, thePwszOld);
    if (aFound == nullptr)
    {
      break;
    }
    ++aCount;
    aScan = aFound + aOldLen;
  }
  if (aCount == 0)
  {
    return 0;
  }

  const unsigned long long aResultLen = static_cast<unsigned long long>(aLen)
    - aCount * aOldLen
    + aCount * aNewLen;
  if (aResultLen > 0xFFFFFFFFULL)
  {
    throw NvException("NvString::replace(): result is too large");
  }

  // Build into a scratch buffer: the result never overlaps the source, and
  // non-overlapping (replace-then-continue-after) semantics are used.
  std::vector<wchar_t> aScratch(static_cast<size_t>(aResultLen) + 1);
  wchar_t* aWrite = aScratch.data();
  const wchar_t* aScan = this->m_wsz;
  for (;;)
  {
    const wchar_t* aFound = std::wcsstr(aScan, thePwszOld);
    if (aFound == nullptr)
    {
      break;
    }
    const size_t aChunk = static_cast<size_t>(aFound - aScan);
    std::wmemcpy(aWrite, aScan, aChunk);
    aWrite += aChunk;
    if (aNewLen > 0)
    {
      std::wmemcpy(aWrite, thePwszNew, aNewLen);
      aWrite += aNewLen;
    }
    aScan = aFound + aOldLen;
  }
  const size_t aTail = static_cast<size_t>(aLen) - static_cast<size_t>(aScan - this->m_wsz);
  std::wmemcpy(aWrite, aScan, aTail);
  aWrite += aTail;
  *aWrite = L'\0';

  CopyStorage(this->m_wsz, aScratch.data(), static_cast<unsigned>(aResultLen));
  return static_cast<int>(aCount);
}

//=================================================================================================

int NvString::replace(wchar_t theOldChar, wchar_t theNewChar)
{
  int aCount = 0;
  for (const wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    if (*aPtr == theOldChar)
    {
      ++aCount;
    }
  }
  if (aCount == 0)
  {
    return 0;
  }
  MakeUnique(this->m_wsz);
  for (wchar_t* aPtr = this->m_wsz; *aPtr != L'\0'; ++aPtr)
  {
    if (*aPtr == theOldChar)
    {
      *aPtr = theNewChar;
    }
  }
  return aCount;
}

//=================================================================================================

int NvString::deleteAtIndex(int theIndex, int theCount)
{
  const unsigned aLen = WszLength(this->m_wsz);
  int aStart = (theIndex < 0) ? 0 : theIndex; // clamped, ARX-style
  if (aStart >= static_cast<int>(aLen) || theCount == 0)
  {
    return static_cast<int>(aLen);
  }
  // A negative count means "delete through the end of the string".
  unsigned aNumChars = (theCount < 0) ? (aLen - aStart) : static_cast<unsigned>(theCount);
  if (aStart + static_cast<int>(aNumChars) > static_cast<int>(aLen))
  {
    aNumChars = aLen - aStart;
  }
  MakeUnique(this->m_wsz);
  std::wmemmove(this->m_wsz + aStart, this->m_wsz + aStart + aNumChars,
    static_cast<size_t>(aLen) - aStart - aNumChars + 1);
  this->m_wsz[aLen - aNumChars] = L'\0';
  return static_cast<int>(aLen - aNumChars);
}

//=================================================================================================

NvString NvString::tokenize(const wchar_t* theTokens, int& theStart) const
{
  const unsigned aLen = WszLength(this->m_wsz);
  if (theTokens == nullptr)
  {
    theTokens = L"";
  }
  if (theStart < 0 || theStart >= static_cast<int>(aLen))
  {
    theStart = -1;
    return NvString();
  }
  const bool aNoDelimiters = (theTokens[0] == L'\0');
  const auto isDelimiter = [theTokens, aNoDelimiters](wchar_t theChar)
  {
    return !aNoDelimiters && std::wcschr(theTokens, theChar) != nullptr;
  };

  unsigned aBegin = static_cast<unsigned>(theStart);
  while (aBegin < aLen && isDelimiter(this->m_wsz[aBegin]))
  {
    ++aBegin;
  }
  if (aBegin >= aLen)
  {
    theStart = -1;
    return NvString();
  }
  unsigned aEnd = aBegin;
  while (aEnd < aLen && !isDelimiter(this->m_wsz[aEnd]))
  {
    ++aEnd;
  }
  NvString aToken;
  CopyStorage(aToken.m_wsz, this->m_wsz + aBegin, aEnd - aBegin);
  theStart = (aEnd < aLen) ? static_cast<int>(aEnd) + 1 : -1;
  return aToken;
}

//=================================================================================================

NvString& NvString::setAt(int theIndex, NCHAR theChar)
{
  const int aLen = static_cast<int>(WszLength(this->m_wsz));
  if (theIndex < 0 || theIndex >= aLen)
  {
    throw NvException("NvString::setAt(): index out of range");
  }
  MakeUnique(this->m_wsz);
  this->m_wsz[theIndex] = theChar;
  return *this;
}

//=================================================================================================

NvString& NvString::insert(int theIndex, wchar_t theChar)
{
  const unsigned aLen = WszLength(this->m_wsz);
  int aIndex = theIndex; // clamped to [0, length], ARX-style
  if (aIndex < 0)
  {
    aIndex = 0;
  }
  if (aIndex > static_cast<int>(aLen))
  {
    aIndex = static_cast<int>(aLen);
  }
  EnsureCapacity(this->m_wsz, aLen + 1);
  std::wmemmove(this->m_wsz + aIndex + 1, this->m_wsz + aIndex,
    static_cast<size_t>(aLen) - aIndex + 1);
  this->m_wsz[aIndex] = theChar;
  return *this;
}

//=================================================================================================

NvString& NvString::insert(int theIndex, const wchar_t* thePwsz)
{
  const unsigned aLen = WszLength(this->m_wsz);
  const unsigned aInsertLen = (thePwsz == nullptr) ? 0u : WszLength(thePwsz);
  if (aInsertLen == 0)
  {
    return *this;
  }
  int aIndex = theIndex; // clamped to [0, length], ARX-style
  if (aIndex < 0)
  {
    aIndex = 0;
  }
  if (aIndex > static_cast<int>(aLen))
  {
    aIndex = static_cast<int>(aLen);
  }

  // The inserted text may live inside this string's own buffer.
  const wchar_t* aSource = thePwsz;
  std::vector<wchar_t> aTemp;
  if (BlockOf(thePwsz) == BlockOf(this->m_wsz))
  {
    aTemp.assign(thePwsz, thePwsz + aInsertLen);
    aSource = aTemp.data();
  }
  EnsureCapacity(this->m_wsz, aLen + aInsertLen);
  std::wmemmove(this->m_wsz + aIndex + aInsertLen, this->m_wsz + aIndex,
    static_cast<size_t>(aLen) - aIndex + 1);
  std::wmemcpy(this->m_wsz + aIndex, aSource, aInsertLen);
  return *this;
}

//=================================================================================================

NCHAR* NvString::getBuffer(int theMinBufferLength)
{
  if (theMinBufferLength < 0)
  {
    return nullptr;
  }
  // Detaches shared buffers, drops the utf-8 cache (clients may write) and
  // guarantees room for the requested characters plus the terminator.
  EnsureCapacity(this->m_wsz, static_cast<unsigned>(theMinBufferLength));
  NvStringBlock* aBlock = BlockOf(this->m_wsz);
  aBlock->openLength = static_cast<int>(aBlock->capacity);
  return this->m_wsz;
}

//=================================================================================================

bool NvString::releaseBuffer(int theNewLength)
{
  NvStringBlock* aBlock = BlockOf(this->m_wsz);
  if (aBlock->openLength < 0)
  {
    return false; // no outstanding getBuffer() loan
  }
  const unsigned aMaxLen = static_cast<unsigned>(aBlock->openLength);
  unsigned aNewLen = 0;
  if (theNewLength < 0)
  {
    while (aNewLen < aMaxLen && this->m_wsz[aNewLen] != L'\0')
    {
      ++aNewLen;
    }
  }
  else
  {
    aNewLen = static_cast<unsigned>(theNewLength);
    if (aNewLen > aMaxLen)
    {
      aNewLen = aMaxLen;
    }
  }
  this->m_wsz[aNewLen] = L'\0';
  DropUtf8Cache(aBlock);
  aBlock->openLength = -1;
  return true;
}
