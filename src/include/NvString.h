#ifndef _Nv_String_h_
#define _Nv_String_h_

#include "nvbasedefs.h"
#include "Nova.h"
#include "NvHeapOpers.h"
#include "NvAChar.h"
#include <cstdarg>

class NvDbHandle;
class NvRxResourceInstance;

// Notes:
// 1. All wchar_t arguments are assumed to be "widechar" Unicode values.
// 2. The pointer returned from utf8Ptr() and kwszPtr() is valid until the next time
//    this NvString is modified.
// 3. Never cast away const from a pointer obtained by utf8Ptr() or kwszPtr() buffer, in order
//    to modify the buffer directly. These buffers may be shared by multiple NvString instances.
//    Instead, to operate on buffers directly, first call getBuffer() to obtain a pointer
// 4. Although utf8Ptr() is a const member function, it may reallocate the string's
//    internal buffer and thus invalidate pointers returned by a previous kwszPtr() call
//    utf8Ptr() is the only const method which can modify the NvString
// 5. kwszPtr(), constPtr(), kTCharPtr(), kNVharPtr() and operator const wchar_t *()
//    are all equivalent.  They return a pointer to the null-terminated widechar string.
//    The redundancy is for historical reasons.
// 6. All index values (also known as position values) are 0-based.  For example, in
//    the string "abcd", the 'c' character has position 2
//

class NvString : public NvHeapOperators
{
public:
    /// <summary>Types of narrow char encoding supported.</summary>
    enum Encoding {
        /// <summary>Unicode utf-8 encoding.</summary>
        Utf8
    };

    //
    // Constructors and destructor
    //
    /// <summary>Default ctor, initializes to empty string.</summary>
    NVBASE_PORT NvString();

    /// <summary>Initialize with a single Unicode character</summary>
    /// <param name="wch">input character</param>
    NVBASE_PORT NvString(wchar_t wch);

    /// <summary>Initialize from a narrow char string.</summary>
    /// <param name="psz">Input narrow string. Null terminated.</param>
    /// <param name="encoding"> Input string's encoding format.</param>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT NvString(const char *psz, Encoding encoding);

    /// <summary>Initialize from a narrow char string.</summary>
    /// <param name="psz">Input narrow string. Null terminated.</param>
    /// <param name="encoding">Input string's encoding format.</param>
    /// <param name="nByteCount">Number of input bytes from psz to use.</param>
    /// <remarks>Currently, only Utf8 encoding is supported.
    /// <remarks> psz is not required to be null terminated.
    ///           If bytecount causes last multi-byte utf8 char in psz to be
    ///           truncated, then the behavior is undefined.
    /// </remarks>
    NVBASE_PORT NvString(const char *psz, Encoding encoding, unsigned int nByteCount);

    /// <summary>Initialize from a Unicode string</summary>
    /// <param name="wpsz">input pointer to zero terminated source string</param>
    NVBASE_PORT NvString(const wchar_t *pwsz);

    /// <summary>Initialize from a Unicode string</summary>
    /// <param name="wpsz">input pointer to source</param>
    /// <param name="count"> number of characters to use from the input string</param>
    NVBASE_PORT NvString(const wchar_t *pwsz, unsigned int count);

    /// <summary>Copy constructor</summary>
    /// <param name="acs">input reference to an existing NvString object</param>
    NVBASE_PORT NvString(const NvString & acs);

    /// <summary>Move constructor</summary>
    /// <param name="acs">input reference to an existing temp NvString object</param>
    NVBASE_PORT NvString(NvString && acs);

    /// <summary>Values for the nCtorFlags arg of the following constructor.</summary>
    enum eFormat {
        /// <summary>Format the arg as signed int</summary>
        kSigned = 0x0002,
        /// <summary>Format the arg as unsigned int</summary>
        kUnSigned = 0x0003,
        /// <summary>Format the arg as hexadecimal</summary>
        kHex = 0x0004
    };

    /// <summary>
    ///  Multi-purpose constructor, takes an unsigned argument and
    ///  uses it either to load a resource string or to create a
    ///  numerical string (base 10 or hex).
    /// </summary>
    /// <param name="nCtorFlags">input flags, indicating type of construction</param>
    /// <param name="nArg">input argument value, interpreted according to flags</param>
    NVBASE_PORT NvString(eFormat nCtorFlags, unsigned nArg);
    /// <summary>repeat a character n times.</summary>
    /// <param name="ch">character value</param>
    /// <param name="nRepeatTimes">repate times</param>
    NVBASE_PORT NvString(NCHAR ch, unsigned nRepeatTimes);
    /// <summary>Formats an NvDbHandle value in hex, as in: "a2f".</summary>
    /// <param name="h">input reference to an acdb handle value</param>
    NVBASE_PORT NvString(const NvDbHandle &h);
    /// <summary>Load String from resource instance.</summary>
    /// <param name="hDll">AxResourceInstance object to load string</param>
    /// <param name="nId">input id of the string resource in the specified resource dll</param>
    NVBASE_PORT NvString(const NvRxResourceInstance& hDll, unsigned int nId);

    /// <summary>Destructor: decrements use count and frees memory if the count
    //           goes to zero.</summary>
    NVBASE_PORT ~NvString();

    //
    // Querying methods
    //

    /// <summary>Get the string as utf-8.</summary>
    /// <returns>A pointer to a null-terminated utf8 string.</returns>
    /// <remarks> The pointer is only valid until this object is next modified.
    ///           Warning: this method can modify the object, even though it is const
    /// </remarks>
    NVBASE_PORT const char * utf8Ptr() const;

    /// <summary>Get the string as widechar unicode.</summary>
    /// <returns>A pointer to a null-terminated widechar string.</returns>
    /// <remarks> The pointer is only valid until this object is next modified.
    //            Note that NCHAR is currently always defined as wchar_t.</remarks>
    const wchar_t * kwszPtr() const;
    const wchar_t *  constPtr() const;
    const wchar_t * kTCharPtr() const;
    const NCHAR * kNVharPtr() const;

    /// <summary>Operator for casting this string to a widechar unicode string pointer.</summary>
    /// <returns>A pointer to a null terminated widechar string.</returns>
    /// <remarks>Pointer is valid only until this NvString is next modified.</remarks>
    operator const wchar_t * () const;

    /// <summary>Test whether this string is null, i.e. has logical length zero.</summary>
    /// <returns>True if the string is empty, else false.</returns>
    bool isEmpty() const;

    /// <summary>Get logical length of this string.</summary>
    /// <returns>The number of characters in the string. Zero if it's empty.</returns>
    /// <remarks>Null terminator is not counted in logical length.</remarks>
    //
    NVBASE_PORT unsigned length() const;

    /// <summary>Get logical length of this string.</summary>
    /// <returns>The number of characters in the string. Zero if it's empty.</returns>
    /// <remarks>This method is dDeprecated. Please use length() instead.</remarks>
    unsigned tcharLength() const
    {
        return this->length();
    }

    /// <summary>Get the maximum logical length that this string can currently achieve
    //           without growing or reallocating its buffer.</summary>
    /// <returns>Number of characters the current buffer can hold.</returns>
    /// <remarks>Null terminator is not counted in logical length.</remarks>
    NVBASE_PORT unsigned capacity() const;

    /// <summary>Grows or (possibly) shrinks the buffer to match the requested capacity .</summary>
    /// <param name="nCapacity">Number of characters of space needed, including terminator.</param>
    /// <returns>True if the buffer was re-allocated, else false.</returns>
    /// <remarks>Shrink requests may be ignored, depending on current buffer size,
    ///          string length and refcount.</remarks>
    NVBASE_PORT bool reserve(unsigned nCapacity);

    /// <summary>Check if all characters are in the ascii range: 0x20..0x7f.</summary>
    /// <returns>True if all characters in the ASCII range, else false.</returns>
    /// <remarks>Codes 0x..0x1f are considered control characters and cause this method
    ///          to return false, for historical reasons.</remarks>
    NVBASE_PORT bool isAscii() const;

    /// <summary>Check if all characters are in the range 0x01 through 0x7f.</summary>
    /// <returns>True if all characters have their high bit (0x80) clear.</returns>
    /// <remarks>Codes in 0x01 through 0x7f tend to have the same meaning across
    ///          all encoding schemes (ansi code pages, utf-8, utf-16, etc.</remarks>
    NVBASE_PORT bool is7Bit() const;

    //
    // Parsing methods.
    //

    /// <summary>Flag values specifying how to handle errors such as
    ///          invalid characters or overflow during string parsing.
    ///
    enum {
        ///<summary>Return zero on errors.</summary>
        ///
        kParseZero = 0,

        ///<summary>Return -1 or ffff.</summary>
        ///
        kParseMinus1 = 0x01,

        ///<summary>Pop an assert in debug build.</summary>
        ///
        kParseAssert = 0x02,

        ///<summary>Throw an int exception.</summary>
        ///
        kParseExcept = 0x04,

        ///<summary>Treat empty string as error.</summary>
        ///
        kParseNoEmpty = 0x08,

        ///<summary>Default error handling behavior.</summary>
        ///
        kParseDefault = (kParseAssert | kParseZero)
    };

    /// <summary>Parse the current string as decimal, return a signed int.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT int asDeci(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as hexadecimal, return a signed int.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT int asHex(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as decimal, return an unsigned int.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT unsigned int asUDeci(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as hexadecimal, return an unsigned int.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT unsigned int asUHex(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as decimal, return a signed int64.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT int64_t asDeci64(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as hexadecimal, return a signed int64.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT int64_t asHex64(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as decimal, return an unsigned int64.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT Nova::UInt64 asUDeci64(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as hexadecimal, return an unsigned int64.</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The int value parsed from the string.</returns>
    NVBASE_PORT Nova::UInt64 asUHex64(int nFlags = kParseDefault) const;

    /// <summary>Parse the current string as hexadecimal.  Return the handle..</summary>
    /// <param name="nFlags">input bits specifying how to do the parsing</param>
    /// <returns>The handle value parsed from the string.</returns>
    NVBASE_PORT NvDbHandle asNvDbHandle(int nFlags = kParseDefault) const;

    //
    // Find methods:
    //   Search for a character, a substring or for any of a group of characters,
    //   or for any character not in the group.
    //   Search for first or last occurrence.
    //

    /// <summary>Find a single character in the string..</summary>
    /// <param name="ch">input character to search for</param>
    /// <returns>The position of the found character. -1 if not found.</returns>
    /// <remarks>The first character position is zero.</remarks>
    int find(NCHAR ch) const;

    /// <summary> Find a single character in the string.  </summary>
    /// <param name="ch">input character to search for</param>
    /// <param name="nStartPos">first character position to look at</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    NVBASE_PORT int find(NCHAR ch, int nStartPos) const;

    /// <summary> Find a substring in the string </summary>
    /// <param name="psz">input string to search for</param>
    /// <param name="nStartPos">first character position in this string to look at</param>
    /// <returns> The position of the found substring. -1 if not found.</returns>
    NVBASE_PORT int find(const NCHAR *psz, int nStartPos = 0) const;

    /// <summary>Find an NvString in the string..</summary>
    /// <param name="acs">input string object to search for
    /// <returns>The position of the found string, -1 if not found.</returns>
    NVBASE_PORT int find(const NvString & acs) const;

    /// <summary> Find first character in this string which matches any in a group of
    ///           input characters</summary>
    /// <param name="psz">input group of characters to search for</param>
    /// <param name="nStartPos">first character position in this string to look at</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    /// <remarks> If psz is null, then we search for whitespace.</remarks>
    NVBASE_PORT int findOneOf(const NCHAR *psz, int nStartPos = 0) const;

    /// <summary> Find first character in this string which does not match any in a group
    ///           of input characters</summary>
    /// <param name="psz">input group of characters to search for</param>
    /// <param name="nStartPos">first character position to look at</param>
    /// <returns> The position of the "not found" character. -1 if all were found.</returns>
    /// <remarks> If psz is null, then we search for whitespace.</remarks>
    NVBASE_PORT int findNoneOf(const NCHAR *psz, int nStartPos = 0) const;

    /// <summary> Find last occurrence of a character in the string</summary>
    /// <param name="ch">input character to search for</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    /// <remarks> This method is DEPRECATED. Please use findLast() instead.</remarks>
    int findRev(NCHAR ch) const;

    /// <summary> Find last occurrence of a substring in the string</summary>
    /// <param name="psz">input substring to search for</param>
    /// <returns> The position of the found substring result. -1 if not found.</returns>
    /// <remarks> This method is DEPRECATED. Please use findLast() instead.</remarks>
    int findRev(const NCHAR *psz) const;

    /// <summary> Find last occurrence of a substring in the string</summary>
    /// <param name="acs">input substring to search for</param>
    /// <returns> The position of the found substring result. -1 if not found.</returns>
    /// <remarks> This method is DEPRECATED. Please use findLast() instead.</remarks>
    int findRev(const NvString & acs) const;

    /// <summary> Find last character in this string from a group of characters</summary>
    /// <param name="psz">input group of characters to match</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    /// <remarks> This method is DEPRECATED. Please use findLast() instead.</remarks>
    int findOneOfRev(const NCHAR *psz) const;

    /// <summary> Find last occurrence of a character in the string</summary>
    /// <param name="ch">input character to search for</param>
    /// <param name="nEndPos">Last character position to look at</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    int findLast(NCHAR ch, int nEndPos = -1) const;

    /// <summary> Find last occurrence of a substring in the string</summary>
    /// <param name="psz">input substring to search for</param>
    /// <param name="nEndPos">Last character position to look at</param>
    /// <returns> The position of the found substring result. -1 if not found.</returns>
    NVBASE_PORT int findLast(const NCHAR *psz, int nEndPos = -1) const;

    /// <summary> Find last character in this string from a group of characters</summary>
    /// <param name="psz">input group of characters to match</param>
    /// <param name="nEndPos">Last character position to look at</param>
    /// <returns> The position of the found character. -1 if not found.</returns>
    /// <remarks> If psz is null, then we search for whitespace.</remarks>
    NVBASE_PORT int findLastOneOf(const NCHAR *psz, int nEndPos = -1) const;

    /// <summary> Find last character in this string which does not match any in a group
    ///           of characters</summary>
    /// <param name="psz">input group of characters to search for</param>
    /// <param name="nEndPos">first character position to look at</param>
    /// <returns> The position of the "not found" character. -1 if all were found.</returns>
    /// <remarks> If psz is null, then we search for whitespace.</remarks>
    NVBASE_PORT int findLastNoneOf(const NCHAR *psz, int nEndPos = -1) const;

    //
    // Extraction methods
    // Note: mid() and substr() are the same thing - we define both
    //       for compatibility with CString and std::string
    //
    // The input index arguments are character indices into the string.

    /// <summary>Get substring from the a specified position to the string's end.</summary>
    /// <param name="nStart">The zero-based start position of the substring to get.</param>
    /// <returns>An NvString consisting of the specified substring</returns>
    NvString mid(int nStart) const;

    /// <summary>Get a substring from the string.  (same as substr() method).</summary>
    /// <param name="nStart">input index (in characters) from the start of the string</param>
    /// <param name="nNumChars">input number of characters to retrieve.
    //              If nNumChars is -1, then return the rest of the string.</param>
    /// <returns>An NvString consisting of the specified substring</returns>
    NvString mid(int nStart, int nNumChars) const;

    /// <summary>Get a substring from the start of string..</summary>
    /// <param name="nNumChars">input number of characters to retrieve.</param>
    //             if nNumChars is -1, then return the rest of the string
    /// <returns>An NvString consisting of the specified substring</returns>
    NvString substr(int numChars) const;

    /// <summary>Get a substring from the string.  (same as mid() method).</summary>
    /// <param name="nStart">input 0-based index from the start of the string</param>
    /// <param name="nNumChars">input number of characters to retrieve.</param>
    //             if nNumChars is -1, then return the rest of the string
    /// <returns>An NvString consisting of the specified substring</returns>
    NVBASE_PORT NvString substr(int nStart, int nNumChars) const;

    /// <summary>Get a substring from the end of string..</summary>
    /// <param name="nNumChars">input number of characters to retrieve.</param>
    /// <returns>An NvString consisting of the specified substring</returns>
    NVBASE_PORT NvString substrRev(int numChars) const;

    /// <summary>
    /// Return a nNumChars length substring from the start of string.
    /// </summary>
    /// <param name="nNumChars">The count of characters of the substring to get.</param>
    /// <returns>An NvString consisting of the specified substring</returns>
    NvString left(int nNumChars) const;

    /// <summary>
    /// Return a nNumChars length substring from the end of string.
    /// </summary>
    /// <param name="nNumChars">The count of characters of the substring to get.</param>
    /// <returns>An NvString consisting of the specified substring</returns>
    NvString right(int nNumChars) const;

    //
    // Assignment operators and methods
    //
    
    /// <summary>assign a Unicode character to the string.</summary>
    /// <param name="wch">input character to assign</param>
    /// <returns>A reference to this string object.</returns>
    NvString & assign(wchar_t wch);

    /// <summary>assign a string of narrow chars to the string.</summary>
    /// <param name="psz">input pointer to the string of narrow chars to assign</param>
    /// <param name="encoding"> input Encoding type</param>
    /// <returns>A reference to this string object.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
   NVBASE_PORT  NvString & assign(const char *psz, Encoding encoding);

    /// <summary>assign a string of Unicode characters to the string.</summary>
    /// <param name="pwsz">input pointer to the string of characters to assign</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & assign(const wchar_t *pwsz);

    /// <summary>assign an NvString object to the string.</summary>
    /// <param name="acs">input reference to the NvString</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & assign(const NvString & acs);

    /// <summary>assign an NvDbHandle object to the string (format it as hex).</summary>
    /// <param name="h">input reference to the NvDbHandle object</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & assign(const NvDbHandle & h);

    /// <summary>assign a Unicode character to the string.</summary>
    /// <param name="wch">input character to assign</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator = (wchar_t wch);

    /// <summary>assign a string of Unicode characters to the string.</summary>
    /// <param name="pwsz">input pointer to the string of characters to assign</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator = (const wchar_t *pwsz);

    /// <summary>assign an NvString object to the string.</summary>
    /// <param name="acs">input reference to the NvString</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator = (const NvString & acs);

    /// <summary>move a temp NvString object to the string.</summary>
    /// <param name="acs">input reference to the temp NvString</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & operator = (NvString && acs);

    /// <summary>assign an NvDbHandle object to the string (format it as hex).</summary>
    /// <param name="h">input reference to the NvDbHandle object</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator = (const NvDbHandle & h);

    /// <summary>Set the string to be empty..</summary>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & setEmpty();

    /// <summary>Set the string from a resource string.</summary>
    /// <param name="hDll">AxResourceInstance object to load string</param>
    /// <param name="nId">input id of the string resource in the specified resource dll</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT bool loadString(const NvRxResourceInstance& hDll, unsigned nId);

    /// <summary>Format the string using "printf" rules..</summary>
    /// <param name="pszFmt">input pointer to the printf format string</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & format (const NCHAR    *pszFmt,  ...);

    /// <summary>Format the string using "printf" rules.</summary>
    /// <param name="pszFmt">input pointer to the printf format string</param>
    /// <param name="args">input variable args list, containing values to be formatted</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & formatV(const NCHAR   *pszFmt,  va_list args);

    /// <summary>
    /// Append formated data to this string using "printf" rules
    /// </summary>
    /// <param name="pszFmt">input pointer to the printf format string</param>
    /// <param name="args">input variable args list, containing values to be formatted</param>
    /// <returns> Reference to this NvString.</returns>
    NVBASE_PORT NvString & appendFormat(const NCHAR   *pszFmt, ...);

    //
    // Modifying operators and methods
    //

    /// <summary>append a Unicode character to the end of the string.</summary>
    /// <param name="wch">input character to append</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator += (wchar_t wch);

    /// <summary>append a Unicode string to the end of the string.</summary>
    /// <param name="pwsz">input pointer to the Unicode string</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator += (const wchar_t * pwsz);

    /// <summary>append an NvString object to the end of the string.</summary>
    /// <param name="acs">input reference to the NvString</param>
    /// <returns>A reference to this string object.</returns>
    NvString & operator += (const NvString & acs);

    /// <summary>append a Unicode character to the end of the string.</summary>
    /// <param name="wch">input character to append</param>
    /// <returns>A reference to this string object.</returns>
    NvString & append(wchar_t wch);

    /// <summary>append a char string to the end of the string.</summary>
    /// <param name="psz">input pointer to the narrow char string</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>A reference to this string object.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT NvString & append(const char *psz, Encoding encoding);

    /// <summary>append a Unicode string to the end of the string.</summary>
    /// <param name="pwsz">input pointer to the Unicode string</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & append(const wchar_t *pwsz);

    /// <summary>append an NvString object to the end of the string.</summary>
    /// <param name="acs">input reference to the NvString</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString & append(const NvString & acs);

    // Catenation operators and methods  These are like append,
    // but they do not modify the current string.  They return a
    // new combined string.

    /// <summary>Copy the string and append a Unicode character to it.</summary>
    /// <param name="ch">input character to append to the string copy</param>
    /// <returns>A reference to this string object.</returns>
    NvString operator + (wchar_t wch) const;

    /// <summary>Copy the string and append a string of Unicode characters to it.</summary>
    /// <param name="pwsz">input pointer to the string to append</param>
    /// <returns>A reference to this string object.</returns>
    NvString operator + (const wchar_t * pwsz) const;

    /// <summary>Copy the string and append an NvString to it.</summary>
    /// <param name="pwsz">input reference to the NvString to append</param>
    /// <returns>A reference to this string object.</returns>
    NvString operator + (const NvString & acs) const;

    /// <summary>Copy the string and append a Unicode character to it.</summary>
    /// <param name="ch">input character to append to the string copy</param>
    /// <returns>A reference to this string object.</returns>
    NvString concat(wchar_t wch) const;

    /// <summary>Copy the string and append a string of narrow chars to it.</summary>
    /// <param name="psz">input pointer to the narrow string to append</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>A reference to this string object.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT NvString concat(const char * psz, Encoding encoding) const;

    /// <summary>Copy the string and append a string of Unicode characters to it.</summary>
    /// <param name="pwsz">input pointer to the string to append</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString concat(const wchar_t * pwsz) const;

    /// <summary>Copy the string and append an NvString to it.</summary>
    /// <param name="pwsz">input reference to the NvString to append</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString concat(const NvString & acs) const;

    // These copy the current string and then insert the character or
    // string in front of it.  They're used by the global "+" operators.

    /// <summary>Copy the string and insert a character in front of it.</summary>
    /// <param name="ch">input character to insert</param>
    /// <returns>A reference to this string object.</returns>
    NvString precat(NCHAR ch) const;

    /// <summary>Copy the string and insert a string of narrow chars in front of it.</summary>
    /// <param name="psz">input pointer to the string of narrow chars to insert</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>A reference to this string object.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT NvString precat(const char * psz, Encoding encoding) const;

    /// <summary>Copy the string and insert a string of characters in front of it.</summary>
    /// <param name="psz">input pointer to the string of characters to insert</param>
    /// <returns>A reference to this string object.</returns>
    NVBASE_PORT NvString precat(const wchar_t * psz) const;

    //
    // Comparison operators and methods
    // The int return value is -1, 0 or 1, indicating <, == or >
    //

    /// <summary>Compare the string to a single Unicode char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>0 if this string equals wch, a value < 0 if this string is less
    ///          than wch and a value > 0 if this string is greater than wch.</returns>
    int  compare(wchar_t wch) const;

    /// <summary>Compare the string to a string of narrow chars.</summary>
    /// <param name="psz">input pointer to the string of narrow chars to compare to</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>0 if this string equals psz, a value < 0 if this string is less
    ///          than psz and a value > 0 if this string is greater than psz.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT int  compare(const char *psz, Encoding encoding) const;

    /// <summary>Compare the string to a string of Unicode characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters to compare to</param>
    /// <returns>0 if this string equals pwsz, a value < 0 if this string is less
    ///          than pwsz and a value > 0 if this string is greater than pwsz.</returns>
    NVBASE_PORT int  compare(const wchar_t *pwsz) const;

    /// <summary>Compare the string to a string of Unicode characters.</summary>
    /// <param name="acs">input reference of the other NvString to compare to</param>
    /// <returns>0 if this string equals acs, a value < 0 if this string is less
    ///          than acs and a value > 0 if this string is greater than acs.</returns>
    int  compare(const NvString & acs) const;


    /// <summary>Compare the string to another string using collation.</summary>
    /// <param name="pwsz">input pointer to the string of characters to compare to</param>
    /// <returns>0 if this string equals pwsz, a value < 0 if this string is less
    ///          than pwsz and a value > 0 if this string is greater than pwsz.</returns>
    NVBASE_PORT int  collate (const wchar_t *pwsz) const;
    
    /// <summary>Compare the string to another NvString object using collation.</summary>
    /// <param name="acs">input NvString object to compare to </param>
    /// <returns>0 if this string equals acs, a value < 0 if this string is less
    ///          than acs and a value > 0 if this string is greater than acs.</returns>
    int  collate(const NvString & acs) const;

    /// <summary>Compare the string case-independently to a Unicode char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>0 if this string equals wch, a value < 0 if this string is less
    ///          than wch and a value > 0 if this string is greater than wch.</returns>
    int  compareNoCase(wchar_t wch) const;

    /// <summary>Compare the string case-independently to a string of narrow chars.</summary>
    /// <param name="psz">input pointer to the string of narrow chars to compare to</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>0 if this string equals psz, a value < 0 if this string is less
    ///          than psz and a value > 0 if this string is greater than psz.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT int  compareNoCase(const char *psz, Encoding encoding) const;

    /// <summary>Compare the string case-independently to a string of Unicode characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters to compare to</param>
    /// <returns>0 if this string equals pwsz, a value < 0 if this string is less
    ///          than pwsz and a value > 0 if this string is greater than pwsz.</returns>
    NVBASE_PORT int  compareNoCase(const wchar_t *pwsz) const;

    /// <summary>Compare the string case-independently to another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>0 if this string equals acs, a value < 0 if this string is less
    ///          than acs and a value > 0 if this string is greater than acs.</returns>
    int  compareNoCase(const NvString & acs) const;

    /// <summary>Compares two NvStrings for equality, ignoring case.</summary>
    /// <param name="left"> one NvString object </param>
    /// <param name="right"> another NvString object </param>
    /// <returns>true if left equals right, else false.</returns>
    /// <remarks>May be useful as a comparator function for STL functions.</remarks>
    static bool equalsNoCase(const NvString& left, const NvString& right);

    /// <summary>Compare case-independently to another string using collation.</summary>
    /// <param name="psz">input pointer to the string of characters to compare to </param>
    /// <returns>0 if this string equals psz, a value < 0 if this string is less
    ///          than psz and a value > 0 if this string is greater than psz.</returns>
    NVBASE_PORT int collateNoCase(const wchar_t *psz) const;

    /// <summary>Compare case-independently to another NvString using collation./// </summary>
    /// <param name="acs"> input reference to the other NvString to compare to </param>
    /// <returns>0 if this string equals acs, a value < 0 if this string is less
    ///          than acs and a value > 0 if this string is greater than acs.</returns>
    int collateNoCase(const NvString& acs) const;

    /// <summary>Compare this string for equality with a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string equals wch, else false.</returns>
    bool operator == (wchar_t wch) const;

    /// <summary>Compare the string for equality with a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>True if this string equals pwsz, else false.</returns>
    bool operator == (const wchar_t *pwsz) const;

    /// <summary>Compare the string for equality with another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string equals acs, else false.</returns>
    bool operator == (const NvString & acs) const;

    /// <summary>Compare the string for non-equality with a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string does not equal wch, false if they are equal.</returns>
    bool operator != (wchar_t wch) const;

    /// <summary>Compare the string for non-equality with a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>True if this string does not equal pwsz, false if they are equal.</returns>
    bool operator != (const wchar_t *pwsz) const;

    /// <summary>Compare the string for non-equality with another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string does not equal acs, false if they are equal.</returns>
    bool operator != (const NvString & acs) const;

    /// <summary>Compare the string for greater than a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string is greater than wch, false otherwise.</returns>
    bool operator >  (wchar_t wch) const;

    /// <summary>Compare the string for greater than a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters to compare to</param>
    /// <returns>True if this string is greater than pwsz, false otherwise.</returns>
    bool operator >  (const wchar_t *pwsz) const;

    /// <summary>Compare the string for greater than another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string is greater than acs, false otherwise.</returns>
    bool operator >  (const NvString & acs) const;

    /// <summary>Compare the string for greater than or equal to a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string is greater than or equal to wch, false otherwise.</returns>
    bool operator >= (wchar_t wch) const;

    /// <summary>Compare for greater than/equal to a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>True if this string is greater than or equal to pwsz, false otherwise.</returns>
    bool operator >= (const wchar_t *pwsz) const;

    /// <summary>Compare the string for greater than or equal to another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string is greater than or equal to acs, false otherwise.</returns>
    bool operator >= (const NvString & acs) const;

    /// <summary>Compare the string for less than a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string is less than wch, false otherwise.</returns>
    bool operator <  (wchar_t wch) const;

    /// <summary>Compare the string for less than a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters to compare to</param>
    /// <returns>True if this string is less than pwsz, false otherwise.</returns>
    bool operator <  (const wchar_t *pwsz) const;

    /// <summary>Compare the string for less than another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string is less than acs, false otherwise.</returns>
    bool operator <  (const NvString & acs) const;

    /// <summary>Compare the string for less than or equal to a wide char.</summary>
    /// <param name="wch">input character to compare to</param>
    /// <returns>True if this string is less than or equal to wch, false otherwise.</returns>
    bool operator <= (wchar_t wch) const;

    /// <summary>Compare the string for less than/equal to a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>True if this string is less than or equal to pwsz, false otherwise.</returns>
    bool operator <= (const wchar_t *pwsz) const;

    /// <summary>Compare the string for less or equal to than another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>True if this string is less than or equal to acs, false otherwise.</returns>
    bool operator <= (const NvString & acs) const;

    // The match() methods return the number of positions in the string that have the same
    // character value as that position in the other string.  E.g., matching "abcd" and
    // "abxyz" returns 2.  Matching "abc" and "xyz" returns 0.

    /// <summary>See how many characters match a string of narrow chars.</summary>
    /// <param name="psz">input pointer to the string of narrow chars</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>The number of characters that match psz.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT int  match(const char *psz, Encoding encoding) const;

    /// <summary>See how many characters match a string of wide characters.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>The number of characters that match pwsz.</returns>
    NVBASE_PORT int  match(const wchar_t *pwsz) const;

    /// <summary>See how many characters match another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>The number of characters that match pwsz.</returns>
    NVBASE_PORT int  match(const NvString & acs) const;

    /// <summary>See how many characterss case-independently match a narrow string.</summary>
    /// <param name="psz">input pointer to the string of narrow chars</param>
    /// <param name="encoding">input Encoding type</param>
    /// <returns>The number of characters that match psz.</returns>
    /// <remarks>Currently, only Utf8 encoding is supported.</remarks>
    NVBASE_PORT int  matchNoCase(const char *psz, Encoding encoding) const;

    /// <summary>See how many characters case-independently match a wide string.</summary>
    /// <param name="pwsz">input pointer to the string of characters</param>
    /// <returns>The number of characters that match pwsz.</returns>
    NVBASE_PORT int  matchNoCase(const wchar_t *pwsz) const;

    /// <summary>See how many characters case-independently match another NvString.</summary>
    /// <param name="acs">input reference to the other NvString</param>
    /// <returns>The number of characters that match acs.</returns>
    NVBASE_PORT int  matchNoCase(const NvString & acs) const;

    /// <summary>Convert this string's lowercase characters to upper case.</summary>
    /// <returns> Reference to this NvString.</returns>
    NVBASE_PORT NvString & makeUpper();
    
    /// <summary>Convert this string's uppercase characters to lower case.</summary>
    /// <returns> Reference to this NvString.</returns>
    NVBASE_PORT NvString & makeLower();
    
    /// <summary>Reverse the characters in this string./// </summary>
    /// <returns>A reference to this NvString.</returns>
    NVBASE_PORT NvString& makeReverse();

    /// <summary> Remove all occurrences of a character from front of this string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> No-op if wch arg is null character.</remarks>
    ///
    NVBASE_PORT NvString & trimLeft(wchar_t wch);

    /// <summary> Remove all occurrences of a character from end of this string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> No-op if wch arg is null character.</remarks>
    ///
    NVBASE_PORT NvString & trimRight(wchar_t wch);

    /// <summary> Remove all occurrences of a character from both ends of this string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> No-op if wch arg is null character.</remarks>
    ///
    NVBASE_PORT NvString & trim(wchar_t wch);

    /// <summary> Remove all whitespace characters from beginning of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    ///
    NvString & trimLeft();

    /// <summary> Remove all designated characters from beginning of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> Trims whitespace if pwszChars arg is null.</remarks>
    ///
    NVBASE_PORT NvString & trimLeft(const wchar_t *pwszChars);

    /// <summary> Remove all whitespace characters from the end of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    ///
    NvString & trimRight();

    /// <summary> Remove all designated characters from the end of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> Trims whitespace if pwszChars arg is null.</remarks>
    ///
    NVBASE_PORT NvString & trimRight(const wchar_t *pwszChars);

    /// <summary> Remove all whitespace characters from both ends of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    ///
    NvString & trim();

    /// <summary> Remove all designated characters from both ends of the string.</summary>
    /// <returns> Reference to this NvString.</returns>
    /// <remarks> Trims whitespace if pwszChars arg is null.</remarks>
    ///
    NVBASE_PORT NvString & trim(const wchar_t *pwszChars);

    /// <summary> Remove all occurrences of the specified character.</summary>
    /// <returns> Number of characters removed. Zero if the string was not changed.</returns>
    /// <remarks> Removes whitespace characters if wch arg is zero.</remarks>
    ///
    NVBASE_PORT int remove(wchar_t wch);

    /// <summary> Remove all occurrences of whitespace.</summary>
    /// <returns> Number of characters removed. Zero if the string was not changed.</returns>
    ///
    int remove()
    {
        return this->remove(0);
    }

    /// <summary> Extract substring up to the first instance of a designated character.</summary>
    /// <returns> NvString that contains the substring</returns>
    ///
    NVBASE_PORT NvString spanExcluding(const wchar_t *pwszChars) const;


#if defined(_AFX) || defined(__OSX_WINAPI_UNIX_STRING_H__) || defined(__ATLSTR_H__)
#if !defined (_NOVA_CROSS_PLATFORM_)  // not allowed in cross-platform code
#define ENABLE_NVSTRING_CSTRING_OPERATORS
#endif
#endif

#if defined(ENABLE_NVSTRING_CSTRING_OPERATORS)
    //
    // MFC CString-using methods.  The CStringA class is the ansi
    // code page based CString, while CStringW is Unicode based.
    // CString maps to one or the other depending on whether the
    // UNICODE preprocessor symbol is defined.
    //

    /// <summary>Construct an NvString from a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    NvString(const CStringW &csw);

    /// <summary>Initialize this NvString from a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>A reference to this NvString object.</returns>
    NvString & operator = (const CStringW &csw);

    /// <summary>Append a CStringW to this NvString.</summary>
    /// <param name="csa">input reference to the CStringW</param>
    /// <returns>A reference to this NvString object.</returns>
    NvString & operator += (const CStringW &csw);

    /// <summary>Compare this string to a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>0 if this string equal csw, a value < 0 if tihs string is
    ///          is less than csw, a value > 0 if this string is greater than csw.</returns>
    int  compare(const CStringW & csw) const;

    /// <summary>Compare this string case independently to a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>0 if this string equal csw, a value < 0 if tihs string is
    ///          is less than csw, a value > 0 if this string is greater than csw.</returns>
    int  compareNoCase(const CStringW & csw) const;

    /// <summary>Compare for equality with a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string equals the CStringW, else false.</returns>
    bool operator == (const CStringW & ) const;

    /// <summary>Compare for non-equality with a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string does not equal the CStringW, false if they're equal.</returns>
    bool operator != (const CStringW & ) const;

    /// <summary>Compare for less than a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string is less than the CStringW, else false.</returns>
    bool operator <  (const CStringW & ) const;

    /// <summary>Compare for less than or equal to a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string is less than or equal to the CStringW, else false.</returns>
    bool operator <= (const CStringW & ) const;

    /// <summary>Compare for greater than a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string is greater than the CStringW, else false.</returns>
    bool operator >  (const CStringW & ) const;

    /// <summary>Compare for greater than or equal to a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>True if this string is greater than or equal to the CStringW, else false.</returns>
    bool operator >= (const CStringW & ) const;

    /// <summary>Get number of characters matching a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>The number of characters in this string matching csw.</returns>
    int  match(const CStringW & csw) const;

    /// <summary>Return number of characters case-independently matching a CStringW.</summary>
    /// <param name="csw">input reference to the CStringW</param>
    /// <returns>The number of characters in this string matching csw.</returns>
    int  matchNoCase(const CStringW & csw) const;
#endif

    /// <summary>
    /// Rplaces instances of the substring with instances of the new string 
    /// </summary>
    /// <param name="pwszOld"> A pointer to a string containing the character to be replaced by lpszNew. </param>
    /// <param name="pwszNew"> A pointer to a string containing the character replacing lpszOld. </param>
    /// <returns>The number of replaced instances of the substring.
    ///          Zero if the string is not changed.</returns>
    NVBASE_PORT  int replace(const wchar_t* pwszOld, const wchar_t* pwszNew);

    /// <summary>
    /// Replace a character with another.
    /// </summary>
    /// <param name="wchOld"> character that will be replaced </param>
    /// <param name="wchNew"> new character that will be replaced with </param>
    /// <returns>The number of replaced instances of the wchOld.
    ///          Zero if the string is not changed.</returns>
    NVBASE_PORT int replace(wchar_t wchOld, wchar_t wchNew);

    /// <summary>
    /// Deletes character(s) from a string starting with the character at given index.
    /// </summary>
    /// <param name="iIndex"> start position to delete </param>
    /// <param name="nCount"> character number to be deleted </param>
    /// <returns>Return the length of the changed string.</returns>
    NVBASE_PORT int deleteAtIndex(int iIndex, int nCount = 1);

    /// <summary>
    /// Finds the next token in a target string
    /// </summary>
    /// <param name="pszTokens">A string containing token delimiters. The order of these delimiters is not important.</param>
    /// <param name="iStart">The zero-based index to begin the search.</param>
    /// <returns>An NvString containing the current token value.</returns>
    NVBASE_PORT NvString tokenize(const wchar_t* pszTokens, int& iStart) const;


    /// <summary>
    /// Set the character at the given postion to the specified character.
    /// </summary>
    /// <param name="nIndex">Zero-based postion of character in the string.</param>
    /// <param name="ch">The new character to replace the old one.</param>
    /// <returns>A reference to this NvString object.</returns>
    NVBASE_PORT NvString& setAt(int nIndex, NCHAR ch);

    /// <summary>
    /// Get one character at the given postion from the string.
    /// </summary>
    /// <param name="nIndex">Zero-based postion of character in the string.</param>
    /// <returns> Return the character at the specified position in the string </returns>
    /// <remarks> Does NOT do range checking on the nIndex arg.
    ///           Results for out of range nIndex args are unpredictable.
    ///           Indexing via [] may also work, causing an implicit call to
    ///           the const wchar_t * operator
    /// </remarks>
    wchar_t getAt(int nIndex) const;

    /// <summary>
    /// Inserts a single character at the given index within the string.
    /// </summary>
    /// <param name="nIndex">The index of the character before which the insertion will take place.</param>
    /// <param name="ch">The character to be inserted.</param>
    /// <returns>A reference to this NvString object.</returns>
    NVBASE_PORT NvString& insert(int nIndex, wchar_t ch);

    /// <summary>
    /// Inserts a substring at the given index within the string.
    /// </summary>
    /// <param name="nIndex">The index of the character before which the insertion will take place.</param>
    /// <param name="ch">A pointer to the substring to be inserted.</param>
    /// <returns>A reference to this NvString object.</returns>
    NVBASE_PORT NvString& insert(int nIndex, const wchar_t* pwsz);

    /// <summary>
    /// Returns a pointer to the internal character buffer of the string object, allowing
    /// direct access to and modification of the string contents.
    /// The returned buffer contains the string contents and is null terminated.
    /// The buffer is at least large enough to hold nMinBufferLength characters plus
    /// a null terminator.  Buffer memory after the terminator may be uninitialized.
    /// Clients should call releaseBuffer() when they're done accessing the buffer, and they
    /// should not call any other methods (except an implicit call to the dtor) before then.
    /// </summary>
    /// <param name="nMinBufferLength">Number of characters that should fit in the buffer,
    /// not including the null terminator.
    /// </param>
    /// <returns>
    /// wchar_t pointer to the NvString's (null-terminated) character buffer.
    /// The call fails and returns nullptr if nMinBufferLength is < 0.
    /// </returns>
    NVBASE_PORT NCHAR* getBuffer(int nMinBufferLength = 0);

    /// <summary>
    /// Use releaseBuffer() to end the use of a buffer allocated by the getBuffer() method.
    /// The pointer returned by getBuffer() is invalid after the call to releaseBuffer().
    /// </summary>
    /// <param name="nMinBufferLength">Sets the new length of the NvString.
    /// If -1, then the string's length is determined by the null terminator's index.
    /// Otherwise the new length is set to the minimum of nMinBufferLength and the null
    /// terminator's index.
    /// </param>
    /// <returns>True if success, false on errors such as invalid length args or
    /// no previous call to getBuffer().</returns>
    NVBASE_PORT bool   releaseBuffer(int nNewLength = -1);

private:

    friend class NvStringImp;
    wchar_t *m_wsz;
};


#ifdef NV_NVARRAY_H
typedef
NvArray< NvString, NvArrayObjectCopyReallocator< NvString > > NvStringArray;
#endif

//
// Global operator declarations
//

/// <summary>Compare an NvString and a Unicode character for equality.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch and acs are equal, else false.</returns>
bool operator == (wchar_t wch, const NvString & acs);

/// <summary>Compare an NvString and a string of Unicode characters for equality.</summary>
/// <param name="pwsz">input character to the string of Unicode characters</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz and acs are equal, else false.</returns>
bool operator == (const wchar_t *pwsz, const NvString & acs);

/// <summary>Compare an NvString and a Unicode character for non-equality.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch and acs are not equal, else false.</returns>
bool operator != (wchar_t wch, const NvString & acs);

/// <summary>Compare an NvString and a string of Unicode characters for non-equality.</summary>
/// <param name="pwsz">input ptr to the string of Unicode characters</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz and acs are not equal, else false.</returns>
bool operator != (const wchar_t *pwsz, const NvString & acs);

/// <summary>Return whether a Unicode character is greater than an NvString.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch is greater than acs, else false.</returns>
bool operator >  (wchar_t wch, const NvString & acs);

/// <summary>Return whether a string of Unicode characters is greater than an NvString.</summary>
/// <param name="pwsz">input pointer to the string of Unicode characters</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz is greater than acs, else false.</returns>
bool operator >  (const wchar_t *pwsz, const NvString & acs);

/// <summary>Check for a Unicode character being greater than or equal to an NvString.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch is greater than or equal to acs, else false.</returns>
bool operator >= (wchar_t wch, const NvString & acs);

/// <summary>Check for a string of Unicode characters being greater than/equal to an NvString.</summary>
/// <param name="pwsz">input string to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz is greater than or equal to acs, else false.</returns>
bool operator >= (const wchar_t *pwsz, const NvString & acs);

/// <summary>Check for a Unicode character being less than an NvString.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch is less than acs, else false.</returns>
bool operator <  (wchar_t wch, const NvString & acs);

/// <summary>Check for a string of Unicode characters being less than an NvString.</summary>
/// <param name="pwsz">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz is less than acs, else false.</returns>
bool operator <  (const wchar_t *pwsz, const NvString & acs);

/// <summary>Check for a Unicode character being less than or equal to an NvString.</summary>
/// <param name="wch">input character to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if wch is less than or equal to acs, else false.</returns>
bool operator <= (wchar_t wch, const NvString & acs);


/// <summary>Check for a string of Unicode characters being less than/equal to an NvString.</summary>
/// <param name="pwsz">input characters to compare</param>
/// <param name="acs">input reference to the NvString</param>
/// <returns>True if pwsz is less than or equal to acs, else false.</returns>
bool operator <= (const wchar_t *pwsz, const NvString & acs);

/// <summary>Copy an NvString and insert a Unicode characters in front of it.</summary>
/// <param name="wch">input characters to insert</param>
/// <returns>An NvString consisting of the concatenation of wch and acs.</returns>
NvString operator + (wchar_t wch, const NvString & acs);

/// <summary>Copy an NvString and insert a string of Unicode characters in front of it.</summary>
/// <param name="pwsz">input pointer to the string of characters to insert</param>
/// <returns>An NvString consisting of the concatenation of pwsz and acs.</returns>
NvString operator + (const wchar_t *pwsz, const NvString & acs);

// Nvcessing inlines
//

inline NvString::operator const wchar_t *() const
{
    return this->kwszPtr();
}

inline const wchar_t * NvString::constPtr() const
{
    return this->kwszPtr();
}

inline const wchar_t * NvString::kTCharPtr() const
{
    return this->kwszPtr();
}

inline const NCHAR * NvString::kNVharPtr() const
{
    return this->kwszPtr();
}

inline bool NvString::isEmpty() const
{
    return this->m_wsz[0] == L'\0';
}

inline wchar_t NvString::getAt(int nIndex) const
{
    return this->m_wsz[nIndex];
}

// Searching inlines
//
inline int NvString::find(NCHAR ch) const
{
    const NCHAR str[2] = {ch, '\0'};
    return this->findOneOf(str);
}

inline int NvString::find(wchar_t wch, int nStartPos) const
{
    const wchar_t wsz[2] = {wch, 0};
    return this->findOneOf(wsz, nStartPos);
}

inline int NvString::find(const NvString &s) const
{
    return this->find(s.kwszPtr());
}

inline int NvString::findRev(NCHAR ch) const                    // deprecated
{
    return this->findLast(ch);
}

inline int NvString::findRev(const wchar_t *pwsz) const         // deprecated
{
    return this->findLast(pwsz, -1);
}

inline int NvString::findRev(const NvString &s) const           // deprecated
{
    // see comment above find() about infinite loop and MB strings
        return this->findLast(s.kwszPtr());
}

inline int NvString::findOneOfRev(const wchar_t *pwsz) const    // deprecated
{
    return this->findLastOneOf(pwsz, -1);
}

inline int NvString::findLast(NCHAR ch, int nStartPos) const
{
    const NCHAR str[2] = {ch, '\0'};
    return this->findLastOneOf(str, nStartPos);
}

// Extraction inlines
//
inline NvString NvString::mid(int nStart, int nNumChars) const
{
    return this->substr(nStart, nNumChars);
}

inline NvString NvString::mid(int nStart) const
{
    return this->mid(nStart, -1);
}

inline NvString NvString::substr(int nNumChars) const
{
    return this->substr(0, nNumChars);
}

inline NvString NvString::left(int nNumChars) const
{
    return this->substr(nNumChars);
}

inline NvString NvString::right(int nNumChars) const
{
    return this->substrRev(nNumChars);
}

inline NvString & NvString::trimLeft(wchar_t wch)
{
    const wchar_t wszChars[] = { wch, L'\0' };
    return this->trimLeft(wszChars);
}

inline NvString & NvString::trimLeft()
{
    return this->trimLeft(nullptr);     // trim whitespace
}

inline NvString & NvString::trimRight(wchar_t wch)
{
    const wchar_t wszChars[] = { wch, L'\0' };
    return this->trimRight(wszChars);
}

inline NvString & NvString::trimRight()
{
    return this->trimRight(nullptr);    // trim whitespace
}

inline NvString & NvString::trim(wchar_t wch)
{
    const wchar_t wszChars[] = { wch, L'\0' };
    return this->trim(wszChars);
}

inline NvString & NvString::trim()
{
    return this->trim(nullptr);         // trim whitespace
}

inline NvString & NvString::trim(const wchar_t *pwszChars)
{
    return this->trimRight(pwszChars).trimLeft(pwszChars);
}

// Assignment inlines
//

inline NvString & NvString::assign(wchar_t wch)
{
    const wchar_t wstr[2] = {wch, L'\0'};
    return this->assign(wstr);
}


inline NvString & NvString::operator = (wchar_t wch)
{
    return this->assign(wch);
}

inline NvString & NvString::operator = (const wchar_t *pwsz)
{
    return this->assign(pwsz);
}

inline NvString & NvString::operator = (const NvString & acs)
{
    return this->assign(acs);
}

inline NvString & NvString::operator = (const NvDbHandle & h)
{
    return this->assign(h);
}

// Modifying inlines
//
inline NvString & NvString::operator += (wchar_t wch)
{
    return this->append(wch);
}

inline NvString & NvString::operator += (const wchar_t *pwsz)
{
    return this->append(pwsz);
}

inline NvString & NvString::operator += (const NvString & acs)
{
    return this->append(acs);
}

inline NvString & NvString::append(wchar_t wch)
{
    const wchar_t wstr[2] = {wch, L'\0'};
    return this->append(wstr);
}

// Concatenation inlines
inline NvString NvString::operator + (wchar_t wch) const
{
    return this->concat(wch);
}

inline NvString NvString::operator + (const wchar_t * pwsz) const
{
    return this->concat(pwsz);
}

inline NvString NvString::operator + (const NvString & acs) const
{
    return this->concat(acs);
}

inline NvString NvString::concat(wchar_t wch) const
{
    const wchar_t wstr[2] = {wch, L'\0'};
    return this->concat(wstr);
}

inline NvString NvString::precat(wchar_t ch) const
{
    const wchar_t str[2] = {ch, '\0'};
    return this->precat(str);
}

// Comparison inlines
//

inline const wchar_t * NvString::kwszPtr() const
{
    return this->m_wsz; // this pointer is never null
}

inline int NvString::compare(wchar_t wch) const
{
    const wchar_t wstr[2] = {wch, L'\0'};
    return this->compare(wstr);
}

inline int NvString::compare(const NvString & acs) const
{
    return this->compare(acs.kwszPtr());
}

inline int NvString::compareNoCase(wchar_t wch) const
{
    const wchar_t wstr[2] = {wch, L'\0'};
    return this->compareNoCase(wstr);
}

inline int NvString::compareNoCase(const NvString & acs) const
{
    return this->compareNoCase(acs.kwszPtr());
}

inline int NvString::collate(const NvString & acs) const
{
    return this->collate(acs.kwszPtr());
}

inline int NvString::collateNoCase(const NvString & acs) const
{
    return this->collateNoCase(acs.kwszPtr());
}

inline bool NvString::operator == (wchar_t wch) const
{
    return this->compare(wch) == 0;
}

inline bool NvString::operator == (const wchar_t *pwsz) const
{
    return this->compare(pwsz) == 0;
}

inline bool NvString::operator == (const NvString & acs) const
{
    return this->compare(acs) == 0;
}

inline bool NvString::operator != (wchar_t wch) const
{
    return this->compare(wch) != 0;
}

inline bool NvString::operator != (const wchar_t *pwsz) const
{
    return this->compare(pwsz) != 0;
}

inline bool NvString::operator != (const NvString & acs) const
{
    return this->compare(acs) != 0;
}

inline bool NvString::operator > (wchar_t wch) const
{
    return this->compare(wch) > 0;
}

inline bool NvString::operator > (const wchar_t *pwsz) const
{
    return this->compare(pwsz) > 0;
}

inline bool NvString::operator > (const NvString & acs) const
{
    return this->compare(acs) > 0;
}

inline bool NvString::operator >= (wchar_t wch) const
{
    return this->compare(wch) >= 0;
}

inline bool NvString::operator >= (const wchar_t *pwsz) const
{
    return this->compare(pwsz) >= 0;
}

inline bool NvString::operator >= (const NvString & acs) const
{
    return this->compare(acs) >= 0;
}

inline bool NvString::operator < (wchar_t wch) const
{
    return this->compare(wch) < 0;
}

inline bool NvString::operator < (const wchar_t *pwsz) const
{
    return this->compare(pwsz) < 0;
}

inline bool NvString::operator < (const NvString & acs) const
{
    return this->compare(acs) < 0;
}

inline bool NvString::operator <= (wchar_t wch) const
{
    return this->compare(wch) <= 0;
}

inline bool NvString::operator <= (const wchar_t *pwsz) const
{
    return this->compare(pwsz) <= 0;
}

inline bool NvString::operator <= (const NvString & acs) const
{
    return this->compare(acs) <= 0;
}

// Inline global operators

inline bool operator == (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) == 0;
}

inline bool operator == (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) == 0;
}

inline bool operator != (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) != 0;
}

inline bool operator != (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) != 0;
}

inline bool operator > (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) < 0;
}

inline bool operator > (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) < 0;
}

inline bool operator >= (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) <= 0;
}

inline bool operator >= (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) <= 0;
}

inline bool operator < (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) > 0;
}

inline bool operator < (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) > 0;
}

inline bool operator <= (wchar_t wch, const NvString & acs)
{
    return acs.compare(wch) >= 0;
}

inline bool operator <= (const wchar_t *pwsz, const NvString & acs)
{
    return acs.compare(pwsz) >= 0;
}

// These don't modify the NvString.  They return a copy.
inline NvString operator + (NCHAR ch, const NvString & acs)
{
    return acs.precat(ch);
}

inline NvString operator + (const wchar_t *pwsz, const NvString & acs)
{
    return acs.precat(pwsz);
}

inline bool NvString::equalsNoCase(const NvString& left, const NvString& right)
{
    return left.compareNoCase(right) == 0;
}

// Return a unique identifier (pointer) for the input string, to allow fast compares
// using pointer values instead of strings.
// Input strings are converted to lowercase, then are looked up in and stored in an
// internal map. So "ABC" and "abc" return the same NvUniqueString pointer.
// NvUniqueString pointers are valid for the process's lifetime
//
class NvUniqueString
{
public:
    NVBASE_PORT static const NvUniqueString *Intern(const wchar_t *);
};


// We can do inline operators that deal with CStrings, without getting
// into binary format dependencies.  Don't make these out-of-line
// functions, because then we'll have a dependency between our
// components and CString-using clients.
//
#if defined(ENABLE_NVSTRING_CSTRING_OPERATORS)


inline NvString::NvString(const CStringW &csw) : NvString()
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    *this = pwsz;
}

inline NvString & NvString::operator=(const CStringW &csw)
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->assign(pwsz);
}

inline NvString & NvString::operator+=(const CStringW &csw)
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->append(pwsz);
}

inline int NvString::compare(const CStringW & csw) const
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->compare(pwsz);
}

inline int NvString::compareNoCase(const CStringW & csw) const
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->compareNoCase(pwsz);
}

inline int NvString::match(const CStringW & csw) const
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->match(pwsz);
}

inline int NvString::matchNoCase(const CStringW & csw) const
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return this->matchNoCase(pwsz);
}

inline bool NvString::operator == (const CStringW & csw) const
{
    return this->compare(csw) == 0;
}

inline bool NvString::operator != (const CStringW & csw) const
{
    return this->compare(csw) != 0;
}

inline bool NvString::operator > (const CStringW & csw) const
{
    return this->compare(csw) > 0;
}

inline bool NvString::operator >= (const CStringW & csw) const
{
    return this->compare(csw) >= 0;
}

inline bool NvString::operator < (const CStringW & csw) const
{
    return this->compare(csw) < 0;
}

inline bool NvString::operator <= (const CStringW & csw) const
{
    return this->compare(csw) <= 0;
}

#if defined(_AFX) && !defined(__cplusplus_cli)
// Global CString-related operators
inline bool operator == (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) == 0;
}

inline bool operator != (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) != 0;
}

inline bool operator >  (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) < 0;
}

inline bool operator >= (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) <= 0;
}

inline bool operator <  (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) > 0;
}

inline bool operator <= (const CStringW & csw, const NvString & acs)
{
    return acs.compare(csw) >= 0;
}

#ifndef DISABLE_CSTRING_PLUS_NVSTRING
inline NvString operator + (const CStringW & csw, const NvString & acs)
{
    const wchar_t *pwsz = (const wchar_t *)csw;
    return acs.precat(pwsz);
}
#endif

#endif

#endif // _AFX

#endif // !_Nv_String_h

