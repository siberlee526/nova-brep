// NvStringTest.cpp - unit tests for the NvString wide-character string class.

#include <NvString.h>
#include <NvException.h>

#include <gtest/gtest.h>

#include <cwchar>

namespace
{

//! Compares a string against a wide literal (EXPECT_STREQ friendly helper).
bool SameAs(const NvString& theString, const wchar_t* theExpected)
{
  return std::wcscmp(theString, theExpected) == 0;
}

} // namespace

//=================================================================================================

TEST(NvStringTest, DefaultConstructor_IsEmpty)
{
  const NvString aString;
  EXPECT_TRUE (aString.isEmpty());
  EXPECT_EQ   (aString.length(), 0u);
  EXPECT_NE   (aString.kwszPtr(), nullptr);
  EXPECT_EQ   (aString.kwszPtr()[0], L'\0');
}

TEST(NvStringTest, WideCharAndPointerConstructors_CopyContent)
{
  const NvString aChar(L'x');
  EXPECT_EQ (aChar.length(), 1u);
  EXPECT_EQ (aChar.getAt(0), L'x');

  const NvString aText(L"hello");
  EXPECT_EQ (aText.length(), 5u);
  EXPECT_STREQ (aText.kwszPtr(), L"hello");

  const NvString aPrefix(L"hello", 3);
  EXPECT_STREQ (aPrefix, L"hel");

  const NvString aNullText(static_cast<const wchar_t*>(nullptr));
  EXPECT_TRUE (aNullText.isEmpty());

  const NvString aRepeat(L'-', 3);
  EXPECT_STREQ (aRepeat, L"---");
  const NvString aZeroRepeat(L'-', 0);
  EXPECT_TRUE (aZeroRepeat.isEmpty());
}

TEST(NvStringTest, Utf8Constructors_DecodeNarrowInput)
{
  const NvString aText("hello", NvString::Utf8);
  EXPECT_STREQ (aText, L"hello");

  const NvString aPrefix("abcdef", NvString::Utf8, 3);
  EXPECT_STREQ (aPrefix, L"abc");

  // Non-ascii sequence (U+00E9 as utf-8 bytes 0xC3 0xA9) survives the decode.
  const NvString anAccent("h\xC3\xA9" "llo", NvString::Utf8);
  EXPECT_EQ (anAccent.length(), 5u);
  EXPECT_EQ (std::wcscmp(anAccent, L"h\x00E9" "llo"), 0);
}

TEST(NvStringTest, NumericFormatConstructors_RenderValues)
{
  EXPECT_STREQ (NvString(NvString::kSigned, 42u),    L"42");
  EXPECT_STREQ (NvString(NvString::kSigned, static_cast<unsigned>(-42)), L"-42");
  EXPECT_STREQ (NvString(NvString::kUnSigned, 42u),  L"42");
  EXPECT_STREQ (NvString(NvString::kHex, 0xA2Fu),    L"a2f");
}

TEST(NvStringTest, CopyConstructor_IsIndependentOfOriginal)
{
  NvString aOriginal(L"shared");
  const NvString aCopy(aOriginal);

  aOriginal += L"-changed";
  EXPECT_STREQ (aCopy, L"shared");
  EXPECT_STREQ (aOriginal, L"shared-changed");

  NvString aMutatedCopy(aCopy);
  aMutatedCopy.makeUpper();
  EXPECT_STREQ (aCopy, L"shared");
  EXPECT_STREQ (aMutatedCopy, L"SHARED");
}

TEST(NvStringTest, MoveConstructor_TakesContentAndLeavesEmpty)
{
  NvString aSource(L"cargo");
  const NvString aMoved(std::move(aSource));

  EXPECT_STREQ (aMoved, L"cargo");
  EXPECT_TRUE  (aSource.isEmpty());
  EXPECT_EQ    (aSource.length(), 0u);
}

TEST(NvStringTest, MoveAssignment_TakesContentAndLeavesEmpty)
{
  NvString aTarget(L"old");
  NvString aSource(L"new");
  aTarget = std::move(aSource);

  EXPECT_STREQ (aTarget, L"new");
  EXPECT_TRUE  (aSource.isEmpty());
}

TEST(NvStringTest, SelfAssignAndSelfAppend_KeepContent)
{
  NvString aString(L"ab");
  aString = aString;
  EXPECT_STREQ (aString, L"ab");

  aString += aString;
  EXPECT_STREQ (aString, L"abab");
}

//=================================================================================================

TEST(NvStringTest, LengthCapacityReserve_TrackBufferState)
{
  NvString aString(L"abc");
  EXPECT_EQ (aString.length(), 3u);
  EXPECT_EQ (aString.tcharLength(), 3u);

  const unsigned aOldCapacity = aString.capacity();
  EXPECT_TRUE (aString.reserve(100)); // 100 chars including terminator
  EXPECT_GE (aString.capacity(), 99u);
  EXPECT_STREQ (aString, L"abc");

  // Shrinking below the live length cannot truncate the content.
  EXPECT_TRUE (aString.reserve(1)); // reallocates down to the live length
  EXPECT_EQ (aString.length(), 3u);
  EXPECT_STREQ (aString, L"abc");
  (void)aOldCapacity;
}

TEST(NvStringTest, SetEmpty_ClearsContent)
{
  NvString aString(L"data");
  aString.setEmpty();
  EXPECT_TRUE (aString.isEmpty());

  aString = L"more";
  aString.setEmpty();
  EXPECT_TRUE (aString.isEmpty());
}

TEST(NvStringTest, IsAsciiAnd7Bit_ClassifyContent)
{
  EXPECT_TRUE  (NvString(L"abc 123").isAscii());
  EXPECT_TRUE  (NvString(L"abc 123").is7Bit());
  EXPECT_FALSE (NvString(L"a\tb").isAscii());   // 0x09 counts as a control char
  EXPECT_TRUE  (NvString(L"a\tb").is7Bit());
  EXPECT_FALSE (NvString(L"caf\x00E9").isAscii());
  EXPECT_FALSE (NvString(L"caf\x00E9").is7Bit());
  EXPECT_TRUE  (NvString().isAscii());
  EXPECT_TRUE  (NvString().is7Bit());
}

//=================================================================================================

TEST(NvStringTest, AppendAndOperators_AddContent)
{
  NvString aString(L"one");
  aString += L"-two";
  aString += L'!';
  EXPECT_STREQ (aString, L"one-two!");

  NvString anOther(L"start");
  anOther.append(NvString(L"-end"));
  anOther.append(L"!");
  anOther.append("u8", NvString::Utf8);
  EXPECT_STREQ (anOther, L"start-end!u8");
}

TEST(NvStringTest, ConcatAndOperators_ReturnNewStrings)
{
  const NvString aString(L"ab");
  EXPECT_STREQ (aString + L"cd", L"abcd");
  EXPECT_STREQ (aString + L'!',  L"ab!");
  EXPECT_STREQ (aString + NvString(L"cd"), L"abcd");
  EXPECT_STREQ (aString.concat(L"cd"), L"abcd");
  EXPECT_STREQ (aString.concat("ef", NvString::Utf8), L"abef");
  EXPECT_STREQ (aString.concat(NvString(L"cd")), L"abcd");
  EXPECT_STREQ (L"cd" + aString, L"cdab");
  EXPECT_STREQ (L'!' + aString,  L"!ab");
  EXPECT_STREQ (aString.precat(L"cd"), L"cdab");
  EXPECT_STREQ (aString.precat("ef", NvString::Utf8), L"efab");
  // The source string is untouched by all of the above.
  EXPECT_STREQ (aString, L"ab");
}

TEST(NvStringTest, FormatAndAppendFormat_ApplyPrintfRules)
{
  NvString aString;
  aString.format(L"%s=%d", L"answer", 42);
  EXPECT_STREQ (aString, L"answer=42");

  aString.appendFormat(L"-%u", 7u);
  EXPECT_STREQ (aString, L"answer=42-7");
}

//=================================================================================================

TEST(NvStringTest, Find_LocatesCharactersAndSubstrings)
{
  const NvString aString(L"hello");

  EXPECT_EQ (aString.find(L'l'), 2);
  EXPECT_EQ (aString.find(L'l', 3), 3);
  EXPECT_EQ (aString.find(L'z'), -1);
  EXPECT_EQ (aString.find(L"llo"), 2);
  EXPECT_EQ (aString.find(L"lz"), -1);
  EXPECT_EQ (aString.find(NvString(L"llo")), 2);
  EXPECT_EQ (aString.find(L"lo", 4), -1);
  EXPECT_EQ (aString.findRev(L'l'), 3);
  EXPECT_EQ (aString.findLast(L'l'), 3);
  EXPECT_EQ (aString.findLast(L'l', 2), 2);
  EXPECT_EQ (aString.findLast(L"ell"), 1);
  EXPECT_EQ (aString.findLast(L"hello", 3), 0); // match start 0 does not exceed endPos 3
}

TEST(NvStringTest, FindOneOfFamily_ScanCharGroups)
{
  const NvString aString(L"a-b c");

  EXPECT_EQ (aString.findOneOf(L"- "), 1);
  EXPECT_EQ (aString.findOneOf(L"- ", 2), 3);
  EXPECT_EQ (aString.findNoneOf(L"ab-"), 3);
  EXPECT_EQ (aString.findOneOf(nullptr), 3);      // null group searches whitespace
  EXPECT_EQ (aString.findNoneOf(L"ab-"), 3);
  EXPECT_EQ (aString.findLastOneOf(L"-c"), 4);
  EXPECT_EQ (aString.findLastNoneOf(L" c-"), 2);
  EXPECT_EQ (aString.findOneOfRev(L" -"), 3);
}

TEST(NvStringTest, MidSubstrLeftRight_ExtractRanges)
{
  const NvString aString(L"abcdef");

  EXPECT_STREQ (aString.mid(2), L"cdef");
  EXPECT_STREQ (aString.mid(2, 2), L"cd");
  EXPECT_STREQ (aString.substr(1, 3), L"bcd");
  EXPECT_STREQ (aString.substr(3), L"abc"); // substr(n) takes the first n chars
  EXPECT_STREQ (aString.substr(3, -1), L"def");
  EXPECT_STREQ (aString.substrRev(2), L"ef");
  EXPECT_STREQ (aString.left(2), L"ab");
  EXPECT_STREQ (aString.right(2), L"ef");

  // ARX-style clamping on out-of-range requests.
  EXPECT_STREQ (aString.left(99), L"abcdef");
  EXPECT_STREQ (aString.right(99), L"abcdef");
  EXPECT_TRUE  (aString.mid(99).isEmpty());
  EXPECT_STREQ (aString.mid(-1, 2), L"ab"); // negative start clamps to 0
  EXPECT_TRUE  (aString.substrRev(0).isEmpty());
  EXPECT_TRUE  (aString.substrRev(-2).isEmpty());
}

//=================================================================================================

TEST(NvStringTest, MakeUpperMakeLowerMakeReverse_TransformInPlace)
{
  NvString aString(L"AbC");
  aString.makeUpper();
  EXPECT_STREQ (aString, L"ABC");

  aString.makeLower();
  EXPECT_STREQ (aString, L"abc");

  aString.makeReverse();
  EXPECT_STREQ (aString, L"cba");
}

TEST(NvStringTest, Trim_RemovesFromEndsOnly)
{
  NvString aWhitespace(L"  hi  ");
  aWhitespace.trim();
  EXPECT_STREQ (aWhitespace, L"hi");

  NvString aChar(L"xxhixx");
  aChar.trim(L'x');
  EXPECT_STREQ (aChar, L"hi");

  NvString aGroup(L"..hi..");
  aGroup.trimLeft(L".");
  EXPECT_STREQ (aGroup, L"hi..");
  aGroup.trimRight(L".");
  EXPECT_STREQ (aGroup, L"hi");

  NvString anInner(L" x y ");
  anInner.trim();
  EXPECT_STREQ (anInner, L"x y"); // inner content is preserved
}

TEST(NvStringTest, RemoveAndReplace_EditContent)
{
  NvString aString(L"banana");
  EXPECT_EQ (aString.remove(L'a'), 3);
  EXPECT_STREQ (aString, L"bnn");

  NvString aWs(L"a b\tc");
  EXPECT_EQ (aWs.remove(), 2); // zero char designates whitespace
  EXPECT_STREQ (aWs, L"abc");

  NvString aCharReplace(L"foo");
  EXPECT_EQ (aCharReplace.replace(L'o', L'0'), 2);
  EXPECT_STREQ (aCharReplace, L"f00");

  NvString aSubReplace(L"a-b-a");
  EXPECT_EQ (aSubReplace.replace(L"-", L"+"), 2);
  EXPECT_STREQ (aSubReplace, L"a+b+a");

  NvString aShrink(L"aaa");
  EXPECT_EQ (aShrink.replace(L"aa", L"x"), 1);
  EXPECT_STREQ (aShrink, L"xa");

  EXPECT_EQ (aShrink.replace(L"zz", L"q"), 0);
}

TEST(NvStringTest, InsertDeleteSetAt_EditPositions)
{
  NvString aString(L"ac");
  aString.insert(1, L'b');
  EXPECT_STREQ (aString, L"abc");
  aString.insert(0, L'-');
  EXPECT_STREQ (aString, L"-abc");
  aString.insert(99, L'+'); // clamped to the end
  EXPECT_STREQ (aString, L"-abc+");

  EXPECT_EQ (aString.deleteAtIndex(0, 1), 4); // "-abc+" -> "abc+"
  EXPECT_STREQ (aString, L"abc+");
  EXPECT_EQ (aString.deleteAtIndex(3, 99), 3); // clamped -> "abc"
  EXPECT_STREQ (aString, L"abc");

  aString.setAt(0, L'X');
  EXPECT_STREQ (aString, L"Xbc");
  EXPECT_THROW (aString.setAt(3, L'x'), NvException);
  EXPECT_THROW (aString.setAt(-1, L'x'), NvException);
  EXPECT_EQ (aString.getAt(0), L'X');
}

TEST(NvStringTest, Tokenize_SplitsOnDelimiters)
{
  const NvString aString(L"a,b,c");
  int aStart = 0;
  EXPECT_STREQ (aString.tokenize(L",", aStart), L"a");
  EXPECT_EQ (aStart, 2);
  EXPECT_STREQ (aString.tokenize(L",", aStart), L"b");
  EXPECT_EQ (aStart, 4);
  EXPECT_STREQ (aString.tokenize(L",", aStart), L"c");
  EXPECT_EQ (aStart, -1);
  EXPECT_TRUE (aString.tokenize(L",", aStart).isEmpty());
  EXPECT_EQ (aStart, -1);
}

TEST(NvStringTest, SpanExcluding_StopsAtFirstDesignatedChar)
{
  EXPECT_STREQ (NvString(L"123abc").spanExcluding(L"abc"), L"123");
  EXPECT_STREQ (NvString(L"abc").spanExcluding(L""), L"abc");
}

//=================================================================================================

TEST(NvStringTest, CompareFamily_OrdersAndEquality)
{
  const NvString aString(L"bcd");

  EXPECT_EQ (aString.compare(NvString(L"bcd")), 0);
  EXPECT_EQ (aString.compare(NvString(L"abc")), 1);
  EXPECT_EQ (aString.compare(NvString(L"cbd")), -1);
  EXPECT_EQ (aString.compare(L"bcd"), 0);
  EXPECT_EQ (aString.compare("abc", NvString::Utf8), 1);
  EXPECT_EQ (aString.compare(L'b'), 1); // longer string outranks the equal prefix

  EXPECT_EQ (aString.collate(L"bcd"), 0);
  EXPECT_EQ (aString.collateNoCase(L"BCD"), 0);

  EXPECT_TRUE  (aString == L"bcd");
  EXPECT_TRUE  (aString != L"bce");
  EXPECT_TRUE  (aString > L"abc");
  EXPECT_TRUE  (aString >= L"bcd");
  EXPECT_TRUE  (aString < L"cde");
  EXPECT_TRUE  (aString <= L"bcd");
  EXPECT_TRUE  (L"abc" < aString);
  EXPECT_TRUE  (L"bcd" == aString);
  EXPECT_TRUE  (NvString(L"a") < NvString(L"b"));

  EXPECT_TRUE  (NvString::equalsNoCase(NvString(L"ABC"), NvString(L"abc")));
  EXPECT_FALSE (NvString::equalsNoCase(NvString(L"ABC"), NvString(L"abd")));
}

TEST(NvStringTest, CompareNoCase_IgnoresCase)
{
  const NvString aString(L"HeLLo");
  EXPECT_EQ (aString.compareNoCase(NvString(L"hello")), 0);
  EXPECT_EQ (aString.compareNoCase(L"HELLO"), 0);
  EXPECT_EQ (aString.compareNoCase(L"hellp"), -1);
  EXPECT_EQ (aString.compareNoCase("helln", NvString::Utf8), 1);
  EXPECT_EQ (aString.compareNoCase(L'i'), -1);
  EXPECT_TRUE  (aString == NvString(L"hello") == false); // == stays case-sensitive
}

TEST(NvStringTest, Match_CountsCommonPrefix)
{
  const NvString aString(L"abcd");
  EXPECT_EQ (aString.match(NvString(L"abxyz")), 2);
  EXPECT_EQ (aString.match(L"abc"), 3);
  EXPECT_EQ (aString.match("abq", NvString::Utf8), 2);
  EXPECT_EQ (aString.matchNoCase(NvString(L"ABxy")), 2);
  EXPECT_EQ (aString.matchNoCase(L"ABCD"), 4);
  EXPECT_EQ (aString.matchNoCase("abQ", NvString::Utf8), 2);
}

//=================================================================================================

TEST(NvStringTest, NumericParsers_DecodeValues)
{
  EXPECT_EQ (NvString(L"42").asDeci(), 42);
  EXPECT_EQ (NvString(L"-17").asDeci(), -17);
  EXPECT_EQ (NvString(L" 42").asDeci(), 42); // leading whitespace is skipped
  EXPECT_EQ (NvString(L"0x2A").asHex(), 42);
  EXPECT_EQ (NvString(L"2a").asHex(), 42);
  EXPECT_EQ (NvString(L"42").asUDeci(), 42u);
  EXPECT_EQ (NvString(L"2A").asUHex(), 42u);
  EXPECT_EQ (NvString(L"9223372036854775807").asDeci64(), static_cast<int64_t>(0x7FFFFFFFFFFFFFFFLL));
  EXPECT_EQ (NvString(L"-9223372036854775808").asDeci64(), static_cast<int64_t>(0x8000000000000000LL));
  EXPECT_EQ (NvString(L"ffffffff").asUHex64(), 0xFFFFFFFFULL);
  EXPECT_EQ (NvString(L"12345678901234567890").asUDeci64(), 12345678901234567890ULL);
}

TEST(NvStringTest, NumericParsers_ReportErrorsPerFlags)
{
  // kParseZero (default value, without the assert bit).
  EXPECT_EQ (NvString(L"12x3").asDeci(NvString::kParseZero), 0);
  EXPECT_EQ (NvString(L"").asDeci(NvString::kParseZero), 0);

  // kParseMinus1: -1 for signed, max value for unsigned.
  EXPECT_EQ (NvString(L"zz").asDeci(NvString::kParseMinus1), -1);
  EXPECT_EQ (NvString(L"-5").asUDeci(NvString::kParseMinus1), 0xFFFFFFFFu);
  EXPECT_EQ (NvString(L"-5").asUDeci(NvString::kParseZero), 0u); // sign is invalid for unsigned

  // kParseNoEmpty treats blank input as an error.
  EXPECT_EQ (NvString(L"").asDeci(NvString::kParseNoEmpty), 0);

  // Overflow beyond the target type is an error.
  EXPECT_EQ (NvString(L"3000000000").asDeci(NvString::kParseZero), 0);
  EXPECT_EQ (NvString(L"4294967296").asUDeci(NvString::kParseZero), 0u);

  // kParseExcept throws the documented int exception.
  EXPECT_THROW (NvString(L"x").asDeci(NvString::kParseExcept), int);
}

TEST(NvStringTest, Utf8Ptr_RoundTripsContent)
{
  const NvString aString("h\xC3\xA9" "llo", NvString::Utf8);
  const char* anUtf8 = aString.utf8Ptr();
  EXPECT_STREQ (anUtf8, "h\xC3\xA9" "llo");
}

//=================================================================================================

TEST(NvStringTest, EmptyString_EdgesAreSafe)
{
  const NvString aString;

  EXPECT_EQ (aString.find(L'a'), -1);
  EXPECT_EQ (aString.find(L""), 0);
  EXPECT_EQ (aString.findOneOf(L"a"), -1);
  EXPECT_EQ (aString.findLast(L'a'), -1);
  EXPECT_TRUE  (aString.mid(0).isEmpty());
  EXPECT_TRUE  (aString.left(3).isEmpty());
  EXPECT_TRUE  (aString.right(3).isEmpty());
  EXPECT_TRUE  (aString.substrRev(2).isEmpty());
  EXPECT_TRUE  (NvString().trim().isEmpty());
  EXPECT_EQ   (aString.compare(L""), 0);
  EXPECT_EQ   (aString.match(L"abc"), 0);
  EXPECT_EQ   (aString.asDeci(NvString::kParseZero), 0);
  EXPECT_TRUE  (aString == L"");
  EXPECT_STREQ (aString + L"x", L"x");
  EXPECT_STREQ (L"x" + aString, L"x");
}

TEST(NvStringTest, NullArguments_AreTreatedAsEmpty)
{
  NvString aString(L"data");
  aString = static_cast<const wchar_t*>(nullptr);
  EXPECT_TRUE (aString.isEmpty());

  NvString anAppend(L"keep");
  anAppend.append(static_cast<const wchar_t*>(nullptr));
  EXPECT_STREQ (anAppend, L"keep");
}

TEST(NvStringTest, GetBufferReleaseBuffer_AllowDirectWrites)
{
  NvString aString(L"abc");
  NCHAR* aBuffer = aString.getBuffer(16);
  EXPECT_NE (aBuffer, nullptr);
  const wchar_t* aSource = L"rewritten";
  for (unsigned anIndex = 0; ; ++anIndex)
  {
    aBuffer[anIndex] = aSource[anIndex];
    if (aSource[anIndex] == L'\0')
    {
      break;
    }
  }
  EXPECT_TRUE (aString.releaseBuffer());
  EXPECT_STREQ (aString, L"rewritten");
  EXPECT_EQ (aString.length(), 9u);

  // A second release without a getBuffer() loan reports failure.
  EXPECT_FALSE (aString.releaseBuffer());
  EXPECT_EQ (aString.getBuffer(-1), nullptr);
}
