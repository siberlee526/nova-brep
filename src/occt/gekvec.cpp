// gekvec.cpp - implementation of NvGeKnotVector, the knot vector of the
// NURBS classes (AutoCAD AcGeKnotVector clone with the Nv prefix).
//
// A knot vector is a growable array of non-decreasing doubles together with
// a knot tolerance. The storage lives in the NvGeDoubleArray member mData,
// which owns its buffer and releases it on destruction, so the knot vector
// itself never manages raw memory. Edit operations that accept user knots
// (append, insert, insertAt) enforce the non-decreasing order within the
// knot tolerance and throw NvException on violation; operations that treat
// the object as a plain array (setLogicalLength, asArrayPtr, the array
// constructor and assignment) copy or resize verbatim, mirroring ARX.

#include <gekvec.h>

#include <NvException.h>
#include <Nova.h>
#include <gedblar.h>
#include <geintrvl.h>

#include <cmath>

//=================================================================================================

// By default the knot tolerance is 1.0e-9 (see gekvec.h).
double NvGeKnotVector::globalKnotTolerance = 1.0e-9;

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (double theEps)
: mTolerance (theEps)
{
}

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (int theSize, int theGrowSize, double theEps)
: mTolerance (theEps)
{
  if (theSize < 0 || theGrowSize <= 0)
  {
    throw NvException ("NvGeKnotVector::NvGeKnotVector(): the size must be non-negative"
                       " and the grow size positive");
  }
  mData.setGrowLength (theGrowSize);
  // The value-initialized overload guarantees zeroed knots; the plain
  // setLogicalLength leaves new cells uninitialized.
  mData.setLogicalLength (theSize, 0.0);
}

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (int theSize, const double theKnots[], double theEps)
: mTolerance (theEps)
{
  if (theSize < 0 || (theSize > 0 && theKnots == nullptr))
  {
    throw NvException ("NvGeKnotVector::NvGeKnotVector(): invalid knot array");
  }
  // Verbatim copy, exactly as ARX does: ordering is the caller's duty here.
  for (int aKnot = 0; aKnot < theSize; ++aKnot)
  {
    mData.append (theKnots[aKnot]);
  }
}

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (int thePlusMult, const NvGeKnotVector& theSrc)
: mTolerance (theSrc.mTolerance)
{
  if (thePlusMult < 0)
  {
    throw NvException ("NvGeKnotVector::NvGeKnotVector(): the multiplicity elevation"
                       " must be non-negative");
  }
  // Every run of equal knots (one distinct knot) gets its multiplicity
  // raised by thePlusMult; the knot order is preserved.
  const int aLength = theSrc.mData.logicalLength();
  int aIndex = 0;
  while (aIndex < aLength)
  {
    int aRunEnd = aIndex + 1;
    while (aRunEnd < aLength
           && theSrc.mData[aRunEnd] - theSrc.mData[aIndex] <= mTolerance)
    {
      ++aRunEnd;
    }
    const int aMultiplicity = (aRunEnd - aIndex) + thePlusMult;
    for (int aCopy = 0; aCopy < aMultiplicity; ++aCopy)
    {
      mData.append (theSrc.mData[aIndex]);
    }
    aIndex = aRunEnd;
  }
}

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (const NvGeKnotVector& theSrc)
: mData (theSrc.mData), // NvGeDoubleArray copy constructor performs the deep copy
  mTolerance (theSrc.mTolerance)
{
}

//=================================================================================================

NvGeKnotVector::NvGeKnotVector (const NvGeDoubleArray& theSrc, double theEps)
: mData (theSrc), // verbatim copy of the doubles, ordering is the caller's duty
  mTolerance (theEps)
{
}

//=================================================================================================

NvGeKnotVector::~NvGeKnotVector ()
{
  // mData releases its own buffer; nothing to free here.
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::operator = (const NvGeKnotVector& theSrc)
{
  if (this != &theSrc)
  {
    mData = theSrc.mData; // deep copy of the doubles
    mTolerance = theSrc.mTolerance;
  }
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::operator = (const NvGeDoubleArray& theSrc)
{
  // Replaces the knots verbatim and keeps the current tolerance; ordering
  // is the caller's duty, as with the matching constructor.
  mData = theSrc;
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeKnotVector::isEqualTo (const NvGeKnotVector& theOther) const
{
  if (mData.logicalLength() != theOther.mData.logicalLength())
  {
    return Adesk::kFalse;
  }
  // Component-wise comparison within the knot tolerance of this vector.
  for (int aKnot = 0; aKnot < mData.logicalLength(); ++aKnot)
  {
    if (std::abs (mData[aKnot] - theOther.mData[aKnot]) > mTolerance)
    {
      return Adesk::kFalse;
    }
  }
  return Adesk::kTrue;
}

//=================================================================================================

double NvGeKnotVector::startParam () const
{
  if (mData.isEmpty())
  {
    throw NvException ("NvGeKnotVector::startParam(): the knot vector is empty");
  }
  return mData.first();
}

//=================================================================================================

double NvGeKnotVector::endParam () const
{
  if (mData.isEmpty())
  {
    throw NvException ("NvGeKnotVector::endParam(): the knot vector is empty");
  }
  return mData.last();
}

//=================================================================================================

int NvGeKnotVector::multiplicityAt (int theIndex) const
{
  if (!isValid (theIndex))
  {
    throw NvException ("NvGeKnotVector::multiplicityAt(): index out of range");
  }
  // The multiplicity of the knot value is the length of the whole run of
  // equal knots, not only the part after theIndex.
  const double aKnot = mData[theIndex];
  int aFirst = theIndex;
  while (aFirst > 0 && aKnot - mData[aFirst - 1] <= mTolerance)
  {
    --aFirst;
  }
  int aLast = theIndex;
  while (aLast + 1 < mData.logicalLength()
         && mData[aLast + 1] - aKnot <= mTolerance)
  {
    ++aLast;
  }
  return aLast - aFirst + 1;
}

//=================================================================================================

int NvGeKnotVector::multiplicityAt (double theParam) const
{
  // Zero when no knot coincides with the parameter within the tolerance.
  int aFirst = -1;
  const int aLength = mData.logicalLength();
  for (int aKnot = 0; aKnot < aLength; ++aKnot)
  {
    if (std::abs (mData[aKnot] - theParam) <= mTolerance)
    {
      aFirst = aKnot;
      break;
    }
  }
  if (aFirst < 0)
  {
    return 0;
  }
  // The knots are non-decreasing, so the run is a contiguous range.
  int aLast = aFirst;
  while (aLast + 1 < aLength && mData[aLast + 1] - theParam <= mTolerance)
  {
    ++aLast;
  }
  return aLast - aFirst + 1;
}

//=================================================================================================

int NvGeKnotVector::numIntervals () const
{
  // Number of distinct knots minus one; an empty or single-knot vector has
  // no interval.
  const int aLength = mData.logicalLength();
  if (aLength < 2)
  {
    return 0;
  }
  int aDistinct = 1;
  for (int aKnot = 1; aKnot < aLength; ++aKnot)
  {
    if (mData[aKnot] - mData[aKnot - 1] > mTolerance)
    {
      ++aDistinct;
    }
  }
  return aDistinct - 1;
}

//=================================================================================================

int NvGeKnotVector::getInterval (int theOrd, double thePar, NvGeInterval& theInterval) const
{
  // Knot span search for a curve of the given order (degree + 1), following
  // the standard B-spline convention: valid spans run from index theOrd - 1
  // to length - theOrd - 1 and cover the parameter domain
  // [knot[theOrd - 1], knot[length - theOrd]]. Returns the index of the span
  // and reports its bounds through theInterval.
  const int aLength = mData.logicalLength();
  const int aFirstSpan = theOrd - 1;
  const int aLastSpan = aLength - theOrd - 1;
  if (theOrd < 1 || aLastSpan < aFirstSpan)
  {
    throw NvException ("NvGeKnotVector::getInterval(): the order is invalid"
                       " for this knot vector");
  }
  if (thePar < mData[aFirstSpan] - mTolerance
      || thePar > mData[aLastSpan + 1] + mTolerance)
  {
    throw NvException ("NvGeKnotVector::getInterval(): the parameter is outside"
                       " the knot domain of the given order");
  }

  int aSpan = aLastSpan;
  if (thePar < mData[aLastSpan + 1] - mTolerance)
  {
    // Bisection for the span with knot[aSpan] <= par < knot[aSpan + 1].
    int aLow = aFirstSpan;
    int aHigh = aLastSpan;
    int aMid = (aLow + aHigh) / 2;
    while (thePar < mData[aMid] - mTolerance
           || thePar >= mData[aMid + 1] + mTolerance)
    {
      if (thePar < mData[aMid])
      {
        aHigh = aMid;
      }
      else
      {
        aLow = aMid;
      }
      aMid = (aLow + aHigh) / 2;
    }
    aSpan = aMid;
  }
  theInterval.set (mData[aSpan], mData[aSpan + 1]);
  return aSpan;
}

//=================================================================================================

void NvGeKnotVector::getDistinctKnots (NvGeDoubleArray& theKnots) const
{
  theKnots.setLogicalLength (0);
  const int aLength = mData.logicalLength();
  for (int aKnot = 0; aKnot < aLength; ++aKnot)
  {
    // A knot starts a new distinct value when it leaves the tolerance of
    // its predecessor (the knots are non-decreasing).
    if (aKnot == 0 || mData[aKnot] - mData[aKnot - 1] > mTolerance)
    {
      theKnots.append (mData[aKnot]);
    }
  }
}

//=================================================================================================

Adesk::Boolean NvGeKnotVector::contains (double theParam) const
{
  // Range containment, in the sense of NvGeInterval::contains: the parameter
  // lies between the first and the last knot (inclusive within tolerance).
  if (mData.isEmpty())
  {
    return Adesk::kFalse;
  }
  return theParam >= mData.first() - mTolerance
      && theParam <= mData.last() + mTolerance;
}

//=================================================================================================

Adesk::Boolean NvGeKnotVector::isOn (double theKnot) const
{
  // Knot membership: the value coincides with one of the knots within the
  // knot tolerance.
  const int aLength = mData.logicalLength();
  for (int aIndex = 0; aIndex < aLength; ++aIndex)
  {
    if (std::abs (mData[aIndex] - theKnot) <= mTolerance)
    {
      return Adesk::kTrue;
    }
    if (mData[aIndex] > theKnot + mTolerance)
    {
      break; // knots are non-decreasing, no later knot can coincide
    }
  }
  return Adesk::kFalse;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::reverse ()
{
  const int aLength = mData.logicalLength();
  if (aLength == 0)
  {
    return *this;
  }
  // ARX semantics: the sequence is reversed and every knot value v is
  // re-mapped to (a + b) - v, where a and b are the old first and last
  // knots. The re-mapped sequence stays non-decreasing.
  const double aSum = mData.first() + mData.last();
  const NvGeDoubleArray anOld (mData);
  for (int aKnot = 0; aKnot < aLength; ++aKnot)
  {
    mData[aKnot] = aSum - anOld[aLength - 1 - aKnot];
  }
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::removeAt (int theIndex)
{
  if (!isValid (theIndex))
  {
    throw NvException ("NvGeKnotVector::removeAt(): index out of range");
  }
  mData.removeAt (theIndex);
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::removeSubVector (int theStartIndex, int theEndIndex)
{
  if (!isValid (theStartIndex) || !isValid (theEndIndex) || theStartIndex > theEndIndex)
  {
    throw NvException ("NvGeKnotVector::removeSubVector(): index out of range");
  }
  // Removes the knots from theStartIndex through theEndIndex, inclusive.
  mData.removeSubArray (theStartIndex, theEndIndex);
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::insertAt (int theIndx, double theU, int theMultiplicity)
{
  if (theIndx < 0 || theIndx > mData.logicalLength() || theMultiplicity < 1)
  {
    throw NvException ("NvGeKnotVector::insertAt(): index out of range"
                       " or multiplicity below one");
  }
  // The neighbors around the insertion point must keep the sequence
  // non-decreasing within the knot tolerance.
  const bool aAfterPrev = theIndx == 0 || mData[theIndx - 1] - theU <= mTolerance;
  const int aLength = mData.logicalLength();
  const bool aBeforeNext = theIndx == aLength || theU - mData[theIndx] <= mTolerance;
  if (!aAfterPrev || !aBeforeNext)
  {
    throw NvException ("NvGeKnotVector::insertAt(): knots must be non-decreasing");
  }
  for (int aCopy = 0; aCopy < theMultiplicity; ++aCopy)
  {
    mData.insertAt (theIndx + aCopy, theU);
  }
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::insert (double theU)
{
  // Ordered insertion: place the knot after every knot that does not exceed
  // it (within tolerance), so the sequence stays non-decreasing by
  // construction.
  int aPosition = 0;
  const int aLength = mData.logicalLength();
  while (aPosition < aLength && mData[aPosition] - theU <= mTolerance)
  {
    ++aPosition;
  }
  mData.insertAt (aPosition, theU);
  return *this;
}

//=================================================================================================

int NvGeKnotVector::append (double theVal)
{
  if (!mData.isEmpty() && theVal < mData.last() - mTolerance)
  {
    throw NvException ("NvGeKnotVector::append(): knots must be non-decreasing");
  }
  return mData.append (theVal); // index of the new last knot
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::append (NvGeKnotVector& theTail, double theKnotRatio)
{
  const int aTailLength = theTail.mData.logicalLength();
  if (aTailLength == 0)
  {
    return *this;
  }
  if (theKnotRatio < 0.0 || theKnotRatio > 1.0)
  {
    throw NvException ("NvGeKnotVector::append(): the knot ratio must be within [0, 1]");
  }
  if (mData.isEmpty())
  {
    // Joining to an empty vector just adopts the tail knots.
    mData = theTail.mData;
    return *this;
  }

  // The tail is re-parameterized so that its leading part coincides with the
  // closing knots of this vector. With knotRatio zero the junction knot is
  // simply duplicated (a C0 junction), exactly as when two curves are joined
  // into one; a positive knotRatio declares int (ratio * tail length) tail
  // knots common with the closing knots of this vector, and they are not
  // appended again.
  const int aLength = mData.logicalLength();
  const int aCommon = static_cast<int> (theKnotRatio * aTailLength);
  if (aCommon > aLength)
  {
    throw NvException ("NvGeKnotVector::append(): the knot ratio requests"
                       " more common knots than this vector provides");
  }
  const double aJunction = aCommon > 0 ? mData[aLength - aCommon] : mData.last();
  const double aShift = aJunction - theTail.mData.first();
  // The common tail knots must already coincide with the closing knots of
  // this vector once shifted; otherwise the append would break the order.
  for (int aKnot = 0; aKnot < aCommon; ++aKnot)
  {
    if (std::abs (theTail.mData[aKnot] + aShift - mData[aLength - aCommon + aKnot])
        > mTolerance)
    {
      throw NvException ("NvGeKnotVector::append(): the common knots at the junction"
                         " do not coincide");
    }
  }
  for (int aKnot = aCommon; aKnot < aTailLength; ++aKnot)
  {
    mData.append (theTail.mData[aKnot] + aShift);
  }
  return *this;
}

//=================================================================================================

int NvGeKnotVector::split (double thePar, NvGeKnotVector* theKnot1, int theMultLast,
                           NvGeKnotVector* theKnot2, int theMultFirst) const
{
  // Splits at thePar: theKnot1 receives the knots up to thePar with its
  // closing knot raised to theMultLast copies; theKnot2 receives theMultFirst
  // copies of thePar followed by the remaining knots. Returns the number of
  // knots placed into theKnot1 before the multiplicity elevation.
  const int aLength = mData.logicalLength();
  if (aLength == 0
      || thePar < mData.first() - mTolerance || thePar > mData.last() + mTolerance
      || theMultLast < 1 || theMultFirst < 1)
  {
    throw NvException ("NvGeKnotVector::split(): the split parameter is outside"
                       " the knot range or a multiplicity is below one");
  }
  // Index of the last knot not exceeding thePar; the first knot always
  // qualifies because thePar is within range.
  int aSplit = 0;
  while (aSplit < aLength && mData[aSplit] - thePar <= mTolerance)
  {
    ++aSplit;
  }
  --aSplit;

  if (theKnot1 != nullptr)
  {
    theKnot1->mData.setLogicalLength (0);
    theKnot1->mTolerance = mTolerance;
    for (int aKnot = 0; aKnot <= aSplit; ++aKnot)
    {
      theKnot1->mData.append (mData[aKnot]);
    }
    // Adjust the run of knots equal to the closing knot (it coincides with
    // thePar within tolerance) to exactly theMultLast copies.
    const int aLast = theKnot1->mData.logicalLength() - 1;
    int aRunStart = aLast;
    while (aRunStart > 0
           && theKnot1->mData[aLast] - theKnot1->mData[aRunStart - 1] <= mTolerance)
    {
      --aRunStart;
    }
    const int aCurrent = aLast - aRunStart + 1;
    if (theMultLast < aCurrent)
    {
      theKnot1->mData.setLogicalLength (aRunStart + theMultLast);
    }
    else
    {
      for (int aCopy = aCurrent; aCopy < theMultLast; ++aCopy)
      {
        theKnot1->mData.append (theKnot1->mData[aLast]);
      }
    }
  }
  if (theKnot2 != nullptr)
  {
    theKnot2->mData.setLogicalLength (0);
    theKnot2->mTolerance = mTolerance;
    for (int aCopy = 0; aCopy < theMultFirst; ++aCopy)
    {
      theKnot2->mData.append (thePar);
    }
    for (int aKnot = aSplit + 1; aKnot < aLength; ++aKnot)
    {
      theKnot2->mData.append (mData[aKnot]);
    }
  }
  return aSplit + 1;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::setRange (double theLower, double theUpper)
{
  if (theLower > theUpper)
  {
    throw NvException ("NvGeKnotVector::setRange(): the lower bound must not"
                       " exceed the upper bound");
  }
  const int aLength = mData.logicalLength();
  if (aLength == 0)
  {
    return *this;
  }
  const double aStart = mData.first();
  const double anEnd = mData.last();
  if (anEnd - aStart <= mTolerance)
  {
    // Degenerate source range: every knot maps to the new lower bound.
    mData.setAll (theLower);
    return *this;
  }
  // Linear reparameterization of [aStart, anEnd] onto [theLower, theUpper];
  // relative knot spacing is preserved.
  const double aScale = (theUpper - theLower) / (anEnd - aStart);
  for (int aKnot = 0; aKnot < aLength; ++aKnot)
  {
    mData[aKnot] = theLower + (mData[aKnot] - aStart) * aScale;
  }
  return *this;
}

//=================================================================================================

int NvGeKnotVector::length () const
{
  return mData.length();
}

//=================================================================================================

Adesk::Boolean NvGeKnotVector::isEmpty () const
{
  return mData.isEmpty();
}

//=================================================================================================

int NvGeKnotVector::logicalLength () const
{
  return mData.logicalLength();
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::setLogicalLength (int theLength)
{
  if (theLength < 0)
  {
    throw NvException ("NvGeKnotVector::setLogicalLength(): the length must not"
                       " be negative");
  }
  const int anOldLength = mData.logicalLength();
  mData.setLogicalLength (theLength);
  // Newly exposed knots are zeroed; the underlying array leaves fresh cells
  // uninitialized.
  for (int aKnot = anOldLength; aKnot < theLength; ++aKnot)
  {
    mData[aKnot] = 0.0;
  }
  return *this;
}

//=================================================================================================

int NvGeKnotVector::physicalLength () const
{
  return mData.physicalLength();
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::setPhysicalLength (int theLength)
{
  if (theLength < 0)
  {
    throw NvException ("NvGeKnotVector::setPhysicalLength(): the length must not"
                       " be negative");
  }
  mData.setPhysicalLength (theLength);
  return *this;
}

//=================================================================================================

int NvGeKnotVector::growLength () const
{
  return mData.growLength();
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::setGrowLength (int theGrowLength)
{
  if (theGrowLength <= 0)
  {
    throw NvException ("NvGeKnotVector::setGrowLength(): the grow length must"
                       " be positive");
  }
  mData.setGrowLength (theGrowLength);
  return *this;
}

//=================================================================================================

NvGeKnotVector& NvGeKnotVector::set (int theSize, const double theKnots[], double theEps)
{
  if (theSize < 0 || (theSize > 0 && theKnots == nullptr))
  {
    throw NvException ("NvGeKnotVector::set(): invalid knot array");
  }
  mTolerance = theEps;
  mData.setLogicalLength (0);
  for (int aKnot = 0; aKnot < theSize; ++aKnot)
  {
    mData.append (theKnots[aKnot]);
  }
  return *this;
}
