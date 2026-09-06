// geintrvl.cpp - implementation of NvGeInterval on NvGeImpInterval.
//
// An unbounded direction is encoded by an infinite double bound; a finite
// interval is [LowerBound, UpperBound] inclusive within its tolerance.

#include <geintrvl.h>

#include <Nova.h>

#include <geimpdata.h>

#include <cmath>
#include <limits>

namespace
{

const double THE_INF = std::numeric_limits<double>::infinity();

bool IsFiniteVal (double theVal)
{
  return theVal > -THE_INF && theVal < THE_INF;
}

} // namespace

//=================================================================================================

NvGeInterval::NvGeInterval (double theTol)
: mpImpInt (new NvGeImpInterval()),
  mDelInt (1)
{
  mpImpInt->Tolerance = theTol;
}

//=================================================================================================

NvGeInterval::NvGeInterval (const NvGeInterval& theSrc)
: mpImpInt (new NvGeImpInterval()),
  mDelInt (1)
{
  mpImpInt->LowerBound = theSrc.mpImpInt->LowerBound;
  mpImpInt->UpperBound = theSrc.mpImpInt->UpperBound;
  mpImpInt->Tolerance = theSrc.mpImpInt->Tolerance;
}

//=================================================================================================

NvGeInterval::NvGeInterval (double theLower, double theUpper, double theTol)
: mpImpInt (new NvGeImpInterval()),
  mDelInt (1)
{
  // A reversed pair is normalized by swapping, mirroring ARX behavior of
  // always producing a valid ordered interval.
  mpImpInt->LowerBound = theLower <= theUpper ? theLower : theUpper;
  mpImpInt->UpperBound = theLower <= theUpper ? theUpper : theLower;
  mpImpInt->Tolerance = theTol;
}

//=================================================================================================

NvGeInterval::NvGeInterval (Nova::Boolean theBoundedBelow, double theBound, double theTol)
: mpImpInt (new NvGeImpInterval()),
  mDelInt (1)
{
  if (theBoundedBelow)
  {
    mpImpInt->LowerBound = theBound;
    mpImpInt->UpperBound = THE_INF;
  }
  else
  {
    mpImpInt->LowerBound = -THE_INF;
    mpImpInt->UpperBound = theBound;
  }
  mpImpInt->Tolerance = theTol;
}

//=================================================================================================

NvGeInterval::NvGeInterval (NvGeImpInterval& theImpInt, int theDelInt)
: mpImpInt (&theImpInt),
  mDelInt (theDelInt)
{
}

//=================================================================================================

NvGeInterval::~NvGeInterval()
{
  if (mDelInt && mpImpInt != nullptr)
  {
    delete mpImpInt;
  }
}

//=================================================================================================

NvGeInterval& NvGeInterval::operator = (const NvGeInterval& theOtherInterval)
{
  if (this == &theOtherInterval)
  {
    return *this;
  }
  mpImpInt->LowerBound = theOtherInterval.mpImpInt->LowerBound;
  mpImpInt->UpperBound = theOtherInterval.mpImpInt->UpperBound;
  mpImpInt->Tolerance = theOtherInterval.mpImpInt->Tolerance;
  return *this;
}

//=================================================================================================

double NvGeInterval::lowerBound() const
{
  return mpImpInt->LowerBound;
}

//=================================================================================================

double NvGeInterval::upperBound() const
{
  return mpImpInt->UpperBound;
}

//=================================================================================================

double NvGeInterval::element() const
{
  // The single bounded end of a half-open interval; the lower bound of a
  // fully bounded interval.
  if (!IsFiniteVal (mpImpInt->UpperBound))
  {
    return mpImpInt->LowerBound;
  }
  if (!IsFiniteVal (mpImpInt->LowerBound))
  {
    return mpImpInt->UpperBound;
  }
  return mpImpInt->LowerBound;
}

//=================================================================================================

void NvGeInterval::getBounds (double& theLower, double& theUpper) const
{
  theLower = mpImpInt->LowerBound;
  theUpper = mpImpInt->UpperBound;
}

//=================================================================================================

double NvGeInterval::length() const
{
  return mpImpInt->UpperBound - mpImpInt->LowerBound;
}

//=================================================================================================

double NvGeInterval::tolerance() const
{
  return mpImpInt->Tolerance;
}

//=================================================================================================

NvGeInterval& NvGeInterval::set (double theLower, double theUpper)
{
  mpImpInt->LowerBound = theLower <= theUpper ? theLower : theUpper;
  mpImpInt->UpperBound = theLower <= theUpper ? theUpper : theLower;
  return *this;
}

//=================================================================================================

NvGeInterval& NvGeInterval::set (Nova::Boolean theBoundedBelow, double theBound)
{
  if (theBoundedBelow)
  {
    mpImpInt->LowerBound = theBound;
    mpImpInt->UpperBound = THE_INF;
  }
  else
  {
    mpImpInt->LowerBound = -THE_INF;
    mpImpInt->UpperBound = theBound;
  }
  return *this;
}

//=================================================================================================

NvGeInterval& NvGeInterval::set()
{
  mpImpInt->LowerBound = -THE_INF;
  mpImpInt->UpperBound = THE_INF;
  return *this;
}

//=================================================================================================

NvGeInterval& NvGeInterval::setUpper (double theUpper)
{
  mpImpInt->UpperBound = theUpper < mpImpInt->LowerBound ? mpImpInt->LowerBound : theUpper;
  return *this;
}

//=================================================================================================

NvGeInterval& NvGeInterval::setLower (double theLower)
{
  mpImpInt->LowerBound = theLower > mpImpInt->UpperBound ? mpImpInt->UpperBound : theLower;
  return *this;
}

//=================================================================================================

NvGeInterval& NvGeInterval::setTolerance (double theTol)
{
  mpImpInt->Tolerance = theTol;
  return *this;
}

//=================================================================================================

void NvGeInterval::getMerge (const NvGeInterval& theOtherInterval, NvGeInterval& theResult) const
{
  const double aLower = mpImpInt->LowerBound < theOtherInterval.mpImpInt->LowerBound
                      ? mpImpInt->LowerBound : theOtherInterval.mpImpInt->LowerBound;
  const double anUpper = mpImpInt->UpperBound > theOtherInterval.mpImpInt->UpperBound
                       ? mpImpInt->UpperBound : theOtherInterval.mpImpInt->UpperBound;
  theResult.set (aLower, anUpper);
}

//=================================================================================================

int NvGeInterval::subtract (const NvGeInterval& theOtherInterval,
                            NvGeInterval& theLInterval,
                            NvGeInterval& theRInterval) const
{
  if (isDisjoint (theOtherInterval))
  {
    theLInterval = *this;
    return 1;
  }
  const bool aCoversLower = theOtherInterval.mpImpInt->LowerBound
                          <= mpImpInt->LowerBound + mpImpInt->Tolerance;
  const bool aCoversUpper = theOtherInterval.mpImpInt->UpperBound
                          >= mpImpInt->UpperBound - mpImpInt->Tolerance;
  if (aCoversLower && aCoversUpper)
  {
    return 0; // this is fully eaten by the other interval
  }
  if (!aCoversLower && aCoversUpper)
  {
    theLInterval.set (mpImpInt->LowerBound, theOtherInterval.mpImpInt->LowerBound);
    return 1;
  }
  if (aCoversLower && !aCoversUpper)
  {
    theLInterval.set (theOtherInterval.mpImpInt->UpperBound, mpImpInt->UpperBound);
    return 1;
  }
  theLInterval.set (mpImpInt->LowerBound, theOtherInterval.mpImpInt->LowerBound);
  theRInterval.set (theOtherInterval.mpImpInt->UpperBound, mpImpInt->UpperBound);
  return 2;
}

//=================================================================================================

Nova::Boolean NvGeInterval::intersectWith (const NvGeInterval& theOtherInterval,
                                            NvGeInterval& theResult) const
{
  if (isDisjoint (theOtherInterval))
  {
    return false;
  }
  const double aLower = mpImpInt->LowerBound > theOtherInterval.mpImpInt->LowerBound
                      ? mpImpInt->LowerBound : theOtherInterval.mpImpInt->LowerBound;
  const double anUpper = mpImpInt->UpperBound < theOtherInterval.mpImpInt->UpperBound
                       ? mpImpInt->UpperBound : theOtherInterval.mpImpInt->UpperBound;
  theResult.set (aLower, anUpper);
  return true;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isBounded() const
{
  return IsFiniteVal (mpImpInt->LowerBound) && IsFiniteVal (mpImpInt->UpperBound);
}

//=================================================================================================

Nova::Boolean NvGeInterval::isBoundedAbove() const
{
  return IsFiniteVal (mpImpInt->UpperBound);
}

//=================================================================================================

Nova::Boolean NvGeInterval::isBoundedBelow() const
{
  return IsFiniteVal (mpImpInt->LowerBound);
}

//=================================================================================================

Nova::Boolean NvGeInterval::isUnBounded() const
{
  return !IsFiniteVal (mpImpInt->LowerBound) && !IsFiniteVal (mpImpInt->UpperBound);
}

//=================================================================================================

Nova::Boolean NvGeInterval::isSingleton() const
{
  return isBounded()
      && std::abs (mpImpInt->UpperBound - mpImpInt->LowerBound) <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isDisjoint (const NvGeInterval& theOtherInterval) const
{
  const double aTol = std::max (mpImpInt->Tolerance, theOtherInterval.mpImpInt->Tolerance);
  return mpImpInt->UpperBound < theOtherInterval.mpImpInt->LowerBound - aTol
      || theOtherInterval.mpImpInt->UpperBound < mpImpInt->LowerBound - aTol;
}

//=================================================================================================

Nova::Boolean NvGeInterval::contains (const NvGeInterval& theOtherInterval) const
{
  return mpImpInt->LowerBound <= theOtherInterval.mpImpInt->LowerBound + mpImpInt->Tolerance
      && mpImpInt->UpperBound >= theOtherInterval.mpImpInt->UpperBound - mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::contains (double theVal) const
{
  return mpImpInt->LowerBound - mpImpInt->Tolerance <= theVal
      && theVal <= mpImpInt->UpperBound + mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isContinuousAtUpper (const NvGeInterval& theOtherInterval) const
{
  // Intervals join when this ends exactly where the other starts.
  return std::abs (mpImpInt->UpperBound - theOtherInterval.mpImpInt->LowerBound)
       <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isOverlapAtUpper (const NvGeInterval& theOtherInterval,
                                               NvGeInterval& theOverlap) const
{
  return intersectWith (theOtherInterval, theOverlap);
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator == (const NvGeInterval& theOtherInterval) const
{
  return isEqualAtLower (theOtherInterval) && isEqualAtUpper (theOtherInterval);
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator != (const NvGeInterval& theOtherInterval) const
{
  return !(*this == theOtherInterval);
}

//=================================================================================================

Nova::Boolean NvGeInterval::isEqualAtUpper (const NvGeInterval& theOtherInterval) const
{
  return std::abs (mpImpInt->UpperBound - theOtherInterval.mpImpInt->UpperBound)
       <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isEqualAtUpper (double theValue) const
{
  return std::abs (mpImpInt->UpperBound - theValue) <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isEqualAtLower (const NvGeInterval& theOtherInterval) const
{
  return std::abs (mpImpInt->LowerBound - theOtherInterval.mpImpInt->LowerBound)
       <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isEqualAtLower (double theValue) const
{
  return std::abs (mpImpInt->LowerBound - theValue) <= mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::isPeriodicallyOn (double thePeriod, double& theVal)
{
  if (std::abs (thePeriod) < mpImpInt->Tolerance || !isBoundedBelow())
  {
    return false;
  }
  const double aPeriod = std::abs (thePeriod);
  const double aBase = mpImpInt->LowerBound;
  double anOffset = std::fmod (theVal - aBase, aPeriod);
  if (anOffset < 0.0)
  {
    anOffset += aPeriod;
  }
  const double anAdjusted = aBase + anOffset;
  if (contains (anAdjusted))
  {
    theVal = anAdjusted;
    return true;
  }
  return false;
}

//=================================================================================================

Nova::Boolean operator > (double theVal, const NvGeInterval& theIntrvl)
{
  return theVal > theIntrvl.mpImpInt->UpperBound + theIntrvl.mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator > (double theVal) const
{
  return mpImpInt->LowerBound > theVal + mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator > (const NvGeInterval& theOtherInterval) const
{
  return mpImpInt->LowerBound > theOtherInterval.mpImpInt->UpperBound + mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean operator >= (double theVal, const NvGeInterval& theIntrvl)
{
  return theVal >= theIntrvl.mpImpInt->UpperBound - theIntrvl.mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator >= (double theVal) const
{
  return mpImpInt->LowerBound >= theVal - mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator >= (const NvGeInterval& theOtherInterval) const
{
  return mpImpInt->LowerBound >= theOtherInterval.mpImpInt->UpperBound - mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean operator < (double theVal, const NvGeInterval& theIntrvl)
{
  return theVal < theIntrvl.mpImpInt->LowerBound - theIntrvl.mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator < (double theVal) const
{
  return mpImpInt->UpperBound < theVal - mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator < (const NvGeInterval& theOtherInterval) const
{
  return mpImpInt->UpperBound < theOtherInterval.mpImpInt->LowerBound - mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean operator <= (double theVal, const NvGeInterval& theIntrvl)
{
  return theVal <= theIntrvl.mpImpInt->LowerBound + theIntrvl.mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator <= (double theVal) const
{
  return mpImpInt->UpperBound <= theVal + mpImpInt->Tolerance;
}

//=================================================================================================

Nova::Boolean NvGeInterval::operator <= (const NvGeInterval& theOtherInterval) const
{
  return mpImpInt->UpperBound <= theOtherInterval.mpImpInt->LowerBound + mpImpInt->Tolerance;
}
