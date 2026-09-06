// gevec2d.cpp - implementation of NvGeVector2d.
//
// A 2d vector stores plain (x, y) components. Operations following from
// elementary trigonometry (rotation, mirroring, angles) are written directly
// from their defining formulas; tolerance-dependent queries compare against
// NvGeTol::equalVector(), mirroring the ARX NvGeVector2d semantics.
// Multiplication by a matrix applies its linear part only: a direction has
// no position, so the translation column never participates.

#include <gevec2d.h>

#include <NvException.h>
#include <gegblabb.h>
#include <gemat2d.h>
#include <getol.h>

#include <gp.hxx>

#include <cmath>

namespace
{

// Full turn, used to shift the negative results of atan2 so that the
// angles measured between vectors always lie in [0, 2*pi).
constexpr double THE_TWO_PI = 6.28318530717958647692;

}

//=================================================================================================

// The default constructor builds the zero vector, so the additive identity
// is simply a default-constructed instance.
const NvGeVector2d NvGeVector2d::kIdentity;

//=================================================================================================

const NvGeVector2d NvGeVector2d::kXAxis (1.0, 0.0);

const NvGeVector2d NvGeVector2d::kYAxis (0.0, 1.0);

//=================================================================================================

NvGeVector2d operator * (const NvGeMatrix2d& theMat, const NvGeVector2d& theVec)
{
  // Linear part only; the translation column does not affect a direction.
  return NvGeVector2d (theMat.entry[0][0] * theVec.x + theMat.entry[0][1] * theVec.y,
                       theMat.entry[1][0] * theVec.x + theMat.entry[1][1] * theVec.y);
}

//=================================================================================================

NvGeVector2d& NvGeVector2d::transformBy (const NvGeMatrix2d& theLeftSide)
{
  const double aX = theLeftSide.entry[0][0] * x + theLeftSide.entry[0][1] * y;
  const double aY = theLeftSide.entry[1][0] * x + theLeftSide.entry[1][1] * y;
  x = aX;
  y = aY;
  return *this;
}

//=================================================================================================

NvGeVector2d& NvGeVector2d::rotateBy (double theAngle)
{
  const double aCos = std::cos (theAngle);
  const double aSin = std::sin (theAngle);
  const double aX = x * aCos - y * aSin;
  const double aY = x * aSin + y * aCos;
  x = aX;
  y = aY;
  return *this;
}

//=================================================================================================

// Reflection about the direction of theLine: v' = 2 * (v . d) / |d|^2 * d - v.
// The result is independent of the length of theLine.
NvGeVector2d& NvGeVector2d::mirror (const NvGeVector2d& theLine)
{
  const double aLenSqrd = theLine.lengthSqrd();
  if (aLenSqrd <= gp::Resolution())
  {
    throw NvException ("NvGeVector2d::mirror(): the mirror line direction is zero-length");
  }
  const double aFactor = 2.0 * this->dotProduct (theLine) / aLenSqrd;
  x = aFactor * theLine.x - x;
  y = aFactor * theLine.y - y;
  return *this;
}

//=================================================================================================

NvGeVector2d operator * (double theScale, const NvGeVector2d& theVec)
{
  return theVec * theScale;
}

//=================================================================================================

double NvGeVector2d::angle () const
{
  return std::atan2 (y, x);
}

//=================================================================================================

// The angle is measured counterclockwise from this vector to theVec and is
// normalized into [0, 2*pi): atan2 returns the signed angle in [-pi, pi],
// and every negative value is shifted by a full turn.
double NvGeVector2d::angleTo (const NvGeVector2d& theVec) const
{
  const double aCross = x * theVec.y - y * theVec.x;
  double anAngle = std::atan2 (aCross, this->dotProduct (theVec));
  if (anAngle < 0.0)
  {
    anAngle += THE_TWO_PI;
  }
  return anAngle;
}

//=================================================================================================

// A zero-length vector has no direction; it is returned unchanged.
NvGeVector2d NvGeVector2d::normal (const NvGeTol& theTol) const
{
  const double aLen = length();
  if (aLen <= theTol.equalVector())
  {
    return *this;
  }
  return *this / aLen;
}

//=================================================================================================

// A zero-length vector has no direction; it is left unchanged.
NvGeVector2d& NvGeVector2d::normalize (const NvGeTol& theTol)
{
  const double aLen = length();
  if (aLen > theTol.equalVector())
  {
    x /= aLen;
    y /= aLen;
  }
  return *this;
}

//=================================================================================================

NvGeVector2d& NvGeVector2d::normalize (const NvGeTol& theTol, NvGeError& theFlag)
{
  const double aLen = length();
  if (aLen <= theTol.equalVector())
  {
    theFlag = k0This;
    return *this;
  }
  theFlag = kOk;
  x /= aLen;
  y /= aLen;
  return *this;
}

//=================================================================================================

double NvGeVector2d::length () const
{
  return std::sqrt (lengthSqrd());
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isUnitLength (const NvGeTol& theTol) const
{
  return std::abs (length() - 1.0) <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isZeroLength (const NvGeTol& theTol) const
{
  return length() <= theTol.equalVector();
}

//=================================================================================================

// Parallel when the magnitude of the cross product (the area of the
// parallelogram spanned by the two vectors) is within the vector tolerance.
Nova::Boolean NvGeVector2d::isParallelTo (const NvGeVector2d& theVec, const NvGeTol& theTol) const
{
  return std::abs (x * theVec.y - y * theVec.x) <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isParallelTo (const NvGeVector2d& theVec, const NvGeTol& theTol,
                                           NvGeError& theFlag) const
{
  if (isZeroLength (theTol))
  {
    theFlag = k0This;
    return false;
  }
  if (theVec.isZeroLength (theTol))
  {
    theFlag = k0Arg1;
    return false;
  }
  theFlag = kOk;
  return isParallelTo (theVec, theTol);
}

//=================================================================================================

// Codirectional adds the same-direction requirement to parallelism: the dot
// product of two parallel vectors is positive exactly when they point the
// same way.
Nova::Boolean NvGeVector2d::isCodirectionalTo (const NvGeVector2d& theVec,
                                                const NvGeTol& theTol) const
{
  return isParallelTo (theVec, theTol) && this->dotProduct (theVec) > 0.0;
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isCodirectionalTo (const NvGeVector2d& theVec, const NvGeTol& theTol,
                                                NvGeError& theFlag) const
{
  if (isZeroLength (theTol))
  {
    theFlag = k0This;
    return false;
  }
  if (theVec.isZeroLength (theTol))
  {
    theFlag = k0Arg1;
    return false;
  }
  theFlag = kOk;
  return isCodirectionalTo (theVec, theTol);
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isPerpendicularTo (const NvGeVector2d& theVec,
                                                const NvGeTol& theTol) const
{
  return std::abs (this->dotProduct (theVec)) <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeVector2d::isPerpendicularTo (const NvGeVector2d& theVec, const NvGeTol& theTol,
                                                NvGeError& theFlag) const
{
  if (isZeroLength (theTol))
  {
    theFlag = k0This;
    return false;
  }
  if (theVec.isZeroLength (theTol))
  {
    theFlag = k0Arg1;
    return false;
  }
  theFlag = kOk;
  return isPerpendicularTo (theVec, theTol);
}

//=================================================================================================

// The Euclidean distance between the component pairs is within the vector
// tolerance.
bool NvGeVector2d::isEqualTo (const NvGeVector2d& theVec, const NvGeTol& theTol) const
{
  const double aDx = x - theVec.x;
  const double aDy = y - theVec.y;
  return std::sqrt (aDx * aDx + aDy * aDy) <= theTol.equalVector();
}

//=================================================================================================

// The matrix form of a vector is the translation by that vector.
NvGeVector2d::operator NvGeMatrix2d () const
{
  return NvGeMatrix2d::translation (*this);
}
