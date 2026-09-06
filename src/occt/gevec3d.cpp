// gevec3d.cpp - implementation of NvGeVector3d.
//
// A 3d vector stores plain (x, y, z) components. Operations following from
// elementary trigonometry (rotation, mirroring, angles) are written directly
// from their defining formulas, with gp_Vec performing the Rodrigues
// rotation; tolerance-dependent queries compare against
// NvGeTol::equalVector(), mirroring the ARX AcGeVector3d semantics.
// Multiplication by a matrix applies its linear part only: a direction has
// no position, so the translation column never participates.

#include <gevec3d.h>

#include <NvException.h>
#include <gegblabb.h>
#include <gemat3d.h>
#include <gepnt3d.h>
#include <geplanar.h>
#include <getol.h>
#include <gevec2d.h>

#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>

namespace
{

// Full turn, used to shift the negative results of atan2 so that the
// angles measured between vectors always lie in [0, 2*pi).
constexpr double THE_TWO_PI = 6.28318530717958647692;

}

//=================================================================================================

// A 2d vector is lifted onto the plane by combining its coordinates with the
// plane's first and second axis; the plane origin is irrelevant for a
// direction.
NvGeVector3d::NvGeVector3d (const NvGePlanarEnt& thePln, const NvGeVector2d& theVec)
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePln.getCoordSystem (anOrigin, anAxis1, anAxis2);
  set (anAxis1.x * theVec.x + anAxis2.x * theVec.y,
       anAxis1.y * theVec.x + anAxis2.y * theVec.y,
       anAxis1.z * theVec.x + anAxis2.z * theVec.y);
}

//=================================================================================================

// The default constructor builds the zero vector, so the additive identity
// is simply a default-constructed instance.
const NvGeVector3d NvGeVector3d::kIdentity;

//=================================================================================================

const NvGeVector3d NvGeVector3d::kXAxis (1.0, 0.0, 0.0);

const NvGeVector3d NvGeVector3d::kYAxis (0.0, 1.0, 0.0);

const NvGeVector3d NvGeVector3d::kZAxis (0.0, 0.0, 1.0);

//=================================================================================================

NvGeVector3d operator * (const NvGeMatrix3d& theMat, const NvGeVector3d& theVec)
{
  // Linear part only; the translation column does not affect a direction.
  return NvGeVector3d (theMat.entry[0][0] * theVec.x
                     + theMat.entry[0][1] * theVec.y
                     + theMat.entry[0][2] * theVec.z,
                       theMat.entry[1][0] * theVec.x
                     + theMat.entry[1][1] * theVec.y
                     + theMat.entry[1][2] * theVec.z,
                       theMat.entry[2][0] * theVec.x
                     + theMat.entry[2][1] * theVec.y
                     + theMat.entry[2][2] * theVec.z);
}

//=================================================================================================

NvGeVector3d& NvGeVector3d::transformBy (const NvGeMatrix3d& theLeftSide)
{
  const double aX = theLeftSide.entry[0][0] * x
                  + theLeftSide.entry[0][1] * y
                  + theLeftSide.entry[0][2] * z;
  const double aY = theLeftSide.entry[1][0] * x
                  + theLeftSide.entry[1][1] * y
                  + theLeftSide.entry[1][2] * z;
  const double aZ = theLeftSide.entry[2][0] * x
                  + theLeftSide.entry[2][1] * y
                  + theLeftSide.entry[2][2] * z;
  x = aX;
  y = aY;
  z = aZ;
  return *this;
}

//=================================================================================================

// Rodrigues rotation about the axis through the origin; gp_Vec::Rotate
// performs the same computation for the normalized axis.
NvGeVector3d& NvGeVector3d::rotateBy (double theAngle, const NvGeVector3d& theAxis)
{
  if (theAxis.length() <= gp::Resolution())
  {
    throw NvException ("NvGeVector3d::rotateBy(): the rotation axis is zero-length");
  }
  const gp_Ax1 anAxis (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (theAxis.x, theAxis.y, theAxis.z));
  gp_Vec aVec (x, y, z);
  aVec.Rotate (anAxis, theAngle);
  x = aVec.X();
  y = aVec.Y();
  z = aVec.Z();
  return *this;
}

//=================================================================================================

// Reflection in the plane through the origin perpendicular to
// theNormalToPlane: v' = v - 2 * (v . n) / |n|^2 * n. The result is
// independent of the length of the normal.
NvGeVector3d& NvGeVector3d::mirror (const NvGeVector3d& theNormalToPlane)
{
  const double aLenSqrd = theNormalToPlane.lengthSqrd();
  if (aLenSqrd <= gp::Resolution())
  {
    throw NvException ("NvGeVector3d::mirror(): the mirror plane normal is zero-length");
  }
  const double aFactor = 2.0 * this->dotProduct (theNormalToPlane) / aLenSqrd;
  x -= aFactor * theNormalToPlane.x;
  y -= aFactor * theNormalToPlane.y;
  z -= aFactor * theNormalToPlane.z;
  return *this;
}

//=================================================================================================

// Expresses the vector in the plane's uv coordinate system; any component
// along the plane normal is dropped.
NvGeVector2d NvGeVector3d::convert2d (const NvGePlanarEnt& thePln) const
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePln.getCoordSystem (anOrigin, anAxis1, anAxis2);
  return NvGeVector2d (this->dotProduct (anAxis1), this->dotProduct (anAxis2));
}

//=================================================================================================

NvGeVector3d operator * (double theScale, const NvGeVector3d& theVec)
{
  return theVec * theScale;
}

//=================================================================================================

// Returns an arbitrary unit vector perpendicular to this one: the coordinate
// axis least aligned with this vector is crossed out, which keeps the result
// well conditioned for every input direction. The zero vector yields the
// zero vector.
NvGeVector3d NvGeVector3d::perpVector () const
{
  NvGeVector3d aResult;
  if (std::abs (x) <= std::abs (y) && std::abs (x) <= std::abs (z))
  {
    aResult = NvGeVector3d (0.0, -z, y);   // cross product with the x-axis
  }
  else if (std::abs (y) <= std::abs (z))
  {
    aResult = NvGeVector3d (z, 0.0, -x);   // cross product with the y-axis
  }
  else
  {
    aResult = NvGeVector3d (-y, x, 0.0);   // cross product with the z-axis
  }
  const double aLen = aResult.length();
  if (aLen > 0.0)
  {
    aResult /= aLen;
  }
  return aResult;
}

//=================================================================================================

// The unsigned angle between the two directions, in [0, pi]; atan2 of the
// cross-product magnitude and the dot product is numerically robust.
double NvGeVector3d::angleTo (const NvGeVector3d& theVec) const
{
  return std::atan2 (this->crossProduct (theVec).length(), this->dotProduct (theVec));
}

//=================================================================================================

// The angle between the two directions measured around theAxis,
// counterclockwise when looking against the axis direction; normalized into
// [0, 2*pi).
double NvGeVector3d::angleTo (const NvGeVector3d& theVec, const NvGeVector3d& theAxis) const
{
  if (theAxis.length() <= gp::Resolution())
  {
    throw NvException ("NvGeVector3d::angleTo(): the reference axis is zero-length");
  }
  const NvGeVector3d anAxis = theAxis / theAxis.length();
  double anAngle = std::atan2 (this->crossProduct (theVec).dotProduct (anAxis),
                               this->dotProduct (theVec));
  if (anAngle < 0.0)
  {
    anAngle += THE_TWO_PI;
  }
  return anAngle;
}

//=================================================================================================

// The angle of the vector within the plane's uv coordinate system measured
// counterclockwise from the plane's first axis; result in [-pi, pi].
double NvGeVector3d::angleOnPlane (const NvGePlanarEnt& thePln) const
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePln.getCoordSystem (anOrigin, anAxis1, anAxis2);
  return std::atan2 (this->dotProduct (anAxis2), this->dotProduct (anAxis1));
}

//=================================================================================================

// A zero-length vector has no direction; it is returned unchanged.
NvGeVector3d NvGeVector3d::normal (const NvGeTol& theTol) const
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
NvGeVector3d& NvGeVector3d::normalize (const NvGeTol& theTol)
{
  const double aLen = length();
  if (aLen > theTol.equalVector())
  {
    x /= aLen;
    y /= aLen;
    z /= aLen;
  }
  return *this;
}

//=================================================================================================

NvGeVector3d& NvGeVector3d::normalize (const NvGeTol& theTol, NvGeError& theFlag)
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
  z /= aLen;
  return *this;
}

//=================================================================================================

double NvGeVector3d::length () const
{
  return std::sqrt (lengthSqrd());
}

//=================================================================================================

Adesk::Boolean NvGeVector3d::isUnitLength (const NvGeTol& theTol) const
{
  return std::abs (length() - 1.0) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGeVector3d::isZeroLength (const NvGeTol& theTol) const
{
  return length() <= theTol.equalVector();
}

//=================================================================================================

// Parallel when the cross product is (nearly) the zero vector.
Adesk::Boolean NvGeVector3d::isParallelTo (const NvGeVector3d& theVec, const NvGeTol& theTol) const
{
  return this->crossProduct (theVec).length() <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGeVector3d::isParallelTo (const NvGeVector3d& theVec, const NvGeTol& theTol,
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
Adesk::Boolean NvGeVector3d::isCodirectionalTo (const NvGeVector3d& theVec,
                                                const NvGeTol& theTol) const
{
  return isParallelTo (theVec, theTol) && this->dotProduct (theVec) > 0.0;
}

//=================================================================================================

Adesk::Boolean NvGeVector3d::isCodirectionalTo (const NvGeVector3d& theVec, const NvGeTol& theTol,
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

Adesk::Boolean NvGeVector3d::isPerpendicularTo (const NvGeVector3d& theVec,
                                                const NvGeTol& theTol) const
{
  return std::abs (this->dotProduct (theVec)) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGeVector3d::isPerpendicularTo (const NvGeVector3d& theVec, const NvGeTol& theTol,
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

NvGeVector3d NvGeVector3d::crossProduct (const NvGeVector3d& theVec) const
{
  return NvGeVector3d (y * theVec.z - z * theVec.y,
                       z * theVec.x - x * theVec.z,
                       x * theVec.y - y * theVec.x);
}

//=================================================================================================

// Builds the rotation about theAxis that maps this vector onto theVec. When
// the axis is the zero-length default, this x theVec is used, which yields
// the smallest rotation; exactly parallel vectors fall back to an arbitrary
// perpendicular axis, so the rotation angle becomes zero (codirectional) or
// pi (opposite). The angle is measured between the components perpendicular
// to the axis, so the result maps this onto theVec for every valid axis.
NvGeMatrix3d NvGeVector3d::rotateTo (const NvGeVector3d& theVec, const NvGeVector3d& theAxis) const
{
  if (length() <= gp::Resolution() || theVec.length() <= gp::Resolution())
  {
    throw NvException ("NvGeVector3d::rotateTo(): a zero-length vector has no direction to rotate");
  }

  NvGeVector3d anAxis = theAxis;
  if (anAxis.length() <= gp::Resolution())
  {
    anAxis = this->crossProduct (theVec);
    if (anAxis.length() <= gp::Resolution())
    {
      anAxis = this->perpVector();
    }
  }
  anAxis.normalize();

  const NvGeVector3d aThisOrtho = *this - anAxis * this->dotProduct (anAxis);
  const NvGeVector3d aVecOrtho  = theVec - anAxis * theVec.dotProduct (anAxis);
  const double anAngle = std::atan2 (aThisOrtho.crossProduct (aVecOrtho).dotProduct (anAxis),
                                     aThisOrtho.dotProduct (aVecOrtho));

  NvGeMatrix3d aResult;
  aResult.setToRotation (anAngle, anAxis);
  return aResult;
}

//=================================================================================================

// Oblique projection onto the plane through the origin perpendicular to
// thePlaneNormal, along theProjectDirection:
// v' = v - d * (v . n) / (d . n).
NvGeVector3d NvGeVector3d::project (const NvGeVector3d& thePlaneNormal,
                                    const NvGeVector3d& theProjectDirection) const
{
  NvGeError aFlag = kOk;
  const NvGeVector3d aResult = project (thePlaneNormal, theProjectDirection,
                                        NvGeContext::gTol, aFlag);
  if (aFlag != kOk)
  {
    throw NvException ("NvGeVector3d::project(): the projection is undefined for the given input");
  }
  return aResult;
}

//=================================================================================================

NvGeVector3d NvGeVector3d::project (const NvGeVector3d& thePlaneNormal,
                                    const NvGeVector3d& theProjectDirection,
                                    const NvGeTol& theTol, NvGeError& theFlag) const
{
  if (thePlaneNormal.isZeroLength (theTol))
  {
    theFlag = k0Arg1;
    return *this;
  }
  if (theProjectDirection.isZeroLength (theTol))
  {
    theFlag = k0Arg2;
    return *this;
  }
  if (std::abs (thePlaneNormal.dotProduct (theProjectDirection)) <= theTol.equalVector())
  {
    // The direction lies in the plane, so a parallel ray never meets it.
    theFlag = kPerpendicularArg1Arg2;
    return *this;
  }
  theFlag = kOk;
  const double aFactor = this->dotProduct (thePlaneNormal)
                       / thePlaneNormal.dotProduct (theProjectDirection);
  return *this - theProjectDirection * aFactor;
}

//=================================================================================================

// Orthogonal projection onto the plane through the origin perpendicular to
// thePlaneNormal: v' = v - (v . n) / |n|^2 * n.
NvGeVector3d NvGeVector3d::orthoProject (const NvGeVector3d& thePlaneNormal) const
{
  NvGeError aFlag = kOk;
  const NvGeVector3d aResult = orthoProject (thePlaneNormal, NvGeContext::gTol, aFlag);
  if (aFlag != kOk)
  {
    throw NvException ("NvGeVector3d::orthoProject(): the plane normal is zero-length");
  }
  return aResult;
}

//=================================================================================================

NvGeVector3d NvGeVector3d::orthoProject (const NvGeVector3d& thePlaneNormal,
                                         const NvGeTol& theTol, NvGeError& theFlag) const
{
  if (thePlaneNormal.isZeroLength (theTol))
  {
    theFlag = k0Arg1;
    return *this;
  }
  theFlag = kOk;
  const double aFactor = this->dotProduct (thePlaneNormal) / thePlaneNormal.lengthSqrd();
  return NvGeVector3d (x - aFactor * thePlaneNormal.x,
                       y - aFactor * thePlaneNormal.y,
                       z - aFactor * thePlaneNormal.z);
}

//=================================================================================================

// The Euclidean distance between the component triples is within the vector
// tolerance.
bool NvGeVector3d::isEqualTo (const NvGeVector3d& theVec, const NvGeTol& theTol) const
{
  const double aDx = x - theVec.x;
  const double aDy = y - theVec.y;
  const double aDz = z - theVec.z;
  return std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz) <= theTol.equalVector();
}

//=================================================================================================

// Index of the component with the largest absolute value; ties resolve to
// the lowest index.
unsigned int NvGeVector3d::largestElement () const
{
  const double aX = std::abs (x);
  const double aY = std::abs (y);
  const double aZ = std::abs (z);
  if (aX >= aY && aX >= aZ)
  {
    return 0;
  }
  if (aY >= aZ)
  {
    return 1;
  }
  return 2;
}

//=================================================================================================

NvGeVector3d& NvGeVector3d::set (const NvGePlanarEnt& thePln, const NvGeVector2d& theVec)
{
  *this = NvGeVector3d (thePln, theVec);
  return *this;
}

//=================================================================================================

// The matrix form of a vector is the translation by that vector.
NvGeVector3d::operator NvGeMatrix3d () const
{
  return NvGeMatrix3d::translation (*this);
}
