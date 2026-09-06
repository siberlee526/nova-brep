// gepnt3d.cpp - implementation of NvGePoint3d.
//
// A point is transformed as an affine image of its homogeneous coordinates:
// applying a matrix (mat * pnt) evaluates the FULL homogeneous transform in
// the column-vector convention, i.e. the 3x3 linear part acts on the
// coordinates and the translation column is added.
//
// Planar operations (construction from a plane, convert2d, projection,
// mirroring) are expressed through the coordinate system (origin, u-axis,
// v-axis) of the planar entity; the plane normal is the cross product of
// the u and v axes.

#include <gepnt3d.h>

#include <NvException.h>
#include <gemat3d.h>
#include <gepnt2d.h>
#include <geplanar.h>
#include <geplane.h>
#include <getol.h>
#include <gevec3d.h>

#include <cmath>

//=================================================================================================

NvGePoint3d::NvGePoint3d (const NvGePlanarEnt& thePln, const NvGePoint2d& thePnt2d)
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePln.getCoordSystem (anOrigin, anAxis1, anAxis2);

  // The 2d coordinates are local coordinates in the plane frame.
  x = anOrigin.x + thePnt2d.x * anAxis1.x + thePnt2d.y * anAxis2.x;
  y = anOrigin.y + thePnt2d.x * anAxis1.y + thePnt2d.y * anAxis2.y;
  z = anOrigin.z + thePnt2d.x * anAxis1.z + thePnt2d.y * anAxis2.z;
}

//=================================================================================================

// NvGePoint3d
const NvGePoint3d NvGePoint3d::kOrigin;

//=================================================================================================

NvGePoint3d operator * (const NvGeMatrix3d& theMat, const NvGePoint3d& thePnt)
{
  // Full homogeneous evaluation: the translation column participates, so a
  // transformed point moves with the whole matrix and not only with the
  // linear part.
  const double aX = theMat.entry[0][0] * thePnt.x + theMat.entry[0][1] * thePnt.y
                  + theMat.entry[0][2] * thePnt.z + theMat.entry[0][3];
  const double aY = theMat.entry[1][0] * thePnt.x + theMat.entry[1][1] * thePnt.y
                  + theMat.entry[1][2] * thePnt.z + theMat.entry[1][3];
  const double aZ = theMat.entry[2][0] * thePnt.x + theMat.entry[2][1] * thePnt.y
                  + theMat.entry[2][2] * thePnt.z + theMat.entry[2][3];
  return NvGePoint3d (aX, aY, aZ);
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::setToProduct (const NvGeMatrix3d& theMat, const NvGePoint3d& thePnt)
{
  *this = theMat * thePnt;
  return *this;
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::transformBy (const NvGeMatrix3d& theLeftSide)
{
  *this = theLeftSide * *this;
  return *this;
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::rotateBy (double theAngle, const NvGeVector3d& theVec,
                                    const NvGePoint3d& theWrtPoint)
{
  // The rotation matrix handles the normalized axis, the center of rotation
  // and the degenerate-axis rejection.
  NvGeMatrix3d aRot;
  aRot.setToRotation (theAngle, theVec, theWrtPoint);
  return setToProduct (aRot, *this);
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::mirror (const NvGePlane& thePlane)
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePlane.getCoordSystem (anOrigin, anAxis1, anAxis2);

  const NvGeVector3d aNormal = anAxis1.crossProduct (anAxis2);
  const double aNormalLen = std::sqrt (aNormal.lengthSqrd());
  if (aNormalLen <= NvGeContext::gTol.equalVector())
  {
    throw NvException ("NvGePoint3d::mirror(): the plane has a degenerate coordinate system");
  }

  // Reflection through the plane: p' = p - 2 * n * (n . (p - origin)) / |n|^2.
  const NvGeVector3d aDelta  = *this - anOrigin;
  const double aFactor = 2.0 * aNormal.dotProduct (aDelta) / aNormal.lengthSqrd();
  x -= aNormal.x * aFactor;
  y -= aNormal.y * aFactor;
  z -= aNormal.z * aFactor;
  return *this;
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::scaleBy (double theScaleFactor, const NvGePoint3d& theWrtPoint)
{
  // Scaling about an arbitrary center keeps the center fixed.
  x = theWrtPoint.x + theScaleFactor * (x - theWrtPoint.x);
  y = theWrtPoint.y + theScaleFactor * (y - theWrtPoint.y);
  z = theWrtPoint.z + theScaleFactor * (z - theWrtPoint.z);
  return *this;
}

//=================================================================================================

NvGePoint2d NvGePoint3d::convert2d (const NvGePlanarEnt& thePln) const
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePln.getCoordSystem (anOrigin, anAxis1, anAxis2);

  // The local coordinates of this point in the plane frame are the dot
  // products with its axes.
  const NvGeVector3d aDelta = *this - anOrigin;
  return NvGePoint2d (aDelta.dotProduct (anAxis1), aDelta.dotProduct (anAxis2));
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::setToSum (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  x = thePnt.x + theVec.x;
  y = thePnt.y + theVec.y;
  z = thePnt.z + theVec.z;
  return *this;
}

//=================================================================================================

double NvGePoint3d::distanceTo (const NvGePoint3d& thePnt) const
{
  return std::sqrt ((*this - thePnt).lengthSqrd());
}

//=================================================================================================

NvGePoint3d NvGePoint3d::project (const NvGePlane& thePlane, const NvGeVector3d& theVec) const
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePlane.getCoordSystem (anOrigin, anAxis1, anAxis2);

  const NvGeVector3d aNormal = anAxis1.crossProduct (anAxis2);
  const double aDenom = aNormal.dotProduct (theVec);
  if (std::abs (aDenom) <= NvGeContext::gTol.equalVector())
  {
    throw NvException ("NvGePoint3d::project(): the projection direction is parallel to the plane");
  }

  // Move along the projection direction until the image lies in the plane.
  const double aParam = -aNormal.dotProduct (*this - anOrigin) / aDenom;
  return *this + theVec * aParam;
}

//=================================================================================================

NvGePoint3d NvGePoint3d::orthoProject (const NvGePlane& thePlane) const
{
  NvGePoint3d  anOrigin;
  NvGeVector3d anAxis1;
  NvGeVector3d anAxis2;
  thePlane.getCoordSystem (anOrigin, anAxis1, anAxis2);

  const NvGeVector3d aNormal = anAxis1.crossProduct (anAxis2);
  const double aNormalLen = std::sqrt (aNormal.lengthSqrd());
  if (aNormalLen <= NvGeContext::gTol.equalVector())
  {
    throw NvException ("NvGePoint3d::orthoProject(): the plane has a degenerate coordinate system");
  }

  // Orthogonal projection removes the full normal component of the offset
  // from the plane.
  const NvGeVector3d aDelta = *this - anOrigin;
  const double aFactor = aNormal.dotProduct (aDelta) / aNormal.lengthSqrd();
  return NvGePoint3d (x - aNormal.x * aFactor, y - aNormal.y * aFactor, z - aNormal.z * aFactor);
}

//=================================================================================================

bool NvGePoint3d::isEqualTo (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  return std::sqrt ((*this - thePnt).lengthSqrd()) <= theTol.equalPoint();
}

//=================================================================================================

NvGePoint3d& NvGePoint3d::set (const NvGePlanarEnt& thePln, const NvGePoint2d& thePnt)
{
  *this = NvGePoint3d (thePln, thePnt);
  return *this;
}
