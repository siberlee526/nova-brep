// gepnt2d.cpp - implementation of NvGePoint2d.
//
// A point is transformed as an affine image of its homogeneous coordinates:
// applying a matrix (mat * pnt) evaluates the FULL homogeneous transform in
// the column-vector convention, i.e. the 2x2 linear part acts on the
// coordinates and the translation column is added:
//
//   x' = entry[0][0]*x + entry[0][1]*y + entry[0][2]
//   y' = entry[1][0]*x + entry[1][1]*y + entry[1][2]
//
// Elementary operations (rotation, scaling about a point, mirroring across
// a line) are written directly from their defining formulas.

#include <gepnt2d.h>

#include <NvException.h>
#include <geline2d.h>
#include <gemat2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <cmath>

//=================================================================================================

// NvGePoint2d
const NvGePoint2d NvGePoint2d::kOrigin;

//=================================================================================================

NvGePoint2d operator * (const NvGeMatrix2d& theMat, const NvGePoint2d& thePnt)
{
  // Full homogeneous evaluation: the translation column participates, so a
  // transformed point moves with the whole matrix and not only with the
  // linear part.
  return NvGePoint2d (theMat.entry[0][0] * thePnt.x + theMat.entry[0][1] * thePnt.y
                        + theMat.entry[0][2],
                      theMat.entry[1][0] * thePnt.x + theMat.entry[1][1] * thePnt.y
                        + theMat.entry[1][2]);
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::setToProduct (const NvGeMatrix2d& theMat, const NvGePoint2d& thePnt)
{
  *this = theMat * thePnt;
  return *this;
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::transformBy (const NvGeMatrix2d& theLeftSide)
{
  *this = theLeftSide * *this;
  return *this;
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::rotateBy (double theAngle, const NvGePoint2d& theWrtPoint)
{
  // Rotation about an arbitrary center: express the point relative to the
  // center, rotate, and shift back.
  const double aCos = std::cos (theAngle);
  const double aSin = std::sin (theAngle);
  const double aDx  = x - theWrtPoint.x;
  const double aDy  = y - theWrtPoint.y;
  x = theWrtPoint.x + aDx * aCos - aDy * aSin;
  y = theWrtPoint.y + aDx * aSin + aDy * aCos;
  return *this;
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::mirror (const NvGeLine2d& theLine)
{
  const NvGePoint2d  aPnt = theLine.pointOnLine();
  const NvGeVector2d aDir = theLine.direction();

  const double aDirLen = std::sqrt (aDir.lengthSqrd());
  if (aDirLen <= NvGeContext::gTol.equalVector())
  {
    throw NvException ("NvGePoint2d::mirror(): the line has a degenerate direction");
  }

  // Reflection across the line: p' = P + (2 * d * d^T - I) * (p - P) with d
  // the unit direction of the line.
  const NvGeVector2d aUnit      = aDir / aDirLen;
  const NvGeVector2d aDelta     = *this - aPnt;
  const NvGeVector2d aReflected = aUnit * (2.0 * aUnit.dotProduct (aDelta)) - aDelta;
  x = aPnt.x + aReflected.x;
  y = aPnt.y + aReflected.y;
  return *this;
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::scaleBy (double theScaleFactor, const NvGePoint2d& theWrtPoint)
{
  // Scaling about an arbitrary center keeps the center fixed.
  x = theWrtPoint.x + theScaleFactor * (x - theWrtPoint.x);
  y = theWrtPoint.y + theScaleFactor * (y - theWrtPoint.y);
  return *this;
}

//=================================================================================================

NvGePoint2d& NvGePoint2d::setToSum (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  x = thePnt.x + theVec.x;
  y = thePnt.y + theVec.y;
  return *this;
}

//=================================================================================================

double NvGePoint2d::distanceTo (const NvGePoint2d& thePnt) const
{
  return std::sqrt ((*this - thePnt).lengthSqrd());
}

//=================================================================================================

bool NvGePoint2d::isEqualTo (const NvGePoint2d& thePnt, const NvGeTol& theTol) const
{
  return std::sqrt ((*this - thePnt).lengthSqrd()) <= theTol.equalPoint();
}
