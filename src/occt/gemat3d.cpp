// gemat3d.cpp - implementation of NvGeMatrix3d.
//
// The matrix is a homogeneous 4x4 transform in the column-vector convention,
// exactly like NvGeMatrix2d one dimension up: entry[row][col], the product
// (A * B) applied to a point equals A applied to (B applied to the point).
// Elementary transformations are written directly from their defining
// formulas; general algebra (product, inversion, transposition) operates on
// the full 4x4 entries so externally mangled matrices stay correct.

#include <gemat3d.h>

#include <NvException.h>
#include <geline3d.h>
#include <gemat2d.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gp.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <limits>

namespace
{

// Full 4x4 homogeneous product; no assumption on the bottom row.
NvGeMatrix3d ProductOf (const NvGeMatrix3d& theLeft, const NvGeMatrix3d& theRight)
{
  NvGeMatrix3d aResult;
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      double aValue = 0.0;
      for (int aTerm = 0; aTerm < 4; ++aTerm)
      {
        aValue += theLeft.entry[aRow][aTerm] * theRight.entry[aTerm][aCol];
      }
      aResult.entry[aRow][aCol] = aValue;
    }
  }
  return aResult;
}

// Applies the linear part of the matrix to a 3d vector.
void ApplyLinear (const NvGeMatrix3d& theMat, double& theX, double& theY, double& theZ)
{
  const double aX = theMat.entry[0][0] * theX + theMat.entry[0][1] * theY + theMat.entry[0][2] * theZ;
  const double aY = theMat.entry[1][0] * theX + theMat.entry[1][1] * theY + theMat.entry[1][2] * theZ;
  const double aZ = theMat.entry[2][0] * theX + theMat.entry[2][1] * theY + theMat.entry[2][2] * theZ;
  theX = aX;
  theY = aY;
  theZ = aZ;
}

// Builds an orthonormal right-handed basis (theU, theV, theN) with theN the
// normalized input direction.
void BasisFromNormal (double theNx, double theNy, double theNz,
                      double& theUx, double& theUy, double& theUz,
                      double& theVx, double& theVy, double& theVz)
{
  const double aLen = std::sqrt (theNx * theNx + theNy * theNy + theNz * theNz);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d: a degenerate direction was given");
  }
  const double aNx = theNx / aLen;
  const double aNy = theNy / aLen;
  const double aNz = theNz / aLen;
  // Pick the coordinate axis least aligned with the normal as the seed.
  double anSx = 0.0, anSy = 0.0, anSz = 0.0;
  if (std::abs (aNx) <= std::abs (aNy) && std::abs (aNx) <= std::abs (aNz))
  {
    anSy = 1.0;
  }
  else if (std::abs (aNy) <= std::abs (aNz))
  {
    anSz = 1.0;
  }
  else
  {
    anSx = 1.0;
  }
  // u = seed x n (normalized), v = n x u.
  theUx = anSy * aNz - anSz * aNy;
  theUy = anSz * aNx - anSx * aNz;
  theUz = anSx * aNy - anSy * aNx;
  const double anULen = std::sqrt (theUx * theUx + theUy * theUy + theUz * theUz);
  theUx /= anULen;
  theUy /= anULen;
  theUz /= anULen;
  theVx = aNy * theUz - aNz * theUy;
  theVy = aNz * theUx - aNx * theUz;
  theVz = aNx * theUy - aNy * theUx;
}

} // namespace

//=================================================================================================

NvGeMatrix3d::NvGeMatrix3d()
{
  setToIdentity();
}

//=================================================================================================

NvGeMatrix3d::NvGeMatrix3d (const NvGeMatrix3d& theSrc)
{
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      entry[aRow][aCol] = theSrc.entry[aRow][aCol];
    }
  }
}

// The default constructor builds the identity matrix.
const NvGeMatrix3d NvGeMatrix3d::kIdentity;

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToIdentity()
{
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      entry[aRow][aCol] = (aRow == aCol) ? 1.0 : 0.0;
    }
  }
  return *this;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::operator * (const NvGeMatrix3d& theMat) const
{
  return ProductOf (*this, theMat);
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::operator *= (const NvGeMatrix3d& theMat)
{
  *this = ProductOf (*this, theMat);
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::preMultBy (const NvGeMatrix3d& theLeftSide)
{
  *this = ProductOf (theLeftSide, *this);
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::postMultBy (const NvGeMatrix3d& theRightSide)
{
  *this = ProductOf (*this, theRightSide);
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToProduct (const NvGeMatrix3d& theMat1, const NvGeMatrix3d& theMat2)
{
  *this = ProductOf (theMat1, theMat2);
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::invert()
{
  // Adjugate inverse of the full 4x4 matrix (cofactor expansion).
  const double* a = &entry[0][0];
  double aCof[16];
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      // 3x3 minor of the element (aRow, aCol).
      double aM[9];
      int anIdx = 0;
      for (int aR = 0; aR < 4; ++aR)
      {
        if (aR == aRow)
        {
          continue;
        }
        for (int aC = 0; aC < 4; ++aC)
        {
          if (aC == aCol)
          {
            continue;
          }
          aM[anIdx++] = a[aR * 4 + aC];
        }
      }
      const double aMinor = aM[0] * (aM[4] * aM[8] - aM[5] * aM[7])
                          - aM[1] * (aM[3] * aM[8] - aM[5] * aM[6])
                          + aM[2] * (aM[3] * aM[7] - aM[4] * aM[6]);
      aCof[aRow * 4 + aCol] = ((aRow + aCol) % 2 == 0) ? aMinor : -aMinor;
    }
  }
  const double aDet = a[0] * aCof[0] + a[1] * aCof[1] + a[2] * aCof[2] + a[3] * aCof[3];
  if (std::abs (aDet) <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::invert(): the matrix is singular and cannot be inverted");
  }
  // inverse = adjugate / determinant (adjugate = transposed cofactor matrix).
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      entry[aRow][aCol] = aCof[aCol * 4 + aRow] / aDet;
    }
  }
  return *this;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::inverse() const
{
  NvGeMatrix3d aResult (*this);
  aResult.invert();
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::inverse (const NvGeTol& theTol) const
{
  if (isSingular (theTol))
  {
    throw NvException ("NvGeMatrix3d::inverse(): the matrix is singular and cannot be inverted");
  }
  return inverse();
}

//=================================================================================================

Adesk::Boolean NvGeMatrix3d::isSingular (const NvGeTol& theTol) const
{
  return std::abs (det()) <= theTol.equalPoint();
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::transposeIt()
{
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = aRow + 1; aCol < 4; ++aCol)
    {
      const double aTmp = entry[aRow][aCol];
      entry[aRow][aCol] = entry[aCol][aRow];
      entry[aCol][aRow] = aTmp;
    }
  }
  return *this;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::transpose() const
{
  NvGeMatrix3d aResult (*this);
  aResult.transposeIt();
  return aResult;
}

//=================================================================================================

bool NvGeMatrix3d::isEqualTo (const NvGeMatrix3d& theMat, const NvGeTol& theTol) const
{
  for (int aRow = 0; aRow < 4; ++aRow)
  {
    for (int aCol = 0; aCol < 4; ++aCol)
    {
      if (std::abs (entry[aRow][aCol] - theMat.entry[aRow][aCol]) > theTol.equalPoint())
      {
        return false;
      }
    }
  }
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeMatrix3d::isUniScaledOrtho (const NvGeTol& theTol) const
{
  const double aTolVec = theTol.equalVector();
  double aNorms[3];
  for (int aCol = 0; aCol < 3; ++aCol)
  {
    aNorms[aCol] = std::sqrt (entry[0][aCol] * entry[0][aCol]
                            + entry[1][aCol] * entry[1][aCol]
                            + entry[2][aCol] * entry[2][aCol]);
  }
  if (aNorms[0] <= aTolVec)
  {
    return false;
  }
  for (int aCol = 1; aCol < 3; ++aCol)
  {
    if (std::abs (aNorms[0] - aNorms[aCol]) > aTolVec)
    {
      return false;
    }
  }
  for (int aC1 = 0; aC1 < 3; ++aC1)
  {
    for (int aC2 = aC1 + 1; aC2 < 3; ++aC2)
    {
      const double aDot = entry[0][aC1] * entry[0][aC2]
                        + entry[1][aC1] * entry[1][aC2]
                        + entry[2][aC1] * entry[2][aC2];
      if (std::abs (aDot) > aTolVec)
      {
        return false;
      }
    }
  }
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeMatrix3d::isScaledOrtho (const NvGeTol& theTol) const
{
  const double aTolVec = theTol.equalVector();
  for (int aCol = 0; aCol < 3; ++aCol)
  {
    if (std::sqrt (entry[0][aCol] * entry[0][aCol]
                 + entry[1][aCol] * entry[1][aCol]
                 + entry[2][aCol] * entry[2][aCol]) <= aTolVec)
    {
      return false;
    }
  }
  for (int aC1 = 0; aC1 < 3; ++aC1)
  {
    for (int aC2 = aC1 + 1; aC2 < 3; ++aC2)
    {
      const double aDot = entry[0][aC1] * entry[0][aC2]
                        + entry[1][aC1] * entry[1][aC2]
                        + entry[2][aC1] * entry[2][aC2];
      if (std::abs (aDot) > aTolVec)
      {
        return false;
      }
    }
  }
  return true;
}

//=================================================================================================

double NvGeMatrix3d::det() const
{
  return entry[0][0] * (entry[1][1] * entry[2][2] - entry[1][2] * entry[2][1])
       - entry[0][1] * (entry[1][0] * entry[2][2] - entry[1][2] * entry[2][0])
       + entry[0][2] * (entry[1][0] * entry[2][1] - entry[1][1] * entry[2][0]);
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setTranslation (const NvGeVector3d& theVec)
{
  entry[0][3] = theVec.x;
  entry[1][3] = theVec.y;
  entry[2][3] = theVec.z;
  return *this;
}

//=================================================================================================

NvGeVector3d NvGeMatrix3d::translation() const
{
  return NvGeVector3d (entry[0][3], entry[1][3], entry[2][3]);
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setCoordSystem (const NvGePoint3d& theOrigin,
                                            const NvGeVector3d& theE0,
                                            const NvGeVector3d& theE1,
                                            const NvGeVector3d& theE2)
{
  // Basis vectors become the columns; the matrix maps local to world.
  entry[0][0] = theE0.x; entry[0][1] = theE1.x; entry[0][2] = theE2.x;
  entry[1][0] = theE0.y; entry[1][1] = theE1.y; entry[1][2] = theE2.y;
  entry[2][0] = theE0.z; entry[2][1] = theE1.z; entry[2][2] = theE2.z;
  entry[0][3] = theOrigin.x;
  entry[1][3] = theOrigin.y;
  entry[2][3] = theOrigin.z;
  entry[3][0] = 0.0; entry[3][1] = 0.0; entry[3][2] = 0.0; entry[3][3] = 1.0;
  return *this;
}

//=================================================================================================

void NvGeMatrix3d::getCoordSystem (NvGePoint3d& theOrigin, NvGeVector3d& theE0,
                                   NvGeVector3d& theE1, NvGeVector3d& theE2) const
{
  theOrigin.set (entry[0][3], entry[1][3], entry[2][3]);
  theE0.set (entry[0][0], entry[1][0], entry[2][0]);
  theE1.set (entry[0][1], entry[1][1], entry[2][1]);
  theE2.set (entry[0][2], entry[1][2], entry[2][2]);
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToTranslation (const NvGeVector3d& theVec)
{
  setToIdentity();
  entry[0][3] = theVec.x;
  entry[1][3] = theVec.y;
  entry[2][3] = theVec.z;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToRotation (double theAngle, const NvGeVector3d& theAxis,
                                           const NvGePoint3d& theCenter)
{
  const double aLen = std::sqrt (theAxis.x * theAxis.x + theAxis.y * theAxis.y
                               + theAxis.z * theAxis.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToRotation(): the axis is degenerate");
  }
  // Rodrigues formula about the normalized axis.
  const double aKx = theAxis.x / aLen;
  const double aKy = theAxis.y / aLen;
  const double aKz = theAxis.z / aLen;
  const double aCos = std::cos (theAngle);
  const double aSin = std::sin (theAngle);
  const double aOneMinusCos = 1.0 - aCos;
  entry[0][0] = aCos + aKx * aKx * aOneMinusCos;
  entry[0][1] = aKx * aKy * aOneMinusCos - aKz * aSin;
  entry[0][2] = aKx * aKz * aOneMinusCos + aKy * aSin;
  entry[1][0] = aKy * aKx * aOneMinusCos + aKz * aSin;
  entry[1][1] = aCos + aKy * aKy * aOneMinusCos;
  entry[1][2] = aKy * aKz * aOneMinusCos - aKx * aSin;
  entry[2][0] = aKz * aKx * aOneMinusCos - aKy * aSin;
  entry[2][1] = aKz * aKy * aOneMinusCos + aKx * aSin;
  entry[2][2] = aCos + aKz * aKz * aOneMinusCos;

  // t = center - R * center
  double aCx = theCenter.x;
  double aCy = theCenter.y;
  double aCz = theCenter.z;
  ApplyLinear (*this, aCx, aCy, aCz);
  entry[0][3] = theCenter.x - aCx;
  entry[1][3] = theCenter.y - aCy;
  entry[2][3] = theCenter.z - aCz;
  entry[3][0] = 0.0; entry[3][1] = 0.0; entry[3][2] = 0.0; entry[3][3] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToScaling (double theScaleAll, const NvGePoint3d& theCenter)
{
  setToIdentity();
  entry[0][0] = theScaleAll;
  entry[1][1] = theScaleAll;
  entry[2][2] = theScaleAll;
  entry[0][3] = (1.0 - theScaleAll) * theCenter.x;
  entry[1][3] = (1.0 - theScaleAll) * theCenter.y;
  entry[2][3] = (1.0 - theScaleAll) * theCenter.z;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToMirroring (const NvGePlane& thePlane)
{
  // Reflection through the plane: x' = x - 2 * ((x - o) . n) * n.
  NvGePoint3d anOrigin;
  NvGeVector3d anU, aV;
  thePlane.getCoordSystem (anOrigin, anU, aV);
  double aNx = anU.y * aV.z - anU.z * aV.y;
  double aNy = anU.z * aV.x - anU.x * aV.z;
  double aNz = anU.x * aV.y - anU.y * aV.x;
  const double aLen = std::sqrt (aNx * aNx + aNy * aNy + aNz * aNz);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToMirroring(): the plane is degenerate");
  }
  aNx /= aLen;
  aNy /= aLen;
  aNz /= aLen;
  const double aTwo = 2.0;
  entry[0][0] = 1.0 - aTwo * aNx * aNx;
  entry[0][1] = -aTwo * aNx * aNy;
  entry[0][2] = -aTwo * aNx * aNz;
  entry[1][0] = -aTwo * aNx * aNy;
  entry[1][1] = 1.0 - aTwo * aNy * aNy;
  entry[1][2] = -aTwo * aNy * aNz;
  entry[2][0] = -aTwo * aNx * aNz;
  entry[2][1] = -aTwo * aNy * aNz;
  entry[2][2] = 1.0 - aTwo * aNz * aNz;
  const double aD = anOrigin.x * aNx + anOrigin.y * aNy + anOrigin.z * aNz;
  entry[0][3] = aTwo * aD * aNx;
  entry[1][3] = aTwo * aD * aNy;
  entry[2][3] = aTwo * aD * aNz;
  entry[3][0] = 0.0; entry[3][1] = 0.0; entry[3][2] = 0.0; entry[3][3] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToMirroring (const NvGePoint3d& thePnt)
{
  setToIdentity();
  entry[0][0] = -1.0;
  entry[1][1] = -1.0;
  entry[2][2] = -1.0;
  entry[0][3] = 2.0 * thePnt.x;
  entry[1][3] = 2.0 * thePnt.y;
  entry[2][3] = 2.0 * thePnt.z;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToMirroring (const NvGeLine3d& theLine)
{
  // Reflection through the line: x' = o + R * (x - o) with R = 2*k*k^T - I.
  const NvGePoint3d aPnt = theLine.pointOnLine();
  const NvGeVector3d aDir = theLine.direction();
  const double aLen = std::sqrt (aDir.x * aDir.x + aDir.y * aDir.y + aDir.z * aDir.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToMirroring(): the line has a degenerate direction");
  }
  const double aKx = aDir.x / aLen;
  const double aKy = aDir.y / aLen;
  const double aKz = aDir.z / aLen;
  const double aTwo = 2.0;
  entry[0][0] = aTwo * aKx * aKx - 1.0;
  entry[0][1] = aTwo * aKx * aKy;
  entry[0][2] = aTwo * aKx * aKz;
  entry[1][0] = aTwo * aKy * aKx;
  entry[1][1] = aTwo * aKy * aKy - 1.0;
  entry[1][2] = aTwo * aKy * aKz;
  entry[2][0] = aTwo * aKz * aKx;
  entry[2][1] = aTwo * aKz * aKy;
  entry[2][2] = aTwo * aKz * aKz - 1.0;
  double anOx = aPnt.x;
  double anOy = aPnt.y;
  double anOz = aPnt.z;
  ApplyLinear (*this, anOx, anOy, anOz);
  entry[0][3] = aPnt.x - anOx;
  entry[1][3] = aPnt.y - anOy;
  entry[2][3] = aPnt.z - anOz;
  entry[3][0] = 0.0; entry[3][1] = 0.0; entry[3][2] = 0.0; entry[3][3] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToProjection (const NvGePlane& thePlane, const NvGeVector3d& theProjDir)
{
  // Parallel projection onto the plane along theProjDir:
  // x' = x - ((x - o) . n) / (d . n) * d.
  NvGePoint3d anOrigin;
  NvGeVector3d anU, aV;
  thePlane.getCoordSystem (anOrigin, anU, aV);
  double aNx = anU.y * aV.z - anU.z * aV.y;
  double aNy = anU.z * aV.x - anU.x * aV.z;
  double aNz = anU.x * aV.y - anU.y * aV.x;
  const double aLen = std::sqrt (aNx * aNx + aNy * aNy + aNz * aNz);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToProjection(): the plane is degenerate");
  }
  aNx /= aLen;
  aNy /= aLen;
  aNz /= aLen;
  const double aDenom = theProjDir.x * aNx + theProjDir.y * aNy + theProjDir.z * aNz;
  if (std::abs (aDenom) <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToProjection(): the direction is parallel to the plane");
  }
  const double aDx = theProjDir.x / aDenom;
  const double aDy = theProjDir.y / aDenom;
  const double aDz = theProjDir.z / aDenom;
  entry[0][0] = 1.0 - aDx * aNx;
  entry[0][1] = -aDx * aNy;
  entry[0][2] = -aDx * aNz;
  entry[1][0] = -aDy * aNx;
  entry[1][1] = 1.0 - aDy * aNy;
  entry[1][2] = -aDy * aNz;
  entry[2][0] = -aDz * aNx;
  entry[2][1] = -aDz * aNy;
  entry[2][2] = 1.0 - aDz * aNz;
  const double aD = anOrigin.x * aNx + anOrigin.y * aNy + anOrigin.z * aNz;
  entry[0][3] = aDx * aD;
  entry[1][3] = aDy * aD;
  entry[2][3] = aDz * aD;
  entry[3][0] = 0.0; entry[3][1] = 0.0; entry[3][2] = 0.0; entry[3][3] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToAlignCoordSys (const NvGePoint3d& theFromOrigin,
                                                const NvGeVector3d& theFromE0,
                                                const NvGeVector3d& theFromE1,
                                                const NvGeVector3d& theFromE2,
                                                const NvGePoint3d& theToOrigin,
                                                const NvGeVector3d& theToE0,
                                                const NvGeVector3d& theToE1,
                                                const NvGeVector3d& theToE2)
{
  // this = T(toOrigin) * [toE] * [fromE]^-1 * T(-fromOrigin).
  NvGeMatrix3d aFrom;
  aFrom.setCoordSystem (NvGePoint3d::kOrigin, theFromE0, theFromE1, theFromE2);
  if (aFrom.isSingular())
  {
    throw NvException ("NvGeMatrix3d::setToAlignCoordSys(): the source coordinate system is degenerate");
  }
  NvGeMatrix3d aTo;
  aTo.setCoordSystem (NvGePoint3d::kOrigin, theToE0, theToE1, theToE2);
  setToProduct (aTo, aFrom.inverse());
  entry[0][3] = theToOrigin.x - (entry[0][0] * theFromOrigin.x
                               + entry[0][1] * theFromOrigin.y
                               + entry[0][2] * theFromOrigin.z);
  entry[1][3] = theToOrigin.y - (entry[1][0] * theFromOrigin.x
                               + entry[1][1] * theFromOrigin.y
                               + entry[1][2] * theFromOrigin.z);
  entry[2][3] = theToOrigin.z - (entry[2][0] * theFromOrigin.x
                               + entry[2][1] * theFromOrigin.y
                               + entry[2][2] * theFromOrigin.z);
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToWorldToPlane (const NvGeVector3d& theNormal)
{
  // Rows are an orthonormal basis built from the normal: world coordinates
  // land in the plane's local (u, v, n) frame, origin unchanged.
  double anUx, anUy, anUz, aVx, aVy, aVz;
  BasisFromNormal (theNormal.x, theNormal.y, theNormal.z, anUx, anUy, anUz, aVx, aVy, aVz);
  const double aNx = anUy * aVz - anUz * aVy;
  const double aNy = anUz * aVx - anUx * aVz;
  const double aNz = anUx * aVy - anUy * aVx;
  setToIdentity();
  entry[0][0] = anUx; entry[0][1] = anUy; entry[0][2] = anUz;
  entry[1][0] = aVx;  entry[1][1] = aVy;  entry[1][2] = aVz;
  entry[2][0] = aNx;  entry[2][1] = aNy;  entry[2][2] = aNz;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToWorldToPlane (const NvGePlane& thePlane)
{
  NvGePoint3d anOrigin;
  NvGeVector3d anU, aV;
  thePlane.getCoordSystem (anOrigin, anU, aV);
  NvGeVector3d aN (anU.y * aV.z - anU.z * aV.y,
                   anU.z * aV.x - anU.x * aV.z,
                   anU.x * aV.y - anU.y * aV.x);
  if (aN.length() <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix3d::setToWorldToPlane(): the plane is degenerate");
  }
  setToIdentity();
  entry[0][0] = anU.x; entry[0][1] = anU.y; entry[0][2] = anU.z;
  entry[1][0] = aV.x;  entry[1][1] = aV.y;  entry[1][2] = aV.z;
  entry[2][0] = aN.x;  entry[2][1] = aN.y;  entry[2][2] = aN.z;
  double anOx = anOrigin.x;
  double anOy = anOrigin.y;
  double anOz = anOrigin.z;
  ApplyLinear (*this, anOx, anOy, anOz);
  entry[0][3] = -anOx;
  entry[1][3] = -anOy;
  entry[2][3] = -anOz;
  return *this;
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToPlaneToWorld (const NvGeVector3d& theNormal)
{
  // Inverse of the world-to-plane frame: basis vectors become columns.
  setToWorldToPlane (theNormal);
  return transposeIt();
}

//=================================================================================================

NvGeMatrix3d& NvGeMatrix3d::setToPlaneToWorld (const NvGePlane& thePlane)
{
  setToWorldToPlane (thePlane);
  NvGeMatrix3d aToWorld = this->transpose();
  *this = aToWorld;
  // Restore the origin column (the transpose cleared the translation).
  NvGePoint3d anOrigin;
  NvGeVector3d anU, aV;
  thePlane.getCoordSystem (anOrigin, anU, aV);
  entry[0][3] = anOrigin.x;
  entry[1][3] = anOrigin.y;
  entry[2][3] = anOrigin.z;
  return *this;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::translation (const NvGeVector3d& theVec)
{
  NvGeMatrix3d aResult;
  aResult.setToTranslation (theVec);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::rotation (double theAngle, const NvGeVector3d& theAxis,
                                     const NvGePoint3d& theCenter)
{
  NvGeMatrix3d aResult;
  aResult.setToRotation (theAngle, theAxis, theCenter);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::scaling (double theScaleAll, const NvGePoint3d& theCenter)
{
  NvGeMatrix3d aResult;
  aResult.setToScaling (theScaleAll, theCenter);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::mirroring (const NvGePlane& thePlane)
{
  NvGeMatrix3d aResult;
  aResult.setToMirroring (thePlane);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::mirroring (const NvGePoint3d& thePnt)
{
  NvGeMatrix3d aResult;
  aResult.setToMirroring (thePnt);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::mirroring (const NvGeLine3d& theLine)
{
  NvGeMatrix3d aResult;
  aResult.setToMirroring (theLine);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::projection (const NvGePlane& thePlane, const NvGeVector3d& theProjDir)
{
  NvGeMatrix3d aResult;
  aResult.setToProjection (thePlane, theProjDir);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::alignCoordSys (const NvGePoint3d& theFromOrigin,
                                          const NvGeVector3d& theFromE0,
                                          const NvGeVector3d& theFromE1,
                                          const NvGeVector3d& theFromE2,
                                          const NvGePoint3d& theToOrigin,
                                          const NvGeVector3d& theToE0,
                                          const NvGeVector3d& theToE1,
                                          const NvGeVector3d& theToE2)
{
  NvGeMatrix3d aResult;
  aResult.setToAlignCoordSys (theFromOrigin, theFromE0, theFromE1, theFromE2,
                              theToOrigin,   theToE0,   theToE1,   theToE2);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::worldToPlane (const NvGeVector3d& theNormal)
{
  NvGeMatrix3d aResult;
  aResult.setToWorldToPlane (theNormal);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::worldToPlane (const NvGePlane& thePlane)
{
  NvGeMatrix3d aResult;
  aResult.setToWorldToPlane (thePlane);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::planeToWorld (const NvGeVector3d& theNormal)
{
  NvGeMatrix3d aResult;
  aResult.setToPlaneToWorld (theNormal);
  return aResult;
}

//=================================================================================================

NvGeMatrix3d NvGeMatrix3d::planeToWorld (const NvGePlane& thePlane)
{
  NvGeMatrix3d aResult;
  aResult.setToPlaneToWorld (thePlane);
  return aResult;
}

//=================================================================================================

double NvGeMatrix3d::scale() const
{
  // Norm of the first column vector; positive for uni-scaled ortho matrices.
  return std::sqrt (entry[0][0] * entry[0][0]
                  + entry[1][0] * entry[1][0]
                  + entry[2][0] * entry[2][0]);
}

//=================================================================================================

double NvGeMatrix3d::norm() const
{
  // Frobenius norm of the linear part.
  double aSum = 0.0;
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      aSum += entry[aRow][aCol] * entry[aRow][aCol];
    }
  }
  return std::sqrt (aSum);
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix3d::convertToLocal (NvGeVector3d& theNormal, double& theErr) const
{
  // Restrict the transform to the coordinate plane it preserves best: for
  // each candidate normal axis, express the images of the two in-plane basis
  // vectors in the local frame; the error is the out-of-plane leakage.
  double aBestErr = std::numeric_limits<double>::infinity();
  int aBestAxis = 0;
  double aBestU[3] = { 0.0, 0.0, 0.0 };
  double aBestV[3] = { 0.0, 0.0, 0.0 };
  double aBestGU[3] = { 0.0, 0.0, 0.0 };
  double aBestGV[3] = { 0.0, 0.0, 0.0 };
  for (int anAxis = 0; anAxis < 3; ++anAxis)
  {
    double anNx = 0.0, anNy = 0.0, anNz = 0.0;
    if (anAxis == 0) { anNx = 1.0; }
    if (anAxis == 1) { anNy = 1.0; }
    if (anAxis == 2) { anNz = 1.0; }
    double anUx, anUy, anUz, aVx, aVy, aVz;
    BasisFromNormal (anNx, anNy, anNz, anUx, anUy, anUz, aVx, aVy, aVz);
    double aGUx = anUx, aGUy = anUy, aGUz = anUz;
    double aGVx = aVx, aGVy = aVy, aGVz = aVz;
    ApplyLinear (*this, aGUx, aGUy, aGUz);
    ApplyLinear (*this, aGVx, aGVy, aGVz);
    const double anErrU = std::abs (aGUx * anNx + aGUy * anNy + aGUz * anNz);
    const double anErrV = std::abs (aGVx * anNx + aGVy * anNy + aGVz * anNz);
    const double anErr = anErrU > anErrV ? anErrU : anErrV;
    if (anErr < aBestErr)
    {
      aBestErr = anErr;
      aBestAxis = anAxis;
      aBestU[0] = anUx; aBestU[1] = anUy; aBestU[2] = anUz;
      aBestV[0] = aVx;  aBestV[1] = aVy;  aBestV[2] = aVz;
      aBestGU[0] = aGUx; aBestGU[1] = aGUy; aBestGU[2] = aGUz;
      aBestGV[0] = aGVx; aBestGV[1] = aGVy; aBestGV[2] = aGVz;
    }
  }
  theNormal.set (aBestAxis == 0 ? 1.0 : 0.0, aBestAxis == 1 ? 1.0 : 0.0,
                 aBestAxis == 2 ? 1.0 : 0.0);
  theErr = aBestErr;
  NvGeMatrix2d aLocal;
  aLocal.entry[0][0] = aBestGU[0] * aBestU[0] + aBestGU[1] * aBestU[1] + aBestGU[2] * aBestU[2];
  aLocal.entry[1][0] = aBestGU[0] * aBestV[0] + aBestGU[1] * aBestV[1] + aBestGU[2] * aBestV[2];
  aLocal.entry[0][1] = aBestGV[0] * aBestU[0] + aBestGV[1] * aBestU[1] + aBestGV[2] * aBestU[2];
  aLocal.entry[1][1] = aBestGV[0] * aBestV[0] + aBestGV[1] * aBestV[1] + aBestGV[2] * aBestV[2];
  aLocal.entry[0][2] = entry[0][3] * aBestU[0] + entry[1][3] * aBestU[1] + entry[2][3] * aBestU[2];
  aLocal.entry[1][2] = entry[0][3] * aBestV[0] + entry[1][3] * aBestV[1] + entry[2][3] * aBestV[2];
  aLocal.entry[2][0] = 0.0; aLocal.entry[2][1] = 0.0; aLocal.entry[2][2] = 1.0;
  return aLocal;
}

//=================================================================================================

Adesk::Boolean NvGeMatrix3d::inverse (NvGeMatrix3d& theInvMat, double theTol) const
{
  if (std::abs (det()) <= theTol)
  {
    return false;
  }
  theInvMat = inverse();
  return true;
}
