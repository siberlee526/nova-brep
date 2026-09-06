// gemat2d.cpp - implementation of NvGeMatrix2d based on OCCT gp classes.
//
// The matrix is a homogeneous 3x3 transform in the column-vector convention:
//
//   | entry[0][0] entry[0][1] entry[0][2] |   | x |   | x' |
//   | entry[1][0] entry[1][1] entry[1][2] | * | y | = | y' |
//   | entry[2][0] entry[2][1] entry[2][2] |   | 1 |   | 1  |
//
// so the product (A * B) applied to a point equals A applied to the result
// of B applied to the point. Elementary transformations (translation,
// rotation, scaling, mirroring) are built with gp_Trsf2d, which represents
// them exactly; general matrix algebra works on the full 3x3 entries with
// gp_Mat2d / gp_XY handling the linear part.

#include <gemat2d.h>

#include <NvException.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gp.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Mat2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf2d.hxx>
#include <gp_Vec2d.hxx>
#include <gp_XY.hxx>

#include <cmath>

namespace
{

// Copies the coefficients of an elementary OCCT transformation into the
// homogeneous entries of the matrix. gp_Trsf2d::Value() already includes
// the scale factor, so the copy is exact.
void SetMatrixFromTrsf2d (NvGeMatrix2d& theMatrix, const gp_Trsf2d& theTrsf)
{
  for (int aRow = 0; aRow < 2; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      theMatrix.entry[aRow][aCol] = theTrsf.Value (aRow + 1, aCol + 1);
    }
  }
  theMatrix.entry[2][0] = 0.0;
  theMatrix.entry[2][1] = 0.0;
  theMatrix.entry[2][2] = 1.0;
}

// Returns the 2x2 linear part of the matrix (columns are the images of the
// x-axis and y-axis basis vectors).
gp_Mat2d LinearPart (const NvGeMatrix2d& theMatrix)
{
  return gp_Mat2d (gp_XY (theMatrix.entry[0][0], theMatrix.entry[1][0]),
                   gp_XY (theMatrix.entry[0][1], theMatrix.entry[1][1]));
}

// Full 3x3 homogeneous product of two matrices. No assumption is made on
// the bottom row, so matrices with externally modified entries still
// multiply correctly.
NvGeMatrix2d ProductOf (const NvGeMatrix2d& theLeft, const NvGeMatrix2d& theRight)
{
  NvGeMatrix2d aResult;
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      double aValue = 0.0;
      for (int aTerm = 0; aTerm < 3; ++aTerm)
      {
        aValue += theLeft.entry[aRow][aTerm] * theRight.entry[aTerm][aCol];
      }
      aResult.entry[aRow][aCol] = aValue;
    }
  }
  return aResult;
}

}

//=================================================================================================

NvGeMatrix2d::NvGeMatrix2d()
{
  setToIdentity();
}

//=================================================================================================

NvGeMatrix2d::NvGeMatrix2d (const NvGeMatrix2d& theSrc)
{
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      entry[aRow][aCol] = theSrc.entry[aRow][aCol];
    }
  }
}

// The default constructor builds the identity matrix, so the constant is
// simply a default-constructed instance.
const NvGeMatrix2d NvGeMatrix2d::kIdentity;

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToIdentity()
{
  entry[0][0] = 1.0; entry[0][1] = 0.0; entry[0][2] = 0.0;
  entry[1][0] = 0.0; entry[1][1] = 1.0; entry[1][2] = 0.0;
  entry[2][0] = 0.0; entry[2][1] = 0.0; entry[2][2] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::operator * (const NvGeMatrix2d& theMat) const
{
  return ProductOf (*this, theMat);
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::operator *= (const NvGeMatrix2d& theMat)
{
  *this = ProductOf (*this, theMat);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::preMultBy (const NvGeMatrix2d& theLeftSide)
{
  *this = ProductOf (theLeftSide, *this);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::postMultBy (const NvGeMatrix2d& theRightSide)
{
  *this = ProductOf (*this, theRightSide);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToProduct (const NvGeMatrix2d& theMat1, const NvGeMatrix2d& theMat2)
{
  *this = ProductOf (theMat1, theMat2);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::invert()
{
  // Adjugate inverse of the full 3x3 matrix; for the affine matrices
  // produced by this class it coincides with inverting the 2x2 linear part
  // and remapping the translation column.
  const double a00 = entry[0][0]; const double a01 = entry[0][1]; const double a02 = entry[0][2];
  const double a10 = entry[1][0]; const double a11 = entry[1][1]; const double a12 = entry[1][2];
  const double a20 = entry[2][0]; const double a21 = entry[2][1]; const double a22 = entry[2][2];

  const double aCof00 =  a11 * a22 - a12 * a21;
  const double aCof01 = -(a10 * a22 - a12 * a20);
  const double aCof02 =  a10 * a21 - a11 * a20;
  const double aCof10 = -(a01 * a22 - a02 * a21);
  const double aCof11 =  a00 * a22 - a02 * a20;
  const double aCof12 = -(a00 * a21 - a01 * a20);
  const double aCof20 =  a01 * a12 - a02 * a11;
  const double aCof21 = -(a00 * a12 - a02 * a10);
  const double aCof22 =  a00 * a11 - a01 * a10;

  const double aDet = a00 * aCof00 + a01 * aCof01 + a02 * aCof02;
  if (std::abs (aDet) <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix2d::invert(): the matrix is singular and cannot be inverted");
  }

  // inverse = transposed cofactor matrix / determinant
  entry[0][0] = aCof00 / aDet; entry[0][1] = aCof10 / aDet; entry[0][2] = aCof20 / aDet;
  entry[1][0] = aCof01 / aDet; entry[1][1] = aCof11 / aDet; entry[1][2] = aCof21 / aDet;
  entry[2][0] = aCof02 / aDet; entry[2][1] = aCof12 / aDet; entry[2][2] = aCof22 / aDet;
  return *this;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::inverse() const
{
  NvGeMatrix2d aResult (*this);
  aResult.invert();
  return aResult;
}

//=================================================================================================

Nova::Boolean NvGeMatrix2d::isSingular (const NvGeTol& theTol) const
{
  // An affine matrix is invertible exactly when its linear part is.
  return std::abs (det()) <= theTol.equalPoint();
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::transposeIt()
{
  // Full 3x3 transposition; the translation column participates like any
  // other column, so it moves into the bottom row.
  const double aTmp01 = entry[0][1]; entry[0][1] = entry[1][0]; entry[1][0] = aTmp01;
  const double aTmp02 = entry[0][2]; entry[0][2] = entry[2][0]; entry[2][0] = aTmp02;
  const double aTmp12 = entry[1][2]; entry[1][2] = entry[2][1]; entry[2][1] = aTmp12;
  return *this;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::transpose() const
{
  NvGeMatrix2d aResult (*this);
  aResult.transposeIt();
  return aResult;
}

//=================================================================================================

bool NvGeMatrix2d::isEqualTo (const NvGeMatrix2d& theMat, const NvGeTol& theTol) const
{
  // Infinity norm: the largest absolute difference over all entry pairs.
  double aMaxDiff = 0.0;
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      const double aDiff = std::abs (entry[aRow][aCol] - theMat.entry[aRow][aCol]);
      if (aDiff > aMaxDiff)
      {
        aMaxDiff = aDiff;
      }
    }
  }
  return aMaxDiff <= theTol.equalPoint();
}

//=================================================================================================

Nova::Boolean NvGeMatrix2d::isUniScaledOrtho (const NvGeTol& theTol) const
{
  const gp_Mat2d aLinear = LinearPart (*this);
  const gp_XY    aCol0   = aLinear.Column (1);
  const gp_XY    aCol1   = aLinear.Column (2);
  const double   aTolVec = theTol.equalVector();

  if (aCol0.Modulus() <= aTolVec)
  {
    return false;
  }
  // Columns must be orthogonal and of equal length (mirror allowed).
  return std::abs (aCol0.Dot (aCol1)) <= aTolVec
      && std::abs (aCol0.Modulus() - aCol1.Modulus()) <= aTolVec;
}

//=================================================================================================

Nova::Boolean NvGeMatrix2d::isScaledOrtho (const NvGeTol& theTol) const
{
  const gp_Mat2d aLinear = LinearPart (*this);
  const gp_XY    aCol0   = aLinear.Column (1);
  const gp_XY    aCol1   = aLinear.Column (2);
  const double   aTolVec = theTol.equalVector();

  if (aCol0.Modulus() <= aTolVec || aCol1.Modulus() <= aTolVec)
  {
    return false;
  }
  // Columns must be orthogonal, but may have different lengths.
  return std::abs (aCol0.Dot (aCol1)) <= aTolVec;
}

//=================================================================================================

double NvGeMatrix2d::scale()
{
  // Norm of the first column vector; meaningful for uni-scaled orthogonal
  // matrices, where it is the (positive) scale factor.
  return LinearPart (*this).Column (1).Modulus();
}

//=================================================================================================

double NvGeMatrix2d::det() const
{
  return LinearPart (*this).Determinant();
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setTranslation (const NvGeVector2d& theVec)
{
  // Only the translation column is replaced; the linear part is kept.
  entry[0][2] = theVec.x;
  entry[1][2] = theVec.y;
  return *this;
}

//=================================================================================================

NvGeVector2d NvGeMatrix2d::translation() const
{
  return NvGeVector2d (entry[0][2], entry[1][2]);
}

//=================================================================================================

Nova::Boolean NvGeMatrix2d::isConformal (double& theScale, double& theAngle,
                                          Nova::Boolean& theIsMirror, NvGeVector2d& theReflex) const
{
  if (!isUniScaledOrtho (NvGeContext::gTol))
  {
    return false;
  }

  const gp_Mat2d aLinear = LinearPart (*this);
  theScale = aLinear.Column (1).Modulus();
  if (aLinear.Determinant() >= 0.0)
  {
    // Rotation (possibly identity): the angle of the first column vector.
    theAngle   = std::atan2 (aLinear.Value (2, 1), aLinear.Value (1, 1));
    theIsMirror = false;
    theReflex   = NvGeVector2d (1.0, 0.0);
  }
  else
  {
    // A 2x2 orthogonal matrix with negative determinant is symmetric, so it
    // is a pure reflection about the line through the origin at half the
    // argument of its first row; no residual rotation remains.
    const double aLineAngle = 0.5 * std::atan2 (aLinear.Value (1, 2), aLinear.Value (1, 1));
    theAngle    = 0.0;
    theIsMirror = true;
    theReflex   = NvGeVector2d (std::cos (aLineAngle), std::sin (aLineAngle));
  }
  return true;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setCoordSystem (const NvGePoint2d& theOrigin,
                                            const NvGeVector2d& theE0,
                                            const NvGeVector2d& theE1)
{
  // The basis vectors become the columns of the linear part; the matrix
  // maps local coordinates to world coordinates.
  const gp_Mat2d aBasis (gp_XY (theE0.x, theE0.y), gp_XY (theE1.x, theE1.y));
  entry[0][0] = aBasis.Value (1, 1); entry[0][1] = aBasis.Value (1, 2);
  entry[1][0] = aBasis.Value (2, 1); entry[1][1] = aBasis.Value (2, 2);
  entry[0][2] = theOrigin.x;         entry[1][2] = theOrigin.y;
  entry[2][0] = 0.0;                 entry[2][1] = 0.0;                 entry[2][2] = 1.0;
  return *this;
}

//=================================================================================================

void NvGeMatrix2d::getCoordSystem (NvGePoint2d& theOrigin, NvGeVector2d& theE0, NvGeVector2d& theE1) const
{
  const gp_Mat2d aLinear = LinearPart (*this);
  const gp_XY    aCol0   = aLinear.Column (1);
  const gp_XY    aCol1   = aLinear.Column (2);
  theOrigin.set (entry[0][2], entry[1][2]);
  theE0.set (aCol0.X(), aCol0.Y());
  theE1.set (aCol1.X(), aCol1.Y());
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToTranslation (const NvGeVector2d& theVec)
{
  gp_Trsf2d aTrsf;
  aTrsf.SetTranslation (gp_Vec2d (theVec.x, theVec.y));
  SetMatrixFromTrsf2d (*this, aTrsf);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToRotation (double theAngle, const NvGePoint2d& theCenter)
{
  gp_Trsf2d aTrsf;
  aTrsf.SetRotation (gp_Pnt2d (theCenter.x, theCenter.y), theAngle);
  SetMatrixFromTrsf2d (*this, aTrsf);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToScaling (double theScaleAll, const NvGePoint2d& theCenter)
{
  gp_Trsf2d aTrsf;
  aTrsf.SetScale (gp_Pnt2d (theCenter.x, theCenter.y), theScaleAll);
  SetMatrixFromTrsf2d (*this, aTrsf);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToMirroring (const NvGePoint2d& thePnt)
{
  gp_Trsf2d aTrsf;
  aTrsf.SetMirror (gp_Pnt2d (thePnt.x, thePnt.y));
  SetMatrixFromTrsf2d (*this, aTrsf);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToMirroring (const NvGeLine2d& theLine)
{
  const NvGePoint2d  aPnt = theLine.pointOnLine();
  const NvGeVector2d aDir = theLine.direction();

  // A degenerate direction would make gp_Dir2d raise Standard_ConstructionError;
  // reject it up front with the project error type instead.
  const double aDirLen = std::sqrt (aDir.lengthSqrd());
  if (aDirLen <= gp::Resolution())
  {
    throw NvException ("NvGeMatrix2d::setToMirroring(): the line has a degenerate direction");
  }

  gp_Trsf2d aTrsf;
  aTrsf.SetMirror (gp_Ax2d (gp_Pnt2d (aPnt.x, aPnt.y), gp_Dir2d (gp_XY (aDir.x, aDir.y))));
  SetMatrixFromTrsf2d (*this, aTrsf);
  return *this;
}

//=================================================================================================

NvGeMatrix2d& NvGeMatrix2d::setToAlignCoordSys (const NvGePoint2d&  theFromOrigin,
                                                const NvGeVector2d& theFromE0,
                                                const NvGeVector2d& theFromE1,
                                                const NvGePoint2d&  theToOrigin,
                                                const NvGeVector2d& theToE0,
                                                const NvGeVector2d& theToE1)
{
  // this = T(toOrigin) * [toE0 toE1] * [fromE0 fromE1]^-1 * T(-fromOrigin):
  // a point keeps its local coordinates when passing from the "from"
  // coordinate system to the "to" coordinate system.
  const gp_Mat2d aFromBasis (gp_XY (theFromE0.x, theFromE0.y),
                             gp_XY (theFromE1.x, theFromE1.y));
  if (aFromBasis.IsSingular())
  {
    throw NvException ("NvGeMatrix2d::setToAlignCoordSys(): the source coordinate system is degenerate");
  }

  const gp_Mat2d aToBasis (gp_XY (theToE0.x, theToE0.y),
                           gp_XY (theToE1.x, theToE1.y));
  const gp_Mat2d aLinear = aToBasis.Multiplied (aFromBasis.Inverted());
  const gp_XY    aShift  = gp_XY (theToOrigin.x, theToOrigin.y).Subtracted (
                           gp_XY (theFromOrigin.x, theFromOrigin.y).Multiplied (aLinear));

  entry[0][0] = aLinear.Value (1, 1); entry[0][1] = aLinear.Value (1, 2);
  entry[1][0] = aLinear.Value (2, 1); entry[1][1] = aLinear.Value (2, 2);
  entry[0][2] = aShift.X();           entry[1][2] = aShift.Y();
  entry[2][0] = 0.0;                  entry[2][1] = 0.0;               entry[2][2] = 1.0;
  return *this;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::translation (const NvGeVector2d& theVec)
{
  NvGeMatrix2d aResult;
  aResult.setToTranslation (theVec);
  return aResult;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::rotation (double theAngle, const NvGePoint2d& theCenter)
{
  NvGeMatrix2d aResult;
  aResult.setToRotation (theAngle, theCenter);
  return aResult;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::scaling (double theScaleAll, const NvGePoint2d& theCenter)
{
  NvGeMatrix2d aResult;
  aResult.setToScaling (theScaleAll, theCenter);
  return aResult;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::mirroring (const NvGePoint2d& thePnt)
{
  NvGeMatrix2d aResult;
  aResult.setToMirroring (thePnt);
  return aResult;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::mirroring (const NvGeLine2d& theLine)
{
  NvGeMatrix2d aResult;
  aResult.setToMirroring (theLine);
  return aResult;
}

//=================================================================================================

NvGeMatrix2d NvGeMatrix2d::alignCoordSys (const NvGePoint2d&  theFromOrigin,
                                          const NvGeVector2d& theFromE0,
                                          const NvGeVector2d& theFromE1,
                                          const NvGePoint2d&  theToOrigin,
                                          const NvGeVector2d& theToE0,
                                          const NvGeVector2d& theToE1)
{
  NvGeMatrix2d aResult;
  aResult.setToAlignCoordSys (theFromOrigin, theFromE0, theFromE1,
                              theToOrigin,   theToE0,   theToE1);
  return aResult;
}
