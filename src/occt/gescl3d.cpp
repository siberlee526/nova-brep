// gescl3d.cpp - implementation of NvGeScale3d.

#include <gescl3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gemat3d.h>
#include <getol.h>

#include <gp.hxx>

#include <cmath>

//=================================================================================================

NvGeScale3d::NvGeScale3d()
: sx (1.0),
  sy (1.0),
  sz (1.0)
{
}

//=================================================================================================

NvGeScale3d::NvGeScale3d (const NvGeScale3d& theSrc)
: sx (theSrc.sx),
  sy (theSrc.sy),
  sz (theSrc.sz)
{
}

//=================================================================================================

NvGeScale3d::NvGeScale3d (double theFactor)
: sx (theFactor),
  sy (theFactor),
  sz (theFactor)
{
}

//=================================================================================================

NvGeScale3d::NvGeScale3d (double theXFactor, double theYFactor, double theZFactor)
: sx (theXFactor),
  sy (theYFactor),
  sz (theZFactor)
{
}

// The default constructor builds the identity scaling.
const NvGeScale3d NvGeScale3d::kIdentity;

//=================================================================================================

NvGeScale3d NvGeScale3d::operator * (const NvGeScale3d& theSclVec) const
{
  return NvGeScale3d (sx * theSclVec.sx, sy * theSclVec.sy, sz * theSclVec.sz);
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::operator *= (const NvGeScale3d& theScl)
{
  sx *= theScl.sx;
  sy *= theScl.sy;
  sz *= theScl.sz;
  return *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::preMultBy (const NvGeScale3d& theLeftSide)
{
  sx = theLeftSide.sx * sx;
  sy = theLeftSide.sy * sy;
  sz = theLeftSide.sz * sz;
  return *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::postMultBy (const NvGeScale3d& theRightSide)
{
  sx *= theRightSide.sx;
  sy *= theRightSide.sy;
  sz *= theRightSide.sz;
  return *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::setToProduct (const NvGeScale3d& theSclVec1, const NvGeScale3d& theSclVec2)
{
  sx = theSclVec1.sx * theSclVec2.sx;
  sy = theSclVec1.sy * theSclVec2.sy;
  sz = theSclVec1.sz * theSclVec2.sz;
  return *this;
}

//=================================================================================================

NvGeScale3d NvGeScale3d::operator * (double theS) const
{
  return NvGeScale3d (sx * theS, sy * theS, sz * theS);
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::operator *= (double theS)
{
  sx *= theS;
  sy *= theS;
  sz *= theS;
  return *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::setToProduct (const NvGeScale3d& theSclVec, double theS)
{
  sx = theSclVec.sx * theS;
  sy = theSclVec.sy * theS;
  sz = theSclVec.sz * theS;
  return *this;
}

//=================================================================================================

NvGeScale3d operator * (double theS, const NvGeScale3d& theScl)
{
  return NvGeScale3d (theS * theScl.sx, theS * theScl.sy, theS * theScl.sz);
}

//=================================================================================================

NvGeScale3d NvGeScale3d::inverse() const
{
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution()
   || std::abs (sz) <= gp::Resolution())
  {
    throw NvException ("NvGeScale3d::inverse(): a scale component is zero");
  }
  return NvGeScale3d (1.0 / sx, 1.0 / sy, 1.0 / sz);
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::invert()
{
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution()
   || std::abs (sz) <= gp::Resolution())
  {
    throw NvException ("NvGeScale3d::invert(): a scale component is zero");
  }
  sx = 1.0 / sx;
  sy = 1.0 / sy;
  sz = 1.0 / sz;
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeScale3d::isProportional (const NvGeTol& theTol) const
{
  return std::abs (std::abs (sx) - std::abs (sy)) <= theTol.equalVector()
      && std::abs (std::abs (sx) - std::abs (sz)) <= theTol.equalVector();
}

//=================================================================================================

bool NvGeScale3d::isEqualTo (const NvGeScale3d& theScaleVec, const NvGeTol& theTol) const
{
  return std::abs (sx - theScaleVec.sx) <= theTol.equalPoint()
      && std::abs (sy - theScaleVec.sy) <= theTol.equalPoint()
      && std::abs (sz - theScaleVec.sz) <= theTol.equalPoint();
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::set (double theSc0, double theSc1, double theSc2)
{
  sx = theSc0;
  sy = theSc1;
  sz = theSc2;
  return *this;
}

//=================================================================================================

NvGeScale3d::operator NvGeMatrix3d() const
{
  NvGeMatrix3d aMatrix;
  aMatrix.entry[0][0] = sx;
  aMatrix.entry[1][1] = sy;
  aMatrix.entry[2][2] = sz;
  return aMatrix;
}

//=================================================================================================

void NvGeScale3d::getMatrix (NvGeMatrix3d& theMat) const
{
  theMat = *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::extractScale (const NvGeMatrix3d& theMat)
{
  // Column norms of a scaled-orthogonal linear part.
  sx = std::sqrt (theMat (0, 0) * theMat (0, 0) + theMat (1, 0) * theMat (1, 0)
                + theMat (2, 0) * theMat (2, 0));
  sy = std::sqrt (theMat (0, 1) * theMat (0, 1) + theMat (1, 1) * theMat (1, 1)
                + theMat (2, 1) * theMat (2, 1));
  sz = std::sqrt (theMat (0, 2) * theMat (0, 2) + theMat (1, 2) * theMat (1, 2)
                + theMat (2, 2) * theMat (2, 2));
  return *this;
}

//=================================================================================================

NvGeScale3d& NvGeScale3d::removeScale (NvGeMatrix3d& theMat)
{
  extractScale (theMat);
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution()
   || std::abs (sz) <= gp::Resolution())
  {
    throw NvException ("NvGeScale3d::removeScale(): a matrix column is degenerate");
  }
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    theMat.entry[aRow][0] /= sx;
    theMat.entry[aRow][1] /= sy;
    theMat.entry[aRow][2] /= sz;
  }
  return *this;
}
