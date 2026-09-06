// gescl2d.cpp - implementation of NvGeScale2d.

#include <gescl2d.h>

#include <Nova.h>
#include <NvException.h>
#include <gemat2d.h>
#include <gescl3d.h>
#include <getol.h>

#include <gp.hxx>

#include <cmath>

//=================================================================================================

NvGeScale2d::NvGeScale2d()
: sx (1.0),
  sy (1.0)
{
}

//=================================================================================================

NvGeScale2d::NvGeScale2d (const NvGeScale2d& theSrc)
: sx (theSrc.sx),
  sy (theSrc.sy)
{
}

//=================================================================================================

NvGeScale2d::NvGeScale2d (double theFactor)
: sx (theFactor),
  sy (theFactor)
{
}

//=================================================================================================

NvGeScale2d::NvGeScale2d (double theXFactor, double theYFactor)
: sx (theXFactor),
  sy (theYFactor)
{
}

// The default constructor builds the identity scaling.
const NvGeScale2d NvGeScale2d::kIdentity;

//=================================================================================================

NvGeScale2d NvGeScale2d::operator * (const NvGeScale2d& theSclVec) const
{
  return NvGeScale2d (sx * theSclVec.sx, sy * theSclVec.sy);
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::operator *= (const NvGeScale2d& theScl)
{
  sx *= theScl.sx;
  sy *= theScl.sy;
  return *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::preMultBy (const NvGeScale2d& theLeftSide)
{
  sx = theLeftSide.sx * sx;
  sy = theLeftSide.sy * sy;
  return *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::postMultBy (const NvGeScale2d& theRightSide)
{
  sx *= theRightSide.sx;
  sy *= theRightSide.sy;
  return *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::setToProduct (const NvGeScale2d& theSclVec1, const NvGeScale2d& theSclVec2)
{
  sx = theSclVec1.sx * theSclVec2.sx;
  sy = theSclVec1.sy * theSclVec2.sy;
  return *this;
}

//=================================================================================================

NvGeScale2d NvGeScale2d::operator * (double theS) const
{
  return NvGeScale2d (sx * theS, sy * theS);
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::operator *= (double theS)
{
  sx *= theS;
  sy *= theS;
  return *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::setToProduct (const NvGeScale2d& theSclVec, double theS)
{
  sx = theSclVec.sx * theS;
  sy = theSclVec.sy * theS;
  return *this;
}

//=================================================================================================

NvGeScale2d operator * (double theS, const NvGeScale2d& theScl)
{
  return NvGeScale2d (theS * theScl.sx, theS * theScl.sy);
}

//=================================================================================================

NvGeScale2d NvGeScale2d::inverse() const
{
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution())
  {
    throw NvException ("NvGeScale2d::inverse(): a scale component is zero");
  }
  return NvGeScale2d (1.0 / sx, 1.0 / sy);
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::invert()
{
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution())
  {
    throw NvException ("NvGeScale2d::invert(): a scale component is zero");
  }
  sx = 1.0 / sx;
  sy = 1.0 / sy;
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeScale2d::isProportional (const NvGeTol& theTol) const
{
  return std::abs (std::abs (sx) - std::abs (sy)) <= theTol.equalVector();
}

//=================================================================================================

bool NvGeScale2d::isEqualTo (const NvGeScale2d& theScaleVec, const NvGeTol& theTol) const
{
  return std::abs (sx - theScaleVec.sx) <= theTol.equalPoint()
      && std::abs (sy - theScaleVec.sy) <= theTol.equalPoint();
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::set (double theSc0, double theSc1)
{
  sx = theSc0;
  sy = theSc1;
  return *this;
}

//=================================================================================================

NvGeScale2d::operator NvGeMatrix2d() const
{
  NvGeMatrix2d aMatrix;
  aMatrix.entry[0][0] = sx;
  aMatrix.entry[1][1] = sy;
  return aMatrix;
}

//=================================================================================================

void NvGeScale2d::getMatrix (NvGeMatrix2d& theMat) const
{
  theMat = *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::extractScale (const NvGeMatrix2d& theMat)
{
  // Column norms of a scaled-orthogonal linear part.
  sx = std::sqrt (theMat (0, 0) * theMat (0, 0) + theMat (1, 0) * theMat (1, 0));
  sy = std::sqrt (theMat (0, 1) * theMat (0, 1) + theMat (1, 1) * theMat (1, 1));
  return *this;
}

//=================================================================================================

NvGeScale2d& NvGeScale2d::removeScale (NvGeMatrix2d& theMat)
{
  extractScale (theMat);
  if (std::abs (sx) <= gp::Resolution() || std::abs (sy) <= gp::Resolution())
  {
    throw NvException ("NvGeScale2d::removeScale(): a matrix column is degenerate");
  }
  theMat.entry[0][0] /= sx;
  theMat.entry[1][0] /= sx;
  theMat.entry[0][1] /= sy;
  theMat.entry[1][1] /= sy;
  return *this;
}

//=================================================================================================

NvGeScale2d::operator NvGeScale3d() const
{
  return NvGeScale3d (sx, sy, 1.0);
}
