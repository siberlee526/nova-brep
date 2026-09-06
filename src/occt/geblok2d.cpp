// geblok2d.cpp - implementation of NvGeBoundBlock2d.
//
// A bound block is a parallelogram spanned by two vectors from a base
// point; a zero vector disables the bound in that direction (unbounded
// side, ARX semantics). Storage lives in the NvGeBoundBlock2dData holder
// from geimpdata.h; the axis-aligned box is the special case where the
// vectors point along the coordinate axes.

#include <geblok2d.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <cmath>

namespace
{

//! Holder of this block, with the COW discipline applied.
NvGeBoundBlock2dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeBoundBlock2dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeBoundBlock2dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeBoundBlock2dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeBoundBlock2dData> (theImp->Geom());
}

//! Installs a fresh holder, replacing the base placeholder impl.
void MakeBlock (NvGeImpEntity3d*& theImp, const gp_Pnt2d& thePoint,
                const gp_Vec2d& theVec1, const gp_Vec2d& theVec2)
{
  if (theImp != nullptr && theImp->RefCount() > 0)
  {
    // placeholder from the base constructor: release the adoption ref
    theImp->Unref();
  }
  occ::handle<NvGeBoundBlock2dData> aData = new NvGeBoundBlock2dData();
  aData->Point = thePoint;
  aData->Vec1 = theVec1;
  aData->Vec2 = theVec2;
  theImp = new NvGeImpEntity3d (NvGe::kBoundBlock2d, aData);
  theImp->Ref();
}

//! The four corners of the parallelogram.
void CornersOf (const NvGeBoundBlock2dData* theData, gp_Pnt2d theCorners[4])
{
  theCorners[0] = theData->Point;
  theCorners[1] = theData->Point.Translated (theData->Vec1);
  theCorners[2] = theData->Point.Translated (theData->Vec1).Translated (theData->Vec2);
  theCorners[3] = theData->Point.Translated (theData->Vec2);
}

} // namespace

//=================================================================================================

NvGeBoundBlock2d::NvGeBoundBlock2d()
{
  MakeBlock (mpImpEnt, gp_Pnt2d (0.0, 0.0), gp_Vec2d (0.0, 0.0), gp_Vec2d (0.0, 0.0));
}

//=================================================================================================

NvGeBoundBlock2d::NvGeBoundBlock2d (const NvGePoint2d& thePoint1, const NvGePoint2d& thePoint2)
{
  // Axis-aligned box between the two corners (order-independent).
  const double aMinX = thePoint1.x < thePoint2.x ? thePoint1.x : thePoint2.x;
  const double aMinY = thePoint1.y < thePoint2.y ? thePoint1.y : thePoint2.y;
  const double aMaxX = thePoint1.x < thePoint2.x ? thePoint2.x : thePoint1.x;
  const double aMaxY = thePoint1.y < thePoint2.y ? thePoint2.y : thePoint1.y;
  MakeBlock (mpImpEnt, gp_Pnt2d (aMinX, aMinY), gp_Vec2d (aMaxX - aMinX, 0.0),
             gp_Vec2d (0.0, aMaxY - aMinY));
}

//=================================================================================================

NvGeBoundBlock2d::NvGeBoundBlock2d (const NvGePoint2d& theBase,
                                    const NvGeVector2d& theDir1, const NvGeVector2d& theDir2)
{
  MakeBlock (mpImpEnt, gp_Pnt2d (theBase.x, theBase.y),
             gp_Vec2d (theDir1.x, theDir1.y), gp_Vec2d (theDir2.x, theDir2.y));
}

//=================================================================================================

NvGeBoundBlock2d::NvGeBoundBlock2d (const NvGeBoundBlock2d& theBlock)
: NvGeEntity2d (theBlock)
{
}

//=================================================================================================

void NvGeBoundBlock2d::getMinMaxPoints (NvGePoint2d& thePoint1, NvGePoint2d& thePoint2) const
{
  const NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  gp_Pnt2d aCorners[4];
  CornersOf (aData, aCorners);
  double aMinX = aCorners[0].X();
  double aMinY = aCorners[0].Y();
  double aMaxX = aMinX;
  double aMaxY = aMinY;
  for (int aCorner = 1; aCorner < 4; ++aCorner)
  {
    aMinX = aCorners[aCorner].X() < aMinX ? aCorners[aCorner].X() : aMinX;
    aMinY = aCorners[aCorner].Y() < aMinY ? aCorners[aCorner].Y() : aMinY;
    aMaxX = aCorners[aCorner].X() > aMaxX ? aCorners[aCorner].X() : aMaxX;
    aMaxY = aCorners[aCorner].Y() > aMaxY ? aCorners[aCorner].Y() : aMaxY;
  }
  thePoint1.set (aMinX, aMinY);
  thePoint2.set (aMaxX, aMaxY);
}

//=================================================================================================

void NvGeBoundBlock2d::get (NvGePoint2d& theBase, NvGeVector2d& theDir1, NvGeVector2d& theDir2) const
{
  const NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  theBase.set (aData->Point.X(), aData->Point.Y());
  theDir1.set (aData->Vec1.X(), aData->Vec1.Y());
  theDir2.set (aData->Vec2.X(),aData->Vec2.Y());
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::set (const NvGePoint2d& thePoint1, const NvGePoint2d& thePoint2)
{
  const double aMinX = thePoint1.x < thePoint2.x ? thePoint1.x : thePoint2.x;
  const double aMinY = thePoint1.y < thePoint2.y ? thePoint1.y : thePoint2.y;
  const double aMaxX = thePoint1.x < thePoint2.x ? thePoint2.x : thePoint1.x;
  const double aMaxY = thePoint1.y < thePoint2.y ? thePoint2.y : thePoint1.y;
  NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  aData->Point = gp_Pnt2d (aMinX, aMinY);
  aData->Vec1 = gp_Vec2d (aMaxX - aMinX, 0.0);
  aData->Vec2 = gp_Vec2d (0.0, aMaxY - aMinY);
  return *this;
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::set (const NvGePoint2d& theBase,
                                         const NvGeVector2d& theDir1, const NvGeVector2d& theDir2)
{
  NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  aData->Point = gp_Pnt2d (theBase.x, theBase.y);
  aData->Vec1 = gp_Vec2d (theDir1.x, theDir1.y);
  aData->Vec2 = gp_Vec2d (theDir2.x, theDir2.y);
  return *this;
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::extend (const NvGePoint2d& thePoint)
{
  NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  const double aPx = thePoint.x - aData->Point.X();
  const double aPy = thePoint.y - aData->Point.Y();

  const double aDet = aData->Vec1.X() * aData->Vec2.Y() - aData->Vec1.Y() * aData->Vec2.X();
  const double aTol = NvGeContext::gTol.equalVector();
  const bool anUnb1 = aData->Vec1.Magnitude() <= aTol;
  const bool anUnb2 = aData->Vec2.Magnitude() <= aTol;
  if (anUnb1 && anUnb2)
  {
    return *this; // fully unbounded already contains everything
  }
  if (anUnb1 || anUnb2 || std::abs (aDet) <= 1e-300)
  {
    // One-sided or degenerate frame: fall back to the axis-aligned hull.
    gp_Pnt2d aCorners[4];
    CornersOf (aData, aCorners);
    double aMinX = thePoint.x, aMinY = thePoint.y, aMaxX = thePoint.x, aMaxY = thePoint.y;
    for (int aCorner = 0; aCorner < 4; ++aCorner)
    {
      aMinX = aCorners[aCorner].X() < aMinX ? aCorners[aCorner].X() : aMinX;
      aMinY = aCorners[aCorner].Y() < aMinY ? aCorners[aCorner].Y() : aMinY;
      aMaxX = aCorners[aCorner].X() > aMaxX ? aCorners[aCorner].X() : aMaxX;
      aMaxY = aCorners[aCorner].Y() > aMaxY ? aCorners[aCorner].Y() : aMaxY;
    }
    aData->Point = gp_Pnt2d (aMinX, aMinY);
    aData->Vec1 = gp_Vec2d (aMaxX - aMinX, 0.0);
    aData->Vec2 = gp_Vec2d (0.0, aMaxY - aMinY);
    return *this;
  }

  // Grow the parallelogram coefficients to [sLo, sHi] x [tLo, tHi] so the
  // point (s, t) is contained while the shape directions stay unchanged.
  const double aS = (aPx * aData->Vec2.Y() - aPy * aData->Vec2.X()) / aDet;
  const double aT = (aData->Vec1.X() * aPy - aData->Vec1.Y() * aPx) / aDet;
  const double aSLo = aS < 0.0 ? aS : 0.0;
  const double aSHi = aS > 1.0 ? aS : 1.0;
  const double aTLo = aT < 0.0 ? aT : 0.0;
  const double aTHi = aT > 1.0 ? aT : 1.0;
  const gp_Pnt2d aNewBase = aData->Point.Translated (aData->Vec1 * aSLo).Translated (aData->Vec2 * aTLo);
  aData->Point = aNewBase;
  aData->Vec1 = aData->Vec1 * (aSHi - aSLo);
  aData->Vec2 = aData->Vec2 * (aTHi - aTLo);
  return *this;
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::swell (double theDistance)
{
  NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  const double aTol = NvGeContext::gTol.equalVector();
  gp_Vec2d aGrow1 (0.0, 0.0);
  gp_Vec2d aGrow2 (0.0, 0.0);
  if (aData->Vec1.Magnitude() > aTol)
  {
    aGrow1 = aData->Vec1.Normalized() * theDistance;
  }
  if (aData->Vec2.Magnitude() > aTol)
  {
    aGrow2 = aData->Vec2.Normalized() * theDistance;
  }
  aData->Point = aData->Point.Translated (-aGrow1).Translated (-aGrow2);
  aData->Vec1 += aGrow1 * 2.0;
  aData->Vec2 += aGrow2 * 2.0;
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock2d::contains (const NvGePoint2d& thePoint) const
{
  const NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  const double aPx = thePoint.x - aData->Point.X();
  const double aPy = thePoint.y - aData->Point.Y();
  const double aDet = aData->Vec1.X() * aData->Vec2.Y() - aData->Vec1.Y() * aData->Vec2.X();
  const double aTol = NvGeContext::gTol.equalPoint();
  const bool anUnb1 = aData->Vec1.Magnitude() <= aTol;
  const bool anUnb2 = aData->Vec2.Magnitude() <= aTol;
  if (anUnb1 && anUnb2)
  {
    return true;
  }
  if (anUnb1)
  {
    // Infinite strip perpendicular to Vec2: only the projection along Vec2
    // is constrained.
    const double aT = (aPx * aData->Vec2.X() + aPy * aData->Vec2.Y())
                    / aData->Vec2.SquareMagnitude();
    return aT >= -aTol && aT <= 1.0 + aTol;
  }
  if (anUnb2)
  {
    const double aS = (aPx * aData->Vec1.X() + aPy * aData->Vec1.Y())
                    / aData->Vec1.SquareMagnitude();
    return aS >= -aTol && aS <= 1.0 + aTol;
  }
  if (std::abs (aDet) <= 1e-300)
  {
    return false; // degenerate bounded block
  }
  const double aS = (aPx * aData->Vec2.Y() - aPy * aData->Vec2.X()) / aDet;
  const double aT = (aData->Vec1.X() * aPy - aData->Vec1.Y() * aPx) / aDet;
  return aS >= -aTol && aS <= 1.0 + aTol
      && aT >= -aTol && aT <= 1.0 + aTol;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock2d::isDisjoint (const NvGeBoundBlock2d& theBlock) const
{
  // Conservative axis-aligned hull test: disjoint hulls imply disjoint
  // blocks; overlapping hulls report non-disjoint (exact for boxes).
  NvGePoint2d aMin1, aMax1;
  NvGePoint2d aMin2, aMax2;
  getMinMaxPoints (aMin1, aMax1);
  theBlock.getMinMaxPoints (aMin2, aMax2);
  return aMax1.x < aMin2.x || aMax2.x < aMin1.x
      || aMax1.y < aMin2.y || aMax2.y < aMin1.y;
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::operator = (const NvGeBoundBlock2d& theBlock)
{
  NvGeEntity2d::operator= (theBlock);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock2d::isBox() const
{
  const NvGeBoundBlock2dData* aData = DataOf (mpImpEnt);
  const double aTol = NvGeContext::gTol.equalVector();
  return std::abs (aData->Vec1.Y()) <= aTol && std::abs (aData->Vec2.X()) <= aTol;
}

//=================================================================================================

NvGeBoundBlock2d& NvGeBoundBlock2d::setToBox (Nova::Boolean theToBox)
{
  if (!theToBox)
  {
    return *this; // already a general block representation
  }
  // Replace with the axis-aligned hull of the current corners.
  NvGePoint2d aPnt1, aPnt2;
  getMinMaxPoints (aPnt1, aPnt2);
  return set (aPnt1, aPnt2);
}
