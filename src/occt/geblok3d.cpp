// geblok3d.cpp - implementation of NvGeBoundBlock3d.
//
// A bound block is a parallelepiped spanned by three vectors from a base
// point; a zero vector disables the bound in that direction (unbounded
// side, ARX semantics). Storage lives in the NvGeBoundBlock3dData holder
// from geimpdata.h; the axis-aligned box is the special case where the
// vectors point along the coordinate axes.

#include <geblok3d.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <cmath>

namespace
{

//! Holder of this block, with the COW discipline applied.
NvGeBoundBlock3dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeBoundBlock3dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeBoundBlock3dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeBoundBlock3dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeBoundBlock3dData> (theImp->Geom());
}

//! Installs a fresh holder, replacing the base placeholder impl.
void MakeBlock (NvGeImpEntity3d*& theImp, const gp_Pnt& thePoint,
                const gp_Vec& theVec1, const gp_Vec& theVec2, const gp_Vec& theVec3)
{
  if (theImp != nullptr && theImp->RefCount() > 0)
  {
    // placeholder from the base constructor: release the adoption ref
    theImp->Unref();
  }
  occ::handle<NvGeBoundBlock3dData> aData = new NvGeBoundBlock3dData();
  aData->Point = thePoint;
  aData->Vec1 = theVec1;
  aData->Vec2 = theVec2;
  aData->Vec3 = theVec3;
  theImp = new NvGeImpEntity3d (NvGe::kBoundBlock3d, aData);
  theImp->Ref();
}

//! The eight corners of the parallelepiped.
void CornersOf (const NvGeBoundBlock3dData* theData, gp_Pnt theCorners[8])
{
  theCorners[0] = theData->Point;
  theCorners[1] = theData->Point.Translated (theData->Vec1);
  theCorners[2] = theData->Point.Translated (theData->Vec2);
  theCorners[3] = theData->Point.Translated (theData->Vec3);
  theCorners[4] = theData->Point.Translated (theData->Vec1).Translated (theData->Vec2);
  theCorners[5] = theData->Point.Translated (theData->Vec1).Translated (theData->Vec3);
  theCorners[6] = theData->Point.Translated (theData->Vec2).Translated (theData->Vec3);
  theCorners[7] = theData->Point.Translated (theData->Vec1)
                     .Translated (theData->Vec2).Translated (theData->Vec3);
}

//! Triple product V1 . (V2 ^ V3); zero for a degenerate frame.
double DetOf (const gp_Vec& theVec1, const gp_Vec& theVec2, const gp_Vec& theVec3)
{
  return theVec1.Dot (theVec2.Crossed (theVec3));
}

//! Projection coefficient of a delta along an axis, relative to [0, 1].
bool InSpan (const gp_Vec& theDelta, const gp_Vec& theAxis, double theTol)
{
  const double aT = theDelta.Dot (theAxis) / theAxis.SquareMagnitude();
  return aT >= -theTol && aT <= 1.0 + theTol;
}

} // namespace

//=================================================================================================

NvGeBoundBlock3d::NvGeBoundBlock3d()
{
  MakeBlock (mpImpEnt, gp_Pnt (0.0, 0.0, 0.0), gp_Vec (0.0, 0.0, 0.0),
             gp_Vec (0.0, 0.0, 0.0), gp_Vec (0.0, 0.0, 0.0));
}

//=================================================================================================

NvGeBoundBlock3d::NvGeBoundBlock3d (const NvGePoint3d& theBase, const NvGeVector3d& theDir1,
                                    const NvGeVector3d& theDir2, const NvGeVector3d& theDir3)
{
  MakeBlock (mpImpEnt, gp_Pnt (theBase.x, theBase.y, theBase.z),
             gp_Vec (theDir1.x, theDir1.y, theDir1.z),
             gp_Vec (theDir2.x, theDir2.y, theDir2.z),
             gp_Vec (theDir3.x, theDir3.y, theDir3.z));
}

//=================================================================================================

NvGeBoundBlock3d::NvGeBoundBlock3d (const NvGeBoundBlock3d& theBlock)
: NvGeEntity3d (theBlock)
{
}

//=================================================================================================

void NvGeBoundBlock3d::getMinMaxPoints (NvGePoint3d& thePoint1, NvGePoint3d& thePoint2) const
{
  const NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  gp_Pnt aCorners[8];
  CornersOf (aData, aCorners);
  double aMinX = aCorners[0].X();
  double aMinY = aCorners[0].Y();
  double aMinZ = aCorners[0].Z();
  double aMaxX = aMinX;
  double aMaxY = aMinY;
  double aMaxZ = aMinZ;
  for (int aCorner = 1; aCorner < 8; ++aCorner)
  {
    aMinX = aCorners[aCorner].X() < aMinX ? aCorners[aCorner].X() : aMinX;
    aMinY = aCorners[aCorner].Y() < aMinY ? aCorners[aCorner].Y() : aMinY;
    aMinZ = aCorners[aCorner].Z() < aMinZ ? aCorners[aCorner].Z() : aMinZ;
    aMaxX = aCorners[aCorner].X() > aMaxX ? aCorners[aCorner].X() : aMaxX;
    aMaxY = aCorners[aCorner].Y() > aMaxY ? aCorners[aCorner].Y() : aMaxY;
    aMaxZ = aCorners[aCorner].Z() > aMaxZ ? aCorners[aCorner].Z() : aMaxZ;
  }
  thePoint1.set (aMinX, aMinY, aMinZ);
  thePoint2.set (aMaxX, aMaxY, aMaxZ);
}

//=================================================================================================

void NvGeBoundBlock3d::get (NvGePoint3d& theBase, NvGeVector3d& theDir1,
                            NvGeVector3d& theDir2, NvGeVector3d& theDir3) const
{
  const NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  theBase.set (aData->Point.X(), aData->Point.Y(), aData->Point.Z());
  theDir1.set (aData->Vec1.X(), aData->Vec1.Y(), aData->Vec1.Z());
  theDir2.set (aData->Vec2.X(), aData->Vec2.Y(), aData->Vec2.Z());
  theDir3.set (aData->Vec3.X(), aData->Vec3.Y(), aData->Vec3.Z());
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::set (const NvGePoint3d& thePoint1, const NvGePoint3d& thePoint2)
{
  const double aMinX = thePoint1.x < thePoint2.x ? thePoint1.x : thePoint2.x;
  const double aMinY = thePoint1.y < thePoint2.y ? thePoint1.y : thePoint2.y;
  const double aMinZ = thePoint1.z < thePoint2.z ? thePoint1.z : thePoint2.z;
  const double aMaxX = thePoint1.x < thePoint2.x ? thePoint2.x : thePoint1.x;
  const double aMaxY = thePoint1.y < thePoint2.y ? thePoint2.y : thePoint1.y;
  const double aMaxZ = thePoint1.z < thePoint2.z ? thePoint2.z : thePoint1.z;
  NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  aData->Point = gp_Pnt (aMinX, aMinY, aMinZ);
  aData->Vec1 = gp_Vec (aMaxX - aMinX, 0.0, 0.0);
  aData->Vec2 = gp_Vec (0.0, aMaxY - aMinY, 0.0);
  aData->Vec3 = gp_Vec (0.0, 0.0, aMaxZ - aMinZ);
  return *this;
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::set (const NvGePoint3d& theBase, const NvGeVector3d& theDir1,
                                         const NvGeVector3d& theDir2, const NvGeVector3d& theDir3)
{
  NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  aData->Point = gp_Pnt (theBase.x, theBase.y, theBase.z);
  aData->Vec1 = gp_Vec (theDir1.x, theDir1.y, theDir1.z);
  aData->Vec2 = gp_Vec (theDir2.x, theDir2.y, theDir2.z);
  aData->Vec3 = gp_Vec (theDir3.x, theDir3.y, theDir3.z);
  return *this;
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::extend (const NvGePoint3d& thePoint)
{
  NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  const gp_Vec aDelta (thePoint.x - aData->Point.X(),
                       thePoint.y - aData->Point.Y(),
                       thePoint.z - aData->Point.Z());

  const double aTol = NvGeContext::gTol.equalVector();
  const bool anUnb1 = aData->Vec1.Magnitude() <= aTol;
  const bool anUnb2 = aData->Vec2.Magnitude() <= aTol;
  const bool anUnb3 = aData->Vec3.Magnitude() <= aTol;
  if (anUnb1 && anUnb2 && anUnb3)
  {
    return *this; // fully unbounded already contains everything
  }
  if (anUnb1 || anUnb2 || anUnb3
      || std::abs (DetOf (aData->Vec1, aData->Vec2, aData->Vec3)) <= 1e-300)
  {
    // Partially unbounded or degenerate frame: fall back to the axis-aligned
    // hull of the current corners and the new point.
    gp_Pnt aCorners[8];
    CornersOf (aData, aCorners);
    double aMinX = thePoint.x, aMinY = thePoint.y, aMinZ = thePoint.z;
    double aMaxX = thePoint.x, aMaxY = thePoint.y, aMaxZ = thePoint.z;
    for (int aCorner = 0; aCorner < 8; ++aCorner)
    {
      aMinX = aCorners[aCorner].X() < aMinX ? aCorners[aCorner].X() : aMinX;
      aMinY = aCorners[aCorner].Y() < aMinY ? aCorners[aCorner].Y() : aMinY;
      aMinZ = aCorners[aCorner].Z() < aMinZ ? aCorners[aCorner].Z() : aMinZ;
      aMaxX = aCorners[aCorner].X() > aMaxX ? aCorners[aCorner].X() : aMaxX;
      aMaxY = aCorners[aCorner].Y() > aMaxY ? aCorners[aCorner].Y() : aMaxY;
      aMaxZ = aCorners[aCorner].Z() > aMaxZ ? aCorners[aCorner].Z() : aMaxZ;
    }
    aData->Point = gp_Pnt (aMinX, aMinY, aMinZ);
    aData->Vec1 = gp_Vec (aMaxX - aMinX, 0.0, 0.0);
    aData->Vec2 = gp_Vec (0.0, aMaxY - aMinY, 0.0);
    aData->Vec3 = gp_Vec (0.0, 0.0, aMaxZ - aMinZ);
    return *this;
  }

  // Grow the parallelepiped coefficients to [sLo, sHi] x [tLo, tHi] x
  // [uLo, uHi] so the point (s, t, u) is contained while the shape
  // directions stay unchanged (Cramer solve of V1*s + V2*t + V3*u = delta).
  const double aDet = DetOf (aData->Vec1, aData->Vec2, aData->Vec3);
  const double aS = DetOf (aDelta, aData->Vec2, aData->Vec3) / aDet;
  const double aT = DetOf (aData->Vec1, aDelta, aData->Vec3) / aDet;
  const double aU = DetOf (aData->Vec1, aData->Vec2, aDelta) / aDet;
  const double aSLo = aS < 0.0 ? aS : 0.0;
  const double aSHi = aS > 1.0 ? aS : 1.0;
  const double aTLo = aT < 0.0 ? aT : 0.0;
  const double aTHi = aT > 1.0 ? aT : 1.0;
  const double aULo = aU < 0.0 ? aU : 0.0;
  const double aUHi = aU > 1.0 ? aU : 1.0;
  const gp_Pnt aNewBase = aData->Point.Translated (aData->Vec1 * aSLo)
                             .Translated (aData->Vec2 * aTLo).Translated (aData->Vec3 * aULo);
  aData->Point = aNewBase;
  aData->Vec1 = aData->Vec1 * (aSHi - aSLo);
  aData->Vec2 = aData->Vec2 * (aTHi - aTLo);
  aData->Vec3 = aData->Vec3 * (aUHi - aULo);
  return *this;
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::swell (double theDistance)
{
  NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  const double aTol = NvGeContext::gTol.equalVector();
  gp_Vec aGrow1 (0.0, 0.0, 0.0);
  gp_Vec aGrow2 (0.0, 0.0, 0.0);
  gp_Vec aGrow3 (0.0, 0.0, 0.0);
  if (aData->Vec1.Magnitude() > aTol)
  {
    aGrow1 = aData->Vec1.Normalized() * theDistance;
  }
  if (aData->Vec2.Magnitude() > aTol)
  {
    aGrow2 = aData->Vec2.Normalized() * theDistance;
  }
  if (aData->Vec3.Magnitude() > aTol)
  {
    aGrow3 = aData->Vec3.Normalized() * theDistance;
  }
  aData->Point = aData->Point.Translated (-aGrow1).Translated (-aGrow2).Translated (-aGrow3);
  aData->Vec1 += aGrow1 * 2.0;
  aData->Vec2 += aGrow2 * 2.0;
  aData->Vec3 += aGrow3 * 2.0;
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock3d::contains (const NvGePoint3d& thePoint) const
{
  const NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  const gp_Vec aDelta (thePoint.x - aData->Point.X(),
                       thePoint.y - aData->Point.Y(),
                       thePoint.z - aData->Point.Z());
  const double aTol = NvGeContext::gTol.equalPoint();
  const bool anUnb1 = aData->Vec1.Magnitude() <= aTol;
  const bool anUnb2 = aData->Vec2.Magnitude() <= aTol;
  const bool anUnb3 = aData->Vec3.Magnitude() <= aTol;
  if (anUnb1 && anUnb2 && anUnb3)
  {
    return true;
  }
  if (anUnb1 && anUnb2)
  {
    // Infinite strip along Vec3: only the projection along Vec3 is bounded.
    return InSpan (aDelta, aData->Vec3, aTol);
  }
  if (anUnb1 && anUnb3)
  {
    return InSpan (aDelta, aData->Vec2, aTol);
  }
  if (anUnb2 && anUnb3)
  {
    return InSpan (aDelta, aData->Vec1, aTol);
  }
  if (anUnb1 || anUnb2 || anUnb3)
  {
    // Single unbounded side: projection-based test along both bounded
    // directions (the unbounded direction itself is unconstrained).
    if (anUnb1)
    {
      return InSpan (aDelta, aData->Vec2, aTol) && InSpan (aDelta, aData->Vec3, aTol);
    }
    if (anUnb2)
    {
      return InSpan (aDelta, aData->Vec1, aTol) && InSpan (aDelta, aData->Vec3, aTol);
    }
    return InSpan (aDelta, aData->Vec1, aTol) && InSpan (aDelta, aData->Vec2, aTol);
  }
  if (std::abs (DetOf (aData->Vec1, aData->Vec2, aData->Vec3)) <= 1e-300)
  {
    return false; // degenerate bounded block
  }
  const double aDet = DetOf (aData->Vec1, aData->Vec2, aData->Vec3);
  const double aS = DetOf (aDelta, aData->Vec2, aData->Vec3) / aDet;
  const double aT = DetOf (aData->Vec1, aDelta, aData->Vec3) / aDet;
  const double aU = DetOf (aData->Vec1, aData->Vec2, aDelta) / aDet;
  return aS >= -aTol && aS <= 1.0 + aTol
      && aT >= -aTol && aT <= 1.0 + aTol
      && aU >= -aTol && aU <= 1.0 + aTol;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock3d::isDisjoint (const NvGeBoundBlock3d& theBlock) const
{
  // Conservative axis-aligned hull test: disjoint hulls imply disjoint
  // blocks; overlapping hulls report non-disjoint (exact for boxes).
  NvGePoint3d aMin1, aMax1;
  NvGePoint3d aMin2, aMax2;
  getMinMaxPoints (aMin1, aMax1);
  theBlock.getMinMaxPoints (aMin2, aMax2);
  return aMax1.x < aMin2.x || aMax2.x < aMin1.x
      || aMax1.y < aMin2.y || aMax2.y < aMin1.y
      || aMax1.z < aMin2.z || aMax2.z < aMin1.z;
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::operator = (const NvGeBoundBlock3d& theBlock)
{
  NvGeEntity3d::operator= (theBlock);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeBoundBlock3d::isBox() const
{
  const NvGeBoundBlock3dData* aData = DataOf (mpImpEnt);
  const double aTol = NvGeContext::gTol.equalVector();
  return std::abs (aData->Vec1.Y()) <= aTol && std::abs (aData->Vec1.Z()) <= aTol
      && std::abs (aData->Vec2.X()) <= aTol && std::abs (aData->Vec2.Z()) <= aTol
      && std::abs (aData->Vec3.X()) <= aTol && std::abs (aData->Vec3.Y()) <= aTol;
}

//=================================================================================================

NvGeBoundBlock3d& NvGeBoundBlock3d::setToBox (Nova::Boolean theToBox)
{
  if (!theToBox)
  {
    return *this; // already a general block representation
  }
  // Replace with the axis-aligned hull of the current corners.
  NvGePoint3d aPnt1, aPnt2;
  getMinMaxPoints (aPnt1, aPnt2);
  return set (aPnt1, aPnt2);
}
