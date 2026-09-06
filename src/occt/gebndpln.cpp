// gebndpln.cpp - implementation of NvGeBoundedPlane.
//
// A bounded plane is a parallelogram patch of a plane: the impl stores a
// Geom_RectangularTrimmedSurface over a Geom_Plane whose parameter
// rectangle [0, |u|] x [0, |v|] spans exactly the patch defined by the
// frame vectors. All generic surface operations (evaluation, projection,
// isOn) work through the trimmed surface; the planar frame queries come
// from the basis plane.

#include <gebndpln.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gelent3d.h>
#include <geline3d.h>
#include <gelnsg3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>
#include <string>

namespace
{

//! Installs the trimmed plane patch, replacing the base placeholder impl.
void MakeBounded (NvGeImpEntity3d*& theImp, const gp_Ax3& thePosition,
                  double theULength, double theVLength)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  const occ::handle<Geom_Plane> aPlane = new Geom_Plane (thePosition);
  theImp = new NvGeImpEntity3d (NvGe::kBoundedPlane,
      occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
        aPlane, 0.0, theULength, 0.0, theVLength)));
  theImp->Ref();
}

//! Trimmed patch geometry of this entity.
occ::handle<Geom_RectangularTrimmedSurface> TrimmedOf (const NvGeImpEntity3d* theImp)
{
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed =
    occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
  if (aTrimmed.IsNull())
  {
    throw NvException ("NvGeBoundedPlane: the entity holds no bounded plane geometry");
  }
  return aTrimmed;
}

//! Basis plane of the trimmed patch.
occ::handle<Geom_Plane> BasisPlaneOf (const NvGeImpEntity3d* theImp)
{
  const occ::handle<Geom_Plane> aPlane =
    occ::down_cast<Geom_Plane> (TrimmedOf (theImp)->BasisSurface());
  if (aPlane.IsNull())
  {
    throw NvException ("NvGeBoundedPlane: the trimmed basis is not a plane");
  }
  return aPlane;
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeBoundedPlane::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

} // namespace

//=================================================================================================

NvGeBoundedPlane::NvGeBoundedPlane()
{
  // Default: unit patch of the world XY plane.
  MakeBounded (mpImpEnt, gp_Ax3 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0), gp_Dir (1.0, 0.0, 0.0)),
               1.0, 1.0);
}

//=================================================================================================

NvGeBoundedPlane::NvGeBoundedPlane (const NvGeBoundedPlane& thePlane)
: NvGePlanarEnt (thePlane)
{
}

//=================================================================================================

NvGeBoundedPlane::NvGeBoundedPlane (const NvGePoint3d& theOrigin, const NvGeVector3d& theUVec,
                                    const NvGeVector3d& theVVec)
{
  const gp_Dir anU = DirOrThrow (theUVec.x, theUVec.y, theUVec.z, "NvGeBoundedPlane");
  const gp_Dir aV = DirOrThrow (theVVec.x, theVVec.y, theVVec.z, "NvGeBoundedPlane");
  const gp_Dir aNormal (anU.Crossed (aV));
  MakeBounded (mpImpEnt, gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), aNormal, anU),
               std::sqrt (theUVec.lengthSqrd()), std::sqrt (theVVec.lengthSqrd()));
}

//=================================================================================================

NvGeBoundedPlane::NvGeBoundedPlane (const NvGePoint3d& theP1, const NvGePoint3d& theOrigin,
                                    const NvGePoint3d& theP2)
{
  const gp_Dir anU = DirOrThrow (theP1.x - theOrigin.x, theP1.y - theOrigin.y,
                                 theP1.z - theOrigin.z, "NvGeBoundedPlane");
  const gp_Dir aV = DirOrThrow (theP2.x - theOrigin.x, theP2.y - theOrigin.y,
                                theP2.z - theOrigin.z, "NvGeBoundedPlane");
  const gp_Dir aNormal (anU.Crossed (aV));
  const double anULen = std::sqrt ((theP1.x - theOrigin.x) * (theP1.x - theOrigin.x)
                                 + (theP1.y - theOrigin.y) * (theP1.y - theOrigin.y)
                                 + (theP1.z - theOrigin.z) * (theP1.z - theOrigin.z));
  const double aVLen = std::sqrt ((theP2.x - theOrigin.x) * (theP2.x - theOrigin.x)
                                 + (theP2.y - theOrigin.y) * (theP2.y - theOrigin.y)
                                 + (theP2.z - theOrigin.z) * (theP2.z - theOrigin.z));
  MakeBounded (mpImpEnt, gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), aNormal, anU),
               anULen, aVLen);
}

//=================================================================================================

Nova::Boolean NvGeBoundedPlane::intersectWith (const NvGeLinearEnt3d& theLinEnt,
                                                NvGePoint3d& thePoint, const NvGeTol& theTol) const
{
  // Infinite-plane hit, then inside-the-patch check in local coordinates.
  const NvGePoint3d anOrigin = pointOnPlane();
  const NvGeVector3d aNormal = normal();
  const NvGePoint3d aLineOrigin = theLinEnt.pointOnLine();
  const NvGeVector3d aDir = theLinEnt.direction();
  const double aDenom = aNormal.dotProduct (aDir);
  if (std::abs (aDenom) <= theTol.equalVector())
  {
    return false;
  }
  const double aParam = (aNormal.dotProduct (NvGeVector3d (anOrigin.x - aLineOrigin.x,
                                                           anOrigin.y - aLineOrigin.y,
                                                           anOrigin.z - aLineOrigin.z))) / aDenom;
  thePoint.set (aLineOrigin.x + aParam * aDir.x,
                aLineOrigin.y + aParam * aDir.y,
                aLineOrigin.z + aParam * aDir.z);

  // Local coordinates within the patch rectangle.
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed = TrimmedOf (mpImpEnt);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  aTrimmed->Bounds (anU1, anU2, aV1, aV2);
  const gp_Pln aPln = BasisPlaneOf (mpImpEnt)->Pln();
  const gp_Pnt aLoc = aPln.Location();
  const gp_Dir anX = aPln.Position().XDirection();
  const gp_Dir aY = aPln.Position().YDirection();
  const double aDx = thePoint.x - aLoc.X();
  const double aDy = thePoint.y - aLoc.Y();
  const double aDz = thePoint.z - aLoc.Z();
  const double anU = aDx * anX.X() + aDy * anX.Y() + aDz * anX.Z();
  const double aV = aDx * aY.X() + aDy * aY.Y() + aDz * aY.Z();
  const double aTol = theTol.equalPoint();
  return anU >= anU1 - aTol && anU <= anU2 + aTol
      && aV >= aV1 - aTol && aV <= aV2 + aTol;
}

//=================================================================================================

Nova::Boolean NvGeBoundedPlane::intersectWith (const NvGePlane& thePlane,
                                                NvGeLineSeg3d& theResults,
                                                const NvGeTol& theTol) const
{
  // Symmetric to NvGePlane::intersectWith(boundedPlane).
  return thePlane.intersectWith (*this, theResults, theTol);
}

//=================================================================================================

Nova::Boolean NvGeBoundedPlane::intersectWith (const NvGeBoundedPlane& thePlane,
                                                NvGeLineSeg3d& theResult,
                                                const NvGeTol& theTol) const
{
  // Intersect the two carriers, then clip the line to both parameter
  // rectangles.
  NvGePlane aCarrier1;
  aCarrier1.set (pointOnPlane(), normal());
  NvGePlane aCarrier2;
  aCarrier2.set (thePlane.pointOnPlane(), thePlane.normal());
  NvGeLine3d aLine;
  if (!aCarrier1.intersectWith (aCarrier2, aLine, theTol))
  {
    return false;
  }
  NvGeLineSeg3d aSegment;
  NvGePlane aSelfAsPlane;
  aSelfAsPlane.set (pointOnPlane(), normal());
  if (!aSelfAsPlane.intersectWith (*this, aSegment, theTol))
  {
    return false;
  }
  // Clip the segment of this patch further against the other patch: keep
  // the portion whose points project inside the other rectangle.
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmedOther = TrimmedOf (thePlane.mpImpEnt);
  double anOtherU1 = 0.0, anOtherU2 = 0.0, anOtherV1 = 0.0, anOtherV2 = 0.0;
  aTrimmedOther->Bounds (anOtherU1, anOtherU2, anOtherV1, anOtherV2);
  const gp_Pln aPlnOther = BasisPlaneOf (thePlane.mpImpEnt)->Pln();
  const gp_Pnt aLocOther = aPlnOther.Location();
  const gp_Dir anXOther = aPlnOther.Position().XDirection();
  const gp_Dir aYOther = aPlnOther.Position().YDirection();
  auto InsideOther = [&] (const NvGePoint3d& thePnt)
  {
    const double aDx = thePnt.x - aLocOther.X();
    const double aDy = thePnt.y - aLocOther.Y();
    const double aDz = thePnt.z - aLocOther.Z();
    const double anU = aDx * anXOther.X() + aDy * anXOther.Y() + aDz * anXOther.Z();
    const double aV = aDx * aYOther.X() + aDy * aYOther.Y() + aDz * aYOther.Z();
    const double aTol = theTol.equalPoint();
    return anU >= anOtherU1 - aTol && anU <= anOtherU2 + aTol
        && aV >= anOtherV1 - aTol && aV <= anOtherV2 + aTol;
  };
  NvGePoint3d aStart;
  NvGePoint3d anEnd;
  aStart = aSegment.startPoint();
  anEnd = aSegment.endPoint();
  const double aLen = aStart.distanceTo (anEnd);
  if (aLen <= theTol.equalPoint())
  {
    return false;
  }
  // Sample along the segment and keep the longest run inside the other.
  const int THE_NB_SAMPLES = 64;
  const NvGeVector3d aStep ((anEnd.x - aStart.x) / THE_NB_SAMPLES,
                            (anEnd.y - aStart.y) / THE_NB_SAMPLES,
                            (anEnd.z - aStart.z) / THE_NB_SAMPLES);
  int aBestStart = -1;
  int aBestLen = 0;
  int aRunStart = -1;
  for (int aSample = 0; aSample <= THE_NB_SAMPLES; ++aSample)
  {
    const NvGePoint3d aPnt (aStart.x + aSample * aStep.x,
                            aStart.y + aSample * aStep.y,
                            aStart.z + aSample * aStep.z);
    const bool anIn = InsideOther (aPnt);
    if (anIn && aRunStart < 0)
    {
      aRunStart = aSample;
    }
    else if (!anIn && aRunStart >= 0)
    {
      if (aSample - aRunStart > aBestLen)
      {
        aBestLen = aSample - aRunStart;
        aBestStart = aRunStart;
      }
      aRunStart = -1;
    }
  }
  if (aRunStart >= 0 && THE_NB_SAMPLES + 1 - aRunStart > aBestLen)
  {
    aBestLen = THE_NB_SAMPLES + 1 - aRunStart;
    aBestStart = aRunStart;
  }
  if (aBestStart < 0)
  {
    return false;
  }
  const int anEndIdx = aBestStart + aBestLen - 1;
  theResult.set (NvGePoint3d (aStart.x + aBestStart * aStep.x, aStart.y + aBestStart * aStep.y,
                              aStart.z + aBestStart * aStep.z),
                 NvGePoint3d (aStart.x + anEndIdx * aStep.x, aStart.y + anEndIdx * aStep.y,
                              aStart.z + anEndIdx * aStep.z));
  return true;
}

//=================================================================================================

NvGeBoundedPlane& NvGeBoundedPlane::set (const NvGePoint3d& theOrigin, const NvGeVector3d& theUVec,
                                         const NvGeVector3d& theVVec)
{
  *this = NvGeBoundedPlane (theOrigin, theUVec, theVVec);
  return *this;
}

//=================================================================================================

NvGeBoundedPlane& NvGeBoundedPlane::set (const NvGePoint3d& theP1, const NvGePoint3d& theOrigin,
                                         const NvGePoint3d& theP2)
{
  *this = NvGeBoundedPlane (theP1, theOrigin, theP2);
  return *this;
}

//=================================================================================================

NvGeBoundedPlane& NvGeBoundedPlane::operator = (const NvGeBoundedPlane& theBplane)
{
  NvGePlanarEnt::operator= (theBplane);
  return *this;
}
