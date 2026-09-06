// gesurf.cpp - implementation of NvGeSurface, the base of all surfaces.
//
// Every surface impl stores its occ::handle<Geom_Surface> in the entity
// implementation object (see geimpdata.h); the base methods below operate
// generically on that handle through projection, evaluation and bounds
// queries.

#include <gesurf.h>

#include <NvException.h>
#include <geimpdata.h>
#include <geponsrf.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevc3dar.h>

#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <limits>

//=================================================================================================

NvGePoint2d NvGeSurface::paramOf (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  (void)theTol;
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  GeomAPI_ProjectPointOnSurf aProjector (gp_Pnt (thePnt.x, thePnt.y, thePnt.z), aSurface);
  if (aProjector.NbPoints() == 0)
  {
    throw NvException ("NvGeSurface::paramOf(): the projection found no solution");
  }
  double anU = 0.0;
  double aV = 0.0;
  aProjector.LowerDistanceParameters (anU, aV);
  return NvGePoint2d (anU, aV);
}

//=================================================================================================

Nova::Boolean NvGeSurface::isOn (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  GeomAPI_ProjectPointOnSurf aProjector (gp_Pnt (thePnt.x, thePnt.y, thePnt.z), aSurface);
  return aProjector.NbPoints() > 0
      && aProjector.LowerDistance() <= theTol.equalPoint();
}

//=================================================================================================

Nova::Boolean NvGeSurface::isOn (const NvGePoint3d& thePnt, NvGePoint2d& theParamPoint,
                                  const NvGeTol& theTol) const
{
  if (!isOn (thePnt, theTol))
  {
    return false;
  }
  theParamPoint = paramOf (thePnt, theTol);
  return true;
}

//=================================================================================================

NvGePoint3d NvGeSurface::closestPointTo (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  (void)theTol;
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  GeomAPI_ProjectPointOnSurf aProjector (gp_Pnt (thePnt.x, thePnt.y, thePnt.z), aSurface);
  if (aProjector.NbPoints() == 0)
  {
    throw NvException ("NvGeSurface::closestPointTo(): the projection found no solution");
  }
  const gp_Pnt aClosest = aProjector.NearestPoint();
  return NvGePoint3d (aClosest.X(), aClosest.Y(), aClosest.Z());
}

//=================================================================================================

void NvGeSurface::getClosestPointTo (const NvGePoint3d& thePnt, NvGePointOnSurface& theResult,
                                     const NvGeTol& theTol) const
{
  const NvGePoint2d aParam = paramOf (thePnt, theTol);
  theResult.setParameter (aParam);
}

//=================================================================================================

double NvGeSurface::distanceTo (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  (void)theTol;
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  GeomAPI_ProjectPointOnSurf aProjector (gp_Pnt (thePnt.x, thePnt.y, thePnt.z), aSurface);
  return aProjector.NbPoints() > 0 ? aProjector.LowerDistance()
                                   : std::numeric_limits<double>::infinity();
}

//=================================================================================================

Nova::Boolean NvGeSurface::isNormalReversed() const
{
  // The base stores no orientation flag; the orientation follows the OCCT
  // surface geometry. Subclasses with explicit orientation override this.
  return false;
}

//=================================================================================================

NvGeSurface& NvGeSurface::reverseNormal()
{
  // Reverse the u parameterization on a private copy (copy-on-write),
  // which flips the surface normal direction.
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  occ::handle<Geom_Surface> aCopy = occ::down_cast<Geom_Surface> (aSurface->Copy());
  aCopy->UReverse();
  mpImpEnt->SetGeom (aCopy);
  return *this;
}

//=================================================================================================

NvGeSurface& NvGeSurface::operator = (const NvGeSurface& theOtherSurface)
{
  NvGeEntity3d::operator= (theOtherSurface);
  return *this;
}

//=================================================================================================

void NvGeSurface::getEnvelope (NvGeInterval& theIntrvlX, NvGeInterval& theIntrvlY) const
{
  // Parameter envelope of the surface: u into X, v into Y. OCCT stores
  // unbounded domains as huge values; map them back to infinities.
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  aSurface->Bounds (anU1, anU2, aV1, aV2);
  const double anInf = std::numeric_limits<double>::infinity();
  const double aBig = 1e100;
  theIntrvlX.set (std::abs (anU1) >= aBig ? -anInf : anU1, std::abs (anU2) >= aBig ? anInf : anU2);
  theIntrvlY.set (std::abs (aV1) >= aBig ? -anInf : aV1, std::abs (aV2) >= aBig ? anInf : aV2);
}

//=================================================================================================

Nova::Boolean NvGeSurface::isClosedInU (const NvGeTol& theTol) const
{
  (void)theTol;
  return NvGeSurfaceOf (mpImpEnt)->IsUClosed();
}

//=================================================================================================

Nova::Boolean NvGeSurface::isClosedInV (const NvGeTol& theTol) const
{
  (void)theTol;
  return NvGeSurfaceOf (mpImpEnt)->IsVClosed();
}

//=================================================================================================

NvGePoint3d NvGeSurface::evalPoint (const NvGePoint2d& theParam) const
{
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  const gp_Pnt aPnt = aSurface->Value (theParam.x, theParam.y);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGeSurface::evalPoint (const NvGePoint2d& theParam, int theDerivOrd,
                                    NvGeVector3dArray& theDerivatives) const
{
  if (theDerivOrd < 1 || theDerivOrd > 2)
  {
    throw NvException ("NvGeSurface::evalPoint(): the derivative order must be 1 or 2");
  }
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (mpImpEnt);
  gp_Pnt aPnt;
  gp_Vec aD1U, aD1V, aD2U, aD2V, aD2UV;
  theDerivatives.setLogicalLength (0);
  if (theDerivOrd == 1)
  {
    aSurface->D1 (theParam.x, theParam.y, aPnt, aD1U, aD1V);
    theDerivatives.append (NvGeVector3d (aD1U.X(), aD1U.Y(), aD1U.Z()));
    theDerivatives.append (NvGeVector3d (aD1V.X(), aD1V.Y(), aD1V.Z()));
  }
  else
  {
    aSurface->D2 (theParam.x, theParam.y, aPnt, aD1U, aD1V, aD2U, aD2V, aD2UV);
    theDerivatives.append (NvGeVector3d (aD1U.X(), aD1U.Y(), aD1U.Z()));
    theDerivatives.append (NvGeVector3d (aD1V.X(), aD1V.Y(), aD1V.Z()));
    theDerivatives.append (NvGeVector3d (aD2U.X(), aD2U.Y(), aD2U.Z()));
    theDerivatives.append (NvGeVector3d (aD2V.X(), aD2V.Y(), aD2V.Z()));
    theDerivatives.append (NvGeVector3d (aD2UV.X(), aD2UV.Y(), aD2UV.Z()));
  }
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGeSurface::evalPoint (const NvGePoint2d& theParam, int theDerivOrd,
                                    NvGeVector3dArray& theDerivatives,
                                    NvGeVector3d& theNormal) const
{
  const NvGePoint3d aPnt = evalPoint (theParam, theDerivOrd, theDerivatives);
  if (theDerivatives.length() >= 2)
  {
    const NvGeVector3d& aD1U = theDerivatives[0];
    const NvGeVector3d& aD1V = theDerivatives[1];
    const NvGeVector3d aCross (aD1U.y * aD1V.z - aD1U.z * aD1V.y,
                               aD1U.z * aD1V.x - aD1U.x * aD1V.z,
                               aD1U.x * aD1V.y - aD1U.y * aD1V.x);
    const double aLen = std::sqrt (aCross.lengthSqrd());
    if (aLen > 0.0)
    {
      theNormal.set (aCross.x / aLen, aCross.y / aLen, aCross.z / aLen);
    }
  }
  return aPnt;
}

//=================================================================================================

NvGeSurface::NvGeSurface()
: NvGeEntity3d()
{
}

//=================================================================================================

NvGeSurface::NvGeSurface (const NvGeSurface& theSrc)
: NvGeEntity3d (theSrc)
{
}
