// geplanar.cpp - implementation of NvGePlanarEnt, the base of planes.
//
// A planar entity stores a Geom_Plane in its impl; the local frame comes
// from the plane's gp_Ax3 position: origin = Location, u = XDirection,
// v = YDirection, normal = main Direction (u x v = normal, right-handed).

#include <geplanar.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gelent3d.h>
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

namespace
{

//! Plane geometry of this entity. Bounded planes store a rectangular
//! trimmed surface over their basis plane; unwrap it transparently.
occ::handle<Geom_Plane> PlaneOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_Plane> aPlane = occ::down_cast<Geom_Plane> (theImp->Geom());
  if (!aPlane.IsNull())
  {
    return aPlane;
  }
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed =
    occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
  if (!aTrimmed.IsNull())
  {
    aPlane = occ::down_cast<Geom_Plane> (aTrimmed->BasisSurface());
    if (!aPlane.IsNull())
    {
      return aPlane;
    }
  }
  throw NvException ("NvGePlanarEnt: the entity holds no plane geometry");
}

} // namespace

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::intersectWith (const NvGeLinearEnt3d& theLinEnt,
                                             NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  // Ray-plane intersection along the line parameterization.
  const NvGePoint3d anOrigin = theLinEnt.pointOnLine();
  const NvGeVector3d aDir = theLinEnt.direction();
  const NvGeVector3d aNormal = normal();
  const NvGePoint3d aPlaneOrigin = pointOnPlane();

  const double aDenom = aNormal.dotProduct (aDir);
  if (std::abs (aDenom) <= theTol.equalVector())
  {
    return false; // line parallel to the plane
  }
  const NvGeVector3d aDelta (anOrigin.x - aPlaneOrigin.x,
                             anOrigin.y - aPlaneOrigin.y,
                             anOrigin.z - aPlaneOrigin.z);
  const double aParam = -aNormal.dotProduct (aDelta) / aDenom;
  thePnt.set (anOrigin.x + aParam * aDir.x,
              anOrigin.y + aParam * aDir.y,
              anOrigin.z + aParam * aDir.z);
  return true;
}

//=================================================================================================

NvGePoint3d NvGePlanarEnt::closestPointToLinearEnt (const NvGeLinearEnt3d& theLine,
                                                    NvGePoint3d& thePntOnLine,
                                                    const NvGeTol& theTol) const
{
  // Closest point pair between the plane and the line: project the line
  // origin onto the plane; the line point is the orthogonal projection of
  // that point back onto the line.
  const NvGePoint3d aLineOrigin = theLine.pointOnLine();
  const NvGeVector3d aDir = theLine.direction();
  const NvGeVector3d aNormal = normal();
  const NvGePoint3d aPlaneOrigin = pointOnPlane();

  const NvGeVector3d aDelta (aLineOrigin.x - aPlaneOrigin.x,
                             aLineOrigin.y - aPlaneOrigin.y,
                             aLineOrigin.z - aPlaneOrigin.z);
  const double aDist = aNormal.dotProduct (aDelta);
  const NvGePoint3d aOnPlane (aLineOrigin.x - aDist * aNormal.x,
                              aLineOrigin.y - aDist * aNormal.y,
                              aLineOrigin.z - aDist * aNormal.z);
  const double aParam = aDir.dotProduct (NvGeVector3d (aOnPlane.x - aLineOrigin.x,
                                                       aOnPlane.y - aLineOrigin.y,
                                                       aOnPlane.z - aLineOrigin.z));
  thePntOnLine.set (aLineOrigin.x + aParam * aDir.x,
                    aLineOrigin.y + aParam * aDir.y,
                    aLineOrigin.z + aParam * aDir.z);
  (void)theTol;
  return aOnPlane;
}

//=================================================================================================

NvGePoint3d NvGePlanarEnt::closestPointToPlanarEnt (const NvGePlanarEnt& theOtherPln,
                                                    NvGePoint3d& thePntOnOtherPln,
                                                    const NvGeTol& theTol) const
{
  // Intersecting planes: the closest points collapse onto the intersection
  // line (any point of it); parallel planes use the perpendicular foot.
  const NvGeVector3d aN1 = normal();
  const NvGeVector3d aN2 = theOtherPln.normal();
  NvGePoint3d anOrigin2 = theOtherPln.pointOnPlane();
  const NvGePoint3d anOrigin1 = pointOnPlane();

  NvGeVector3d aCross (aN1.y * aN2.z - aN1.z * aN2.y,
                       aN1.z * aN2.x - aN1.x * aN2.z,
                       aN1.x * aN2.y - aN1.y * aN2.x);
  const double aCrossLen = std::sqrt (aCross.lengthSqrd());
  if (aCrossLen <= theTol.equalVector())
  {
    // Parallel planes: project this origin onto the other plane.
    const NvGeVector3d aDelta (anOrigin1.x - anOrigin2.x,
                               anOrigin1.y - anOrigin2.y,
                               anOrigin1.z - anOrigin2.z);
    const double aDist = aN2.dotProduct (aDelta);
    thePntOnOtherPln.set (anOrigin1.x - aDist * aN2.x,
                          anOrigin1.y - aDist * aN2.y,
                          anOrigin1.z - aDist * aN2.z);
    return anOrigin1;
  }
  // A point of the intersection line: solve n1.x = d1, n2.x = d2 with the
  // point closest to this plane's origin.
  const double aD1 = aN1.dotProduct (NvGeVector3d (anOrigin1.x, anOrigin1.y, anOrigin1.z));
  const double aD2 = aN2.dotProduct (NvGeVector3d (anOrigin2.x, anOrigin2.y, anOrigin2.z));
  aCross = aCross / aCrossLen;
  const double aDet = aCross.dotProduct (aCross);
  // p = (d1 * (n2 x c) + d2 * (c x n1)) / |c|^2
  const NvGeVector3d aN2xC (aN2.y * aCross.z - aN2.z * aCross.y,
                            aN2.z * aCross.x - aN2.x * aCross.z,
                            aN2.x * aCross.y - aN2.y * aCross.x);
  const NvGeVector3d aCxN1 (aCross.y * aN1.z - aCross.z * aN1.y,
                            aCross.z * aN1.x - aCross.x * aN1.z,
                            aCross.x * aN1.y - aCross.y * aN1.x);
  const double anInvDet = 1.0 / aDet;
  thePntOnOtherPln.set ((aD1 * aN2xC.x + aD2 * aCxN1.x) * anInvDet,
                        (aD1 * aN2xC.y + aD2 * aCxN1.y) * anInvDet,
                        (aD1 * aN2xC.z + aD2 * aCxN1.z) * anInvDet);
  return thePntOnOtherPln;
}

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::isParallelTo (const NvGeLinearEnt3d& theLinEnt,
                                            const NvGeTol& theTol) const
{
  return std::abs (normal().dotProduct (theLinEnt.direction())) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::isParallelTo (const NvGePlanarEnt& theOtherPlnEnt,
                                            const NvGeTol& theTol) const
{
  const NvGeVector3d aN1 = normal();
  const NvGeVector3d aN2 = theOtherPlnEnt.normal();
  const NvGeVector3d aCross (aN1.y * aN2.z - aN1.z * aN2.y,
                             aN1.z * aN2.x - aN1.x * aN2.z,
                             aN1.x * aN2.y - aN1.y * aN2.x);
  return std::sqrt (aCross.lengthSqrd()) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::isPerpendicularTo (const NvGeLinearEnt3d& theLinEnt,
                                                 const NvGeTol& theTol) const
{
  return std::abs (normal().dotProduct (theLinEnt.direction()) - 1.0) <= theTol.equalVector()
      || std::abs (normal().dotProduct (theLinEnt.direction()) + 1.0) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::isPerpendicularTo (const NvGePlanarEnt& theLinEnt,
                                                 const NvGeTol& theTol) const
{
  return std::abs (normal().dotProduct (theLinEnt.normal())) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGePlanarEnt::isCoplanarTo (const NvGePlanarEnt& theOtherPlnEnt,
                                            const NvGeTol& theTol) const
{
  if (!isParallelTo (theOtherPlnEnt, theTol))
  {
    return false;
  }
  const NvGeVector3d aNormal = normal();
  const NvGePoint3d anOrigin1 = pointOnPlane();
  const NvGePoint3d anOrigin2 = theOtherPlnEnt.pointOnPlane();
  const double aDist = aNormal.dotProduct (NvGeVector3d (anOrigin2.x - anOrigin1.x,
                                                         anOrigin2.y - anOrigin1.y,
                                                         anOrigin2.z - anOrigin1.z));
  return std::abs (aDist) <= theTol.equalPoint();
}

//=================================================================================================

void NvGePlanarEnt::get (NvGePoint3d& theOrigin, NvGeVector3d& theUVec, NvGeVector3d& theVVec) const
{
  getCoordSystem (theOrigin, theUVec, theVVec);
}

//=================================================================================================

void NvGePlanarEnt::get (NvGePoint3d& thePnt1, NvGePoint3d& thePnt2, NvGePoint3d& thePnt3) const
{
  NvGePoint3d anOrigin;
  NvGeVector3d anU, aV;
  getCoordSystem (anOrigin, anU, aV);
  thePnt1 = anOrigin;
  thePnt2.set (anOrigin.x + anU.x, anOrigin.y + anU.y, anOrigin.z + anU.z);
  thePnt3.set (anOrigin.x + aV.x, anOrigin.y + aV.y, anOrigin.z + aV.z);
}

//=================================================================================================

NvGePoint3d NvGePlanarEnt::pointOnPlane() const
{
  const gp_Ax3 anAx3 = PlaneOf (mpImpEnt)->Position();
  const gp_Pnt aLocation = anAx3.Location();
  return NvGePoint3d (aLocation.X(), aLocation.Y(), aLocation.Z());
}

//=================================================================================================

NvGeVector3d NvGePlanarEnt::normal() const
{
  const gp_Ax3 anAx3 = PlaneOf (mpImpEnt)->Position();
  const gp_Dir aDir = anAx3.Direction();
  return NvGeVector3d (aDir.X(), aDir.Y(), aDir.Z());
}

//=================================================================================================

void NvGePlanarEnt::getCoefficients (double& theA, double& theB, double& theC, double& theD) const
{
  // Unit-normal plane equation: a*x + b*y + c*z + d = 0 with d = -(n . o).
  const gp_Pln aPln = PlaneOf (mpImpEnt)->Pln();
  aPln.Coefficients (theA, theB, theC, theD);
}

//=================================================================================================

void NvGePlanarEnt::getCoordSystem (NvGePoint3d& theOrigin, NvGeVector3d& theAxis1,
                                    NvGeVector3d& theAxis2) const
{
  const gp_Ax3 anAx3 = PlaneOf (mpImpEnt)->Position();
  const gp_Pnt aLocation = anAx3.Location();
  const gp_Dir anX = anAx3.XDirection();
  const gp_Dir aY = anAx3.YDirection();
  theOrigin.set (aLocation.X(), aLocation.Y(), aLocation.Z());
  theAxis1.set (anX.X(), anX.Y(), anX.Z());
  theAxis2.set (aY.X(), aY.Y(), aY.Z());
}

//=================================================================================================

NvGePlanarEnt& NvGePlanarEnt::operator = (const NvGePlanarEnt& theSrc)
{
  NvGeSurface::operator= (theSrc);
  return *this;
}

//=================================================================================================

NvGePlanarEnt::NvGePlanarEnt()
: NvGeSurface()
{
}

//=================================================================================================

NvGePlanarEnt::NvGePlanarEnt (const NvGePlanarEnt& theSrc)
: NvGeSurface (theSrc)
{
}
