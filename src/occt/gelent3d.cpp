// gelent3d.cpp - implementation of NvGeLinearEnt3d, the 3d linear entity base.
//
// The carrier line lives in the impl geometry as a Geom_Line (infinite
// line) or a Geom_TrimmedCurve over a Geom_Line (ray: [0, +inf);
// segment: [0, length]). The curve parameter equals the distance from the
// origin point along the (unit) direction, so all queries are solved in
// closed form on that parameter axis; directions are normalized gp_Dir
// objects taken from the carrier line.

#include <gelent3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geline3d.h>
#include <gelnsg3d.h>
#include <geimpdata.h>
#include <geplanar.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>
#include <geray3d.h>

#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Dir.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <limits>
#include <utility>

namespace
{

// Returns the Geom_Line carrying the entity geometry: either the stored
// curve itself or the basis of a trimmed curve. Null when the geometry is
// not line-based.
occ::handle<Geom_Line> LineOf (const occ::handle<Standard_Transient>& theGeom)
{
  const occ::handle<Geom_Curve> aCurve = occ::down_cast<Geom_Curve> (theGeom);
  if (aCurve.IsNull())
  {
    return occ::handle<Geom_Line>();
  }
  const occ::handle<Geom_Line> aLine = occ::down_cast<Geom_Line> (aCurve);
  if (!aLine.IsNull())
  {
    return aLine;
  }
  const occ::handle<Geom_TrimmedCurve> aTrimmed = occ::down_cast<Geom_TrimmedCurve> (aCurve);
  if (aTrimmed.IsNull())
  {
    return occ::handle<Geom_Line>();
  }
  return occ::down_cast<Geom_Line> (aTrimmed->BasisCurve());
}

// Parameter range of a linear entity on its carrier line (parameter =
// distance from the origin point): lines are unbounded, a ray is
// [0, +inf), a segment is [0, length].
void LinearRangeOf (NvGe::EntityId theType, const occ::handle<Standard_Transient>& theGeom,
                    double& theFirst, double& theLast)
{
  theFirst = -std::numeric_limits<double>::infinity();
  theLast  =  std::numeric_limits<double>::infinity();
  if (theType != NvGe::kRay3d && theType != NvGe::kLineSeg3d)
  {
    return;
  }
  const occ::handle<Geom_TrimmedCurve> aTrimmed =
    occ::down_cast<Geom_TrimmedCurve> (occ::down_cast<Geom_Curve> (theGeom));
  if (!aTrimmed.IsNull())
  {
    theFirst = aTrimmed->FirstParameter();
    theLast  = aTrimmed->LastParameter();
    return;
  }
  theFirst = 0.0; // bounded entity without trimmed storage starts at its origin
}

bool InRange (double theParam, double theFirst, double theLast, double theTol)
{
  return theParam >= theFirst - theTol && theParam <= theLast + theTol;
}

// Point at the given distance parameter on the carrier line.
NvGePoint3d PointAt (const gp_Pnt& theOrigin, const gp_Dir& theDir, double theParam)
{
  const gp_XYZ aXYZ = theOrigin.XYZ() + theParam * theDir.XYZ();
  return NvGePoint3d (aXYZ.X(), aXYZ.Y(), aXYZ.Z());
}

} // namespace

//=================================================================================================

NvGeLinearEnt3d::NvGeLinearEnt3d ()
: NvGeCurve3d ()
{
}

//=================================================================================================

NvGeLinearEnt3d::NvGeLinearEnt3d (const NvGeLinearEnt3d& theSrc)
: NvGeCurve3d (theSrc)
{
  // The NvGeCurve3d copy constructor does not propagate the implementation
  // yet, so adopt a shallow clone of the source impl here: copies share the
  // geometry and stay copy-on-write.
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = theSrc.mpImpEnt != nullptr ? theSrc.mpImpEnt->CloneShallow() : nullptr;
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
}

//=================================================================================================

NvGePoint3d NvGeLinearEnt3d::pointOnLine () const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    throw NvException ("NvGeLinearEnt3d::pointOnLine(): the entity has no line geometry");
  }
  const gp_Pnt& aLoc = aLine->Position().Location();
  return NvGePoint3d (aLoc.X(), aLoc.Y(), aLoc.Z());
}

//=================================================================================================

NvGeVector3d NvGeLinearEnt3d::direction () const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    throw NvException ("NvGeLinearEnt3d::direction(): the entity has no line geometry");
  }
  const gp_Dir& aDir = aLine->Position().Direction();
  return NvGeVector3d (aDir.X(), aDir.Y(), aDir.Z());
}

//=================================================================================================

void NvGeLinearEnt3d::getLine (NvGeLine3d& theLine) const
{
  theLine = NvGeLine3d (pointOnLine(), direction());
}

//=================================================================================================

NvGeLinearEnt3d& NvGeLinearEnt3d::operator = (const NvGeLinearEnt3d& theLine)
{
  NvGeEntity3d::operator = (theLine);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::intersectWith (const NvGeLinearEnt3d& theLine, NvGePoint3d& theIntPnt,
                                               const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine1->Position().Location();
  const gp_Dir& aDir1 = aLine1->Position().Direction();
  const gp_Pnt& aPnt2 = aLine2->Position().Location();
  const gp_Dir& aDir2 = aLine2->Position().Direction();

  const gp_Vec aD1 (aDir1.XYZ());
  const gp_Vec aD2 (aDir2.XYZ());
  const gp_Vec aW (aPnt2.XYZ() - aPnt1.XYZ());
  const gp_Vec aCross = aD1.Crossed (aD2);
  if (aCross.Magnitude() <= theTol.equalVector())
  {
    return false; // parallel or colinear: no single intersection point
  }
  const double aDot   = aD1.Dot (aD2);
  const double aDenom = 1.0 - aDot * aDot;
  const double aW1 = aW.Dot (aD1);
  const double aW2 = aW.Dot (aD2);
  const double aS = (aW1 - aDot * aW2) / aDenom;
  const double aT = (aDot * aW1 - aW2) / aDenom;

  double aFirst1 = 0.0, aLast1 = 0.0, aFirst2 = 0.0, aLast2 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  LinearRangeOf (theLine.type(), theLine.mpImpEnt->Geom(), aFirst2, aLast2);
  if (!InRange (aS, aFirst1, aLast1, theTol.equalPoint())
   || !InRange (aT, aFirst2, aLast2, theTol.equalPoint()))
  {
    return false;
  }

  // Non-parallel infinite lines always meet in the plane spanned by their
  // directions; a remaining gap means the lines are skew.
  const gp_Pnt aClosest1 (aPnt1.XYZ() + aS * aD1.XYZ());
  const gp_Pnt aClosest2 (aPnt2.XYZ() + aT * aD2.XYZ());
  if (aClosest1.Distance (aClosest2) > theTol.equalPoint())
  {
    return false;
  }
  const gp_XYZ aMid = (aClosest1.XYZ() + aClosest2.XYZ()).Multiplied (0.5);
  theIntPnt = NvGePoint3d (aMid.X(), aMid.Y(), aMid.Z());
  return true;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::intersectWith (const NvGePlanarEnt& thePlane, NvGePoint3d& theIntPnt,
                                               const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine->Position().Location();
  const gp_Dir& aDir1 = aLine->Position().Direction();
  const NvGePoint3d  aPlnPnt = thePlane.pointOnPlane();
  const NvGeVector3d aNormal = thePlane.normal();
  const gp_Vec aN (aNormal.x, aNormal.y, aNormal.z);
  const gp_Vec aD1 (aDir1.XYZ());
  const double aDenom = aD1.Dot (aN);
  if (std::abs (aDenom) <= theTol.equalVector())
  {
    return false; // the line is parallel to (or contained in) the plane
  }
  const gp_Vec aW (aPnt1.XYZ() - gp_Pnt (aPlnPnt.x, aPlnPnt.y, aPlnPnt.z).XYZ());
  const double aS = -aW.Dot (aN) / aDenom;

  double aFirst1 = 0.0, aLast1 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  if (!InRange (aS, aFirst1, aLast1, theTol.equalPoint()))
  {
    return false;
  }
  theIntPnt = PointAt (aPnt1, aDir1, aS);
  return true;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::projIntersectWith (const NvGeLinearEnt3d& theLine,
                                                   const NvGeVector3d& theProjDir,
                                                   NvGePoint3d& thePntOnThisLine,
                                                   NvGePoint3d& thePntOnOtherLine,
                                                   const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const double aProjLen = std::sqrt (theProjDir.x * theProjDir.x
                                   + theProjDir.y * theProjDir.y
                                   + theProjDir.z * theProjDir.z);
  if (aProjLen <= gp::Resolution())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine1->Position().Location();
  const gp_Dir& aDir1 = aLine1->Position().Direction();
  const gp_Pnt& aPnt2 = aLine2->Position().Location();
  const gp_Dir& aDir2 = aLine2->Position().Direction();
  const gp_Dir aProj (theProjDir.x / aProjLen, theProjDir.y / aProjLen, theProjDir.z / aProjLen);

  // Solve s * D1 - t * D2 - l * P = P2 - P1: the projected rays meet when
  // the connection vector of the solutions is parallel to the projection
  // direction. Columns of the system: D1, -D2, -P.
  const gp_Vec aD1 (aDir1.XYZ());
  const gp_Vec aD2 (aDir2.XYZ());
  const gp_Vec aW (aPnt2.XYZ() - aPnt1.XYZ());
  const gp_Mat aM ( aD1.X(), -aD2.X(), -aProj.X(),
                    aD1.Y(), -aD2.Y(), -aProj.Y(),
                    aD1.Z(), -aD2.Z(), -aProj.Z());
  const double aDet = aM.Determinant();
  if (std::abs (aDet) <= theTol.equalVector())
  {
    return false; // the projected directions are parallel
  }
  gp_Mat aSystemS = aM;
  aSystemS.SetCol (1, aW.XYZ());
  const double aS = aSystemS.Determinant() / aDet;
  gp_Mat aSystemT = aM;
  aSystemT.SetCol (2, aW.XYZ());
  const double aT = aSystemT.Determinant() / aDet;

  double aFirst1 = 0.0, aLast1 = 0.0, aFirst2 = 0.0, aLast2 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  LinearRangeOf (theLine.type(), theLine.mpImpEnt->Geom(), aFirst2, aLast2);
  if (!InRange (aS, aFirst1, aLast1, theTol.equalPoint())
   || !InRange (aT, aFirst2, aLast2, theTol.equalPoint()))
  {
    return false;
  }
  thePntOnThisLine  = PointAt (aPnt1, aDir1, aS);
  thePntOnOtherLine = PointAt (aPnt2, aDir2, aT);
  return true;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::overlap (const NvGeLinearEnt3d& theLine, NvGeLinearEnt3d*& theOverlap,
                                         const NvGeTol& theTol) const
{
  theOverlap = nullptr;
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine1->Position().Location();
  const gp_Dir& aDir1 = aLine1->Position().Direction();
  const gp_Pnt& aPnt2 = aLine2->Position().Location();
  const gp_Dir& aDir2 = aLine2->Position().Direction();

  const gp_Vec aD1 (aDir1.XYZ());
  const gp_Vec aD2 (aDir2.XYZ());
  const gp_Vec aW (aPnt2.XYZ() - aPnt1.XYZ());
  if (aD1.Crossed (aD2).Magnitude() > theTol.equalVector())
  {
    return false; // not parallel
  }
  if (std::sqrt (aW.Crossed (aD1).SquareMagnitude()) > theTol.equalPoint())
  {
    return false; // parallel but not colinear
  }

  double aFirst1 = 0.0, aLast1 = 0.0, aFirst2 = 0.0, aLast2 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  LinearRangeOf (theLine.type(), theLine.mpImpEnt->Geom(), aFirst2, aLast2);
  if (aFirst1 == -std::numeric_limits<double>::infinity()
   && aLast1 == std::numeric_limits<double>::infinity()
   && aFirst2 == -std::numeric_limits<double>::infinity()
   && aLast2 == std::numeric_limits<double>::infinity())
  {
    theOverlap = new NvGeLine3d (pointOnLine(), direction());
    return true;
  }

  // Map the other entity's range onto this carrier line's parameter axis.
  const double aSign = aD1.Dot (aD2) > 0.0 ? 1.0 : -1.0;
  const double anOffset = aW.Dot (aD1);
  double aOtherFirst = anOffset + aSign * aFirst2;
  double aOtherLast  = anOffset + aSign * aLast2;
  if (aOtherFirst > aOtherLast)
  {
    std::swap (aOtherFirst, aOtherLast);
  }
  const double aLow  = aFirst1 > aOtherFirst ? aFirst1 : aOtherFirst;
  const double aHigh = aLast1  < aOtherLast  ? aLast1  : aOtherLast;
  if (!(aHigh - aLow > theTol.equalPoint()))
  {
    return false; // empty or point-touching overlap
  }
  if (aLow == -std::numeric_limits<double>::infinity())
  {
    theOverlap = new NvGeRay3d (PointAt (aPnt1, aDir1, aHigh),
                                NvGeVector3d (-aDir1.X(), -aDir1.Y(), -aDir1.Z()));
  }
  else if (aHigh == std::numeric_limits<double>::infinity())
  {
    theOverlap = new NvGeRay3d (PointAt (aPnt1, aDir1, aLow),
                                NvGeVector3d (aDir1.X(), aDir1.Y(), aDir1.Z()));
  }
  else
  {
    theOverlap = new NvGeLineSeg3d (PointAt (aPnt1, aDir1, aLow), PointAt (aPnt1, aDir1, aHigh));
  }
  return true;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isOn (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  double aParam = 0.0;
  return isOn (thePnt, aParam, theTol);
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isOn (const NvGePoint3d& thePnt, double& theParam,
                                      const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine->Position().Location();
  const gp_Dir& aDir1 = aLine->Position().Direction();
  const gp_Vec aD1 (aDir1.XYZ());
  const gp_Vec aW (thePnt.x - aPnt1.X(), thePnt.y - aPnt1.Y(), thePnt.z - aPnt1.Z());
  const double aS = aW.Dot (aD1);
  if (aW.Subtracted (aS * aD1).Magnitude() > theTol.equalPoint())
  {
    return false;
  }
  double aFirst1 = 0.0, aLast1 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  if (!InRange (aS, aFirst1, aLast1, theTol.equalPoint()))
  {
    return false;
  }
  theParam = aS;
  return true;
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isOn (double theParam, const NvGeTol& theTol) const
{
  double aFirst1 = 0.0, aLast1 = 0.0;
  LinearRangeOf (type(), mpImpEnt->Geom(), aFirst1, aLast1);
  return InRange (theParam, aFirst1, aLast1, theTol.equalPoint());
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isOn (const NvGePlane& thePlane, const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    return false;
  }
  const gp_Pnt& aPnt1 = aLine->Position().Location();
  const gp_Dir& aDir1 = aLine->Position().Direction();
  const NvGePoint3d  aPlnPnt = thePlane.pointOnPlane();
  const NvGeVector3d aNormal = thePlane.normal();
  const gp_Vec aN (aNormal.x, aNormal.y, aNormal.z);
  if (std::abs (gp_Vec (aDir1.XYZ()).Dot (aN)) > theTol.equalVector())
  {
    return false; // the direction leaves the plane
  }
  const gp_Vec aW (aPnt1.XYZ() - gp_Pnt (aPlnPnt.x, aPlnPnt.y, aPlnPnt.z).XYZ());
  return std::abs (aW.Dot (aN)) <= theTol.equalPoint();
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isParallelTo (const NvGeLinearEnt3d& theLine,
                                              const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const gp_Vec aCross = gp_Vec (aLine1->Position().Direction().XYZ())
                       .Crossed (gp_Vec (aLine2->Position().Direction().XYZ()));
  return aCross.Magnitude() <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isParallelTo (const NvGePlanarEnt& thePlane,
                                              const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    return false;
  }
  const NvGeVector3d aNormal = thePlane.normal();
  const gp_Vec aN (aNormal.x, aNormal.y, aNormal.z);
  const double aDot = gp_Vec (aLine->Position().Direction().XYZ()).Dot (aN);
  return std::abs (aDot) <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isPerpendicularTo (const NvGeLinearEnt3d& theLine,
                                                   const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const double aDot = gp_Vec (aLine1->Position().Direction().XYZ())
                     .Dot (gp_Vec (aLine2->Position().Direction().XYZ()));
  return std::abs (aDot) <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isPerpendicularTo (const NvGePlanarEnt& thePlane,
                                                   const NvGeTol& theTol) const
{
  const occ::handle<Geom_Line> aLine = LineOf (mpImpEnt->Geom());
  if (aLine.IsNull())
  {
    return false;
  }
  const NvGeVector3d aNormal = thePlane.normal();
  const gp_Vec aCross = gp_Vec (aLine->Position().Direction().XYZ()).Crossed (
    gp_Vec (aNormal.x, aNormal.y, aNormal.z));
  return aCross.Magnitude() <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGeLinearEnt3d::isColinearTo (const NvGeLinearEnt3d& theLine,
                                              const NvGeTol& theTol) const
{
  if (!isParallelTo (theLine, theTol))
  {
    return false;
  }
  const occ::handle<Geom_Line> aLine1 = LineOf (mpImpEnt->Geom());
  const occ::handle<Geom_Line> aLine2 = LineOf (theLine.mpImpEnt->Geom());
  if (aLine1.IsNull() || aLine2.IsNull())
  {
    return false;
  }
  const gp_Vec aW (aLine2->Position().Location().XYZ() - aLine1->Position().Location().XYZ());
  return std::sqrt (aW.Crossed (gp_Vec (aLine1->Position().Direction().XYZ())).SquareMagnitude())
       <= theTol.equalPoint();
}

//=================================================================================================

void NvGeLinearEnt3d::getPerpPlane (const NvGePoint3d& thePnt, NvGePlane& thePlane) const
{
  thePlane = NvGePlane (thePnt, direction());
}
