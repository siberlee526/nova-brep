// gelnsg3d.cpp - implementation of NvGeLineSeg3d.
//
// A segment is stored as a Geom_TrimmedCurve over a Geom_Line whose unit
// direction points from the start point to the end point; the parameter
// range is [0, length], so the curve parameter equals the distance from
// the start point. Zero-length segments cannot be represented; every
// defining input is validated and a degenerate result throws NvException.

#include <gelnsg3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <geimpdata.h>

#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <gp.hxx>
#include <cmath>
#include <string>

namespace
{

// Returns the trimmed carrier curve of the segment; throws when the impl
// does not hold one.
occ::handle<Geom_TrimmedCurve> SegmentCurveOf (const occ::handle<Standard_Transient>& theGeom,
                                               const char* theMethod)
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    occ::down_cast<Geom_TrimmedCurve> (occ::down_cast<Geom_Curve> (theGeom));
  if (aCurve.IsNull())
  {
    throw NvException (std::string (theMethod) + ": the entity has no segment geometry");
  }
  return aCurve;
}

} // namespace

//=================================================================================================

NvGeLineSeg3d::NvGeLineSeg3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  // The default segment is the unit segment from the origin along +X:
  // a zero-length segment is forbidden by the storage convention.
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (0.0, 0.0, 0.0),
                                                  gp_Dir (1.0, 0.0, 0.0)));
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg3d,
               occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0, 1.0)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLineSeg3d::NvGeLineSeg3d (const NvGeLineSeg3d& theLine)
: NvGeLinearEnt3d (theLine)
{
}

//=================================================================================================

NvGeLineSeg3d::NvGeLineSeg3d (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLineSeg3d::NvGeLineSeg3d(): the vector is degenerate");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (thePnt.x, thePnt.y, thePnt.z),
                                                  gp_Dir (theVec.x, theVec.y, theVec.z)));
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg3d,
               occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0, aLen)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLineSeg3d::NvGeLineSeg3d (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  const double aLen = std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLineSeg3d::NvGeLineSeg3d(): the points are coincident");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (thePnt1.x, thePnt1.y, thePnt1.z),
                                                  gp_Dir (aDx, aDy, aDz)));
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg3d,
               occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0, aLen)));
  mpImpEnt->Ref();
}

//=================================================================================================

void NvGeLineSeg3d::getBisector (NvGePlane& thePlane) const
{
  // The bisector plane passes through the midpoint and is perpendicular
  // to the segment.
  thePlane = NvGePlane (midPoint(), direction());
}

//=================================================================================================

NvGePoint3d NvGeLineSeg3d::baryComb (double theBlendCoeff) const
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    SegmentCurveOf (mpImpEnt->Geom(), "NvGeLineSeg3d::baryComb()");
  const double aFirst = aCurve->FirstParameter();
  const double aLast  = aCurve->LastParameter();
  const gp_Pnt aPnt = aCurve->Value (aFirst + theBlendCoeff * (aLast - aFirst));
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGeLineSeg3d::startPoint () const
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    SegmentCurveOf (mpImpEnt->Geom(), "NvGeLineSeg3d::startPoint()");
  const gp_Pnt aPnt = aCurve->Value (aCurve->FirstParameter());
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGeLineSeg3d::midPoint () const
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    SegmentCurveOf (mpImpEnt->Geom(), "NvGeLineSeg3d::midPoint()");
  const gp_Pnt aPnt = aCurve->Value (0.5 * (aCurve->FirstParameter() + aCurve->LastParameter()));
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGeLineSeg3d::endPoint () const
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    SegmentCurveOf (mpImpEnt->Geom(), "NvGeLineSeg3d::endPoint()");
  const gp_Pnt aPnt = aCurve->Value (aCurve->LastParameter());
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

double NvGeLineSeg3d::length () const
{
  const occ::handle<Geom_TrimmedCurve> aCurve =
    SegmentCurveOf (mpImpEnt->Geom(), "NvGeLineSeg3d::length()");
  return aCurve->LastParameter() - aCurve->FirstParameter();
}

//=================================================================================================

double NvGeLineSeg3d::length (double theFromParam, double theToParam, double /*theTol*/) const
{
  return std::abs (theToParam - theFromParam);
}

//=================================================================================================

NvGeLineSeg3d& NvGeLineSeg3d::set (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLineSeg3d::set(): the vector is degenerate");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (thePnt.x, thePnt.y, thePnt.z),
                                                  gp_Dir (theVec.x, theVec.y, theVec.z)));
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0, aLen)));
  return *this;
}

//=================================================================================================

NvGeLineSeg3d& NvGeLineSeg3d::set (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  const double aLen = std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLineSeg3d::set(): the points are coincident");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (thePnt1.x, thePnt1.y, thePnt1.z),
                                                  gp_Dir (aDx, aDy, aDz)));
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0, aLen)));
  return *this;
}

//=================================================================================================

NvGeLineSeg3d& NvGeLineSeg3d::set (const NvGeCurve3d& theCurve1, const NvGeCurve3d& theCurve2,
                                   double& theParam1, double& theParam2,
                                   Adesk::Boolean& theSuccess)
{
  // Shortest segment between the two curves: take the closest pair of
  // points and report their parameters; touching curves produce no
  // positive-length segment.
  NvGePoint3d aPntOnCurve2;
  const NvGePoint3d aPntOnCurve1 = theCurve1.closestPointTo (theCurve2, aPntOnCurve2,
                                                             NvGeContext::gTol);
  const gp_Pnt aPnt1 (aPntOnCurve1.x, aPntOnCurve1.y, aPntOnCurve1.z);
  const gp_Pnt aPnt2 (aPntOnCurve2.x, aPntOnCurve2.y, aPntOnCurve2.z);
  if (aPnt1.Distance (aPnt2) <= NvGeContext::gTol.equalPoint())
  {
    theSuccess = false;
    return *this;
  }
  theParam1 = theCurve1.paramOf (aPntOnCurve1, NvGeContext::gTol);
  theParam2 = theCurve2.paramOf (aPntOnCurve2, NvGeContext::gTol);
  set (aPntOnCurve1, aPntOnCurve2);
  theSuccess = true;
  return *this;
}

//=================================================================================================

NvGeLineSeg3d& NvGeLineSeg3d::set (const NvGeCurve3d& theCurve, const NvGePoint3d& thePnt,
                                   double& theParam, Adesk::Boolean& theSuccess)
{
  // Shortest segment from a point to a curve: the closest point on the
  // curve defines the far endpoint and its parameter.
  const NvGePoint3d aPntOnCurve = theCurve.closestPointTo (thePnt, NvGeContext::gTol);
  const gp_Pnt aClosest (aPntOnCurve.x, aPntOnCurve.y, aPntOnCurve.z);
  const gp_Pnt aStart (thePnt.x, thePnt.y, thePnt.z);
  if (aStart.Distance (aClosest) <= NvGeContext::gTol.equalPoint())
  {
    theSuccess = false;
    return *this;
  }
  theParam = theCurve.paramOf (aPntOnCurve, NvGeContext::gTol);
  set (thePnt, aPntOnCurve);
  theSuccess = true;
  return *this;
}

//=================================================================================================

NvGeLineSeg3d& NvGeLineSeg3d::operator = (const NvGeLineSeg3d& theLine)
{
  NvGeEntity3d::operator = (theLine);
  return *this;
}
