// gelnsg2d.cpp - implementation of NvGeLineSeg2d based on OCCT Geom2d
// classes.
//
// A segment is stored as a Geom2d_TrimmedCurve wrapping a Geom2d_Line that
// is built with the unit direction from pnt1 to pnt2; the parameter range
// is [0, length], so the parameter measures the distance from the start
// point (ARX semantics). A zero-length segment has no direction and is
// rejected with NvException. Modifiers follow the copy-on-write discipline
// of geimpdata.h.

#include <gelnsg2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_ExtremaCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <string>

namespace
{

// Point wrapper for the OCCT evaluations below.
NvGePoint2d PointOf (const gp_Pnt2d& thePnt)
{
  return NvGePoint2d (thePnt.X(), thePnt.Y());
}

// Creates the segment geometry between two distinct points. The unit
// direction and the length are pre-checked, so the trimmed curve can never
// raise (its range stays inside the line domain).
occ::handle<Geom2d_Curve> SegmentBetween (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2,
                                          const char* theMethod)
{
  const double aLen = std::sqrt ((thePnt2 - thePnt1).lengthSqrd());
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeLineSeg2d::") + theMethod
                       + "(): the segment is degenerate (zero length)");
  }
  const NvGeVector2d aVec (thePnt2.x - thePnt1.x, thePnt2.y - thePnt1.y);
  const occ::handle<Geom2d_Line> aLine (new Geom2d_Line (gp_Pnt2d (thePnt1.x, thePnt1.y),
                                                         gp_Dir2d (aVec.x, aVec.y)));
  return occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (aLine, 0.0, aLen));
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom2d_Curve>& theCurve)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theCurve);
}

// Returns the trimmed curve of a segment entity; reports a missing or
// foreign impl geometry with the project error type.
occ::handle<Geom2d_TrimmedCurve> TrimmedOf (const occ::handle<Standard_Transient>& theGeom,
                                            const char* theMethod)
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (theGeom);
  if (aTrimmed.IsNull())
  {
    throw NvException (std::string ("NvGeLineSeg2d::") + theMethod
                       + "(): the entity has no segment geometry");
  }
  return aTrimmed;
}

}

//=================================================================================================

NvGeLineSeg2d::NvGeLineSeg2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg2d,
                                  SegmentBetween (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 0.0),
                                                  "NvGeLineSeg2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLineSeg2d::NvGeLineSeg2d (const NvGeLineSeg2d& theSrc)
: NvGeLinearEnt2d (theSrc)
{
}

//=================================================================================================

NvGeLineSeg2d::NvGeLineSeg2d (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg2d, SegmentBetween (thePnt1, thePnt2, "NvGeLineSeg2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLineSeg2d::NvGeLineSeg2d (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLineSeg2d,
                                  SegmentBetween (thePnt, thePnt + theVec, "NvGeLineSeg2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLineSeg2d& NvGeLineSeg2d::set (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  ReplaceGeometry (mpImpEnt, SegmentBetween (thePnt, thePnt + theVec, "set"));
  return *this;
}

//=================================================================================================

NvGeLineSeg2d& NvGeLineSeg2d::set (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  ReplaceGeometry (mpImpEnt, SegmentBetween (thePnt1, thePnt2, "set"));
  return *this;
}

//=================================================================================================

NvGeLineSeg2d& NvGeLineSeg2d::set (const NvGeCurve2d& theCurve1, const NvGeCurve2d& theCurve2,
                                   double& theParam1, double& theParam2,
                                   Adesk::Boolean& theSuccess)
{
  theSuccess = false;
  try
  {
    // A foreign entity's protected impl is reachable only through objects of
    // this class, so each argument's impl is borrowed via the public base
    // assignment into scratch shells that release it again on destruction.
    NvGeLineSeg2d aScratch1;
    aScratch1.NvGeEntity2d::operator= (theCurve1);
    NvGeLineSeg2d aScratch2;
    aScratch2.NvGeEntity2d::operator= (theCurve2);
    const occ::handle<Geom2d_Curve> aCurve1 = NvGeCurve2dOf (aScratch1.mpImpEnt);
    const occ::handle<Geom2d_Curve> aCurve2 = NvGeCurve2dOf (aScratch2.mpImpEnt);
    const Geom2dAPI_ExtremaCurveCurve anExtrema (aCurve1, aCurve2,
                                                 aCurve1->FirstParameter(), aCurve1->LastParameter(),
                                                 aCurve2->FirstParameter(), aCurve2->LastParameter());
    if (anExtrema.NbExtrema() == 0)
    {
      return *this;
    }
    double aParam1 = 0.0;
    double aParam2 = 0.0;
    anExtrema.LowerDistanceParameters (aParam1, aParam2);
    const gp_Pnt2d aPnt1 = aCurve1->Value (aParam1);
    const gp_Pnt2d aPnt2 = aCurve2->Value (aParam2);
    if (aPnt1.Distance (aPnt2) <= gp::Resolution())
    {
      return *this; // the curves touch: no non-degenerate segment exists
    }
    theParam1 = aParam1;
    theParam2 = aParam2;
    ReplaceGeometry (mpImpEnt, SegmentBetween (PointOf (aPnt1), PointOf (aPnt2), "set"));
    theSuccess = true;
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  return *this;
}

//=================================================================================================

NvGeLineSeg2d& NvGeLineSeg2d::set (const NvGeCurve2d& theCurve, const NvGePoint2d& thePoint,
                                   double& theParam, Adesk::Boolean& theSuccess)
{
  theSuccess = false;
  try
  {
    // See set (curve1, curve2): the argument's impl is borrowed through a
    // scratch shell of this class.
    NvGeLineSeg2d aScratch;
    aScratch.NvGeEntity2d::operator= (theCurve);
    const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (aScratch.mpImpEnt);
    const Geom2dAPI_ProjectPointOnCurve aProjector (gp_Pnt2d (thePoint.x, thePoint.y), aCurve);
    if (aProjector.NbPoints() == 0)
    {
      return *this;
    }
    const double aParam = aProjector.LowerDistanceParameter();
    const gp_Pnt2d aFootPnt = aProjector.NearestPoint();
    if (aFootPnt.Distance (gp_Pnt2d (thePoint.x, thePoint.y)) <= gp::Resolution())
    {
      return *this; // the point lies on the curve: no non-degenerate segment
    }
    theParam = aParam;
    ReplaceGeometry (mpImpEnt, SegmentBetween (thePoint, PointOf (aFootPnt), "set"));
    theSuccess = true;
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  return *this;
}

//=================================================================================================

void NvGeLineSeg2d::getBisector (NvGeLine2d& theLine) const
{
  const NvGePoint2d  aMid = midPoint();
  const NvGeVector2d aDir = direction();
  theLine.set (aMid, NvGeVector2d (-aDir.y, aDir.x));
}

//=================================================================================================

NvGePoint2d NvGeLineSeg2d::baryComb (double theBlendCoeff) const
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = TrimmedOf (mpImpEnt->Geom(), "baryComb");
  const double aFirst = aTrimmed->FirstParameter();
  const double aLast  = aTrimmed->LastParameter();
  return PointOf (aTrimmed->Value (aFirst + theBlendCoeff * (aLast - aFirst)));
}

//=================================================================================================

NvGePoint2d NvGeLineSeg2d::startPoint () const
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = TrimmedOf (mpImpEnt->Geom(), "startPoint");
  return PointOf (aTrimmed->Value (aTrimmed->FirstParameter()));
}

//=================================================================================================

NvGePoint2d NvGeLineSeg2d::midPoint () const
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = TrimmedOf (mpImpEnt->Geom(), "midPoint");
  const double aMid = 0.5 * (aTrimmed->FirstParameter() + aTrimmed->LastParameter());
  return PointOf (aTrimmed->Value (aMid));
}

//=================================================================================================

NvGePoint2d NvGeLineSeg2d::endPoint () const
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = TrimmedOf (mpImpEnt->Geom(), "endPoint");
  return PointOf (aTrimmed->Value (aTrimmed->LastParameter()));
}

//=================================================================================================

double NvGeLineSeg2d::length () const
{
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = TrimmedOf (mpImpEnt->Geom(), "length");
  return aTrimmed->LastParameter() - aTrimmed->FirstParameter();
}

//=================================================================================================

double NvGeLineSeg2d::length (double theFromParam, double theToParam, double theTol) const
{
  // Length of the parameter interval clipped to the segment domain; the
  // parameters may be given in either order and may exceed the domain by
  // theTol without changing the result.
  const double aLen = length();
  const double aFrom = std::max (-theTol, std::min (theFromParam, aLen + theTol));
  const double aTo   = std::max (-theTol, std::min (theToParam,  aLen + theTol));
  return std::abs (aTo - aFrom);
}

//=================================================================================================

NvGeLineSeg2d& NvGeLineSeg2d::operator = (const NvGeLineSeg2d& theSeg)
{
  NvGeEntity2d::operator= (theSeg);
  return *this;
}
