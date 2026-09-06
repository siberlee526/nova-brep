// geplin2d.cpp - implementation of NvGePolyline2d based on a degree-1
// OCCT Geom2d_BSplineCurve.
//
// The fit points are the poles of a clamped degree-1 B-spline: n poles,
// distinct knots 0, 1, ..., n-1 with multiplicity 2 at both ends and 1 in
// between. With that layout the flat knot vector holds n + 2 entries and
// fit point i (0-based) is interpolated exactly at parameter i, which is
// also the curve parameter range [0, n-1].
//
// The knot-vector constructor takes that FLAT clamped knot sequence (length
// = point count + 2), matching the NvGeKnotVector convention used by
// NvGeSplineEnt2d::knots().
//
// The curve approximation constructor flattens bounded curves analytically:
// lines degenerate to their endpoints, circles and ellipse arcs are sampled
// with a step angle derived from the sagitta (deflection) criterion, and an
// existing polyline is copied. Unbounded entities cannot be approximated
// and raise NvException.
//
// The default polyline is a degenerate two-pole line at the origin.

#include <geplin2d.h>

#include <NvException.h>
#include <gearc2d.h>
#include <geell2d.h>
#include <geimpdata.h>
#include <gekvec.h>
#include <gelnsg2d.h>
#include <gepnt2d.h>
#include <gept2dar.h>
#include <gesent2d.h>

#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Geometry.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <string>
#include <vector>

namespace
{

constexpr double THE_TWO_PI = 6.283185307179586476925286766559;

constexpr double THE_FULL_SWEEP_TOL = 1e-12;

// Knot compression tolerance.
constexpr double THE_KNOT_TOL = 1e-12;

// Normalizes an angle into [0, THE_TWO_PI).
double NormalizeAngle (double theAngle)
{
  double aResult = std::fmod (theAngle, THE_TWO_PI);
  if (aResult < 0.0)
  {
    aResult += THE_TWO_PI;
  }
  return aResult;
}

// Returns the B-spline stored in theImp; throws when the geometry is not a
// B-spline.
occ::handle<Geom2d_BSplineCurve> SplineOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (theImp);
  const occ::handle<Geom2d_BSplineCurve> aSpline = occ::down_cast<Geom2d_BSplineCurve> (aCurve);
  if (aSpline.IsNull())
  {
    throw NvException (std::string ("NvGePolyline2d::") + theMethod
                       + "(): the implementation geometry is not a B-spline");
  }
  return aSpline;
}

void CheckIndex (int theIdx, int theCount, const char* theMethod, const char* theWhat)
{
  if (theIdx < 0 || theIdx >= theCount)
  {
    throw NvException (std::string ("NvGePolyline2d::") + theMethod + "(): " + theWhat
                       + " is out of range");
  }
}

// Detaches the impl when shared and deep-copies the B-spline geometry so an
// in-place edit cannot leak into other holders of the handle
// (copy-on-write; see the banner of geimpdata.h).
occ::handle<Geom2d_BSplineCurve> EditableSplineOf (NvGeImpEntity3d*& theImp, const char* theMethod)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  const occ::handle<Geom2d_BSplineCurve> aCopy =
    occ::down_cast<Geom2d_BSplineCurve> (NvGeCurve2dOf (theImp)->Copy());
  if (aCopy.IsNull())
  {
    throw NvException (std::string ("NvGePolyline2d::") + theMethod
                       + "(): the polyline geometry cannot be copied");
  }
  theImp->SetGeom (aCopy);
  return aCopy;
}

// Builds the degree-1 clamped B-spline for a flat clamped knot sequence and
// the pole list. Throws NvException when the data is inconsistent.
occ::handle<Geom2d_BSplineCurve> BuildPolylineGeometry (std::vector<double> theFlatKnots,
                                                        std::vector<gp_Pnt2d> thePoles,
                                                        const char* theMethod)
{
  const std::string aContext = std::string ("NvGePolyline2d::") + theMethod + "()";
  if (thePoles.size() < 2)
  {
    throw NvException (aContext + ": at least two points are required");
  }
  if (theFlatKnots.size() != thePoles.size() + 2)
  {
    throw NvException (aContext + ": the flat knot count must be the point count plus two");
  }
  for (std::size_t i = 1; i < theFlatKnots.size(); ++i)
  {
    if (theFlatKnots[i] < theFlatKnots[i - 1] - THE_KNOT_TOL)
    {
      throw NvException (aContext + ": the knots must be non-decreasing");
    }
  }
  // Compress the flat sequence into distinct knots and multiplicities.
  std::vector<double> aKnots;
  std::vector<int> aMults;
  for (std::size_t i = 0; i < theFlatKnots.size(); ++i)
  {
    if (!aKnots.empty() && theFlatKnots[i] - aKnots.back() <= THE_KNOT_TOL)
    {
      ++aMults.back();
    }
    else
    {
      aKnots.push_back (theFlatKnots[i]);
      aMults.push_back (1);
    }
  }
  try
  {
    NCollection_Array1<gp_Pnt2d> aPoleArray (1, static_cast<int> (thePoles.size()));
    for (std::size_t i = 0; i < thePoles.size(); ++i)
    {
      aPoleArray.SetValue (static_cast<int> (i) + 1, thePoles[i]);
    }
    NCollection_Array1<double> aKnotArray (1, static_cast<int> (aKnots.size()));
    for (std::size_t i = 0; i < aKnots.size(); ++i)
    {
      aKnotArray.SetValue (static_cast<int> (i) + 1, aKnots[i]);
    }
    NCollection_Array1<int> aMultArray (1, static_cast<int> (aMults.size()));
    for (std::size_t i = 0; i < aMults.size(); ++i)
    {
      aMultArray.SetValue (static_cast<int> (i) + 1, aMults[i]);
    }
    return occ::handle<Geom2d_BSplineCurve> (
      new Geom2d_BSplineCurve (aPoleArray, aKnotArray, aMultArray, 1, false));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (aContext + ": the points and knots do not form a valid polyline");
  }
}

// Flat clamped knots (0, 0, 1, ..., n-1, n-1) for n fit points: fit point i
// is interpolated at parameter i.
std::vector<double> UniformFlatKnots (int thePointCount)
{
  std::vector<double> aFlat;
  aFlat.reserve (thePointCount + 2);
  aFlat.push_back (0.0);
  aFlat.push_back (0.0);
  for (int i = 1; i < thePointCount - 1; ++i)
  {
    aFlat.push_back (static_cast<double> (i));
  }
  aFlat.push_back (static_cast<double> (thePointCount - 1));
  aFlat.push_back (static_cast<double> (thePointCount - 1));
  return aFlat;
}

// Appends the polyline vertices approximating theCurve within theApprEps.
std::vector<NvGePoint2d> ApproximatePoints (const NvGeCurve2d& theCurve, double theApprEps)
{
  std::vector<NvGePoint2d> aPoints;
  if (theCurve.isKindOf (NvGe::kPolyline2d))
  {
    const NvGePolyline2d& aSource = static_cast<const NvGePolyline2d&> (theCurve);
    const int aCount = aSource.numFitPoints();
    for (int i = 0; i < aCount; ++i)
    {
      aPoints.push_back (aSource.fitPointAt (i));
    }
    return aPoints;
  }
  if (theCurve.isKindOf (NvGe::kLineSeg2d))
  {
    const NvGeLineSeg2d& aSeg = static_cast<const NvGeLineSeg2d&> (theCurve);
    aPoints.push_back (aSeg.startPoint());
    aPoints.push_back (aSeg.endPoint());
    return aPoints;
  }
  if (theCurve.isKindOf (NvGe::kLine2d) || theCurve.isKindOf (NvGe::kRay2d))
  {
    throw NvException ("NvGePolyline2d::NvGePolyline2d(): unbounded linear entities"
                       " cannot be approximated by a polyline");
  }
  if (theCurve.isKindOf (NvGe::kCircArc2d))
  {
    const NvGeCircArc2d& anArc = static_cast<const NvGeCircArc2d&> (theCurve);
    const NvGePoint2d aCenter = anArc.center();
    const NvGeVector2d aRef = anArc.refVec();
    const double aRadius = std::max (anArc.radius(), gp::Resolution());
    double aSweep = anArc.isClockWise() == Adesk::kTrue
      ? NormalizeAngle (anArc.startAng() - anArc.endAng())
      : NormalizeAngle (anArc.endAng() - anArc.startAng());
    if (aSweep <= THE_FULL_SWEEP_TOL)
    {
      aSweep = THE_TWO_PI;
    }
    // Sagitta (deflection) criterion: r * (1 - cos (step / 2)) <= apprEps.
    double aStep = 2.0 * std::acos (std::max (-1.0, 1.0 - theApprEps / aRadius));
    if (aStep <= THE_FULL_SWEEP_TOL)
    {
      aStep = THE_TWO_PI / 8.0;   // cap the segment count for tiny tolerances
    }
    const int aSegCount = std::max (1, static_cast<int> (std::ceil (aSweep / aStep)));
    const double aSign = (anArc.isClockWise() == Adesk::kTrue) ? -1.0 : 1.0;
    for (int i = 0; i <= aSegCount; ++i)
    {
      const double anAngle = anArc.startAng() + aSign * aSweep * (static_cast<double> (i) / static_cast<double> (aSegCount));
      const double aCos = std::cos (anAngle);
      const double aSin = std::sin (anAngle);
      aPoints.push_back (NvGePoint2d (
        aCenter.x + aRadius * (aCos * aRef.x - aSin * aRef.y),
        aCenter.y + aRadius * (aSin * aRef.x + aCos * aRef.y)));
    }
    return aPoints;
  }
  if (theCurve.isKindOf (NvGe::kEllipArc2d))
  {
    const NvGeEllipArc2d& anEll = static_cast<const NvGeEllipArc2d&> (theCurve);
    const NvGePoint2d aCenter = anEll.center();
    const NvGeVector2d aMajor = anEll.majorAxis();
    const NvGeVector2d aMinor = anEll.minorAxis();
    const double aMajorR = std::max (anEll.majorRadius(), gp::Resolution());
    const double aMinorR = std::max (anEll.minorRadius(), gp::Resolution());
    // Minimal curvature radius of the ellipse: b^2 / a.
    const double aRhoMin = std::max (aMinorR * aMinorR / aMajorR, gp::Resolution());
    double aSweep = anEll.isClockWise() == Adesk::kTrue
      ? NormalizeAngle (anEll.startAng() - anEll.endAng())
      : NormalizeAngle (anEll.endAng() - anEll.startAng());
    if (aSweep <= THE_FULL_SWEEP_TOL)
    {
      aSweep = THE_TWO_PI;
    }
    double aStep = 2.0 * std::acos (std::max (-1.0, 1.0 - theApprEps / aRhoMin));
    if (aStep <= THE_FULL_SWEEP_TOL)
    {
      aStep = THE_TWO_PI / 8.0;
    }
    const int aSegCount = std::max (1, static_cast<int> (std::ceil (aSweep / aStep)));
    const double aSign = (anEll.isClockWise() == Adesk::kTrue) ? -1.0 : 1.0;
    for (int i = 0; i <= aSegCount; ++i)
    {
      const double anAngle = anEll.startAng() + aSign * aSweep * (static_cast<double> (i) / static_cast<double> (aSegCount));
      const double aCos = std::cos (anAngle);
      const double aSin = std::sin (anAngle);
      aPoints.push_back (NvGePoint2d (
        aCenter.x + aMajorR * aCos * aMajor.x + aMinorR * aSin * aMinor.x,
        aCenter.y + aMajorR * aCos * aMajor.y + aMinorR * aSin * aMinor.y));
    }
    return aPoints;
  }
  throw NvException ("NvGePolyline2d::NvGePolyline2d(): the curve kind cannot be"
                     " approximated by a polyline");
}

}

//=================================================================================================

NvGePolyline2d::NvGePolyline2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  std::vector<double> aFlatKnots;
  aFlatKnots.push_back (0.0);
  aFlatKnots.push_back (0.0);
  aFlatKnots.push_back (1.0);
  aFlatKnots.push_back (1.0);
  std::vector<gp_Pnt2d> aPoles;
  aPoles.push_back (gp_Pnt2d (0.0, 0.0));
  aPoles.push_back (gp_Pnt2d (0.0, 0.0));
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline2d,
                                  BuildPolylineGeometry (aFlatKnots, aPoles, "NvGePolyline2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePolyline2d::NvGePolyline2d (const NvGePolyline2d& theSrc)
: NvGeSplineEnt2d (theSrc)
{
}

//=================================================================================================

NvGePolyline2d::NvGePolyline2d (const NvGePoint2dArray& thePoints)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const int aCount = thePoints.length();
  if (aCount < 2)
  {
    throw NvException ("NvGePolyline2d::NvGePolyline2d(): at least two points are required");
  }
  std::vector<gp_Pnt2d> aPoles;
  aPoles.reserve (aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aPoles.push_back (gp_Pnt2d (thePoints[i].x, thePoints[i].y));
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline2d,
                                  BuildPolylineGeometry (UniformFlatKnots (aCount), aPoles,
                                                         "NvGePolyline2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePolyline2d::NvGePolyline2d (const NvGeKnotVector& theKnots, const NvGePoint2dArray& thePoints)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const int aCount = thePoints.length();
  if (aCount < 2)
  {
    throw NvException ("NvGePolyline2d::NvGePolyline2d(): at least two points are required");
  }
  if (theKnots.length() != aCount + 2)
  {
    throw NvException ("NvGePolyline2d::NvGePolyline2d(): the flat knot vector must hold the"
                       " point count plus two entries (clamped degree-1 knots)");
  }
  std::vector<double> aFlatKnots;
  aFlatKnots.reserve (aCount + 2);
  for (int i = 0; i < aCount + 2; ++i)
  {
    aFlatKnots.push_back (theKnots[i]);
  }
  std::vector<gp_Pnt2d> aPoles;
  aPoles.reserve (aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aPoles.push_back (gp_Pnt2d (thePoints[i].x, thePoints[i].y));
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline2d,
                                  BuildPolylineGeometry (aFlatKnots, aPoles, "NvGePolyline2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePolyline2d::NvGePolyline2d (const NvGeCurve2d& theCurve, double theApprEps)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (theApprEps <= 0.0)
  {
    throw NvException ("NvGePolyline2d::NvGePolyline2d(): the approximation tolerance"
                       " must be positive");
  }
  const std::vector<NvGePoint2d> aPoints = ApproximatePoints (theCurve, theApprEps);
  std::vector<gp_Pnt2d> aPoles;
  aPoles.reserve (aPoints.size());
  for (std::size_t i = 0; i < aPoints.size(); ++i)
  {
    aPoles.push_back (gp_Pnt2d (aPoints[i].x, aPoints[i].y));
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline2d,
                                  BuildPolylineGeometry (UniformFlatKnots (static_cast<int> (aPoles.size())),
                                                         aPoles, "NvGePolyline2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

int NvGePolyline2d::numFitPoints () const
{
  return SplineOf (mpImpEnt, "numFitPoints")->NbPoles();
}

//=================================================================================================

NvGePoint2d NvGePolyline2d::fitPointAt (int theIdx) const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "fitPointAt");
  CheckIndex (theIdx, aSpline->NbPoles(), "fitPointAt", "the fit point index");
  const gp_Pnt2d aPnt = aSpline->Pole (theIdx + 1);
  return NvGePoint2d (aPnt.X(), aPnt.Y());
}

//=================================================================================================

NvGeSplineEnt2d& NvGePolyline2d::setFitPointAt (int theIdx, const NvGePoint2d& thePnt)
{
  {
    const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "setFitPointAt");
    CheckIndex (theIdx, aSpline->NbPoles(), "setFitPointAt", "the fit point index");
  }
  occ::handle<Geom2d_BSplineCurve> aSpline = EditableSplineOf (mpImpEnt, "setFitPointAt");
  aSpline->SetPole (theIdx + 1, gp_Pnt2d (thePnt.x, thePnt.y));
  return *this;
}

//=================================================================================================

NvGePolyline2d& NvGePolyline2d::operator = (const NvGePolyline2d& thePolyline)
{
  NvGeSplineEnt2d::operator= (thePolyline);
  return *this;
}
