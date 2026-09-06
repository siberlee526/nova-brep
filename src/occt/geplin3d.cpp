// geplin3d.cpp - implementation of NvGePolyline3d.
//
// A polyline is stored as a degree 1 Geom_BSplineCurve whose poles are the
// fit points, over the knots 0 .. n - 1 with the multiplicities
// [2, 1, ..., 1, 2] so that the parameter range is [0, n - 1]. The curve
// constructor accepts an explicit knot vector; the approximation
// constructor samples a supported source curve with an adaptive
// chord-height subdivision.

#include <geplin3d.h>
#include <Nova.h>
#include <NvException.h>
#include <gearc3d.h>
#include <geell3d.h>
#include <gekvec.h>
#include <gepnt3d.h>
#include <gept3dar.h>
#include <gesent3d.h>
#include <geimpdata.h>

#include <Geom_BSplineCurve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <functional>
#include <vector>

namespace
{

// Tolerance used when grouping equal knot values of the knot constructor.
constexpr double THE_KNOT_TOL = 1e-9;

// Depth cap of the adaptive subdivision of the approximation constructor.
constexpr int THE_MAX_DEPTH = 24;

// Returns the stored B-spline curve.
occ::handle<Geom_BSplineCurve> SplineOf (const occ::handle<Standard_Transient>& theGeom)
{
  if (theGeom.IsNull())
  {
    throw NvException ("NvGePolyline3d: null curve data");
  }
  const occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (theGeom);
  if (aSpline.IsNull())
  {
    throw NvException ("NvGePolyline3d: the entity does not store a B-spline curve");
  }
  return aSpline;
}

// Validates the 1-based index against the item count.
void CheckIndex (int theIdx, int theCount, const char* theMessage)
{
  if (theIdx < 1 || theIdx > theCount)
  {
    throw NvException (theMessage);
  }
}

// Builds the degree 1 curve over the fit points: the knots 0 .. n - 1 with
// the multiplicities [2, 1, ..., 1, 2].
occ::handle<Geom_Curve> PolylineCurveOf (const std::vector<gp_Pnt>& thePoles)
{
  const int aNbPoles = static_cast<int> (thePoles.size());
  if (aNbPoles < 2)
  {
    throw NvException ("NvGePolyline3d: at least two points are required");
  }
  NCollection_Array1<gp_Pnt> aPoleArr (1, aNbPoles);
  NCollection_Array1<double> aKnotArr (1, aNbPoles);
  NCollection_Array1<int> aMultArr (1, aNbPoles);
  for (int aPole = 1; aPole <= aNbPoles; ++aPole)
  {
    aPoleArr.SetValue (aPole, thePoles[aPole - 1]);
    aKnotArr.SetValue (aPole, static_cast<double> (aPole - 1));
    aMultArr.SetValue (aPole, (aPole == 1 || aPole == aNbPoles) ? 2 : 1);
  }
  return occ::handle<Geom_Curve> (new Geom_BSplineCurve (aPoleArr, aKnotArr, aMultArr, 1));
}

// Groups the flat knot vector into distinct values and multiplicities;
// the values must be non-descending.
void KnotsAndMultsOf (const NvGeKnotVector& theKnots,
                      std::vector<double>& theDistinct, std::vector<int>& theMults)
{
  const int aNbKnots = theKnots.length();
  for (int aKnot = 0; aKnot < aNbKnots; ++aKnot)
  {
    const double aValue = theKnots[aKnot];
    if (!theDistinct.empty() && aValue - theDistinct.back() <= THE_KNOT_TOL)
    {
      if (aValue < theDistinct.back() - THE_KNOT_TOL)
      {
        throw NvException ("NvGePolyline3d: the knot values must be non-descending");
      }
      ++theMults.back();
    }
    else
    {
      if (!theDistinct.empty() && aValue < theDistinct.back())
      {
        throw NvException ("NvGePolyline3d: the knot values must be non-descending");
      }
      theDistinct.push_back (aValue);
      theMults.push_back (1);
    }
  }
}

// Samples the source curve into fit points with the chord-height tolerance
// theApprEps, through the public API of the supported curve kinds.
void ApproximateCurve (const NvGeCurve3d& theCurve, double theApprEps,
                       std::vector<gp_Pnt>& thePoles)
{
  std::function<gp_Pnt (double)> anEval;
  double aFirst = 0.0;
  double aLast = 0.0;
  if (theCurve.isKindOf (NvGe::kCircArc3d))
  {
    const NvGeCircArc3d& anArc = static_cast<const NvGeCircArc3d&> (theCurve);
    const gp_Pnt aCenter (anArc.center().x, anArc.center().y, anArc.center().z);
    const gp_Dir aNormal (anArc.normal().x, anArc.normal().y, anArc.normal().z);
    const gp_XYZ aRefDir = gp_Dir (anArc.refVec().x, anArc.refVec().y,
                                   anArc.refVec().z).XYZ();
    const gp_XYZ aCross = aNormal.XYZ().Crossed (aRefDir);
    const double aRadius = anArc.radius();
    aFirst = anArc.startAng();
    aLast = anArc.endAng();
    anEval = [aCenter, aRefDir, aCross, aRadius] (double theU)
    {
      const gp_XYZ anOffset = (aRefDir * std::cos (theU)
                             + aCross * std::sin (theU)) * aRadius;
      return aCenter.Translated (gp_Vec (anOffset));
    };
  }
  else if (theCurve.isKindOf (NvGe::kEllipArc3d))
  {
    const NvGeEllipArc3d& anEllipse = static_cast<const NvGeEllipArc3d&> (theCurve);
    const gp_Pnt aCenter (anEllipse.center().x, anEllipse.center().y,
                          anEllipse.center().z);
    const gp_XYZ aMajorDir = gp_Dir (anEllipse.majorAxis().x, anEllipse.majorAxis().y,
                                     anEllipse.majorAxis().z).XYZ();
    const gp_XYZ aMinorDir = gp_Dir (anEllipse.minorAxis().x, anEllipse.minorAxis().y,
                                     anEllipse.minorAxis().z).XYZ();
    const double aMajorRadius = anEllipse.majorRadius();
    const double aMinorRadius = anEllipse.minorRadius();
    aFirst = anEllipse.startAng();
    aLast = anEllipse.endAng();
    anEval = [aCenter, aMajorDir, aMinorDir, aMajorRadius, aMinorRadius] (double theU)
    {
      const gp_XYZ anOffset = aMajorDir * (aMajorRadius * std::cos (theU))
                            + aMinorDir * (aMinorRadius * std::sin (theU));
      return aCenter.Translated (gp_Vec (anOffset));
    };
  }
  else if (theCurve.isKindOf (NvGe::kPolyline3d))
  {
    // A polyline source is reproduced exactly by copying its poles.
    const NvGeSplineEnt3d& aPline = static_cast<const NvGeSplineEnt3d&> (theCurve);
    const int aNbPoles = aPline.numControlPoints();
    for (int aPole = 1; aPole <= aNbPoles; ++aPole)
    {
      const NvGePoint3d aPnt = aPline.controlPointAt (aPole);
      thePoles.push_back (gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
    }
    return;
  }
  else if (theCurve.isKindOf (NvGe::kSplineEnt3d))
  {
    // Rebuild the non-rational B-spline carrier from the public accessors.
    const NvGeSplineEnt3d& aSplineEnt = static_cast<const NvGeSplineEnt3d&> (theCurve);
    if (aSplineEnt.isRational())
    {
      throw NvException ("NvGePolyline3d: rational splines are not supported"
                         " as approximation sources");
    }
    const int aDegree = aSplineEnt.degree();
    const int aNbPoles = aSplineEnt.numControlPoints();
    const int aNbKnots = aSplineEnt.numKnots();
    NCollection_Array1<gp_Pnt> aPoleArr (1, aNbPoles);
    for (int aPole = 1; aPole <= aNbPoles; ++aPole)
    {
      const NvGePoint3d aPnt = aSplineEnt.controlPointAt (aPole);
      aPoleArr.SetValue (aPole, gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
    }
    NCollection_Array1<double> aKnotArr (1, aNbKnots);
    NCollection_Array1<int> aMultArr (1, aNbKnots);
    for (int aKnot = 1; aKnot <= aNbKnots; ++aKnot)
    {
      aKnotArr.SetValue (aKnot, aSplineEnt.knotAt (aKnot));
      aMultArr.SetValue (aKnot, aDegree - aSplineEnt.continuityAtKnot (aKnot));
    }
    occ::handle<Geom_BSplineCurve> aCarrier;
    try
    {
      aCarrier = new Geom_BSplineCurve (aPoleArr, aKnotArr, aMultArr, aDegree);
    }
    catch (const Standard_Failure&)
    {
      throw NvException ("NvGePolyline3d: the source spline data are inconsistent");
    }
    aFirst = aSplineEnt.startParam();
    aLast = aSplineEnt.endParam();
    anEval = [aCarrier] (double theU)
    {
      return aCarrier->EvalD0 (theU);
    };
  }
  else
  {
    throw NvException ("NvGePolyline3d: the source curve kind is not supported"
                       " for approximation");
  }
  // Adaptive chord-height subdivision over the parameter interval.
  std::vector<gp_Pnt> aSamples;
  std::function<void (double, double, const gp_Pnt&, const gp_Pnt&, int)> aSubdivide =
    [&] (double theU0, double theU1, const gp_Pnt& theP0, const gp_Pnt& theP1,
         int theDepth)
  {
    const double aMidU = 0.5 * (theU0 + theU1);
    const gp_Pnt aMidPnt = anEval (aMidU);
    const gp_Vec aChord (theP0, theP1);
    const double aChordLen = aChord.Magnitude();
    const double aDeviation = (aChordLen <= gp::Resolution())
      ? gp_Vec (theP0, aMidPnt).Magnitude()
      : gp_Vec (theP0, aMidPnt).Crossed (aChord).Magnitude() / aChordLen;
    if (theDepth >= THE_MAX_DEPTH || aDeviation <= theApprEps)
    {
      aSamples.push_back (theP0);
      return;
    }
    aSubdivide (theU0, aMidU, theP0, aMidPnt, theDepth + 1);
    aSubdivide (aMidU, theU1, aMidPnt, theP1, theDepth + 1);
  };
  const gp_Pnt aStartPnt = anEval (aFirst);
  const gp_Pnt anEndPnt = anEval (aLast);
  aSubdivide (aFirst, aLast, aStartPnt, anEndPnt, 0);
  aSamples.push_back (anEndPnt);
  for (const gp_Pnt& aPnt : aSamples)
  {
    if (thePoles.empty() || thePoles.back().Distance (aPnt) > gp::Resolution())
    {
      thePoles.push_back (aPnt);
    }
  }
}

}

// NvGePolyline3d

//=======================================================================
// function : NvGePolyline3d
// purpose  : Degenerate placeholder: a degree 1 curve over two coincident
//            poles at the origin.
//=======================================================================
NvGePolyline3d::NvGePolyline3d ()
{
  std::vector<gp_Pnt> aPoles (2, gp_Pnt (0.0, 0.0, 0.0));
  const occ::handle<Geom_Curve> aCurve = PolylineCurveOf (aPoles);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGePolyline3d
// purpose  : Copy constructor; shares the implementation of the source.
//=======================================================================
NvGePolyline3d::NvGePolyline3d (const NvGePolyline3d& theSrc)
: NvGeSplineEnt3d (theSrc)
{

}

//=======================================================================
// function : NvGePolyline3d
// purpose  : Fits the degree 1 curve through the given points.
//=======================================================================
NvGePolyline3d::NvGePolyline3d (const NvGePoint3dArray& thePoints)
{
  const int aNbPoints = thePoints.logicalLength();
  if (aNbPoints < 2)
  {
    throw NvException ("NvGePolyline3d: at least two points are required");
  }
  std::vector<gp_Pnt> aPoles;
  aPoles.reserve (aNbPoints);
  for (int aPoint = 0; aPoint < aNbPoints; ++aPoint)
  {
    const NvGePoint3d& aPnt = thePoints[aPoint];
    aPoles.push_back (gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
  }
  const occ::handle<Geom_Curve> aCurve = PolylineCurveOf (aPoles);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGePolyline3d
// purpose  : Builds the degree 1 curve over the control points and the
//            given knot vector; the multiplicities implied by equal knot
//            values must be consistent with a degree 1 curve.
//=======================================================================
NvGePolyline3d::NvGePolyline3d (const NvGeKnotVector& theKnots,
                                const NvGePoint3dArray& theCntrlPnts)
{
  const int aNbPoles = theCntrlPnts.logicalLength();
  if (aNbPoles < 2)
  {
    throw NvException ("NvGePolyline3d: at least two control points are required");
  }
  std::vector<double> aDistinct;
  std::vector<int> aMults;
  KnotsAndMultsOf (theKnots, aDistinct, aMults);
  if (aDistinct.size() < 2)
  {
    throw NvException ("NvGePolyline3d: at least two distinct knots are required");
  }
  int aSum = 0;
  for (const int aMult : aMults)
  {
    aSum += aMult;
  }
  if (aSum != aNbPoles + 2)
  {
    throw NvException ("NvGePolyline3d: the knot multiplicities are inconsistent"
                       " with the control point count of a degree 1 curve");
  }
  for (int aKnot = 1; aKnot + 1 < static_cast<int> (aMults.size()); ++aKnot)
  {
    if (aMults[aKnot] > 1)
    {
      throw NvException ("NvGePolyline3d: an interior knot multiplicity exceeds"
                         " the degree");
    }
  }
  if (aMults.front() > 2 || aMults.back() > 2)
  {
    throw NvException ("NvGePolyline3d: an end knot multiplicity exceeds"
                       " the degree plus one");
  }
  NCollection_Array1<gp_Pnt> aPoleArr (1, aNbPoles);
  for (int aPole = 1; aPole <= aNbPoles; ++aPole)
  {
    const NvGePoint3d& aPnt = theCntrlPnts[aPole - 1];
    aPoleArr.SetValue (aPole, gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
  }
  NCollection_Array1<double> aKnotArr (1, static_cast<int> (aDistinct.size()));
  NCollection_Array1<int> aMultArr (1, static_cast<int> (aMults.size()));
  for (int aKnot = 1; aKnot <= aKnotArr.Length(); ++aKnot)
  {
    aKnotArr.SetValue (aKnot, aDistinct[aKnot - 1]);
    aMultArr.SetValue (aKnot, aMults[aKnot - 1]);
  }
  occ::handle<Geom_Curve> aCurve;
  try
  {
    aCurve = new Geom_BSplineCurve (aPoleArr, aKnotArr, aMultArr, 1);
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGePolyline3d: the knot vector is rejected by the curve");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGePolyline3d
// purpose  : Approximates the source curve with the chord-height tolerance
//            theApprEps; supported sources are circles, ellipses,
//            polylines and non-rational splines.
//=======================================================================
NvGePolyline3d::NvGePolyline3d (const NvGeCurve3d& theCrv, double theApprEps)
{
  if (!(theApprEps > 0.0))
  {
    throw NvException ("NvGePolyline3d: the approximation tolerance must be positive");
  }
  std::vector<gp_Pnt> aPoles;
  ApproximateCurve (theCrv, theApprEps, aPoles);
  const occ::handle<Geom_Curve> aCurve = PolylineCurveOf (aPoles);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPolyline3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : numFitPoints
// purpose  : Number of fit points (the poles of the degree 1 curve).
//=======================================================================
int NvGePolyline3d::numFitPoints () const
{
  return SplineOf (mpImpEnt->Geom())->NbPoles();
}

//=======================================================================
// function : fitPointAt
// purpose  : The 1-based fit point.
//=======================================================================
NvGePoint3d NvGePolyline3d::fitPointAt (int theIdx) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbPoles(),
              "NvGePolyline3d::fitPointAt(): the fit point index is out of range");
  const gp_Pnt& aPole = aSpline->Pole (theIdx);
  return NvGePoint3d (aPole.X(), aPole.Y(), aPole.Z());
}

//=======================================================================
// function : setFitPointAt
// purpose  : Moves the 1-based fit point on a private copy of the curve.
//=======================================================================
NvGeSplineEnt3d& NvGePolyline3d::setFitPointAt (int theIdx, const NvGePoint3d& thePnt)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbPoles(),
              "NvGePolyline3d::setFitPointAt(): the fit point index is out of range");
  const occ::handle<Geom_BSplineCurve> aCopy =
    occ::down_cast<Geom_BSplineCurve> (aSpline->Copy());
  aCopy->SetPole (theIdx, gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (aCopy));
  return *this;
}

//=======================================================================
// function : operator =
// purpose  : Assignment; delegates the implementation sharing to the base.
//=======================================================================
NvGePolyline3d& NvGePolyline3d::operator = (const NvGePolyline3d& thePline)
{
  NvGeEntity3d::operator = (thePline);
  return *this;
}
