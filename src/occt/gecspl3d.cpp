// gecspl3d.cpp - implementation of NvGeCubicSplineCurve3d based on OCCT
// Geom_BSplineCurve.
//
// A cubic spline curve is a C2 cubic B-spline through fit points (optionally
// clamped by end derivatives), or a C1 piecewise-Hermite cubic when the fit
// points are given together with first derivatives. The OCCT B-spline is
// stored inside the entity impl (see geimpdata.h); the fit points, their
// parameters and first derivatives ride on a file-local Geom_BSplineCurve
// subclass (NvGeCspl3dData). Modifiers follow the copy-on-write discipline:
// they build a NEW spline object (never mutating the possibly shared one)
// and replace the impl geometry after detaching a shared impl.
//
// Periodic convention: the first fit point must be equal to the last one
// (header contract). The stored parameters hold exactly one parameter per
// stored fit point; on a periodic spline the last parameter is the closing
// (wrap) parameter.
//
// Boolean-returning methods report invalid input with kFalse; methods
// returning a value or a reference throw NvException with the method name.

#include <gecspl3d.h>

#include <NvException.h>
#include <gecurv3d.h>
#include <gegbl.h>
#include <geimpdata.h>
#include <geintrvl.h>

#include <GeomAPI_Interpolate.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Geometry.hxx>
#include <NCollection_HArray1.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{

// Side data riding on NvGeCspl3dData: the fit definition. Copied along with
// every private copy.
struct NvGeCsplData3d
{
  NvGePoint3dArray    Points;        // fit points (periodic: first == last)
  std::vector<double> Params;        // one parameter per fit point (periodic:
                                     // the last one closes the loop)
  NvGeVector3dArray   Derivs;        // first derivatives at the fit points
  bool                Periodic = false;
  NvGeTol             Tolerance;
  bool                EvalModeFlag = false;
};

// B-spline storage: the OCCT definition plus the fit side data above.
class NvGeCspl3dData : public Geom_BSplineCurve
{
public:

  NvGeCspl3dData (const NCollection_Array1<gp_Pnt>& thePoles,
                  const NCollection_Array1<double>& theKnots,
                  const NCollection_Array1<int>& theMults,
                  const int theDegree,
                  const bool thePeriodic)
  : Geom_BSplineCurve (thePoles, theKnots, theMults, theDegree, thePeriodic)
  {
  }

  //! Definition copy without side data.
  explicit NvGeCspl3dData (const Geom_BSplineCurve& theSrc)
  : Geom_BSplineCurve (theSrc)
  {
  }

  //! Definition copy carrying fit side data (used after interpolation).
  NvGeCspl3dData (const Geom_BSplineCurve& theSrc, const NvGeCsplData3d& theData)
  : Geom_BSplineCurve (theSrc),
    Data (theData)
  {
  }

  //! Full copy: definition plus side data (copy-on-write private copies).
  NvGeCspl3dData (const NvGeCspl3dData& theSrc)
  : Geom_BSplineCurve (theSrc),
    Data (theSrc.Data)
  {
  }

  NvGeCsplData3d Data;
};

// The stored B-spline; throws when the impl holds something else.
occ::handle<Geom_BSplineCurve> SplineOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (NvGeCurve3dOf (theImp));
  if (aSpline.IsNull())
  {
    throw NvException ("NvGeCubicSplineCurve3d: the entity does not hold a B-spline curve");
  }
  return aSpline;
}

// The side data behind theSpline; null when the spline carries none.
const NvGeCspl3dData* FitDataOf (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  return dynamic_cast<const NvGeCspl3dData*> (theSpline.get());
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom_BSplineCurve>& theSpline)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theSpline);
}

// Chord-length parameters over thePoints. On a periodic fit (theWrapEnd set)
// the caller keeps the repeated closing point as the last entry, so its
// parameter IS the closing parameter.
std::vector<double> ChordParams (const NvGePoint3dArray& thePoints, const bool theWrapEnd)
{
  const int aCount = thePoints.length();
  std::vector<double> aParams;
  aParams.reserve (aCount + (theWrapEnd ? 1 : 0));
  if (aCount == 0)
  {
    return aParams;
  }
  aParams.push_back (0.0);
  for (int i = 1; i < aCount; ++i)
  {
    const double aPrev = aParams.back();
    aParams.push_back (aPrev + thePoints[i - 1].distanceTo (thePoints[i]));
  }
  if (theWrapEnd && aCount >= 2)
  {
    const double aPrev = aParams.back();
    aParams.push_back (aPrev + thePoints[aCount - 1].distanceTo (thePoints[0]));
  }
  return aParams;
}

// First derivatives of theSpline at theParams (dn/du at order 1).
NvGeVector3dArray DerivsAt (const Geom_BSplineCurve& theSpline, const std::vector<double>& theParams)
{
  NvGeVector3dArray aDerivs;
  for (size_t i = 0; i < theParams.size(); ++i)
  {
    const gp_Vec aVec = theSpline.DN (theParams[i], 1);
    aDerivs.append (NvGeVector3d (aVec.X(), aVec.Y(), aVec.Z()));
  }
  return aDerivs;
}

// Distance from thePnt to the segment [theSegA, theSegB].
double DistToSegment (const NvGePoint3d& thePnt, const NvGePoint3d& theSegA, const NvGePoint3d& theSegB)
{
  const double aDx = theSegB.x - theSegA.x;
  const double aDy = theSegB.y - theSegA.y;
  const double aDz = theSegB.z - theSegA.z;
  const double aLen2 = aDx * aDx + aDy * aDy + aDz * aDz;
  if (aLen2 <= 0.0)
  {
    return thePnt.distanceTo (theSegA);
  }
  double aT = ((thePnt.x - theSegA.x) * aDx + (thePnt.y - theSegA.y) * aDy + (thePnt.z - theSegA.z) * aDz) / aLen2;
  aT = std::max (0.0, std::min (1.0, aT));
  return thePnt.distanceTo (NvGePoint3d (theSegA.x + aT * aDx, theSegA.y + aT * aDy, theSegA.z + aT * aDz));
}

// Sample parameters over [theStart, theEnd] refining at midpoints whose
// chord deviation exceeds theEps (bounded to keep sampling predictable).
std::vector<double> AdaptiveParams (const NvGeCurve3d& theCurve,
                                    double theStart, double theEnd, double theEps)
{
  std::vector<double> aParams;
  constexpr int THE_SEEDS = 17;
  for (int i = 0; i < THE_SEEDS; ++i)
  {
    aParams.push_back (theStart + (theEnd - theStart) * i / static_cast<double> (THE_SEEDS - 1));
  }
  for (int aRound = 0; aRound < 7; ++aRound)
  {
    std::vector<double> aRefined;
    aRefined.reserve (aParams.size() * 2);
    bool aSplit = false;
    for (size_t i = 0; i + 1 < aParams.size(); ++i)
    {
      aRefined.push_back (aParams[i]);
      if (aRefined.size() + (aParams.size() - i) > 1024u)
      {
        continue;
      }
      const double aMid = 0.5 * (aParams[i] + aParams[i + 1]);
      const NvGePoint3d aP0 = theCurve.evalPoint (aParams[i]);
      const NvGePoint3d aPm = theCurve.evalPoint (aMid);
      const NvGePoint3d aP1 = theCurve.evalPoint (aParams[i + 1]);
      if (DistToSegment (aPm, aP0, aP1) > theEps)
      {
        aRefined.push_back (aMid);
        aSplit = true;
      }
    }
    aRefined.push_back (aParams.back());
    aParams.swap (aRefined);
    if (!aSplit)
    {
      break;
    }
  }
  return aParams;
}

// Detached full copy of theSpline (definition plus side data) for editing.
occ::handle<NvGeCspl3dData> PrivateCopy (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  const NvGeCspl3dData* aSrc = FitDataOf (theSpline);
  if (aSrc != nullptr)
  {
    return occ::handle<NvGeCspl3dData> (new NvGeCspl3dData (*aSrc));
  }
  return occ::handle<NvGeCspl3dData> (new NvGeCspl3dData (*theSpline));
}

// Interpolates a C2 cubic through (thePoints, theParams), optionally
// clamping the end derivatives (Scale = false keeps their magnitudes), and
// carries theData with derivatives recovered from the result. Non-periodic.
// Throws NvException with theMethod context on invalid input or failure.
occ::handle<NvGeCspl3dData> InterpolateData (const NvGePoint3dArray& thePoints,
                                             const std::vector<double>& theParams,
                                             const NvGeTol& theTol,
                                             const bool theClampEnds,
                                             const NvGeVector3d& theStartDeriv,
                                             const NvGeVector3d& theEndDeriv,
                                             const char* theMethod)
{
  const std::string aCtx = std::string ("NvGeCubicSplineCurve3d::") + theMethod + "(): ";
  const int aCount = thePoints.length();
  if (aCount < 2)
  {
    throw NvException (aCtx + "at least two fit points are required");
  }
  if (static_cast<int> (theParams.size()) != aCount)
  {
    throw NvException (aCtx + "the parameter count does not match the fit point count");
  }
  occ::handle<NCollection_HArray1<gp_Pnt>> aPoints = new NCollection_HArray1<gp_Pnt> (1, aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aPoints->SetValue (i + 1, gp_Pnt (thePoints[i].x, thePoints[i].y, thePoints[i].z));
  }
  occ::handle<NCollection_HArray1<double>> aParameters = new NCollection_HArray1<double> (1, aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aParameters->SetValue (i + 1, theParams[i]);
  }
  GeomAPI_Interpolate anInterp (aPoints, aParameters, false, theTol.equalPoint());
  if (theClampEnds)
  {
    anInterp.Load (gp_Vec (theStartDeriv.x, theStartDeriv.y, theStartDeriv.z),
                   gp_Vec (theEndDeriv.x, theEndDeriv.y, theEndDeriv.z), false);
  }
  try
  {
    anInterp.Perform();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  if (!anInterp.IsDone())
  {
    throw NvException (aCtx + "the fit interpolation failed");
  }
  NvGeCsplData3d aData;
  aData.Points = thePoints;
  aData.Params = theParams;
  aData.Derivs = DerivsAt (*anInterp.Curve(), theParams);
  aData.Periodic = false;
  aData.Tolerance = theTol;
  return occ::handle<NvGeCspl3dData> (new NvGeCspl3dData (*anInterp.Curve(), aData));
}

// Piecewise-Hermite builder: a C1 cubic through theData.Points with
// theData.Derivs at theData.Params. Non-periodic: interior knots of
// multiplicity 3 between the spans. Periodic: one wrap span, all knots of
// multiplicity 3. Throws NvException with theMethod context on inconsistent
// data.
occ::handle<NvGeCspl3dData> HermiteSpline (const NvGeCsplData3d& theData, const char* theMethod)
{
  const std::string aCtx = std::string ("NvGeCubicSplineCurve3d::") + theMethod + "(): ";
  const bool aPer = theData.Periodic;
  const int aCount = theData.Points.length();
  if (aCount < (aPer ? 3 : 2))
  {
    throw NvException (aCtx + "at least two distinct fit points are required");
  }
  if (theData.Derivs.length() != aCount || static_cast<int> (theData.Params.size()) != aCount)
  {
    throw NvException (aCtx + "the fit data counts are inconsistent");
  }
  for (int i = 1; i < aCount; ++i)
  {
    if (theData.Params[i] - theData.Params[i - 1] <= 0.0)
    {
      throw NvException (aCtx + "the fit parameters must be strictly increasing");
    }
  }
  if (aPer
      && theData.Points[0].distanceTo (theData.Points[aCount - 1]) > theData.Tolerance.equalPoint())
  {
    throw NvException (aCtx + "the first fit point must be equal to the last fit point");
  }
  // Span i runs from Params[i] to Params[i + 1]; on a periodic spline the
  // last span wraps back to the first point (the stored duplicate is not
  // used as a pole).
  const int aSpanCount = aCount - 1;
  auto anEndIdx = [aPer, aCount] (int theI)
  {
    return (aPer && theI == aCount - 2) ? 0 : theI + 1;
  };
  std::vector<gp_Pnt> aPoles;
  aPoles.reserve (3 * aSpanCount + (aPer ? 0 : 1));
  aPoles.push_back (gp_Pnt (theData.Points[0].x, theData.Points[0].y, theData.Points[0].z));
  for (int i = 0; i < aSpanCount; ++i)
  {
    const int anEnd = anEndIdx (i);
    const double aH = (theData.Params[i + 1] - theData.Params[i]) / 3.0;
    aPoles.push_back (gp_Pnt (theData.Points[i].x + theData.Derivs[i].x * aH,
                              theData.Points[i].y + theData.Derivs[i].y * aH,
                              theData.Points[i].z + theData.Derivs[i].z * aH));
    aPoles.push_back (gp_Pnt (theData.Points[anEnd].x - theData.Derivs[anEnd].x * aH,
                              theData.Points[anEnd].y - theData.Derivs[anEnd].y * aH,
                              theData.Points[anEnd].z - theData.Derivs[anEnd].z * aH));
    if (!aPer || i < aSpanCount - 1)
    {
      aPoles.push_back (gp_Pnt (theData.Points[anEnd].x, theData.Points[anEnd].y, theData.Points[anEnd].z));
    }
  }
  NCollection_Array1<gp_Pnt> aPoleArr (1, static_cast<int> (aPoles.size()));
  for (size_t i = 0; i < aPoles.size(); ++i)
  {
    aPoleArr.SetValue (static_cast<int> (i) + 1, aPoles[i]);
  }
  NCollection_Array1<double> aKnots (1, aCount);
  NCollection_Array1<int> aMults (1, aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aKnots.SetValue (i + 1, theData.Params[i]);
    aMults.SetValue (i + 1, aPer ? 3 : ((i == 0 || i == aCount - 1) ? 4 : 3));
  }
  try
  {
    occ::handle<NvGeCspl3dData> aSpline (new NvGeCspl3dData (aPoleArr, aKnots, aMults, 3, aPer));
    // Carry the fit definition on the spline (as interpolation does) so the
    // fit-point accessors and the rebuild-on-edit modifiers see the data.
    aSpline->Data = theData;
    return aSpline;
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
}

} // namespace

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  // A valid degree-1 spline collapsed onto the origin (2 poles, knots 0..1).
  NCollection_Array1<gp_Pnt> aPoles (1, 2);
  aPoles.SetValue (1, gp_Pnt (0.0, 0.0, 0.0));
  aPoles.SetValue (2, gp_Pnt (0.0, 0.0, 0.0));
  NCollection_Array1<double> aKnots (1, 2);
  aKnots.SetValue (1, 0.0);
  aKnots.SetValue (2, 1.0);
  NCollection_Array1<int> aMults (1, 2);
  aMults.SetValue (1, 2);
  aMults.SetValue (2, 2);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCubicSplineCurve3d,
                                  occ::handle<Geom_BSplineCurve> (new NvGeCspl3dData (aPoles, aKnots, aMults, 1, false)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d (const NvGeCubicSplineCurve3d& theSrc)
: NvGeSplineEnt3d (theSrc)
{
}

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d (const NvGePoint3dArray& theFitPoints,
                                                const NvGeTol& theFitTolerance)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const int aCount = theFitPoints.length();
  if (aCount < 3)
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): a periodic fit needs"
                       " at least three fit points with the first point repeated at the end");
  }
  if (theFitPoints[0].distanceTo (theFitPoints[aCount - 1]) > theFitTolerance.equalPoint())
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the first fit point"
                       " must be equal to the last fit point");
  }
  NvGePoint3dArray aPoints;
  for (int i = 0; i < aCount; ++i)
  {
    aPoints.append (theFitPoints[i]);
  }
  const std::vector<double> aParams = ChordParams (aPoints, false);
  // The interpolator gets the distinct points plus the closing parameter.
  occ::handle<NCollection_HArray1<gp_Pnt>> aPnts = new NCollection_HArray1<gp_Pnt> (1, aCount - 1);
  for (int i = 0; i < aCount - 1; ++i)
  {
    aPnts->SetValue (i + 1, gp_Pnt (aPoints[i].x, aPoints[i].y, aPoints[i].z));
  }
  occ::handle<NCollection_HArray1<double>> aPrms = new NCollection_HArray1<double> (1, aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aPrms->SetValue (i + 1, aParams[i]);
  }
  GeomAPI_Interpolate anInterp (aPnts, aPrms, true, theFitTolerance.equalPoint());
  try
  {
    anInterp.Perform();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  if (!anInterp.IsDone())
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the fit interpolation"
                       " failed");
  }
  NvGeCsplData3d aData;
  aData.Points = aPoints;
  aData.Params = aParams;
  aData.Derivs = DerivsAt (*anInterp.Curve(), aParams);
  aData.Periodic = true;
  aData.Tolerance = theFitTolerance;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCubicSplineCurve3d,
                                  occ::handle<Geom_BSplineCurve> (new NvGeCspl3dData (*anInterp.Curve(), aData)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d (const NvGePoint3dArray& theFitPoints,
                                                const NvGeVector3d& theStartDeriv,
                                                const NvGeVector3d& theEndDeriv,
                                                const NvGeTol& theFitTolerance)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const std::vector<double> aParams = ChordParams (theFitPoints, false);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCubicSplineCurve3d,
                                  occ::handle<Geom_BSplineCurve> (InterpolateData (theFitPoints,
                                                                                   aParams,
                                                                                   theFitTolerance,
                                                                                   true,
                                                                                   theStartDeriv,
                                                                                   theEndDeriv,
                                                                                   "NvGeCubicSplineCurve3d")));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d (const NvGeCurve3d& theCurve, double theEpsilon)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (theEpsilon <= 0.0)
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the approximation"
                       " tolerance must be positive");
  }
  NvGeInterval anInterval;
  theCurve.getInterval (anInterval);
  if (!anInterval.isBounded())
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the curve has no"
                       " bounded interval");
  }
  const std::vector<double> aParams = AdaptiveParams (theCurve, anInterval.lowerBound(),
                                                      anInterval.upperBound(), theEpsilon);
  NvGePoint3dArray aPoints;
  for (size_t i = 0; i < aParams.size(); ++i)
  {
    aPoints.append (theCurve.evalPoint (aParams[i]));
  }
  NvGeTol aTol;
  aTol.setEqualPoint (theEpsilon);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCubicSplineCurve3d,
                                  occ::handle<Geom_BSplineCurve> (InterpolateData (aPoints,
                                                                                   aParams,
                                                                                   aTol,
                                                                                   false,
                                                                                   NvGeVector3d(),
                                                                                   NvGeVector3d(),
                                                                                   "NvGeCubicSplineCurve3d")));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d (const NvGeKnotVector& theKnots,
                                                const NvGePoint3dArray& theFitPoints,
                                                const NvGeVector3dArray& theFirstDerivs,
                                                Nova::Boolean theIsPeriodic)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const int aCount = theFitPoints.length();
  if (aCount < 2)
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): at least two fit"
                       " points are required");
  }
  if (theFirstDerivs.length() != aCount || theKnots.length() != aCount)
  {
    throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the fit point,"
                       " derivative and knot counts must match");
  }
  NvGeCsplData3d aData;
  aData.Points = theFitPoints;
  aData.Derivs = theFirstDerivs;
  aData.Params.reserve (aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aData.Params.push_back (theKnots[i]);
  }
  for (int i = 1; i < aCount; ++i)
  {
    if (aData.Params[i] - aData.Params[i - 1] <= theKnots.tolerance())
    {
      throw NvException ("NvGeCubicSplineCurve3d::NvGeCubicSplineCurve3d(): the fit knots must"
                         " be strictly increasing");
    }
  }
  aData.Periodic = theIsPeriodic != Nova::kFalse;
  aData.Tolerance = NvGeContext::gTol;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCubicSplineCurve3d,
                                  occ::handle<Geom_BSplineCurve> (HermiteSpline (aData,
                                                                                 "NvGeCubicSplineCurve3d")));
  mpImpEnt->Ref();
}

//=================================================================================================

int NvGeCubicSplineCurve3d::numFitPoints () const
{
  const NvGeCspl3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  return aData != nullptr ? aData->Data.Points.length() : 0;
}

//=================================================================================================

NvGePoint3d NvGeCubicSplineCurve3d::fitPointAt (int theIdx) const
{
  const NvGeCspl3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr || theIdx < 0 || theIdx >= aData->Data.Points.length())
  {
    throw NvException ("NvGeCubicSplineCurve3d::fitPointAt(): the index is out of range");
  }
  return aData->Data.Points[theIdx];
}

//=================================================================================================

NvGeCubicSplineCurve3d& NvGeCubicSplineCurve3d::setFitPointAt (int theIdx, const NvGePoint3d& thePoint)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeCspl3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || theIdx < 0 || theIdx >= aSrc->Data.Points.length())
  {
    throw NvException ("NvGeCubicSplineCurve3d::setFitPointAt(): the index is out of range");
  }
  occ::handle<NvGeCspl3dData> aData = PrivateCopy (aSpline);
  aData->Data.Points[theIdx] = thePoint;
  if (aData->Data.Periodic && (theIdx == 0 || theIdx == aData->Data.Points.length() - 1))
  {
    // Keep the first point and its wrap duplicate coincident.
    aData->Data.Points[0] = thePoint;
    aData->Data.Points[aData->Data.Points.length() - 1] = thePoint;
  }
  ReplaceGeometry (mpImpEnt, HermiteSpline (aData->Data, "setFitPointAt"));
  return *this;
}

//=================================================================================================

NvGeVector3d NvGeCubicSplineCurve3d::firstDerivAt (int theIdx) const
{
  const NvGeCspl3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr || theIdx < 0 || theIdx >= aData->Data.Derivs.length())
  {
    throw NvException ("NvGeCubicSplineCurve3d::firstDerivAt(): the index is out of range");
  }
  return aData->Data.Derivs[theIdx];
}

//=================================================================================================

NvGeCubicSplineCurve3d& NvGeCubicSplineCurve3d::setFirstDerivAt (int theIdx, const NvGeVector3d& theDeriv)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeCspl3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || theIdx < 0 || theIdx >= aSrc->Data.Derivs.length())
  {
    throw NvException ("NvGeCubicSplineCurve3d::setFirstDerivAt(): the index is out of range");
  }
  occ::handle<NvGeCspl3dData> aData = PrivateCopy (aSpline);
  aData->Data.Derivs[theIdx] = theDeriv;
  ReplaceGeometry (mpImpEnt, HermiteSpline (aData->Data, "setFirstDerivAt"));
  return *this;
}

//=================================================================================================

NvGeCubicSplineCurve3d& NvGeCubicSplineCurve3d::operator = (const NvGeCubicSplineCurve3d& theSpline)
{
  NvGeSplineEnt3d::operator= (theSpline);
  return *this;
}
