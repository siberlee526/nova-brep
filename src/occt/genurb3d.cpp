// genurb3d.cpp - implementation of NvGeNurbCurve3d based on OCCT Geom_BSplineCurve.
//
// The OCCT B-spline is stored inside the entity impl (see geimpdata.h). The
// interpolation ("fit") data has no OCCT counterpart, so it rides on a
// file-local Geom_BSplineCurve subclass (NvGeNurb3dData); every OCCT
// algorithm keeps seeing a plain B-spline. Modifiers follow the copy-on-write
// discipline: they build a NEW spline object (never mutating the possibly
// shared one) and replace the impl geometry after detaching a shared impl.
//
// Knot convention: NvGeKnotVector is FLAT - every knot is repeated according
// to its multiplicity - exactly like Geom_BSplineCurve::KnotSequence().
// Distinct knots and their multiplicities are recovered by grouping
// consecutive equal values within NvGeKnotVector::tolerance().
//
// Boolean-returning methods report invalid input with kFalse; methods
// returning a value or a reference throw NvException with the method name.

#include <genurb3d.h>

#include <NvException.h>
#include <geell3d.h>
#include <gegbl.h>
#include <geimpdata.h>
#include <geintrvl.h>
#include <gelnsg3d.h>

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

constexpr double THE_TAU = 6.28318530717958647692;

// Side data riding on NvGeNurb3dData: the interpolation ("fit") definition
// and the evaluation-mode flag. Copied along with every private copy.
struct NvGeFitData3d
{
  NvGePoint3dArray            Points;                   // fit points
  std::vector<double>         Params;                   // explicit parameters; empty = compute
  NvGeVector3dArray           Tangents;                 // per-point tangents (TangentsExist)
  bool                        TangentsExist = false;
  bool                        EndTangentsExist = false; // start/end tangent data present
  NvGeVector3d                StartTangent;
  NvGeVector3d                EndTangent;
  bool                        StartTangentDefined = false;
  bool                        EndTangentDefined = false;
  bool                        Periodic = false;
  double                      PeriodEndParam = 0.0;     // closing parameter (periodic)
  NvGe::KnotParameterization  KnotParam = NvGe::kChord;
  NvGeTol                     Tolerance;
  bool                        EvalModeFlag = false;
};

// B-spline storage: the OCCT definition plus the fit side data above.
class NvGeNurb3dData : public Geom_BSplineCurve
{
public:

  NvGeNurb3dData (const NCollection_Array1<gp_Pnt>& thePoles,
                  const NCollection_Array1<double>& theKnots,
                  const NCollection_Array1<int>& theMults,
                  const int theDegree,
                  const bool thePeriodic)
  : Geom_BSplineCurve (thePoles, theKnots, theMults, theDegree, thePeriodic)
  {
  }

  NvGeNurb3dData (const NCollection_Array1<gp_Pnt>& thePoles,
                  const NCollection_Array1<double>& theWeights,
                  const NCollection_Array1<double>& theKnots,
                  const NCollection_Array1<int>& theMults,
                  const int theDegree,
                  const bool thePeriodic)
  : Geom_BSplineCurve (thePoles, theWeights, theKnots, theMults, theDegree, thePeriodic)
  {
  }

  //! Definition copy without side data.
  explicit NvGeNurb3dData (const Geom_BSplineCurve& theSrc)
  : Geom_BSplineCurve (theSrc)
  {
  }

  //! Definition copy carrying fit side data (used after interpolation).
  NvGeNurb3dData (const Geom_BSplineCurve& theSrc, const NvGeFitData3d& theFit)
  : Geom_BSplineCurve (theSrc),
    Fit (theFit)
  {
  }

  //! Full copy: definition plus side data (copy-on-write private copies).
  NvGeNurb3dData (const NvGeNurb3dData& theSrc)
  : Geom_BSplineCurve (theSrc),
    Fit (theSrc.Fit)
  {
  }

  NvGeFitData3d Fit;
};

// The stored B-spline; throws when the impl holds something else.
occ::handle<Geom_BSplineCurve> SplineOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (NvGeCurve3dOf (theImp));
  if (aSpline.IsNull())
  {
    throw NvException ("NvGeNurbCurve3d: the entity does not hold a B-spline curve");
  }
  return aSpline;
}

// The side data behind theSpline; null when the spline carries none.
const NvGeNurb3dData* FitDataOf (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  return dynamic_cast<const NvGeNurb3dData*> (theSpline.get());
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

// Knot equality tolerance for parameter-range checks on theSpline.
double KnotToleranceOf (const Geom_BSplineCurve& theSpline)
{
  const double aSpan = theSpline.LastParameter() - theSpline.FirstParameter();
  return std::max (aSpan * 1e-9, gp::Resolution());
}

// Distinct knots with multiplicities, grouped within theKnots.tolerance().
struct KnotStructure
{
  std::vector<double> Knots;
  std::vector<int>    Mults;
};

KnotStructure KnotsAndMults (const NvGeKnotVector& theKnots)
{
  KnotStructure aRes;
  const double aTol = theKnots.tolerance();
  for (int i = 0; i < theKnots.length(); ++i)
  {
    if (!aRes.Knots.empty() && std::abs (theKnots[i] - aRes.Knots.back()) <= aTol)
    {
      ++aRes.Mults.back();
    }
    else
    {
      aRes.Knots.push_back (theKnots[i]);
      aRes.Mults.push_back (1);
    }
  }
  return aRes;
}

// Flat knot vector (knots repeated per multiplicity), NvGeKnotVector form.
NvGeKnotVector FlatKnotVector (const Geom_BSplineCurve& theSpline)
{
  NvGeDoubleArray aFlat;
  const NCollection_Array1<double>& aSeq = theSpline.KnotSequence();
  for (int i = aSeq.Lower(); i <= aSeq.Upper(); ++i)
  {
    aFlat.append (aSeq.Value (i));
  }
  return NvGeKnotVector (aFlat);
}

// Complete B-spline definition used for rebuilds.
struct Definition
{
  std::vector<gp_Pnt> Poles;
  std::vector<double> Weights;   // empty = non-rational
  std::vector<double> Knots;     // distinct
  std::vector<int>    Mults;
  int                 Degree = 1;
  bool                Periodic = false;
  bool                Rational = false;
};

Definition DefinitionOf (const Geom_BSplineCurve& theSpline)
{
  Definition aDef;
  aDef.Degree = theSpline.Degree();
  aDef.Periodic = theSpline.IsPeriodic();
  aDef.Rational = theSpline.Weights() != nullptr;
  const NCollection_Array1<gp_Pnt>& aPoles = theSpline.Poles();
  for (int i = aPoles.Lower(); i <= aPoles.Upper(); ++i)
  {
    aDef.Poles.push_back (aPoles.Value (i));
  }
  if (aDef.Rational)
  {
    const NCollection_Array1<double>& aWeights = *theSpline.Weights();
    for (int i = aWeights.Lower(); i <= aWeights.Upper(); ++i)
    {
      aDef.Weights.push_back (aWeights.Value (i));
    }
  }
  const NCollection_Array1<double>& aKnots = theSpline.Knots();
  const NCollection_Array1<int>& aMults = theSpline.Multiplicities();
  for (int i = aKnots.Lower(); i <= aKnots.Upper(); ++i)
  {
    aDef.Knots.push_back (aKnots.Value (i));
    aDef.Mults.push_back (aMults.Value (i));
  }
  return aDef;
}

// Validates theDef and builds a new spline from it; throws NvException with
// theMethod context on any inconsistency.
occ::handle<NvGeNurb3dData> DataFromDefinition (const Definition& theDef, const char* theMethod)
{
  const std::string aCtx = std::string ("NvGeNurbCurve3d::") + theMethod + "(): ";
  if (theDef.Degree < 1 || theDef.Degree > Geom_BSplineCurve::MaxDegree())
  {
    throw NvException (aCtx + "the degree is out of range");
  }
  if (theDef.Knots.size() < 2 || theDef.Knots.size() != theDef.Mults.size())
  {
    throw NvException (aCtx + "the knot vector is degenerate");
  }
  const size_t aKnotCount = theDef.Mults.size();
  for (size_t i = 0; i < aKnotCount; ++i)
  {
    const bool anEnd = (i == 0 || i == aKnotCount - 1) && !theDef.Periodic;
    const int aMaxMult = anEnd ? theDef.Degree + 1 : theDef.Degree;
    if (theDef.Mults[i] < 1 || theDef.Mults[i] > aMaxMult)
    {
      throw NvException (aCtx + "a knot multiplicity is out of range");
    }
  }
  if (theDef.Periodic && theDef.Mults.front() != theDef.Mults.back())
  {
    throw NvException (aCtx + "the first and last multiplicities must match on a periodic spline");
  }
  long long aSum = 0;
  for (size_t i = 0; i < aKnotCount; ++i)
  {
    aSum += theDef.Mults[i];
  }
  const long long anExpected = theDef.Periodic
                             ? aSum - theDef.Mults.front()
                             : aSum - theDef.Degree - 1;
  if (static_cast<long long> (theDef.Poles.size()) != anExpected)
  {
    throw NvException (aCtx + "the control point count does not match the knot vector");
  }
  if (!theDef.Weights.empty() && theDef.Weights.size() != theDef.Poles.size())
  {
    throw NvException (aCtx + "the weight count does not match the control point count");
  }
  for (size_t i = 0; i < theDef.Weights.size(); ++i)
  {
    if (theDef.Weights[i] <= gp::Resolution())
    {
      throw NvException (aCtx + "a weight is not positive");
    }
  }
  NCollection_Array1<gp_Pnt> aPoles (1, static_cast<int> (theDef.Poles.size()));
  for (size_t i = 0; i < theDef.Poles.size(); ++i)
  {
    aPoles.SetValue (static_cast<int> (i) + 1, theDef.Poles[i]);
  }
  NCollection_Array1<double> aKnots (1, static_cast<int> (theDef.Knots.size()));
  NCollection_Array1<int> aMults (1, static_cast<int> (theDef.Mults.size()));
  for (size_t i = 0; i < theDef.Knots.size(); ++i)
  {
    aKnots.SetValue (static_cast<int> (i) + 1, theDef.Knots[i]);
    aMults.SetValue (static_cast<int> (i) + 1, theDef.Mults[i]);
  }
  try
  {
    if (theDef.Weights.empty())
    {
      return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (aPoles, aKnots, aMults, theDef.Degree, theDef.Periodic));
    }
    NCollection_Array1<double> aWeights (1, static_cast<int> (theDef.Weights.size()));
    for (size_t i = 0; i < theDef.Weights.size(); ++i)
    {
      aWeights.SetValue (static_cast<int> (i) + 1, theDef.Weights[i]);
    }
    return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (aPoles, aWeights, aKnots, aMults, theDef.Degree, theDef.Periodic));
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
}

// Distance from thePnt to the segment [theSegA, theSegB].
double DistToSegment (const NvGePoint3d& thePnt, const NvGePoint3d& theSegA, const NvGePoint3d& theSegB)
{
  const NvGeVector3d aDir = theSegB - theSegA;
  const double aLen2 = aDir.lengthSqrd();
  if (aLen2 <= 0.0)
  {
    return thePnt.distanceTo (theSegA);
  }
  const NvGeVector3d anAp = thePnt - theSegA;
  double aT = anAp.dotProduct (aDir) / aLen2;
  aT = std::max (0.0, std::min (1.0, aT));
  return thePnt.distanceTo (theSegA + aDir * aT);
}

// Cumulative parameters over thePoints for theKp parameterization; appends
// the closing (wrap) parameter when theWrapEnd is set (periodic fit).
std::vector<double> ComputeParams (const NvGePoint3dArray& thePoints,
                                   NvGe::KnotParameterization theKp,
                                   const bool theWrapEnd)
{
  const int aCount = thePoints.length();
  std::vector<double> aParams;
  aParams.reserve (aCount + (theWrapEnd ? 1 : 0));
  if (theKp == NvGe::kUniform)
  {
    for (int i = 0; i < aCount; ++i)
    {
      aParams.push_back (static_cast<double> (i));
    }
  }
  else
  {
    aParams.push_back (0.0);
    for (int i = 1; i < aCount; ++i)
    {
      const double aStep = thePoints[i - 1].distanceTo (thePoints[i]);
      const double aPrev = aParams.back();
      aParams.push_back (theKp == NvGe::kSqrtChord ? aPrev + std::sqrt (std::max (aStep, 0.0))
                                                   : aPrev + aStep);
    }
  }
  if (theWrapEnd && aCount >= 2)
  {
    const double aPrev = aParams.back();
    if (theKp == NvGe::kUniform)
    {
      aParams.push_back (aPrev + 1.0);
    }
    else
    {
      const double aStep = thePoints[aCount - 1].distanceTo (thePoints[0]);
      aParams.push_back (aPrev + (theKp == NvGe::kSqrtChord ? std::sqrt (std::max (aStep, 0.0)) : aStep));
    }
  }
  return aParams;
}

// Builds an interpolating spline through theFit and attaches theFit as side
// data. Throws NvException with theMethod context on invalid fit data or a
// failed interpolation.
occ::handle<NvGeNurb3dData> InterpolateFit (const NvGeFitData3d& theFit, const char* theMethod)
{
  const std::string aCtx = std::string ("NvGeNurbCurve3d::") + theMethod + "(): ";
  const int aCount = theFit.Points.length();
  if (aCount < 2)
  {
    throw NvException (aCtx + "at least two fit points are required");
  }
  if (theFit.TangentsExist && theFit.Tangents.length() != aCount)
  {
    throw NvException (aCtx + "the tangent count does not match the fit point count");
  }
  // Periodic fit data conventionally repeats the first point at the end;
  // the interpolator closes the curve itself, so drop that duplicate.
  std::vector<gp_Pnt> aPnts;
  aPnts.reserve (aCount);
  for (int i = 0; i < aCount; ++i)
  {
    aPnts.push_back (gp_Pnt (theFit.Points[i].x, theFit.Points[i].y, theFit.Points[i].z));
  }
  const bool aDropWrap = theFit.Periodic && aCount >= 2
    && theFit.Points[0].distanceTo (theFit.Points[aCount - 1]) <= theFit.Tolerance.equalPoint();
  if (aDropWrap)
  {
    aPnts.pop_back();
  }
  const int aUseCount = static_cast<int> (aPnts.size());
  if (aUseCount < 2)
  {
    throw NvException (aCtx + "at least two distinct fit points are required");
  }

  // Parameters: explicit when consistent, otherwise per knot parameterization.
  std::vector<double> aParams;
  const int anExplicit = static_cast<int> (theFit.Params.size());
  if (anExplicit == aCount + (theFit.Periodic ? 1 : 0))
  {
    aParams = theFit.Params;
  }
  else
  {
    aParams = ComputeParams (theFit.Points, theFit.KnotParam, theFit.Periodic);
  }
  if (aDropWrap)
  {
    aParams.erase (aParams.begin() + aUseCount);
  }

  occ::handle<NCollection_HArray1<gp_Pnt>> aPoints = new NCollection_HArray1<gp_Pnt> (1, aUseCount);
  for (int i = 0; i < aUseCount; ++i)
  {
    aPoints->SetValue (i + 1, aPnts[i]);
  }
  occ::handle<NCollection_HArray1<double>> aParameters = new NCollection_HArray1<double> (1, static_cast<int> (aParams.size()));
  for (size_t i = 0; i < aParams.size(); ++i)
  {
    aParameters->SetValue (static_cast<int> (i) + 1, aParams[i]);
  }
  GeomAPI_Interpolate anInterp (aPoints, aParameters, theFit.Periodic, theFit.Tolerance.equalPoint());

  // Tangency constraints: per-point tangents when present, otherwise the
  // end tangents through the tangent-defined flags.
  NCollection_Array1<gp_Vec> aTangents (1, aUseCount);
  occ::handle<NCollection_HArray1<bool>> aFlags = new NCollection_HArray1<bool> (1, aUseCount);
  bool aLoad = false;
  if (theFit.TangentsExist)
  {
    for (int i = 0; i < aUseCount; ++i)
    {
      aTangents.SetValue (i + 1, gp_Vec (theFit.Tangents[i].x, theFit.Tangents[i].y, theFit.Tangents[i].z));
      aFlags->SetValue (i + 1, true);
    }
    aLoad = true;
  }
  else if (theFit.StartTangentDefined || theFit.EndTangentDefined)
  {
    aTangents.SetValue (1, gp_Vec (theFit.StartTangent.x, theFit.StartTangent.y, theFit.StartTangent.z));
    aTangents.SetValue (aUseCount, gp_Vec (theFit.EndTangent.x, theFit.EndTangent.y, theFit.EndTangent.z));
    aFlags->SetValue (1, theFit.StartTangentDefined);
    aFlags->SetValue (aUseCount, theFit.EndTangentDefined);
    aLoad = true;
  }
  if (aLoad)
  {
    anInterp.Load (aTangents, aFlags, false);
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
  return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (*anInterp.Curve(), theFit));
}

// Exact rational quadratic B-spline over the ellipse arc: spans of at most a
// quadrant with the tangent-intersection middle pole and cos (span / 2)
// middle weights. A 3d ellipse arc runs counterclockwise about its normal.
occ::handle<NvGeNurb3dData> EllipseSpline (const NvGeEllipArc3d& theEllipse)
{
  const NvGePoint3d aCenter = theEllipse.center();
  const NvGeVector3d anXAxis = theEllipse.majorAxis();
  const NvGeVector3d aYAxis = theEllipse.minorAxis();
  const double aMajorR = theEllipse.majorRadius();
  const double aMinorR = theEllipse.minorRadius();
  const double aStart = theEllipse.startAng();
  const double anEnd = theEllipse.endAng();
  if (anXAxis.isZeroLength() || aYAxis.isZeroLength() || aMajorR <= 0.0 || aMinorR <= 0.0)
  {
    throw NvException ("NvGeNurbCurve3d::NvGeNurbCurve3d(): the ellipse is degenerate");
  }
  double aSweep = anEnd - aStart;
  while (aSweep < 0.0)
  {
    aSweep += THE_TAU;
  }
  while (aSweep > THE_TAU)
  {
    aSweep -= THE_TAU;
  }
  if (aSweep <= 0.0)
  {
    aSweep = THE_TAU;
  }
  const int aSpanCount = std::max (1, static_cast<int> (std::ceil (aSweep / (THE_TAU / 4.0) - 1e-9)));
  const double aDt = aSweep / aSpanCount;

  // Ellipse arc point: affine image of the circle arc point.
  auto aPointAt = [aCenter, anXAxis, aYAxis, aMajorR, aMinorR] (double theAng)
  {
    return gp_Pnt (aCenter.x + aMajorR * std::cos (theAng) * anXAxis.x + aMinorR * std::sin (theAng) * aYAxis.x,
                   aCenter.y + aMajorR * std::cos (theAng) * anXAxis.y + aMinorR * std::sin (theAng) * aYAxis.y,
                   aCenter.z + aMajorR * std::cos (theAng) * anXAxis.z + aMinorR * std::sin (theAng) * aYAxis.z);
  };

  const double aMidWeight = std::cos (0.5 * aDt);
  const double aMidScale = 1.0 / aMidWeight;
  auto aMidPoleAt = [aCenter, anXAxis, aYAxis, aMajorR, aMinorR, aMidScale] (double theAng)
  {
    return gp_Pnt (aCenter.x + aMajorR * std::cos (theAng) * anXAxis.x * aMidScale + aMinorR * std::sin (theAng) * aYAxis.x * aMidScale,
                   aCenter.y + aMajorR * std::cos (theAng) * anXAxis.y * aMidScale + aMinorR * std::sin (theAng) * aYAxis.y * aMidScale,
                   aCenter.z + aMajorR * std::cos (theAng) * anXAxis.z * aMidScale + aMinorR * std::sin (theAng) * aYAxis.z * aMidScale);
  };

  Definition aDef;
  aDef.Degree = 2;
  aDef.Rational = true;
  for (int i = 0; i <= aSpanCount; ++i)
  {
    aDef.Poles.push_back (aPointAt (aStart + i * aDt));
    aDef.Weights.push_back (1.0);
    aDef.Knots.push_back (aStart + i * aDt);
    aDef.Mults.push_back (i == 0 || i == aSpanCount ? 3 : 2);
    if (i < aSpanCount)
    {
      aDef.Poles.push_back (aMidPoleAt (aStart + (i + 0.5) * aDt));
      aDef.Weights.push_back (aMidWeight);
    }
  }
  return DataFromDefinition (aDef, "NvGeNurbCurve3d");
}

// Appends the closing span (last pole = first pole) to a copy of theSrc when
// it is not geometrically closed yet; requires a clamped spline.
occ::handle<NvGeNurb3dData> ClosedCopy (const NvGeNurb3dData& theSrc, const char* theMethod)
{
  if (theSrc.EndPoint().Distance (theSrc.StartPoint()) <= NvGeContext::gTol.equalPoint())
  {
    return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (theSrc));
  }
  Definition aDef = DefinitionOf (theSrc);
  if (aDef.Mults.front() != aDef.Degree + 1 || aDef.Mults.back() != aDef.Degree + 1)
  {
    throw NvException (std::string ("NvGeNurbCurve3d::") + theMethod + "(): a clamped spline is required");
  }
  aDef.Poles.push_back (aDef.Poles.front());
  if (aDef.Rational)
  {
    aDef.Weights.push_back (aDef.Weights.front());
  }
  aDef.Mults.back() = 1;
  const double aLastSpan = aDef.Knots.back() - aDef.Knots[aDef.Knots.size() - 2];
  aDef.Knots.push_back (aDef.Knots.back() + aLastSpan);
  aDef.Mults.push_back (aDef.Degree + 1);
  occ::handle<NvGeNurb3dData> aRes = DataFromDefinition (aDef, theMethod);
  aRes->Fit = theSrc.Fit;
  return aRes;
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
occ::handle<NvGeNurb3dData> PrivateCopy (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  const NvGeNurb3dData* aSrc = FitDataOf (theSpline);
  if (aSrc != nullptr)
  {
    return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (*aSrc));
  }
  return occ::handle<NvGeNurb3dData> (new NvGeNurb3dData (*theSpline));
}

} // namespace

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d ()
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
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d,
                                  occ::handle<Geom_BSplineCurve> (new NvGeNurb3dData (aPoles, aKnots, aMults, 1, false)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGeNurbCurve3d& theSrc)
: NvGeSplineEnt3d (theSrc)
{
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (int theDegree, const NvGeKnotVector& theKnots,
                                  const NvGePoint3dArray& theCntrlPnts, Nova::Boolean theIsPeriodic)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  Definition aDef;
  aDef.Degree = theDegree;
  aDef.Periodic = theIsPeriodic != Nova::kFalse;
  aDef.Poles.reserve (theCntrlPnts.length());
  for (int i = 0; i < theCntrlPnts.length(); ++i)
  {
    aDef.Poles.push_back (gp_Pnt (theCntrlPnts[i].x, theCntrlPnts[i].y, theCntrlPnts[i].z));
  }
  const KnotStructure aKnots = KnotsAndMults (theKnots);
  aDef.Knots = aKnots.Knots;
  aDef.Mults = aKnots.Mults;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, DataFromDefinition (aDef, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (int theDegree, const NvGeKnotVector& theKnots,
                                  const NvGePoint3dArray& theCntrlPnts,
                                  const NvGeDoubleArray& theWeights, Nova::Boolean theIsPeriodic)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  Definition aDef;
  aDef.Degree = theDegree;
  aDef.Periodic = theIsPeriodic != Nova::kFalse;
  aDef.Poles.reserve (theCntrlPnts.length());
  for (int i = 0; i < theCntrlPnts.length(); ++i)
  {
    aDef.Poles.push_back (gp_Pnt (theCntrlPnts[i].x, theCntrlPnts[i].y, theCntrlPnts[i].z));
    aDef.Weights.push_back (theWeights[i]);
  }
  const KnotStructure aKnots = KnotsAndMults (theKnots);
  aDef.Knots = aKnots.Knots;
  aDef.Mults = aKnots.Mults;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, DataFromDefinition (aDef, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (int theDegree, const NvGePolyline3d& theFitPolyline,
                                  Nova::Boolean theIsPeriodic)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (theDegree < 2)
  {
    throw NvException ("NvGeNurbCurve3d::NvGeNurbCurve3d(): the degree must be at least 2");
  }
  NvGeFitData3d aFit;
  aFit.Periodic = theIsPeriodic != Nova::kFalse;
  const int aCount = theFitPolyline.numFitPoints();
  for (int i = 0; i < aCount; ++i)
  {
    aFit.Points.append (theFitPolyline.fitPointAt (i));
  }
  occ::handle<NvGeNurb3dData> aData = InterpolateFit (aFit, "NvGeNurbCurve3d");
  if (theDegree > 3)
  {
    try
    {
      aData->IncreaseDegree (theDegree);
    }
    catch (const Standard_Failure& aFailure)
    {
      throw NvException::FromFailure (aFailure);
    }
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGePoint3dArray& theFitPoints,
                                  const NvGeVector3d& theStartTangent,
                                  const NvGeVector3d& theEndTangent,
                                  Nova::Boolean theStartTangentDefined,
                                  Nova::Boolean theEndTangentDefined,
                                  const NvGeTol& theFitTolerance)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.EndTangentsExist = true;
  aFit.StartTangent = theStartTangent;
  aFit.EndTangent = theEndTangent;
  aFit.StartTangentDefined = theStartTangentDefined != Nova::kFalse;
  aFit.EndTangentDefined = theEndTangentDefined != Nova::kFalse;
  aFit.Tolerance = theFitTolerance;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, InterpolateFit (aFit, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGePoint3dArray& theFitPoints,
                                  const NvGeVector3d& theStartTangent,
                                  const NvGeVector3d& theEndTangent,
                                  Nova::Boolean theStartTangentDefined,
                                  Nova::Boolean theEndTangentDefined,
                                  NvGe::KnotParameterization theKnotParam,
                                  const NvGeTol& theFitTolerance)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (theKnotParam == NvGe::kNotDefinedKnotParam)
  {
    throw NvException ("NvGeNurbCurve3d::NvGeNurbCurve3d(): the knot parameterization is not defined");
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.EndTangentsExist = true;
  aFit.StartTangent = theStartTangent;
  aFit.EndTangent = theEndTangent;
  aFit.StartTangentDefined = theStartTangentDefined != Nova::kFalse;
  aFit.EndTangentDefined = theEndTangentDefined != Nova::kFalse;
  aFit.KnotParam = theKnotParam;
  aFit.Tolerance = theFitTolerance;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, InterpolateFit (aFit, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGePoint3dArray& theFitPoints, const NvGeTol& theFitTolerance)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.Tolerance = theFitTolerance;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, InterpolateFit (aFit, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGePoint3dArray& theFitPoints,
                                  const NvGeVector3dArray& theFitTangents,
                                  const NvGeTol& theFitTolerance,
                                  Nova::Boolean theIsPeriodic)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.Tangents = theFitTangents;
  aFit.TangentsExist = true;
  aFit.Periodic = theIsPeriodic != Nova::kFalse;
  aFit.Tolerance = theFitTolerance;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, InterpolateFit (aFit, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGeCurve3d& theCurve, double theEpsilon)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (theEpsilon <= 0.0)
  {
    throw NvException ("NvGeNurbCurve3d::NvGeNurbCurve3d(): the approximation tolerance must be positive");
  }
  NvGeInterval anInterval;
  theCurve.getInterval (anInterval);
  if (!anInterval.isBounded())
  {
    throw NvException ("NvGeNurbCurve3d::NvGeNurbCurve3d(): the curve has no bounded interval");
  }
  NvGeFitData3d aFit;
  aFit.Tolerance.setEqualPoint (theEpsilon);
  const std::vector<double> aParams = AdaptiveParams (theCurve, anInterval.lowerBound(),
                                                      anInterval.upperBound(), theEpsilon);
  for (size_t i = 0; i < aParams.size(); ++i)
  {
    aFit.Points.append (theCurve.evalPoint (aParams[i]));
    aFit.Params.push_back (aParams[i]);
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, InterpolateFit (aFit, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGeEllipArc3d& theEllipse)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, EllipseSpline (theEllipse));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbCurve3d::NvGeNurbCurve3d (const NvGeLineSeg3d& theLinSeg)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const NvGePoint3d aStart = theLinSeg.startPoint();
  const NvGePoint3d anEnd = theLinSeg.endPoint();
  Definition aDef;
  aDef.Degree = 1;
  aDef.Poles.push_back (gp_Pnt (aStart.x, aStart.y, aStart.z));
  aDef.Poles.push_back (gp_Pnt (anEnd.x, anEnd.y, anEnd.z));
  aDef.Knots.push_back (0.0);
  aDef.Knots.push_back (1.0);
  aDef.Mults.push_back (2);
  aDef.Mults.push_back (2);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbCurve3d, DataFromDefinition (aDef, "NvGeNurbCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

int NvGeNurbCurve3d::numFitPoints () const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  return aData != nullptr ? aData->Fit.Points.length() : 0;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitPointAt (int theIndex, NvGePoint3d& thePoint) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr || theIndex < 0 || theIndex >= aData->Fit.Points.length())
  {
    return Nova::kFalse;
  }
  thePoint = aData->Fit.Points[theIndex];
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitTolerance (NvGeTol& theFitTolerance) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr)
  {
    return Nova::kFalse;
  }
  theFitTolerance = aData->Fit.Tolerance;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitTangents (NvGeVector3d& theStartTangent,
                                                NvGeVector3d& theEndTangent) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr || !aData->Fit.EndTangentsExist)
  {
    return Nova::kFalse;
  }
  theStartTangent = aData->Fit.StartTangent;
  theEndTangent = aData->Fit.EndTangent;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitTangents (NvGeVector3d& theStartTangent,
                                                NvGeVector3d& theEndTangent,
                                                Nova::Boolean& theStartTangentDefined,
                                                Nova::Boolean& theEndTangentDefined) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr)
  {
    return Nova::kFalse;
  }
  theStartTangent = aData->Fit.StartTangent;
  theEndTangent = aData->Fit.EndTangent;
  theStartTangentDefined = aData->Fit.StartTangentDefined ? Nova::kTrue : Nova::kFalse;
  theEndTangentDefined = aData->Fit.EndTangentDefined ? Nova::kTrue : Nova::kFalse;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitKnotParameterization (KnotParameterization& theKnotParam) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr)
  {
    return Nova::kFalse;
  }
  theKnotParam = aData->Fit.KnotParam;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitData (NvGePoint3dArray& theFitPoints,
                                            NvGeTol& theFitTolerance,
                                            Nova::Boolean& theTangentsExist,
                                            NvGeVector3d& theStartTangent,
                                            NvGeVector3d& theEndTangent) const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  if (aData == nullptr)
  {
    return Nova::kFalse;
  }
  theFitPoints = aData->Fit.Points;
  theFitTolerance = aData->Fit.Tolerance;
  theTangentsExist = aData->Fit.EndTangentsExist ? Nova::kTrue : Nova::kFalse;
  theStartTangent = aData->Fit.StartTangent;
  theEndTangent = aData->Fit.EndTangent;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getFitData (NvGePoint3dArray& theFitPoints,
                                            NvGeTol& theFitTolerance,
                                            Nova::Boolean& theTangentsExist,
                                            NvGeVector3d& theStartTangent,
                                            NvGeVector3d& theEndTangent,
                                            KnotParameterization& theKnotParam) const
{
  if (!getFitData (theFitPoints, theFitTolerance, theTangentsExist, theStartTangent, theEndTangent))
  {
    return Nova::kFalse;
  }
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  theKnotParam = aData->Fit.KnotParam;
  return Nova::kTrue;
}

//=================================================================================================

void NvGeNurbCurve3d::getDefinitionData (int& theDegree, Nova::Boolean& theRational,
                                         Nova::Boolean& thePeriodic, NvGeKnotVector& theKnots,
                                         NvGePoint3dArray& theControlPoints,
                                         NvGeDoubleArray& theWeights) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  theDegree = aSpline->Degree();
  theRational = aSpline->Weights() != nullptr ? Nova::kTrue : Nova::kFalse;
  thePeriodic = aSpline->IsPeriodic() ? Nova::kTrue : Nova::kFalse;
  theKnots = FlatKnotVector (*aSpline);
  theControlPoints.removeAll();
  const NCollection_Array1<gp_Pnt>& aPoles = aSpline->Poles();
  for (int i = aPoles.Lower(); i <= aPoles.Upper(); ++i)
  {
    theControlPoints.append (NvGePoint3d (aPoles.Value (i).X(), aPoles.Value (i).Y(), aPoles.Value (i).Z()));
  }
  theWeights.removeAll();
  if (aSpline->Weights() != nullptr)
  {
    const NCollection_Array1<double>& aWeights = *aSpline->Weights();
    for (int i = aWeights.Lower(); i <= aWeights.Upper(); ++i)
    {
      theWeights.append (aWeights.Value (i));
    }
  }
}

//=================================================================================================

int NvGeNurbCurve3d::numWeights () const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  return aSpline->Weights() != nullptr ? aSpline->NbPoles() : 0;
}

//=================================================================================================

double NvGeNurbCurve3d::weightAt (int theIdx) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (theIdx < 0 || theIdx >= aSpline->NbPoles())
  {
    throw NvException ("NvGeNurbCurve3d::weightAt(): the index is out of range");
  }
  return aSpline->Weights() != nullptr ? aSpline->Weight (theIdx + 1) : 1.0;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::evalMode () const
{
  const NvGeNurb3dData* aData = FitDataOf (SplineOf (mpImpEnt));
  return aData != nullptr && aData->Fit.EvalModeFlag ? Nova::kTrue : Nova::kFalse;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getParamsOfC1Discontinuity (NvGeDoubleArray& theParams,
                                                            const NvGeTol&) const
{
  theParams.removeAll();
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->IsPeriodic())
  {
    return Nova::kTrue;
  }
  const int aNbKnots = aSpline->NbKnots();
  for (int i = 2; i <= aNbKnots - 1; ++i)
  {
    if (aSpline->Multiplicity (i) >= aSpline->Degree())
    {
      theParams.append (aSpline->Knot (i));
    }
  }
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::getParamsOfG1Discontinuity (NvGeDoubleArray& theParams,
                                                            const NvGeTol& theTol) const
{
  theParams.removeAll();
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const int aNbKnots = aSpline->NbKnots();
  const double anAngTol = theTol.equalVector();
  for (int i = 2; i <= aNbKnots - 1; ++i)
  {
    const double aKnot = aSpline->Knot (i);
    gp_Pnt aPnt;
    gp_Vec aLeft;
    gp_Vec aRight;
    aSpline->LocalD1 (aKnot, i - 1, i, aPnt, aLeft);
    aSpline->LocalD1 (aKnot, i, i + 1, aPnt, aRight);
    if (aLeft.Magnitude() <= gp::Resolution() || aRight.Magnitude() <= gp::Resolution()
        || aLeft.Angle (aRight) > anAngTol)
    {
      theParams.append (aKnot);
    }
  }
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::setFitPointAt (int theIndex, const NvGePoint3d& thePoint)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || aSrc->Fit.Points.length() == 0
      || theIndex < 0 || theIndex >= aSrc->Fit.Points.length())
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit.Points[theIndex] = thePoint;
  if (aData->Fit.Periodic && (theIndex == 0 || theIndex == aData->Fit.Points.length() - 1))
  {
    // Keep the first point and its wrap duplicate coincident.
    aData->Fit.Points[0] = thePoint;
    aData->Fit.Points[aData->Fit.Points.length() - 1] = thePoint;
  }
  ReplaceGeometry (mpImpEnt, InterpolateFit (aData->Fit, "setFitPointAt"));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::addFitPointAt (int theIndex, const NvGePoint3d& thePoint)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || aSrc->Fit.Periodic
      || theIndex < 0 || theIndex > aSrc->Fit.Points.length())
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  NvGePoint3dArray aPoints;
  for (int i = 0; i <= aData->Fit.Points.length(); ++i)
  {
    if (i == theIndex)
    {
      aPoints.append (thePoint);
    }
    if (i < aData->Fit.Points.length())
    {
      aPoints.append (aData->Fit.Points[i]);
    }
  }
  aData->Fit.Points = aPoints;
  aData->Fit.Params.clear();   // parameters are recomputed on rebuild
  ReplaceGeometry (mpImpEnt, InterpolateFit (aData->Fit, "addFitPointAt"));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::deleteFitPointAt (int theIndex)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || aSrc->Fit.Periodic
      || theIndex < 0 || theIndex >= aSrc->Fit.Points.length()
      || aSrc->Fit.Points.length() <= 2)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  NvGePoint3dArray aPoints;
  for (int i = 0; i < aData->Fit.Points.length(); ++i)
  {
    if (i != theIndex)
    {
      aPoints.append (aData->Fit.Points[i]);
    }
  }
  aData->Fit.Points = aPoints;
  aData->Fit.Params.clear();
  ReplaceGeometry (mpImpEnt, InterpolateFit (aData->Fit, "deleteFitPointAt"));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::setFitTolerance (const NvGeTol& theFitTol)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (FitDataOf (aSpline) == nullptr)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit.Tolerance = theFitTol;
  ReplaceGeometry (mpImpEnt, aData);   // tolerance affects future fits only
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::setFitTangents (const NvGeVector3d& theStartTangent,
                                                const NvGeVector3d& theEndTangent)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || aSrc->Fit.Periodic)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit.EndTangentsExist = true;
  aData->Fit.StartTangent = theStartTangent;
  aData->Fit.EndTangent = theEndTangent;
  aData->Fit.StartTangentDefined = true;
  aData->Fit.EndTangentDefined = true;
  ReplaceGeometry (mpImpEnt, InterpolateFit (aData->Fit, "setFitTangents"));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::setFitTangents (const NvGeVector3d& theStartTangent,
                                                const NvGeVector3d& theEndTangent,
                                                Nova::Boolean theStartTangentDefined,
                                                Nova::Boolean theEndTangentDefined) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc == nullptr || aSrc->Fit.Periodic)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit.EndTangentsExist = true;
  aData->Fit.StartTangent = theStartTangent;
  aData->Fit.EndTangent = theEndTangent;
  aData->Fit.StartTangentDefined = theStartTangentDefined != Nova::kFalse;
  aData->Fit.EndTangentDefined = theEndTangentDefined != Nova::kFalse;
  occ::handle<Geom_BSplineCurve> aRes = InterpolateFit (aData->Fit, "setFitTangents");
  // The header marks this overload const; run the copy-on-write detach
  // through a local imp pointer and write it back explicitly.
  NvGeImpEntity3d* anImp = mpImpEnt;
  ReplaceGeometry (anImp, aRes);
  const_cast<NvGeNurbCurve3d*> (this)->mpImpEnt = anImp;
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::setFitKnotParameterization (KnotParameterization theKnotParam)
{
  if (theKnotParam != NvGe::kChord && theKnotParam != NvGe::kSqrtChord
      && theKnotParam != NvGe::kUniform)
  {
    return Nova::kFalse;
  }
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (FitDataOf (aSpline) == nullptr)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit.KnotParam = theKnotParam;
  aData->Fit.Params.clear();
  ReplaceGeometry (mpImpEnt, InterpolateFit (aData->Fit, "setFitKnotParameterization"));
  return Nova::kTrue;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::setFitData (const NvGePoint3dArray& theFitPoints,
                                              const NvGeVector3d& theStartTangent,
                                              const NvGeVector3d& theEndTangent,
                                              const NvGeTol& theFitTol)
{
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.EndTangentsExist = true;
  aFit.StartTangent = theStartTangent;
  aFit.EndTangent = theEndTangent;
  aFit.StartTangentDefined = true;
  aFit.EndTangentDefined = true;
  aFit.Tolerance = theFitTol;
  ReplaceGeometry (mpImpEnt, InterpolateFit (aFit, "setFitData"));
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::setFitData (const NvGePoint3dArray& theFitPoints,
                                              const NvGeVector3d& theStartTangent,
                                              const NvGeVector3d& theEndTangent,
                                              KnotParameterization theKnotParam,
                                              const NvGeTol& theFitTol)
{
  if (theKnotParam == NvGe::kNotDefinedKnotParam)
  {
    throw NvException ("NvGeNurbCurve3d::setFitData(): the knot parameterization is not defined");
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.EndTangentsExist = true;
  aFit.StartTangent = theStartTangent;
  aFit.EndTangent = theEndTangent;
  aFit.StartTangentDefined = true;
  aFit.EndTangentDefined = true;
  aFit.KnotParam = theKnotParam;
  aFit.Tolerance = theFitTol;
  ReplaceGeometry (mpImpEnt, InterpolateFit (aFit, "setFitData"));
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::setFitData (const NvGeKnotVector& theFitKnots,
                                              const NvGePoint3dArray& theFitPoints,
                                              const NvGeVector3d& theStartTangent,
                                              const NvGeVector3d& theEndTangent,
                                              const NvGeTol& theFitTol,
                                              Nova::Boolean theIsPeriodic)
{
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.EndTangentsExist = true;
  aFit.StartTangent = theStartTangent;
  aFit.EndTangent = theEndTangent;
  aFit.StartTangentDefined = true;
  aFit.EndTangentDefined = true;
  aFit.Periodic = theIsPeriodic != Nova::kFalse;
  aFit.KnotParam = NvGe::kCustomParameterization;
  aFit.Tolerance = theFitTol;
  const int aCount = theFitPoints.length();
  const int aNeed = theFitKnots.length();
  if (aNeed != aCount + (aFit.Periodic ? 1 : 0))
  {
    throw NvException ("NvGeNurbCurve3d::setFitData(): the fit knot count does not match the"
                       " fit point count (one parameter per point, plus the closing parameter"
                       " for a periodic fit)");
  }
  for (int i = 1; i < aNeed; ++i)
  {
    if (theFitKnots[i] - theFitKnots[i - 1] <= theFitKnots.tolerance())
    {
      throw NvException ("NvGeNurbCurve3d::setFitData(): the fit knots must be strictly increasing");
    }
  }
  for (int i = 0; i < aNeed; ++i)
  {
    aFit.Params.push_back (theFitKnots[i]);
  }
  if (aFit.Periodic)
  {
    aFit.PeriodEndParam = theFitKnots[aNeed - 1];
  }
  ReplaceGeometry (mpImpEnt, InterpolateFit (aFit, "setFitData"));
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::setFitData (int theDegree,
                                              const NvGePoint3dArray& theFitPoints,
                                              const NvGeTol& theFitTol)
{
  if (theDegree < 2)
  {
    throw NvException ("NvGeNurbCurve3d::setFitData(): the degree must be at least 2");
  }
  if (theDegree > Geom_BSplineCurve::MaxDegree())
  {
    throw NvException ("NvGeNurbCurve3d::setFitData(): the degree is out of range");
  }
  NvGeFitData3d aFit;
  aFit.Points = theFitPoints;
  aFit.Tolerance = theFitTol;
  occ::handle<NvGeNurb3dData> aData = InterpolateFit (aFit, "setFitData");
  if (theDegree > 3)
  {
    try
    {
      aData->IncreaseDegree (theDegree);
    }
    catch (const Standard_Failure& aFailure)
    {
      throw NvException::FromFailure (aFailure);
    }
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::purgeFitData ()
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (FitDataOf (aSpline) == nullptr)
  {
    return Nova::kFalse;
  }
  // Copy() yields a plain Geom_BSplineCurve without side data.
  ReplaceGeometry (mpImpEnt, occ::down_cast<Geom_BSplineCurve> (aSpline->Copy()));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::buildFitData ()
{
  return buildFitData (NvGe::kChord);
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::buildFitData (KnotParameterization theKp)
{
  if (theKp != NvGe::kChord && theKp != NvGe::kSqrtChord && theKp != NvGe::kUniform)
  {
    return Nova::kFalse;
  }
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  NvGeFitData3d aFit;
  aFit.KnotParam = theKp;
  aFit.Tolerance = NvGeContext::gTol;
  aFit.Periodic = aSpline->IsPeriodic();
  const int aNbKnots = aSpline->NbKnots();
  for (int i = 1; i <= aNbKnots; ++i)
  {
    const double aKnot = aSpline->Knot (i);
    if (aFit.Periodic && i == aNbKnots)
    {
      aFit.PeriodEndParam = aKnot;
      break;
    }
    aFit.Points.append (evalPoint (aKnot));
    aFit.Params.push_back (aKnot);
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  aData->Fit = aFit;
  ReplaceGeometry (mpImpEnt, aData);   // geometry is unchanged; fit data attached
  return Nova::kTrue;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::addKnot (double theNewKnot)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (theNewKnot < aSpline->FirstParameter() || theNewKnot > aSpline->LastParameter())
  {
    throw NvException ("NvGeNurbCurve3d::addKnot(): the knot is outside the parameter range");
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  try
  {
    aData->InsertKnot (theNewKnot, 1, KnotToleranceOf (*aSpline));
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::insertKnot (double theNewKnot)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (theNewKnot < aSpline->FirstParameter() || theNewKnot > aSpline->LastParameter())
  {
    throw NvException ("NvGeNurbCurve3d::insertKnot(): the knot is outside the parameter range");
  }
  const double aKnotTol = KnotToleranceOf (*aSpline);
  const NCollection_Array1<double>& aKnots = aSpline->Knots();
  for (int i = aKnots.Lower(); i <= aKnots.Upper(); ++i)
  {
    if (std::abs (aKnots.Value (i) - theNewKnot) <= aKnotTol)
    {
      return *this;   // the knot is already present
    }
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  try
  {
    aData->InsertKnot (theNewKnot, 1, aKnotTol);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeSplineEnt3d& NvGeNurbCurve3d::setWeightAt (int theIdx, double theVal)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (theVal <= 0.0)
  {
    throw NvException ("NvGeNurbCurve3d::setWeightAt(): the weight must be positive");
  }
  if (theIdx < 0 || theIdx >= aSpline->NbPoles())
  {
    throw NvException ("NvGeNurbCurve3d::setWeightAt(): the index is out of range");
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  try
  {
    aData->SetWeight (theIdx + 1, theVal);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::setEvalMode (Nova::Boolean theEvalMode)
{
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (SplineOf (mpImpEnt));
  aData->Fit.EvalModeFlag = theEvalMode != Nova::kFalse;
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::joinWith (const NvGeNurbCurve3d& theCurve)
{
  const occ::handle<Geom_BSplineCurve> aLeft = SplineOf (mpImpEnt);
  const occ::handle<Geom_BSplineCurve> aRight = SplineOf (theCurve.mpImpEnt);
  if (aLeft->IsPeriodic() || aRight->IsPeriodic())
  {
    throw NvException ("NvGeNurbCurve3d::joinWith(): a periodic spline cannot be joined");
  }
  if (aLeft->EndPoint().Distance (aRight->StartPoint()) > NvGeContext::gTol.equalPoint())
  {
    throw NvException ("NvGeNurbCurve3d::joinWith(): the spline ends do not meet");
  }
  // Both sides must be clamped so the first/last poles coincide with the
  // curve ends and the junction multiplicity merge stays consistent.
  Definition aDefL = DefinitionOf (*aLeft);
  Definition aDefR = DefinitionOf (*aRight);
  if (aDefL.Mults.front() != aDefL.Degree + 1 || aDefL.Mults.back() != aDefL.Degree + 1
      || aDefR.Mults.front() != aDefR.Degree + 1 || aDefR.Mults.back() != aDefR.Degree + 1)
  {
    throw NvException ("NvGeNurbCurve3d::joinWith(): clamped splines are required");
  }
  const int aDegree = std::max (aDefL.Degree, aDefR.Degree);
  if (aDefL.Degree < aDegree)
  {
    occ::handle<NvGeNurb3dData> aLeftElev = PrivateCopy (aLeft);
    try
    {
      aLeftElev->IncreaseDegree (aDegree);
    }
    catch (const Standard_Failure& aFailure)
    {
      throw NvException::FromFailure (aFailure);
    }
    aDefL = DefinitionOf (*aLeftElev);
  }
  // Unify rationality with unit weights on a non-rational side.
  aDefL.Rational = aDefL.Rational || aDefR.Rational;
  aDefR.Rational = aDefL.Rational;
  if (aDefL.Rational && aDefL.Weights.empty())
  {
    aDefL.Weights.assign (aDefL.Poles.size(), 1.0);
  }
  if (aDefR.Rational && aDefR.Weights.empty())
  {
    aDefR.Weights.assign (aDefR.Poles.size(), 1.0);
  }
  aDefL.Degree = aDegree;
  aDefL.Periodic = false;
  // Merge: the left closing knot drops to C0 multiplicity; the right opening
  // knot and its first pole are absorbed at the junction.
  aDefL.Mults.back() = aDegree;
  const double anOffset = aDefL.Knots.back() - aDefR.Knots.front();
  for (size_t i = 1; i < aDefR.Knots.size(); ++i)
  {
    aDefL.Knots.push_back (aDefR.Knots[i] + anOffset);
    aDefL.Mults.push_back (aDefR.Mults[i]);
  }
  for (size_t i = 1; i < aDefR.Poles.size(); ++i)
  {
    aDefL.Poles.push_back (aDefR.Poles[i]);
  }
  if (aDefL.Rational)
  {
    for (size_t i = 1; i < aDefR.Weights.size(); ++i)
    {
      aDefL.Weights.push_back (aDefR.Weights[i]);
    }
  }
  ReplaceGeometry (mpImpEnt, DataFromDefinition (aDefL, "joinWith"));
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::hardTrimByParams (double theNewStartParam, double theNewEndParam)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (theNewEndParam - theNewStartParam <= 0.0)
  {
    throw NvException ("NvGeNurbCurve3d::hardTrimByParams(): the end parameter must be greater"
                       " than the start parameter");
  }
  if (theNewStartParam < aSpline->FirstParameter() || theNewEndParam > aSpline->LastParameter())
  {
    throw NvException ("NvGeNurbCurve3d::hardTrimByParams(): the parameters are outside the"
                       " curve interval");
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  const bool anEvalMode = aData->Fit.EvalModeFlag;
  try
  {
    aData->Segment (theNewStartParam, theNewEndParam);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  aData->Fit = NvGeFitData3d();   // fit parameters no longer match the trimmed curve
  aData->Fit.EvalModeFlag = anEvalMode;
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::makeRational (double theWeight)
{
  if (theWeight <= 0.0)
  {
    throw NvException ("NvGeNurbCurve3d::makeRational(): the weight must be positive");
  }
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->Weights() != nullptr)
  {
    return *this;   // already rational
  }
  // Uniform weights cancel in the homogeneous form, so the geometry is kept.
  Definition aDef = DefinitionOf (*aSpline);
  aDef.Weights.assign (aDef.Poles.size(), theWeight);
  occ::handle<NvGeNurb3dData> aData = DataFromDefinition (aDef, "makeRational");
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc != nullptr)
  {
    aData->Fit = aSrc->Fit;
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::makeClosed ()
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->IsPeriodic()
      || aSpline->EndPoint().Distance (aSpline->StartPoint()) <= NvGeContext::gTol.equalPoint())
  {
    return *this;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  ReplaceGeometry (mpImpEnt, ClosedCopy (*aData, "makeClosed"));
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::makePeriodic ()
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->IsPeriodic())
  {
    return *this;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  occ::handle<NvGeNurb3dData> aClosed = ClosedCopy (*aData, "makePeriodic");
  try
  {
    aClosed->SetPeriodic();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aClosed);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::makeNonPeriodic ()
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (!aSpline->IsPeriodic())
  {
    return *this;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  try
  {
    aData->SetNotPeriodic();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::makeOpen ()
{
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (SplineOf (mpImpEnt));
  if (aData->IsPeriodic())
  {
    try
    {
      aData->SetNotPeriodic();
    }
    catch (const Standard_Failure& aFailure)
    {
      throw NvException::FromFailure (aFailure);
    }
  }
  Definition aDef = DefinitionOf (*aData);
  if (aDef.Mults.empty()
      || aDef.Mults.front() != aDef.Degree + 1 || aDef.Mults.back() != aDef.Degree + 1
      || aDef.Poles.size() <= static_cast<size_t> (aDef.Degree + 1)
      || aData->EndPoint().Distance (aData->StartPoint()) > NvGeContext::gTol.equalPoint())
  {
    return *this;   // nothing to open
  }
  // Remove the closing span: last pole, last knot, and restore the previous
  // last knot multiplicity to the clamped value.
  aDef.Poles.pop_back();
  if (aDef.Rational)
  {
    aDef.Weights.pop_back();
  }
  aDef.Knots.pop_back();
  aDef.Mults.pop_back();
  aDef.Mults.back() = aDef.Degree + 1;
  occ::handle<NvGeNurb3dData> aRes = DataFromDefinition (aDef, "makeOpen");
  aRes->Fit = NvGeFitData3d();   // fit parameters spanned the removed wrap span
  aRes->Fit.EvalModeFlag = aData->Fit.EvalModeFlag;
  ReplaceGeometry (mpImpEnt, aRes);
  return *this;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::elevateDegree (int thePlusDegree)
{
  if (thePlusDegree < 0)
  {
    throw NvException ("NvGeNurbCurve3d::elevateDegree(): the degree increment must not be negative");
  }
  if (thePlusDegree == 0)
  {
    return *this;
  }
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  const int aTarget = aSpline->Degree() + thePlusDegree;
  if (aTarget > Geom_BSplineCurve::MaxDegree())
  {
    throw NvException ("NvGeNurbCurve3d::elevateDegree(): the resulting degree is out of range");
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  try
  {
    aData->IncreaseDegree (aTarget);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::addControlPointAt (double theNewKnot, const NvGePoint3d& thePoint,
                                                   double theWeight)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->IsPeriodic() || theWeight <= 0.0)
  {
    return Nova::kFalse;
  }
  const double aKnotTol = KnotToleranceOf (*aSpline);
  if (theNewKnot <= aSpline->FirstParameter() + aKnotTol
      || theNewKnot >= aSpline->LastParameter() - aKnotTol)
  {
    return Nova::kFalse;
  }
  occ::handle<NvGeNurb3dData> aData = PrivateCopy (aSpline);
  const int aNbKnotsBefore = aData->NbKnots();
  try
  {
    aData->InsertKnot (theNewKnot, aData->Degree(), aKnotTol);
  }
  catch (const Standard_Failure&)
  {
    return Nova::kFalse;
  }
  // The pole controlling the point at the inserted knot follows the poles of
  // all knots strictly before it.
  int aPoleIdx = 1;
  for (int i = 1; i <= aNbKnotsBefore; ++i)
  {
    if (aData->Knot (i) < theNewKnot - aKnotTol)
    {
      aPoleIdx += aData->Multiplicity (i);
    }
    else
    {
      break;
    }
  }
  try
  {
    if (theWeight == 1.0 && aSpline->Weights() == nullptr)
    {
      aData->SetPole (aPoleIdx, gp_Pnt (thePoint.x, thePoint.y, thePoint.z));
    }
    else
    {
      aData->SetPole (aPoleIdx, gp_Pnt (thePoint.x, thePoint.y, thePoint.z), theWeight);
    }
  }
  catch (const Standard_Failure&)
  {
    return Nova::kFalse;
  }
  ReplaceGeometry (mpImpEnt, aData);
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeNurbCurve3d::deleteControlPointAt (int theIndex)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt);
  if (aSpline->IsPeriodic()
      || theIndex < 0 || theIndex >= aSpline->NbPoles()
      || aSpline->NbPoles() <= aSpline->Degree() + 1)
  {
    return Nova::kFalse;
  }
  Definition aDef = DefinitionOf (*aSpline);
  // Reduce the multiplicity of the knot owning the pole, then drop the pole.
  int aCum = 0;
  size_t aKnotIdx = aDef.Mults.size();
  for (size_t j = 0; j < aDef.Mults.size(); ++j)
  {
    aCum += aDef.Mults[j];
    if (aCum >= theIndex + 1)
    {
      aKnotIdx = j;
      break;
    }
  }
  --aDef.Mults[aKnotIdx];
  if (aDef.Mults[aKnotIdx] == 0)
  {
    aDef.Knots.erase (aDef.Knots.begin() + static_cast<long> (aKnotIdx));
    aDef.Mults.erase (aDef.Mults.begin() + static_cast<long> (aKnotIdx));
  }
  aDef.Poles.erase (aDef.Poles.begin() + static_cast<long> (theIndex));
  if (aDef.Rational)
  {
    aDef.Weights.erase (aDef.Weights.begin() + static_cast<long> (theIndex));
  }
  occ::handle<NvGeNurb3dData> aData = DataFromDefinition (aDef, "deleteControlPointAt");
  const NvGeNurb3dData* aSrc = FitDataOf (aSpline);
  if (aSrc != nullptr)
  {
    aData->Fit = aSrc->Fit;
  }
  ReplaceGeometry (mpImpEnt, aData);
  return Nova::kTrue;
}

//=================================================================================================

NvGeNurbCurve3d& NvGeNurbCurve3d::operator = (const NvGeNurbCurve3d& theSpline)
{
  NvGeSplineEnt3d::operator= (theSpline);
  return *this;
}
