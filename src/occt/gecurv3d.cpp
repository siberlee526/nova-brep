// gecurv3d.cpp - implementation of NvGeCurve3d, the base of all 3d curves.
//
// Every curve entity stores an occ::handle<Geom_Curve> in its impl object:
// unbounded lines as Geom_Line, bounded arcs/segments as Geom_TrimmedCurve,
// rational/periodic curves as Geom_BSplineCurve, offsets as Geom_OffsetCurve.
// The base class implements the shared curve behavior (evaluation, closest
// points, intervals, planarity, arc length, projection) generically on top
// of that handle. Approximate results are computed by uniform sampling;
// the sample counts below are the single tuning point for accuracy vs. speed.

#include <gecurv3d.h>
#include <Nova.h>
#include <NvException.h>
#include <geblok3d.h>
#include <gedblar.h>
#include <geent3d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <geintarr.h>
#include <geintrvl.h>
#include <geline3d.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <geponc3d.h>
#include <gept3dar.h>
#include <getol.h>
#include <gevc3dar.h>
#include <gevec3d.h>
#include <gevptar.h>

#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Conic.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <NCollection_Array1.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Number of sample points for closest-point scans, bounding boxes and the
// point sets behind the planarity / linearity fits.
constexpr int THE_NB_SAMPLES = 33;

// Number of sample points for the polygonal area integration.
constexpr int THE_NB_AREA_SAMPLES = 65;

// Two pi; circle and ellipse curves are periodic with exactly this period.
constexpr double THE_TWO_PI = 6.28318530717958647692;

//! Converts an OCCT parameter bound (+-Precision::Infinite for unbounded
//! curves) into the corresponding native infinity so plain comparisons work.
double NormalizedBound (double theValue)
{
  if (Precision::IsInfinite (theValue))
  {
    return theValue > 0.0 ? std::numeric_limits<double>::infinity()
                          : -std::numeric_limits<double>::infinity();
  }
  return theValue;
}

//! Re-throws an OCCT failure as NvException, prefixed with the failing method.
NvException TranslatedFailure (const char* theMethod, const Standard_Failure& theFailure)
{
  const char* aDetail = theFailure.GetMessageString();
  if (aDetail == nullptr || aDetail[0] == '\0')
  {
    return NvException (std::string ("NvGeCurve3d::") + theMethod
                        + "(): the underlying OCCT operation failed");
  }
  return NvException (std::string ("NvGeCurve3d::") + theMethod + "(): " + aDetail);
}

//! Wraps an NvGePoint3d into a gp_Pnt.
gp_Pnt PntOf (const NvGePoint3d& thePnt)
{
  return gp_Pnt (thePnt.x, thePnt.y, thePnt.z);
}

//! Wraps a gp_Pnt into an NvGePoint3d.
NvGePoint3d Pnt3dOf (const gp_Pnt& thePnt)
{
  return NvGePoint3d (thePnt.X(), thePnt.Y(), thePnt.Z());
}

//! Returns the basis curve of a trimmed curve; any other curve is returned
//! unchanged, so callers always see the full underlying geometry.
occ::handle<Geom_Curve> BasisOf (const occ::handle<Geom_Curve>& theCurve)
{
  const occ::handle<Geom_TrimmedCurve> aTrimmed
    = occ::down_cast<Geom_TrimmedCurve> (theCurve);
  return aTrimmed.IsNull() ? theCurve : aTrimmed->BasisCurve();
}

//! Resolves a parameter window into a finite sampling window of the curve.
//! Fully bounded domains are used as-is; unbounded (or half-bounded) domains
//! fall back to a unit window anchored at the finite end or the origin, the
//! same convention as the curve equality test in geent3d.cpp.
void SamplingWindowOf (const occ::handle<Geom_Curve>& theCurve,
                       double& theLower, double& theUpper)
{
  const double aFirst = NormalizedBound (theCurve->FirstParameter());
  const double aLast  = NormalizedBound (theCurve->LastParameter());
  const bool aFirstInf = std::isinf (aFirst);
  const bool aLastInf  = std::isinf (aLast);
  if (!aFirstInf && !aLastInf)
  {
    theLower = aFirst;
    theUpper = aLast;
  }
  else if (aFirstInf && aLastInf)
  {
    theLower = -1.0;
    theUpper = 1.0;
  }
  else if (aFirstInf)
  {
    theLower = aLast - 2.0;
    theUpper = aLast;
  }
  else
  {
    theLower = aFirst;
    theUpper = aFirst + 2.0;
  }
}

//! Resolves an infinite requested window bound against the natural domain:
//! a finite domain clamps to its edge, an infinite domain keeps the bound at
//! the OCCT-infinite scale so that Geom_TrimmedCurve accepts it.
double WindowBoundOf (double theBound, double theDomainBound)
{
  if (!std::isinf (theBound))
  {
    return theBound;
  }
  if (!std::isinf (theDomainBound))
  {
    return theDomainBound;
  }
  return theBound < 0.0 ? -Precision::Infinite() : Precision::Infinite();
}

//! Samples the curve uniformly over [theFirst, theLast]. OCCT evaluation
//! failures are translated into NvException under the given method name.
void SampleCurve (const occ::handle<Geom_Curve>& theCurve,
                  double theFirst, double theLast, int theCount,
                  std::vector<gp_Pnt>& thePoints, const char* theMethod)
{
  thePoints.clear();
  thePoints.reserve (theCount);
  const double aStep = (theCount == 1)
    ? 0.0 : (theLast - theFirst) / double (theCount - 1);
  try
  {
    for (int i = 0; i < theCount; ++i)
    {
      gp_Pnt aPnt;
      theCurve->D0 (theFirst + i * aStep, aPnt);
      thePoints.push_back (aPnt);
    }
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure (theMethod, aFailure);
  }
}

//! Coordinate-wise min/max box of the sampled points.
void SampledBoundsOf (const std::vector<gp_Pnt>& thePoints,
                      gp_Pnt& theMinPnt, gp_Pnt& theMaxPnt)
{
  double aMin[3] = { thePoints[0].X(), thePoints[0].Y(), thePoints[0].Z() };
  double aMax[3] = { aMin[0], aMin[1], aMin[2] };
  for (const gp_Pnt& aPnt : thePoints)
  {
    const double aCoord[3] = { aPnt.X(), aPnt.Y(), aPnt.Z() };
    for (int i = 0; i < 3; ++i)
    {
      aMin[i] = std::min (aMin[i], aCoord[i]);
      aMax[i] = std::max (aMax[i], aCoord[i]);
    }
  }
  theMinPnt.SetCoord (aMin[0], aMin[1], aMin[2]);
  theMaxPnt.SetCoord (aMax[0], aMax[1], aMax[2]);
}

//! Coordinate-parallel bound block spanned by the sampled point box.
//! (boundBlock and orthoBoundBlock share this approximation; a true oriented
//! block would require a principal-axis fit that the base class does not need.)
NvGeBoundBlock3d SampledBoundBlockOf (const std::vector<gp_Pnt>& thePoints)
{
  gp_Pnt aMinPnt;
  gp_Pnt aMaxPnt;
  SampledBoundsOf (thePoints, aMinPnt, aMaxPnt);
  return NvGeBoundBlock3d (Pnt3dOf (aMinPnt),
                           NvGeVector3d (aMaxPnt.X() - aMinPnt.X(), 0.0, 0.0),
                           NvGeVector3d (0.0, aMaxPnt.Y() - aMinPnt.Y(), 0.0),
                           NvGeVector3d (0.0, 0.0, aMaxPnt.Z() - aMinPnt.Z()));
}

//! Newell-method plane normal of the sampled point set (robust for closed
//! loops); the magnitude may vanish for degenerate or collinear point sets.
gp_Vec FittedNormalOf (const std::vector<gp_Pnt>& thePoints)
{
  double aNx = 0.0;
  double aNy = 0.0;
  double aNz = 0.0;
  const int aNb = int (thePoints.size());
  for (int i = 0; i < aNb; ++i)
  {
    const gp_XYZ& aCur = thePoints[i].Coord();
    const gp_XYZ& aNxt = thePoints[(i + 1) % aNb].Coord();
    aNx += aCur.Y() * aNxt.Z() - aCur.Z() * aNxt.Y();
    aNy += aCur.Z() * aNxt.X() - aCur.X() * aNxt.Z();
    aNz += aCur.X() * aNxt.Y() - aCur.Y() * aNxt.X();
  }
  return gp_Vec (aNx, aNy, aNz);
}

//! Stable perpendicular of a non-degenerate vector: cross with the coordinate
//! axis the vector is least aligned with, so the result never degenerates.
gp_Vec PerpendicularOf (const gp_Vec& theVec)
{
  const gp_Vec aX (1.0, 0.0, 0.0);
  const gp_Vec aY (0.0, 1.0, 0.0);
  const gp_Vec aZ (0.0, 0.0, 1.0);
  const double aXDev = std::abs (theVec.Dot (aX));
  const double aYDev = std::abs (theVec.Dot (aY));
  const double aZDev = std::abs (theVec.Dot (aZ));
  const gp_Vec* anAxis = &aZ;
  if (aXDev <= aYDev && aXDev <= aZDev)
  {
    anAxis = &aX;
  }
  else if (aYDev <= aZDev)
  {
    anAxis = &aY;
  }
  return theVec.Crossed (*anAxis);
}

//! Fits a plane through the sampled points; returns false when they deviate
//! from one plane by more than the point tolerance. Collinear point sets are
//! planar in infinitely many ways; a stable arbitrary normal is chosen then.
bool FittedPlaneOf (const std::vector<gp_Pnt>& thePoints, const NvGeTol& theTol,
                    gp_Pnt& theOrigin, gp_Vec& theNormal)
{
  gp_Vec aFitted = FittedNormalOf (thePoints);
  if (aFitted.Magnitude() <= gp::Resolution())
  {
    const gp_Vec aChord (thePoints.front(), thePoints.back());
    aFitted = aChord.Magnitude() <= gp::Resolution()
      ? gp_Vec (0.0, 0.0, 1.0) : PerpendicularOf (aChord);
  }
  const gp_Vec aUnit = aFitted.Normalized();
  const gp_Pnt& aBase = thePoints.front();
  double aMaxDist = 0.0;
  for (const gp_Pnt& aPnt : thePoints)
  {
    aMaxDist = std::max (aMaxDist, std::abs (gp_Vec (aBase, aPnt).Dot (aUnit)));
  }
  if (aMaxDist > theTol.equalPoint())
  {
    return false;
  }
  theOrigin = aBase;
  theNormal = aUnit;
  return true;
}

//! Orthonormal in-plane axes for the plane with the given (non-degenerate)
//! normal.
void PlaneBasisOf (const gp_Vec& theNormal, gp_Vec& theU, gp_Vec& theV)
{
  const gp_Vec aPerp = PerpendicularOf (theNormal);
  theU = aPerp.Normalized();
  theV = theNormal.Crossed (theU).Normalized();
}

//! Returns the direction as gp_Dir; throws for a degenerate input vector.
gp_Dir NormalizedDirOf (const gp_Vec& theVec, const char* theMethod)
{
  if (theVec.Magnitude() <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCurve3d::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theVec);
}

//! Copy-on-write bookkeeping shared by every modifier: clones the impl when
//! its geometry handle is shared with other wrappers.
void MakeUnique (NvGeImpEntity3d*& theImpEnt)
{
  if (theImpEnt->RefCount() > 1)
  {
    theImpEnt->Unref();
    theImpEnt = theImpEnt->CloneShallow();
  }
}

//! Replaces the impl geometry under the copy-on-write discipline.
void SetCurve (NvGeImpEntity3d*& theImpEnt, const occ::handle<Geom_Curve>& theCurve)
{
  MakeUnique (theImpEnt);
  theImpEnt->SetGeom (theCurve);
}

//! Clamps the parameter window into the natural domain and wraps it as a
//! trimmed curve of the full geometry; throws when the window is empty.
occ::handle<Geom_TrimmedCurve> WindowedCurveOf (const occ::handle<Geom_Curve>& theCurve,
                                                double theFirst, double theLast,
                                                const char* theMethod)
{
  const double aDomFirst = NormalizedBound (theCurve->FirstParameter());
  const double aDomLast  = NormalizedBound (theCurve->LastParameter());
  double aFirst = WindowBoundOf (theFirst, aDomFirst);
  double aLast  = WindowBoundOf (theLast,  aDomLast);
  if (!std::isinf (aDomFirst) && aFirst < aDomFirst)
  {
    aFirst = aDomFirst;
  }
  if (!std::isinf (aDomLast) && aLast > aDomLast)
  {
    aLast = aDomLast;
  }
  if (aLast - aFirst <= Precision::Confusion())
  {
    throw NvException (std::string ("NvGeCurve3d::") + theMethod
                       + "(): the parameter window does not intersect the"
                       " curve domain");
  }
  try
  {
    return new Geom_TrimmedCurve (theCurve, aFirst, aLast);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure (theMethod, aFailure);
  }
}

//! Coarse closest-point search between two curves: uniformly samples this
//! curve and projects every sample onto the other one, keeping the best pair.
//! Parameters refer to the sampling windows of the respective curves; throws
//! when no sample projects.
void ClosestParamsBetween (const occ::handle<Geom_Curve>& theCurve,
                           const occ::handle<Geom_Curve>& theOther,
                           double& theParamOnCurve, double& theParamOnOther,
                           double& theDistance, const char* theMethod)
{
  std::vector<gp_Pnt> aPoints;
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theCurve, aFirst, aLast);
  SampleCurve (theCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, theMethod);

  bool aFound = false;
  double aBestDist = 0.0;
  double aBestParam = 0.0;
  double aBestOther = 0.0;
  try
  {
    for (int i = 0; i < int (aPoints.size()); ++i)
    {
      const GeomAPI_ProjectPointOnCurve aProjector (aPoints[i], theOther);
      if (aProjector.NbPoints() == 0)
      {
        continue;
      }
      const double aDist = aProjector.LowerDistance();
      if (!aFound || aDist < aBestDist)
      {
        aFound = true;
        aBestDist = aDist;
        aBestParam = aFirst + (aLast - aFirst) * double (i)
                     / double (int (aPoints.size()) - 1);
        aBestOther = aProjector.LowerDistanceParameter();
      }
    }
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure (theMethod, aFailure);
  }
  if (!aFound)
  {
    throw NvException (std::string ("NvGeCurve3d::") + theMethod
                       + "(): the point does not project onto the curve");
  }
  theParamOnCurve = aBestParam;
  theParamOnOther = aBestOther;
  theDistance = aBestDist;
}

//! Parameter of the curve point whose projection along theDir onto the plane
//! through thePnt normal to theDir comes closest to thePnt itself.
double ProjectedClosestParamOf (const occ::handle<Geom_Curve>& theCurve,
                                const gp_Pnt& thePnt, const gp_Vec& theDir,
                                const char* theMethod)
{
  std::vector<gp_Pnt> aPoints;
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theCurve, aFirst, aLast);
  SampleCurve (theCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, theMethod);

  bool aFound = false;
  double aBestDist = 0.0;
  double aBestParam = 0.0;
  for (int i = 0; i < int (aPoints.size()); ++i)
  {
    const gp_Vec aOffset (thePnt, aPoints[i]);
    const gp_Vec aDelta (aOffset.XYZ()
      - theDir.XYZ() * (aOffset.Dot (theDir) / theDir.SquareMagnitude()));
    const double aDist = aDelta.SquareMagnitude();
    if (!aFound || aDist < aBestDist)
    {
      aFound = true;
      aBestDist = aDist;
      aBestParam = aFirst + (aLast - aFirst) * double (i)
                   / double (int (aPoints.size()) - 1);
    }
  }
  if (!aFound)
  {
    throw NvException (std::string ("NvGeCurve3d::") + theMethod
                       + "(): the projection direction is degenerate");
  }
  return aBestParam;
}

//! Parameters of the closest pair between two curves when both are projected
//! along theDir onto a plane normal to it.
void ProjectedClosestParamsBetween (const occ::handle<Geom_Curve>& theCurve,
                                    const occ::handle<Geom_Curve>& theOther,
                                    const gp_Vec& theDir,
                                    double& theParamOnCurve,
                                    double& theParamOnOther,
                                    const char* theMethod)
{
  std::vector<gp_Pnt> aPoints;
  std::vector<gp_Pnt> anOtherPoints;
  double aFirst = 0.0;
  double aLast = 0.0;
  double anOtherFirst = 0.0;
  double anOtherLast = 0.0;
  SamplingWindowOf (theCurve, aFirst, aLast);
  SamplingWindowOf (theOther, anOtherFirst, anOtherLast);
  SampleCurve (theCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, theMethod);
  SampleCurve (theOther, anOtherFirst, anOtherLast, THE_NB_SAMPLES,
               anOtherPoints, theMethod);

  bool aFound = false;
  double aBestDist = 0.0;
  double aBestParam = 0.0;
  double aBestOther = 0.0;
  for (int i = 0; i < int (aPoints.size()); ++i)
  {
    for (int j = 0; j < int (anOtherPoints.size()); ++j)
    {
      const gp_Vec aOffset (aPoints[i], anOtherPoints[j]);
      const gp_Vec aDelta (aOffset.XYZ()
        - theDir.XYZ() * (aOffset.Dot (theDir) / theDir.SquareMagnitude()));
      const double aDist = aDelta.SquareMagnitude();
      if (!aFound || aDist < aBestDist)
      {
        aFound = true;
        aBestDist = aDist;
        aBestParam = aFirst + (aLast - aFirst) * double (i)
                     / double (int (aPoints.size()) - 1);
        aBestOther = anOtherFirst + (anOtherLast - anOtherFirst) * double (j)
                     / double (int (anOtherPoints.size()) - 1);
      }
    }
  }
  if (!aFound)
  {
    throw NvException (std::string ("NvGeCurve3d::") + theMethod
                       + "(): the projection direction is degenerate");
  }
  theParamOnCurve = aBestParam;
  theParamOnOther = aBestOther;
}

//! Degree-1 BSpline approximation of the curve projected onto the plane
//! through theOrigin with normal theNormal, along theDir. The projected
//! points lie in the plane, so the polyline is a planar 3d curve.
occ::handle<Geom_BSplineCurve> ProjectedPolylineOf (const occ::handle<Geom_Curve>& theCurve,
                                                    const gp_Pnt& theOrigin,
                                                    const gp_Vec& theNormal,
                                                    const gp_Vec& theDir,
                                                    const char* theMethod)
{
  std::vector<gp_Pnt> aPoints;
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theCurve, aFirst, aLast);
  SampleCurve (theCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, theMethod);

  const double aDirDotNormal = theDir.Dot (theNormal);
  std::vector<gp_Pnt> aProjPnts;
  aProjPnts.reserve (aPoints.size());
  for (const gp_Pnt& aPnt : aPoints)
  {
    const gp_Vec aHeight (theOrigin, aPnt);
    aProjPnts.push_back (gp_Pnt (aPnt.XYZ()
      - theDir.XYZ() * (aHeight.Dot (theNormal) / aDirDotNormal)));
  }

  // A degree-1 non-periodic BSpline over N poles needs N knots 0..N-1 with
  // multiplicity 2 at the ends and 1 inside (Sum mults = poles + degree + 1).
  const int aNb = int (aProjPnts.size());
  NCollection_Array1<gp_Pnt> aPoles (1, aNb);
  NCollection_Array1<double> aKnots (1, aNb);
  NCollection_Array1<int> aMults (1, aNb);
  for (int i = 0; i < aNb; ++i)
  {
    aPoles (i + 1) = aProjPnts[i];
    aKnots (i + 1) = double (i);
    aMults (i + 1) = (i == 0 || i == aNb - 1) ? 2 : 1;
  }
  try
  {
    return new Geom_BSplineCurve (aPoles, aKnots, aMults, 1, false);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure (theMethod, aFailure);
  }
}

} // namespace

//=================================================================================================
// Parametrization.
//
//=================================================================================================

void NvGeCurve3d::getInterval (NvGeInterval& theIntrvl) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aFirst = NormalizedBound (aCurve->FirstParameter());
  const double aLast  = NormalizedBound (aCurve->LastParameter());
  if (std::isinf (aFirst) && std::isinf (aLast))
  {
    theIntrvl.set();
  }
  else if (std::isinf (aFirst))
  {
    theIntrvl.set (false, aLast);
  }
  else if (std::isinf (aLast))
  {
    theIntrvl.set (true, aFirst);
  }
  else
  {
    theIntrvl.set (aFirst, aLast);
  }
}

//=================================================================================================

void NvGeCurve3d::getInterval (NvGeInterval& theIntrvl, NvGePoint3d& theStart,
                               NvGePoint3d& theEnd) const
{
  getInterval (theIntrvl);
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  try
  {
    gp_Pnt aPnt;
    aCurve->D0 (aCurve->FirstParameter(), aPnt);
    theStart = Pnt3dOf (aPnt);
    aCurve->D0 (aCurve->LastParameter(), aPnt);
    theEnd = Pnt3dOf (aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getInterval", aFailure);
  }
}

//=================================================================================================

NvGeCurve3d& NvGeCurve3d::reverseParam ()
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  occ::handle<Geom_Curve> aReversed;
  try
  {
    aReversed = aCurve->Reversed();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("reverseParam", aFailure);
  }
  SetCurve (mpImpEnt, aReversed);
  return *this;
}

//=================================================================================================

NvGeCurve3d& NvGeCurve3d::setInterval ()
{
  // Restoring the natural bounds simply drops the trimming wrapper.
  const occ::handle<Geom_TrimmedCurve> aTrimmed
    = occ::down_cast<Geom_TrimmedCurve> (NvGeCurve3dOf (mpImpEnt));
  if (!aTrimmed.IsNull())
  {
    SetCurve (mpImpEnt, aTrimmed->BasisCurve());
  }
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::setInterval (const NvGeInterval& theIntrvl)
{
  // Re-trim always from the basis curve so a later wider interval still works.
  const occ::handle<Geom_Curve> aBasis = BasisOf (NvGeCurve3dOf (mpImpEnt));
  const double aDomFirst = NormalizedBound (aBasis->FirstParameter());
  const double aDomLast  = NormalizedBound (aBasis->LastParameter());
  double aFirst = theIntrvl.lowerBound();
  double aLast  = theIntrvl.upperBound();
  // The requested window must lie within the natural bounds; infinite
  // requested bounds mean "up to the natural edge".
  if (!std::isinf (aFirst) && !std::isinf (aDomFirst)
      && aFirst < aDomFirst - theIntrvl.tolerance())
  {
    return Nova::kFalse;
  }
  if (!std::isinf (aLast) && !std::isinf (aDomLast)
      && aLast > aDomLast + theIntrvl.tolerance())
  {
    return Nova::kFalse;
  }
  aFirst = WindowBoundOf (aFirst, aDomFirst);
  aLast  = WindowBoundOf (aLast,  aDomLast);
  if (aLast - aFirst <= Precision::Confusion())
  {
    return Nova::kFalse;
  }
  occ::handle<Geom_Curve> aTrimmed;
  try
  {
    aTrimmed = new Geom_TrimmedCurve (aBasis, aFirst, aLast);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("setInterval", aFailure);
  }
  SetCurve (mpImpEnt, aTrimmed);
  return Nova::kTrue;
}

//=================================================================================================
// Distance to other geometric objects.
//
//=================================================================================================

double NvGeCurve3d::distanceTo (const NvGePoint3d& thePnt, const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  try
  {
    const GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePnt), aCurve);
    if (aProjector.NbPoints() == 0)
    {
      throw NvException ("NvGeCurve3d::distanceTo(): the point does not"
                         " project onto the curve");
    }
    return aProjector.LowerDistance();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("distanceTo", aFailure);
  }
}

//=================================================================================================

double NvGeCurve3d::distanceTo (const NvGeCurve3d& theCurve, const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  double aParam = 0.0;
  double anOtherParam = 0.0;
  double aDistance = 0.0;
  ClosestParamsBetween (aCurve, anOther, aParam, anOtherParam, aDistance,
                        "distanceTo");
  return aDistance;
}

//=================================================================================================
// Return the point on this object that is closest to the other object.
//
//=================================================================================================

NvGePoint3d NvGeCurve3d::closestPointTo (const NvGePoint3d& thePnt, const NvGeTol&) const
{
  return evalPoint (paramOf (thePnt));
}

//=================================================================================================

NvGePoint3d NvGeCurve3d::closestPointTo (const NvGeCurve3d& theCurve,
                                         NvGePoint3d& thePntOnOtherCrv,
                                         const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  double aParam = 0.0;
  double anOtherParam = 0.0;
  double aDistance = 0.0;
  ClosestParamsBetween (aCurve, anOther, aParam, anOtherParam, aDistance,
                        "closestPointTo");
  try
  {
    gp_Pnt aPnt;
    anOther->D0 (anOtherParam, aPnt);
    thePntOnOtherCrv = Pnt3dOf (aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("closestPointTo", aFailure);
  }
  return evalPoint (aParam);
}

//=================================================================================================

void NvGeCurve3d::getClosestPointTo (const NvGePoint3d& thePnt,
                                     NvGePointOnCurve3d& thePntOnCrv,
                                     const NvGeTol&) const
{
  const double aParam = paramOf (thePnt);
  thePntOnCrv.setCurve (*this);
  thePntOnCrv.setParameter (aParam);
}

//=================================================================================================

void NvGeCurve3d::getClosestPointTo (const NvGeCurve3d& theCurve,
                                     NvGePointOnCurve3d& thePntOnThisCrv,
                                     NvGePointOnCurve3d& thePntOnOtherCrv,
                                     const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  double aParam = 0.0;
  double anOtherParam = 0.0;
  double aDistance = 0.0;
  ClosestParamsBetween (aCurve, anOther, aParam, anOtherParam, aDistance,
                        "getClosestPointTo");
  thePntOnThisCrv.setCurve (*this);
  thePntOnThisCrv.setParameter (aParam);
  thePntOnOtherCrv.setCurve (theCurve);
  thePntOnOtherCrv.setParameter (anOtherParam);
}

//=================================================================================================
// Return closest points when projected in a given direction.
//
//=================================================================================================

NvGePoint3d NvGeCurve3d::projClosestPointTo (const NvGePoint3d& thePnt,
                                             const NvGeVector3d& theProjectDirection,
                                             const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const gp_Vec aDir (theProjectDirection.x, theProjectDirection.y,
                     theProjectDirection.z);
  NormalizedDirOf (aDir, "projClosestPointTo");
  return evalPoint (ProjectedClosestParamOf (aCurve, PntOf (thePnt), aDir,
                                             "projClosestPointTo"));
}

//=================================================================================================

NvGePoint3d NvGeCurve3d::projClosestPointTo (const NvGeCurve3d& theCurve,
                                             const NvGeVector3d& theProjectDirection,
                                             NvGePoint3d& thePntOnOtherCrv,
                                             const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  const gp_Vec aDir (theProjectDirection.x, theProjectDirection.y,
                     theProjectDirection.z);
  NormalizedDirOf (aDir, "projClosestPointTo");
  double aParam = 0.0;
  double anOtherParam = 0.0;
  ProjectedClosestParamsBetween (aCurve, anOther, aDir, aParam, anOtherParam,
                                 "projClosestPointTo");
  try
  {
    gp_Pnt aPnt;
    anOther->D0 (anOtherParam, aPnt);
    thePntOnOtherCrv = Pnt3dOf (aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("projClosestPointTo", aFailure);
  }
  return evalPoint (aParam);
}

//=================================================================================================

void NvGeCurve3d::getProjClosestPointTo (const NvGePoint3d& thePnt,
                                         const NvGeVector3d& theProjectDirection,
                                         NvGePointOnCurve3d& thePntOnCrv,
                                         const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const gp_Vec aDir (theProjectDirection.x, theProjectDirection.y,
                     theProjectDirection.z);
  NormalizedDirOf (aDir, "getProjClosestPointTo");
  const double aParam = ProjectedClosestParamOf (aCurve, PntOf (thePnt), aDir,
                                                 "getProjClosestPointTo");
  thePntOnCrv.setCurve (*this);
  thePntOnCrv.setParameter (aParam);
}

//=================================================================================================

void NvGeCurve3d::getProjClosestPointTo (const NvGeCurve3d& theCurve,
                                         const NvGeVector3d& theProjectDirection,
                                         NvGePointOnCurve3d& thePntOnThisCrv,
                                         NvGePointOnCurve3d& thePntOnOtherCrv,
                                         const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  const gp_Vec aDir (theProjectDirection.x, theProjectDirection.y,
                     theProjectDirection.z);
  NormalizedDirOf (aDir, "getProjClosestPointTo");
  double aParam = 0.0;
  double anOtherParam = 0.0;
  ProjectedClosestParamsBetween (aCurve, anOther, aDir, aParam, anOtherParam,
                                 "getProjClosestPointTo");
  thePntOnThisCrv.setCurve (*this);
  thePntOnThisCrv.setParameter (aParam);
  thePntOnOtherCrv.setCurve (theCurve);
  thePntOnOtherCrv.setParameter (anOtherParam);
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::getNormalPoint (const NvGePoint3d& thePnt,
                                            NvGePointOnCurve3d& thePntOnCrv,
                                            const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt aPnt;
  gp_Vec aTangent;
  try
  {
    const GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePnt), aCurve);
    if (aProjector.NbPoints() == 0)
    {
      return Nova::kFalse;
    }
    aParam = aProjector.LowerDistanceParameter();
    aCurve->D1 (aParam, aPnt, aTangent);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getNormalPoint", aFailure);
  }
  const double aTangentLen = aTangent.Magnitude();
  if (aTangentLen <= gp::Resolution())
  {
    return Nova::kFalse;
  }
  const gp_Vec aConnection (aPnt, PntOf (thePnt));
  // The point qualifies when the connection to thePnt is perpendicular to
  // the tangent, i.e. a normal at that point passes through thePnt.
  if (std::abs (aConnection.Dot (aTangent)) / aTangentLen > theTol.equalVector())
  {
    return Nova::kFalse;
  }
  thePntOnCrv.setCurve (*this);
  thePntOnCrv.setParameter (aParam);
  return Nova::kTrue;
}

//=================================================================================================
// Bounding blocks. The block built here is coordinate-parallel, so both the
// general and the orthogonal variants share one implementation; the resulting
// parallelepiped is spanned by the sampled extents along x, y and z.
//
//=================================================================================================

NvGeBoundBlock3d NvGeCurve3d::boundBlock () const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (aCurve, aFirst, aLast);
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, "boundBlock");
  return SampledBoundBlockOf (aPoints);
}

//=================================================================================================

NvGeBoundBlock3d NvGeCurve3d::boundBlock (const NvGeInterval& theRange) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aDomFirst = NormalizedBound (aCurve->FirstParameter());
  const double aDomLast  = NormalizedBound (aCurve->LastParameter());
  double aFirst = theRange.lowerBound();
  double aLast  = theRange.upperBound();
  // Clamp the requested range into the finite parts of the natural domain.
  if (!std::isinf (aDomFirst) && aFirst < aDomFirst)
  {
    aFirst = aDomFirst;
  }
  if (!std::isinf (aDomLast) && aLast > aDomLast)
  {
    aLast = aDomLast;
  }
  if (aLast <= aFirst + theRange.tolerance())
  {
    throw NvException ("NvGeCurve3d::boundBlock(): the given range misses"
                       " the curve domain");
  }
  // Resolve remaining infinite window bounds against the sampling window.
  if (std::isinf (aFirst) || std::isinf (aLast))
  {
    double aWinFirst = 0.0;
    double aWinLast = 0.0;
    SamplingWindowOf (aCurve, aWinFirst, aWinLast);
    if (std::isinf (aFirst))
    {
      aFirst = std::isinf (aLast) ? aWinFirst : aLast - 2.0;
    }
    if (std::isinf (aLast))
    {
      aLast = aFirst + 2.0;
    }
  }
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, "boundBlock");
  return SampledBoundBlockOf (aPoints);
}

//=================================================================================================

NvGeBoundBlock3d NvGeCurve3d::orthoBoundBlock () const
{
  return boundBlock();
}

//=================================================================================================

NvGeBoundBlock3d NvGeCurve3d::orthoBoundBlock (const NvGeInterval& theRange) const
{
  return boundBlock (theRange);
}

//=================================================================================================
// Project methods.
//
//=================================================================================================

NvGeEntity3d* NvGeCurve3d::project (const NvGePlane& theProjectionPlane,
                                    const NvGeVector3d& theProjectDirection,
                                    const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const NvGeVector3d aPlaneNormal = theProjectionPlane.normal();
  const gp_Vec aNormalVec (aPlaneNormal.x, aPlaneNormal.y, aPlaneNormal.z);
  const gp_Vec aDirVec (theProjectDirection.x, theProjectDirection.y,
                        theProjectDirection.z);
  NormalizedDirOf (aNormalVec, "project");
  NormalizedDirOf (aDirVec, "project");
  if (std::abs (aNormalVec.Normalized().Dot (aDirVec.Normalized()))
      <= Precision::Confusion())
  {
    throw NvException ("NvGeCurve3d::project(): the projection direction is"
                       " parallel to the projection plane");
  }
  const occ::handle<Geom_BSplineCurve> aPolyline = ProjectedPolylineOf (
    aCurve, PntOf (theProjectionPlane.pointOnPlane()), aNormalVec, aDirVec,
    "project");
  return newEntity3d (new NvGeImpEntity3d (NvGe::kNurbCurve3d,
    occ::handle<Standard_Transient> (aPolyline)));
}

//=================================================================================================

NvGeEntity3d* NvGeCurve3d::orthoProject (const NvGePlane& theProjectionPlane,
                                         const NvGeTol&) const
{
  return project (theProjectionPlane, theProjectionPlane.normal());
}

//=================================================================================================
// Tests if point is on curve.
//
//=================================================================================================

Nova::Boolean NvGeCurve3d::isOn (const NvGePoint3d& thePnt,
                                  const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  try
  {
    const GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePnt), aCurve);
    return aProjector.NbPoints() > 0
        && aProjector.LowerDistance() <= theTol.equalPoint()
      ? Nova::kTrue : Nova::kFalse;
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("isOn", aFailure);
  }
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isOn (const NvGePoint3d& thePnt, double& theParam,
                                  const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  try
  {
    const GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePnt), aCurve);
    if (aProjector.NbPoints() == 0
        || aProjector.LowerDistance() > theTol.equalPoint())
    {
      return Nova::kFalse;
    }
    theParam = aProjector.LowerDistanceParameter();
    return Nova::kTrue;
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("isOn", aFailure);
  }
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isOn (double theParam, const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aFirst = NormalizedBound (aCurve->FirstParameter());
  const double aLast  = NormalizedBound (aCurve->LastParameter());
  return theParam >= aFirst - theTol.equalPoint()
      && theParam <= aLast + theTol.equalPoint()
    ? Nova::kTrue : Nova::kFalse;
}

//=================================================================================================

double NvGeCurve3d::paramOf (const NvGePoint3d& thePnt, const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  try
  {
    const GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePnt), aCurve);
    if (aProjector.NbPoints() == 0)
    {
      throw NvException ("NvGeCurve3d::paramOf(): the point does not"
                         " project onto the curve");
    }
    return aProjector.LowerDistanceParameter();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("paramOf", aFailure);
  }
}

//=================================================================================================

void NvGeCurve3d::getTrimmedOffset (double theDistance,
                                    const NvGeVector3d& thePlaneNormal,
                                    NvGeVoidPointerArray& theOffsetCurveList,
                                    NvGe::OffsetCrvExtType,
                                    const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const gp_Vec aNormalVec (thePlaneNormal.x, thePlaneNormal.y, thePlaneNormal.z);
  const gp_Dir aNormalDir = NormalizedDirOf (aNormalVec, "getTrimmedOffset");
  occ::handle<Geom_Curve> anOffset;
  try
  {
    anOffset = new Geom_OffsetCurve (aCurve, theDistance, aNormalDir);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getTrimmedOffset", aFailure);
  }
  theOffsetCurveList.append ((NvGeCurve3d*) newEntity3d (
    new NvGeImpEntity3d (NvGe::kOffsetCurve3d,
                         occ::handle<Standard_Transient> (anOffset))));
}

//=================================================================================================
// Geometric inquiry methods.
//
//=================================================================================================

Nova::Boolean NvGeCurve3d::isClosed (const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aFirst = aCurve->FirstParameter();
  const double aLast  = aCurve->LastParameter();
  if (Precision::IsInfinite (aFirst) || Precision::IsInfinite (aLast))
  {
    return Nova::kFalse;
  }
  gp_Pnt aStartPnt;
  gp_Pnt anEndPnt;
  try
  {
    aCurve->D0 (aFirst, aStartPnt);
    aCurve->D0 (aLast, anEndPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("isClosed", aFailure);
  }
  return aStartPnt.Distance (anEndPnt) <= theTol.equalPoint()
    ? Nova::kTrue : Nova::kFalse;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isPlanar (NvGePlane& thePlane, const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (aCurve, aFirst, aLast);
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, "isPlanar");
  gp_Pnt anOrigin;
  gp_Vec aNormal;
  if (!FittedPlaneOf (aPoints, theTol, anOrigin, aNormal))
  {
    return Nova::kFalse;
  }
  thePlane.set (Pnt3dOf (anOrigin),
                NvGeVector3d (aNormal.X(), aNormal.Y(), aNormal.Z()));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isLinear (NvGeLine3d& theLine, const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  // Fast path: the stored geometry is already (the basis of) a line.
  const occ::handle<Geom_Line> aLineGeom
    = occ::down_cast<Geom_Line> (BasisOf (aCurve));
  if (!aLineGeom.IsNull())
  {
    const gp_Ax1& anAxis = aLineGeom->Position();
    const gp_Dir& aDir = anAxis.Direction();
    theLine.set (Pnt3dOf (anAxis.Location()),
                 NvGeVector3d (aDir.X(), aDir.Y(), aDir.Z()));
    return Nova::kTrue;
  }
  // General case: every sample must deviate from the chord by at most the
  // point tolerance.
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (aCurve, aFirst, aLast);
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, "isLinear");
  const gp_Vec aChord (aPoints.front(), aPoints.back());
  if (aChord.Magnitude() <= gp::Resolution())
  {
    return Nova::kFalse;
  }
  const gp_Vec aUnitChord = aChord.Normalized();
  for (const gp_Pnt& aPnt : aPoints)
  {
    if (gp_Vec (aPoints.front(), aPnt).Crossed (aUnitChord).Magnitude()
        > theTol.equalPoint())
    {
      return Nova::kFalse;
    }
  }
  theLine.set (Pnt3dOf (aPoints.front()),
               NvGeVector3d (aUnitChord.X(), aUnitChord.Y(), aUnitChord.Z()));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isCoplanarWith (const NvGeCurve3d& theCurve,
                                            NvGePlane& thePlane,
                                            const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theCurve.mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  double anOtherFirst = 0.0;
  double anOtherLast = 0.0;
  SamplingWindowOf (aCurve, aFirst, aLast);
  SamplingWindowOf (anOther, anOtherFirst, anOtherLast);
  std::vector<gp_Pnt> aPoints;
  std::vector<gp_Pnt> anOtherPoints;
  SampleCurve (aCurve, aFirst, aLast, THE_NB_SAMPLES, aPoints, "isCoplanarWith");
  SampleCurve (anOther, anOtherFirst, anOtherLast, THE_NB_SAMPLES,
               anOtherPoints, "isCoplanarWith");
  std::vector<gp_Pnt> aCombined (aPoints);
  aCombined.insert (aCombined.end(), anOtherPoints.begin(), anOtherPoints.end());
  gp_Pnt anOrigin;
  gp_Vec aNormal;
  if (!FittedPlaneOf (aCombined, theTol, anOrigin, aNormal))
  {
    return Nova::kFalse;
  }
  thePlane.set (Pnt3dOf (anOrigin),
                NvGeVector3d (aNormal.X(), aNormal.Y(), aNormal.Z()));
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isPeriodic (double& thePeriod) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  // Circle and ellipse are periodic with a two pi parameter period.
  if (!occ::down_cast<Geom_Conic> (aCurve).IsNull())
  {
    thePeriod = THE_TWO_PI;
    return Nova::kTrue;
  }
  if (aCurve->IsPeriodic())
  {
    thePeriod = aCurve->Period();
    return Nova::kTrue;
  }
  return Nova::kFalse;
}

//=================================================================================================
// Length based methods.
//
//=================================================================================================

double NvGeCurve3d::length (double theFromParam, double theToParam, double theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aFrom = theFromParam;
  double aTo = theToParam;
  if (aFrom > aTo)
  {
    std::swap (aFrom, aTo);
  }
  // Clamp into the finite parts of the natural domain.
  const double aDomFirst = NormalizedBound (aCurve->FirstParameter());
  const double aDomLast  = NormalizedBound (aCurve->LastParameter());
  if (!std::isinf (aDomFirst) && aFrom < aDomFirst)
  {
    aFrom = aDomFirst;
  }
  if (!std::isinf (aDomLast) && aTo > aDomLast)
  {
    aTo = aDomLast;
  }
  // An unbounded domain yields an unbounded length.
  if (Precision::IsInfinite (aFrom) || Precision::IsInfinite (aTo))
  {
    return std::numeric_limits<double>::infinity();
  }
  if (aTo - aFrom <= Precision::Confusion())
  {
    return 0.0;
  }
  try
  {
    const GeomAdaptor_Curve anAdaptor (aCurve);
    return GCPnts_AbscissaPoint::Length (anAdaptor, aFrom, aTo, theTol);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("length", aFailure);
  }
}

//=================================================================================================

double NvGeCurve3d::paramAtLength (double theDatumParam, double theLength,
                                   Nova::Boolean thePosParamDir, double theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  if (theLength < 0.0)
  {
    throw NvException ("NvGeCurve3d::paramAtLength(): the arc length must"
                       " not be negative");
  }
  const double aDomFirst = NormalizedBound (aCurve->FirstParameter());
  const double aDomLast  = NormalizedBound (aCurve->LastParameter());
  double aDatum = theDatumParam;
  // Clamp the datum into the finite parts of the natural domain.
  if (!std::isinf (aDomFirst) && aDatum < aDomFirst)
  {
    aDatum = aDomFirst;
  }
  if (!std::isinf (aDomLast) && aDatum > aDomLast)
  {
    aDatum = aDomLast;
  }
  // Solve over a finite window: the domain itself when bounded, otherwise a
  // window around the datum that surely contains the requested arc length.
  const double aFirst = std::isinf (aDomFirst)
    ? aDatum - (theLength + 1.0 + theTol) : aDomFirst;
  const double aLast  = std::isinf (aDomLast)
    ? aDatum + (theLength + 1.0 + theTol) : aDomLast;
  try
  {
    const GeomAdaptor_Curve anAdaptor (aCurve, aFirst, aLast);
    if (!std::isinf (aFirst) && !std::isinf (aLast))
    {
      // GCPnts_AbscissaPoint solves blindly for length-parametrized carriers
      // (line, circle): it reports IsDone() and lands past the window end
      // when the requested arc length exceeds the length available from the
      // datum. Reject such requests up front.
      const double anEdge = thePosParamDir ? aLast : aFirst;
      const double anAvail = GCPnts_AbscissaPoint::Length (
        anAdaptor, std::min (aDatum, anEdge), std::max (aDatum, anEdge), theTol);
      if (theLength > anAvail + std::max (theTol, Precision::Confusion()))
      {
        throw NvException ("NvGeCurve3d::paramAtLength(): the requested arc"
                           " length exceeds the curve length from the datum");
      }
    }
    const GCPnts_AbscissaPoint aSolver (theTol, anAdaptor,
      thePosParamDir ? theLength : -theLength, aDatum);
    if (!aSolver.IsDone())
    {
      throw NvException ("NvGeCurve3d::paramAtLength(): the requested arc"
                         " length exceeds the curve length from the datum");
    }
    return aSolver.Parameter();
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("paramAtLength", aFailure);
  }
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::area (double theStartParam, double theEndParam,
                                  double& theValue, const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  if (std::isinf (theStartParam) || std::isinf (theEndParam)
      || theEndParam <= theStartParam)
  {
    return Nova::kFalse;
  }
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, theStartParam, theEndParam, THE_NB_AREA_SAMPLES, aPoints,
               "area");
  // Shoelace sum in the plane fitted through the samples; an open segment
  // closes with its chord, exactly the region this method reports.
  const gp_Vec aFitted = FittedNormalOf (aPoints);
  if (aFitted.Magnitude() <= gp::Resolution())
  {
    theValue = 0.0;
    return Nova::kTrue;
  }
  gp_Vec anU;
  gp_Vec aV;
  PlaneBasisOf (aFitted, anU, aV);
  const gp_Pnt& aBase = aPoints.front();
  const int aNb = int (aPoints.size());
  double aSum = 0.0;
  for (int i = 0; i < aNb; ++i)
  {
    const gp_Vec aCur (aBase, aPoints[i]);
    const gp_Vec aNxt (aBase, aPoints[(i + 1) % aNb]);
    aSum += aCur.Dot (anU) * aNxt.Dot (aV) - aNxt.Dot (anU) * aCur.Dot (aV);
  }
  theValue = std::abs (aSum) / 2.0;
  return Nova::kTrue;
}

//=================================================================================================
// Degeneracy.
//
//=================================================================================================

Nova::Boolean NvGeCurve3d::isDegenerate (NvGe::EntityId& theDegenerateType,
                                          const NvGeTol& theTol) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aTotal = length (aCurve->FirstParameter(),
                                aCurve->LastParameter(), theTol.equalPoint());
  if (!std::isinf (aTotal) && aTotal <= theTol.equalPoint())
  {
    theDegenerateType = NvGe::kPosition3d;
    return Nova::kTrue;
  }
  theDegenerateType = NvGe::kCurve3d;
  return Nova::kFalse;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::isDegenerate (NvGeEntity3d*& theConvertedEntity,
                                          const NvGeTol& theTol) const
{
  NvGe::EntityId aType = NvGe::kEntity3d;
  if (!isDegenerate (aType, theTol))
  {
    return Nova::kFalse;
  }
  // A zero-length curve degenerates into a point at its start.
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  gp_Pnt aPnt;
  try
  {
    aCurve->D0 (aCurve->FirstParameter(), aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("isDegenerate", aFailure);
  }
  occ::handle<NvGePosition3dData> aData = new NvGePosition3dData();
  aData->Point = aPnt;
  theConvertedEntity = newEntity3d (new NvGeImpEntity3d (NvGe::kPosition3d,
    occ::handle<Standard_Transient> (aData)));
  return Nova::kTrue;
}

//=================================================================================================
// Modify methods.
//
//=================================================================================================

void NvGeCurve3d::getSplitCurves (double theParam, NvGeCurve3d*& thePiece1,
                                  NvGeCurve3d*& thePiece2) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aRawFirst = aCurve->FirstParameter();
  const double aRawLast  = aCurve->LastParameter();
  if (!(NormalizedBound (aRawFirst) < theParam
        && theParam < NormalizedBound (aRawLast)))
  {
    throw NvException ("NvGeCurve3d::getSplitCurves(): the split parameter"
                       " must lie strictly inside the curve domain");
  }
  occ::handle<Geom_Curve> aLow;
  occ::handle<Geom_Curve> aHigh;
  try
  {
    // Trim the current curve itself so the pieces keep the current bounds.
    aLow  = new Geom_TrimmedCurve (aCurve, aRawFirst, theParam);
    aHigh = new Geom_TrimmedCurve (aCurve, theParam, aRawLast);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getSplitCurves", aFailure);
  }
  thePiece1 = (NvGeCurve3d*) newEntity3d (new NvGeImpEntity3d (mpImpEnt->Type(),
    occ::handle<Standard_Transient> (aLow)));
  thePiece2 = (NvGeCurve3d*) newEntity3d (new NvGeImpEntity3d (mpImpEnt->Type(),
    occ::handle<Standard_Transient> (aHigh)));
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::explode (NvGeVoidPointerArray&, NvGeIntArray&,
                                     const NvGeInterval*) const
{
  // A generic curve is a single component; composite curve classes that do
  // have sub-curves override this.
  return Nova::kFalse;
}

//=================================================================================================
// Local closest points.
//
//=================================================================================================

void NvGeCurve3d::getLocalClosestPoints (const NvGePoint3d& thePoint,
                                         NvGePointOnCurve3d& theApproxPnt,
                                         const NvGeInterval* theNbhd,
                                         const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  // Restrict the search to the neighborhood when one is supplied.
  occ::handle<Geom_Curve> aWindow = aCurve;
  if (theNbhd != nullptr)
  {
    aWindow = WindowedCurveOf (aCurve, theNbhd->lowerBound(),
                               theNbhd->upperBound(), "getLocalClosestPoints");
  }
  double aParam = 0.0;
  try
  {
    bool aFound = false;
    GeomAPI_ProjectPointOnCurve aProjector (PntOf (thePoint), aWindow);
    if (aProjector.NbPoints() > 0)
    {
      aParam = aProjector.LowerDistanceParameter();
      aFound = true;
    }
    else if (theNbhd != nullptr)
    {
      // The projector comes up empty on a window trimmed from an unbounded
      // curve (e.g. a line) whenever the perpendicular foot of the point
      // lies outside it; the closest point inside the neighborhood is then
      // the window boundary nearest to the projection onto the full
      // geometry.
      GeomAPI_ProjectPointOnCurve aFullProjector (PntOf (thePoint), aCurve);
      if (aFullProjector.NbPoints() > 0)
      {
        aParam = std::min (std::max (aFullProjector.LowerDistanceParameter(),
                                     theNbhd->lowerBound()),
                           theNbhd->upperBound());
        aFound = true;
      }
    }
    if (!aFound)
    {
      throw NvException ("NvGeCurve3d::getLocalClosestPoints(): the point"
                         " does not project onto the curve");
    }
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getLocalClosestPoints", aFailure);
  }
  theApproxPnt.setCurve (*this);
  theApproxPnt.setParameter (aParam);
}

//=================================================================================================

void NvGeCurve3d::getLocalClosestPoints (const NvGeCurve3d& theOtherCurve,
                                         NvGePointOnCurve3d& theApproxPntOnThisCrv,
                                         NvGePointOnCurve3d& theApproxPntOnOtherCrv,
                                         const NvGeInterval* theNbhd1,
                                         const NvGeInterval* theNbhd2,
                                         const NvGeTol&) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const occ::handle<Geom_Curve> anOther = NvGeCurve3dOf (theOtherCurve.mpImpEnt);
  occ::handle<Geom_Curve> aWindow = aCurve;
  occ::handle<Geom_Curve> anOtherWindow = anOther;
  if (theNbhd1 != nullptr)
  {
    aWindow = WindowedCurveOf (aCurve, theNbhd1->lowerBound(),
                               theNbhd1->upperBound(), "getLocalClosestPoints");
  }
  if (theNbhd2 != nullptr)
  {
    anOtherWindow = WindowedCurveOf (anOther, theNbhd2->lowerBound(),
                                     theNbhd2->upperBound(),
                                     "getLocalClosestPoints");
  }
  double aParam = 0.0;
  double anOtherParam = 0.0;
  double aDistance = 0.0;
  ClosestParamsBetween (aWindow, anOtherWindow, aParam, anOtherParam,
                        aDistance, "getLocalClosestPoints");
  theApproxPntOnThisCrv.setCurve (*this);
  theApproxPntOnThisCrv.setParameter (aParam);
  theApproxPntOnOtherCrv.setCurve (theOtherCurve);
  theApproxPntOnOtherCrv.setParameter (anOtherParam);
}

//=================================================================================================
// Return start and end points.
//
//=================================================================================================

Nova::Boolean NvGeCurve3d::hasStartPoint (NvGePoint3d& theStartPnt) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aFirst = aCurve->FirstParameter();
  if (Precision::IsInfinite (aFirst))
  {
    return Nova::kFalse;
  }
  gp_Pnt aPnt;
  try
  {
    aCurve->D0 (aFirst, aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("hasStartPoint", aFailure);
  }
  theStartPnt = Pnt3dOf (aPnt);
  return Nova::kTrue;
}

//=================================================================================================

Nova::Boolean NvGeCurve3d::hasEndPoint (NvGePoint3d& theEndPnt) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  const double aLast = aCurve->LastParameter();
  if (Precision::IsInfinite (aLast))
  {
    return Nova::kFalse;
  }
  gp_Pnt aPnt;
  try
  {
    aCurve->D0 (aLast, aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("hasEndPoint", aFailure);
  }
  theEndPnt = Pnt3dOf (aPnt);
  return Nova::kTrue;
}

//=================================================================================================
// Evaluate methods.
//
//=================================================================================================

NvGePoint3d NvGeCurve3d::evalPoint (double theParam) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  gp_Pnt aPnt;
  try
  {
    aCurve->D0 (theParam, aPnt);
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("evalPoint", aFailure);
  }
  return Pnt3dOf (aPnt);
}

//=================================================================================================

NvGePoint3d NvGeCurve3d::evalPoint (double theParam, int theNumDeriv,
                                    NvGeVector3dArray& theDerivArray) const
{
  if (theNumDeriv < 0 || theNumDeriv > 3)
  {
    throw NvException ("NvGeCurve3d::evalPoint(): the derivative order must"
                       " be between 0 and 3");
  }
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  gp_Pnt aPnt;
  gp_Vec aD1;
  gp_Vec aD2;
  gp_Vec aD3;
  try
  {
    switch (theNumDeriv)
    {
      case 0:
        aCurve->D0 (theParam, aPnt);
        break;
      case 1:
        aCurve->D1 (theParam, aPnt, aD1);
        break;
      case 2:
        aCurve->D2 (theParam, aPnt, aD1, aD2);
        break;
      default:
        aCurve->D3 (theParam, aPnt, aD1, aD2, aD3);
        break;
    }
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("evalPoint", aFailure);
  }
  // Derivative i lands at array index i - 1.
  theDerivArray.setLogicalLength (theNumDeriv);
  if (theNumDeriv >= 1)
  {
    theDerivArray[0] = NvGeVector3d (aD1.X(), aD1.Y(), aD1.Z());
  }
  if (theNumDeriv >= 2)
  {
    theDerivArray[1] = NvGeVector3d (aD2.X(), aD2.Y(), aD2.Z());
  }
  if (theNumDeriv >= 3)
  {
    theDerivArray[2] = NvGeVector3d (aD3.X(), aD3.Y(), aD3.Z());
  }
  return Pnt3dOf (aPnt);
}

//=================================================================================================
// Polygonize curve to within a specified tolerance.
//
//=================================================================================================

void NvGeCurve3d::getSamplePoints (double theFromParam, double theToParam,
                                   double theApproxEps,
                                   NvGePoint3dArray& thePointArray,
                                   NvGeDoubleArray& theParamArray, bool) const
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aFirst = theFromParam;
  double aLast = theToParam;
  if (aFirst > aLast)
  {
    std::swap (aFirst, aLast);
  }
  // Clamp into the finite parts of the natural domain.
  const double aDomFirst = NormalizedBound (aCurve->FirstParameter());
  const double aDomLast  = NormalizedBound (aCurve->LastParameter());
  if (!std::isinf (aDomFirst) && aFirst < aDomFirst)
  {
    aFirst = aDomFirst;
  }
  if (!std::isinf (aDomLast) && aLast > aDomLast)
  {
    aLast = aDomLast;
  }
  // Resolve remaining infinite window bounds against the sampling window.
  if (std::isinf (aFirst) || std::isinf (aLast))
  {
    double aWinFirst = 0.0;
    double aWinLast = 0.0;
    SamplingWindowOf (aCurve, aWinFirst, aWinLast);
    if (std::isinf (aFirst))
    {
      aFirst = std::isinf (aLast) ? aWinFirst : aLast - 2.0;
    }
    if (std::isinf (aLast))
    {
      aLast = aFirst + 2.0;
    }
  }
  try
  {
    const GeomAdaptor_Curve anAdaptor (aCurve, aFirst, aLast);
    const GCPnts_QuasiUniformDeflection aDistrib (
      anAdaptor, std::max (theApproxEps, Precision::Confusion()));
    if (!aDistrib.IsDone())
    {
      throw NvException ("NvGeCurve3d::getSamplePoints(): the deflection"
                         " sampler did not converge");
    }
    const int aNb = aDistrib.NbPoints();
    thePointArray.setLogicalLength (aNb);
    theParamArray.setLogicalLength (aNb);
    for (int i = 0; i < aNb; ++i)
    {
      thePointArray[i] = Pnt3dOf (aDistrib.Value (i + 1));
      theParamArray[i] = aDistrib.Parameter (i + 1);
    }
  }
  catch (const Standard_Failure& aFailure)
  {
    throw TranslatedFailure ("getSamplePoints", aFailure);
  }
}

//=================================================================================================

void NvGeCurve3d::getSamplePoints (int theNumSample,
                                   NvGePoint3dArray& thePointArray) const
{
  NvGeDoubleArray aParams;
  getSamplePoints (theNumSample, thePointArray, aParams);
}

//=================================================================================================

void NvGeCurve3d::getSamplePoints (int theNumSample,
                                   NvGePoint3dArray& thePointArray,
                                   NvGeDoubleArray& theParamArray) const
{
  if (theNumSample < 2)
  {
    throw NvException ("NvGeCurve3d::getSamplePoints(): at least two sample"
                       " points are required");
  }
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (aCurve, aFirst, aLast);
  std::vector<gp_Pnt> aPoints;
  SampleCurve (aCurve, aFirst, aLast, theNumSample, aPoints, "getSamplePoints");
  const double aStep = (aLast - aFirst) / double (theNumSample - 1);
  thePointArray.setLogicalLength (theNumSample);
  theParamArray.setLogicalLength (theNumSample);
  for (int i = 0; i < theNumSample; ++i)
  {
    thePointArray[i] = Pnt3dOf (aPoints[i]);
    theParamArray[i] = aFirst + i * aStep;
  }
}

//=================================================================================================
// Assignment operator and constructors.
//
//=================================================================================================

NvGeCurve3d& NvGeCurve3d::operator = (const NvGeCurve3d& theCurve)
{
  NvGeEntity3d::operator = (theCurve);
  return *this;
}

//=================================================================================================

NvGeCurve3d::NvGeCurve3d ()
: NvGeEntity3d ()
{
}

//=================================================================================================

NvGeCurve3d::NvGeCurve3d (const NvGeCurve3d& theSrc)
: NvGeEntity3d (theSrc)
{
}
