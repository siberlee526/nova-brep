// gecurv2d.cpp - implementation of NvGeCurve2d, the generic 2d curve base.
//
// Every NvGeCurve2d instance carries an occ::handle<Geom2d_Curve> in its
// impl; bounded arcs and segments are stored as Geom2d_TrimmedCurve over an
// analytic basis, full conics and free lines directly. The algorithms below
// therefore work generically on any Geom2d_Curve: parametric bounds are
// normalized to plain doubles (OCCT infinite markers become +/- infinity),
// restricted ranges are handled through Geom2d_TrimmedCurve windows, and
// lengths / projections go through the GCPnts and Geom2dAPI algorithms.
// Modifiers follow the copy-on-write discipline: the impl is detached when
// shared and the OCCT geometry itself is never mutated.

#include <gecurv2d.h>

#include <Nova.h>
#include <NvException.h>
#include <geblok2d.h>
#include <gedblar.h>
#include <geent2d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geintarr.h>
#include <geintrvl.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <geponc2d.h>
#include <gept2dar.h>
#include <getol.h>
#include <gevc2dar.h>
#include <gevptar.h>

#include <geimpdata.h>

#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace
{

// Number of samples used by the sampling-based fallback algorithms.
constexpr int THE_NB_SAMPLES = 33;

// Number of samples used by the area quadrature.
constexpr int THE_NB_AREA_SAMPLES = 129;

// Maps an OCCT parametric bound to a plain double: the OCCT infinite
// markers (+/-2e100) become +/- infinity so that bounds compare and
// combine naturally with std::isfinite.
double PlainParam (const double theParam)
{
  if (Precision::IsPositiveInfinite (theParam))
  {
    return std::numeric_limits<double>::infinity();
  }
  if (Precision::IsNegativeInfinite (theParam))
  {
    return -std::numeric_limits<double>::infinity();
  }
  return theParam;
}

// Inverse of PlainParam: maps a plain infinite bound back to the OCCT
// infinite marker expected by the Geom2d constructors.
double OcctParam (const double theParam)
{
  if (std::isinf (theParam))
  {
    return theParam > 0.0 ? Precision::Infinite() : -Precision::Infinite();
  }
  return theParam;
}

// Unwraps trimmed curves down to their basis geometry.
occ::handle<Geom2d_Curve> BasisOf (const occ::handle<Geom2d_Curve>& theCurve)
{
  occ::handle<Geom2d_Curve> aBasis = theCurve;
  for (;;)
  {
    const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (aBasis);
    if (aTrimmed.IsNull())
    {
      break;
    }
    aBasis = aTrimmed->BasisCurve();
  }
  return aBasis;
}

// Natural parameter bounds of the basis curve as plain doubles.
void NaturalBoundsOf (const occ::handle<Geom2d_Curve>& theCurve, double& theFirst, double& theLast)
{
  const occ::handle<Geom2d_Curve> aBasis = BasisOf (theCurve);
  theFirst = PlainParam (aBasis->FirstParameter());
  theLast = PlainParam (aBasis->LastParameter());
}

// Resolves a sampling window from a requested range: each side takes the
// range bound when it is finite and the natural bound otherwise; a side
// that is unbounded on both accounts gets a local window around the finite
// midpoint (or the origin) so that sampling-based algorithms terminate.
void SamplingWindowOf (const NvGeInterval& theRange, double theNatFirst, double theNatLast,
                       double& theFirst, double& theLast)
{
  theFirst = theRange.isBoundedBelow() ? theRange.lowerBound() : theNatFirst;
  theLast = theRange.isBoundedAbove() ? theRange.upperBound() : theNatLast;
  if (theFirst > theLast)
  {
    std::swap (theFirst, theLast);
  }
  const double aMid = std::isfinite (theFirst + theLast) ? 0.5 * (theFirst + theLast) : 0.0;
  if (!std::isfinite (theFirst))
  {
    theFirst = aMid - 1.0;
  }
  if (!std::isfinite (theLast))
  {
    theLast = aMid + 1.0;
  }
}

// Sampling window of a curve over its own parameter range.
void WindowOfCurve (const occ::handle<Geom2d_Curve>& theCurve, double& theFirst, double& theLast)
{
  SamplingWindowOf (NvGeInterval(), PlainParam (theCurve->FirstParameter()),
                    PlainParam (theCurve->LastParameter()), theFirst, theLast);
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

NvGePoint2d ToPoint2d (const gp_Pnt2d& thePnt)
{
  return NvGePoint2d (thePnt.X(), thePnt.Y());
}

// Projects a point onto a curve restricted to [theFirst, theLast]; returns
// false when the projection found no solution.
bool ProjectPointOnto (const gp_Pnt2d& thePnt, const occ::handle<Geom2d_Curve>& theCurve,
                       double theFirst, double theLast,
                       double& theParam, gp_Pnt2d& thePointOnCurve, double& theDistance)
{
  try
  {
    const Geom2dAPI_ProjectPointOnCurve aProjector (thePnt, theCurve,
                                                    OcctParam (theFirst), OcctParam (theLast));
    if (aProjector.NbPoints() <= 0)
    {
      // No projection solution exists when the point is equidistant from
      // every curve point (e.g. the center of a circular window); the
      // closer window endpoint is then the best representable answer.
      const gp_Pnt2d aStart = theCurve->Value (OcctParam (theFirst));
      const gp_Pnt2d anEnd = theCurve->Value (OcctParam (theLast));
      const double aStartDist = aStart.SquareDistance (thePnt);
      const double anEndDist = anEnd.SquareDistance (thePnt);
      thePointOnCurve = aStartDist <= anEndDist ? aStart : anEnd;
      theParam = aStartDist <= anEndDist ? OcctParam (theFirst) : OcctParam (theLast);
      theDistance = std::sqrt (std::min (aStartDist, anEndDist));
      return true;
    }
    theParam = aProjector.LowerDistanceParameter();
    thePointOnCurve = aProjector.NearestPoint();
    theDistance = aProjector.LowerDistance();
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  return true;
}

// Projects onto the curve's own sampling window; throws when the projection
// finds no solution.
void ProjectOrThrow (const occ::handle<Geom2d_Curve>& theCurve, const gp_Pnt2d& thePnt,
                     const char* theMethod,
                     double& theParam, gp_Pnt2d& thePointOnCurve, double& theDistance)
{
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (theCurve, aFirst, aLast);
  if (!ProjectPointOnto (thePnt, theCurve, aFirst, aLast, theParam, thePointOnCurve, theDistance))
  {
    throw NvException (std::string ("NvGeCurve2d::") + theMethod
                       + "(): the projection onto the curve failed");
  }
}

// Closest points between two curves restricted to their windows: samples
// of either curve are projected onto the other and the best mutual pair
// is kept. Returns the best distance; infinity when all projections fail.
double CurveCurveClosestOf (const occ::handle<Geom2d_Curve>& theCurve1, double theFirst1, double theLast1,
                            const occ::handle<Geom2d_Curve>& theCurve2, double theFirst2, double theLast2,
                            double& theParam1, gp_Pnt2d& thePoint1,
                            double& theParam2, gp_Pnt2d& thePoint2)
{
  double aBestDist = std::numeric_limits<double>::infinity();
  for (int aDir = 0; aDir < 2; ++aDir)
  {
    const occ::handle<Geom2d_Curve>& aSource = aDir == 0 ? theCurve2 : theCurve1;
    const double aSrcFirst = aDir == 0 ? theFirst2 : theFirst1;
    const double aSrcLast = aDir == 0 ? theLast2 : theLast1;
    const occ::handle<Geom2d_Curve>& aTarget = aDir == 0 ? theCurve1 : theCurve2;
    const double aTgtFirst = aDir == 0 ? theFirst1 : theFirst2;
    const double aTgtLast = aDir == 0 ? theLast1 : theLast2;
    for (int aSample = 0; aSample < THE_NB_SAMPLES; ++aSample)
    {
      const double aSrcParam = aSrcFirst + (aSrcLast - aSrcFirst) * aSample / (THE_NB_SAMPLES - 1);
      double aTgtParam = 0.0;
      gp_Pnt2d aSrcPnt;
      gp_Pnt2d aTgtPnt;
      double aDist = 0.0;
      if (!ProjectPointOnto (aSource->Value (aSrcParam), aTarget, aTgtFirst, aTgtLast,
                             aTgtParam, aTgtPnt, aDist)
          || aDist >= aBestDist)
      {
        continue;
      }
      aBestDist = aDist;
      aSrcPnt = aSource->Value (aSrcParam);
      if (aDir == 0)
      {
        theParam1 = aTgtParam;
        thePoint1 = aTgtPnt;
        theParam2 = aSrcParam;
        thePoint2 = aSrcPnt;
      }
      else
      {
        theParam2 = aTgtParam;
        thePoint2 = aTgtPnt;
        theParam1 = aSrcParam;
        thePoint1 = aSrcPnt;
      }
    }
  }
  return aBestDist;
}

// Residual (C - Target) . T of the normal-foot condition at a parameter.
double NormalResidualOf (const occ::handle<Geom2d_Curve>& theCurve, const gp_Pnt2d& theTarget,
                         double theParam)
{
  gp_Pnt2d aPnt;
  gp_Vec2d aTangent;
  theCurve->D1 (theParam, aPnt, aTangent);
  return aTangent.Dot (gp_Vec2d (aPnt, theTarget));
}

// Newton iteration for the foot of the normal from theTarget onto theCurve
// starting at theParam.
void RefineNormalFoot (const occ::handle<Geom2d_Curve>& theCurve, double theFirst, double theLast,
                       const gp_Pnt2d& theTarget, double& theParam)
{
  for (int anIter = 0; anIter < 16; ++anIter)
  {
    gp_Pnt2d aPnt;
    gp_Vec2d aTangent;
    gp_Vec2d aSecond;
    theCurve->D2 (theParam, aPnt, aTangent, aSecond);
    const gp_Vec2d aRadial (aPnt, theTarget);
    // Derivative of the residual T . (Target - C) with respect to the
    // parameter: C" . (Target - C) - |T|^2.
    const double aDenom = aSecond.Dot (aRadial) - aTangent.Dot (aTangent);
    if (std::abs (aDenom) <= Precision::Confusion())
    {
      break;
    }
    double aStep = aTangent.Dot (aRadial) / aDenom;
    aStep = std::min (std::max (aStep, -1.0), 1.0);
    theParam -= aStep;
    theParam = std::min (std::max (theParam, theFirst), theLast);
  }
}

// Min/max coordinate extents of a curve sampled over its window.
void SampledExtentsOf (const occ::handle<Geom2d_Curve>& theCurve, double theFirst, double theLast,
                       gp_Pnt2d& theMinPnt, gp_Pnt2d& theMaxPnt)
{
  gp_Pnt2d aPnt = theCurve->Value (theFirst);
  theMinPnt = aPnt;
  theMaxPnt = aPnt;
  for (int aSample = 1; aSample < THE_NB_SAMPLES; ++aSample)
  {
    const double aParam = theFirst + (theLast - theFirst) * aSample / (THE_NB_SAMPLES - 1);
    aPnt = theCurve->Value (aParam);
    theMinPnt.SetX (std::min (theMinPnt.X(), aPnt.X()));
    theMinPnt.SetY (std::min (theMinPnt.Y(), aPnt.Y()));
    theMaxPnt.SetX (std::max (theMaxPnt.X(), aPnt.X()));
    theMaxPnt.SetY (std::max (theMaxPnt.Y(), aPnt.Y()));
  }
}

// Axis-aligned block from the sampled extents.
NvGeBoundBlock2d SampledOrthoBlock (const occ::handle<Geom2d_Curve>& theCurve,
                                    double theFirst, double theLast)
{
  gp_Pnt2d aMinPnt;
  gp_Pnt2d aMaxPnt;
  SampledExtentsOf (theCurve, theFirst, theLast, aMinPnt, aMaxPnt);
  return NvGeBoundBlock2d (ToPoint2d (aMinPnt),
                           NvGeVector2d (aMaxPnt.X() - aMinPnt.X(), 0.0),
                           NvGeVector2d (0.0, aMaxPnt.Y() - aMinPnt.Y()));
}

// Oriented block of a curve over its window: lines get an exact block along
// their direction, other curves a sampled axis-aligned one.
NvGeBoundBlock2d OrientedBlockOf (const occ::handle<Geom2d_Curve>& theCurve,
                                  double theFirst, double theLast)
{
  const occ::handle<Geom2d_Line> aLine = occ::down_cast<Geom2d_Line> (theCurve);
  if (!aLine.IsNull())
  {
    // Free line: unbounded in both directions (zero vectors per the
    // NvGeBoundBlock2d convention).
    const gp_Pnt2d anOrigin = aLine->Lin2d().Location();
    return NvGeBoundBlock2d (ToPoint2d (anOrigin), NvGeVector2d (0.0, 0.0), NvGeVector2d (0.0, 0.0));
  }
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (theCurve);
  if (!aTrimmed.IsNull() && !occ::down_cast<Geom2d_Line> (aTrimmed->BasisCurve()).IsNull())
  {
    // Bounded line segment: exact oriented block along the segment.
    const gp_Pnt2d aStartPnt = theCurve->Value (theFirst);
    const gp_Pnt2d anEndPnt = theCurve->Value (theLast);
    return NvGeBoundBlock2d (ToPoint2d (aStartPnt),
                             NvGeVector2d (anEndPnt.X() - aStartPnt.X(), anEndPnt.Y() - aStartPnt.Y()),
                             NvGeVector2d (0.0, 0.0));
  }
  return SampledOrthoBlock (theCurve, theFirst, theLast);
}

// Uniform samples of a curve over its window.
void UniformSamplesOf (const occ::handle<Geom2d_Curve>& theCurve, double theFirst, double theLast,
                       int theNbSamples, NvGePoint2dArray& thePointArray, NvGeDoubleArray& theParamArray)
{
  thePointArray.setLogicalLength (theNbSamples);
  theParamArray.setLogicalLength (theNbSamples);
  for (int aSample = 0; aSample < theNbSamples; ++aSample)
  {
    const double aParam = theFirst + (theLast - theFirst) * aSample / (theNbSamples - 1);
    theParamArray[aSample] = aParam;
    thePointArray[aSample] = ToPoint2d (theCurve->Value (aParam));
  }
}

} // namespace

//=================================================================================================

void NvGeCurve2d::getInterval (NvGeInterval& theIntrvl) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  theIntrvl.set (PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()));
}

//=================================================================================================

void NvGeCurve2d::getInterval (NvGeInterval& theIntrvl, NvGePoint2d& theStart, NvGePoint2d& theEnd) const
{
  getInterval (theIntrvl);
  double aFirst = 0.0;
  double aLast = 0.0;
  theIntrvl.getBounds (aFirst, aLast);
  if (std::isfinite (aFirst))
  {
    theStart = evalPoint (aFirst);
  }
  if (std::isfinite (aLast))
  {
    theEnd = evalPoint (aLast);
  }
}

//=================================================================================================

NvGeCurve2d& NvGeCurve2d::reverseParam ()
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  ReplaceGeometry (mpImpEnt, aCurve->Reversed());
  return *this;
}

//=================================================================================================

NvGeCurve2d& NvGeCurve2d::setInterval ()
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  // Restores the natural bounds: the trimmed restriction over the basis is
  // dropped so that the curve spans its basis domain again.
  ReplaceGeometry (mpImpEnt, BasisOf (aCurve));
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::setInterval (const NvGeInterval& theIntrvl)
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aNatFirst = 0.0;
  double aNatLast = 0.0;
  NaturalBoundsOf (aCurve, aNatFirst, aNatLast);
  const double aSlack = std::max (theIntrvl.tolerance(), Precision::PConfusion());
  if ((theIntrvl.isBoundedBelow() && std::isfinite (aNatFirst)
       && theIntrvl.lowerBound() < aNatFirst - aSlack)
   || (theIntrvl.isBoundedAbove() && std::isfinite (aNatLast)
       && theIntrvl.upperBound() > aNatLast + aSlack))
  {
    return false;
  }
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theIntrvl, aNatFirst, aNatLast, aFirst, aLast);
  // Degeneracy is judged against the interval's own tolerance only: the
  // PConfusion slack above guards the natural-bounds checks, but must not
  // reject legitimately tight windows.
  if (aLast - aFirst <= theIntrvl.tolerance())
  {
    throw NvException ("NvGeCurve2d::setInterval(): the interval is degenerate");
  }
  occ::handle<Geom2d_Curve> aTrimmed;
  try
  {
    aTrimmed = occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (BasisOf (aCurve),
                                                                   OcctParam (aFirst), OcctParam (aLast)));
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  ReplaceGeometry (mpImpEnt, aTrimmed);
  return true;
}

//=================================================================================================

double NvGeCurve2d::distanceTo (const NvGePoint2d& thePnt, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  ProjectOrThrow (aCurve, gp_Pnt2d (thePnt.x, thePnt.y), "distanceTo", aParam, aPntOnCurve, aDistance);
  return aDistance;
}

//=================================================================================================

double NvGeCurve2d::distanceTo (const NvGeCurve2d& theCurve, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const occ::handle<Geom2d_Curve> anOther = NvGeCurve2dOf (theCurve.mpImpEnt);
  double aFirst1 = 0.0;
  double aLast1 = 0.0;
  double aFirst2 = 0.0;
  double aLast2 = 0.0;
  WindowOfCurve (aCurve, aFirst1, aLast1);
  WindowOfCurve (anOther, aFirst2, aLast2);
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  gp_Pnt2d aPnt1;
  gp_Pnt2d aPnt2;
  const double aDist = CurveCurveClosestOf (aCurve, aFirst1, aLast1, anOther, aFirst2, aLast2,
                                            aParam1, aPnt1, aParam2, aPnt2);
  if (std::isinf (aDist))
  {
    throw NvException ("NvGeCurve2d::distanceTo(): the projection between the curves failed");
  }
  return aDist;
}

//=================================================================================================

NvGePoint2d NvGeCurve2d::closestPointTo (const NvGePoint2d& thePnt, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  ProjectOrThrow (aCurve, gp_Pnt2d (thePnt.x, thePnt.y), "closestPointTo", aParam, aPntOnCurve, aDistance);
  return ToPoint2d (aPntOnCurve);
}

//=================================================================================================

NvGePoint2d NvGeCurve2d::closestPointTo (const NvGeCurve2d& theCurve, NvGePoint2d& thePntOnOtherCrv,
                                         const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const occ::handle<Geom2d_Curve> anOther = NvGeCurve2dOf (theCurve.mpImpEnt);
  double aFirst1 = 0.0;
  double aLast1 = 0.0;
  double aFirst2 = 0.0;
  double aLast2 = 0.0;
  WindowOfCurve (aCurve, aFirst1, aLast1);
  WindowOfCurve (anOther, aFirst2, aLast2);
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  gp_Pnt2d aPnt1;
  gp_Pnt2d aPnt2;
  const double aDist = CurveCurveClosestOf (aCurve, aFirst1, aLast1, anOther, aFirst2, aLast2,
                                            aParam1, aPnt1, aParam2, aPnt2);
  if (std::isinf (aDist))
  {
    throw NvException ("NvGeCurve2d::closestPointTo(): the projection between the curves failed");
  }
  thePntOnOtherCrv = ToPoint2d (aPnt2);
  return ToPoint2d (aPnt1);
}

//=================================================================================================

void NvGeCurve2d::getClosestPointTo (const NvGePoint2d& thePnt, NvGePointOnCurve2d& thePntOnCrv,
                                     const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  ProjectOrThrow (aCurve, gp_Pnt2d (thePnt.x, thePnt.y), "getClosestPointTo", aParam, aPntOnCurve, aDistance);
  thePntOnCrv.setCurve (*this);
  thePntOnCrv.setParameter (aParam);
}

//=================================================================================================

void NvGeCurve2d::getClosestPointTo (const NvGeCurve2d& theCurve, NvGePointOnCurve2d& thePntOnThisCrv,
                                     NvGePointOnCurve2d& thePntOnOtherCrv, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const occ::handle<Geom2d_Curve> anOther = NvGeCurve2dOf (theCurve.mpImpEnt);
  double aFirst1 = 0.0;
  double aLast1 = 0.0;
  double aFirst2 = 0.0;
  double aLast2 = 0.0;
  WindowOfCurve (aCurve, aFirst1, aLast1);
  WindowOfCurve (anOther, aFirst2, aLast2);
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  gp_Pnt2d aPnt1;
  gp_Pnt2d aPnt2;
  const double aDist = CurveCurveClosestOf (aCurve, aFirst1, aLast1, anOther, aFirst2, aLast2,
                                            aParam1, aPnt1, aParam2, aPnt2);
  if (std::isinf (aDist))
  {
    throw NvException ("NvGeCurve2d::getClosestPointTo(): the projection between the curves failed");
  }
  thePntOnThisCrv.setCurve (*this);
  thePntOnThisCrv.setParameter (aParam1);
  thePntOnOtherCrv.setCurve (theCurve);
  thePntOnOtherCrv.setParameter (aParam2);
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::getNormalPoint (const NvGePoint2d& thePnt, NvGePointOnCurve2d& thePntOnCrv,
                                            const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  const gp_Pnt2d aTarget (thePnt.x, thePnt.y);
  // Newton can stall on a vanishing denominator, so pick the best of the
  // sampled window residuals and refine from there.
  double aBestParam = aFirst;
  double aBestResidual = std::abs (NormalResidualOf (aCurve, aTarget, aBestParam));
  for (int aSample = 1; aSample < THE_NB_SAMPLES; ++aSample)
  {
    const double aTrial = aFirst + (aLast - aFirst) * aSample / (THE_NB_SAMPLES - 1);
    const double aTrialResidual = std::abs (NormalResidualOf (aCurve, aTarget, aTrial));
    if (aTrialResidual < aBestResidual)
    {
      aBestResidual = aTrialResidual;
      aBestParam = aTrial;
    }
  }
  RefineNormalFoot (aCurve, aFirst, aLast, aTarget, aBestParam);
  // The target lies on the normal line through the foot point when the
  // residual vanishes relative to the tangent and radial magnitudes.
  gp_Pnt2d aFootPnt;
  gp_Vec2d aTangent;
  aCurve->D1 (aBestParam, aFootPnt, aTangent);
  const gp_Vec2d aRadial (aFootPnt, aTarget);
  const double aScale = std::max (1.0, aTangent.Magnitude() * aRadial.Magnitude());
  if (std::abs (aTangent.Dot (aRadial)) > theTol.equalVector() * aScale)
  {
    return false;
  }
  thePntOnCrv.setCurve (*this);
  thePntOnCrv.setParameter (aBestParam);
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isOn (const NvGePoint2d& thePnt, const NvGeTol& theTol) const
{
  return distanceTo (thePnt, theTol) <= theTol.equalPoint();
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isOn (const NvGePoint2d& thePnt, double& theParam, const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  ProjectOrThrow (aCurve, gp_Pnt2d (thePnt.x, thePnt.y), "isOn", aParam, aPntOnCurve, aDistance);
  if (aDistance > theTol.equalPoint())
  {
    return false;
  }
  theParam = aParam;
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isOn (double theParam, const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = PlainParam (aCurve->FirstParameter());
  const double aLast = PlainParam (aCurve->LastParameter());
  const double aSlack = theTol.equalPoint();
  return theParam >= aFirst - aSlack && theParam <= aLast + aSlack;
}

//=================================================================================================

double NvGeCurve2d::paramOf (const NvGePoint2d& thePnt, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  ProjectOrThrow (aCurve, gp_Pnt2d (thePnt.x, thePnt.y), "paramOf", aParam, aPntOnCurve, aDistance);
  return aParam;
}

//=================================================================================================

void NvGeCurve2d::getTrimmedOffset (double theDistance, NvGeVoidPointerArray& theOffsetCurveList,
                                    NvGe::OffsetCrvExtType, const NvGeTol&) const
{
  // The ARX extension types (fillet / chamfer / extend) have no counterpart
  // for a single OCCT offset curve and are ignored: the offset covers the
  // exact curve range.
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  occ::handle<Geom2d_OffsetCurve> anOffset;
  try
  {
    anOffset = new Geom2d_OffsetCurve (aCurve, theDistance);
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  theOffsetCurveList.append ((NvGeCurve2d*) newEntity2d (new NvGeImpEntity3d (NvGe::kOffsetCurve2d,
                                                                              anOffset)));
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isClosed (const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = PlainParam (aCurve->FirstParameter());
  const double aLast = PlainParam (aCurve->LastParameter());
  if (!std::isfinite (aFirst) || !std::isfinite (aLast))
  {
    return false; // unbounded curves are never closed
  }
  // A trimmed restriction over a closed conic is closed only when it
  // covers the full periodic domain.
  const occ::handle<Geom2d_Curve> aBasis = BasisOf (aCurve);
  const bool isClosedConic = !occ::down_cast<Geom2d_Circle> (aBasis).IsNull()
                          || !occ::down_cast<Geom2d_Ellipse> (aBasis).IsNull();
  if (isClosedConic)
  {
    return (aLast - aFirst) >= 2.0 * std::acos (-1.0) - theTol.equalVector();
  }
  return aCurve->Value (aFirst).Distance (aCurve->Value (aLast)) <= theTol.equalPoint();
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isPeriodic (double& thePeriod) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  // A trimmed restriction carries a sub-range of the periodic domain and is
  // therefore not periodic itself.
  if (!occ::down_cast<Geom2d_TrimmedCurve> (aCurve).IsNull())
  {
    return false;
  }
  const occ::handle<Geom2d_Curve> aBasis = BasisOf (aCurve);
  const bool isConic = !occ::down_cast<Geom2d_Circle> (aBasis).IsNull()
                    || !occ::down_cast<Geom2d_Ellipse> (aBasis).IsNull();
  if (isConic)
  {
    thePeriod = 2.0 * std::acos (-1.0);
    return true;
  }
  if (aCurve->IsPeriodic())
  {
    thePeriod = aCurve->Period();
    return true;
  }
  return false;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isLinear (NvGeLine2d& theLine, const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const occ::handle<Geom2d_Line> aLine = occ::down_cast<Geom2d_Line> (BasisOf (aCurve));
  if (!aLine.IsNull())
  {
    const gp_Ax2d anAx2d = aLine->Position();
    theLine.set (NvGePoint2d (anAx2d.Location().X(), anAx2d.Location().Y()),
                 NvGeVector2d (anAx2d.Direction().X(), anAx2d.Direction().Y()));
    return true;
  }
  // Chord collinearity test: every sample must lie within the vector
  // tolerance of the chord through the window endpoints.
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  const gp_Pnt2d aStartPnt = aCurve->Value (aFirst);
  const gp_Pnt2d anEndPnt = aCurve->Value (aLast);
  const gp_Vec2d aChord (aStartPnt, anEndPnt);
  const double aChordLen = aChord.Magnitude();
  if (aChordLen <= theTol.equalPoint())
  {
    return false; // a degenerate window collapses to a point, not a line
  }
  for (int aSample = 1; aSample < THE_NB_SAMPLES - 1; ++aSample)
  {
    const double aParam = aFirst + (aLast - aFirst) * aSample / (THE_NB_SAMPLES - 1);
    const gp_Vec2d aRadial (aStartPnt, aCurve->Value (aParam));
    if (aChord.CrossMagnitude (aRadial) / aChordLen > theTol.equalVector())
    {
      return false;
    }
  }
  theLine.set (ToPoint2d (aStartPnt),
               NvGeVector2d (anEndPnt.X() - aStartPnt.X(), anEndPnt.Y() - aStartPnt.Y()));
  return true;
}

//=================================================================================================

double NvGeCurve2d::length (double theFromParam, double theToParam, double theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  double aFrom = std::min (std::max (theFromParam, aFirst), aLast);
  double aTo = std::min (std::max (theToParam, aFirst), aLast);
  if (aFrom > aTo)
  {
    std::swap (aFrom, aTo);
  }
  if (!std::isfinite (aFrom) || !std::isfinite (aTo))
  {
    throw NvException ("NvGeCurve2d::length(): the curve is unbounded on the requested range");
  }
  const Geom2dAdaptor_Curve anAdaptor (aCurve, OcctParam (aFirst), OcctParam (aLast));
  return GCPnts_AbscissaPoint::Length (anAdaptor, aFrom, aTo, theTol);
}

//=================================================================================================

double NvGeCurve2d::paramAtLength (double theDatumParam, double theLength, Adesk::Boolean thePosParamDir,
                                   double theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  const double aDatum = std::min (std::max (theDatumParam, aFirst), aLast);
  if (!std::isfinite (aDatum))
  {
    throw NvException ("NvGeCurve2d::paramAtLength(): the datum parameter is unbounded");
  }
  if (theLength <= theTol)
  {
    return aDatum;
  }
  const Geom2dAdaptor_Curve anAdaptor (aCurve, OcctParam (aFirst), OcctParam (aLast));
  // When the requested length reaches beyond the window end, the result is
  // the end parameter itself.
  const double anEndParam = thePosParamDir ? aLast : aFirst;
  if (std::isfinite (anEndParam))
  {
    const double anEndLength = GCPnts_AbscissaPoint::Length (anAdaptor, aDatum, anEndParam, theTol);
    if (theLength >= anEndLength - theTol)
    {
      return anEndParam;
    }
  }
  const GCPnts_AbscissaPoint aSolver (theTol, anAdaptor,
                                      thePosParamDir ? theLength : -theLength, aDatum);
  if (!aSolver.IsDone())
  {
    throw NvException ("NvGeCurve2d::paramAtLength(): the computation did not converge");
  }
  return PlainParam (aSolver.Parameter());
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::area (double theStartParam, double theEndParam, double& theValue,
                                  const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  const double aStart = std::min (std::max (theStartParam, aFirst), aLast);
  const double anEnd = std::min (std::max (theEndParam, aFirst), aLast);
  if (!std::isfinite (aStart) || !std::isfinite (anEnd))
  {
    throw NvException ("NvGeCurve2d::area(): the requested range is unbounded");
  }
  // Shoelace quadrature over the sampled arc plus the closing chord edge.
  const gp_Pnt2d aStartPnt = aCurve->Value (aStart);
  gp_Pnt2d aPrev = aStartPnt;
  double aSum = 0.0;
  for (int aSample = 1; aSample <= THE_NB_AREA_SAMPLES; ++aSample)
  {
    const double aParam = aStart + (anEnd - aStart) * aSample / THE_NB_AREA_SAMPLES;
    const gp_Pnt2d aPnt = aCurve->Value (aParam);
    aSum += aPrev.X() * aPnt.Y() - aPnt.X() * aPrev.Y();
    aPrev = aPnt;
  }
  aSum += aPrev.X() * aStartPnt.Y() - aStartPnt.X() * aPrev.Y();
  theValue = 0.5 * aSum;
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isDegenerate (NvGe::EntityId& theDegenerateType, const NvGeTol& theTol) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = PlainParam (aCurve->FirstParameter());
  const double aLast = PlainParam (aCurve->LastParameter());
  if (std::isfinite (aFirst) && std::isfinite (aLast) && aLast - aFirst <= theTol.equalPoint())
  {
    theDegenerateType = NvGe::kPointEnt2d;
    return true;
  }
  // A B-spline whose poles all coincide within the tolerance is a point
  // regardless of its parameter range.
  const occ::handle<Geom2d_BSplineCurve> aSpline = occ::down_cast<Geom2d_BSplineCurve> (aCurve);
  if (!aSpline.IsNull())
  {
    const gp_Pnt2d aRefPole = aSpline->Pole (1);
    for (int aPole = 2; aPole <= aSpline->NbPoles(); ++aPole)
    {
      if (aRefPole.Distance (aSpline->Pole (aPole)) > theTol.equalPoint())
      {
        return false;
      }
    }
    theDegenerateType = NvGe::kPointEnt2d;
    return true;
  }
  return false;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::isDegenerate (NvGeEntity2d*& theConvertedEntity, const NvGeTol& theTol) const
{
  NvGe::EntityId aType = NvGe::kEntity2d;
  if (!isDegenerate (aType, theTol))
  {
    theConvertedEntity = nullptr;
    return false;
  }
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = PlainParam (aCurve->FirstParameter());
  const double aLast = PlainParam (aCurve->LastParameter());
  const double aMid = 0.5 * (aFirst + aLast);
  const double anEvalParam = std::isfinite (aMid) ? aMid : (std::isfinite (aFirst) ? aFirst : 0.0);
  const occ::handle<NvGePosition2dData> aData = new NvGePosition2dData();
  aData->Point = aCurve->Value (anEvalParam);
  theConvertedEntity = newEntity2d (new NvGeImpEntity3d (NvGe::kPosition2d, aData));
  return true;
}

//=================================================================================================

void NvGeCurve2d::getSplitCurves (double theParam, NvGeCurve2d*& thePiece1, NvGeCurve2d*& thePiece2) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = PlainParam (aCurve->FirstParameter());
  const double aLast = PlainParam (aCurve->LastParameter());
  if (!std::isfinite (aFirst) || !std::isfinite (aLast) || theParam <= aFirst || theParam >= aLast)
  {
    throw NvException ("NvGeCurve2d::getSplitCurves(): the split parameter is outside the curve bounds");
  }
  // Both pieces are trimmed over the basis so that a split arc stays an
  // arc of the same circle.
  const occ::handle<Geom2d_Curve> aBasis = BasisOf (aCurve);
  occ::handle<Geom2d_Curve> aPiece1;
  occ::handle<Geom2d_Curve> aPiece2;
  try
  {
    aPiece1 = occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (aBasis, OcctParam (aFirst),
                                                                  OcctParam (theParam)));
    aPiece2 = occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (aBasis, OcctParam (theParam),
                                                                  OcctParam (aLast)));
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  thePiece1 = (NvGeCurve2d*) newEntity2d (new NvGeImpEntity3d (mpImpEnt->Type(), aPiece1));
  thePiece2 = (NvGeCurve2d*) newEntity2d (new NvGeImpEntity3d (mpImpEnt->Type(), aPiece2));
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::explode (NvGeVoidPointerArray&, NvGeIntArray&, const NvGeInterval*) const
{
  // A generic curve has no sub-curve structure to explode into.
  return false;
}

//=================================================================================================

void NvGeCurve2d::getLocalClosestPoints (const NvGePoint2d& thePoint, NvGePointOnCurve2d& theApproxPnt,
                                         const NvGeInterval* theNbhd, const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theNbhd != nullptr ? *theNbhd : NvGeInterval(),
                    PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()),
                    aFirst, aLast);
  double aParam = 0.0;
  gp_Pnt2d aPntOnCurve;
  double aDistance = 0.0;
  if (!ProjectPointOnto (gp_Pnt2d (thePoint.x, thePoint.y), aCurve, aFirst, aLast,
                         aParam, aPntOnCurve, aDistance))
  {
    throw NvException ("NvGeCurve2d::getLocalClosestPoints(): the projection onto the curve failed");
  }
  theApproxPnt.setCurve (*this);
  theApproxPnt.setParameter (aParam);
}

//=================================================================================================

void NvGeCurve2d::getLocalClosestPoints (const NvGeCurve2d& theOtherCurve,
                                         NvGePointOnCurve2d& theApproxPntOnThisCrv,
                                         NvGePointOnCurve2d& theApproxPntOnOtherCrv,
                                         const NvGeInterval* theNbhd1, const NvGeInterval* theNbhd2,
                                         const NvGeTol&) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const occ::handle<Geom2d_Curve> anOther = NvGeCurve2dOf (theOtherCurve.mpImpEnt);
  double aFirst1 = 0.0;
  double aLast1 = 0.0;
  double aFirst2 = 0.0;
  double aLast2 = 0.0;
  SamplingWindowOf (theNbhd1 != nullptr ? *theNbhd1 : NvGeInterval(),
                    PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()),
                    aFirst1, aLast1);
  SamplingWindowOf (theNbhd2 != nullptr ? *theNbhd2 : NvGeInterval(),
                    PlainParam (anOther->FirstParameter()), PlainParam (anOther->LastParameter()),
                    aFirst2, aLast2);
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  gp_Pnt2d aPnt1;
  gp_Pnt2d aPnt2;
  const double aDist = CurveCurveClosestOf (aCurve, aFirst1, aLast1, anOther, aFirst2, aLast2,
                                            aParam1, aPnt1, aParam2, aPnt2);
  if (std::isinf (aDist))
  {
    throw NvException ("NvGeCurve2d::getLocalClosestPoints(): the projection between the curves failed");
  }
  theApproxPntOnThisCrv.setCurve (*this);
  theApproxPntOnThisCrv.setParameter (aParam1);
  theApproxPntOnOtherCrv.setCurve (theOtherCurve);
  theApproxPntOnOtherCrv.setParameter (aParam2);
}

//=================================================================================================

NvGeBoundBlock2d NvGeCurve2d::boundBlock () const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  return OrientedBlockOf (aCurve, aFirst, aLast);
}

//=================================================================================================

NvGeBoundBlock2d NvGeCurve2d::boundBlock (const NvGeInterval& theRange) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theRange, PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()),
                    aFirst, aLast);
  return OrientedBlockOf (aCurve, aFirst, aLast);
}

//=================================================================================================

NvGeBoundBlock2d NvGeCurve2d::orthoBoundBlock () const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  return SampledOrthoBlock (aCurve, aFirst, aLast);
}

//=================================================================================================

NvGeBoundBlock2d NvGeCurve2d::orthoBoundBlock (const NvGeInterval& theRange) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (theRange, PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()),
                    aFirst, aLast);
  return SampledOrthoBlock (aCurve, aFirst, aLast);
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::hasStartPoint (NvGePoint2d& theStartPoint) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aFirst = aCurve->FirstParameter();
  if (Precision::IsNegativeInfinite (aFirst))
  {
    return false;
  }
  theStartPoint = evalPoint (aFirst);
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCurve2d::hasEndPoint (NvGePoint2d& theEndPoint) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  const double aLast = aCurve->LastParameter();
  if (Precision::IsPositiveInfinite (aLast))
  {
    return false;
  }
  theEndPoint = evalPoint (aLast);
  return true;
}

//=================================================================================================

NvGePoint2d NvGeCurve2d::evalPoint (double theParam) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  return ToPoint2d (aCurve->Value (theParam));
}

//=================================================================================================

NvGePoint2d NvGeCurve2d::evalPoint (double theParam, int theNumDeriv, NvGeVector2dArray& theDerivArray) const
{
  if (theNumDeriv < 0 || theNumDeriv > 3)
  {
    throw NvException ("NvGeCurve2d::evalPoint(): the derivative order must be between 0 and 3");
  }
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  theDerivArray.setLogicalLength (theNumDeriv);
  gp_Pnt2d aPnt;
  if (theNumDeriv == 0)
  {
    aPnt = aCurve->Value (theParam);
  }
  else
  {
    gp_Vec2d aDeriv1;
    gp_Vec2d aDeriv2;
    gp_Vec2d aDeriv3;
    if (theNumDeriv == 1)
    {
      aCurve->D1 (theParam, aPnt, aDeriv1);
    }
    else if (theNumDeriv == 2)
    {
      aCurve->D2 (theParam, aPnt, aDeriv1, aDeriv2);
    }
    else
    {
      aCurve->D3 (theParam, aPnt, aDeriv1, aDeriv2, aDeriv3);
    }
    theDerivArray[0] = NvGeVector2d (aDeriv1.X(), aDeriv1.Y());
    if (theNumDeriv >= 2)
    {
      theDerivArray[1] = NvGeVector2d (aDeriv2.X(), aDeriv2.Y());
    }
    if (theNumDeriv >= 3)
    {
      theDerivArray[2] = NvGeVector2d (aDeriv3.X(), aDeriv3.Y());
    }
  }
  return ToPoint2d (aPnt);
}

//=================================================================================================

void NvGeCurve2d::getSamplePoints (double theFromParam, double theToParam, double theApproxEps,
                                   NvGePoint2dArray& thePointArray, NvGeDoubleArray& theParamArray) const
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  SamplingWindowOf (NvGeInterval (theFromParam, theToParam),
                    PlainParam (aCurve->FirstParameter()), PlainParam (aCurve->LastParameter()),
                    aFirst, aLast);
  // The deflection sampler is the primary path; uniform sampling is the
  // documented recovery when the algorithm refuses the input.
  bool isSampled = false;
  try
  {
    const Geom2dAdaptor_Curve anAdaptor (aCurve, OcctParam (aFirst), OcctParam (aLast));
    const GCPnts_QuasiUniformDeflection aSampler (anAdaptor, std::max (theApproxEps, Precision::Confusion()),
                                                  aFirst, aLast);
    if (aSampler.IsDone() && aSampler.NbPoints() >= 2)
    {
      const int aNbPoints = aSampler.NbPoints();
      thePointArray.setLogicalLength (aNbPoints);
      theParamArray.setLogicalLength (aNbPoints);
      for (int anIdx = 0; anIdx < aNbPoints; ++anIdx)
      {
        const double aParam = PlainParam (aSampler.Parameter (anIdx + 1));
        theParamArray[anIdx] = aParam;
        thePointArray[anIdx] = ToPoint2d (aCurve->Value (aParam));
      }
      isSampled = true;
    }
  }
  catch (const Standard_Failure&)
  {
    isSampled = false; // handled below through uniform sampling
  }
  if (!isSampled)
  {
    UniformSamplesOf (aCurve, aFirst, aLast, THE_NB_SAMPLES, thePointArray, theParamArray);
  }
}

//=================================================================================================

void NvGeCurve2d::getSamplePoints (int theNumSample, NvGePoint2dArray& thePointArray) const
{
  if (theNumSample < 2)
  {
    throw NvException ("NvGeCurve2d::getSamplePoints(): at least two samples are required");
  }
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (mpImpEnt);
  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOfCurve (aCurve, aFirst, aLast);
  NvGeDoubleArray aParamArray;
  UniformSamplesOf (aCurve, aFirst, aLast, theNumSample, thePointArray, aParamArray);
}

//=================================================================================================

NvGeCurve2d& NvGeCurve2d::operator = (const NvGeCurve2d& theCurve)
{
  NvGeEntity2d::operator= (theCurve);
  return *this;
}

//=================================================================================================

NvGeCurve2d::NvGeCurve2d ()
{
}

//=================================================================================================

NvGeCurve2d::NvGeCurve2d (const NvGeCurve2d& theSrc)
: NvGeEntity2d (theSrc)
{
}
