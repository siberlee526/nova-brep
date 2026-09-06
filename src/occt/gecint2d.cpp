// gecint2d.cpp - implementation of NvGeCurveCurveInt2d.
//
// The intersection entity is an empty NvGeEntity2d shell whose impl carries
// a file-local NvGeCurveCurveInt2dData holder: the input curve impls, the
// query ranges / tolerance and the eager intersection results (parameter,
// point, configuration and tangency vectors plus the tangential overlap
// segments). Constructors and set() run Geom2dAPI_InterCurveCurve eagerly
// and fill the holder; every query only reads it. Interval-restricted
// variants wrap their inputs into Geom2d_TrimmedCurve windows first; the
// trimmed curves keep the basis parameterization, so reported parameters
// stay in the parameterization of the original curves.

#include <gecint2d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <geintrvl.h>
#include <gepnt2d.h>
#include <geponc2d.h>
#include <getol.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity2d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Storage behind NvGeCurveCurveInt2d: inputs and eager results.
class NvGeCurveCurveInt2dData : public NvGeEntityData
{
public:

  occ::handle<NvGeImpEntity3d> Curve1;
  occ::handle<NvGeImpEntity3d> Curve2;
  NvGeInterval                 Range1;
  NvGeInterval                 Range2;
  NvGeTol                      Tol;

  // Transverse (and formally tangential) intersection points.
  std::vector<double>            Param1;
  std::vector<double>            Param2;
  std::vector<gp_Pnt2d>          Points;
  std::vector<NvGe::NvGeXConfig> Config1; // curve1 relative to curve2
  std::vector<NvGe::NvGeXConfig> Config2; // curve2 relative to curve1
  std::vector<Nova::Boolean>    Tangential;
  std::vector<double>            PointTols;

  // Tangential overlap segments: parameter ranges on both curves.
  std::vector<double> OvFirst1;
  std::vector<double> OvLast1;
  std::vector<double> OvFirst2;
  std::vector<double> OvLast2;
  Nova::Boolean      OverlapDir = true;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeCurveCurveInt2dData> aCopy = new NvGeCurveCurveInt2dData();
    aCopy->Curve1 = Curve1;
    aCopy->Curve2 = Curve2;
    aCopy->Range1 = Range1;
    aCopy->Range2 = Range2;
    aCopy->Tol = Tol;
    aCopy->Param1 = Param1;
    aCopy->Param2 = Param2;
    aCopy->Points = Points;
    aCopy->Config1 = Config1;
    aCopy->Config2 = Config2;
    aCopy->Tangential = Tangential;
    aCopy->PointTols = PointTols;
    aCopy->OvFirst1 = OvFirst1;
    aCopy->OvLast1 = OvLast1;
    aCopy->OvFirst2 = OvFirst2;
    aCopy->OvLast2 = OvLast2;
    aCopy->OverlapDir = OverlapDir;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeCurveCurveInt2dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeCurveCurveInt2dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeCurveCurveInt2dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeCurveCurveInt2dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeCurveCurveInt2dData> (theImp->Geom());
}

//! Validates a query index against the result count.
void CheckIndex (int theIntNum, int theCount, const char* theMethod)
{
  if (theIntNum < 0 || theIntNum >= theCount)
  {
    throw NvException (std::string ("NvGeCurveCurveInt2d::") + theMethod
                       + "(): intersection index out of range");
  }
}

//! Maps an OCCT parametric bound to a plain double (+/-2e100 markers
//! become +/- infinity so bounds combine with std::isfinite).
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

//! Inverse of PlainParam: maps an infinite bound back to the OCCT infinite
//! marker expected by the Geom2d_TrimmedCurve constructor.
double OcctParam (const double theParam)
{
  if (std::isinf (theParam))
  {
    return theParam > 0.0 ? Precision::Infinite() : -Precision::Infinite();
  }
  return theParam;
}

//! Restricts a curve to the requested range. A fully unbounded range keeps
//! the natural domain; finite bounds become a Geom2d_TrimmedCurve window
//! (which preserves the basis parameterization).
occ::handle<Geom2d_Curve> RestrictedOf (const occ::handle<Geom2d_Curve>& theCurve,
                                        const NvGeInterval& theRange)
{
  if (theRange.isUnBounded())
  {
    return theCurve;
  }
  try
  {
    return occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (
      theCurve, OcctParam (theRange.lowerBound()), OcctParam (theRange.upperBound()), true, true));
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Parameter step used to sample a curve around an intersection point:
//! a tiny fraction of the natural span, clamped for huge / unbounded
//! domains.
double StepOf (const occ::handle<Geom2d_Curve>& theCurve)
{
  const double aSpan = PlainParam (theCurve->LastParameter())
                     - PlainParam (theCurve->FirstParameter());
  double anEffSpan = aSpan;
  if (!std::isfinite (anEffSpan) || anEffSpan > 1e3)
  {
    anEffSpan = 2.0;
  }
  return std::max (anEffSpan * 1e-5, 1e-9);
}

//! Signed side of a point relative to the unit tangent of a curve at a
//! parameter: > 0 left of the curve, < 0 right, ~0 on the tangent line.
double SideOf (const gp_Pnt2d& thePoint, const occ::handle<Geom2d_Curve>& theCurve,
               double theParam)
{
  gp_Vec2d aTangent = theCurve->DN (theParam, 1);
  const double aLen = aTangent.Magnitude();
  if (aLen <= 1e-12)
  {
    return 0.0;
  }
  gp_Vec2d aRel (thePoint.XY() - theCurve->Value (theParam).XY());
  return (aTangent.X() * aRel.Y() - aTangent.Y() * aRel.X()) / aLen;
}

//! Maps the before/after sides of one curve to its configuration with
//! respect to the other curve at the intersection.
NvGe::NvGeXConfig ConfigOf (double theBefore, double theAfter, double theTol)
{
  if (theBefore > theTol && theAfter < -theTol)
  {
    return NvGe::kLeftRight;
  }
  if (theBefore < -theTol && theAfter > theTol)
  {
    return NvGe::kRightLeft;
  }
  if (theBefore > theTol && theAfter > theTol)
  {
    return NvGe::kLeftLeft;
  }
  if (theBefore < -theTol && theAfter < -theTol)
  {
    return NvGe::kRightRight;
  }
  // Touching configuration: the curve stays on one side or collapses to a
  // point on the tangent line.
  if (theAfter > theTol || theBefore > theTol)
  {
    return NvGe::kPointLeft;
  }
  if (theAfter < -theTol || theBefore < -theTol)
  {
    return NvGe::kPointRight;
  }
  return NvGe::kUnknown;
}

//! Configuration of theCurve around its intersection parameter with
//! respect to the tangent of theOtherCurve at theOtherParam.
NvGe::NvGeXConfig ConfigWrtOf (const occ::handle<Geom2d_Curve>& theCurve, double theParam,
                               const occ::handle<Geom2d_Curve>& theOtherCurve,
                               double theOtherParam, double theTol)
{
  const double aStep = StepOf (theCurve);
  const double aBefore = SideOf (theCurve->Value (theParam - aStep), theOtherCurve, theOtherParam);
  const double anAfter = SideOf (theCurve->Value (theParam + aStep), theOtherCurve, theOtherParam);
  return ConfigOf (aBefore, anAfter, theTol);
}

//! Tangency test by relative tangent alignment at the intersection.
bool IsTangentialAt (const occ::handle<Geom2d_Curve>& theCurve1, double theParam1,
                     const occ::handle<Geom2d_Curve>& theCurve2, double theParam2)
{
  gp_Vec2d aTan1 = theCurve1->DN (theParam1, 1);
  gp_Vec2d aTan2 = theCurve2->DN (theParam2, 1);
  const double aNorm = aTan1.Magnitude() * aTan2.Magnitude();
  if (aNorm <= 1e-12)
  {
    return true; // degenerate tangents: no transversal evidence
  }
  return std::abs (aTan1.X() * aTan2.Y() - aTan1.Y() * aTan2.X()) / aNorm <= 1e-6;
}

//! Runs the eager intersection computation and fills the holder.
void ComputeInto (NvGeCurveCurveInt2dData& theData)
{
  theData.Param1.clear();
  theData.Param2.clear();
  theData.Points.clear();
  theData.Config1.clear();
  theData.Config2.clear();
  theData.Tangential.clear();
  theData.PointTols.clear();
  theData.OvFirst1.clear();
  theData.OvLast1.clear();
  theData.OvFirst2.clear();
  theData.OvLast2.clear();
  theData.OverlapDir = true;

  const occ::handle<Geom2d_Curve> aCurve1 = RestrictedOf (NvGeCurve2dOf (theData.Curve1.get()),
                                                          theData.Range1);
  const occ::handle<Geom2d_Curve> aCurve2 = RestrictedOf (NvGeCurve2dOf (theData.Curve2.get()),
                                                          theData.Range2);
  try
  {
    Geom2dAPI_InterCurveCurve anInter (aCurve1, aCurve2, theData.Tol.equalPoint());

    // Transverse intersection points: parameters are recovered by
    // projecting the intersection point onto each restricted curve.
    const int aNbPoints = anInter.NbPoints();
    theData.Param1.reserve (aNbPoints);
    theData.Param2.reserve (aNbPoints);
    theData.Points.reserve (aNbPoints);
    theData.Config1.reserve (aNbPoints);
    theData.Config2.reserve (aNbPoints);
    theData.Tangential.reserve (aNbPoints);
    theData.PointTols.reserve (aNbPoints);
    for (int anIdx = 1; anIdx <= aNbPoints; ++anIdx)
    {
      const gp_Pnt2d anXPoint = anInter.Point (anIdx);
      const Geom2dAPI_ProjectPointOnCurve aProj1 (anXPoint, aCurve1);
      const Geom2dAPI_ProjectPointOnCurve aProj2 (anXPoint, aCurve2);
      if (aProj1.NbPoints() == 0 || aProj2.NbPoints() == 0)
      {
        continue;
      }
      const double aParam1 = aProj1.LowerDistanceParameter();
      const double aParam2 = aProj2.LowerDistanceParameter();
      theData.Param1.push_back (aParam1);
      theData.Param2.push_back (aParam2);
      theData.Points.push_back (anXPoint);
      theData.Config1.push_back (ConfigWrtOf (aCurve1, aParam1, aCurve2, aParam2,
                                              theData.Tol.equalPoint()));
      theData.Config2.push_back (ConfigWrtOf (aCurve2, aParam2, aCurve1, aParam1,
                                              theData.Tol.equalPoint()));
      theData.Tangential.push_back (IsTangentialAt (aCurve1, aParam1, aCurve2, aParam2));
      // Actual gap between the two curves at the reported parameters.
      theData.PointTols.push_back (aProj1.NearestPoint().Distance (aProj2.NearestPoint()));
    }

    // Tangential overlap segments: the algorithm returns the coincident
    // portions as trimmed curves on both curves (basis parameterization).
    const int aNbSegments = anInter.NbSegments();
    theData.OvFirst1.reserve (aNbSegments);
    theData.OvLast1.reserve (aNbSegments);
    theData.OvFirst2.reserve (aNbSegments);
    theData.OvLast2.reserve (aNbSegments);
    for (int anIdx = 1; anIdx <= aNbSegments; ++anIdx)
    {
      occ::handle<Geom2d_Curve> aSeg1;
      occ::handle<Geom2d_Curve> aSeg2;
      anInter.Segment (anIdx, aSeg1, aSeg2);
      const double aFirst1 = aSeg1->FirstParameter();
      const double aLast1 = aSeg1->LastParameter();
      const double aFirst2 = aSeg2->FirstParameter();
      const double aLast2 = aSeg2->LastParameter();
      theData.OvFirst1.push_back (aFirst1);
      theData.OvLast1.push_back (aLast1);
      theData.OvFirst2.push_back (aFirst2);
      theData.OvLast2.push_back (aLast2);
      // Direction agreement at the middle of the overlap.
      gp_Vec2d aTan1 = aCurve1->DN (0.5 * (aFirst1 + aLast1), 1);
      gp_Vec2d aTan2 = aCurve2->DN (0.5 * (aFirst2 + aLast2), 1);
      if (theData.OvFirst1.size() == 1)
      {
        theData.OverlapDir = aTan1.Dot (aTan2) >= 0.0;
      }
    }
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Permutes the per-point results by the given index permutation.
void PermutePoints (NvGeCurveCurveInt2dData& theData, const std::vector<int>& theOrder)
{
  auto aPermute = [theOrder] (std::vector<double>& theVec)
  {
    std::vector<double> aCopy = theVec;
    for (size_t anIdx = 0; anIdx < theOrder.size(); ++anIdx)
    {
      theVec[anIdx] = aCopy[theOrder[anIdx]];
    }
  };
  aPermute (theData.Param1);
  aPermute (theData.Param2);
  auto aPermutePnts = [theOrder] (std::vector<gp_Pnt2d>& theVec)
  {
    std::vector<gp_Pnt2d> aCopy = theVec;
    for (size_t anIdx = 0; anIdx < theOrder.size(); ++anIdx)
    {
      theVec[anIdx] = aCopy[theOrder[anIdx]];
    }
  };
  aPermutePnts (theData.Points);
  auto aPermuteCfg = [theOrder] (std::vector<NvGe::NvGeXConfig>& theVec)
  {
    std::vector<NvGe::NvGeXConfig> aCopy = theVec;
    for (size_t anIdx = 0; anIdx < theOrder.size(); ++anIdx)
    {
      theVec[anIdx] = aCopy[theOrder[anIdx]];
    }
  };
  aPermuteCfg (theData.Config1);
  aPermuteCfg (theData.Config2);
  auto aPermuteFlg = [theOrder] (std::vector<Nova::Boolean>& theVec)
  {
    std::vector<Nova::Boolean> aCopy = theVec;
    for (size_t anIdx = 0; anIdx < theOrder.size(); ++anIdx)
    {
      theVec[anIdx] = aCopy[theOrder[anIdx]];
    }
  };
  aPermuteFlg (theData.Tangential);
  aPermute (theData.PointTols);
}

//! Fills a point-on-curve output wrapper through the documented layout
//! bridge: NvGePointOnCurve2d is an empty entity shell around the same
//! mpImpEnt / mDelEnt pair (mpImpEnt first, pack 8). The previous slot
//! value is overwritten without unref - the point-on-curve constructors
//! leave the slot unowned, so it can never be released from here.
void FillPointOnCurve (NvGePointOnCurve2d& thePnt, const occ::handle<NvGeImpEntity3d>& theCurve,
                       double theParam)
{
  occ::handle<NvGePointOnCurve2dData> aData = new NvGePointOnCurve2dData();
  aData->Curve = theCurve;
  aData->Param = theParam;
  occ::handle<NvGeImpEntity3d> anImpl = new NvGeImpEntity3d (NvGe::kPointOnCurve2d, aData);
  anImpl->Ref();
  *reinterpret_cast<NvGeImpEntity3d**> (&thePnt) = anImpl.get();
  *reinterpret_cast<int*> (reinterpret_cast<char*> (&thePnt) + sizeof (NvGeImpEntity3d*)) = 1;
}

//! Validates the input curves and installs them into a fresh holder.
void PrepareData (NvGeCurveCurveInt2dData& theData, const NvGeCurve2d& theCurve1,
                  const NvGeCurve2d& theCurve2, const NvGeInterval& theRange1,
                  const NvGeInterval& theRange2, const NvGeTol& theTol)
{
  // Validates that both wrappers really carry 2d curve geometry.
  NvGeCurve2dOf (ImplOf (&theCurve1));
  NvGeCurve2dOf (ImplOf (&theCurve2));
  theData.Curve1 = ImplOf (&theCurve1);
  theData.Curve2 = ImplOf (&theCurve2);
  theData.Range1 = theRange1;
  theData.Range2 = theRange2;
  theData.Tol = theTol;
}

} // namespace

//=================================================================================================

NvGeCurveCurveInt2d::NvGeCurveCurveInt2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt2d,
                                  occ::handle<NvGeCurveCurveInt2dData> (new NvGeCurveCurveInt2dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt2d::NvGeCurveCurveInt2d (const NvGeCurve2d& theCurve1, const NvGeCurve2d& theCurve2,
                                          const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeCurveCurveInt2dData> aData = new NvGeCurveCurveInt2dData();
  PrepareData (*aData, theCurve1, theCurve2, NvGeInterval(), NvGeInterval(), theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt2d::NvGeCurveCurveInt2d (const NvGeCurve2d& theCurve1, const NvGeCurve2d& theCurve2,
                                          const NvGeInterval& theRange1, const NvGeInterval& theRange2,
                                          const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeCurveCurveInt2dData> aData = new NvGeCurveCurveInt2dData();
  PrepareData (*aData, theCurve1, theCurve2, theRange1, theRange2, theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt2d::NvGeCurveCurveInt2d (const NvGeCurveCurveInt2d& theSource)
: NvGeEntity2d (theSource)
{
}

//=================================================================================================

void NvGeCurveCurveInt2d::getIntRanges (NvGeInterval& theRange1, NvGeInterval& theRange2) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  theRange1 = aData->Range1;
  theRange2 = aData->Range2;
}

//=================================================================================================

NvGeTol NvGeCurveCurveInt2d::tolerance () const
{
  return DataOf (mpImpEnt)->Tol;
}

//=================================================================================================

int NvGeCurveCurveInt2d::numIntPoints () const
{
  return static_cast<int> (DataOf (mpImpEnt)->Points.size());
}

//=================================================================================================

NvGePoint2d NvGeCurveCurveInt2d::intPoint (int theIntNum) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "intPoint");
  const gp_Pnt2d& aPoint = aData->Points[theIntNum];
  return NvGePoint2d (aPoint.X(), aPoint.Y());
}

//=================================================================================================

void NvGeCurveCurveInt2d::getIntParams (int theIntNum, double& theParam1, double& theParam2) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getIntParams");
  theParam1 = aData->Param1[theIntNum];
  theParam2 = aData->Param2[theIntNum];
}

//=================================================================================================

void NvGeCurveCurveInt2d::getPointOnCurve1 (int theIntNum, NvGePointOnCurve2d& thePntOnCrv) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getPointOnCurve1");
  FillPointOnCurve (thePntOnCrv, aData->Curve1, aData->Param1[theIntNum]);
}

//=================================================================================================

void NvGeCurveCurveInt2d::getPointOnCurve2 (int theIntNum, NvGePointOnCurve2d& thePntOnCrv) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getPointOnCurve2");
  FillPointOnCurve (thePntOnCrv, aData->Curve2, aData->Param2[theIntNum]);
}

//=================================================================================================

void NvGeCurveCurveInt2d::getIntConfigs (int theIntNum, NvGe::NvGeXConfig& theConfig1wrt2,
                                         NvGe::NvGeXConfig& theConfig2wrt1) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getIntConfigs");
  theConfig1wrt2 = aData->Config1[theIntNum];
  theConfig2wrt1 = aData->Config2[theIntNum];
}

//=================================================================================================

Nova::Boolean NvGeCurveCurveInt2d::isTangential (int theIntNum) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  if (theIntNum < 0 || theIntNum >= static_cast<int> (aData->Tangential.size()))
  {
    return false; // predicates never throw
  }
  return aData->Tangential[theIntNum];
}

//=================================================================================================

Nova::Boolean NvGeCurveCurveInt2d::isTransversal (int theIntNum) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  if (theIntNum < 0 || theIntNum >= static_cast<int> (aData->Tangential.size()))
  {
    return false; // predicates never throw
  }
  return !aData->Tangential[theIntNum];
}

//=================================================================================================

double NvGeCurveCurveInt2d::intPointTol (int theIntNum) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->PointTols.size()), "intPointTol");
  return aData->PointTols[theIntNum];
}

//=================================================================================================

int NvGeCurveCurveInt2d::overlapCount () const
{
  return static_cast<int> (DataOf (mpImpEnt)->OvFirst1.size());
}

//=================================================================================================

Nova::Boolean NvGeCurveCurveInt2d::overlapDirection () const
{
  return DataOf (mpImpEnt)->OverlapDir;
}

//=================================================================================================

void NvGeCurveCurveInt2d::getOverlapRanges (int theOverlapNum, NvGeInterval& theRange1,
                                            NvGeInterval& theRange2) const
{
  const NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  CheckIndex (theOverlapNum, static_cast<int> (aData->OvFirst1.size()), "getOverlapRanges");
  theRange1 = NvGeInterval (aData->OvFirst1[theOverlapNum], aData->OvLast1[theOverlapNum]);
  theRange2 = NvGeInterval (aData->OvFirst2[theOverlapNum], aData->OvLast2[theOverlapNum]);
}

//=================================================================================================

void NvGeCurveCurveInt2d::changeCurveOrder ()
{
  NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  std::swap (aData->Curve1, aData->Curve2);
  std::swap (aData->Range1, aData->Range2);
  std::swap (aData->Param1, aData->Param2);
  std::swap (aData->Config1, aData->Config2);
  std::swap (aData->OvFirst1, aData->OvFirst2);
  std::swap (aData->OvLast1, aData->OvLast2);
}

//=================================================================================================

NvGeCurveCurveInt2d& NvGeCurveCurveInt2d::orderWrt1 ()
{
  NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  std::vector<int> anOrder (aData->Param1.size());
  for (size_t anIdx = 0; anIdx < anOrder.size(); ++anIdx)
  {
    anOrder[anIdx] = static_cast<int> (anIdx);
  }
  std::sort (anOrder.begin(), anOrder.end(),
             [aData] (int theLeft, int theRight)
             {
               return aData->Param1[theLeft] < aData->Param1[theRight];
             });
  PermutePoints (*aData, anOrder);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt2d& NvGeCurveCurveInt2d::orderWrt2 ()
{
  NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  std::vector<int> anOrder (aData->Param2.size());
  for (size_t anIdx = 0; anIdx < anOrder.size(); ++anIdx)
  {
    anOrder[anIdx] = static_cast<int> (anIdx);
  }
  std::sort (anOrder.begin(), anOrder.end(),
             [aData] (int theLeft, int theRight)
             {
               return aData->Param2[theLeft] < aData->Param2[theRight];
             });
  PermutePoints (*aData, anOrder);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt2d& NvGeCurveCurveInt2d::set (const NvGeCurve2d& theCurve1,
                                               const NvGeCurve2d& theCurve2, const NvGeTol& theTol)
{
  NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theCurve1, theCurve2, NvGeInterval(), NvGeInterval(), theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt2d& NvGeCurveCurveInt2d::set (const NvGeCurve2d& theCurve1,
                                               const NvGeCurve2d& theCurve2,
                                               const NvGeInterval& theRange1,
                                               const NvGeInterval& theRange2, const NvGeTol& theTol)
{
  NvGeCurveCurveInt2dData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theCurve1, theCurve2, theRange1, theRange2, theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt2d& NvGeCurveCurveInt2d::operator = (const NvGeCurveCurveInt2d& theSource)
{
  NvGeEntity2d::operator= (theSource);
  return *this;
}
