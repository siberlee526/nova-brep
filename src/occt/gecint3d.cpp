// gecint3d.cpp - implementation of NvGeCurveCurveInt3d.
//
// The intersection entity is an empty NvGeEntity3d shell whose impl carries
// a file-local NvGeCurveCurveInt3dData holder: the input curve impls, the
// query ranges / tolerance / plane-normal hint and the eager results.
// GeomAPI_ExtremaCurveCurve computes closest-point pairs; only pairs whose
// distance is within the equal-point tolerance are kept as intersection
// points. Interval-restricted variants wrap their inputs into
// Geom_TrimmedCurve windows first; trimmed curves keep the basis
// parameterization, so reported parameters stay in the parameterization of
// the original curves. Overlap queries (coincident-curve segments) have no
// extrema counterpart and always report empty ranges.

#include <gecint3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <geintrvl.h>
#include <gepnt3d.h>
#include <geponc3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_Curve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Storage behind NvGeCurveCurveInt3d: inputs and eager results.
class NvGeCurveCurveInt3dData : public NvGeEntityData
{
public:

  occ::handle<NvGeImpEntity3d> Curve1;
  occ::handle<NvGeImpEntity3d> Curve2;
  NvGeInterval                 Range1;
  NvGeInterval                 Range2;
  NvGeVector3d                 PlaneNormal; // kIdentity (zero) = derive per point
  NvGeTol                      Tol;

  std::vector<double>            Param1;
  std::vector<double>            Param2;
  std::vector<gp_Pnt>            Points;
  std::vector<NvGe::NvGeXConfig> Config1; // curve1 relative to curve2
  std::vector<NvGe::NvGeXConfig> Config2; // curve2 relative to curve1
  std::vector<Adesk::Boolean>    Tangential;
  std::vector<double>            PointTols;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeCurveCurveInt3dData> aCopy = new NvGeCurveCurveInt3dData();
    aCopy->Curve1 = Curve1;
    aCopy->Curve2 = Curve2;
    aCopy->Range1 = Range1;
    aCopy->Range2 = Range2;
    aCopy->PlaneNormal = PlaneNormal;
    aCopy->Tol = Tol;
    aCopy->Param1 = Param1;
    aCopy->Param2 = Param2;
    aCopy->Points = Points;
    aCopy->Config1 = Config1;
    aCopy->Config2 = Config2;
    aCopy->Tangential = Tangential;
    aCopy->PointTols = PointTols;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeCurveCurveInt3dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeCurveCurveInt3dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeCurveCurveInt3dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeCurveCurveInt3dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeCurveCurveInt3dData> (theImp->Geom());
}

//! Validates a query index against the result count.
void CheckIndex (int theIntNum, int theCount, const char* theMethod)
{
  if (theIntNum < 0 || theIntNum >= theCount)
  {
    throw NvException (std::string ("NvGeCurveCurveInt3d::") + theMethod
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
//! marker expected by the Geom_TrimmedCurve constructor.
double OcctParam (const double theParam)
{
  if (std::isinf (theParam))
  {
    return theParam > 0.0 ? Precision::Infinite() : -Precision::Infinite();
  }
  return theParam;
}

//! Restricts a curve to the requested range. A fully unbounded range keeps
//! the natural domain; finite bounds become a Geom_TrimmedCurve window
//! (which preserves the basis parameterization).
occ::handle<Geom_Curve> RestrictedOf (const occ::handle<Geom_Curve>& theCurve,
                                      const NvGeInterval& theRange)
{
  if (theRange.isUnBounded())
  {
    return theCurve;
  }
  try
  {
    return occ::handle<Geom_Curve> (new Geom_TrimmedCurve (
      theCurve, OcctParam (theRange.lowerBound()), OcctParam (theRange.upperBound()), true, true));
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Parameter step used to sample a curve around an intersection point.
double StepOf (const occ::handle<Geom_Curve>& theCurve)
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

//! Unit tangent of a curve at a parameter; zero vector for degenerate ones.
gp_Vec TangentOf (const occ::handle<Geom_Curve>& theCurve, double theParam)
{
  gp_Vec aTangent = theCurve->DN (theParam, 1);
  const double aLen = aTangent.Magnitude();
  if (aLen > 1e-12)
  {
    aTangent.Divide (aLen);
  }
  else
  {
    aTangent.SetCoord (0.0, 0.0, 0.0);
  }
  return aTangent;
}

//! Signed side of a point relative to the plane through theOtherPoint with
//! normal theRefNormal, oriented by theOtherTangent: > 0 left, < 0 right.
double SideOf (const gp_Pnt& thePoint, const gp_Pnt& theOtherPoint,
               const gp_Vec& theOtherTangent, const gp_Vec& theRefNormal)
{
  if (theOtherTangent.Magnitude() <= 0.5 || theRefNormal.Magnitude() <= 0.5)
  {
    return 0.0;
  }
  gp_Vec aRel (theOtherPoint.XYZ() - thePoint.XYZ());
  return theRefNormal.Dot (theOtherTangent.Crossed (aRel));
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

//! Relative tangent alignment at an intersection: true when the tangents
//! are (anti)parallel, i.e. the crossing carries no transversal evidence.
bool IsTangentialAt (const occ::handle<Geom_Curve>& theCurve1, double theParam1,
                     const occ::handle<Geom_Curve>& theCurve2, double theParam2)
{
  const gp_Vec aTan1 = TangentOf (theCurve1, theParam1);
  const gp_Vec aTan2 = TangentOf (theCurve2, theParam2);
  const double aCross = aTan1.CrossMagnitude (aTan2);
  return aCross <= 1e-6;
}

//! Reference normal classifying left/right sides at an intersection: the
//! plane-normal hint when given, otherwise the local transversal direction.
gp_Vec RefNormalOf (const NvGeVector3d& theHint, const gp_Vec& theTangent1,
                    const gp_Vec& theTangent2)
{
  const gp_Vec aHint (theHint.x, theHint.y, theHint.z);
  if (aHint.Magnitude() > 1e-12)
  {
    gp_Vec aNormalized = aHint;
    aNormalized.Divide (aHint.Magnitude());
    return aNormalized;
  }
  gp_Vec aCross = theTangent1.Crossed (theTangent2);
  const double aLen = aCross.Magnitude();
  if (aLen > 1e-12)
  {
    aCross.Divide (aLen);
    return aCross;
  }
  return gp_Vec (0.0, 0.0, 0.0);
}

//! Runs the eager intersection computation and fills the holder.
void ComputeInto (NvGeCurveCurveInt3dData& theData)
{
  theData.Param1.clear();
  theData.Param2.clear();
  theData.Points.clear();
  theData.Config1.clear();
  theData.Config2.clear();
  theData.Tangential.clear();
  theData.PointTols.clear();

  const occ::handle<Geom_Curve> aCurve1 = RestrictedOf (NvGeCurve3dOf (theData.Curve1.get()),
                                                        theData.Range1);
  const occ::handle<Geom_Curve> aCurve2 = RestrictedOf (NvGeCurve3dOf (theData.Curve2.get()),
                                                        theData.Range2);
  try
  {
    GeomAPI_ExtremaCurveCurve anExtrema (aCurve1, aCurve2);

    // Extrema give closest-point pairs; keep only the pairs that actually
    // meet within the equal-point tolerance.
    const int aNbExtrema = anExtrema.NbExtrema();
    theData.Param1.reserve (aNbExtrema);
    theData.Param2.reserve (aNbExtrema);
    theData.Points.reserve (aNbExtrema);
    theData.Config1.reserve (aNbExtrema);
    theData.Config2.reserve (aNbExtrema);
    theData.Tangential.reserve (aNbExtrema);
    theData.PointTols.reserve (aNbExtrema);
    for (int anIdx = 1; anIdx <= aNbExtrema; ++anIdx)
    {
      const double aGap = anExtrema.Distance (anIdx);
      if (aGap > theData.Tol.equalPoint())
      {
        continue;
      }
      gp_Pnt aPoint1;
      gp_Pnt aPoint2;
      anExtrema.Points (anIdx, aPoint1, aPoint2);
      double aParam1 = 0.0;
      double aParam2 = 0.0;
      anExtrema.Parameters (anIdx, aParam1, aParam2);

      const gp_Pnt anXPoint (0.5 * (aPoint1.XYZ() + aPoint2.XYZ()));
      const gp_Vec aTan1 = TangentOf (aCurve1, aParam1);
      const gp_Vec aTan2 = TangentOf (aCurve2, aParam2);
      const gp_Vec aRefNormal = RefNormalOf (theData.PlaneNormal, aTan1, aTan2);

      const double aStep1 = StepOf (aCurve1);
      const double aStep2 = StepOf (aCurve2);
      const double aSide1Before = SideOf (aCurve1->Value (aParam1 - aStep1), aPoint2, aTan2, aRefNormal);
      const double aSide1After = SideOf (aCurve1->Value (aParam1 + aStep1), aPoint2, aTan2, aRefNormal);
      const double aSide2Before = SideOf (aCurve2->Value (aParam2 - aStep2), aPoint1, aTan1, aRefNormal);
      const double aSide2After = SideOf (aCurve2->Value (aParam2 + aStep2), aPoint1, aTan1, aRefNormal);

      theData.Param1.push_back (aParam1);
      theData.Param2.push_back (aParam2);
      theData.Points.push_back (anXPoint);
      theData.Config1.push_back (ConfigOf (aSide1Before, aSide1After, theData.Tol.equalPoint()));
      theData.Config2.push_back (ConfigOf (aSide2Before, aSide2After, theData.Tol.equalPoint()));
      theData.Tangential.push_back (IsTangentialAt (aCurve1, aParam1, aCurve2, aParam2));
      theData.PointTols.push_back (aGap);
    }
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Permutes the per-point results by the given index permutation.
void PermutePoints (NvGeCurveCurveInt3dData& theData, const std::vector<int>& theOrder)
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
  auto aPermutePnts = [theOrder] (std::vector<gp_Pnt>& theVec)
  {
    std::vector<gp_Pnt> aCopy = theVec;
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
  auto aPermuteFlg = [theOrder] (std::vector<Adesk::Boolean>& theVec)
  {
    std::vector<Adesk::Boolean> aCopy = theVec;
    for (size_t anIdx = 0; anIdx < theOrder.size(); ++anIdx)
    {
      theVec[anIdx] = aCopy[theOrder[anIdx]];
    }
  };
  aPermuteFlg (theData.Tangential);
  aPermute (theData.PointTols);
}

//! Fills a point-on-curve output wrapper through the documented layout
//! bridge (see gecint2d.cpp for the full rationale).
void FillPointOnCurve (NvGePointOnCurve3d& thePntOnCrv, const occ::handle<NvGeImpEntity3d>& theCurve,
                       double theParam)
{
  occ::handle<NvGePointOnCurve3dData> aData = new NvGePointOnCurve3dData();
  aData->Curve = theCurve;
  aData->Param = theParam;
  occ::handle<NvGeImpEntity3d> anImpl = new NvGeImpEntity3d (NvGe::kPointOnCurve3d, aData);
  anImpl->Ref();
  *reinterpret_cast<NvGeImpEntity3d**> (&thePntOnCrv) = anImpl.get();
  *reinterpret_cast<int*> (reinterpret_cast<char*> (&thePntOnCrv) + sizeof (NvGeImpEntity3d*)) = 1;
}

//! Validates the input curves and installs them into a fresh holder.
void PrepareData (NvGeCurveCurveInt3dData& theData, const NvGeCurve3d& theCurve1,
                  const NvGeCurve3d& theCurve2, const NvGeInterval& theRange1,
                  const NvGeInterval& theRange2, const NvGeVector3d& thePlaneNormal,
                  const NvGeTol& theTol)
{
  // Validates that both wrappers really carry 3d curve geometry.
  NvGeCurve3dOf (ImplOf (&theCurve1));
  NvGeCurve3dOf (ImplOf (&theCurve2));
  theData.Curve1 = ImplOf (&theCurve1);
  theData.Curve2 = ImplOf (&theCurve2);
  theData.Range1 = theRange1;
  theData.Range2 = theRange2;
  theData.PlaneNormal = thePlaneNormal;
  theData.Tol = theTol;
}

} // namespace

//=================================================================================================

NvGeCurveCurveInt3d::NvGeCurveCurveInt3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt3d,
                                  occ::handle<NvGeCurveCurveInt3dData> (new NvGeCurveCurveInt3dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt3d::NvGeCurveCurveInt3d (const NvGeCurve3d& theCurve1, const NvGeCurve3d& theCurve2,
                                          const NvGeVector3d& thePlaneNormal, const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeCurveCurveInt3dData> aData = new NvGeCurveCurveInt3dData();
  PrepareData (*aData, theCurve1, theCurve2, NvGeInterval(), NvGeInterval(), thePlaneNormal, theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt3d::NvGeCurveCurveInt3d (const NvGeCurve3d& theCurve1, const NvGeCurve3d& theCurve2,
                                          const NvGeInterval& theRange1, const NvGeInterval& theRange2,
                                          const NvGeVector3d& thePlaneNormal, const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeCurveCurveInt3dData> aData = new NvGeCurveCurveInt3dData();
  PrepareData (*aData, theCurve1, theCurve2, theRange1, theRange2, thePlaneNormal, theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveCurveInt3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveCurveInt3d::NvGeCurveCurveInt3d (const NvGeCurveCurveInt3d& theSource)
: NvGeEntity3d (theSource)
{
}

//=================================================================================================

void NvGeCurveCurveInt3d::getIntRanges (NvGeInterval& theRange1, NvGeInterval& theRange2) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  theRange1 = aData->Range1;
  theRange2 = aData->Range2;
}

//=================================================================================================

NvGeVector3d NvGeCurveCurveInt3d::planeNormal () const
{
  return DataOf (mpImpEnt)->PlaneNormal;
}

//=================================================================================================

NvGeTol NvGeCurveCurveInt3d::tolerance () const
{
  return DataOf (mpImpEnt)->Tol;
}

//=================================================================================================

int NvGeCurveCurveInt3d::numIntPoints () const
{
  return static_cast<int> (DataOf (mpImpEnt)->Points.size());
}

//=================================================================================================

NvGePoint3d NvGeCurveCurveInt3d::intPoint (int theIntNum) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "intPoint");
  const gp_Pnt& aPoint = aData->Points[theIntNum];
  return NvGePoint3d (aPoint.X(), aPoint.Y(), aPoint.Z());
}

//=================================================================================================

void NvGeCurveCurveInt3d::getIntParams (int theIntNum, double& theParam1, double& theParam2) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getIntParams");
  theParam1 = aData->Param1[theIntNum];
  theParam2 = aData->Param2[theIntNum];
}

//=================================================================================================

void NvGeCurveCurveInt3d::getPointOnCurve1 (int theIntNum, NvGePointOnCurve3d& thePntOnCrv) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getPointOnCurve1");
  FillPointOnCurve (thePntOnCrv, aData->Curve1, aData->Param1[theIntNum]);
}

//=================================================================================================

void NvGeCurveCurveInt3d::getPointOnCurve2 (int theIntNum, NvGePointOnCurve3d& thePntOnCrv) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getPointOnCurve2");
  FillPointOnCurve (thePntOnCrv, aData->Curve2, aData->Param2[theIntNum]);
}

//=================================================================================================

void NvGeCurveCurveInt3d::getIntConfigs (int theIntNum, NvGe::NvGeXConfig& theConfig1wrt2,
                                         NvGe::NvGeXConfig& theConfig2wrt1) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->Points.size()), "getIntConfigs");
  theConfig1wrt2 = aData->Config1[theIntNum];
  theConfig2wrt1 = aData->Config2[theIntNum];
}

//=================================================================================================

Adesk::Boolean NvGeCurveCurveInt3d::isTangential (int theIntNum) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  if (theIntNum < 0 || theIntNum >= static_cast<int> (aData->Tangential.size()))
  {
    return false; // predicates never throw
  }
  return aData->Tangential[theIntNum];
}

//=================================================================================================

Adesk::Boolean NvGeCurveCurveInt3d::isTransversal (int theIntNum) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  if (theIntNum < 0 || theIntNum >= static_cast<int> (aData->Tangential.size()))
  {
    return false; // predicates never throw
  }
  return !aData->Tangential[theIntNum];
}

//=================================================================================================

double NvGeCurveCurveInt3d::intPointTol (int theIntNum) const
{
  const NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  CheckIndex (theIntNum, static_cast<int> (aData->PointTols.size()), "intPointTol");
  return aData->PointTols[theIntNum];
}

//=================================================================================================

int NvGeCurveCurveInt3d::overlapCount () const
{
  // The extrema algorithm has no coincident-segment counterpart; overlapping
  // (collinear) curves report their common points through numIntPoints, but
  // no parameter-ranged overlap segments are produced.
  return 0;
}

//=================================================================================================

Adesk::Boolean NvGeCurveCurveInt3d::overlapDirection () const
{
  return false;
}

//=================================================================================================

void NvGeCurveCurveInt3d::getOverlapRanges (int theOverlapNum, NvGeInterval& theRange1,
                                            NvGeInterval& theRange2) const
{
  (void)theRange1;
  (void)theRange2;
  CheckIndex (theOverlapNum, 0, "getOverlapRanges");
}

//=================================================================================================

void NvGeCurveCurveInt3d::changeCurveOrder ()
{
  NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  std::swap (aData->Curve1, aData->Curve2);
  std::swap (aData->Range1, aData->Range2);
  std::swap (aData->Param1, aData->Param2);
  std::swap (aData->Config1, aData->Config2);
}

//=================================================================================================

NvGeCurveCurveInt3d& NvGeCurveCurveInt3d::orderWrt1 ()
{
  NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
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

NvGeCurveCurveInt3d& NvGeCurveCurveInt3d::orderWrt2 ()
{
  NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
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

NvGeCurveCurveInt3d& NvGeCurveCurveInt3d::set (const NvGeCurve3d& theCurve1,
                                               const NvGeCurve3d& theCurve2,
                                               const NvGeVector3d& thePlaneNormal,
                                               const NvGeTol& theTol)
{
  NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theCurve1, theCurve2, NvGeInterval(), NvGeInterval(), thePlaneNormal, theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt3d& NvGeCurveCurveInt3d::set (const NvGeCurve3d& theCurve1,
                                               const NvGeCurve3d& theCurve2,
                                               const NvGeInterval& theRange1,
                                               const NvGeInterval& theRange2,
                                               const NvGeVector3d& thePlaneNormal,
                                               const NvGeTol& theTol)
{
  NvGeCurveCurveInt3dData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theCurve1, theCurve2, theRange1, theRange2, thePlaneNormal, theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeCurveCurveInt3d& NvGeCurveCurveInt3d::operator = (const NvGeCurveCurveInt3d& theSource)
{
  NvGeEntity3d::operator= (theSource);
  return *this;
}
