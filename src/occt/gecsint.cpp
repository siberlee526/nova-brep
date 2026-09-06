// gecsint.cpp - implementation of NvGeCurveSurfInt.
//
// The intersection entity is an empty NvGeEntity3d shell whose impl carries
// a file-local NvGeCurveSurfIntData holder: the input curve / surface impls,
// the tolerance and the eager intersection results. The constructor and
// set() run GeomAPI_IntCS eagerly and fill the holder; every query only
// reads it and reports failures through the NvGeIntersectError channel
// (ARX semantics for this class), not through exceptions. The csiConfig of
// each intersection is classified by sampling the curve slightly before /
// after the intersection parameter and projecting the samples onto the
// surface: the side of the projected sample with respect to the surface
// normal (outside along the normal / inside against it) selects kXOut/kXIn,
// with the kXTan* variants when a sample lies on the surface itself.

#include <gecsint.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <geponc3d.h>
#include <geponsrf.h>
#include <gesurf.h>
#include <getol.h>

#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
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

//! Per-point classification of the curve/surface intersection.
class NvGeCurveSurfIntPoint
{
public:

  double              W = 0.0;                 // parameter on the curve
  gp_Pnt              Point;                   // intersection point
  gp_Pnt2d            SurfParam;               // (u, v) on the surface
  NvGe::csiConfig     Lower = NvGe::kXUnknown; // neighborhood below on the curve
  NvGe::csiConfig     Higher = NvGe::kXUnknown; // neighborhood above on the curve
  Nova::Boolean      SmallAngle = false;      // grazing (near-tangent) crossing
};

//! Storage behind NvGeCurveSurfInt: inputs and eager results.
class NvGeCurveSurfIntData : public NvGeEntityData
{
public:

  occ::handle<NvGeImpEntity3d> Curve;
  occ::handle<NvGeImpEntity3d> Surface;
  NvGeTol                      Tol;
  bool                         Failed = false; // algorithm did not complete

  std::vector<NvGeCurveSurfIntPoint> Points;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeCurveSurfIntData> aCopy = new NvGeCurveSurfIntData();
    aCopy->Curve = Curve;
    aCopy->Surface = Surface;
    aCopy->Tol = Tol;
    aCopy->Failed = Failed;
    aCopy->Points = Points;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeCurveSurfIntData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeCurveSurfIntData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeCurveSurfIntData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeCurveSurfIntData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeCurveSurfIntData> (theImp->Geom());
}

//! Validates the error channel and the query index; returns false when the
//! query must not read the holder (the error code is set).
bool CheckQuery (const NvGeCurveSurfIntData* theData, int theIntNum,
                 NvGeIntersectError& theErr)
{
  if (theData->Failed)
  {
    theErr = NvGe::kXXUnknown;
    return false;
  }
  if (theIntNum < 0 || theIntNum >= static_cast<int> (theData->Points.size()))
  {
    theErr = NvGe::kXXIndexOutOfRange;
    return false;
  }
  theErr = NvGe::kXXOk;
  return true;
}

//! Maps an OCCT parametric bound to a plain double.
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

//! Parameter step used to sample the curve around an intersection point.
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

//! Signed distance of a point from the surface (> 0 outside along the
//! normal, < 0 inside); 0 when the side cannot be determined.
double SignedSideOf (const occ::handle<Geom_Surface>& theSurface, const gp_Pnt& theSample,
                     const NvGeTol& theTol)
{
  GeomAPI_ProjectPointOnSurf aProjector (theSample, theSurface);
  if (aProjector.NbPoints() == 0)
  {
    return 0.0;
  }
  double anU = 0.0;
  double aV = 0.0;
  aProjector.LowerDistanceParameters (anU, aV);
  GeomLProp_SLProps aProps (theSurface, anU, aV, 1, theTol.equalPoint());
  if (!aProps.IsNormalDefined())
  {
    return 0.0;
  }
  gp_Vec aRel (theSample.XYZ() - aProjector.NearestPoint().XYZ());
  // gp_Vec::Dot takes gp_Vec only; the normal comes back as a gp_Dir.
  return aRel.XYZ().Dot (aProps.Normal().XYZ());
}

//! Configuration of one curve neighborhood side; the tangent variants apply
//! when the sample lies on the surface, classified by the opposite side.
NvGe::csiConfig ConfigSideOf (double theSigned, double theOtherSigned, double theTol)
{
  if (theSigned > theTol)
  {
    return NvGe::kXOut;
  }
  if (theSigned < -theTol)
  {
    return NvGe::kXIn;
  }
  if (theOtherSigned > theTol)
  {
    return NvGe::kXTanOut;
  }
  if (theOtherSigned < -theTol)
  {
    return NvGe::kXTanIn;
  }
  return NvGe::kXCoincidentUnbounded;
}

//! Classifies one intersection: side of the curve neighborhoods below and
//! above the intersection parameter plus the grazing-angle flag.
NvGeCurveSurfIntPoint ClassifyPoint (const occ::handle<Geom_Curve>& theCurve,
                                     const occ::handle<Geom_Surface>& theSurface,
                                     double theW, const gp_Pnt& thePoint, double theU, double theV,
                                     const NvGeTol& theTol)
{
  NvGeCurveSurfIntPoint aResult;
  aResult.W = theW;
  aResult.Point = thePoint;
  aResult.SurfParam = gp_Pnt2d (theU, theV);

  const double aStep = StepOf (theCurve);
  const double aBefore = SignedSideOf (theSurface, theCurve->Value (theW - aStep), theTol);
  const double anAfter = SignedSideOf (theSurface, theCurve->Value (theW + aStep), theTol);
  aResult.Lower = ConfigSideOf (aBefore, anAfter, theTol.equalPoint());
  aResult.Higher = ConfigSideOf (anAfter, aBefore, theTol.equalPoint());

  // Grazing crossing: small angle between the curve tangent and the
  // surface tangent plane, i.e. |sin(angle)| = |dot(T_unit, N_unit)|.
  const gp_Vec aTangent = theCurve->DN (theW, 1);
  const double aTanLen = aTangent.Magnitude();
  GeomLProp_SLProps aProps (theSurface, theU, theV, 1, theTol.equalPoint());
  if (aTanLen > 1e-12 && aProps.IsNormalDefined())
  {
    // gp_Vec::Dot takes gp_Vec only; the normal comes back as a gp_Dir.
    const double aSinAngle = std::abs (aTangent.XYZ().Dot (aProps.Normal().XYZ())) / aTanLen;
    aResult.SmallAngle = aSinAngle < 0.05;
  }
  return aResult;
}

//! Runs the eager intersection computation and fills the holder.
void ComputeInto (NvGeCurveSurfIntData& theData)
{
  theData.Failed = false;
  theData.Points.clear();

  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (theData.Curve.get());
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (theData.Surface.get());
  try
  {
    GeomAPI_IntCS anInter (aCurve, aSurface);
    if (!anInter.IsDone())
    {
      theData.Failed = true;
      return;
    }
    const int aNbPoints = anInter.NbPoints();
    theData.Points.reserve (aNbPoints);
    for (int anIdx = 1; anIdx <= aNbPoints; ++anIdx)
    {
      double anU = 0.0;
      double aV = 0.0;
      double aW = 0.0;
      anInter.Parameters (anIdx, anU, aV, aW);
      theData.Points.push_back (ClassifyPoint (aCurve, aSurface, aW, anInter.Point (anIdx),
                                               anU, aV, theData.Tol));
    }
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Fills a point-on-curve output wrapper through the documented layout
//! bridge: the point entities are empty shells around the mpImpEnt /
//! mDelEnt pair (mpImpEnt first, pack 8). The previous slot value is
//! overwritten without unref - the point constructors leave the slot
//! unowned, so it can never be released from here.
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

//! Fills a point-on-surface output wrapper (same layout bridge).
void FillPointOnSurface (NvGePointOnSurface& thePntOnSrf,
                         const occ::handle<NvGeImpEntity3d>& theSurface, double theU, double theV)
{
  occ::handle<NvGePointOnSurfaceData> aData = new NvGePointOnSurfaceData();
  aData->Surface = theSurface;
  aData->U = theU;
  aData->V = theV;
  occ::handle<NvGeImpEntity3d> anImpl = new NvGeImpEntity3d (NvGe::kPointOnSurface, aData);
  anImpl->Ref();
  *reinterpret_cast<NvGeImpEntity3d**> (&thePntOnSrf) = anImpl.get();
  *reinterpret_cast<int*> (reinterpret_cast<char*> (&thePntOnSrf) + sizeof (NvGeImpEntity3d*)) = 1;
}

//! Validates the inputs and installs them into the holder.
void PrepareData (NvGeCurveSurfIntData& theData, const NvGeCurve3d& theCurve,
                  const NvGeSurface& theSurface, const NvGeTol& theTol)
{
  // Validates that both wrappers really carry the expected geometry.
  NvGeCurve3dOf (ImplOf (&theCurve));
  NvGeSurfaceOf (ImplOf (&theSurface));
  theData.Curve = ImplOf (&theCurve);
  theData.Surface = ImplOf (&theSurface);
  theData.Tol = theTol;
}

} // namespace

//=================================================================================================

NvGeCurveSurfInt::NvGeCurveSurfInt ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveSurfaceInt,
                                  occ::handle<NvGeCurveSurfIntData> (new NvGeCurveSurfIntData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveSurfInt::NvGeCurveSurfInt (const NvGeCurve3d& theCrv, const NvGeSurface& theSrf,
                                    const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeCurveSurfIntData> aData = new NvGeCurveSurfIntData();
  PrepareData (*aData, theCrv, theSrf, theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCurveSurfaceInt, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCurveSurfInt::NvGeCurveSurfInt (const NvGeCurveSurfInt& theSource)
: NvGeEntity3d (theSource)
{
}

//=================================================================================================

NvGeTol NvGeCurveSurfInt::tolerance () const
{
  return DataOf (mpImpEnt)->Tol;
}

//=================================================================================================

int NvGeCurveSurfInt::numIntPoints (NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, 0, theErr))
  {
    return 0;
  }
  return static_cast<int> (aData->Points.size());
}

//=================================================================================================

NvGePoint3d NvGeCurveSurfInt::intPoint (int theIntNum, NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return NvGePoint3d();
  }
  const gp_Pnt& aPoint = aData->Points[theIntNum].Point;
  return NvGePoint3d (aPoint.X(), aPoint.Y(), aPoint.Z());
}

//=================================================================================================

void NvGeCurveSurfInt::getIntParams (int theIntNum, double& theParam1, NvGePoint2d& theParam2,
                                     NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    theParam1 = 0.0;
    theParam2 = NvGePoint2d();
    return;
  }
  const NvGeCurveSurfIntPoint& anEntry = aData->Points[theIntNum];
  theParam1 = anEntry.W;
  theParam2 = NvGePoint2d (anEntry.SurfParam.X(), anEntry.SurfParam.Y());
}

//=================================================================================================

void NvGeCurveSurfInt::getPointOnCurve (int theIntNum, NvGePointOnCurve3d& thePntOnCrv,
                                        NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return;
  }
  FillPointOnCurve (thePntOnCrv, aData->Curve, aData->Points[theIntNum].W);
}

//=================================================================================================

void NvGeCurveSurfInt::getPointOnSurface (int theIntNum, NvGePointOnSurface& thePntOnSrf,
                                          NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return;
  }
  const NvGeCurveSurfIntPoint& anEntry = aData->Points[theIntNum];
  FillPointOnSurface (thePntOnSrf, aData->Surface, anEntry.SurfParam.X(), anEntry.SurfParam.Y());
}

//=================================================================================================

void NvGeCurveSurfInt::getIntConfigs (int theIntNum, NvGe::csiConfig& theLower,
                                      NvGe::csiConfig& theHigher, Nova::Boolean& theSmallAngle,
                                      NvGeIntersectError& theErr) const
{
  const NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return;
  }
  const NvGeCurveSurfIntPoint& anEntry = aData->Points[theIntNum];
  theLower = anEntry.Lower;
  theHigher = anEntry.Higher;
  theSmallAngle = anEntry.SmallAngle;
}

//=================================================================================================

NvGeCurveSurfInt& NvGeCurveSurfInt::set (const NvGeCurve3d& theCrv, const NvGeSurface& theSrf,
                                         const NvGeTol& theTol)
{
  NvGeCurveSurfIntData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theCrv, theSrf, theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeCurveSurfInt& NvGeCurveSurfInt::operator = (const NvGeCurveSurfInt& theSource)
{
  NvGeEntity3d::operator= (theSource);
  return *this;
}
