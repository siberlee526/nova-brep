// geponc2d.cpp - implementation of NvGePointOnCurve2d.
//
// The point stores an NvGePointOnCurve2dData holder (see geimpdata.h)
// carrying a private shallow clone of the curve entity impl plus the curve
// parameter. The holder is never mutated: setCurve / setParameter install
// a fresh holder behind a copy-on-write detached impl, so copies sharing
// the impl (through NvGeEntity2d::operator=) are never disturbed.
// Evaluation goes through the shared Geom2d_Curve of the bound curve
// entity; the parameter-taking evaluators reset the stored parameter first
// (ARX semantics of the non-const overloads).

#include <geponc2d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <geent2d.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

#include <cmath>
#include <string>

namespace
{

// Curvature guard: below this first-derivative magnitude the point is
// treated as singular and the curvature is not computable.
constexpr double THE_SINGULAR_TOL = 1e-12;

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity2d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Holder access; throws when the impl carries no point-on-curve data.
const NvGePointOnCurve2dData* DataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  if (theImp == nullptr)
  {
    throw NvException (std::string ("NvGePointOnCurve2d::") + theMethod
                       + "(): the entity has no implementation object");
  }
  const NvGePointOnCurve2dData* aData = NvGeDataOf<NvGePointOnCurve2dData> (theImp->Geom());
  if (aData == nullptr)
  {
    throw NvException (std::string ("NvGePointOnCurve2d::") + theMethod
                       + "(): the entity holds no point-on-curve data");
  }
  return aData;
}

//! Holder access with a bound curve; throws when the point is not bound.
const NvGePointOnCurve2dData* BoundDataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const NvGePointOnCurve2dData* aData = DataOf (theImp, theMethod);
  if (aData->Curve.IsNull())
  {
    throw NvException (std::string ("NvGePointOnCurve2d::") + theMethod
                       + "(): the point is not bound to a curve");
  }
  return aData;
}

//! Geom2d_Curve stored behind the bound curve entity.
occ::handle<Geom2d_Curve> CurveGeomOf (const NvGePointOnCurve2dData* theData)
{
  return NvGeCurve2dOf (theData->Curve.get());
}

//! Owning impl handle over the curve entity: clones the wrapper's impl and
//! releases the temporary copy wrapper, so the handle's reference is the
//! only owner of the clone (copy + delete refcount dance over the layout
//! bridge).
occ::handle<NvGeImpEntity3d> CurveImpOf (const NvGeCurve2d& theCurve)
{
  NvGeEntity2d* aCopy = theCurve.copy();
  occ::handle<NvGeImpEntity3d> anImp (ImplOf (aCopy));
  delete aCopy;
  NvGeCurve2dOf (anImp.get()); // validate early: rejects null / non-curve entities
  return anImp;
}

//! Detaches the impl when shared and installs a fresh holder carrying
//! theCurveImp / theParam (copy-on-write; see the banner of geimpdata.h).
void SetOnCurveData (NvGeImpEntity3d*& theImp, const occ::handle<NvGeImpEntity3d>& theCurveImp,
                     double theParam)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  occ::handle<NvGePointOnCurve2dData> aData = new NvGePointOnCurve2dData();
  aData->Curve = theCurveImp;
  aData->Param = theParam;
  theImp->SetGeom (aData);
}

//! Derivative of the requested order (1, 2 or 3) at theParam.
NvGeVector2d EvaluateDeriv (const occ::handle<Geom2d_Curve>& theCurve, double theParam,
                            int theOrder, const char* theMethod)
{
  gp_Pnt2d aPnt;
  gp_Vec2d aV1;
  gp_Vec2d aV2;
  gp_Vec2d aV3;
  switch (theOrder)
  {
    case 1:
      theCurve->D1 (theParam, aPnt, aV1);
      return NvGeVector2d (aV1.X(), aV1.Y());
    case 2:
      theCurve->D2 (theParam, aPnt, aV1, aV2);
      return NvGeVector2d (aV2.X(), aV2.Y());
    case 3:
      theCurve->D3 (theParam, aPnt, aV1, aV2, aV3);
      return NvGeVector2d (aV3.X(), aV3.Y());
    default:
      throw NvException (std::string ("NvGePointOnCurve2d::") + theMethod
                         + "(): the derivative order must be 1, 2 or 3");
  }
}

//! Curvature |v1 x v2| / |v1|^3 at theParam; false at a singular point.
Adesk::Boolean CurvatureAt (const occ::handle<Geom2d_Curve>& theCurve, double theParam,
                            double& theRes)
{
  gp_Pnt2d aPnt;
  gp_Vec2d aV1;
  gp_Vec2d aV2;
  theCurve->D2 (theParam, aPnt, aV1, aV2);
  const double aSpeed = aV1.Magnitude();
  if (aSpeed <= THE_SINGULAR_TOL)
  {
    return false;
  }
  theRes = std::abs (aV1.X() * aV2.Y() - aV1.Y() * aV2.X()) / (aSpeed * aSpeed * aSpeed);
  return true;
}

//! Single-slot cache for the borrowed curve wrapper handed out by curve().
NvGeCurve2d*& CurveWrapperSlot()
{
  static NvGeCurve2d* THE_WRAPPER = nullptr;
  return THE_WRAPPER;
}

} // namespace

//=================================================================================================

NvGePointOnCurve2d::NvGePointOnCurve2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve2d,
                                  occ::handle<NvGePointOnCurve2dData> (new NvGePointOnCurve2dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve2d::NvGePointOnCurve2d (const NvGeCurve2d& theCurve)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnCurve2dData> aData = new NvGePointOnCurve2dData();
  aData->Curve = aCurveImp;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve2d::NvGePointOnCurve2d (const NvGeCurve2d& theCurve, double theParam)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnCurve2dData> aData = new NvGePointOnCurve2dData();
  aData->Curve = aCurveImp;
  aData->Param = theParam;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve2d::NvGePointOnCurve2d (const NvGePointOnCurve2d& theSrc)
: NvGePointEnt2d (theSrc)
{
}

//=================================================================================================

NvGePointOnCurve2d& NvGePointOnCurve2d::operator = (const NvGePointOnCurve2d& theSrc)
{
  NvGePointEnt2d::operator = (theSrc);
  return *this;
}

//=================================================================================================

const NvGeCurve2d* NvGePointOnCurve2d::curve () const
{
  const NvGePointOnCurve2dData* aData = BoundDataOf (mpImpEnt, "curve");
  // Refresh the single-slot cache: the previous wrapper is released here.
  // The fresh base wrapper shares the bound curve impl (layout bridge: an
  // NvGeEntity2d is the prefix of NvGeCurve2d).
  delete CurveWrapperSlot();
  CurveWrapperSlot() = (NvGeCurve2d*) newEntity2d (aData->Curve.get());
  return CurveWrapperSlot();
}

//=================================================================================================

double NvGePointOnCurve2d::parameter () const
{
  return DataOf (mpImpEnt, "parameter")->Param;
}

//=================================================================================================

NvGePoint2d NvGePointOnCurve2d::point () const
{
  const NvGePointOnCurve2dData* aData = BoundDataOf (mpImpEnt, "point");
  const gp_Pnt2d aPnt = CurveGeomOf (aData)->Value (aData->Param);
  return NvGePoint2d (aPnt.X(), aPnt.Y());
}

//=================================================================================================

NvGePoint2d NvGePointOnCurve2d::point (double theParam)
{
  setParameter (theParam);
  return point();
}

//=================================================================================================

NvGePoint2d NvGePointOnCurve2d::point (const NvGeCurve2d& theCurve, double theParam)
{
  setCurve (theCurve);
  return point (theParam);
}

//=================================================================================================

NvGeVector2d NvGePointOnCurve2d::deriv (int theOrder) const
{
  const NvGePointOnCurve2dData* aData = BoundDataOf (mpImpEnt, "deriv");
  return EvaluateDeriv (CurveGeomOf (aData), aData->Param, theOrder, "deriv");
}

//=================================================================================================

NvGeVector2d NvGePointOnCurve2d::deriv (int theOrder, double theParam)
{
  setParameter (theParam);
  return deriv (theOrder);
}

//=================================================================================================

NvGeVector2d NvGePointOnCurve2d::deriv (int theOrder, const NvGeCurve2d& theCurve, double theParam)
{
  setCurve (theCurve);
  return deriv (theOrder, theParam);
}

//=================================================================================================

Adesk::Boolean NvGePointOnCurve2d::isSingular (const NvGeTol& theTol) const
{
  // Predicate: degenerate input reports false instead of throwing.
  if (mpImpEnt == nullptr)
  {
    return false;
  }
  const NvGePointOnCurve2dData* aData = NvGeDataOf<NvGePointOnCurve2dData> (mpImpEnt->Geom());
  if (aData == nullptr || aData->Curve.IsNull())
  {
    return false;
  }
  const occ::handle<Geom2d_Curve> aCurve = occ::down_cast<Geom2d_Curve> (aData->Curve->Geom());
  if (aCurve.IsNull())
  {
    return false;
  }
  gp_Pnt2d aPnt;
  gp_Vec2d aV1;
  aCurve->D1 (aData->Param, aPnt, aV1);
  return aV1.Magnitude() <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGePointOnCurve2d::curvature (double& theRes)
{
  const NvGePointOnCurve2dData* aData = BoundDataOf (mpImpEnt, "curvature");
  return CurvatureAt (CurveGeomOf (aData), aData->Param, theRes);
}

//=================================================================================================

Adesk::Boolean NvGePointOnCurve2d::curvature (double theParam, double& theRes)
{
  setParameter (theParam);
  return curvature (theRes);
}

//=================================================================================================

NvGePointOnCurve2d& NvGePointOnCurve2d::setCurve (const NvGeCurve2d& theCurve)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  SetOnCurveData (mpImpEnt, aCurveImp, parameter());
  return *this;
}

//=================================================================================================

NvGePointOnCurve2d& NvGePointOnCurve2d::setParameter (double theParam)
{
  SetOnCurveData (mpImpEnt, DataOf (mpImpEnt, "setParameter")->Curve, theParam);
  return *this;
}
