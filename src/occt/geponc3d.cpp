// geponc3d.cpp - implementation of NvGePointOnCurve3d.
//
// The point stores an NvGePointOnCurve3dData holder (see geimpdata.h)
// carrying a private shallow clone of the curve entity impl plus the curve
// parameter. The holder is never mutated: setCurve / setParameter install
// a fresh holder behind a copy-on-write detached impl, so copies sharing
// the impl (through NvGeEntity3d::operator=) are never disturbed.
// Evaluation goes through the shared Geom_Curve of the bound curve entity;
// the parameter-taking evaluators reset the stored parameter first (ARX
// semantics of the non-const overloads).

#include <geponc3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <geent3d.h>
#include <geimpdata.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_Curve.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <string>

namespace
{

// Curvature guard: below this first-derivative magnitude the point is
// treated as singular and the curvature is not computable.
constexpr double THE_SINGULAR_TOL = 1e-12;

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Holder access; throws when the impl carries no point-on-curve data.
const NvGePointOnCurve3dData* DataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  if (theImp == nullptr)
  {
    throw NvException (std::string ("NvGePointOnCurve3d::") + theMethod
                       + "(): the entity has no implementation object");
  }
  const NvGePointOnCurve3dData* aData = NvGeDataOf<NvGePointOnCurve3dData> (theImp->Geom());
  if (aData == nullptr)
  {
    throw NvException (std::string ("NvGePointOnCurve3d::") + theMethod
                       + "(): the entity holds no point-on-curve data");
  }
  return aData;
}

//! Holder access with a bound curve; throws when the point is not bound.
const NvGePointOnCurve3dData* BoundDataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const NvGePointOnCurve3dData* aData = DataOf (theImp, theMethod);
  if (aData->Curve.IsNull())
  {
    throw NvException (std::string ("NvGePointOnCurve3d::") + theMethod
                       + "(): the point is not bound to a curve");
  }
  return aData;
}

//! Geom_Curve stored behind the bound curve entity.
occ::handle<Geom_Curve> CurveGeomOf (const NvGePointOnCurve3dData* theData)
{
  return NvGeCurve3dOf (theData->Curve.get());
}

//! Owning impl handle over the curve entity: clones the wrapper's impl and
//! releases the temporary copy wrapper, so the handle's reference is the
//! only owner of the clone (copy + delete refcount dance over the layout
//! bridge).
occ::handle<NvGeImpEntity3d> CurveImpOf (const NvGeCurve3d& theCurve)
{
  NvGeEntity3d* aCopy = theCurve.copy();
  occ::handle<NvGeImpEntity3d> anImp (ImplOf (aCopy));
  delete aCopy;
  NvGeCurve3dOf (anImp.get()); // validate early: rejects null / non-curve entities
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
  occ::handle<NvGePointOnCurve3dData> aData = new NvGePointOnCurve3dData();
  aData->Curve = theCurveImp;
  aData->Param = theParam;
  theImp->SetGeom (aData);
}

//! Derivative of the requested order (1, 2 or 3) at theParam.
NvGeVector3d EvaluateDeriv (const occ::handle<Geom_Curve>& theCurve, double theParam,
                            int theOrder, const char* theMethod)
{
  gp_Pnt aPnt;
  gp_Vec aV1;
  gp_Vec aV2;
  gp_Vec aV3;
  switch (theOrder)
  {
    case 1:
      theCurve->D1 (theParam, aPnt, aV1);
      return NvGeVector3d (aV1.X(), aV1.Y(), aV1.Z());
    case 2:
      theCurve->D2 (theParam, aPnt, aV1, aV2);
      return NvGeVector3d (aV2.X(), aV2.Y(), aV2.Z());
    case 3:
      theCurve->D3 (theParam, aPnt, aV1, aV2, aV3);
      return NvGeVector3d (aV3.X(), aV3.Y(), aV3.Z());
    default:
      throw NvException (std::string ("NvGePointOnCurve3d::") + theMethod
                         + "(): the derivative order must be 1, 2 or 3");
  }
}

//! Curvature |v1 x v2| / |v1|^3 at theParam; false at a singular point.
Nova::Boolean CurvatureAt (const occ::handle<Geom_Curve>& theCurve, double theParam,
                            double& theRes)
{
  gp_Pnt aPnt;
  gp_Vec aV1;
  gp_Vec aV2;
  theCurve->D2 (theParam, aPnt, aV1, aV2);
  const double aSpeed = aV1.Magnitude();
  if (aSpeed <= THE_SINGULAR_TOL)
  {
    return false;
  }
  theRes = std::sqrt (aV1.CrossSquareMagnitude (aV2)) / (aSpeed * aSpeed * aSpeed);
  return true;
}

//! Single-slot cache for the borrowed curve wrapper handed out by curve().
NvGeCurve3d*& CurveWrapperSlot()
{
  static NvGeCurve3d* THE_WRAPPER = nullptr;
  return THE_WRAPPER;
}

} // namespace

//=================================================================================================

NvGePointOnCurve3d::NvGePointOnCurve3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve3d,
                                  occ::handle<NvGePointOnCurve3dData> (new NvGePointOnCurve3dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve3d::NvGePointOnCurve3d (const NvGeCurve3d& theCurve)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnCurve3dData> aData = new NvGePointOnCurve3dData();
  aData->Curve = aCurveImp;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve3d::NvGePointOnCurve3d (const NvGeCurve3d& theCurve, double theParam)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnCurve3dData> aData = new NvGePointOnCurve3dData();
  aData->Curve = aCurveImp;
  aData->Param = theParam;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnCurve3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnCurve3d::NvGePointOnCurve3d (const NvGePointOnCurve3d& theSrc)
: NvGePointEnt3d (theSrc)
{
}

//=================================================================================================

NvGePointOnCurve3d& NvGePointOnCurve3d::operator = (const NvGePointOnCurve3d& theSrc)
{
  NvGePointEnt3d::operator = (theSrc);
  return *this;
}

//=================================================================================================

const NvGeCurve3d* NvGePointOnCurve3d::curve () const
{
  const NvGePointOnCurve3dData* aData = BoundDataOf (mpImpEnt, "curve");
  // Refresh the single-slot cache: the previous wrapper is released here.
  // The fresh base wrapper shares the bound curve impl (layout bridge: an
  // NvGeEntity3d is the prefix of NvGeCurve3d).
  delete CurveWrapperSlot();
  CurveWrapperSlot() = (NvGeCurve3d*) newEntity3d (aData->Curve.get());
  return CurveWrapperSlot();
}

//=================================================================================================

double NvGePointOnCurve3d::parameter () const
{
  return DataOf (mpImpEnt, "parameter")->Param;
}

//=================================================================================================

NvGePoint3d NvGePointOnCurve3d::point () const
{
  const NvGePointOnCurve3dData* aData = BoundDataOf (mpImpEnt, "point");
  const gp_Pnt aPnt = CurveGeomOf (aData)->Value (aData->Param);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGePointOnCurve3d::point (double theParam)
{
  setParameter (theParam);
  return point();
}

//=================================================================================================

NvGePoint3d NvGePointOnCurve3d::point (const NvGeCurve3d& theCurve, double theParam)
{
  setCurve (theCurve);
  return point (theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnCurve3d::deriv (int theOrder) const
{
  const NvGePointOnCurve3dData* aData = BoundDataOf (mpImpEnt, "deriv");
  return EvaluateDeriv (CurveGeomOf (aData), aData->Param, theOrder, "deriv");
}

//=================================================================================================

NvGeVector3d NvGePointOnCurve3d::deriv (int theOrder, double theParam)
{
  setParameter (theParam);
  return deriv (theOrder);
}

//=================================================================================================

NvGeVector3d NvGePointOnCurve3d::deriv (int theOrder, const NvGeCurve3d& theCurve, double theParam)
{
  setCurve (theCurve);
  return deriv (theOrder, theParam);
}

//=================================================================================================

Nova::Boolean NvGePointOnCurve3d::isSingular (const NvGeTol& theTol) const
{
  // Predicate: degenerate input reports false instead of throwing.
  if (mpImpEnt == nullptr)
  {
    return false;
  }
  const NvGePointOnCurve3dData* aData = NvGeDataOf<NvGePointOnCurve3dData> (mpImpEnt->Geom());
  if (aData == nullptr || aData->Curve.IsNull())
  {
    return false;
  }
  const occ::handle<Geom_Curve> aCurve = occ::down_cast<Geom_Curve> (aData->Curve->Geom());
  if (aCurve.IsNull())
  {
    return false;
  }
  gp_Pnt aPnt;
  gp_Vec aV1;
  aCurve->D1 (aData->Param, aPnt, aV1);
  return aV1.Magnitude() <= theTol.equalVector();
}

//=================================================================================================

Nova::Boolean NvGePointOnCurve3d::curvature (double& theRes)
{
  const NvGePointOnCurve3dData* aData = BoundDataOf (mpImpEnt, "curvature");
  return CurvatureAt (CurveGeomOf (aData), aData->Param, theRes);
}

//=================================================================================================

Nova::Boolean NvGePointOnCurve3d::curvature (double theParam, double& theRes)
{
  setParameter (theParam);
  return curvature (theRes);
}

//=================================================================================================

NvGePointOnCurve3d& NvGePointOnCurve3d::setCurve (const NvGeCurve3d& theCurve)
{
  const occ::handle<NvGeImpEntity3d> aCurveImp = CurveImpOf (theCurve);
  SetOnCurveData (mpImpEnt, aCurveImp, parameter());
  return *this;
}

//=================================================================================================

NvGePointOnCurve3d& NvGePointOnCurve3d::setParameter (double theParam)
{
  SetOnCurveData (mpImpEnt, DataOf (mpImpEnt, "setParameter")->Curve, theParam);
  return *this;
}
