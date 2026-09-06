// geponsrf.cpp - implementation of NvGePointOnSurface.
//
// The point stores an NvGePointOnSurfaceData holder (see geimpdata.h)
// carrying a private shallow clone of the surface entity impl plus the
// (u, v) parameter. The holder is never mutated: setSurface / setParameter
// install a fresh holder behind a copy-on-write detached impl, so copies
// sharing the impl (through NvGeEntity3d::operator=) are never disturbed.
// Evaluation goes through the shared Geom_Surface of the bound surface
// entity; the parameter-taking evaluators reset the stored parameter first
// (ARX semantics of the non-const overloads).

#include <geponsrf.h>

#include <NvException.h>
#include <geent3d.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <gesurf.h>
#include <gevec2d.h>
#include <gevec3d.h>

#include <Geom_Surface.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <string>

namespace
{

// Singular point guard for normals and tangent inversion.
constexpr double THE_SINGULAR_TOL = 1e-12;

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Holder access; throws when the impl carries no point-on-surface data.
const NvGePointOnSurfaceData* DataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  if (theImp == nullptr)
  {
    throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                       + "(): the entity has no implementation object");
  }
  const NvGePointOnSurfaceData* aData = NvGeDataOf<NvGePointOnSurfaceData> (theImp->Geom());
  if (aData == nullptr)
  {
    throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                       + "(): the entity holds no point-on-surface data");
  }
  return aData;
}

//! Holder access with a bound surface; throws when the point is not bound.
const NvGePointOnSurfaceData* BoundDataOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const NvGePointOnSurfaceData* aData = DataOf (theImp, theMethod);
  if (aData->Surface.IsNull())
  {
    throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                       + "(): the point is not bound to a surface");
  }
  return aData;
}

//! Geom_Surface stored behind the bound surface entity.
occ::handle<Geom_Surface> SurfaceGeomOf (const NvGePointOnSurfaceData* theData)
{
  return NvGeSurfaceOf (theData->Surface.get());
}

//! Owning impl handle over the surface entity: clones the wrapper's impl
//! and releases the temporary copy wrapper, so the handle's reference is
//! the only owner of the clone (copy + delete refcount dance over the
//! layout bridge).
occ::handle<NvGeImpEntity3d> SurfaceImpOf (const NvGeSurface& theSurface)
{
  NvGeEntity3d* aCopy = theSurface.copy();
  occ::handle<NvGeImpEntity3d> anImp (ImplOf (aCopy));
  delete aCopy;
  NvGeSurfaceOf (anImp.get()); // validate early: rejects null / non-surface entities
  return anImp;
}

//! Detaches the impl when shared and installs a fresh holder carrying
//! theSurfaceImp / theU / theV (copy-on-write; see the banner of geimpdata.h).
void SetOnSurfaceData (NvGeImpEntity3d*& theImp, const occ::handle<NvGeImpEntity3d>& theSurfaceImp,
                       double theU, double theV)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  occ::handle<NvGePointOnSurfaceData> aData = new NvGePointOnSurfaceData();
  aData->Surface = theSurfaceImp;
  aData->U = theU;
  aData->V = theV;
  theImp->SetGeom (aData);
}

//! Unit normal from the cross product of the first partials; throws at a
//! singular point where the partials are parallel.
NvGeVector3d NormalOf (const occ::handle<Geom_Surface>& theSurface, double theU, double theV,
                       const char* theMethod)
{
  gp_Pnt aPnt;
  gp_Vec aD1U;
  gp_Vec aD1V;
  theSurface->D1 (theU, theV, aPnt, aD1U, aD1V);
  const gp_Vec aCross = aD1U.Crossed (aD1V);
  const double aMag = aCross.Magnitude();
  if (aMag <= THE_SINGULAR_TOL)
  {
    throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                       + "(): the normal is undefined at a singular point");
  }
  return NvGeVector3d (aCross.X() / aMag, aCross.Y() / aMag, aCross.Z() / aMag);
}

//! Partial derivative along u (theAlongV = false) or v; theOrder is 1 or 2.
NvGeVector3d PartialOf (const occ::handle<Geom_Surface>& theSurface, double theU, double theV,
                        int theOrder, bool theAlongV, const char* theMethod)
{
  gp_Pnt aPnt;
  gp_Vec aD1U;
  gp_Vec aD1V;
  gp_Vec aD2U;
  gp_Vec aD2V;
  gp_Vec aD2UV;
  switch (theOrder)
  {
    case 1:
      theSurface->D1 (theU, theV, aPnt, aD1U, aD1V);
      return theAlongV ? NvGeVector3d (aD1V.X(), aD1V.Y(), aD1V.Z())
                       : NvGeVector3d (aD1U.X(), aD1U.Y(), aD1U.Z());
    case 2:
      theSurface->D2 (theU, theV, aPnt, aD1U, aD1V, aD2U, aD2V, aD2UV);
      return theAlongV ? NvGeVector3d (aD2V.X(), aD2V.Y(), aD2V.Z())
                       : NvGeVector3d (aD2U.X(), aD2U.Y(), aD2U.Z());
    default:
      throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                         + "(): the derivative order must be 1 or 2");
  }
}

//! Mixed partial d2S/du dv from the second order evaluation.
NvGeVector3d MixedPartialOf (const occ::handle<Geom_Surface>& theSurface, double theU, double theV)
{
  gp_Pnt aPnt;
  gp_Vec aD1U;
  gp_Vec aD1V;
  gp_Vec aD2U;
  gp_Vec aD2V;
  gp_Vec aD2UV;
  theSurface->D2 (theU, theV, aPnt, aD1U, aD1V, aD2U, aD2V, aD2UV);
  return NvGeVector3d (aD2UV.X(), aD2UV.Y(), aD2UV.Z());
}

//! Directional derivative dS/dt = Su * theDir.x + Sv * theDir.y.
NvGeVector3d TangentOf (const occ::handle<Geom_Surface>& theSurface, double theU, double theV,
                        const NvGeVector2d& theDir)
{
  gp_Pnt aPnt;
  gp_Vec aD1U;
  gp_Vec aD1V;
  theSurface->D1 (theU, theV, aPnt, aD1U, aD1V);
  const gp_Vec aTangent = aD1U.Multiplied (theDir.x) + aD1V.Multiplied (theDir.y);
  return NvGeVector3d (aTangent.X(), aTangent.Y(), aTangent.Z());
}

//! Solves Su * du + Sv * dv = theVec in the least squares sense (Gram
//! system); throws when the partials are parallel (singular point).
NvGeVector2d InverseTangentOf (const occ::handle<Geom_Surface>& theSurface, double theU,
                               double theV, const NvGeVector3d& theVec, const char* theMethod)
{
  gp_Pnt aPnt;
  gp_Vec aD1U;
  gp_Vec aD1V;
  theSurface->D1 (theU, theV, aPnt, aD1U, aD1V);
  const gp_Vec aTarget (theVec.x, theVec.y, theVec.z);
  const double anAA = aD1U.Dot (aD1U);
  const double anAB = aD1U.Dot (aD1V);
  const double anBB = aD1V.Dot (aD1V);
  const double aRU = aD1U.Dot (aTarget);
  const double aRV = aD1V.Dot (aTarget);
  const double aDet = anAA * anBB - anAB * anAB;
  if (std::abs (aDet) <= THE_SINGULAR_TOL * THE_SINGULAR_TOL)
  {
    throw NvException (std::string ("NvGePointOnSurface::") + theMethod
                       + "(): the tangent vector cannot be inverted at a singular point");
  }
  return NvGeVector2d ((anBB * aRU - anAB * aRV) / aDet,
                       (anAA * aRV - anAB * aRU) / aDet);
}

//! Single-slot cache for the borrowed surface wrapper handed out by surface().
NvGeSurface*& SurfaceWrapperSlot()
{
  static NvGeSurface* THE_WRAPPER = nullptr;
  return THE_WRAPPER;
}

} // namespace

//=================================================================================================

NvGePointOnSurface::NvGePointOnSurface ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnSurface,
                                  occ::handle<NvGePointOnSurfaceData> (new NvGePointOnSurfaceData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnSurface::NvGePointOnSurface (const NvGeSurface& theSurface)
{
  const occ::handle<NvGeImpEntity3d> aSurfaceImp = SurfaceImpOf (theSurface);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnSurfaceData> aData = new NvGePointOnSurfaceData();
  aData->Surface = aSurfaceImp;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnSurface, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnSurface::NvGePointOnSurface (const NvGeSurface& theSurface, const NvGePoint2d& theParam)
{
  const occ::handle<NvGeImpEntity3d> aSurfaceImp = SurfaceImpOf (theSurface);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePointOnSurfaceData> aData = new NvGePointOnSurfaceData();
  aData->Surface = aSurfaceImp;
  aData->U = theParam.x;
  aData->V = theParam.y;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPointOnSurface, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePointOnSurface::NvGePointOnSurface (const NvGePointOnSurface& theSrc)
: NvGePointEnt3d (theSrc)
{
}

//=================================================================================================

NvGePointOnSurface& NvGePointOnSurface::operator = (const NvGePointOnSurface& theSrc)
{
  NvGePointEnt3d::operator = (theSrc);
  return *this;
}

//=================================================================================================

const NvGeSurface* NvGePointOnSurface::surface () const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "surface");
  // Refresh the single-slot cache: the previous wrapper is released here.
  // The fresh base wrapper shares the bound surface impl (layout bridge:
  // an NvGeEntity3d is the prefix of NvGeSurface).
  delete SurfaceWrapperSlot();
  SurfaceWrapperSlot() = (NvGeSurface*) newEntity3d (aData->Surface.get());
  return SurfaceWrapperSlot();
}

//=================================================================================================

NvGePoint2d NvGePointOnSurface::parameter () const
{
  const NvGePointOnSurfaceData* aData = DataOf (mpImpEnt, "parameter");
  return NvGePoint2d (aData->U, aData->V);
}

//=================================================================================================

NvGePoint3d NvGePointOnSurface::point () const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "point");
  const gp_Pnt aPnt = SurfaceGeomOf (aData)->Value (aData->U, aData->V);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=================================================================================================

NvGePoint3d NvGePointOnSurface::point (const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return point();
}

//=================================================================================================

NvGePoint3d NvGePointOnSurface::point (const NvGeSurface& theSurface, const NvGePoint2d& theParam)
{
  setSurface (theSurface);
  return point (theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::normal () const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "normal");
  return NormalOf (SurfaceGeomOf (aData), aData->U, aData->V, "normal");
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::normal (const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return normal();
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::normal (const NvGeSurface& theSurface, const NvGePoint2d& theParam)
{
  setSurface (theSurface);
  return normal (theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::uDeriv (int theOrder) const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "uDeriv");
  return PartialOf (SurfaceGeomOf (aData), aData->U, aData->V, theOrder, false, "uDeriv");
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::uDeriv (int theOrder, const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return uDeriv (theOrder);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::uDeriv (int theOrder, const NvGeSurface& theSurface,
                                         const NvGePoint2d& theParam)
{
  setSurface (theSurface);
  return uDeriv (theOrder, theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::vDeriv (int theOrder) const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "vDeriv");
  return PartialOf (SurfaceGeomOf (aData), aData->U, aData->V, theOrder, true, "vDeriv");
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::vDeriv (int theOrder, const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return vDeriv (theOrder);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::vDeriv (int theOrder, const NvGeSurface& theSurface,
                                         const NvGePoint2d& theParam)
{
  setSurface (theSurface);
  return vDeriv (theOrder, theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::mixedPartial () const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "mixedPartial");
  return MixedPartialOf (SurfaceGeomOf (aData), aData->U, aData->V);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::mixedPartial (const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return mixedPartial();
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::mixedPartial (const NvGeSurface& theSurface,
                                               const NvGePoint2d& theParam)
{
  setSurface (theSurface);
  return mixedPartial (theParam);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::tangentVector (const NvGeVector2d& theVec) const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "tangentVector");
  return TangentOf (SurfaceGeomOf (aData), aData->U, aData->V, theVec);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::tangentVector (const NvGeVector2d& theVec,
                                                const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return tangentVector (theVec);
}

//=================================================================================================

NvGeVector3d NvGePointOnSurface::tangentVector (const NvGeVector2d& theVec,
                                                const NvGeSurface& theSurf,
                                                const NvGePoint2d& theParam)
{
  setSurface (theSurf);
  return tangentVector (theVec, theParam);
}

//=================================================================================================

NvGeVector2d NvGePointOnSurface::inverseTangentVector (const NvGeVector3d& theVec) const
{
  const NvGePointOnSurfaceData* aData = BoundDataOf (mpImpEnt, "inverseTangentVector");
  return InverseTangentOf (SurfaceGeomOf (aData), aData->U, aData->V, theVec,
                           "inverseTangentVector");
}

//=================================================================================================

NvGeVector2d NvGePointOnSurface::inverseTangentVector (const NvGeVector3d& theVec,
                                                       const NvGePoint2d& theParam)
{
  setParameter (theParam);
  return inverseTangentVector (theVec);
}

//=================================================================================================

NvGeVector2d NvGePointOnSurface::inverseTangentVector (const NvGeVector3d& theVec,
                                                       const NvGeSurface& theSurf,
                                                       const NvGePoint2d& theParam)
{
  setSurface (theSurf);
  return inverseTangentVector (theVec, theParam);
}

//=================================================================================================

NvGePointOnSurface& NvGePointOnSurface::setSurface (const NvGeSurface& theSurface)
{
  const occ::handle<NvGeImpEntity3d> aSurfaceImp = SurfaceImpOf (theSurface);
  const NvGePointOnSurfaceData* aData = DataOf (mpImpEnt, "setSurface");
  SetOnSurfaceData (mpImpEnt, aSurfaceImp, aData->U, aData->V);
  return *this;
}

//=================================================================================================

NvGePointOnSurface& NvGePointOnSurface::setParameter (const NvGePoint2d& theParam)
{
  const NvGePointOnSurfaceData* aData = DataOf (mpImpEnt, "setParameter");
  SetOnSurfaceData (mpImpEnt, aData->Surface, theParam.x, theParam.y);
  return *this;
}
