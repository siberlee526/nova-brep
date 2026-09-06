// geoffc3d.cpp - implementation of NvGeOffsetCurve3d.
//
// The impl geometry is the Geom_OffsetCurve itself, so every generic
// NvGeCurve3d operation (evaluation, projection, splitting, ...) works
// through the shared base implementation. The ARX plane normal is stored
// as the reference direction V of the OCCT offset: the offset side is
// T ^ V, i.e. the tangent crossed with the plane normal. OCCT only
// produces meaningful offsets for planar basis curves; construction
// failures (degenerate normal, non-C1 basis) are translated to
// NvException. The basis curve is recovered from the offset geometry on
// demand; curve() hands out a borrowed wrapper that stays valid until
// the next curve() call or a modification (single-slot cache, documented
// deviation from the ARX contract).

#include <geoffc3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <geimpdata.h>
#include <gemat3d.h>
#include <gevec3d.h>

#include <Geom_Curve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <gp_Dir.hxx>

#include <string>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Offset geometry of this entity.
occ::handle<Geom_OffsetCurve> OffsetOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_OffsetCurve> anOffset = occ::down_cast<Geom_OffsetCurve> (theImp->Geom());
  if (anOffset.IsNull())
  {
    throw NvException ("NvGeOffsetCurve3d: the entity holds no offset curve geometry");
  }
  return anOffset;
}

//! Plane normal of this entity as a wrapper vector.
NvGeVector3d NormalOf (const NvGeImpEntity3d* theImp)
{
  const gp_Dir& aDirection = OffsetOf (theImp)->Direction();
  return NvGeVector3d (aDirection.X(), aDirection.Y(), aDirection.Z());
}

//! Builds the offset curve geometry over a basis curve; the plane normal
//! doubles as the OCCT reference direction (offset side T ^ V).
occ::handle<Geom_Curve> MakeOffset (const occ::handle<Geom_Curve>& theBasis,
                                    double theDistance, const NvGeVector3d& theNormal,
                                    const char* theMethod)
{
  try
  {
    return occ::handle<Geom_Curve> (new Geom_OffsetCurve (theBasis, theDistance,
                                     gp_Dir (theNormal.x, theNormal.y, theNormal.z)));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeOffsetCurve3d::") + theMethod
                       + "(): OCCT rejected the offset of this basis curve"
                         " (degenerate normal or non-planar basis)");
  }
}

//! Single-slot cache for the borrowed basis wrapper handed out by curve().
NvGeCurve3d*& BasisWrapperSlot()
{
  static NvGeCurve3d* THE_WRAPPER = nullptr;
  return THE_WRAPPER;
}

} // namespace

//=================================================================================================

NvGeOffsetCurve3d::NvGeOffsetCurve3d (const NvGeCurve3d& theBaseCurve,
                                      const NvGeVector3d& thePlaneNormal,
                                      double theOffsetDistance)
{
  const occ::handle<Geom_Curve> aBasis = NvGeCurve3dOf (ImplOf (&theBaseCurve));
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kOffsetCurve3d,
                                  MakeOffset (aBasis, theOffsetDistance, thePlaneNormal,
                                              "NvGeOffsetCurve3d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeOffsetCurve3d::NvGeOffsetCurve3d (const NvGeOffsetCurve3d& theOffsetCurve)
: NvGeCurve3d (theOffsetCurve)
{
}

//=================================================================================================

const NvGeCurve3d* NvGeOffsetCurve3d::curve() const
{
  // Refresh the single-slot cache: the previous wrapper is released here.
  delete BasisWrapperSlot();
  BasisWrapperSlot() = (NvGeCurve3d*) newEntity3d (
      new NvGeImpEntity3d (NvGe::kCurve3d, OffsetOf (mpImpEnt)->BasisCurve()));
  return BasisWrapperSlot();
}

//=================================================================================================

NvGeVector3d NvGeOffsetCurve3d::normal() const
{
  return NormalOf (mpImpEnt);
}

//=================================================================================================

double NvGeOffsetCurve3d::offsetDistance() const
{
  return OffsetOf (mpImpEnt)->Offset();
}

//=================================================================================================

Nova::Boolean NvGeOffsetCurve3d::paramDirection() const
{
  // The offset follows the basis parameterization: same direction, so the
  // parameter mapping of the base curve is preserved.
  return true;
}

//=================================================================================================

NvGeMatrix3d NvGeOffsetCurve3d::transformation() const
{
  // Offsetting is modeled purely by the offset distance (OCCT convention);
  // no additional rigid transformation is applied to the basis.
  return NvGeMatrix3d();
}

//=================================================================================================

NvGeOffsetCurve3d& NvGeOffsetCurve3d::setCurve (const NvGeCurve3d& theBaseCurve)
{
  const occ::handle<Geom_Curve> aBasis = NvGeCurve3dOf (ImplOf (&theBaseCurve));
  const NvGeVector3d aNormal = NormalOf (mpImpEnt);
  const double aDistance = OffsetOf (mpImpEnt)->Offset();
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (MakeOffset (aBasis, aDistance, aNormal, "setCurve"));
  return *this;
}

//=================================================================================================

NvGeOffsetCurve3d& NvGeOffsetCurve3d::setNormal (const NvGeVector3d& thePlaneNormal)
{
  const occ::handle<Geom_Curve> aBasis = OffsetOf (mpImpEnt)->BasisCurve();
  const double aDistance = OffsetOf (mpImpEnt)->Offset();
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (MakeOffset (aBasis, aDistance, thePlaneNormal, "setNormal"));
  return *this;
}

//=================================================================================================

NvGeOffsetCurve3d& NvGeOffsetCurve3d::setOffsetDistance (double theDistance)
{
  const occ::handle<Geom_Curve> aBasis = OffsetOf (mpImpEnt)->BasisCurve();
  const NvGeVector3d aNormal = NormalOf (mpImpEnt);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (MakeOffset (aBasis, theDistance, aNormal, "setOffsetDistance"));
  return *this;
}

//=================================================================================================

NvGeOffsetCurve3d& NvGeOffsetCurve3d::operator = (const NvGeOffsetCurve3d& theOffsetCurve)
{
  NvGeCurve3d::operator= (theOffsetCurve);
  return *this;
}
