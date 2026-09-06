// geoffc2d.cpp - implementation of NvGeOffsetCurve2d.
//
// The impl geometry is the Geom2d_OffsetCurve itself, so every generic
// NvGeCurve2d operation (evaluation, projection, splitting, ...) works
// through the shared base implementation. The basis curve is recovered
// from the offset geometry on demand; curve() hands out a borrowed wrapper
// that stays valid until the next curve() call or a modification
// (single-slot cache, documented deviation from the ARX contract).

#include <geoffc2d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <geimpdata.h>
#include <gemat2d.h>
#include <gepnt2d.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <gp_Pnt2d.hxx>

#include <string>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity2d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Offset geometry of this entity.
occ::handle<Geom2d_OffsetCurve> OffsetOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom2d_OffsetCurve> anOffset = occ::down_cast<Geom2d_OffsetCurve> (theImp->Geom());
  if (anOffset.IsNull())
  {
    throw NvException ("NvGeOffsetCurve2d: the entity holds no offset curve geometry");
  }
  return anOffset;
}

//! Builds the offset curve geometry over a basis curve.
occ::handle<Geom2d_OffsetCurve> MakeOffset (const occ::handle<Geom2d_Curve>& theBasis,
                                            double theDistance, const char* theMethod)
{
  try
  {
    return occ::handle<Geom2d_OffsetCurve> (new Geom2d_OffsetCurve (theBasis, theDistance));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeOffsetCurve2d::") + theMethod
                       + "(): OCCT rejected the offset of this basis curve");
  }
}

//! Single-slot cache for the borrowed basis wrapper handed out by curve().
NvGeCurve2d*& BasisWrapperSlot()
{
  static NvGeCurve2d* THE_WRAPPER = nullptr;
  return THE_WRAPPER;
}

} // namespace

//=================================================================================================

NvGeOffsetCurve2d::NvGeOffsetCurve2d (const NvGeCurve2d& theBaseCurve, double theOffsetDistance)
{
  const occ::handle<Geom2d_Curve> aBasis = NvGeCurve2dOf (ImplOf (&theBaseCurve));
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kOffsetCurve2d,
                                  occ::handle<Geom2d_Curve> (
                                    MakeOffset (aBasis, theOffsetDistance, "NvGeOffsetCurve2d")));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeOffsetCurve2d::NvGeOffsetCurve2d (const NvGeOffsetCurve2d& theOffsetCurve)
: NvGeCurve2d (theOffsetCurve)
{
}

//=================================================================================================

const NvGeCurve2d* NvGeOffsetCurve2d::curve() const
{
  // Refresh the single-slot cache: the previous wrapper is released here.
  delete BasisWrapperSlot();
  BasisWrapperSlot() = (NvGeCurve2d*) newEntity2d (
      new NvGeImpEntity3d (NvGe::kCurve2d, OffsetOf (mpImpEnt)->BasisCurve()));
  return BasisWrapperSlot();
}

//=================================================================================================

double NvGeOffsetCurve2d::offsetDistance() const
{
  return OffsetOf (mpImpEnt)->Offset();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetCurve2d::paramDirection() const
{
  // The offset follows the basis parameterization: same direction, so the
  // parameter mapping of the base curve is preserved.
  return true;
}

//=================================================================================================

NvGeMatrix2d NvGeOffsetCurve2d::transformation() const
{
  // Offsetting is modeled purely by the offset distance (OCCT convention);
  // no additional rigid transformation is applied to the basis.
  return NvGeMatrix2d();
}

//=================================================================================================

NvGeOffsetCurve2d& NvGeOffsetCurve2d::setCurve (const NvGeCurve2d& theBaseCurve)
{
  const occ::handle<Geom2d_Curve> aBasis = NvGeCurve2dOf (ImplOf (&theBaseCurve));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom2d_Curve> (
    MakeOffset (aBasis, OffsetOf (mpImpEnt)->Offset(), "setCurve")));
  return *this;
}

//=================================================================================================

NvGeOffsetCurve2d& NvGeOffsetCurve2d::setOffsetDistance (double theDistance)
{
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom2d_Curve> (
    MakeOffset (OffsetOf (mpImpEnt)->BasisCurve(), theDistance, "setOffsetDistance")));
  return *this;
}

//=================================================================================================

NvGeOffsetCurve2d& NvGeOffsetCurve2d::operator = (const NvGeOffsetCurve2d& theOffsetCurve)
{
  NvGeCurve2d::operator= (theOffsetCurve);
  return *this;
}
