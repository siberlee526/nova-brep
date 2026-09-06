// geoffsf.cpp - implementation of NvGeOffsetSurface based on OCCT
// Geom_OffsetSurface.
//
// The impl geometry is the Geom_OffsetSurface itself, so every generic
// NvGeSurface operation (evaluation, projection, ...) works through the
// shared base implementation. The basis surface and the offset distance
// are recovered from the offset geometry on demand. A default-constructed
// offset surface stays undefined: the predicates answer kFalse and the
// value-returning accessors throw.
//
// getSurface() converts the offset of an elementary carrier back to a
// simple wrapper with a closed form: plane -> plane, rectangular-trimmed
// plane -> bounded plane, sphere/cylinder -> grown or shrunk radius,
// cone -> same half angle with a shifted base circle, torus -> same major
// radius with a grown or shrunk minor radius. The offset is measured along
// the parametric normal D1U ^ D1V of the basis, which equals + ZDir for a
// direct position and - ZDir otherwise (the sense factor below). A
// conversion reports kFalse when the offset makes a radius non-positive or
// when the basis is not an elementary surface. getSurface and
// getConstructionSurface hand out newly allocated wrappers owned by the
// caller. OCCT failures are translated to NvException.

#include <geoffsf.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gebndpln.h>
#include <gecone.h>
#include <gecylndr.h>
#include <geplane.h>
#include <gesurf.h>
#include <gesphere.h>
#include <getorus.h>

#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <string>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Offset geometry of this entity, or a null handle for the undefined
//! entity (predicates never throw).
occ::handle<Geom_OffsetSurface> OffsetOrNullOf (const NvGeImpEntity3d* theImp)
{
  return occ::down_cast<Geom_OffsetSurface> (theImp->Geom());
}

//! Offset geometry of this entity; throws when the impl holds none.
occ::handle<Geom_OffsetSurface> OffsetOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (theImp);
  if (anOffset.IsNull())
  {
    throw NvException ("NvGeOffsetSurface: the entity holds no offset surface geometry");
  }
  return anOffset;
}

//! Extracts the basis geometry of a base wrapper; when theMakeCopy is set,
//! deep-copies it so the offset owns private geometry. Throws on a
//! missing or unusable base surface.
occ::handle<Geom_Surface> BasisHandleOf (NvGeSurface* theBaseSurface, bool theMakeCopy,
                                         const char* theMethod)
{
  if (theBaseSurface == nullptr)
  {
    throw NvException (std::string ("NvGeOffsetSurface::") + theMethod
                       + "(): the base surface is missing");
  }
  const occ::handle<Geom_Surface> aBasis = NvGeSurfaceOf (ImplOf (theBaseSurface));
  if (!theMakeCopy)
  {
    return aBasis;
  }
  try
  {
    return occ::down_cast<Geom_Surface> (aBasis->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeOffsetSurface::") + theMethod
                       + "(): the basis surface cannot be copied");
  }
}

//! Builds the offset surface geometry over a basis surface; translates an
//! OCCT rejection (non-C1 basis) to NvException.
occ::handle<Geom_Surface> MakeOffset (const occ::handle<Geom_Surface>& theBasis,
                                      double theDistance, const char* theMethod)
{
  try
  {
    return occ::handle<Geom_Surface> (new Geom_OffsetSurface (theBasis, theDistance));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeOffsetSurface::") + theMethod
                       + "(): OCCT rejected the offset of this basis surface");
  }
}

//! Point form of an OCCT point.
NvGePoint3d PointOf (const gp_Pnt& thePnt)
{
  return NvGePoint3d (thePnt.X(), thePnt.Y(), thePnt.Z());
}

//! Vector form of an OCCT direction.
NvGeVector3d VectorOf (const gp_Dir& theDir)
{
  return NvGeVector3d (theDir.X(), theDir.Y(), theDir.Z());
}

//! Sense of the parametric normal D1U ^ D1V relative to the position main
//! direction: +1 for a direct position, -1 otherwise.
double NormalSense (const gp_Ax3& thePos)
{
  return thePos.Direct() ? 1.0 : -1.0;
}

} // namespace

//=================================================================================================

NvGeOffsetSurface::NvGeOffsetSurface ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  // Undefined entity: no geometry; predicates answer kFalse and the
  // value-returning accessors throw.
  mpImpEnt = new NvGeImpEntity3d (NvGe::kOffsetSurface,
                                  occ::handle<Standard_Transient>());
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeOffsetSurface::NvGeOffsetSurface (NvGeSurface* theBaseSurface, double theOffsetDist,
                                      Adesk::Boolean theMakeCopy)
{
  // Validate and build BEFORE touching this entity, so a rejection leaves
  // the offset surface unchanged.
  const occ::handle<Geom_Surface> aBasis = BasisHandleOf (theBaseSurface,
                                                          theMakeCopy != Adesk::kFalse,
                                                          "NvGeOffsetSurface");
  const occ::handle<Geom_Surface> anOffset = MakeOffset (aBasis, theOffsetDist,
                                                         "NvGeOffsetSurface");
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kOffsetSurface, anOffset);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeOffsetSurface::NvGeOffsetSurface (const NvGeOffsetSurface& theOffset)
: NvGeSurface (theOffset)
{
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isPlane () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  return !anOffset.IsNull()
      && !occ::down_cast<Geom_Plane> (anOffset->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isBoundedPlane () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  if (anOffset.IsNull())
  {
    return Adesk::kFalse;
  }
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed =
    occ::down_cast<Geom_RectangularTrimmedSurface> (anOffset->BasisSurface());
  return !aTrimmed.IsNull()
      && !occ::down_cast<Geom_Plane> (aTrimmed->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isSphere () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  return !anOffset.IsNull()
      && !occ::down_cast<Geom_SphericalSurface> (anOffset->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isCylinder () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  return !anOffset.IsNull()
      && !occ::down_cast<Geom_CylindricalSurface> (anOffset->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isCone () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  return !anOffset.IsNull()
      && !occ::down_cast<Geom_ConicalSurface> (anOffset->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::isTorus () const
{
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  return !anOffset.IsNull()
      && !occ::down_cast<Geom_ToroidalSurface> (anOffset->BasisSurface()).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeOffsetSurface::getSurface (NvGeSurface*& theSurface) const
{
  theSurface = nullptr;
  const occ::handle<Geom_OffsetSurface> anOffset = OffsetOrNullOf (mpImpEnt);
  if (anOffset.IsNull())
  {
    return Adesk::kFalse;
  }
  const double aDist = anOffset->Offset();
  const occ::handle<Geom_Surface> aBasis = anOffset->BasisSurface();

  // Plane: a parallel plane.
  const occ::handle<Geom_Plane> aPlane = occ::down_cast<Geom_Plane> (aBasis);
  if (!aPlane.IsNull())
  {
    const gp_Ax3& aPos = aPlane->Position();
    gp_Vec aShift (aPos.Direction());
    aShift.Multiply (NormalSense (aPos) * aDist);
    theSurface = new NvGePlane (PointOf (aPos.Location().Translated (aShift)),
                                VectorOf (aPos.XDirection()), VectorOf (aPos.YDirection()));
    return Adesk::kTrue;
  }

  // Rectangular-trimmed plane: a bounded plane of the same parametric box.
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed =
    occ::down_cast<Geom_RectangularTrimmedSurface> (aBasis);
  if (!aTrimmed.IsNull())
  {
    const occ::handle<Geom_Plane> aBasisPlane =
      occ::down_cast<Geom_Plane> (aTrimmed->BasisSurface());
    if (aBasisPlane.IsNull())
    {
      return Adesk::kFalse;
    }
    double u1 = 0.0;
    double u2 = 0.0;
    double v1 = 0.0;
    double v2 = 0.0;
    aTrimmed->Bounds (u1, u2, v1, v2);
    const gp_Ax3& aPos = aBasisPlane->Position();
    gp_Vec aShift (aPos.Direction());
    aShift.Multiply (NormalSense (aPos) * aDist);
    gp_Pnt anOrigin = aPos.Location().Translated (aShift);
    gp_Vec anUShift (aPos.XDirection());
    anUShift.Multiply (u1);
    anOrigin.Translate (anUShift);
    gp_Vec aVShift (aPos.YDirection());
    aVShift.Multiply (v1);
    anOrigin.Translate (aVShift);
    gp_Vec anUVec (aPos.XDirection());
    anUVec.Multiply (u2 - u1);
    gp_Vec aVVec (aPos.YDirection());
    aVVec.Multiply (v2 - v1);
    theSurface = new NvGeBoundedPlane (PointOf (anOrigin),
                                       NvGeVector3d (anUVec.X(), anUVec.Y(), anUVec.Z()),
                                       NvGeVector3d (aVVec.X(), aVVec.Y(), aVVec.Z()));
    return Adesk::kTrue;
  }

  // Sphere: a concentric sphere with the radius grown (or shrunk) by the
  // offset distance.
  const occ::handle<Geom_SphericalSurface> aSphere =
    occ::down_cast<Geom_SphericalSurface> (aBasis);
  if (!aSphere.IsNull())
  {
    const gp_Ax3& aPos = aSphere->Position();
    const double aRadius = aSphere->Radius() + NormalSense (aPos) * aDist;
    if (aRadius <= 0.0)
    {
      return Adesk::kFalse;
    }
    theSurface = new NvGeSphere (aRadius, PointOf (aPos.Location()));
    return Adesk::kTrue;
  }

  // Cylinder: a coaxial cylinder with the radius grown (or shrunk) by the
  // offset distance.
  const occ::handle<Geom_CylindricalSurface> aCylinder =
    occ::down_cast<Geom_CylindricalSurface> (aBasis);
  if (!aCylinder.IsNull())
  {
    const gp_Ax3& aPos = aCylinder->Position();
    const double aRadius = aCylinder->Radius() + NormalSense (aPos) * aDist;
    if (aRadius <= 0.0)
    {
      return Adesk::kFalse;
    }
    theSurface = new NvGeCylinder (aRadius, PointOf (aPos.Location()),
                                   VectorOf (aPos.Direction()));
    return Adesk::kTrue;
  }

  // Cone: the same half angle; the base circle radius grows by
  // d * cos(a) and the base origin slides by -d * sin(a) along the axis
  // (the apex slides along the axis by -d / sin(a)).
  const occ::handle<Geom_ConicalSurface> aCone = occ::down_cast<Geom_ConicalSurface> (aBasis);
  if (!aCone.IsNull())
  {
    const gp_Ax3& aPos = aCone->Position();
    const double aSense = NormalSense (aPos);
    const double aCos = std::cos (aCone->SemiAngle());
    const double aSin = std::sin (aCone->SemiAngle());
    const double aBaseRadius = aCone->RefRadius() + aSense * aDist * aCos;
    if (aBaseRadius <= 0.0)
    {
      return Adesk::kFalse;
    }
    gp_Vec aShift (aPos.Direction());
    aShift.Multiply (-aSense * aDist * aSin);
    theSurface = new NvGeCone (aCos, aSin, PointOf (aPos.Location().Translated (aShift)),
                               aBaseRadius, VectorOf (aPos.Direction()));
    return Adesk::kTrue;
  }

  // Torus: the same major radius and center; the minor radius grows (or
  // shrinks) by the offset distance.
  const occ::handle<Geom_ToroidalSurface> aTorus = occ::down_cast<Geom_ToroidalSurface> (aBasis);
  if (!aTorus.IsNull())
  {
    const gp_Ax3& aPos = aTorus->Position();
    const double aMinor = aTorus->MinorRadius() + NormalSense (aPos) * aDist;
    if (aMinor <= 0.0)
    {
      return Adesk::kFalse;
    }
    theSurface = new NvGeTorus (aTorus->MajorRadius(), aMinor,
                                PointOf (aPos.Location()), VectorOf (aPos.Direction()));
    return Adesk::kTrue;
  }

  return Adesk::kFalse;
}

//=================================================================================================

void NvGeOffsetSurface::getConstructionSurface (NvGeSurface*& theBase) const
{
  const occ::handle<Geom_Surface> aBasis = OffsetOf (mpImpEnt)->BasisSurface();
  // Generic native wrapper around the shared basis geometry (documented
  // layout bridge: all ge wrappers are {mpImpEnt, mDelEnt} shells, so the
  // entity base wrapper doubles as an NvGeSurface). The caller owns it.
  theBase = (NvGeSurface*) newEntity3d (new NvGeImpEntity3d (NvGe::kSurface, aBasis));
}

//=================================================================================================

double NvGeOffsetSurface::offsetDist () const
{
  return OffsetOf (mpImpEnt)->Offset();
}

//=================================================================================================

NvGeOffsetSurface& NvGeOffsetSurface::set (NvGeSurface* theBaseSurface, double theOffsetDist,
                                           Adesk::Boolean theMakeCopy)
{
  // Build the new geometry BEFORE touching this entity, so a rejection
  // leaves the offset surface unchanged.
  const occ::handle<Geom_Surface> aBasis = BasisHandleOf (theBaseSurface,
                                                          theMakeCopy != Adesk::kFalse, "set");
  const occ::handle<Geom_Surface> anOffset = MakeOffset (aBasis, theOffsetDist, "set");
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (anOffset);
  return *this;
}

//=================================================================================================

NvGeOffsetSurface& NvGeOffsetSurface::operator = (const NvGeOffsetSurface& theOffset)
{
  NvGeSurface::operator= (theOffset);
  return *this;
}
