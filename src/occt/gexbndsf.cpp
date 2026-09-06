// gexbndsf.cpp - implementation of NvGeExternalBoundedSurface.
//
// A bounded external surface pairs an external (unbounded) base surface
// with contour data. Like the external surface it is an honest
// passthrough: it stores the external surface pointer as supplied plus
// the ExternalEntityKind flag. The only definition kind this layer can
// interpret is a pointer to a native NvGeSurface wrapper; its
// Geom_Surface is captured into the holder so that the type predicates
// (isPlane, isNurbs, ...) can downcast real geometry and the base surface
// accessors can materialize wrappers over it. The contour data is not
// modeled in this layer (there is no contour installer either):
// numContours() answers 0 and getContours() clears its outputs.
// isExternalSurface() answers kFalse because the base surface is always
// stored as captured native geometry, never as an external-surface
// holder. A default-constructed (or null-reset) surface stays undefined
// (isDefined() = false, predicates false). The generic NvGeSurface
// operations throw for external holders.

#include <gexbndsf.h>

#include <Nova.h>
#include <NvException.h>
#include <gecbndry.h>
#include <geextsf.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <gesurf.h>

#include <Geom_BSplineSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>

#include <string>

namespace
{

//! Storage: the passthrough pointer, the captured geometry and the flags.
class NvGeExternalBndSurfData : public NvGeEntityData
{
public:
  void* ExternalDef = nullptr;       // opaque definition pointer as supplied
  occ::handle<Geom_Surface> Surface; // geometry captured from a native surfaceDef
  NvGe::ExternalEntityKind Kind = NvGe::kExternalEntityUndefined;
  bool IsOwner = false;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeExternalBndSurfData> aCopy = new NvGeExternalBndSurfData();
    aCopy->ExternalDef = ExternalDef;
    aCopy->Surface = Surface;
    aCopy->Kind = Kind;
    aCopy->IsOwner = IsOwner;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeExternalBndSurfData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeExternalBndSurfData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeExternalBndSurfData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeExternalBndSurfData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeExternalBndSurfData> (theImp->Geom());
}

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Extracts the geometry of a native surface definition and, when
//! theMakeCopy is set, deep-copies it so the entity owns private geometry.
occ::handle<Geom_Surface> CaptureSurface (void* theSurfaceDef, bool theMakeCopy,
                                          const char* theMethod)
{
  // Documented bridge: the definition pointer of a native surface kind is
  // a NvGeSurface wrapper (all ge wrappers are {mpImpEnt, mDelEnt} shells).
  const NvGeSurface* aNative = static_cast<const NvGeSurface*> (theSurfaceDef);
  const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (ImplOf (aNative));
  if (!theMakeCopy)
  {
    return aSurface;
  }
  try
  {
    return occ::down_cast<Geom_Surface> (aSurface->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeExternalBoundedSurface::") + theMethod
                       + "(): the external surface definition cannot be copied");
  }
}

//! Installs a fresh holder, replacing the base placeholder impl.
void InstallData (NvGeImpEntity3d*& theImp, void* theSurfaceDef,
                  NvGe::ExternalEntityKind theKind, Nova::Boolean theMakeCopy,
                  const char* theMethod)
{
  occ::handle<NvGeExternalBndSurfData> aData = new NvGeExternalBndSurfData();
  aData->ExternalDef = theSurfaceDef;
  aData->Kind = theKind;
  aData->IsOwner = theMakeCopy != Nova::kFalse;
  if (theSurfaceDef != nullptr)
  {
    // Validate and capture the geometry BEFORE touching this entity, so a
    // rejection leaves the bounded surface unchanged.
    aData->Surface = CaptureSurface (theSurfaceDef, aData->IsOwner, theMethod);
  }
  if (theImp != nullptr && theImp->RefCount() > 0)
  {
    // placeholder from the base constructor: release the adoption ref
    theImp->Unref();
  }
  theImp = new NvGeImpEntity3d (NvGe::kExternalBoundedSurface, aData);
  theImp->Ref();
}

} // namespace

//=================================================================================================

NvGeExternalBoundedSurface::NvGeExternalBoundedSurface ()
{
  InstallData (mpImpEnt, nullptr, NvGe::kExternalEntityUndefined, Nova::kFalse,
               "NvGeExternalBoundedSurface");
}

//=================================================================================================

NvGeExternalBoundedSurface::NvGeExternalBoundedSurface (void* theSurfaceDef,
                                                        NvGe::ExternalEntityKind theSurfaceKind,
                                                        Nova::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theSurfaceDef, theSurfaceKind, theMakeCopy,
               "NvGeExternalBoundedSurface");
}

//=================================================================================================

NvGeExternalBoundedSurface::NvGeExternalBoundedSurface (const NvGeExternalBoundedSurface& theSource)
: NvGeSurface (theSource)
{
}

//=================================================================================================

NvGe::ExternalEntityKind NvGeExternalBoundedSurface::externalSurfaceKind () const
{
  return DataOf (mpImpEnt)->Kind;
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isDefined () const
{
  return !DataOf (mpImpEnt)->Surface.IsNull();
}

//=================================================================================================

void NvGeExternalBoundedSurface::getExternalSurface (void*& theSurfaceDef) const
{
  theSurfaceDef = DataOf (mpImpEnt)->ExternalDef;
}

//=================================================================================================

void NvGeExternalBoundedSurface::getBaseSurface (NvGeSurface*& theSurfaceDef) const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  if (aData->Surface.IsNull())
  {
    theSurfaceDef = nullptr;
    return;
  }
  // Generic native wrapper around the shared base geometry (documented
  // layout bridge: all ge wrappers are {mpImpEnt, mDelEnt} shells, so the
  // entity base wrapper doubles as an NvGeSurface). The caller owns it.
  theSurfaceDef = (NvGeSurface*) newEntity3d (
      new NvGeImpEntity3d (NvGe::kSurface, aData->Surface));
}

//=================================================================================================

void NvGeExternalBoundedSurface::getBaseSurface (NvGeExternalSurface& theUnbounded) const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  if (aData->Surface.IsNull())
  {
    theUnbounded.set (nullptr, NvGe::kExternalEntityUndefined, Nova::kFalse);
    return;
  }
  // Wrap the captured base geometry in a transient native wrapper and hand
  // it to the external surface as its definition (shared, not copied).
  NvGeSurface* aBase = (NvGeSurface*) newEntity3d (
      new NvGeImpEntity3d (NvGe::kSurface, aData->Surface));
  theUnbounded.set (aBase, NvGe::kAcisEntity, Nova::kFalse);
  delete aBase;
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isPlane () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_Plane> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isSphere () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_SphericalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isCylinder () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_CylindricalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isCone () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_ConicalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isTorus () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_ToroidalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isNurbs () const
{
  const NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_BSplineSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isExternalSurface () const
{
  // The base surface is always stored as captured native geometry in this
  // layer, never as an external-surface holder.
  return Nova::kFalse;
}

//=================================================================================================

int NvGeExternalBoundedSurface::numContours () const
{
  return 0;
}

//=================================================================================================

void NvGeExternalBoundedSurface::getContours (int& theNumContours,
                                              NvGeCurveBoundary*& theCurveBoundaries) const
{
  // Contour data is not modeled in this layer (see the file banner).
  theNumContours = 0;
  theCurveBoundaries = nullptr;
}

//=================================================================================================

NvGeExternalBoundedSurface& NvGeExternalBoundedSurface::set (void* theSurfaceDef,
                                                             NvGe::ExternalEntityKind theSurfaceKind,
                                                             Nova::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theSurfaceDef, theSurfaceKind, theMakeCopy, "set");
  return *this;
}

//=================================================================================================

NvGeExternalBoundedSurface& NvGeExternalBoundedSurface::operator = (
  const NvGeExternalBoundedSurface& theSource)
{
  NvGeSurface::operator= (theSource);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeExternalBoundedSurface::isOwnerOfSurface () const
{
  return DataOf (mpImpEnt)->IsOwner;
}

//=================================================================================================

NvGeExternalBoundedSurface& NvGeExternalBoundedSurface::setToOwnSurface ()
{
  NvGeExternalBndSurfData* aData = DataOf (mpImpEnt);
  if (aData->IsOwner || aData->Surface.IsNull())
  {
    return *this;
  }
  try
  {
    aData->Surface = occ::down_cast<Geom_Surface> (aData->Surface->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGeExternalBoundedSurface::setToOwnSurface(): the external surface"
                       " definition cannot be copied");
  }
  aData->IsOwner = true;
  return *this;
}
