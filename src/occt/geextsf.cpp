// geextsf.cpp - implementation of NvGeExternalSurface.
//
// An external surface is an honest passthrough: it stores the external
// surface pointer as supplied plus the ExternalEntityKind flag. The only
// definition kind this layer can interpret is a pointer to a native
// NvGeSurface wrapper; its Geom_Surface is captured into the holder so
// that the type predicates (isPlane, isSphere, ...) can downcast real
// geometry and isNativeSurface() can materialize a native wrapper. Type
// conventions follow the native entities: planes are Geom_Plane, spheres
// Geom_SphericalSurface, cylinders Geom_CylindricalSurface, cones
// Geom_ConicalSurface, tori Geom_ToroidalSurface, nurbs
// Geom_BSplineSurface. A default-constructed (or null-reset) external
// surface stays undefined (isDefined() = false, predicates false). The
// generic NvGeSurface operations throw for external holders, exactly like
// for the external curves.

#include <geextsf.h>

#include <Nova.h>
#include <NvException.h>
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
class NvGeExternalSurfData : public NvGeEntityData
{
public:
  void* ExternalDef = nullptr;       // opaque definition pointer as supplied
  occ::handle<Geom_Surface> Surface; // geometry captured from a native surfaceDef
  NvGe::ExternalEntityKind Kind = NvGe::kExternalEntityUndefined;
  bool IsOwner = false;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeExternalSurfData> aCopy = new NvGeExternalSurfData();
    aCopy->ExternalDef = ExternalDef;
    aCopy->Surface = Surface;
    aCopy->Kind = Kind;
    aCopy->IsOwner = IsOwner;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeExternalSurfData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeExternalSurfData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeExternalSurfData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeExternalSurfData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeExternalSurfData> (theImp->Geom());
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
    throw NvException (std::string ("NvGeExternalSurface::") + theMethod
                       + "(): the external surface definition cannot be copied");
  }
}

//! Installs a fresh holder, replacing the base placeholder impl.
void InstallData (NvGeImpEntity3d*& theImp, void* theSurfaceDef,
                  NvGe::ExternalEntityKind theKind, Nova::Boolean theMakeCopy,
                  const char* theMethod)
{
  occ::handle<NvGeExternalSurfData> aData = new NvGeExternalSurfData();
  aData->ExternalDef = theSurfaceDef;
  aData->Kind = theKind;
  aData->IsOwner = theMakeCopy != Nova::kFalse;
  if (theSurfaceDef != nullptr)
  {
    // Validate and capture the geometry BEFORE touching this entity, so a
    // rejection leaves the external surface unchanged.
    aData->Surface = CaptureSurface (theSurfaceDef, aData->IsOwner, theMethod);
  }
  if (theImp != nullptr && theImp->RefCount() > 0)
  {
    // placeholder from the base constructor: release the adoption ref
    theImp->Unref();
  }
  theImp = new NvGeImpEntity3d (NvGe::kExternalSurface, aData);
  theImp->Ref();
}

} // namespace

//=================================================================================================

NvGeExternalSurface::NvGeExternalSurface ()
{
  InstallData (mpImpEnt, nullptr, NvGe::kExternalEntityUndefined, Nova::kFalse,
               "NvGeExternalSurface");
}

//=================================================================================================

NvGeExternalSurface::NvGeExternalSurface (void* theSurfaceDef,
                                          NvGe::ExternalEntityKind theSurfaceKind,
                                          Nova::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theSurfaceDef, theSurfaceKind, theMakeCopy, "NvGeExternalSurface");
}

//=================================================================================================

NvGeExternalSurface::NvGeExternalSurface (const NvGeExternalSurface& theSource)
: NvGeSurface (theSource)
{
}

//=================================================================================================

void NvGeExternalSurface::getExternalSurface (void*& theSurfaceDef) const
{
  theSurfaceDef = DataOf (mpImpEnt)->ExternalDef;
}

//=================================================================================================

NvGe::ExternalEntityKind NvGeExternalSurface::externalSurfaceKind () const
{
  return DataOf (mpImpEnt)->Kind;
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isPlane () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_Plane> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isSphere () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_SphericalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isCylinder () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_CylindricalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isCone () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_ConicalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isTorus () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_ToroidalSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isNurbSurface () const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  return !aData->Surface.IsNull()
      && !occ::down_cast<Geom_BSplineSurface> (aData->Surface).IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isDefined () const
{
  return !DataOf (mpImpEnt)->Surface.IsNull();
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isNativeSurface (NvGeSurface*& theNativeSurface) const
{
  const NvGeExternalSurfData* aData = DataOf (mpImpEnt);
  if (aData->Surface.IsNull())
  {
    theNativeSurface = nullptr;
    return Nova::kFalse;
  }
  // Generic native wrapper around the shared geometry (documented layout
  // bridge: all ge wrappers are {mpImpEnt, mDelEnt} shells, so the entity
  // base wrapper doubles as an NvGeSurface).
  theNativeSurface = (NvGeSurface*) newEntity3d (
      new NvGeImpEntity3d (NvGe::kSurface, aData->Surface));
  return Nova::kTrue;
}

//=================================================================================================

NvGeExternalSurface& NvGeExternalSurface::operator = (const NvGeExternalSurface& theSource)
{
  NvGeSurface::operator= (theSource);
  return *this;
}

//=================================================================================================

NvGeExternalSurface& NvGeExternalSurface::set (void* theSurfaceDef,
                                               NvGe::ExternalEntityKind theSurfaceKind,
                                               Nova::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theSurfaceDef, theSurfaceKind, theMakeCopy, "set");
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeExternalSurface::isOwnerOfSurface () const
{
  return DataOf (mpImpEnt)->IsOwner;
}

//=================================================================================================

NvGeExternalSurface& NvGeExternalSurface::setToOwnSurface ()
{
  NvGeExternalSurfData* aData = DataOf (mpImpEnt);
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
    throw NvException ("NvGeExternalSurface::setToOwnSurface(): the external surface"
                       " definition cannot be copied");
  }
  aData->IsOwner = true;
  return *this;
}
