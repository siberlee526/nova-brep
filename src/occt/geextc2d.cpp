// geextc2d.cpp - implementation of NvGeExternalCurve2d.
//
// An external curve is an honest passthrough: it stores the external
// curve pointer as supplied plus the ExternalEntityKind flag. The only
// definition kind this layer can interpret is a pointer to a native
// NvGeCurve2d wrapper; its Geom2d_Curve is captured into the holder so
// that isNurbCurve() and setToOwnCurve() operate on real geometry. A
// default-constructed (or null-reset) external curve stays undefined
// (isDefined() = false). The generic NvGeCurve2d operations throw for
// external holders, exactly like for composite holders.

#include <geextc2d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <genurb2d.h>

#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>

#include <string>

namespace
{

//! Storage: the passthrough pointer, the captured geometry and the flags.
class NvGeExternalCurve2dData : public NvGeEntityData
{
public:
  void* ExternalDef = nullptr;     // opaque definition pointer as supplied
  occ::handle<Geom2d_Curve> Curve; // geometry captured from a native curveDef
  NvGe::ExternalEntityKind Kind = NvGe::kExternalEntityUndefined;
  bool IsOwner = false;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeExternalCurve2dData> aCopy = new NvGeExternalCurve2dData();
    aCopy->ExternalDef = ExternalDef;
    aCopy->Curve = Curve;
    aCopy->Kind = Kind;
    aCopy->IsOwner = IsOwner;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeExternalCurve2dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeExternalCurve2dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeExternalCurve2dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeExternalCurve2dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeExternalCurve2dData> (theImp->Geom());
}

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
NvGeImpEntity3d* ImplOf (const NvGeEntity2d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Extracts the geometry of a native curve definition and, when
//! theMakeCopy is set, deep-copies it so the entity owns private geometry.
occ::handle<Geom2d_Curve> CaptureCurve (void* theCurveDef, bool theMakeCopy, const char* theMethod)
{
  // Documented bridge: the definition pointer of a native curve kind is a
  // NvGeCurve2d wrapper (all ge wrappers are {mpImpEnt, mDelEnt} shells).
  const NvGeCurve2d* aNative = static_cast<const NvGeCurve2d*> (theCurveDef);
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (ImplOf (aNative));
  if (!theMakeCopy)
  {
    return aCurve;
  }
  try
  {
    return occ::down_cast<Geom2d_Curve> (aCurve->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeExternalCurve2d::") + theMethod
                       + "(): the external curve definition cannot be copied");
  }
}

//! Installs a fresh holder, replacing the base placeholder impl.
void InstallData (NvGeImpEntity3d*& theImp, void* theCurveDef,
                  NvGe::ExternalEntityKind theKind, Adesk::Boolean theMakeCopy,
                  const char* theMethod)
{
  occ::handle<NvGeExternalCurve2dData> aData = new NvGeExternalCurve2dData();
  aData->ExternalDef = theCurveDef;
  aData->Kind = theKind;
  aData->IsOwner = theMakeCopy != Adesk::kFalse;
  if (theCurveDef != nullptr)
  {
    // Validate and capture the geometry BEFORE touching this entity, so a
    // rejection leaves the external curve unchanged.
    aData->Curve = CaptureCurve (theCurveDef, aData->IsOwner, theMethod);
  }
  if (theImp != nullptr && theImp->RefCount() > 0)
  {
    // placeholder from the base constructor: release the adoption ref
    theImp->Unref();
  }
  theImp = new NvGeImpEntity3d (NvGe::kExternalCurve2d, aData);
  theImp->Ref();
}

} // namespace

//=================================================================================================

NvGeExternalCurve2d::NvGeExternalCurve2d()
{
  InstallData (mpImpEnt, nullptr, NvGe::kExternalEntityUndefined, Adesk::kFalse,
               "NvGeExternalCurve2d");
}

//=================================================================================================

NvGeExternalCurve2d::NvGeExternalCurve2d (const NvGeExternalCurve2d& theSource)
: NvGeCurve2d (theSource)
{
}

//=================================================================================================

NvGeExternalCurve2d::NvGeExternalCurve2d (void* theCurveDef, NvGe::ExternalEntityKind theCurveKind,
                                          Adesk::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theCurveDef, theCurveKind, theMakeCopy, "NvGeExternalCurve2d");
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve2d::isNurbCurve() const
{
  const NvGeExternalCurve2dData* aData = DataOf (mpImpEnt);
  return !aData->Curve.IsNull()
      && !occ::down_cast<Geom2d_BSplineCurve> (aData->Curve).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve2d::isNurbCurve (NvGeNurbCurve2d& theNurbCurve) const
{
  const NvGeExternalCurve2dData* aData = DataOf (mpImpEnt);
  if (aData->Curve.IsNull()
      || occ::down_cast<Geom2d_BSplineCurve> (aData->Curve).IsNull())
  {
    return false;
  }
  // Convert through the public approximating constructor: build a borrowed
  // native wrapper around the shared geometry (documented layout bridge),
  // then assign the conversion result to the caller's nurb curve.
  NvGeCurve2d* aNative = (NvGeCurve2d*) newEntity2d (
      new NvGeImpEntity3d (NvGe::kCurve2d, aData->Curve));
  try
  {
    theNurbCurve = NvGeNurbCurve2d (*aNative);
  }
  catch (const Standard_Failure&)
  {
    delete aNative;
    return false;
  }
  catch (const NvException&)
  {
    delete aNative;
    return false;
  }
  delete aNative;
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve2d::isDefined() const
{
  return !DataOf (mpImpEnt)->Curve.IsNull();
}

//=================================================================================================

void NvGeExternalCurve2d::getExternalCurve (void*& theCurveDef) const
{
  theCurveDef = DataOf (mpImpEnt)->ExternalDef;
}

//=================================================================================================

NvGe::ExternalEntityKind NvGeExternalCurve2d::externalCurveKind() const
{
  return DataOf (mpImpEnt)->Kind;
}

//=================================================================================================

NvGeExternalCurve2d& NvGeExternalCurve2d::set (void* theCurveDef,
                                               NvGe::ExternalEntityKind theCurveKind,
                                               Adesk::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theCurveDef, theCurveKind, theMakeCopy, "set");
  return *this;
}

//=================================================================================================

NvGeExternalCurve2d& NvGeExternalCurve2d::operator = (const NvGeExternalCurve2d& theSource)
{
  NvGeCurve2d::operator= (theSource);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve2d::isOwnerOfCurve() const
{
  return DataOf (mpImpEnt)->IsOwner;
}

//=================================================================================================

NvGeExternalCurve2d& NvGeExternalCurve2d::setToOwnCurve()
{
  NvGeExternalCurve2dData* aData = DataOf (mpImpEnt);
  if (aData->IsOwner || aData->Curve.IsNull())
  {
    return *this;
  }
  try
  {
    aData->Curve = occ::down_cast<Geom2d_Curve> (aData->Curve->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGeExternalCurve2d::setToOwnCurve(): the external curve"
                       " definition cannot be copied");
  }
  aData->IsOwner = true;
  return *this;
}
