// geextc3d.cpp - implementation of NvGeExternalCurve3d.
//
// An external curve is an honest passthrough: it stores the external
// curve pointer as supplied plus the ExternalEntityKind flag. The only
// definition kind this layer can interpret is a pointer to a native
// NvGeCurve3d wrapper; its Geom_Curve is captured into the holder so
// that the type predicates (isLine, isCircArc, ...) can downcast real
// geometry and isNativeCurve() can materialize a native wrapper. Type
// conventions follow the native entities: lines are Geom_Line, rays and
// segments are Geom_TrimmedCurve over a Geom_Line (a ray is trimmed to
// [0, max double)), circles/ellipses are full or trimmed carrier conics,
// nurbs are Geom_BSplineCurve. A default-constructed (or null-reset)
// external curve stays undefined (isDefined() = false, predicates false).
// The generic NvGeCurve3d operations throw for external holders, exactly
// like for composite holders.

#include <geextc3d.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>

#include <Geom_BSplineCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>

#include <limits>
#include <string>

namespace
{

//! Storage: the passthrough pointer, the captured geometry and the flags.
class NvGeExternalCurve3dData : public NvGeEntityData
{
public:
  void* ExternalDef = nullptr;   // opaque definition pointer as supplied
  occ::handle<Geom_Curve> Curve; // geometry captured from a native curveDef
  NvGe::ExternalEntityKind Kind = NvGe::kExternalEntityUndefined;
  bool IsOwner = false;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeExternalCurve3dData> aCopy = new NvGeExternalCurve3dData();
    aCopy->ExternalDef = ExternalDef;
    aCopy->Curve = Curve;
    aCopy->Kind = Kind;
    aCopy->IsOwner = IsOwner;
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeExternalCurve3dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeExternalCurve3dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeExternalCurve3dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeExternalCurve3dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeExternalCurve3dData> (theImp->Geom());
}

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Unwraps one trimming level (arcs, segments and rays are stored as
//! Geom_TrimmedCurve over their carrier geometry).
occ::handle<Geom_Curve> BasisOf (const occ::handle<Geom_Curve>& theCurve)
{
  const occ::handle<Geom_TrimmedCurve> aTrimmed = occ::down_cast<Geom_TrimmedCurve> (theCurve);
  return aTrimmed.IsNull() ? theCurve : aTrimmed->BasisCurve();
}

//! True when the trimmed curve extends to an infinite parameter bound,
//! i.e. it represents a ray rather than a finite segment.
bool IsHalfBoundless (const occ::handle<Geom_TrimmedCurve>& theTrimmed)
{
  const double aHuge = std::numeric_limits<double>::max();
  return theTrimmed->FirstParameter() <= -aHuge || theTrimmed->LastParameter() >= aHuge;
}

//! Extracts the geometry of a native curve definition and, when
//! theMakeCopy is set, deep-copies it so the entity owns private geometry.
occ::handle<Geom_Curve> CaptureCurve (void* theCurveDef, bool theMakeCopy, const char* theMethod)
{
  // Documented bridge: the definition pointer of a native curve kind is a
  // NvGeCurve3d wrapper (all ge wrappers are {mpImpEnt, mDelEnt} shells).
  const NvGeCurve3d* aNative = static_cast<const NvGeCurve3d*> (theCurveDef);
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (ImplOf (aNative));
  if (!theMakeCopy)
  {
    return aCurve;
  }
  try
  {
    return occ::down_cast<Geom_Curve> (aCurve->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeExternalCurve3d::") + theMethod
                       + "(): the external curve definition cannot be copied");
  }
}

//! Installs a fresh holder, replacing the base placeholder impl.
void InstallData (NvGeImpEntity3d*& theImp, void* theCurveDef,
                  NvGe::ExternalEntityKind theKind, Adesk::Boolean theMakeCopy,
                  const char* theMethod)
{
  occ::handle<NvGeExternalCurve3dData> aData = new NvGeExternalCurve3dData();
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
  theImp = new NvGeImpEntity3d (NvGe::kExternalCurve3d, aData);
  theImp->Ref();
}

} // namespace

//=================================================================================================

NvGeExternalCurve3d::NvGeExternalCurve3d()
{
  InstallData (mpImpEnt, nullptr, NvGe::kExternalEntityUndefined, Adesk::kFalse,
               "NvGeExternalCurve3d");
}

//=================================================================================================

NvGeExternalCurve3d::NvGeExternalCurve3d (const NvGeExternalCurve3d& theSource)
: NvGeCurve3d (theSource)
{
}

//=================================================================================================

NvGeExternalCurve3d::NvGeExternalCurve3d (void* theCurveDef, NvGe::ExternalEntityKind theCurveKind,
                                          Adesk::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theCurveDef, theCurveKind, theMakeCopy, "NvGeExternalCurve3d");
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isLine() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  return !aData->Curve.IsNull()
      && !occ::down_cast<Geom_Line> (aData->Curve).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isRay() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  const occ::handle<Geom_TrimmedCurve> aTrimmed = aData->Curve.IsNull()
      ? occ::handle<Geom_TrimmedCurve>()
      : occ::down_cast<Geom_TrimmedCurve> (aData->Curve);
  return !aTrimmed.IsNull()
      && !occ::down_cast<Geom_Line> (aTrimmed->BasisCurve()).IsNull()
      && IsHalfBoundless (aTrimmed);
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isLineSeg() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  const occ::handle<Geom_TrimmedCurve> aTrimmed = aData->Curve.IsNull()
      ? occ::handle<Geom_TrimmedCurve>()
      : occ::down_cast<Geom_TrimmedCurve> (aData->Curve);
  return !aTrimmed.IsNull()
      && !occ::down_cast<Geom_Line> (aTrimmed->BasisCurve()).IsNull()
      && !IsHalfBoundless (aTrimmed);
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isCircArc() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  return !aData->Curve.IsNull()
      && !occ::down_cast<Geom_Circle> (BasisOf (aData->Curve)).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isEllipArc() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  return !aData->Curve.IsNull()
      && !occ::down_cast<Geom_Ellipse> (BasisOf (aData->Curve)).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isNurbCurve() const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  return !aData->Curve.IsNull()
      && !occ::down_cast<Geom_BSplineCurve> (aData->Curve).IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isDefined() const
{
  return !DataOf (mpImpEnt)->Curve.IsNull();
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isNativeCurve (NvGeCurve3d*& theNativeCurve) const
{
  const NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  if (aData->Curve.IsNull())
  {
    theNativeCurve = nullptr;
    return false;
  }
  // Generic native wrapper around the shared geometry (documented layout
  // bridge: all ge wrappers are {mpImpEnt, mDelEnt} shells, so the entity
  // base wrapper doubles as an NvGeCurve3d).
  theNativeCurve = (NvGeCurve3d*) newEntity3d (
      new NvGeImpEntity3d (NvGe::kCurve3d, aData->Curve));
  return true;
}

//=================================================================================================

void NvGeExternalCurve3d::getExternalCurve (void*& theCurveDef) const
{
  theCurveDef = DataOf (mpImpEnt)->ExternalDef;
}

//=================================================================================================

NvGe::ExternalEntityKind NvGeExternalCurve3d::externalCurveKind() const
{
  return DataOf (mpImpEnt)->Kind;
}

//=================================================================================================

NvGeExternalCurve3d& NvGeExternalCurve3d::set (void* theCurveDef,
                                               NvGe::ExternalEntityKind theCurveKind,
                                               Adesk::Boolean theMakeCopy)
{
  InstallData (mpImpEnt, theCurveDef, theCurveKind, theMakeCopy, "set");
  return *this;
}

//=================================================================================================

NvGeExternalCurve3d& NvGeExternalCurve3d::operator = (const NvGeExternalCurve3d& theSource)
{
  NvGeCurve3d::operator= (theSource);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeExternalCurve3d::isOwnerOfCurve() const
{
  return DataOf (mpImpEnt)->IsOwner;
}

//=================================================================================================

NvGeExternalCurve3d& NvGeExternalCurve3d::setToOwnCurve()
{
  NvGeExternalCurve3dData* aData = DataOf (mpImpEnt);
  if (aData->IsOwner || aData->Curve.IsNull())
  {
    return *this;
  }
  try
  {
    aData->Curve = occ::down_cast<Geom_Curve> (aData->Curve->Copy());
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGeExternalCurve3d::setToOwnCurve(): the external curve"
                       " definition cannot be copied");
  }
  aData->IsOwner = true;
  return *this;
}
