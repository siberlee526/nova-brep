#ifndef NV_GEEXTSF_H
#define NV_GEEXTSF_H

#include "gegbl.h"
#include "gesurf.h"
#pragma pack (push, 8)

class NvGePlane;
class NvGeCylinder;
class NvGeCone;
class NvGeSphere;
class NvGeTorus;
class NvGeNurbSurface;
class surface;

class
GX_DLLEXPIMPORT
NvGeExternalSurface : public NvGeSurface
{
public:
   NvGeExternalSurface();
   NvGeExternalSurface(void* surfaceDef, NvGe::ExternalEntityKind surfaceKind,
                       Adesk::Boolean makeCopy = Adesk::kTrue);
   NvGeExternalSurface(const NvGeExternalSurface&);

   // Defining surface.
   //
   void getExternalSurface(void*& surfaceDef) const;

   // Type of the external surface.
   //
   NvGe::ExternalEntityKind  externalSurfaceKind() const;

   Adesk::Boolean    isPlane      () const;
   Adesk::Boolean    isSphere     () const;
   Adesk::Boolean    isCylinder   () const;
   Adesk::Boolean    isCone       () const;
   Adesk::Boolean    isTorus      () const;
   Adesk::Boolean    isNurbSurface() const;
   Adesk::Boolean    isDefined    () const;

   // Conversion to gelib entity
   //
   Adesk::Boolean isNativeSurface(NvGeSurface*& nativeSurface) const;

   // Assignment operator.
   //
   NvGeExternalSurface& operator = (const NvGeExternalSurface& src);

   // Reset surface
   //
   NvGeExternalSurface& set(void* surfaceDef,
                            NvGe::ExternalEntityKind surfaceKind,
                            Adesk::Boolean makeCopy = Adesk::kTrue);
   // Ownership of surface.
   //
   Adesk::Boolean       isOwnerOfSurface    () const;
   NvGeExternalSurface& setToOwnSurface     ();
};

#pragma pack (pop)
#endif
