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
                       Nova::Boolean makeCopy = Nova::kTrue);
   NvGeExternalSurface(const NvGeExternalSurface&);

   // Defining surface.
   //
   void getExternalSurface(void*& surfaceDef) const;

   // Type of the external surface.
   //
   NvGe::ExternalEntityKind  externalSurfaceKind() const;

   Nova::Boolean    isPlane      () const;
   Nova::Boolean    isSphere     () const;
   Nova::Boolean    isCylinder   () const;
   Nova::Boolean    isCone       () const;
   Nova::Boolean    isTorus      () const;
   Nova::Boolean    isNurbSurface() const;
   Nova::Boolean    isDefined    () const;

   // Conversion to gelib entity
   //
   Nova::Boolean isNativeSurface(NvGeSurface*& nativeSurface) const;

   // Assignment operator.
   //
   NvGeExternalSurface& operator = (const NvGeExternalSurface& src);

   // Reset surface
   //
   NvGeExternalSurface& set(void* surfaceDef,
                            NvGe::ExternalEntityKind surfaceKind,
                            Nova::Boolean makeCopy = Nova::kTrue);
   // Ownership of surface.
   //
   Nova::Boolean       isOwnerOfSurface    () const;
   NvGeExternalSurface& setToOwnSurface     ();
};

#pragma pack (pop)
#endif
