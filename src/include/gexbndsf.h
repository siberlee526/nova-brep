#ifndef NV_GEXBNDSF_H
#define NV_GEXBNDSF_H

#include "gegbl.h"
#include "gesurf.h"
#pragma pack (push, 8)

class NvGeExternalSurface;
class NvGeCurveBoundary;

class
GX_DLLEXPIMPORT
NvGeExternalBoundedSurface : public NvGeSurface
{
public:
   NvGeExternalBoundedSurface();
   NvGeExternalBoundedSurface(void* surfaceDef, NvGe::ExternalEntityKind surfaceKind,
                              Adesk::Boolean makeCopy = Adesk::kTrue);
   NvGeExternalBoundedSurface(const NvGeExternalBoundedSurface&);

   // Surface data.
   //
   NvGe::ExternalEntityKind   externalSurfaceKind  () const;
   Adesk::Boolean             isDefined            () const;
   void                       getExternalSurface   (void*& surfaceDef) const;

    // Access to unbounded surface.
    //

    void getBaseSurface        (NvGeSurface*& surfaceDef) const;

	void getBaseSurface        (NvGeExternalSurface& unboundedSurfaceDef) const;

    // Type queries on the unbounded base surface.
    Adesk::Boolean isPlane() const;
    Adesk::Boolean isSphere() const;
    Adesk::Boolean isCylinder() const;
    Adesk::Boolean isCone() const;
    Adesk::Boolean isTorus() const;
    Adesk::Boolean isNurbs() const;
    Adesk::Boolean isExternalSurface() const;

         // Access to the boundary data.
    //
    int          numContours  () const;
    void         getContours  (int& numContours, NvGeCurveBoundary*& curveBoundaries) const;

    // Set methods
    //
    NvGeExternalBoundedSurface& set  (void* surfaceDef,
                                      NvGe::ExternalEntityKind surfaceKind,
                                      Adesk::Boolean makeCopy = Adesk::kTrue);

    // Assignment operator.
    //
    NvGeExternalBoundedSurface& operator = (const NvGeExternalBoundedSurface&);

    // Surface ownership.
    //
        Adesk::Boolean               isOwnerOfSurface() const;
    NvGeExternalBoundedSurface&  setToOwnSurface();
};

#pragma pack (pop)
#endif
