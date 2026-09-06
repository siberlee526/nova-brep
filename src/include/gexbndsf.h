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
                              Nova::Boolean makeCopy = Nova::kTrue);
   NvGeExternalBoundedSurface(const NvGeExternalBoundedSurface&);

   // Surface data.
   //
   NvGe::ExternalEntityKind   externalSurfaceKind  () const;
   Nova::Boolean             isDefined            () const;
   void                       getExternalSurface   (void*& surfaceDef) const;

    // Access to unbounded surface.
    //

    void getBaseSurface        (NvGeSurface*& surfaceDef) const;

	void getBaseSurface        (NvGeExternalSurface& unboundedSurfaceDef) const;

    // Type queries on the unbounded base surface.
    Nova::Boolean isPlane() const;
    Nova::Boolean isSphere() const;
    Nova::Boolean isCylinder() const;
    Nova::Boolean isCone() const;
    Nova::Boolean isTorus() const;
    Nova::Boolean isNurbs() const;
    Nova::Boolean isExternalSurface() const;

         // Access to the boundary data.
    //
    int          numContours  () const;
    void         getContours  (int& numContours, NvGeCurveBoundary*& curveBoundaries) const;

    // Set methods
    //
    NvGeExternalBoundedSurface& set  (void* surfaceDef,
                                      NvGe::ExternalEntityKind surfaceKind,
                                      Nova::Boolean makeCopy = Nova::kTrue);

    // Assignment operator.
    //
    NvGeExternalBoundedSurface& operator = (const NvGeExternalBoundedSurface&);

    // Surface ownership.
    //
        Nova::Boolean               isOwnerOfSurface() const;
    NvGeExternalBoundedSurface&  setToOwnSurface();
};

#pragma pack (pop)
#endif
