#ifndef NV_GEOFFSF_H
#define NV_GEOFFSF_H

#include "gegbl.h"
#include "gepnt3d.h"
#include "gesurf.h"
#pragma pack (push, 8)

class NvGePlane;
class NvGeBoundedPlane;
class NvGeCylinder;
class NvGeCone;
class NvGeSphere;
class NvGeTorus;

class
GX_DLLEXPIMPORT
NvGeOffsetSurface : public NvGeSurface
{
public:
    NvGeOffsetSurface();
    NvGeOffsetSurface(NvGeSurface* baseSurface,
                      double offsetDist,
                      Nova::Boolean makeCopy = Nova::kTrue);
    NvGeOffsetSurface(const NvGeOffsetSurface& offset);

    // Test whether this offset surface can be converted to a simple surface
    //
    Nova::Boolean    isPlane        () const;
    Nova::Boolean    isBoundedPlane () const;
    Nova::Boolean    isSphere       () const;
    Nova::Boolean    isCylinder     () const;
    Nova::Boolean    isCone         () const;
    Nova::Boolean    isTorus        () const;

    // Convert this offset surface to a simple surface
    //
        Nova::Boolean    getSurface(NvGeSurface*&) const;

    // Get a copy of the construction surface.
    //
    void              getConstructionSurface (NvGeSurface*& base) const;

    double            offsetDist     () const;

    // Reset surface
    //
    NvGeOffsetSurface& set        (NvGeSurface*, double offsetDist,
                                    Nova::Boolean makeCopy = Nova::kTrue);

    // Assignment operator.
    //
    NvGeOffsetSurface& operator =  (const NvGeOffsetSurface& offset);
};

#pragma pack (pop)
#endif
