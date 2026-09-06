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
                      Adesk::Boolean makeCopy = Adesk::kTrue);
    NvGeOffsetSurface(const NvGeOffsetSurface& offset);

    // Test whether this offset surface can be converted to a simple surface
    //
    Adesk::Boolean    isPlane        () const;
    Adesk::Boolean    isBoundedPlane () const;
    Adesk::Boolean    isSphere       () const;
    Adesk::Boolean    isCylinder     () const;
    Adesk::Boolean    isCone         () const;
    Adesk::Boolean    isTorus        () const;

    // Convert this offset surface to a simple surface
    //
        Adesk::Boolean    getSurface(NvGeSurface*&) const;

    // Get a copy of the construction surface.
    //
    void              getConstructionSurface (NvGeSurface*& base) const;

    double            offsetDist     () const;

    // Reset surface
    //
    NvGeOffsetSurface& set        (NvGeSurface*, double offsetDist,
                                    Adesk::Boolean makeCopy = Adesk::kTrue);

    // Assignment operator.
    //
    NvGeOffsetSurface& operator =  (const NvGeOffsetSurface& offset);
};

#pragma pack (pop)
#endif
