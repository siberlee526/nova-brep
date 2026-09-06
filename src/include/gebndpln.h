#ifndef NV_GEBNDPLN_H
#define NV_GEBNDPLN_H

#include "geplanar.h"
#include "geplane.h"
#pragma pack (push, 8)

class NvGePlane;
class NvGeVector3d;
class NvGePoint3d;
class NvGePoint2d;
class NvGeLineSeg3d;

class

NvGeBoundedPlane : public NvGePlanarEnt
{
public:
    GE_DLLEXPIMPORT NvGeBoundedPlane();
    GE_DLLEXPIMPORT NvGeBoundedPlane(const NvGeBoundedPlane& plane);
    GE_DLLEXPIMPORT NvGeBoundedPlane(const NvGePoint3d& origin, const NvGeVector3d& uVec,
                     const NvGeVector3d& vVec);
    GE_DLLEXPIMPORT NvGeBoundedPlane(const NvGePoint3d& p1, const NvGePoint3d& origin,
                     const NvGePoint3d& p2);

    // Intersection.
    //
    GE_DLLEXPIMPORT Nova::Boolean    intersectWith (const NvGeLinearEnt3d& linEnt, NvGePoint3d& point,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean    intersectWith (const NvGePlane& plane, NvGeLineSeg3d& results,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean    intersectWith (const NvGeBoundedPlane& plane, NvGeLineSeg3d& result,
                                     const NvGeTol& tol = NvGeContext::gTol) const;

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeBoundedPlane& set           (const NvGePoint3d& origin,
                                     const NvGeVector3d& uVec,
                                     const NvGeVector3d& vVec);
    GE_DLLEXPIMPORT NvGeBoundedPlane& set           (const NvGePoint3d& p1,
                                     const NvGePoint3d& origin,
                                     const NvGePoint3d& p2);
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeBoundedPlane& operator =    (const NvGeBoundedPlane& bplane);
};

#pragma pack (pop)
#endif
