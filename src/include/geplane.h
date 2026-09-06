#ifndef NV_GEPLANE_H
#define NV_GEPLANE_H

#include "geplanar.h"
#pragma pack (push, 8)

class NvGeBoundedPlane;
class NvGeLine3d;

class
NvGePlane : public NvGePlanarEnt
{
public:
    // Global plane objects.
    //
    GE_DLLDATAEXIMP static const NvGePlane kXYPlane;
    GE_DLLDATAEXIMP static const NvGePlane kYZPlane;
    GE_DLLDATAEXIMP static const NvGePlane kZXPlane;

    GE_DLLEXPIMPORT NvGePlane();
    GE_DLLEXPIMPORT NvGePlane(const NvGePlane& src);
    GE_DLLEXPIMPORT NvGePlane(const NvGePoint3d& origin, const NvGeVector3d& normal);
    GE_DLLEXPIMPORT NvGePlane(const NvGePoint3d& pntU, const NvGePoint3d& org, const NvGePoint3d& pntV);
    GE_DLLEXPIMPORT NvGePlane(const NvGePoint3d& org, const NvGeVector3d& uAxis,
              const NvGeVector3d& vAxis);
    GE_DLLEXPIMPORT NvGePlane(double a, double b, double c, double d);

    // Signed distance from a point to a plane.
    //
    GE_DLLEXPIMPORT double         signedDistanceTo (const NvGePoint3d& pnt) const;

    // Intersection
    //
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith    (const NvGeLinearEnt3d& linEnt, NvGePoint3d& resultPnt,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith    (const NvGePlane& otherPln, NvGeLine3d& resultLine,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith    (const NvGeBoundedPlane& bndPln, NvGeLineSeg3d& resultLineSeg,
                                     const NvGeTol& tol = NvGeContext::gTol) const;

    // Geometry redefinition.
    //
    GE_DLLEXPIMPORT NvGePlane&     set              (const NvGePoint3d& pnt, const NvGeVector3d& normal);
    GE_DLLEXPIMPORT NvGePlane&     set              (const NvGePoint3d& pntU, const NvGePoint3d& org,
                                     const NvGePoint3d& pntV);
    GE_DLLEXPIMPORT NvGePlane&     set              (double a, double b, double c, double d);
    GE_DLLEXPIMPORT NvGePlane&     set              (const NvGePoint3d& org,
                                     const NvGeVector3d& uAxis,
                                     const NvGeVector3d& vAxis);
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePlane&     operator =       (const NvGePlane& src);
};

#pragma pack (pop)
#endif
