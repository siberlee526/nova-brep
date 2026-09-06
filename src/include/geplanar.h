#ifndef NV_GEPLANAR_H
#define NV_GEPLANAR_H

#include "gesurf.h"
#include "gevec3d.h"
#pragma pack (push, 8)

class NvGeLineSeg3d;
class NvGeLinearEnt3d;
class NvGeCircArc3d;

class

NvGePlanarEnt : public NvGeSurface
{
public:
    // Intersection
    //
    GE_DLLEXPIMPORT Adesk::Boolean  intersectWith    (const NvGeLinearEnt3d& linEnt, NvGePoint3d& pnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;
    // Closest point
    //
    GE_DLLEXPIMPORT NvGePoint3d     closestPointToLinearEnt (const NvGeLinearEnt3d& line,
                                             NvGePoint3d& pointOnLine,
                                             const NvGeTol& tol
                                               = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGePoint3d     closestPointToPlanarEnt (const NvGePlanarEnt& otherPln,
                                             NvGePoint3d& pointOnOtherPln,
                                             const NvGeTol& tol
                                               = NvGeContext::gTol) const;
    // Direction tests.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isParallelTo      (const NvGeLinearEnt3d& linEnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isParallelTo      (const NvGePlanarEnt& otherPlnEnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isPerpendicularTo (const NvGeLinearEnt3d& linEnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isPerpendicularTo (const NvGePlanarEnt& linEnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;

    // Point set equality.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isCoplanarTo      (const NvGePlanarEnt& otherPlnEnt,
                                      const NvGeTol& tol = NvGeContext::gTol) const;

    // Get methods.
    //
    GE_DLLEXPIMPORT void              get            (NvGePoint3d&, NvGeVector3d& uVec,
                                      NvGeVector3d& vVec) const;
    GE_DLLEXPIMPORT void              get            (NvGePoint3d&, NvGePoint3d& origin,
                                      NvGePoint3d&) const;

    // Geometric properties.
    //
    GE_DLLEXPIMPORT NvGePoint3d    pointOnPlane      () const;
    GE_DLLEXPIMPORT NvGeVector3d   normal            () const;
    GE_DLLEXPIMPORT void           getCoefficients(double& a, double& b, double& c, double& d) const;
    GE_DLLEXPIMPORT void           getCoordSystem(NvGePoint3d& origin, NvGeVector3d& axis1,
                                  NvGeVector3d& axis2) const;
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePlanarEnt& operator =        (const NvGePlanarEnt& src);

protected:
    GE_DLLEXPIMPORT NvGePlanarEnt ();
    GE_DLLEXPIMPORT NvGePlanarEnt (const NvGePlanarEnt&);
};

#pragma pack (pop)
#endif
