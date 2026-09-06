#ifndef NV_GELENT3D_H
#define NV_GELENT3D_H

#include "gecurv3d.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#pragma pack (push, 8)

class NvGeLine3d;
class NvGeCircArc3d;
class NvGePlanarEnt;
class

NvGeLinearEnt3d : public NvGeCurve3d
{
public:
    // Intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith (const NvGeLinearEnt3d& line,
                                  NvGePoint3d& intPt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith (const NvGePlanarEnt& plane, NvGePoint3d& intPnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Projection-intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean projIntersectWith(const NvGeLinearEnt3d& line,
                                  const NvGeVector3d& projDir,
                                  NvGePoint3d& pntOnThisLine,
                                  NvGePoint3d& pntOnOtherLine,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Find the overlap with other NvGeLinearEnt object
    //
    GE_DLLEXPIMPORT Adesk::Boolean overlap       (const NvGeLinearEnt3d& line,
                                  NvGeLinearEnt3d*& overlap,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Containment tests.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isOn          (const NvGePoint3d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isOn          (const NvGePoint3d& pnt, double& param,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isOn          (double param,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isOn          (const NvGePlane& plane,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Direction tests.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isParallelTo  (const NvGeLinearEnt3d& line,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isParallelTo  (const NvGePlanarEnt& plane,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isPerpendicularTo(const NvGeLinearEnt3d& line,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isPerpendicularTo(const NvGePlanarEnt& plane,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Test if two lines are colinear.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isColinearTo  (const NvGeLinearEnt3d& line,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Perpendicular through a given point
    //
    GE_DLLEXPIMPORT void          getPerpPlane   (const NvGePoint3d& pnt, NvGePlane& plane) const;

    // Definition of line.
    //
    GE_DLLEXPIMPORT NvGePoint3d    pointOnLine   () const;
    GE_DLLEXPIMPORT NvGeVector3d   direction     () const;
    GE_DLLEXPIMPORT void           getLine       (NvGeLine3d&) const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLinearEnt3d& operator =  (const NvGeLinearEnt3d& line);

protected:
    GE_DLLEXPIMPORT NvGeLinearEnt3d ();
    GE_DLLEXPIMPORT NvGeLinearEnt3d (const NvGeLinearEnt3d&);
};

#pragma pack (pop)
#endif
