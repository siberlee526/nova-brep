#ifndef NV_GELENT2D_H
#define NV_GELENT2D_H

#include "gecurv2d.h"
#include "gepnt2d.h"
#include "gevec2d.h"
#pragma pack (push, 8)

class NvGeCircArc2d;

class

NvGeLinearEnt2d : public NvGeCurve2d
{
public:
    // Intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean   intersectWith  (const NvGeLinearEnt2d& line, NvGePoint2d& intPnt,
                                     const NvGeTol& tol = NvGeContext::gTol) const;

    // Find the overlap with other NvGeLinearEnt object
    //
    GE_DLLEXPIMPORT Adesk::Boolean   overlap        (const NvGeLinearEnt2d& line,
                                     NvGeLinearEnt2d*& overlap,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    // Direction tests.
    //
    GE_DLLEXPIMPORT Adesk::Boolean   isParallelTo   (const NvGeLinearEnt2d& line,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean   isPerpendicularTo(const NvGeLinearEnt2d& line,
                                      const NvGeTol& tol = NvGeContext::gTol) const;
    // Test if two lines are colinear.
    //
    GE_DLLEXPIMPORT Adesk::Boolean   isColinearTo   (const NvGeLinearEnt2d& line,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    // Perpendicular through a given point
    //
    GE_DLLEXPIMPORT void             getPerpLine    (const NvGePoint2d& pnt, NvGeLine2d& perpLine) const;

    // Definition of line.
    //
    GE_DLLEXPIMPORT NvGePoint2d      pointOnLine    () const;
    GE_DLLEXPIMPORT NvGeVector2d     direction      () const;
    GE_DLLEXPIMPORT void             getLine        (NvGeLine2d& line) const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLinearEnt2d& operator =     (const NvGeLinearEnt2d& line);

protected:
    GE_DLLEXPIMPORT NvGeLinearEnt2d ();
    GE_DLLEXPIMPORT NvGeLinearEnt2d (const NvGeLinearEnt2d&);
};

#pragma pack (pop)
#endif
