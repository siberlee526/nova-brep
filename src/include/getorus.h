#ifndef NV_GETORUS_H
#define NV_GETORUS_H

#include "gegbl.h"
#include "gesurf.h"
#include "geintrvl.h"
#include "gevec3d.h"
#pragma pack (push, 8)


class NvGeCircArc3d;

class
GX_DLLEXPIMPORT
NvGeTorus : public NvGeSurface
{
public:
    NvGeTorus();
    NvGeTorus(double majorRadius, double minorRadius,
              const NvGePoint3d& origin, const NvGeVector3d& axisOfSymmetry);
    NvGeTorus(double majorRadius, double minorRadius,
              const NvGePoint3d&  origin, const NvGeVector3d& axisOfSymmetry,
              const NvGeVector3d& refAxis,
              double startAngleU, double endAngleU,
              double startAngleV, double endAngleV);
    NvGeTorus(const NvGeTorus& torus);

    // Geometric properties.
    //
    double          majorRadius    () const;
    double          minorRadius    () const;
    void            getAnglesInU   (double& start, double& end) const;
    void            getAnglesInV   (double& start, double& end) const;
    NvGePoint3d     center         () const;
    NvGeVector3d    axisOfSymmetry () const;
    NvGeVector3d    refAxis        () const;
    Nova::Boolean  isOuterNormal  () const;

    NvGeTorus&      setMajorRadius (double radius);
    NvGeTorus&      setMinorRadius (double radius);
    NvGeTorus&      setAnglesInU   (double start, double end);
    NvGeTorus&      setAnglesInV   (double start, double end);
    NvGeTorus&      set            (double majorRadius, double minorRadius,
                                    const NvGePoint3d& origin,
                                    const NvGeVector3d& axisOfSymmetry);
    NvGeTorus&      set            (double majorRadius, double minorRadius,
                                    const NvGePoint3d&  origin,
                                    const NvGeVector3d& axisOfSymmetry,
                                    const NvGeVector3d& refAxis,
                                    double startAngleU, double endAngleU,
                                    double startAngleV, double endAngleV);
    // Assignment operator.
    //
    NvGeTorus&      operator =     (const NvGeTorus& torus);

    // Intersection with a linear entity
    //
    Nova::Boolean  intersectWith  (const NvGeLinearEnt3d& linEnt, int& intn,
                                    NvGePoint3d& p1, NvGePoint3d& p2,
                                    NvGePoint3d& p3, NvGePoint3d& p4,
                                    const NvGeTol& tol = NvGeContext::gTol) const;


    // The following methods classify the shape according to the
    // relationship between the major and minor radii of the torus.
    // Exactly one of the first four functions should return TRUE
    // for any given torus.
    //
    Nova::Boolean isLemon     () const;
    Nova::Boolean isApple     () const;
    Nova::Boolean isVortex    () const;
    Nova::Boolean isDoughnut  () const;
    Nova::Boolean isDegenerate() const;
    Nova::Boolean isHollow    () const;
};

#pragma pack (pop)
#endif
