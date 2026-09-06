#ifndef NV_GECONE_H
#define NV_GECONE_H

#include "gegbl.h"
#include "geintrvl.h"
#include "gevec3d.h"
#include "gesurf.h"
#include "gearc3d.h"
#pragma pack (push, 8)

class NvGePoint3d;
class NvGeVector3d;
class NvGeCircArc3d;
class NvGeInterval;
class NvGeLinearEnt3d;

class 
GX_DLLEXPIMPORT
NvGeCone : public NvGeSurface
{
public:
    NvGeCone();
    NvGeCone(double cosineAngle, double sineAngle,
             const  NvGePoint3d& baseOrigin, double baseRadius,
             const  NvGeVector3d& axisOfSymmetry);
    NvGeCone(double cosineAngle, double sineAngle,
             const  NvGePoint3d& baseOrigin, double baseRadius,
             const  NvGeVector3d& axisOfSymmetry,
             const  NvGeVector3d& refAxis, const  NvGeInterval& height,
             double startAngle, double endAngle);
    NvGeCone(const NvGeCone& cone);

    // Geometric properties.
    //
    double           baseRadius        () const;
    NvGePoint3d      baseCenter        () const;
    void             getAngles         (double& start, double& end) const;
    double           halfAngle         () const;
    void             getHalfAngle      (double& cosineAngle, double& sineAngle)
                                                 const;
    void             getHeight         (NvGeInterval& range) const;
    double           heightAt          (double u) const;
    NvGeVector3d     axisOfSymmetry    () const;
    NvGeVector3d     refAxis           () const;
    NvGePoint3d      apex              () const;
    Nova::Boolean   isClosed          (const NvGeTol& tol = NvGeContext::gTol) const;
    Nova::Boolean   isOuterNormal     () const;


    NvGeCone&        setBaseRadius     (double radius);
    NvGeCone&        setAngles         (double startAngle, double endAngle);
    NvGeCone&        setHeight         (const NvGeInterval& height);
    NvGeCone&        set               (double cosineAngle, double sineAngle,
                                        const  NvGePoint3d& baseCenter,
                                        double baseRadius,
                                        const  NvGeVector3d& axisOfSymmetry);
    NvGeCone&        set               (double cosineAngle, double sineAngle,
                                        const  NvGePoint3d& baseCenter,
                                        double baseRadius,
                                        const  NvGeVector3d& axisOfSymmetry,
                                        const  NvGeVector3d& refAxis,
                                        const  NvGeInterval& height,
                                        double startAngle, double endAngle);
    // Assignment operator.
    //
    NvGeCone&        operator =        (const NvGeCone& cone);

    // Intersection with a linear entity
    //
    Nova::Boolean   intersectWith     (const NvGeLinearEnt3d& linEnt, int& intn,
                                        NvGePoint3d& p1, NvGePoint3d& p2,
                                        const NvGeTol& tol = NvGeContext::gTol) const;
};


#pragma pack (pop)
#endif
