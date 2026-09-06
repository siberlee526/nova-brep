#ifndef NV_GESPHERE_H
#define NV_GESPHERE_H

#include "gegbl.h"
#include "gepnt3d.h"
#include "gevec3d.h"
#include "geintrvl.h"
#include "gesurf.h"
#pragma pack (push, 8)

class NvGeCircArc3d;

class
GX_DLLEXPIMPORT
NvGeSphere : public NvGeSurface
{
public:
    NvGeSphere();
    NvGeSphere(double radius, const NvGePoint3d& center);
    NvGeSphere(double radius, const NvGePoint3d& center,
               const NvGeVector3d& northAxis, const NvGeVector3d& refAxis,
               double startAngleU, double endAngleU,
               double startAngleV, double endAngleV);
    NvGeSphere(const NvGeSphere& sphere);

    // Geometric properties.
    //
    double         radius            () const;
    NvGePoint3d    center            () const;
    void           getAnglesInU      (double& start, double& end) const;
    void           getAnglesInV      (double& start, double& end) const;
    NvGeVector3d   northAxis         () const;
    NvGeVector3d   refAxis           () const;
    NvGePoint3d    northPole         () const;
    NvGePoint3d    southPole         () const;
    Nova::Boolean isOuterNormal     () const;
    Nova::Boolean isClosed       (const NvGeTol& tol = NvGeContext::gTol) const;

    NvGeSphere&    setRadius         (double);
    NvGeSphere&    setAnglesInU      (double start, double end);
    NvGeSphere&    setAnglesInV      (double start, double end);
    NvGeSphere&    set               (double radius, const NvGePoint3d& center);
    NvGeSphere&    set               (double radius, const NvGePoint3d& center,
                                      const NvGeVector3d& northAxis,
                                      const NvGeVector3d& refAxis,
                                      double startAngleU,
                                      double endAngleU,
                                      double startAngleV,
                                      double endAngleV);
    // Assignment operator.
    //
    NvGeSphere&    operator =     (const NvGeSphere& sphere);

    // Intersection with a linear entity
    //
    Nova::Boolean intersectWith  (const NvGeLinearEnt3d&, int& intn,
                                   NvGePoint3d& p1, NvGePoint3d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
};

#pragma pack (pop)
#endif
