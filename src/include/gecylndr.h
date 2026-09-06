#ifndef NV_GECYLNDR_H
#define NV_GECYLNDR_H

#include "gegbl.h"
#include "gesurf.h"
#include "gevec3d.h"
#include "geintrvl.h"
#include "gearc3d.h"
#pragma pack (push, 8)

class NvGeCircArc3d;

class
GX_DLLEXPIMPORT
NvGeCylinder : public NvGeSurface
{
public:
    NvGeCylinder ();
    NvGeCylinder (double radius, const NvGePoint3d& origin,
                  const NvGeVector3d& axisOfSymmetry);
    NvGeCylinder (double radius, const NvGePoint3d& origin,
                  const NvGeVector3d& axisOfSymmetry,
                  const NvGeVector3d& refAxis,
                  const NvGeInterval& height,
                  double startAngle, double endAngle);
    NvGeCylinder (const NvGeCylinder&);

    // Geometric properties.
    //
    double         radius        () const;
    NvGePoint3d    origin        () const;
    void           getAngles     (double& start, double& end) const;
    void           getHeight     (NvGeInterval& height) const;
    double         heightAt      (double u) const;
    NvGeVector3d   axisOfSymmetry() const;
    NvGeVector3d   refAxis       () const;
    Adesk::Boolean isOuterNormal () const;
    Adesk::Boolean isClosed      (const NvGeTol& tol = NvGeContext::gTol) const;

    NvGeCylinder&  setRadius     (double radius);
    NvGeCylinder&  setAngles     (double start, double end);
    NvGeCylinder&  setHeight     (const NvGeInterval& height);
    NvGeCylinder&  set           (double radius, const NvGePoint3d& origin,
                                  const NvGeVector3d& axisOfSym);
    NvGeCylinder&  set           (double radius, const NvGePoint3d& origin,
                                  const NvGeVector3d& axisOfSym,
                                  const NvGeVector3d& refAxis,
                                  const NvGeInterval& height,
                                  double startAngle, double endAngle);
    // Assignment operator.
    //
    NvGeCylinder&  operator =    (const NvGeCylinder& cylinder);

    // Intersection with a linear entity
    //
    Adesk::Boolean intersectWith (const NvGeLinearEnt3d& linEnt, int& intn,
                                  NvGePoint3d& p1, NvGePoint3d& p2,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
};

#pragma pack (pop)
#endif
