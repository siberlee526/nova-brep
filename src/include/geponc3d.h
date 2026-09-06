#ifndef NV_GEPONC3D_H
#define NV_GEPONC3D_H

#include "gepent3d.h"
#pragma pack (push, 8)

class NvGeCurve3d;

class

NvGePointOnCurve3d : public NvGePointEnt3d
{
public:
    GE_DLLEXPIMPORT NvGePointOnCurve3d();
    GE_DLLEXPIMPORT NvGePointOnCurve3d(const NvGeCurve3d& crv);
    GE_DLLEXPIMPORT NvGePointOnCurve3d(const NvGeCurve3d& crv, double param);
    GE_DLLEXPIMPORT NvGePointOnCurve3d(const NvGePointOnCurve3d& src);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePointOnCurve3d& operator =     (const NvGePointOnCurve3d& src);

    // Query functions.
    //
    GE_DLLEXPIMPORT const NvGeCurve3d*  curve          () const;
    GE_DLLEXPIMPORT double              parameter      () const;

    // Functions to evaluate a point.
    //
    GE_DLLEXPIMPORT NvGePoint3d         point          () const;
    GE_DLLEXPIMPORT NvGePoint3d         point          (double param);
    GE_DLLEXPIMPORT NvGePoint3d         point          (const NvGeCurve3d& crv, double param);

    // Functions to evaluate the derivatives.
    //
    GE_DLLEXPIMPORT NvGeVector3d        deriv          (int order) const;
    GE_DLLEXPIMPORT NvGeVector3d        deriv          (int order, double param);
    GE_DLLEXPIMPORT NvGeVector3d        deriv          (int order, const NvGeCurve3d& crv,
                                        double param);
    // Singularity
    //
    GE_DLLEXPIMPORT Nova::Boolean      isSingular     (const NvGeTol& tol =
	                                NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean     	curvature      (double& res);
    GE_DLLEXPIMPORT Nova::Boolean     	curvature      (double param, double& res);

    // Set functions.
    //
    GE_DLLEXPIMPORT NvGePointOnCurve3d& setCurve       (const NvGeCurve3d& crv);
    GE_DLLEXPIMPORT NvGePointOnCurve3d& setParameter   (double param);
};

#pragma pack (pop)
#endif

