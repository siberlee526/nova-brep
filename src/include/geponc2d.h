#ifndef NV_GEPONC2D_H
#define NV_GEPONC2D_H

#include "gepent2d.h"
#pragma pack (push, 8)

class NvGeCurve2d;

class

NvGePointOnCurve2d : public NvGePointEnt2d
{
public:
    GE_DLLEXPIMPORT NvGePointOnCurve2d  ();
    GE_DLLEXPIMPORT NvGePointOnCurve2d  (const NvGeCurve2d& crv);
    GE_DLLEXPIMPORT NvGePointOnCurve2d  (const NvGeCurve2d& crv, double param);
    GE_DLLEXPIMPORT NvGePointOnCurve2d  (const NvGePointOnCurve2d& src);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePointOnCurve2d& operator =     (const NvGePointOnCurve2d& src);

    // Query functions.
    //
    GE_DLLEXPIMPORT const NvGeCurve2d*  curve          () const;
    GE_DLLEXPIMPORT double              parameter      () const;

    // Functions to evaluate a point.
    //
    GE_DLLEXPIMPORT NvGePoint2d         point          () const;
    GE_DLLEXPIMPORT NvGePoint2d         point          (double param);
    GE_DLLEXPIMPORT NvGePoint2d         point          (const NvGeCurve2d& crv, double param);

    // Functions to evaluate the derivatives.
    //
    GE_DLLEXPIMPORT NvGeVector2d        deriv          (int order) const;
    GE_DLLEXPIMPORT NvGeVector2d        deriv          (int order, double param);
    GE_DLLEXPIMPORT NvGeVector2d        deriv          (int order, const NvGeCurve2d& crv,
                                        double param);
    // Singularity
    //
    GE_DLLEXPIMPORT Adesk::Boolean      isSingular     (const NvGeTol&  tol =
                                        NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean      curvature      (double& res);
    GE_DLLEXPIMPORT Adesk::Boolean      curvature      (double param, double& res);
    // Set functions.
    //
    GE_DLLEXPIMPORT NvGePointOnCurve2d& setCurve       (const NvGeCurve2d& crv);
    GE_DLLEXPIMPORT NvGePointOnCurve2d& setParameter   (double param);
};

#pragma pack (pop)
#endif

