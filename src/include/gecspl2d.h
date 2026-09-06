#ifndef NV_GECSPL2D_H
#define NV_GECSPL2D_H

class NvGePointOnCurve2d;
class NvGeCurve2dIntersection;
class NvGeInterval;
class NvGePlane;

#include "gesent2d.h"
#include "gept2dar.h"
#include "gevc2dar.h"
#include "gevec2d.h"
#include "gekvec.h"
#pragma pack (push, 8)

class

NvGeCubicSplineCurve2d : public NvGeSplineEnt2d
{
public:

    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d();
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d(const NvGeCubicSplineCurve2d& spline);

    // Construct a periodic cubic spline curve
    // Contract: the first fit point must be equal to the last fit point
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d(const NvGePoint2dArray& fitPnts,
			   const NvGeTol& tol = NvGeContext::gTol);

    // Construct a cubic spline curve with clamped end condition
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d(const NvGePoint2dArray& fitPnts,
			   const NvGeVector2d& startDeriv,
			   const NvGeVector2d& endDeriv,
                           const NvGeTol& tol = NvGeContext::gTol);

    // Construct a cubic spline approximating the curve
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d(const NvGeCurve2d& curve,
                           double epsilon = NvGeContext::gTol.equalPoint());

    // Construct a cubic spline curve with given fit points and 1st derivatives
    //
	GE_DLLEXPIMPORT NvGeCubicSplineCurve2d(const NvGeKnotVector& knots,
	                       const NvGePoint2dArray& fitPnts,
			       const NvGeVector2dArray& firstDerivs,
	                       Adesk::Boolean isPeriodic = Adesk::kFalse );

    // Definition of spline
    //
    GE_DLLEXPIMPORT int                     numFitPoints   ()        const;
    GE_DLLEXPIMPORT NvGePoint2d             fitPointAt     (int idx) const;
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d& setFitPointAt  (int idx, const NvGePoint2d& point);
    GE_DLLEXPIMPORT NvGeVector2d            firstDerivAt   (int idx) const;
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d& setFirstDerivAt(int idx, const NvGeVector2d& deriv);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve2d&  operator = (const NvGeCubicSplineCurve2d& spline);
};

#pragma pack (pop)
#endif
