#ifndef NV_GECSPL3D_H
#define NV_GECSPL3D_H

class NvGePointOnCurve3d;
class NvGeCurve3dIntersection;
class NvGeInterval;
class NvGePlane;

#include "gesent3d.h"
#include "gept3dar.h"
#include "gevc3dar.h"
#include "gevec3d.h"
#include "gekvec.h"
#pragma pack (push, 8)

class

NvGeCubicSplineCurve3d : public NvGeSplineEnt3d
{
public:

    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d();
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d(const NvGeCubicSplineCurve3d& spline);

    // Construct a periodic cubic spline curve
    // Contract: the first fit point must be equal to the last fit point
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d(const NvGePoint3dArray& fitPnts,
			   const NvGeTol& tol = NvGeContext::gTol);

    // Construct a cubic spline curve with clamped end condition
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d(const NvGePoint3dArray& fitPnts,
			   const NvGeVector3d& startDeriv,
			   const NvGeVector3d& endDeriv,
                           const NvGeTol& tol = NvGeContext::gTol);

    // Construct a cubic spline approximating the curve
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d(const NvGeCurve3d& curve,
                           double epsilon = NvGeContext::gTol.equalPoint());

    // Construct a cubic spline curve with given fit points and 1st derivatives
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d(const NvGeKnotVector& knots,
	                   const NvGePoint3dArray& fitPnts,
			   const NvGeVector3dArray& firstDerivs,
	                   Adesk::Boolean isPeriodic = Adesk::kFalse);

    // Definition of spline
    //
    GE_DLLEXPIMPORT int                     numFitPoints   ()        const;
    GE_DLLEXPIMPORT NvGePoint3d             fitPointAt     (int idx) const;
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d& setFitPointAt  (int idx, const NvGePoint3d& point);
    GE_DLLEXPIMPORT NvGeVector3d            firstDerivAt   (int idx) const;
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d& setFirstDerivAt(int idx, const NvGeVector3d& deriv);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCubicSplineCurve3d&  operator = (const NvGeCubicSplineCurve3d& spline);
};

#pragma pack (pop)
#endif
