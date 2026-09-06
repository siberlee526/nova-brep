#ifndef NV_GEPLIN3D_H
#define NV_GEPLIN3D_H

#include "gecurv3d.h"
#include "gekvec.h"
#include "gept3dar.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "gesent3d.h"
#pragma pack (push, 8)


class

NvGePolyline3d : public NvGeSplineEnt3d
{
public:

    GE_DLLEXPIMPORT NvGePolyline3d();
    GE_DLLEXPIMPORT NvGePolyline3d( const NvGePolyline3d& src );
    GE_DLLEXPIMPORT NvGePolyline3d( const NvGePoint3dArray& points);
    GE_DLLEXPIMPORT NvGePolyline3d( const NvGeKnotVector& knots,
                    const NvGePoint3dArray& cntrlPnts );

    // Approximate curve with polyline
    //
    GE_DLLEXPIMPORT NvGePolyline3d( const NvGeCurve3d& crv, double apprEps );

    // Interpolation data
    //
    GE_DLLEXPIMPORT int              numFitPoints () const;
    GE_DLLEXPIMPORT NvGePoint3d      fitPointAt   (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt3d& setFitPointAt(int idx, const NvGePoint3d& point);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePolyline3d& operator =     (const NvGePolyline3d& pline);
};

#pragma pack (pop)
#endif

