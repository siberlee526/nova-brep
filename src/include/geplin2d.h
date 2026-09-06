#ifndef NV_GEPLIN2D_H
#define NV_GEPLIN2D_H

#include "gecurv2d.h"
#include "gekvec.h"
#include "gept2dar.h"
#include "gevec2d.h"
#include "gepnt2d.h"
#include "gesent2d.h"
#pragma pack (push, 8)

class

NvGePolyline2d : public NvGeSplineEnt2d
{
public:
    GE_DLLEXPIMPORT NvGePolyline2d();
    GE_DLLEXPIMPORT NvGePolyline2d(const NvGePolyline2d& src);
    GE_DLLEXPIMPORT NvGePolyline2d(const NvGePoint2dArray&);
    GE_DLLEXPIMPORT NvGePolyline2d(const NvGeKnotVector& knots,
                   const NvGePoint2dArray& points);

    // Approximate curve with polyline
    //
    GE_DLLEXPIMPORT NvGePolyline2d(const NvGeCurve2d& crv, double apprEps);

    // Interpolation data
    //
    GE_DLLEXPIMPORT int              numFitPoints () const;
    GE_DLLEXPIMPORT NvGePoint2d      fitPointAt   (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt2d& setFitPointAt(int idx, const NvGePoint2d& point);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePolyline2d& operator =     (const NvGePolyline2d& pline);
};

#pragma pack (pop)
#endif
