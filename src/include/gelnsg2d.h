#ifndef NV_GELNSG2D_H
#define NV_GELNSG2D_H

#include "geline2d.h"
#pragma pack (push, 8)

class

NvGeLineSeg2d : public NvGeLinearEnt2d
{
public:
    GE_DLLEXPIMPORT NvGeLineSeg2d();
    GE_DLLEXPIMPORT NvGeLineSeg2d(const NvGeLineSeg2d& line);
    GE_DLLEXPIMPORT NvGeLineSeg2d(const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);
    GE_DLLEXPIMPORT NvGeLineSeg2d(const NvGePoint2d& pnt, const NvGeVector2d& vec);

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeLineSeg2d& set(const NvGePoint2d& pnt, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeLineSeg2d& set(const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);
    GE_DLLEXPIMPORT NvGeLineSeg2d& set(const NvGeCurve2d& curve1,
                       const NvGeCurve2d& curve2,
                       double& param1, double& param2,
                       Nova::Boolean& success);
    GE_DLLEXPIMPORT NvGeLineSeg2d& set(const NvGeCurve2d& curve, const NvGePoint2d& point,
                       double& param, Nova::Boolean& success);


    // Bisector.
    //
    GE_DLLEXPIMPORT void           getBisector(NvGeLine2d& line) const;

    // Barycentric combination of end points.
    //
    GE_DLLEXPIMPORT NvGePoint2d    baryComb   (double blendCoeff) const;

    // Definition of linear segment
    //
    GE_DLLEXPIMPORT NvGePoint2d    startPoint   () const;
    GE_DLLEXPIMPORT NvGePoint2d    midPoint     () const;
    GE_DLLEXPIMPORT NvGePoint2d    endPoint     () const;
    GE_DLLEXPIMPORT double         length       () const;
    GE_DLLEXPIMPORT double         length       (double fromParam, double toParam,
                                 double tol = NvGeContext::gTol.equalPoint())
                                const;
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLineSeg2d& operator =  (const NvGeLineSeg2d& line);
};

#pragma pack (pop)
#endif
