#ifndef NV_GELNSG3D_H
#define NV_GELNSG3D_H

#include "geline3d.h"
#include "geplane.h"
#pragma pack (push, 8)

class NvGeLineSeg2d;

class 

NvGeLineSeg3d : public NvGeLinearEnt3d
{
public:
    GE_DLLEXPIMPORT NvGeLineSeg3d();
    GE_DLLEXPIMPORT NvGeLineSeg3d(const NvGeLineSeg3d& line);
    GE_DLLEXPIMPORT NvGeLineSeg3d(const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeLineSeg3d(const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);

    // Bisector.
    //
    GE_DLLEXPIMPORT void           getBisector  (NvGePlane& plane) const;

    // Barycentric combination of end points.
    //
    GE_DLLEXPIMPORT NvGePoint3d    baryComb     (double blendCoeff) const;

    // Definition of linear segment
    //
    GE_DLLEXPIMPORT NvGePoint3d    startPoint   () const;
    GE_DLLEXPIMPORT NvGePoint3d    midPoint     () const;
    GE_DLLEXPIMPORT NvGePoint3d    endPoint     () const;
    GE_DLLEXPIMPORT double         length       () const;
    GE_DLLEXPIMPORT double         length       (double fromParam, double toParam,
                                 double tol = NvGeContext::gTol.equalPoint()) const;
    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeLineSeg3d& set          (const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeLineSeg3d& set          (const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);
   	GE_DLLEXPIMPORT NvGeLineSeg3d& set          (const NvGeCurve3d& curve1,
                                 const NvGeCurve3d& curve2,
                                 double& param1, double& param2,
                                 Adesk::Boolean& success);
  	GE_DLLEXPIMPORT NvGeLineSeg3d& set          (const NvGeCurve3d& curve,
                                 const NvGePoint3d& point, double& param,
                                 Adesk::Boolean& success);


    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLineSeg3d& operator =   (const NvGeLineSeg3d& line);
};

#pragma pack (pop)
#endif
