#ifndef NV_GEPONSRF_H
#define NV_GEPONSRF_H

#include "gepent3d.h"
#pragma pack (push, 8)

class NvGeSurface;

class

NvGePointOnSurface : public NvGePointEnt3d
{
public:
    GE_DLLEXPIMPORT NvGePointOnSurface();
    GE_DLLEXPIMPORT NvGePointOnSurface(const NvGeSurface& surf);
    GE_DLLEXPIMPORT NvGePointOnSurface(const NvGeSurface& surf, const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGePointOnSurface(const NvGePointOnSurface& src);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePointOnSurface& operator =     (const NvGePointOnSurface& src);

    // Query functions.
    //
    GE_DLLEXPIMPORT const NvGeSurface*  surface        () const;
    GE_DLLEXPIMPORT NvGePoint2d         parameter      () const;

    // Functions to evaluate a point.
    //
    GE_DLLEXPIMPORT NvGePoint3d         point          () const;
    GE_DLLEXPIMPORT NvGePoint3d         point          (const NvGePoint2d& param );
    GE_DLLEXPIMPORT NvGePoint3d         point          (const NvGeSurface& surf,
                                        const NvGePoint2d& param);

    // Functions to evaluate surface normal.
    //
    GE_DLLEXPIMPORT NvGeVector3d        normal         () const;
    GE_DLLEXPIMPORT NvGeVector3d        normal         (const NvGePoint2d& param );
    GE_DLLEXPIMPORT NvGeVector3d        normal         (const NvGeSurface& surf,
                                        const NvGePoint2d& param);
    // Functions to evaluate derivatives.
    //
    GE_DLLEXPIMPORT NvGeVector3d        uDeriv         (int order) const;
    GE_DLLEXPIMPORT NvGeVector3d        uDeriv         (int order, const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGeVector3d        uDeriv         (int order, const NvGeSurface& surf,
                                        const NvGePoint2d& param);

    GE_DLLEXPIMPORT NvGeVector3d        vDeriv         (int order) const;
    GE_DLLEXPIMPORT NvGeVector3d        vDeriv         (int order, const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGeVector3d        vDeriv         (int order, const NvGeSurface& surf,
                                        const NvGePoint2d& param);

    // Functions to evaluate the mixed partial.
    //
    GE_DLLEXPIMPORT NvGeVector3d        mixedPartial   () const;
    GE_DLLEXPIMPORT NvGeVector3d        mixedPartial   (const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGeVector3d        mixedPartial   (const NvGeSurface& surf,
                                        const NvGePoint2d& param);

    // Functions to compute the tangent vector in a given direction.
    //
    GE_DLLEXPIMPORT NvGeVector3d        tangentVector  (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT NvGeVector3d        tangentVector  (const NvGeVector2d& vec,
                                        const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGeVector3d        tangentVector  (const NvGeVector2d& vec,
                                        const NvGeSurface& vecSurf,
                                        const NvGePoint2d& param);

    // Functions to invert a tangent vector to parameter space.
    //
    GE_DLLEXPIMPORT NvGeVector2d        inverseTangentVector  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGeVector2d        inverseTangentVector  (const NvGeVector3d& vec,
                                               const NvGePoint2d& param);
    GE_DLLEXPIMPORT NvGeVector2d        inverseTangentVector  (const NvGeVector3d& vec,
                                               const NvGeSurface& surf,
                                               const NvGePoint2d& param);
    // Set functions.
    //
    GE_DLLEXPIMPORT NvGePointOnSurface& setSurface     (const NvGeSurface& surf);
    GE_DLLEXPIMPORT NvGePointOnSurface& setParameter   (const NvGePoint2d& param);
};

#pragma pack (pop)
#endif

