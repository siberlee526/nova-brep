#ifndef NV_GESURF_H
#define NV_GESURF_H

#include "geent3d.h"
#include "gevc3dar.h"
#pragma pack (push, 8)

class NvGePoint2d;
class NvGeCurve3d;
class NvGePointOnCurve3d;
class NvGePointOnSurface;
class NvGePointOnSurfaceData;
class NvGeInterval;


class

NvGeSurface : public NvGeEntity3d
{
public:
    // Parameter related.
    //
    GE_DLLEXPIMPORT NvGePoint2d     paramOf        (const NvGePoint3d& pnt,
                                    const NvGeTol& tol = NvGeContext::gTol) const;
    // Point containment
    //
    GE_DLLEXPIMPORT Adesk::Boolean  isOn           (const NvGePoint3d& pnt,
                                    const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean  isOn           (const NvGePoint3d& pnt, NvGePoint2d& paramPoint,
                                    const NvGeTol& tol = NvGeContext::gTol ) const;
    // Operations.
    //
    GE_DLLEXPIMPORT NvGePoint3d closestPointTo(const NvGePoint3d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    GE_DLLEXPIMPORT void getClosestPointTo(const NvGePoint3d& pnt, NvGePointOnSurface& result,
                           const NvGeTol& tol = NvGeContext::gTol) const;
	
    GE_DLLEXPIMPORT double          distanceTo     (const NvGePoint3d& pnt,
                                    const NvGeTol& tol = NvGeContext::gTol) const;

    GE_DLLEXPIMPORT Adesk::Boolean  isNormalReversed () const;
    GE_DLLEXPIMPORT NvGeSurface&    reverseNormal    ();

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeSurface&    operator =     (const NvGeSurface& otherSurface);

    // Bounds in parameter space.
    //
    GE_DLLEXPIMPORT void   getEnvelope  (NvGeInterval& intrvlX, NvGeInterval& intrvlY) const;

    // Geometric inquiry methods.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isClosedInU       (const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isClosedInV       (const NvGeTol& tol = NvGeContext::gTol) const;

    // Evaluators.
    // Derivative arrays are indexed partialU, partialV followed by
    // the mixed partial.
    //
    GE_DLLEXPIMPORT NvGePoint3d   evalPoint   (const NvGePoint2d& param) const;
    GE_DLLEXPIMPORT NvGePoint3d   evalPoint   (const NvGePoint2d& param, int derivOrd,
                               NvGeVector3dArray& derivatives) const;
    GE_DLLEXPIMPORT NvGePoint3d   evalPoint   (const NvGePoint2d& param, int derivOrd,
                               NvGeVector3dArray& derivatives,
                               NvGeVector3d& normal) const;
protected:
    GE_DLLEXPIMPORT NvGeSurface ();
    GE_DLLEXPIMPORT NvGeSurface (const NvGeSurface&);
};

#pragma pack (pop)
#endif
