#ifndef NV_GEEXTC3D_H
#define NV_GEEXTC3D_H

#include "gecurv3d.h"
#include "gearc3d.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#pragma pack (push, 8)

class NvGeLine3d;
class NvGeLineSeg3d;
class NvGeRay3d;
class NvGeEllipArc3d;
class NvGeNurbCurve3d;
class NvGeExternalCurve2d;
class NvGeExternalCurve3d;

class
GX_DLLEXPIMPORT
NvGeExternalCurve3d : public NvGeCurve3d
{
public:
    NvGeExternalCurve3d();
    NvGeExternalCurve3d(const NvGeExternalCurve3d& src);
    NvGeExternalCurve3d(void* curveDef, NvGe::ExternalEntityKind curveKind,
                            Nova::Boolean makeCopy = Nova::kTrue);

    Nova::Boolean isLine     () const;
    Nova::Boolean isRay      () const;
    Nova::Boolean isLineSeg  () const;
    Nova::Boolean isCircArc  () const;
    Nova::Boolean isEllipArc () const;
    Nova::Boolean isNurbCurve() const;
    Nova::Boolean isDefined  () const;

    // Conversion to native gelib curve
    //
    Nova::Boolean isNativeCurve  (NvGeCurve3d*& nativeCurve) const;
    void           getExternalCurve (void*& curveDef) const;

    // Type of the external curve.
    //
    NvGe::ExternalEntityKind externalCurveKind() const;

    // Reset surface
    //
    NvGeExternalCurve3d& set(void* curveDef, NvGe::ExternalEntityKind curveKind,
                             Nova::Boolean makeCopy = Nova::kTrue);
    // Assignment operator
    //
    NvGeExternalCurve3d& operator = (const NvGeExternalCurve3d& src);

    // Ownership of curve
    //
    Nova::Boolean       isOwnerOfCurve   () const;
    NvGeExternalCurve3d& setToOwnCurve    ();
};

#pragma pack (pop)
#endif
