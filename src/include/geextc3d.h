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
                            Adesk::Boolean makeCopy = Adesk::kTrue);

    Adesk::Boolean isLine     () const;
    Adesk::Boolean isRay      () const;
    Adesk::Boolean isLineSeg  () const;
    Adesk::Boolean isCircArc  () const;
    Adesk::Boolean isEllipArc () const;
    Adesk::Boolean isNurbCurve() const;
    Adesk::Boolean isDefined  () const;

    // Conversion to native gelib curve
    //
    Adesk::Boolean isNativeCurve  (NvGeCurve3d*& nativeCurve) const;
    void           getExternalCurve (void*& curveDef) const;

    // Type of the external curve.
    //
    NvGe::ExternalEntityKind externalCurveKind() const;

    // Reset surface
    //
    NvGeExternalCurve3d& set(void* curveDef, NvGe::ExternalEntityKind curveKind,
                             Adesk::Boolean makeCopy = Adesk::kTrue);
    // Assignment operator
    //
    NvGeExternalCurve3d& operator = (const NvGeExternalCurve3d& src);

    // Ownership of curve
    //
    Adesk::Boolean       isOwnerOfCurve   () const;
    NvGeExternalCurve3d& setToOwnCurve    ();
};

#pragma pack (pop)
#endif
