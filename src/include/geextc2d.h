#ifndef NV_GEEXTC2D_H
#define NV_GEEXTC2D_H

#include "gecurv2d.h"
#include "gearc2d.h"
#include "gevec2d.h"
#include "gepnt2d.h"
#pragma pack (push, 8)

class NvGeNurbCurve2d;
class NvGeExternalCurve2d;
class NvGeExternalCurve2d;

class
GX_DLLEXPIMPORT
NvGeExternalCurve2d : public NvGeCurve2d
{
public:
    NvGeExternalCurve2d();
    NvGeExternalCurve2d(const NvGeExternalCurve2d&);
    NvGeExternalCurve2d(void* curveDef, NvGe::ExternalEntityKind curveKind,
                        Nova::Boolean makeCopy = Nova::kTrue);

    Nova::Boolean isNurbCurve() const;
    Nova::Boolean isNurbCurve(NvGeNurbCurve2d& nurbCurve) const;
    Nova::Boolean isDefined  () const;

    void           getExternalCurve(void*& curveDef) const;

    // Type of the external curve.
    //
    NvGe::ExternalEntityKind externalCurveKind() const;

    // Reset surface
    //
    NvGeExternalCurve2d& set(void* curveDef, NvGe::ExternalEntityKind curveKind,
                             Nova::Boolean makeCopy = Nova::kTrue);
    // Assignment operator
    //
    NvGeExternalCurve2d& operator = (const NvGeExternalCurve2d& src);

    // Ownership of curve
    //
    Nova::Boolean       isOwnerOfCurve() const;
    NvGeExternalCurve2d& setToOwnCurve();
};

#pragma pack (pop)
#endif
