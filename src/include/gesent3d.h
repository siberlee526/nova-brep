#ifndef NV_GESPNT3D_H
#define NV_GESPNT3D_H

#include "gecurv3d.h"
#include "gekvec.h"
#include "gept3dar.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "gept3dar.h"
#pragma pack (push, 8)

class NvGeKnotVector;

class

NvGeSplineEnt3d : public NvGeCurve3d
{
public:
    // Definition of spline
    //
    GE_DLLEXPIMPORT Nova::Boolean    isRational            () const;
    GE_DLLEXPIMPORT int               degree                () const;
    GE_DLLEXPIMPORT int               order                 () const;
    GE_DLLEXPIMPORT int               numKnots              () const;
    GE_DLLEXPIMPORT const
    NvGeKnotVector&   knots                 () const;
    GE_DLLEXPIMPORT int               numControlPoints      () const;
    GE_DLLEXPIMPORT int               continuityAtKnot      (int idx, const NvGeTol& tol =
                                             NvGeContext::gTol ) const;

    GE_DLLEXPIMPORT double            startParam            () const;
    GE_DLLEXPIMPORT double            endParam              () const;
    GE_DLLEXPIMPORT NvGePoint3d       startPoint            () const;
    GE_DLLEXPIMPORT NvGePoint3d       endPoint              () const;

    // Interpolation data
    //
    GE_DLLEXPIMPORT Nova::Boolean    hasFitData            () const;

    // Editting
    //
    GE_DLLEXPIMPORT double            knotAt                (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt3d&  setKnotAt             (int idx, double val);

    GE_DLLEXPIMPORT NvGePoint3d       controlPointAt        (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt3d&  setControlPointAt     (int idx, const NvGePoint3d& pnt);


    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeSplineEnt3d&  operator =            (const NvGeSplineEnt3d& spline);

protected:
    GE_DLLEXPIMPORT NvGeSplineEnt3d ();
    GE_DLLEXPIMPORT NvGeSplineEnt3d (const NvGeSplineEnt3d&);
};

#pragma pack (pop)
#endif

