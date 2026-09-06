#ifndef NV_GESPNT2d_H
#define NV_GESPNT2d_H

#include "gecurv2d.h"
#include "gekvec.h"
#include "gept2dar.h"
#include "gevec2d.h"
#include "gepnt2d.h"
#include "gept2dar.h"
#pragma pack (push, 8)

class NvGeKnotVector;

class 

NvGeSplineEnt2d : public NvGeCurve2d
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
                                             NvGeContext::gTol) const;

    GE_DLLEXPIMPORT double            startParam            () const;
    GE_DLLEXPIMPORT double            endParam              () const;
    GE_DLLEXPIMPORT NvGePoint2d       startPoint            () const;
    GE_DLLEXPIMPORT NvGePoint2d       endPoint              () const;

    // Interpolation data
    //
    GE_DLLEXPIMPORT Nova::Boolean    hasFitData            () const;

    // Editting
    //
    GE_DLLEXPIMPORT double            knotAt                (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt2d&  setKnotAt             (int idx, double val);

    GE_DLLEXPIMPORT NvGePoint2d       controlPointAt        (int idx) const;
    GE_DLLEXPIMPORT NvGeSplineEnt2d&  setControlPointAt     (int idx, const NvGePoint2d& pnt);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeSplineEnt2d&  operator =            (const NvGeSplineEnt2d& spline);

protected:
    GE_DLLEXPIMPORT NvGeSplineEnt2d ();
    GE_DLLEXPIMPORT NvGeSplineEnt2d (const NvGeSplineEnt2d&);
};

#pragma pack (pop)
#endif

