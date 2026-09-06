#ifndef NV_GECSINT_H
#define NV_GECSINT_H

#include "Nova.h"
#include "geent3d.h"
#include "geponc3d.h"
#include "geponsrf.h"
#include "geintrvl.h"
#include "gegbl.h"
#include "gegblabb.h"
#pragma pack (push, 8)

class NvGeCurve3d;

class NvGeSurface;

// The intersection class constructor references curve and surface objects, but the
// intersection object does not own them.  The curve and surface objects are linked to
// the intersection object.  On deletion or modification of one of them, internal
// intersection results are marked as invalid and to be re-computed.
//
// Computation of the intersection does not happen on construction or set(), but
// on demand from one of the query functions.
//
// Any output geometry from an intersection object is owned by the caller. The
// const base objects returned by curve() and surface() are not considered
// output objects.
//

class  
GX_DLLEXPIMPORT
NvGeCurveSurfInt : public NvGeEntity3d
{

public:
    // Constructors.
    //
    NvGeCurveSurfInt ();
    NvGeCurveSurfInt (const NvGeCurve3d& crv, const NvGeSurface& srf,
                         const NvGeTol& tol = NvGeContext::gTol );
    NvGeCurveSurfInt (const NvGeCurveSurfInt& src);

    // General query functions.
    //
    const NvGeCurve3d  *curve           () const;
    const NvGeSurface  *surface         () const;
    NvGeTol            tolerance        () const;

    // Intersection query methods.
    //
    int  numIntPoints (NvGeIntersectError& err) const;
    NvGePoint3d  intPoint (int intNum, NvGeIntersectError& err) const;
    void               getIntParams (int intNum,
                                         double& param1, NvGePoint2d& param2, NvGeIntersectError& err) const;
    void               getPointOnCurve (int intNum, NvGePointOnCurve3d&, NvGeIntersectError& err) const;
    void               getPointOnSurface (int intNum, NvGePointOnSurface&, NvGeIntersectError& err) const;
    void			   getIntConfigs (int intNum, NvGe::csiConfig& lower, 
								NvGe::csiConfig& higher, Adesk::Boolean& smallAngle, NvGeIntersectError& err) const;

        
   
    // Set functions.
    //
    NvGeCurveSurfInt& set (const NvGeCurve3d& cvr,
                                     const NvGeSurface& srf,
                                     const NvGeTol& tol = NvGeContext::gTol);

    // Assignment operator.
    //
    NvGeCurveSurfInt& operator = (const NvGeCurveSurfInt& crvInt);
};
#pragma pack (pop)
#endif
