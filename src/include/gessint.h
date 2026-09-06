#ifndef NV_GESSINT_H
#define NV_GESSINT_H

#include "Nova.h"
#include "geent3d.h"
#include "gegbl.h"
#include "gegblabb.h"
#pragma pack (push, 8)

class NvGeCurve3d;
class NvGeCurve2d;

class NvGeSurface;

// The intersection class constructor references surface objects, but the
// intersection object does not own them.  The surface objects are linked to the
// intersection object.  On deletion or modification of one of them, internal
// intersection results are marked as invalid and to be re-computed.
//
// Computation of the intersection does not happen on construction or set(), but
// on demand from one of the query functions..
//
// Any output geometry from an intersection object is owned by the caller.  The
// const base objects returned by surface1() and surface2() are not considered
// output objects.
//

class  
GX_DLLEXPIMPORT
NvGeSurfSurfInt  : public NvGeEntity3d
{

public:
    // Constructors.
    //
    NvGeSurfSurfInt ();
    NvGeSurfSurfInt (
						const NvGeSurface& srf1, 
						const NvGeSurface& srf2,
						const NvGeTol& tol = NvGeContext::gTol );
    NvGeSurfSurfInt (const NvGeSurfSurfInt& src);

    // General query functions.
    //
    const NvGeSurface  *surface1        () const;
    const NvGeSurface  *surface2        () const;
    NvGeTol            tolerance        () const;

    // Intersection query methods.
    //
    int                numResults (NvGe::NvGeIntersectError& err) const;
                    // Counts the number of intersection results of any dimension.
    NvGeCurve3d*   intCurve (int intNum, Nova::Boolean isExternal, NvGe::NvGeIntersectError& err) const; 
					// Returns NULL if the dimension of  this intersection is not 1.
    NvGeCurve2d*   intParamCurve(int num, Nova::Boolean isExternal, Nova::Boolean isFirst, NvGe::NvGeIntersectError& err) const;
					// Returns NULL if the dimension of  this intersection is not 1.
                    // if isFirst returns parameter curve on 1st surface, otherwise returns parameter curve on 2nd surface.
	NvGePoint3d  intPoint (int intNum, NvGe::NvGeIntersectError& err) const;
					//  Invalid return if the dimension of this intersection is not 0.
    void               getIntPointParams (int intNum,
                                         NvGePoint2d& param1, NvGePoint2d& param2, NvGe::NvGeIntersectError& err) const;
	void getIntConfigs (int intNum, NvGe::ssiConfig& surf1Left,  NvGe::ssiConfig& surf1Right, 
							NvGe::ssiConfig& surf2Left,  NvGe::ssiConfig& surf2Right,  
							NvGe::ssiType& intType, int& dim, NvGe::NvGeIntersectError& err ) const; 
    int		getDimension (int intNum, NvGe::NvGeIntersectError& err) const;
	NvGe::ssiType	getType(int intNum, NvGe::NvGeIntersectError& err ) const;

    // Set functions.
    //
    NvGeSurfSurfInt& set (const NvGeSurface& srf1,
                                     const NvGeSurface& srf2,
                                     const NvGeTol& tol = NvGeContext::gTol);

    // Assignment operator.
    //
    NvGeSurfSurfInt& operator = (const NvGeSurfSurfInt& crvInt);
};
#pragma pack (pop)
#endif
