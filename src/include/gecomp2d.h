#ifndef NV_GECOMP2D_H
#define NV_GECOMP2D_H

#include "gecurv2d.h"
#include "gevptar.h"
#include "geintarr.h"
#pragma pack (push, 8)

class 

NvGeCompositeCurve2d : public NvGeCurve2d
{
public:
    GE_DLLEXPIMPORT NvGeCompositeCurve2d  ();
    GE_DLLEXPIMPORT NvGeCompositeCurve2d  (const NvGeVoidPointerArray& curveList);
    GE_DLLEXPIMPORT NvGeCompositeCurve2d  (const NvGeVoidPointerArray& curveList,
		                   const NvGeIntArray& isOwnerOfCurves);
    GE_DLLEXPIMPORT NvGeCompositeCurve2d  (const NvGeCompositeCurve2d& compCurve);

    // Definition of trimmed curve
    //
    GE_DLLEXPIMPORT void		          getCurveList       (NvGeVoidPointerArray& curveList) const;

    // Set methods
    //
    GE_DLLEXPIMPORT NvGeCompositeCurve2d& setCurveList       (const NvGeVoidPointerArray& curveList);
    GE_DLLEXPIMPORT NvGeCompositeCurve2d& setCurveList       (const NvGeVoidPointerArray& curveList,
		                                      const NvGeIntArray& isOwnerOfCurves);
	
	// Convert parameter on composite to parameter on component curve and vice-versa.
	//
	GE_DLLEXPIMPORT double				  globalToLocalParam ( double param, int& crvNum ) const; 
	GE_DLLEXPIMPORT double				  localToGlobalParam ( double param, int crvNum ) const; 

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCompositeCurve2d& operator =         (const NvGeCompositeCurve2d& compCurve);
};

#pragma pack (pop)
#endif
