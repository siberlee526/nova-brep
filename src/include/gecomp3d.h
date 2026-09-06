#ifndef NV_GECOMP3D_H
#define NV_GECOMP3D_H

#include "gecurv3d.h"
#include "gevptar.h"
#include "geintarr.h"
#pragma pack (push, 8)

class 

NvGeCompositeCurve3d : public NvGeCurve3d
{
public:
    GE_DLLEXPIMPORT NvGeCompositeCurve3d  ();
    GE_DLLEXPIMPORT NvGeCompositeCurve3d  (const NvGeVoidPointerArray& curveList);
    GE_DLLEXPIMPORT NvGeCompositeCurve3d  (const NvGeVoidPointerArray& curveList,
		                   const NvGeIntArray& isOwnerOfCurves);
    GE_DLLEXPIMPORT NvGeCompositeCurve3d  (const NvGeCompositeCurve3d& compCurve);

    // Definition of trimmed curve
    //
    GE_DLLEXPIMPORT void		          getCurveList       (NvGeVoidPointerArray& curveList) const;

    // Set methods
    //
    GE_DLLEXPIMPORT NvGeCompositeCurve3d& setCurveList       (const NvGeVoidPointerArray& curveList);
    GE_DLLEXPIMPORT NvGeCompositeCurve3d& setCurveList       (const NvGeVoidPointerArray& curveList,
		                                      const NvGeIntArray& isOwnerOfCurves);
	
	// Convert parameter on composite to parameter on component curve and vice-versa.
	//
	GE_DLLEXPIMPORT double				  globalToLocalParam ( double param, int& segNum ) const; 
	GE_DLLEXPIMPORT double				  localToGlobalParam ( double param, int segNum ) const; 

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCompositeCurve3d& operator =         (const NvGeCompositeCurve3d& compCurve);
};

#pragma pack (pop)
#endif
