#ifndef NV_GEOFFC2D_H
#define NV_GEOFFC2D_H

#include "gecurv2d.h"
#pragma pack (push, 8)

class 

NvGeOffsetCurve2d : public NvGeCurve2d
{
public:

    // Constructors
    //
    GE_DLLEXPIMPORT NvGeOffsetCurve2d (const NvGeCurve2d& baseCurve, double offsetDistance);
    GE_DLLEXPIMPORT NvGeOffsetCurve2d (const NvGeOffsetCurve2d& offsetCurve);

	// Query methods
	//
    GE_DLLEXPIMPORT const NvGeCurve2d*  curve             () const;
    GE_DLLEXPIMPORT double              offsetDistance    () const;
	GE_DLLEXPIMPORT Nova::Boolean		paramDirection    () const;
	GE_DLLEXPIMPORT NvGeMatrix2d		transformation    () const;

	// Set methods
	//
    GE_DLLEXPIMPORT NvGeOffsetCurve2d&  setCurve          (const NvGeCurve2d& baseCurve);
    GE_DLLEXPIMPORT NvGeOffsetCurve2d&  setOffsetDistance (double distance);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeOffsetCurve2d&  operator = (const NvGeOffsetCurve2d& offsetCurve);
};

#pragma pack (pop)
#endif
