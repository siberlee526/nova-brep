#ifndef NV_GEOFFC3D_H
#define NV_GEOFFC3D_H

#include "gecurv3d.h"
#pragma pack (push, 8)

class 

NvGeOffsetCurve3d : public NvGeCurve3d
{
public:

    // Constructors
    //
    GE_DLLEXPIMPORT NvGeOffsetCurve3d (const NvGeCurve3d& baseCurve, const NvGeVector3d& planeNormal,
                       double offsetDistance);
    GE_DLLEXPIMPORT NvGeOffsetCurve3d (const NvGeOffsetCurve3d& offsetCurve);

	// Query methods
	//
    GE_DLLEXPIMPORT const NvGeCurve3d*  curve             () const;
    GE_DLLEXPIMPORT NvGeVector3d        normal            () const; 
    GE_DLLEXPIMPORT double              offsetDistance    () const;
	GE_DLLEXPIMPORT Adesk::Boolean		paramDirection    () const;
	GE_DLLEXPIMPORT NvGeMatrix3d		transformation    () const;

	// Set methods
	//
    GE_DLLEXPIMPORT NvGeOffsetCurve3d&  setCurve          (const NvGeCurve3d& baseCurve);
    GE_DLLEXPIMPORT NvGeOffsetCurve3d&  setNormal         (const NvGeVector3d& planeNormal);
    GE_DLLEXPIMPORT NvGeOffsetCurve3d&  setOffsetDistance (double offsetDistance);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeOffsetCurve3d&  operator = (const NvGeOffsetCurve3d& offsetCurve);
};

#pragma pack (pop)
#endif
