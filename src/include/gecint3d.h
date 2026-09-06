#ifndef NV_GECINT3D_H
#define NV_GECINT3D_H

#include "Nova.h"
#include "geent3d.h"
#include "geponc3d.h"
#include "geintrvl.h"
#pragma pack (push, 8)

class NvGeCurve3d;


class  

NvGeCurveCurveInt3d : public NvGeEntity3d
{

public:
    // Constructors.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d ();
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d (const NvGeCurve3d& curve1, const NvGeCurve3d& curve2,
		                 const NvGeVector3d& planeNormal =
						 NvGeVector3d::kIdentity,
                         const NvGeTol& tol = NvGeContext::gTol );
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d (const NvGeCurve3d& curve1, const NvGeCurve3d& curve2,
                         const NvGeInterval& range1, const NvGeInterval& range2,
		                 const NvGeVector3d& planeNormal=NvGeVector3d::kIdentity,
                         const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d (const NvGeCurveCurveInt3d& src);

    // General query functions.
    //
    GE_DLLEXPIMPORT const NvGeCurve3d  *curve1          () const;
    GE_DLLEXPIMPORT const NvGeCurve3d  *curve2          () const;
    GE_DLLEXPIMPORT void               getIntRanges     (NvGeInterval& range1,
                                         NvGeInterval& range2) const;
	GE_DLLEXPIMPORT NvGeVector3d	   planeNormal		() const;
    GE_DLLEXPIMPORT NvGeTol            tolerance        () const;

    // Intersection query methods.
    //
    GE_DLLEXPIMPORT int                numIntPoints     () const;
    GE_DLLEXPIMPORT NvGePoint3d        intPoint         (int intNum) const;
    GE_DLLEXPIMPORT void               getIntParams     (int intNum,
                                         double& param1, double& param2) const;
    GE_DLLEXPIMPORT void               getPointOnCurve1 (int intNum, NvGePointOnCurve3d& pntOnCrv) const;
    GE_DLLEXPIMPORT void               getPointOnCurve2 (int intNum, NvGePointOnCurve3d& pntOnCrv) const;
    GE_DLLEXPIMPORT void			   getIntConfigs    (int intNum, NvGe::NvGeXConfig& config1wrt2, 
                                         NvGe::NvGeXConfig& config2wrt1) const;
    GE_DLLEXPIMPORT Nova::Boolean     isTangential     (int intNum) const;
    GE_DLLEXPIMPORT Nova::Boolean     isTransversal    (int intNum) const;
    GE_DLLEXPIMPORT double             intPointTol      (int intNum) const;
    GE_DLLEXPIMPORT int                overlapCount     () const;
	GE_DLLEXPIMPORT Nova::Boolean	   overlapDirection () const;
    GE_DLLEXPIMPORT void               getOverlapRanges (int overlapNum,
                                         NvGeInterval& range1,
                                         NvGeInterval& range2) const;

    // Curves change their places
    //
    GE_DLLEXPIMPORT void               changeCurveOrder (); 
        
    // Order with respect to parameter on the first/second curve.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d& orderWrt1  ();    
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d& orderWrt2  ();
    
    // Set functions.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d& set        (const NvGeCurve3d& curve1,
                                     const NvGeCurve3d& curve2,
		                             const NvGeVector3d& planeNormal = 
								     NvGeVector3d::kIdentity,
                                     const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d& set        (const NvGeCurve3d& curve1,
                                     const NvGeCurve3d& curve2,
                                     const NvGeInterval& range1,
                                     const NvGeInterval& range2,
		                             const NvGeVector3d& planeNormal = 
							         NvGeVector3d::kIdentity,
                                     const NvGeTol& tol = NvGeContext::gTol);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt3d& operator = (const NvGeCurveCurveInt3d& src);
};

#pragma pack (pop)
#endif
