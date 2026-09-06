#ifndef NV_GECINT2D_H
#define NV_GECINT2D_H

#include "Nova.h"
#include "gegbl.h"
#include "geent2d.h"
#include "geponc2d.h"
#include "geintrvl.h"
#pragma pack (push, 8)

class NvGeCurve2d;


class  

NvGeCurveCurveInt2d : public NvGeEntity2d
{

public:
    // Constructors.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d ();
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d (const NvGeCurve2d& curve1, const NvGeCurve2d& curve2,
                         const NvGeTol& tol = NvGeContext::gTol );
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d (const NvGeCurve2d& curve1, const NvGeCurve2d& curve2,
                         const NvGeInterval& range1, const NvGeInterval& range2,
                         const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d (const NvGeCurveCurveInt2d& src);

    // General query functions.
    //
    GE_DLLEXPIMPORT const NvGeCurve2d  *curve1          () const;
    GE_DLLEXPIMPORT const NvGeCurve2d  *curve2          () const;
    GE_DLLEXPIMPORT void               getIntRanges     (NvGeInterval& range1,
                                         NvGeInterval& range2) const;
    GE_DLLEXPIMPORT NvGeTol            tolerance        () const;

    // Intersection query methods.
    //
    GE_DLLEXPIMPORT int                numIntPoints     () const;
    GE_DLLEXPIMPORT NvGePoint2d        intPoint         (int intNum) const;
    GE_DLLEXPIMPORT void               getIntParams     (int intNum,
                                         double& param1, double& param2) const;
    GE_DLLEXPIMPORT void               getPointOnCurve1 (int intNum, NvGePointOnCurve2d&) const;
    GE_DLLEXPIMPORT void               getPointOnCurve2 (int intNum, NvGePointOnCurve2d&) const;
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
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d& orderWrt1  ();    
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d& orderWrt2  ();
    
	// Set functions.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d& set        (const NvGeCurve2d& curve1,
                                     const NvGeCurve2d& curve2,
                                     const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d& set        (const NvGeCurve2d& curve1,
                                     const NvGeCurve2d& curve2,
                                     const NvGeInterval& range1,
                                     const NvGeInterval& range2,
                                     const NvGeTol& tol = NvGeContext::gTol);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCurveCurveInt2d& operator = (const NvGeCurveCurveInt2d& src);
};

#pragma pack (pop)
#endif
