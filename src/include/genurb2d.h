#ifndef NV_GENURB2d_H
#define NV_GENURB2d_H

#include "gecurv2d.h"
#include "geintrvl.h"
#include "gekvec.h"
#include "gept2dar.h"
#include "gevec2d.h"
#include "gepnt2d.h"
#include "gesent2d.h"
#include "geplin2d.h"
#include "gedblar.h"
#include "gept2dar.h"
#include "gevc2dar.h"
#pragma pack (push, 8)

class NvGeEllipArc2d;
class NvGeLineSeg2d;

class 

NvGeNurbCurve2d : public NvGeSplineEnt2d
{
public:
    // Construct spline from control points.
	//
    GE_DLLEXPIMPORT NvGeNurbCurve2d ();
    GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGeNurbCurve2d& src );
    GE_DLLEXPIMPORT NvGeNurbCurve2d (int degree, const NvGeKnotVector& knots,
                     const NvGePoint2dArray& cntrlPnts, 
                     Adesk::Boolean isPeriodic = Adesk::kFalse );
    GE_DLLEXPIMPORT NvGeNurbCurve2d (int degree, const NvGeKnotVector& knots,
                     const NvGePoint2dArray& cntrlPnts, 
                     const NvGeDoubleArray&  weights,
                     Adesk::Boolean isPeriodic = Adesk::kFalse );

    // Construct spline from interpolation data.
    //
    GE_DLLEXPIMPORT NvGeNurbCurve2d (int degree, const NvGePolyline2d& fitPolyline,
                     Adesk::Boolean isPeriodic = Adesk::kFalse );

    GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGePoint2dArray& fitPoints, 
				     const NvGeVector2d& startTangent, 
				     const NvGeVector2d& endTangent,
				     Adesk::Boolean startTangentDefined = Adesk::kTrue,
					 Adesk::Boolean endTangentDefined   = Adesk::kTrue,
				     const NvGeTol& fitTolerance = NvGeContext::gTol);

    // specify the fitting points and the wanted knot parameterization
    GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGePoint2dArray& fitPoints, 
				     const NvGeVector2d& startTangent, 
				     const NvGeVector2d& endTangent,
				     Adesk::Boolean startTangentDefined,
					 Adesk::Boolean endTangentDefined,
                     NvGe::KnotParameterization knotParam,
				     const NvGeTol& fitTolerance = NvGeContext::gTol);

    GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGePoint2dArray& fitPoints, 
				     const NvGeTol& fitTolerance = NvGeContext::gTol);

    GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGePoint2dArray& fitPoints, 
                     const NvGeVector2dArray& fitTangents,
				     const NvGeTol& fitTolerance = NvGeContext::gTol,
				     Adesk::Boolean isPeriodic = Adesk::kFalse);
    
    // Spline representation of ellipse
	//
	GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGeEllipArc2d&  ellipse);

    // Spline representation of line segment
	//
	GE_DLLEXPIMPORT NvGeNurbCurve2d (const NvGeLineSeg2d& linSeg);

    // Construct a cubic spline approximating the curve
    GE_DLLEXPIMPORT NvGeNurbCurve2d(const NvGeCurve2d& curve, 
                    double epsilon = NvGeContext::gTol.equalPoint());

	// Query methods.
	//
    GE_DLLEXPIMPORT int             numFitPoints      () const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitPointAt     (int index, NvGePoint2d& point) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitTolerance   (NvGeTol& fitTolerance) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitTangents    (NvGeVector2d& startTangent, 
				                       NvGeVector2d& endTangent) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitKnotParameterization(KnotParameterization& knotParam) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitData        (NvGePoint2dArray& fitPoints,
		                               NvGeTol& fitTolerance,
				                       Adesk::Boolean& tangentsExist,
				                       NvGeVector2d& startTangent, 
				                       NvGeVector2d& endTangent) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitData        (NvGePoint2dArray& fitPoints,
		                               NvGeTol& fitTolerance,
				                       Adesk::Boolean& tangentsExist,
				                       NvGeVector2d& startTangent, 
				                       NvGeVector2d& endTangent,
                                       KnotParameterization& knotParam) const;
    GE_DLLEXPIMPORT void            getDefinitionData (int& degree, Adesk::Boolean& rational,
								       Adesk::Boolean& periodic,
			                           NvGeKnotVector& knots,
			                           NvGePoint2dArray& controlPoints,
			                           NvGeDoubleArray& weights) const;
    GE_DLLEXPIMPORT int             numWeights        () const;
    GE_DLLEXPIMPORT double          weightAt          (int idx) const;
    GE_DLLEXPIMPORT Adesk::Boolean  evalMode          () const;        
	GE_DLLEXPIMPORT Adesk::Boolean  getParamsOfC1Discontinuity (NvGeDoubleArray& params,
				                                const NvGeTol& tol 
					                            = NvGeContext::gTol) const;
	GE_DLLEXPIMPORT Adesk::Boolean	getParamsOfG1Discontinuity (NvGeDoubleArray& params,
					                            const NvGeTol& tol 
					                            = NvGeContext::gTol) const;

	// Modification methods.
	//
    GE_DLLEXPIMPORT Adesk::Boolean   setFitPointAt    (int index, const NvGePoint2d& point);
    GE_DLLEXPIMPORT Adesk::Boolean   addFitPointAt    (int index, const NvGePoint2d& point);
    GE_DLLEXPIMPORT Adesk::Boolean   deleteFitPointAt (int index);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitTolerance  (const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitTangents   (const NvGeVector2d& startTangent, 
	                        	       const NvGeVector2d& endTangent);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitKnotParameterization(KnotParameterization knotParam);
    GE_DLLEXPIMPORT NvGeNurbCurve2d& setFitData       (const NvGePoint2dArray& fitPoints,                                             
				                       const NvGeVector2d& startTangent, 
				                       const NvGeVector2d& endTangent,
				                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeNurbCurve2d& setFitData       (const NvGePoint2dArray& fitPoints,                                             
				                       const NvGeVector2d& startTangent, 
				                       const NvGeVector2d& endTangent,
                                       KnotParameterization knotParam,
				                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeNurbCurve2d& setFitData       (const NvGeKnotVector& fitKnots,
		                               const NvGePoint2dArray& fitPoints,
				                       const NvGeVector2d& startTangent, 
				                       const NvGeVector2d& endTangent,										 
                        			   const NvGeTol& fitTol=NvGeContext::gTol,
				                       Adesk::Boolean isPeriodic=Adesk::kFalse);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  setFitData      (int degree, 
                                       const NvGePoint2dArray& fitPoints,
				                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT Adesk::Boolean    purgeFitData    ();
    GE_DLLEXPIMPORT Adesk::Boolean    buildFitData    ();
    GE_DLLEXPIMPORT Adesk::Boolean    buildFitData    (KnotParameterization kp);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  addKnot         (double newKnot);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  insertKnot      (double newKnot);
    GE_DLLEXPIMPORT NvGeSplineEnt2d&  setWeightAt     (int idx, double val);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  setEvalMode     (Adesk::Boolean evalMode=Adesk::kFalse );
	GE_DLLEXPIMPORT NvGeNurbCurve2d&  joinWith        (const NvGeNurbCurve2d& curve);
	GE_DLLEXPIMPORT NvGeNurbCurve2d&  hardTrimByParams(double newStartParam, 
		                               double newEndParam);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  makeRational    (double weight = 1.0);
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  makeClosed      ();
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  makePeriodic    ();
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  makeNonPeriodic ();
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  makeOpen        ();
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  elevateDegree   (int plusDegree);

    // add/remove control point.
    GE_DLLEXPIMPORT Adesk::Boolean    addControlPointAt(double newKnot, const NvGePoint2d& point, double weight = 1.0);
    GE_DLLEXPIMPORT Adesk::Boolean    deleteControlPointAt(int index);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeNurbCurve2d&  operator =      (const NvGeNurbCurve2d& spline);
};

#pragma pack (pop)
#endif
