#ifndef NV_GENURB3D_H
#define NV_GENURB3D_H

#include "gecurv3d.h"
#include "geintrvl.h"
#include "gekvec.h"
#include "gept3dar.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "gesent3d.h"
#include "geplin3d.h"
#include "gedblar.h"
#include "gept3dar.h"
#include "gevc3dar.h"
#pragma pack (push, 8)

class NvGeEllipArc3d;
class NvGeLineSeg3d;

class 

NvGeNurbCurve3d : public NvGeSplineEnt3d
{
public:
    // Construct spline from control points.
    //
    GE_DLLEXPIMPORT NvGeNurbCurve3d ();
    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGeNurbCurve3d& src );
    GE_DLLEXPIMPORT NvGeNurbCurve3d (int degree, const NvGeKnotVector& knots,
                     const NvGePoint3dArray& cntrlPnts, 
                     Adesk::Boolean isPeriodic = Adesk::kFalse );
    GE_DLLEXPIMPORT NvGeNurbCurve3d (int degree, const NvGeKnotVector& knots,
                     const NvGePoint3dArray& cntrlPnts, 
                     const NvGeDoubleArray&  weights,
                     Adesk::Boolean isPeriodic = Adesk::kFalse );

    // Construct spline from interpolation data.
    //
    GE_DLLEXPIMPORT NvGeNurbCurve3d (int degree, const NvGePolyline3d& fitPolyline,
                     Adesk::Boolean isPeriodic = Adesk::kFalse );

    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGePoint3dArray& fitPoints, 
                     const NvGeVector3d& startTangent, 
                     const NvGeVector3d& endTangent,
                     Adesk::Boolean startTangentDefined = Adesk::kTrue,
                     Adesk::Boolean endTangentDefined   = Adesk::kTrue,
                     const NvGeTol& fitTolerance = NvGeContext::gTol); 

    // specify the fitting points and the wanted knot parameterization
    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGePoint3dArray& fitPoints, 
                     const NvGeVector3d& startTangent, 
                     const NvGeVector3d& endTangent,
                     Adesk::Boolean startTangentDefined,
                     Adesk::Boolean endTangentDefined,
                     NvGe::KnotParameterization knotParam,
                     const NvGeTol& fitTolerance = NvGeContext::gTol);

    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGePoint3dArray& fitPoints, 
                     const NvGeTol& fitTolerance = NvGeContext::gTol);

    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGePoint3dArray& fitPoints, 
                     const NvGeVector3dArray& fitTangents,
                     const NvGeTol& fitTolerance = NvGeContext::gTol,
                     Adesk::Boolean isPeriodic = Adesk::kFalse);   

    // Construct a cubic spline approximating the curve
    GE_DLLEXPIMPORT NvGeNurbCurve3d(const NvGeCurve3d& curve, 
                    double epsilon = NvGeContext::gTol.equalPoint());


    // Spline representation of ellipse
    //
    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGeEllipArc3d&  ellipse);

    // Spline representation of line segment
    //
    GE_DLLEXPIMPORT NvGeNurbCurve3d (const NvGeLineSeg3d& linSeg);

    // Query methods.
    //
    GE_DLLEXPIMPORT int             numFitPoints      () const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitPointAt     (int index, NvGePoint3d& point) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitTolerance   (NvGeTol& fitTolerance) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitTangents    (NvGeVector3d& startTangent, 
                                       NvGeVector3d& endTangent) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitTangents    (NvGeVector3d& startTangent, 
                                       NvGeVector3d& endTangent,
                                       Adesk::Boolean& startTangentDefined,
                                       Adesk::Boolean& endTangentDefined) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitKnotParameterization(KnotParameterization& knotParam) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitData        (NvGePoint3dArray& fitPoints,
                                       NvGeTol& fitTolerance,
                                       Adesk::Boolean& tangentsExist,
                                       NvGeVector3d& startTangent, 
                                       NvGeVector3d& endTangent) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getFitData        (NvGePoint3dArray& fitPoints,
                                       NvGeTol& fitTolerance,
                                       Adesk::Boolean& tangentsExist,
                                       NvGeVector3d& startTangent, 
                                       NvGeVector3d& endTangent,
                                       KnotParameterization& knotParam) const;
    GE_DLLEXPIMPORT void            getDefinitionData (int& degree, Adesk::Boolean& rational,
                                       Adesk::Boolean& periodic,
                                       NvGeKnotVector& knots,
                                       NvGePoint3dArray& controlPoints,
                                       NvGeDoubleArray& weights) const;
    GE_DLLEXPIMPORT int             numWeights        () const;
    GE_DLLEXPIMPORT double          weightAt          (int idx) const;
    GE_DLLEXPIMPORT Adesk::Boolean  evalMode          () const;        
    GE_DLLEXPIMPORT Adesk::Boolean  getParamsOfC1Discontinuity (NvGeDoubleArray& params,
                                                const NvGeTol& tol 
                                                = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean  getParamsOfG1Discontinuity (NvGeDoubleArray& params,
                                                const NvGeTol& tol 
                                                = NvGeContext::gTol) const;

    // Modification methods.
    //
    GE_DLLEXPIMPORT Adesk::Boolean   setFitPointAt    (int index, const NvGePoint3d& point);
    GE_DLLEXPIMPORT Adesk::Boolean   addFitPointAt    (int index, const NvGePoint3d& point);
    GE_DLLEXPIMPORT Adesk::Boolean   deleteFitPointAt (int index);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitTolerance  (const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitTangents   (const NvGeVector3d& startTangent, 
                                       const NvGeVector3d& endTangent);
    GE_DLLEXPIMPORT Adesk::Boolean   setFitTangents   (const NvGeVector3d& startTangent, 
                                       const NvGeVector3d& endTangent,
                                       Adesk::Boolean startTangentDefined,
                                       Adesk::Boolean endTangentDefined) const;
    GE_DLLEXPIMPORT Adesk::Boolean   setFitKnotParameterization(KnotParameterization knotParam);
    GE_DLLEXPIMPORT NvGeNurbCurve3d& setFitData       (const NvGePoint3dArray& fitPoints,                                             
                                       const NvGeVector3d& startTangent, 
                                       const NvGeVector3d& endTangent,
                                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeNurbCurve3d& setFitData       (const NvGePoint3dArray& fitPoints,                                             
                                       const NvGeVector3d& startTangent, 
                                       const NvGeVector3d& endTangent,
                                       KnotParameterization knotParam,
                                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeNurbCurve3d& setFitData       (const NvGeKnotVector& fitKnots,
                                       const NvGePoint3dArray& fitPoints,
                                       const NvGeVector3d& startTangent, 
                                       const NvGeVector3d& endTangent,                                         
                                       const NvGeTol& fitTol=NvGeContext::gTol,
                                       Adesk::Boolean isPeriodic=Adesk::kFalse);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  setFitData      (int degree, 
                                       const NvGePoint3dArray& fitPoints,
                                       const NvGeTol& fitTol=NvGeContext::gTol);
    GE_DLLEXPIMPORT Adesk::Boolean    purgeFitData    ();
    GE_DLLEXPIMPORT Adesk::Boolean    buildFitData    ();
    GE_DLLEXPIMPORT Adesk::Boolean    buildFitData    (KnotParameterization kp);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  addKnot         (double newKnot);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  insertKnot      (double newKnot);
    GE_DLLEXPIMPORT NvGeSplineEnt3d&  setWeightAt     (int idx, double val);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  setEvalMode     (Adesk::Boolean evalMode=Adesk::kFalse );
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  joinWith        (const NvGeNurbCurve3d& curve);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  hardTrimByParams(double newStartParam, 
                                       double newEndParam);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  makeRational    (double weight = 1.0);
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  makeClosed      ();
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  makePeriodic    ();
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  makeNonPeriodic ();
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  makeOpen        ();
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  elevateDegree   (int plusDegree);

    // add/remove control point.
    GE_DLLEXPIMPORT Adesk::Boolean    addControlPointAt(double newKnot, const NvGePoint3d& point, double weight = 1.0);
    GE_DLLEXPIMPORT Adesk::Boolean    deleteControlPointAt(int index);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeNurbCurve3d&  operator =      (const NvGeNurbCurve3d& spline);
};

#pragma pack (pop)
#endif
