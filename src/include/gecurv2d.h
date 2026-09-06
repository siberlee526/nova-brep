#ifndef NV_GECURV2D_H
#define NV_GECURV2D_H

#include "Nova.h"
#include "geent2d.h"
#include "geponc2d.h"
#include "gept2dar.h"
#include "gevc2dar.h"
#include "gedblar.h"
#include "gevptar.h"
#include "geintarr.h"
#pragma pack (push, 8)

class NvGePoint2d;
class NvGeVector2d;
class NvGePointOnCurve2d;
class NvGeInterval;
class NvGeMatrix2d;
class NvGeLine2d;
class NvGePointOnCurve2dData;
class NvGeBoundBlock2d;

class

NvGeCurve2d : public NvGeEntity2d
{
public:

    // Parametrization.
    //
    GE_DLLEXPIMPORT void           getInterval(NvGeInterval& intrvl) const;
    GE_DLLEXPIMPORT void           getInterval(NvGeInterval& intrvl, NvGePoint2d& start,
			                   NvGePoint2d& end) const;
    GE_DLLEXPIMPORT NvGeCurve2d&   reverseParam();
	GE_DLLEXPIMPORT NvGeCurve2d&   setInterval();
	GE_DLLEXPIMPORT Nova::Boolean setInterval(const NvGeInterval& intrvl);

    // Distance to other geometric objects.
    //
    GE_DLLEXPIMPORT double         distanceTo(const NvGePoint2d& pnt,
                              const NvGeTol& = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT double         distanceTo(const NvGeCurve2d&,
                              const NvGeTol& tol = NvGeContext::gTol) const;

    // Return the point on this object that is closest to the other object.
    // These methods return point on this curve as a simple 2d point.
    //
    GE_DLLEXPIMPORT NvGePoint2d closestPointTo(const NvGePoint2d& pnt,
                               const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGePoint2d closestPointTo(const NvGeCurve2d& curve2d,
                               NvGePoint2d& pntOnOtherCrv,
                               const NvGeTol& tol= NvGeContext::gTol) const;


    // Alternate signatures for above functions.  These methods return point
    // on this curve as an NvGePointOnCurve2d.
    //
    GE_DLLEXPIMPORT void getClosestPointTo(const NvGePoint2d& pnt,
                           NvGePointOnCurve2d& pntOnCrv,
                           const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT void getClosestPointTo(const NvGeCurve2d& curve2d,
                           NvGePointOnCurve2d& pntOnThisCrv,
                           NvGePointOnCurve2d& pntOnOtherCrv,
                           const NvGeTol& tol = NvGeContext::gTol) const;

    // Return point on curve whose normal vector passes thru input point.
    // Second parameter contains initial guess value and also contains output point.
    // Returns true or false depending on whether a normal point was found.
    //
    GE_DLLEXPIMPORT Nova::Boolean getNormalPoint (const NvGePoint2d& pnt,
	                           NvGePointOnCurve2d& pntOnCrv,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Tests if point is on curve.
    //
    GE_DLLEXPIMPORT Nova::Boolean isOn(const NvGePoint2d& pnt,
                        const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isOn(const NvGePoint2d& pnt, double& param,
                        const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isOn(double param,
                        const NvGeTol& tol = NvGeContext::gTol) const;

    // Parameter of the point on curve.  Contract: point IS on curve
    //
    GE_DLLEXPIMPORT double         paramOf(const NvGePoint2d& pnt,
                           const NvGeTol& tol = NvGeContext::gTol) const;

        // Return the offset of the curve.
        //
	GE_DLLEXPIMPORT void           getTrimmedOffset (double distance,
									 NvGeVoidPointerArray& offsetCurveList,
									 NvGe::OffsetCrvExtType extensionType = NvGe::kFillet,
                                     const NvGeTol& = NvGeContext::gTol) const;

    // Geometric inquiry methods.
    //
    GE_DLLEXPIMPORT Nova::Boolean isClosed  (const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isPeriodic(double& period) const;
    GE_DLLEXPIMPORT Nova::Boolean isLinear  (NvGeLine2d& line,
                              const NvGeTol& tol = NvGeContext::gTol) const;

    // Length based methods.
    //
    GE_DLLEXPIMPORT double         length       (double fromParam, double toParam,
                                 double tol = NvGeContext::gTol.equalPoint()) const;
    GE_DLLEXPIMPORT double         paramAtLength(double datumParam, double length,
                                 Nova::Boolean posParamDir = Nova::kTrue,
                                 double tol = NvGeContext::gTol.equalPoint()) const;
    GE_DLLEXPIMPORT Nova::Boolean area         (double startParam, double endParam,
                                 double& value,
                                 const NvGeTol& tol = NvGeContext::gTol) const;

    // Degeneracy.
    //
    GE_DLLEXPIMPORT Nova::Boolean isDegenerate(NvGe::EntityId& degenerateType,
                                const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isDegenerate(NvGeEntity2d*& pConvertedEntity,
                                const NvGeTol& tol = NvGeContext::gTol) const;

    // Modify methods.
    //
    GE_DLLEXPIMPORT void           getSplitCurves (double param, NvGeCurve2d* & piece1,
                                   NvGeCurve2d* & piece2) const;

	// Explode curve into its component sub-curves.
	//
	GE_DLLEXPIMPORT Nova::Boolean explode      (NvGeVoidPointerArray& explodedCurves,
	                             NvGeIntArray& newExplodedCurve,
				     const NvGeInterval* intrvl = NULL ) const;

    // Local closest points
    //
    GE_DLLEXPIMPORT void getLocalClosestPoints(const NvGePoint2d& point,
                               NvGePointOnCurve2d& approxPnt,
                               const NvGeInterval* nbhd = 0,
                               const NvGeTol& = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT void getLocalClosestPoints(const NvGeCurve2d& otherCurve,
                               NvGePointOnCurve2d& approxPntOnThisCrv,
                               NvGePointOnCurve2d& approxPntOnOtherCrv,
                               const NvGeInterval* nbhd1 = 0,
                               const NvGeInterval* nbhd2 = 0,
                               const NvGeTol& tol = NvGeContext::gTol) const;

    // Return oriented bounding box of curve.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock2d  boundBlock() const;
    GE_DLLEXPIMPORT NvGeBoundBlock2d  boundBlock(const NvGeInterval& range) const;

    // Return bounding box whose sides are parallel to coordinate axes.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock2d  orthoBoundBlock() const;
    GE_DLLEXPIMPORT NvGeBoundBlock2d  orthoBoundBlock(const NvGeInterval& range) const;

    // Return start and end points.
    //
    GE_DLLEXPIMPORT Nova::Boolean hasStartPoint(NvGePoint2d& startPoint) const;
    GE_DLLEXPIMPORT Nova::Boolean hasEndPoint  (NvGePoint2d& endPoint) const;

    // Evaluate methods.
    //
    GE_DLLEXPIMPORT NvGePoint2d    evalPoint(double param) const;
    GE_DLLEXPIMPORT NvGePoint2d    evalPoint(double param, int numDeriv,
                             NvGeVector2dArray& derivArray) const;

    // Polygonize curve to within a specified tolerance.
    //
    GE_DLLEXPIMPORT void     getSamplePoints(double fromParam, double toParam,
                             double approxEps, NvGePoint2dArray& pointArray,
			     NvGeDoubleArray& paramArray) const;
    GE_DLLEXPIMPORT void     getSamplePoints(int numSample, NvGePoint2dArray&) const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCurve2d&   operator =  (const NvGeCurve2d& curve);

protected:

    // Private constructors so that no object of this class can be instantiated.
    GE_DLLEXPIMPORT NvGeCurve2d ();
    GE_DLLEXPIMPORT NvGeCurve2d (const NvGeCurve2d&);
};

#pragma pack (pop)
#endif
