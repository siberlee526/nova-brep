#ifndef NV_GECURV3D_H
#define NV_GECURV3D_H

#include "Nova.h"
#include "geent3d.h"
#include "geponc3d.h"
#include "gept3dar.h"
#include "gevc3dar.h"
#include "gedblar.h"
#include "gevptar.h"
#include "geintarr.h"
#pragma pack (push, 8)

class NvGeCurve2d;
class NvGeSurface;
class NvGePoint3d;
class NvGePlane;
class NvGeVector3d;
class NvGeLinearEnt3d;
class NvGeLine3d;
class NvGePointOnCurve3d;
class NvGePointOnSurface;
class NvGeInterval;
class NvGeMatrix3d;
class NvGePointOnCurve3dData;
class NvGeBoundBlock3d;

class

NvGeCurve3d : public NvGeEntity3d
{
public:

    // Parametrization.
    //
    GE_DLLEXPIMPORT void           getInterval(NvGeInterval& intrvl) const;
    GE_DLLEXPIMPORT void           getInterval(NvGeInterval& intrvl, NvGePoint3d& start,
                               NvGePoint3d& end) const;
    GE_DLLEXPIMPORT NvGeCurve3d&   reverseParam();
	GE_DLLEXPIMPORT NvGeCurve3d&   setInterval();
	GE_DLLEXPIMPORT Nova::Boolean setInterval(const NvGeInterval& intrvl);

    // Distance to other geometric objects.
    //
    GE_DLLEXPIMPORT double       distanceTo(const NvGePoint3d& pnt,
                            const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT double       distanceTo(const NvGeCurve3d& curve,
                            const NvGeTol& tol = NvGeContext::gTol) const;

    // Return the point on this object that is closest to the other object.
    // These methods return point on this curve as a simple 3d point.
    //
    GE_DLLEXPIMPORT NvGePoint3d closestPointTo(const NvGePoint3d& pnt,
                               const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGePoint3d closestPointTo(const NvGeCurve3d& curve3d,
                               NvGePoint3d& pntOnOtherCrv,
                               const NvGeTol& tol = NvGeContext::gTol) const;

    // Alternate signatures for above functions.  These methods return point
    // on this curve as an NvGePointOnCurve3d.
    //
    GE_DLLEXPIMPORT void getClosestPointTo(const NvGePoint3d& pnt, NvGePointOnCurve3d& pntOnCrv,
                           const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT void getClosestPointTo(const NvGeCurve3d& curve3d,
                           NvGePointOnCurve3d& pntOnThisCrv,
                           NvGePointOnCurve3d& pntOnOtherCrv,
                           const NvGeTol& tol = NvGeContext::gTol) const;

    // Return closest points when projected in a given direction.
    // These methods return point on this curve as a simple 3d point.
    //
    GE_DLLEXPIMPORT NvGePoint3d projClosestPointTo(const NvGePoint3d& pnt,
                                   const NvGeVector3d& projectDirection,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGePoint3d projClosestPointTo(const NvGeCurve3d& curve3d,
                                   const NvGeVector3d& projectDirection,
                                   NvGePoint3d& pntOnOtherCrv,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Alternate signatures for above functions.  These methods return point
    // on this curve as an NvGePointOnCurve3d.
    //
    GE_DLLEXPIMPORT void getProjClosestPointTo(const NvGePoint3d& pnt,
                               const NvGeVector3d& projectDirection,
                               NvGePointOnCurve3d& pntOnCrv,
                               const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT void getProjClosestPointTo(const NvGeCurve3d& curve3d,
                               const NvGeVector3d& projectDirection,
                               NvGePointOnCurve3d& pntOnThisCrv,
                               NvGePointOnCurve3d& pntOnOtherCrv,
                               const NvGeTol& tol = NvGeContext::gTol) const;

    // Return point on curve whose normal vector passes thru input point.
    // Second parameter contains initial guess value and also 
    // contains output point.

	// Returns true or false depending on whether a normal point was found.
        //
	GE_DLLEXPIMPORT Nova::Boolean getNormalPoint(const NvGePoint3d& pnt,
	                              NvGePointOnCurve3d& pntOnCrv,
                                      const NvGeTol& tol = NvGeContext::gTol) const;

    // Return oriented bounding box of curve.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock3d  boundBlock() const;
    GE_DLLEXPIMPORT NvGeBoundBlock3d  boundBlock(const NvGeInterval& range) const;

    // Return bounding box whose sides are parallel to coordinate axes.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock3d  orthoBoundBlock() const;
    GE_DLLEXPIMPORT NvGeBoundBlock3d  orthoBoundBlock(const NvGeInterval& range) const;

    // Project methods.
    //
    GE_DLLEXPIMPORT NvGeEntity3d*  project(const NvGePlane& projectionPlane,
                           const NvGeVector3d& projectDirection,
                           const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGeEntity3d*  orthoProject(const NvGePlane& projectionPlane,
                                const NvGeTol& tol = NvGeContext::gTol) const;

    // Tests if point is on curve.
    //
    GE_DLLEXPIMPORT Nova::Boolean isOn(const NvGePoint3d& pnt,
                        const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isOn(const NvGePoint3d& pnt, double& param,
                        const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isOn(double param,
                        const NvGeTol& tol = NvGeContext::gTol) const;

    // Parameter of the point on curve.  Contract: point IS on curve
    //
    GE_DLLEXPIMPORT double paramOf(const NvGePoint3d& pnt, const NvGeTol& tol = NvGeContext::gTol)const;

	// Return the offset of the curve.
	//
	GE_DLLEXPIMPORT void getTrimmedOffset(double distance,
		              const NvGeVector3d& planeNormal,
			      NvGeVoidPointerArray& offsetCurveList,
			      NvGe::OffsetCrvExtType extensionType = NvGe::kFillet,
                              const NvGeTol& tol = NvGeContext::gTol) const;

    // Geometric inquiry methods.
    //
    GE_DLLEXPIMPORT Nova::Boolean isClosed      (const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isPlanar      (NvGePlane& plane,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isLinear      (NvGeLine3d& line,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isCoplanarWith(const NvGeCurve3d& curve3d,
                                  NvGePlane& plane,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isPeriodic    (double& period) const;

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
    GE_DLLEXPIMPORT Nova::Boolean isDegenerate(NvGeEntity3d*& pConvertedEntity,
                                const NvGeTol& tol = NvGeContext::gTol) const;

    // Modify methods.
    //
    GE_DLLEXPIMPORT void getSplitCurves(double param, NvGeCurve3d* & piece1,
                        NvGeCurve3d* & piece2) const;

	// Explode curve into its component sub-curves.
	//
	GE_DLLEXPIMPORT Nova::Boolean explode       (NvGeVoidPointerArray& explodedCurves,
	                              NvGeIntArray& newExplodedCurves,
				      const NvGeInterval* intrvl = NULL ) const;

    // Local closest points
    //
    GE_DLLEXPIMPORT void getLocalClosestPoints(const NvGePoint3d& point,
                               NvGePointOnCurve3d& approxPnt,
                               const NvGeInterval* nbhd = 0,
                               const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT void getLocalClosestPoints(const NvGeCurve3d& otherCurve,
                               NvGePointOnCurve3d& approxPntOnThisCrv,
                               NvGePointOnCurve3d& approxPntOnOtherCrv,
                               const NvGeInterval* nbhd1 = 0,
                               const NvGeInterval* nbhd2 = 0,
                               const NvGeTol& tol = NvGeContext::gTol) const;

    // Return start and end points.
    //
    GE_DLLEXPIMPORT Nova::Boolean hasStartPoint(NvGePoint3d& startPnt) const;
    GE_DLLEXPIMPORT Nova::Boolean hasEndPoint  (NvGePoint3d& endPnt) const;

    // Evaluate methods.
    //
    GE_DLLEXPIMPORT NvGePoint3d    evalPoint(double param) const;
    GE_DLLEXPIMPORT NvGePoint3d    evalPoint(double param, int numDeriv,
                             NvGeVector3dArray& derivArray) const;

    // Polygonize curve to within a specified tolerance.
    // Note: forceResampling will make sure that the actual error is
	//       as close to approxEps as possible
    GE_DLLEXPIMPORT void           getSamplePoints(double fromParam, double toParam, double approxEps, 
                            NvGePoint3dArray& pointArray, NvGeDoubleArray& paramArray,
 		                    bool forceResampling = false) const;
    GE_DLLEXPIMPORT void           getSamplePoints(int numSample, NvGePoint3dArray& pointArray) const;
    GE_DLLEXPIMPORT void           getSamplePoints(int numSample, NvGePoint3dArray& pointArray,
                                   NvGeDoubleArray& paramArray) const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCurve3d&   operator =  (const NvGeCurve3d& curve);

protected:

    // Private constructors so that no object of this class can be instantiated.
    GE_DLLEXPIMPORT NvGeCurve3d();
    GE_DLLEXPIMPORT NvGeCurve3d(const NvGeCurve3d&);

};

#pragma pack (pop)
#endif
