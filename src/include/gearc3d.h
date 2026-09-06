#ifndef NV_GEARC3D_H
#define NV_GEARC3D_H

#include "gecurv3d.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "geplane.h"
#include "gegblabb.h"
#pragma pack (push, 8)
class NvGeLine3d;
class NvGeCircArc2d;
class NvGePlanarEnt;

class 

NvGeCircArc3d : public NvGeCurve3d
{
public:
    GE_DLLEXPIMPORT NvGeCircArc3d();
    GE_DLLEXPIMPORT NvGeCircArc3d(const NvGeCircArc3d& arc);
    GE_DLLEXPIMPORT NvGeCircArc3d(const NvGePoint3d& cent,
                  const NvGeVector3d& nrm, double radius);
    GE_DLLEXPIMPORT NvGeCircArc3d(const NvGePoint3d& cent, const NvGeVector3d& nrm,
                  const NvGeVector3d& refVec, double radius,
                  double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeCircArc3d(const NvGePoint3d& startPoint, const NvGePoint3d& pnt, const NvGePoint3d& endPoint);
                                            
    // Return the point on this object that is closest to the other object.
    //
    GE_DLLEXPIMPORT NvGePoint3d    closestPointToPlane (const NvGePlanarEnt& plane,
                                        NvGePoint3d& pointOnPlane,
                                        const NvGeTol& tol
                                        = NvGeContext::gTol) const;

    // Intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith  (const NvGeLinearEnt3d& line, int& intn,
                                   NvGePoint3d& p1, NvGePoint3d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith  (const NvGeCircArc3d& arc, int& intn,
                                   NvGePoint3d& p1, NvGePoint3d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean  intersectWith (const NvGePlanarEnt& plane, int& numOfIntersect,
                                   NvGePoint3d& p1, NvGePoint3d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Projection-intersection with other geometric objects.
    GE_DLLEXPIMPORT Adesk::Boolean projIntersectWith (const NvGeLinearEnt3d& line,
                                      const NvGeVector3d& projDir, int& numInt,
                                      NvGePoint3d& pntOnArc1,
                                      NvGePoint3d& pntOnArc2,
                                      NvGePoint3d& pntOnLine1,
                                      NvGePoint3d& pntOnLine2,
                                      const NvGeTol& tol = NvGeContext::gTol) const;

    // Tangent to the circular arc.
    //
    GE_DLLEXPIMPORT Adesk::Boolean tangent        (const NvGePoint3d& pnt, NvGeLine3d& line,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean tangent        (const NvGePoint3d& pnt, NvGeLine3d& line,
                                   const NvGeTol& tol, NvGeError& error) const;
		 // Possible error conditions:  kArg1TooBig, kArg1InsideThis, 
		 // kArg1OnThis, kThisIsInfiniteLine

    // Plane of the arc
    //
    GE_DLLEXPIMPORT void           getPlane       (NvGePlane& plane) const;

    // Test if point is inside circle.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isInside       (const NvGePoint3d& pnt,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Definition of circular arc
    //
    GE_DLLEXPIMPORT NvGePoint3d    center         () const;
    GE_DLLEXPIMPORT NvGeVector3d   normal         () const;
    GE_DLLEXPIMPORT NvGeVector3d   refVec         () const;
    GE_DLLEXPIMPORT double         radius         () const;
    GE_DLLEXPIMPORT double         startAng       () const;
    GE_DLLEXPIMPORT double         endAng         () const;
    GE_DLLEXPIMPORT NvGePoint3d    startPoint     () const;
    GE_DLLEXPIMPORT NvGePoint3d    endPoint       () const;

    GE_DLLEXPIMPORT NvGeCircArc3d& setCenter      (const NvGePoint3d&);
    GE_DLLEXPIMPORT NvGeCircArc3d& setAxes        (const NvGeVector3d& normal,
                                   const NvGeVector3d& refVec);
    GE_DLLEXPIMPORT NvGeCircArc3d& setRadius      (double);
    GE_DLLEXPIMPORT NvGeCircArc3d& setAngles      (double startAngle, double endAngle);

    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGePoint3d& cent,
                                   const NvGeVector3d& nrm, double radius);
    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGePoint3d& cent,
                                   const NvGeVector3d& nrm,
                                   const NvGeVector3d& refVec, double radius,
                                   double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGePoint3d& startPoint, const NvGePoint3d& pnt,
                                   const NvGePoint3d& endPoint);
    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGePoint3d& startPoint, const NvGePoint3d& pnt,
                                   const NvGePoint3d& endPoint, NvGeError& error);
			 // Possible errors:  kEqualArg1Arg2, kEqualArg1Arg3, kEqualArg2Arg3, 
			 // kLinearlyDependentArg1Arg2Arg3.
			 // Degenerate results: none.
			 // On error, the object is unchanged.

    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGeCurve3d& curve1,
                                   const NvGeCurve3d& curve2,
                                   double radius, double& param1, double& param2,
								   Adesk::Boolean& success);
		// On success, this arc becomes the fillet of the given radius between the two curves,
	    // whose points of tangency are nearest param1 and param2 respectively.
    GE_DLLEXPIMPORT NvGeCircArc3d& set            (const NvGeCurve3d& curve1,
                                   const NvGeCurve3d& curve2,
                                   const NvGeCurve3d& curve3,
                                   double& param1, double& param2, double& param3,
								   Adesk::Boolean& success);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCircArc3d& operator =     (const NvGeCircArc3d& arc);
};

#pragma pack (pop)
#endif
