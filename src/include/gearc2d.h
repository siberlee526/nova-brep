#ifndef NV_GEARC2D_H
#define NV_GEARC2D_H

#include "gecurv2d.h"
#include "gepnt2d.h"
#include "gevec2d.h"
#pragma pack (push, 8)

class NvGeLine2d;
class NvGeLinearEnt2d;

class

NvGeCircArc2d : public NvGeCurve2d
{
public:
    GE_DLLEXPIMPORT NvGeCircArc2d();
    GE_DLLEXPIMPORT NvGeCircArc2d(const NvGeCircArc2d& arc);
    GE_DLLEXPIMPORT NvGeCircArc2d(const NvGePoint2d& cent, double radius);
    GE_DLLEXPIMPORT NvGeCircArc2d(const NvGePoint2d& cent, double radius,
                  double startAngle, double endAngle,
                  const NvGeVector2d& refVec = NvGeVector2d::kXAxis,
                  Adesk::Boolean isClockWise = Adesk::kFalse);
    GE_DLLEXPIMPORT NvGeCircArc2d(const NvGePoint2d& startPoint, const NvGePoint2d& point, 
                  const NvGePoint2d& endPoint);

	// If bulgeFlag is kTrue, then bulge is interpreted to be the maximum
	// distance between the arc and the chord between the two input points.
	// If bulgeFlag is kFalse, then bulge is interpreted to be tan(ang/4),
	// where ang is the angle of the arc segment between the two input points.
    GE_DLLEXPIMPORT NvGeCircArc2d(const NvGePoint2d& startPoint, const NvGePoint2d& endPoint, double bulge, 
                  Adesk::Boolean bulgeFlag = Adesk::kTrue);


    // Intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith  (const NvGeLinearEnt2d& line, int& intn,
                                   NvGePoint2d& p1, NvGePoint2d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith  (const NvGeCircArc2d& arc, int& intn,
                                   NvGePoint2d& p1, NvGePoint2d& p2,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Tangent line to the circular arc at given point.
    //
    GE_DLLEXPIMPORT Adesk::Boolean tangent        (const NvGePoint2d& pnt, NvGeLine2d& line,
                                   const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean tangent        (const NvGePoint2d& pnt, NvGeLine2d& line,
                                   const NvGeTol& tol, NvGeError& error) const;
		 // Possible error conditions:  kArg1TooBig, kArg1InsideThis, 
		 // kArg1OnThis
     

    // Test if point is inside circle.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isInside       (const NvGePoint2d& pnt,
                                   const NvGeTol& tol = NvGeContext::gTol) const;

    // Definition of circular arc
    //
    GE_DLLEXPIMPORT NvGePoint2d    center         () const;
    GE_DLLEXPIMPORT double         radius         () const;
    GE_DLLEXPIMPORT double         startAng       () const;
    GE_DLLEXPIMPORT double         endAng         () const;
    GE_DLLEXPIMPORT Adesk::Boolean isClockWise    () const;
    GE_DLLEXPIMPORT NvGeVector2d   refVec         () const;
    GE_DLLEXPIMPORT NvGePoint2d    startPoint     () const;
    GE_DLLEXPIMPORT NvGePoint2d    endPoint       () const;

    GE_DLLEXPIMPORT NvGeCircArc2d& setCenter      (const NvGePoint2d& cent);
    GE_DLLEXPIMPORT NvGeCircArc2d& setRadius      (double radius);
    GE_DLLEXPIMPORT NvGeCircArc2d& setAngles      (double startAng, double endAng);
    GE_DLLEXPIMPORT NvGeCircArc2d& setToComplement();
    GE_DLLEXPIMPORT NvGeCircArc2d& setRefVec      (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGePoint2d& cent, double radius);
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGePoint2d& cent, double radius,
                                   double ang1, double ang2,
                                   const NvGeVector2d& refVec =
                                   NvGeVector2d::kXAxis,
                                   Adesk::Boolean isClockWise = Adesk::kFalse);
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGePoint2d& startPoint, const NvGePoint2d& pnt,
                                   const NvGePoint2d& endPoint);
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGePoint2d& startPoint, const NvGePoint2d& pnt,
                                   const NvGePoint2d& endPoint, NvGeError& error);
		 // Possible errors:  kEqualArg1Arg2, kEqualArg1Arg3, kEqualArg2Arg3, 
		 // kLinearlyDependentArg1Arg2Arg3.
		 // Degenerate results: none.
		 // On error, the object is unchanged.

	// If bulgeFlag is kTrue, then bulge is interpreted to be the maximum
	// distance between the arc and the chord between the two input points.
	// If bulgeFlag is kFalse, then bulge is interpreted to be tan(ang/4),
	// where ang is the angle of the arc segment between the two input points.
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGePoint2d& startPoint, 
                                   const NvGePoint2d& endPoint,
                                   double bulge, Adesk::Boolean bulgeFlag = Adesk::kTrue);
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGeCurve2d& curve1,
                                   const NvGeCurve2d& curve2,
                                   double radius, double& param1, double& param2,
								   Adesk::Boolean& success);
		// On success, this arc becomes the fillet of the given radius between the two curves,
	    // whose points of tangency are nearest param1 and param2 respectively.
    GE_DLLEXPIMPORT NvGeCircArc2d& set            (const NvGeCurve2d& curve1,
                                   const NvGeCurve2d& curve2,
                                   const NvGeCurve2d& curve3,
                                   double& param1, double& param2, double& param3,
								   Adesk::Boolean& success);
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeCircArc2d& operator =     (const NvGeCircArc2d& arc);
};

#pragma pack (pop)
#endif
