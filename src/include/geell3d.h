#ifndef NV_GEELL3D_H
#define NV_GEELL3D_H

#include "gecurv3d.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "geintrvl.h"
#include "geponc3d.h"
#include "geplane.h"
#pragma pack (push, 8)

class NvGeEllipArc2d;
class NvGeCircArc3d;
class NvGeLineEnt3d;
class NvGePlanarEnt;


class

NvGeEllipArc3d : public NvGeCurve3d
{
public:

    GE_DLLEXPIMPORT NvGeEllipArc3d();
    GE_DLLEXPIMPORT NvGeEllipArc3d(const NvGeEllipArc3d& ell);
    GE_DLLEXPIMPORT NvGeEllipArc3d(const NvGeCircArc3d& arc);
    GE_DLLEXPIMPORT NvGeEllipArc3d(const NvGePoint3d& cent, const NvGeVector3d& majorAxis,
                   const NvGeVector3d& minorAxis, double majorRadius,
                   double minorRadius);
    GE_DLLEXPIMPORT NvGeEllipArc3d(const NvGePoint3d& cent, const NvGeVector3d& majorAxis,
                   const NvGeVector3d& minorAxis, double majorRadius,
                   double minorRadius, double ang1, double ang2);
                                         
    // Return the point on this object that is closest to the other object.
    //
    GE_DLLEXPIMPORT NvGePoint3d    closestPointToPlane(const NvGePlanarEnt& plane,
                                  NvGePoint3d& pointOnPlane,
                                  const NvGeTol& = NvGeContext::gTol) const;

    // Intersection with other geometric objects.
    // 
    GE_DLLEXPIMPORT Nova::Boolean intersectWith (const NvGeLinearEnt3d& line, int& intn,
                                  NvGePoint3d& p1, NvGePoint3d& p2,
                                  const NvGeTol& tol = NvGeContext::gTol) const;  
    GE_DLLEXPIMPORT Nova::Boolean intersectWith (const NvGePlanarEnt& plane, int& numOfIntersect,
                                  NvGePoint3d& p1, NvGePoint3d& p2,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
 
    // Projection-intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Nova::Boolean projIntersectWith(const NvGeLinearEnt3d& line,
                                  const NvGeVector3d& projDir, int &numInt,
                                  NvGePoint3d& pntOnEllipse1,
                                  NvGePoint3d& pntOnEllipse2,
                                  NvGePoint3d& pntOnLine1,
                                  NvGePoint3d& pntOnLine2,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    // Plane of the ellipse
    //
    GE_DLLEXPIMPORT void           getPlane      (NvGePlane& plane) const;

    // Inquiry Methods
    //
    GE_DLLEXPIMPORT Nova::Boolean isCircular    (const NvGeTol& tol = NvGeContext::gTol) const;

    // Test if point is inside full ellipse
    //
    GE_DLLEXPIMPORT Nova::Boolean isInside      (const NvGePoint3d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;

    // Definition of ellipse
    //
    GE_DLLEXPIMPORT NvGePoint3d    center        () const;
    GE_DLLEXPIMPORT double         minorRadius   () const;
    GE_DLLEXPIMPORT double         majorRadius   () const;
    GE_DLLEXPIMPORT NvGeVector3d   minorAxis     () const;
    GE_DLLEXPIMPORT NvGeVector3d   majorAxis     () const;
    GE_DLLEXPIMPORT NvGeVector3d   normal        () const; 
    GE_DLLEXPIMPORT double         startAng      () const;
    GE_DLLEXPIMPORT double         endAng        () const;
    GE_DLLEXPIMPORT NvGePoint3d    startPoint    () const;
    GE_DLLEXPIMPORT NvGePoint3d    endPoint      () const;

    GE_DLLEXPIMPORT NvGeEllipArc3d& setCenter     (const NvGePoint3d& cent);
    GE_DLLEXPIMPORT NvGeEllipArc3d& setMinorRadius(double rad);
    GE_DLLEXPIMPORT NvGeEllipArc3d& setMajorRadius(double rad);
    GE_DLLEXPIMPORT NvGeEllipArc3d& setAxes       (const NvGeVector3d& majorAxis, const NvGeVector3d& minorAxis);
    GE_DLLEXPIMPORT NvGeEllipArc3d& setAngles     (double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeEllipArc3d& set           (const NvGePoint3d& cent,
                                   const NvGeVector3d& majorAxis,
                                   const NvGeVector3d& minorAxis,
                                   double majorRadius, double minorRadius);
    GE_DLLEXPIMPORT NvGeEllipArc3d& set           (const NvGePoint3d& cent,
                                   const NvGeVector3d& majorAxis,
                                   const NvGeVector3d& minorAxis,
                                   double majorRadius, double minorRadius,
                                   double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeEllipArc3d& set           (const NvGeCircArc3d&);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeEllipArc3d& operator =    (const NvGeEllipArc3d& ell);
};

#pragma pack (pop)
#endif
