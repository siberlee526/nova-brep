#ifndef NV_GEELL2D_H
#define NV_GEELL2D_H

#include "gecurv2d.h"
#include "gevec2d.h"
#include "gepnt2d.h"
#include "geponc2d.h"
#include "geintrvl.h"
#pragma pack (push, 8)

class NvGeCircArc2d;
class NvGePlanarEnt;
class NvGeEllipArc2d;
class NvGeLinearEnt2d;


class

NvGeEllipArc2d : public NvGeCurve2d
{
public:
    GE_DLLEXPIMPORT NvGeEllipArc2d();
    GE_DLLEXPIMPORT NvGeEllipArc2d(const NvGeEllipArc2d& ell);
    GE_DLLEXPIMPORT NvGeEllipArc2d(const NvGeCircArc2d& arc);
    GE_DLLEXPIMPORT NvGeEllipArc2d(const NvGePoint2d& cent, const NvGeVector2d& majorAxis,
                   const NvGeVector2d& minorAxis, double majorRadius,
                   double minorRadius);
    GE_DLLEXPIMPORT NvGeEllipArc2d(const NvGePoint2d& cent, const NvGeVector2d& majorAxis,
                   const NvGeVector2d& minorAxis, double majorRadius,
                   double minorRadius, double startAngle, double endAngle);

    // Intersection with other geometric objects.
    //
    GE_DLLEXPIMPORT Adesk::Boolean intersectWith (const NvGeLinearEnt2d& line, int& intn,
                                  NvGePoint2d& p1, NvGePoint2d& p2,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    // Inquiry Methods
    //
    GE_DLLEXPIMPORT Adesk::Boolean isCircular    (const NvGeTol& tol = NvGeContext::gTol) const;

    // Test if point is inside full ellipse
    //
    GE_DLLEXPIMPORT Adesk::Boolean isInside      (const NvGePoint2d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;


    // Definition of ellipse
    //
    GE_DLLEXPIMPORT NvGePoint2d    center        () const;
    GE_DLLEXPIMPORT double         minorRadius   () const;
    GE_DLLEXPIMPORT double         majorRadius   () const;
    GE_DLLEXPIMPORT NvGeVector2d   minorAxis     () const;
    GE_DLLEXPIMPORT NvGeVector2d   majorAxis     () const;
    GE_DLLEXPIMPORT double         startAng      () const;
    GE_DLLEXPIMPORT double         endAng        () const;
    GE_DLLEXPIMPORT NvGePoint2d    startPoint    () const;
    GE_DLLEXPIMPORT NvGePoint2d    endPoint      () const;
    GE_DLLEXPIMPORT Adesk::Boolean isClockWise   () const;

    GE_DLLEXPIMPORT NvGeEllipArc2d& setCenter     (const NvGePoint2d& cent);
    GE_DLLEXPIMPORT NvGeEllipArc2d& setMinorRadius(double rad);
    GE_DLLEXPIMPORT NvGeEllipArc2d& setMajorRadius(double rad);
    GE_DLLEXPIMPORT NvGeEllipArc2d& setAxes       (const NvGeVector2d& majorAxis, const NvGeVector2d& minorAxis);
    GE_DLLEXPIMPORT NvGeEllipArc2d& setAngles     (double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeEllipArc2d& set           (const NvGePoint2d& cent,
                                   const NvGeVector2d& majorAxis,
                                   const NvGeVector2d& minorAxis,
                                   double majorRadius, double minorRadius);
    GE_DLLEXPIMPORT NvGeEllipArc2d& set           (const NvGePoint2d& cent,
                                   const NvGeVector2d& majorAxis,
                                   const NvGeVector2d& minorAxis,
                                   double majorRadius, double minorRadius,
                                   double startAngle, double endAngle);
    GE_DLLEXPIMPORT NvGeEllipArc2d& set           (const NvGeCircArc2d& arc);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeEllipArc2d& operator =    (const NvGeEllipArc2d& ell);
};

#pragma pack (pop)
#endif

