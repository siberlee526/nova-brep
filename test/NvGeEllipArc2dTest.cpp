// NvGeEllipArc2dTest.cpp - unit tests for NvGeEllipArc2d (OCCT-backed).
//
// NOTE: the EvalPoint / IsClosed tests exercise NvGeCurve2d::evalPoint() and
// NvGeCurve2d::isClosed(); those live in the base curve layer (gecurv2d.cpp)
// and pin the ellipse parameterization contract: the curve parameter is the
// public angle measured from the major axis, counter-clockwise positive.

#include <NvException.h>
#include <gearc2d.h>
#include <geell2d.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{

const double THE_PI = 3.14159265358979323846;

const double THE_TEST_TOL = 1e-9;

void ExpectNear (const NvGePoint2d& thePnt, double theX, double theY)
{
  EXPECT_NEAR (thePnt.x, theX, THE_TEST_TOL);
  EXPECT_NEAR (thePnt.y, theY, THE_TEST_TOL);
}

void ExpectNear (const NvGeVector2d& theVec, double theX, double theY)
{
  EXPECT_NEAR (theVec.x, theX, THE_TEST_TOL);
  EXPECT_NEAR (theVec.y, theY, THE_TEST_TOL);
}

}

class NvGeEllipArc2dTest : public testing::Test
{
protected:

  void SetUp () override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }

};

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, DefaultConstructor_IsUnitCircleAtOrigin)
{
  const NvGeEllipArc2d anEll;
  EXPECT_EQ (anEll.type(), NvGe::kEllipArc2d);
  ExpectNear (anEll.center(), 0.0, 0.0);
  EXPECT_NEAR (anEll.majorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.minorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (anEll.isCircular());
  EXPECT_NEAR (anEll.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kFalse);
  ExpectNear (anEll.majorAxis(), 1.0, 0.0);
  ExpectNear (anEll.minorAxis(), 0.0, 1.0);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AxesConstructor_RoundTrips)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (1.0, 2.0), NvGeVector2d (2.0, 0.0),
                              NvGeVector2d (0.0, 4.0), 3.0, 1.0);
  ExpectNear (anEll.center(), 1.0, 2.0);
  EXPECT_NEAR (anEll.majorRadius(), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.minorRadius(), 1.0, THE_TEST_TOL);
  // Axes are returned normalized.
  ExpectNear (anEll.majorAxis(), 1.0, 0.0);
  ExpectNear (anEll.minorAxis(), 0.0, 1.0);
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kFalse);
  EXPECT_TRUE (anEll.isCircular() == Adesk::kFalse);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AxesConstructor_RejectsMajorSmallerThanMinor)
{
  EXPECT_THROW ((NvGeEllipArc2d (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                                 NvGeVector2d (0.0, 1.0), 2.0, 3.0)), NvException);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AxesConstructor_RejectsNonPositiveMinorRadius)
{
  EXPECT_THROW ((NvGeEllipArc2d (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                                 NvGeVector2d (0.0, 1.0), 2.0, 0.0)), NvException);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AngleConstructor_QuarterArc_RoundTrips)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 3.0, 1.0, 0.0, THE_PI / 2.0);
  EXPECT_NEAR (anEll.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kFalse);
  ExpectNear (anEll.startPoint(), 4.0, 2.0);
  ExpectNear (anEll.endPoint(), 1.0, 3.0);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AngleConstructor_EqualAngles_MakesFullEllipse)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 3.0, 1.0, 0.7, 0.7);
  EXPECT_NEAR (anEll.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AngleConstructor_Clockwise_RoundTrips)
{
  // Direction is derived from the axis handedness: minor = -Y makes the arc
  // clockwise, and increasing angles then run clockwise around the center.
  const NvGeEllipArc2d anEll (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, -1.0), 3.0, 1.0, THE_PI / 2.0, 0.0);
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kTrue);
  EXPECT_NEAR (anEll.startAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), 0.0, THE_TEST_TOL);
  ExpectNear (anEll.startPoint(), 1.0, 1.0);
  ExpectNear (anEll.endPoint(), 4.0, 2.0);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, CircArcConstructor_Converts)
{
  const NvGeCircArc2d anArc (NvGePoint2d (5.0, 5.0), 2.0);
  const NvGeEllipArc2d anEll (anArc);
  EXPECT_TRUE (anEll.isCircular());
  ExpectNear (anEll.center(), 5.0, 5.0);
  EXPECT_NEAR (anEll.majorRadius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.minorRadius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kFalse);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, IsInside_ClassifiesByQuadraticForm)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 3.0, 1.0);
  EXPECT_TRUE (anEll.isInside (NvGePoint2d (0.0, 0.0)));
  EXPECT_TRUE (anEll.isInside (NvGePoint2d (1.0, 0.0)));
  EXPECT_TRUE (anEll.isInside (NvGePoint2d (0.0, 0.5)));
  EXPECT_TRUE (anEll.isInside (NvGePoint2d (5.0, 0.0)) == Adesk::kFalse);
  EXPECT_TRUE (anEll.isInside (NvGePoint2d (0.0, 2.0)) == Adesk::kFalse);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, IntersectWithLine_FindsTwoPoints)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 2.0, 1.0);
  const NvGeLine2d aLine (NvGePoint2d (1.0, 0.0), NvGeVector2d (0.0, 1.0));
  int anIntn = 0;
  NvGePoint2d aP1, aP2;
  EXPECT_TRUE (anEll.intersectWith (aLine, anIntn, aP1, aP2));
  EXPECT_EQ (anIntn, 2);
  const double aY = std::sqrt (3.0) / 2.0;
  // Points are reported in ascending line parameter order.
  ExpectNear (aP1, 1.0, -aY);
  ExpectNear (aP2, 1.0, aY);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, Setters_RoundTrip)
{
  NvGeEllipArc2d anEll;
  anEll.setCenter (NvGePoint2d (1.0, 1.0));
  anEll.setMajorRadius (4.0);
  anEll.setMinorRadius (2.0);
  anEll.setAngles (0.0, THE_PI);
  ExpectNear (anEll.center(), 1.0, 1.0);
  EXPECT_NEAR (anEll.majorRadius(), 4.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.minorRadius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.endAng(), THE_PI, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, SetMinorRadius_RejectsLargerThanMajor)
{
  NvGeEllipArc2d anEll (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                        NvGeVector2d (0.0, 1.0), 3.0, 1.0);
  EXPECT_THROW (anEll.setMinorRadius (5.0), NvException);
  EXPECT_THROW (anEll.setMinorRadius (0.0), NvException);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, SetAxes_FlipsDirectionForOppositeHandedness)
{
  NvGeEllipArc2d anEll;
  anEll.setAxes (NvGeVector2d (1.0, 0.0), NvGeVector2d (0.0, -1.0));
  EXPECT_TRUE (anEll.isClockWise() == Adesk::kTrue);
  ExpectNear (anEll.majorAxis(), 1.0, 0.0);
  ExpectNear (anEll.minorAxis(), 0.0, -1.0);
  EXPECT_NEAR (anEll.majorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anEll.minorRadius(), 1.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, AssignmentOperator_SharesUntilModified)
{
  const NvGeEllipArc2d aBase (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 3.0, 1.0);
  NvGeEllipArc2d aCopy;
  aCopy = aBase;
  aCopy.setMajorRadius (5.0);
  EXPECT_NEAR (aBase.majorRadius(), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.majorRadius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.minorRadius(), 1.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, EvalPoint_AtZero_IsMajorAxisPoint)
{
  const NvGeEllipArc2d anEll (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0),
                              NvGeVector2d (0.0, 1.0), 3.0, 1.0);
  ExpectNear (anEll.evalPoint (0.0), 4.0, 2.0);
  ExpectNear (anEll.evalPoint (THE_PI / 2.0), 1.0, 3.0);
}

//=================================================================================================

TEST_F (NvGeEllipArc2dTest, IsClosed_TrueOnlyForFullEllipse)
{
  EXPECT_TRUE (NvGeEllipArc2d().isClosed());
  const NvGeEllipArc2d aQuarter (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                                 NvGeVector2d (0.0, 1.0), 3.0, 1.0, 0.0, THE_PI / 2.0);
  EXPECT_TRUE (aQuarter.isClosed() == Adesk::kFalse);
}
