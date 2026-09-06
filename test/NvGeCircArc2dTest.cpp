// NvGeCircArc2dTest.cpp - unit tests for NvGeCircArc2d (OCCT-backed).
//
// NOTE: the EvalPoint / IsClosed tests exercise NvGeCurve2d::evalPoint() and
// NvGeCurve2d::isClosed(); those live in the base curve layer (gecurv2d.cpp)
// and pin the arc parameterization contract: the curve parameter is the
// public angle measured from refVec, counter-clockwise positive.

#include <NvException.h>
#include <gearc2d.h>
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

class NvGeCircArc2dTest : public testing::Test
{
protected:

  void SetUp () override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }

};

//=================================================================================================

TEST_F (NvGeCircArc2dTest, DefaultConstructor_BuildsUnitFullCircle)
{
  const NvGeCircArc2d anArc;
  EXPECT_EQ (anArc.type(), NvGe::kCircArc2d);
  ExpectNear (anArc.center(), 0.0, 0.0);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
  EXPECT_TRUE (anArc.isClockWise() == Nova::kFalse);
  ExpectNear (anArc.refVec(), 1.0, 0.0);
  ExpectNear (anArc.startPoint(), 1.0, 0.0);
  ExpectNear (anArc.endPoint(), 1.0, 0.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, CenterRadiusConstructor_BuildsFullCircle)
{
  const NvGeCircArc2d anArc (NvGePoint2d (3.0, 4.0), 2.5);
  ExpectNear (anArc.center(), 3.0, 4.0);
  EXPECT_NEAR (anArc.radius(), 2.5, THE_TEST_TOL);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
  ExpectNear (anArc.startPoint(), 5.5, 4.0);
  ExpectNear (anArc.endPoint(), 5.5, 4.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, CenterRadiusConstructor_RejectsNonPositiveRadius)
{
  EXPECT_THROW (NvGeCircArc2d (NvGePoint2d (0.0, 0.0), 0.0), NvException);
  EXPECT_THROW (NvGeCircArc2d (NvGePoint2d (0.0, 0.0), -1.0), NvException);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, AngleConstructor_CcwArc_RoundTrips)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 2.0, 0.0, THE_PI / 2.0,
                             NvGeVector2d (1.0, 0.0), Nova::kFalse);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.isClockWise() == Nova::kFalse);
  ExpectNear (anArc.startPoint(), 2.0, 0.0);
  ExpectNear (anArc.endPoint(), 0.0, 2.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, AngleConstructor_ClockwiseArc_RoundTrips)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 2.0, THE_PI / 3.0, 5.0 * THE_PI / 3.0,
                             NvGeVector2d (1.0, 0.0), Nova::kTrue);
  EXPECT_NEAR (anArc.startAng(), THE_PI / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), 5.0 * THE_PI / 3.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.isClockWise() == Nova::kTrue);
  const double aSqrt3 = std::sqrt (3.0);
  ExpectNear (anArc.startPoint(), 1.0, aSqrt3);
  // PointAtAngle (5*PI/3) = 2 * (cos 5*PI/3, sin 5*PI/3) = (1, -sqrt (3)).
  ExpectNear (anArc.endPoint(), 1.0, -aSqrt3);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, AngleConstructor_EqualAngles_MakesFullCircle)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.5, 0.7, 0.7);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), 2.0 * THE_PI, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, ThreePointConstructor_BuildsCcwArcThroughMid)
{
  const NvGeCircArc2d anArc (NvGePoint2d (2.0, 0.0), NvGePoint2d (0.0, 2.0),
                             NvGePoint2d (-2.0, 0.0));
  ExpectNear (anArc.center(), 0.0, 0.0);
  EXPECT_NEAR (anArc.radius(), 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.isClockWise() == Nova::kFalse);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_PI, THE_TEST_TOL);
  ExpectNear (anArc.startPoint(), 2.0, 0.0);
  ExpectNear (anArc.endPoint(), -2.0, 0.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, ThreePointConstructor_CollinearPoints_Throw)
{
  EXPECT_THROW ((NvGeCircArc2d (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 0.0),
                                NvGePoint2d (2.0, 0.0))), NvException);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, SetThreePoints_ReportsErrorCodes_AndKeepsObject)
{
  NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  NvGeError anError = NvGe::kOk;
  anArc.set (NvGePoint2d (0.0, 0.0), NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0), anError);
  EXPECT_EQ (anError, NvGe::kEqualArg1Arg2);
  anArc.set (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0), NvGePoint2d (0.0, 0.0), anError);
  EXPECT_EQ (anError, NvGe::kEqualArg1Arg3);
  // arg2 == arg3 with a distinct arg1 isolates kEqualArg2Arg3 (an arg1 ==
  // arg3 input would trip the kEqualArg1Arg3 check first).
  anArc.set (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0), NvGePoint2d (1.0, 1.0), anError);
  EXPECT_EQ (anError, NvGe::kEqualArg2Arg3);
  anArc.set (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 0.0), NvGePoint2d (2.0, 0.0), anError);
  EXPECT_EQ (anError, NvGe::kLinearlyDependentArg1Arg2Arg3);
  // The object is unchanged on error.
  ExpectNear (anArc.center(), 0.0, 0.0);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  anArc.set (NvGePoint2d (2.0, 0.0), NvGePoint2d (0.0, 2.0), NvGePoint2d (-2.0, 0.0), anError);
  EXPECT_EQ (anError, NvGe::kOk);
  EXPECT_NEAR (anArc.radius(), 2.0, THE_TEST_TOL);
  ExpectNear (anArc.center(), 0.0, 0.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, BulgeConstructor_Sagitta_Semicircle)
{
  // bulge = 1 with bulgeFlag kTrue: sagitta equals the radius for a semicircle.
  const NvGeCircArc2d anArc (NvGePoint2d (-1.0, 0.0), NvGePoint2d (1.0, 0.0), 1.0);
  ExpectNear (anArc.center(), 0.0, 0.0);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  ExpectNear (anArc.startPoint(), -1.0, 0.0);
  ExpectNear (anArc.endPoint(), 1.0, 0.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, BulgeConstructor_TanQuarter_QuarterTurn)
{
  // bulge = tan(ang / 4) with bulgeFlag kFalse: a 90 degree turn gives tan (PI / 8).
  const double aBulge = std::tan (THE_PI / 8.0);
  const NvGeCircArc2d anArc (NvGePoint2d (1.0, 0.0), NvGePoint2d (0.0, 1.0), aBulge,
                             Nova::kFalse);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  ExpectNear (anArc.center(), 0.0, 0.0);
  ExpectNear (anArc.startPoint(), 1.0, 0.0);
  ExpectNear (anArc.endPoint(), 0.0, 1.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, IsInside_ClassifiesByRadius)
{
  const NvGeCircArc2d anArc (NvGePoint2d (1.0, 1.0), 2.0);
  EXPECT_TRUE (anArc.isInside (NvGePoint2d (1.5, 1.0)));
  EXPECT_TRUE (anArc.isInside (NvGePoint2d (3.5, 1.0)) == Nova::kFalse);
  EXPECT_TRUE (anArc.isInside (NvGePoint2d (1.0, -1.5)) == Nova::kFalse);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, IntersectWithLine_FindsTwoPoints)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  const NvGeLine2d aLine (NvGePoint2d (0.0, 0.5), NvGeVector2d (1.0, 0.0));
  int anIntn = 0;
  NvGePoint2d aP1, aP2;
  EXPECT_TRUE (anArc.intersectWith (aLine, anIntn, aP1, aP2));
  EXPECT_EQ (anIntn, 2);
  const double anX = std::sqrt (0.75);
  // Points are reported in ascending line parameter order.
  ExpectNear (aP1, -anX, 0.5);
  ExpectNear (aP2, anX, 0.5);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, IntersectWithLine_NoIntersection_ReturnsFalse)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  const NvGeLine2d aLine (NvGePoint2d (0.0, 2.0), NvGeVector2d (1.0, 0.0));
  int anIntn = 7;
  NvGePoint2d aP1, aP2;
  EXPECT_TRUE (anArc.intersectWith (aLine, anIntn, aP1, aP2) == Nova::kFalse);
  EXPECT_EQ (anIntn, 0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, IntersectWithArc_FindsTwoPoints)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  const NvGeCircArc2d anOther (NvGePoint2d (2.0, 0.0), 2.0);
  int anIntn = 0;
  NvGePoint2d aP1, aP2;
  EXPECT_TRUE (anArc.intersectWith (anOther, anIntn, aP1, aP2));
  EXPECT_EQ (anIntn, 2);
  const double aY = std::sqrt (0.9375);
  const bool aForward = std::abs (aP1.y - aY) <= THE_TEST_TOL
                     && std::abs (aP2.y + aY) <= THE_TEST_TOL;
  const bool aReversed = std::abs (aP1.y + aY) <= THE_TEST_TOL
                      && std::abs (aP2.y - aY) <= THE_TEST_TOL;
  EXPECT_TRUE (aForward || aReversed);
  EXPECT_NEAR (aP1.x, 0.25, THE_TEST_TOL);
  EXPECT_NEAR (aP2.x, 0.25, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, Tangent_AtBoundaryPoint_IsPerpendicularToRadius)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  NvGeLine2d aLine (NvGePoint2d (9.0, 9.0), NvGeVector2d (1.0, 0.0));
  NvGeError anError = NvGe::kOk;
  EXPECT_TRUE (anArc.tangent (NvGePoint2d (1.0, 0.0), aLine, NvGeContext::gTol, anError));
  EXPECT_EQ (anError, NvGe::kArg1OnThis);
  ExpectNear (aLine.pointOnLine(), 1.0, 0.0);
  EXPECT_NEAR (std::abs (aLine.direction().x), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (std::abs (aLine.direction().y), 1.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, Tangent_OutsidePoint_TouchesAtRadialProjection)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  NvGeLine2d aLine;
  NvGeError anError = NvGe::kOk;
  EXPECT_TRUE (anArc.tangent (NvGePoint2d (2.0, 0.0), aLine, NvGeContext::gTol, anError));
  EXPECT_EQ (anError, NvGe::kArg1TooBig);
  ExpectNear (aLine.pointOnLine(), 1.0, 0.0);
  EXPECT_NEAR (std::abs (aLine.direction().x), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (std::abs (aLine.direction().y), 1.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, Tangent_InsidePoint_ReturnsFalse)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  NvGeLine2d aLine;
  NvGeError anError = NvGe::kOk;
  EXPECT_TRUE (anArc.tangent (NvGePoint2d (0.5, 0.0), aLine, NvGeContext::gTol, anError)
               == Nova::kFalse);
  EXPECT_EQ (anError, NvGe::kArg1InsideThis);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, Setters_RoundTrip)
{
  NvGeCircArc2d anArc;
  anArc.setCenter (NvGePoint2d (1.0, 2.0));
  anArc.setRadius (3.0);
  anArc.setRefVec (NvGeVector2d (0.0, 1.0));
  anArc.setAngles (THE_PI / 2.0, THE_PI);
  ExpectNear (anArc.center(), 1.0, 2.0);
  EXPECT_NEAR (anArc.radius(), 3.0, THE_TEST_TOL);
  ExpectNear (anArc.refVec(), 0.0, 1.0);
  EXPECT_NEAR (anArc.startAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_PI, THE_TEST_TOL);
  ExpectNear (anArc.startPoint(), -2.0, 2.0);
  ExpectNear (anArc.endPoint(), 1.0, -1.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, SetToComplement_FlipsDirectionKeepingAngles)
{
  NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);
  anArc.setToComplement();
  EXPECT_TRUE (anArc.isClockWise() == Nova::kTrue);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, AssignmentOperator_SharesUntilModified)
{
  const NvGeCircArc2d aBase (NvGePoint2d (0.0, 0.0), 1.0);
  NvGeCircArc2d aCopy;
  aCopy = aBase;
  aCopy.setRadius (5.0);
  EXPECT_NEAR (aBase.radius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.radius(), 5.0, THE_TEST_TOL);
  ExpectNear (aCopy.center(), 0.0, 0.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, SetFillet_BetweenTwoAxes_TouchesBothLines)
{
  const NvGeLine2d aLine1 (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aLine2 (NvGePoint2d (0.0, 0.0), NvGeVector2d (0.0, 1.0));
  NvGeCircArc2d anArc;
  double aParam1 = 0.5;
  double aParam2 = 0.5;
  Nova::Boolean aSuccess = Nova::kFalse;
  anArc.set (aLine1, aLine2, 1.0, aParam1, aParam2, aSuccess);
  EXPECT_TRUE (aSuccess == Nova::kTrue);
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  ExpectNear (anArc.center(), 1.0, 1.0);
  // Parameters of tangency along the carrier lines.
  EXPECT_NEAR (aParam1, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, 1.0, THE_TEST_TOL);
  ExpectNear (anArc.startPoint(), 1.0, 0.0);
  ExpectNear (anArc.endPoint(), 0.0, 1.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, SetFillet_ParallelLines_HasNoSolution)
{
  const NvGeLine2d aLine1 (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aLine2 (NvGePoint2d (0.0, 3.0), NvGeVector2d (1.0, 0.0));
  NvGeCircArc2d anArc;
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  Nova::Boolean aSuccess = Nova::kTrue;
  anArc.set (aLine1, aLine2, 1.0, aParam1, aParam2, aSuccess);
  EXPECT_TRUE (aSuccess == Nova::kFalse);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, SetFillet_ThreeLines_InscribesIncircle)
{
  // Right triangle with vertices (0,0), (2,0), (0,2): inradius 2 - sqrt (2).
  const NvGeLine2d aLine1 (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aLine2 (NvGePoint2d (0.0, 0.0), NvGeVector2d (0.0, 1.0));
  const NvGeLine2d aLine3 (NvGePoint2d (2.0, 0.0), NvGeVector2d (-1.0, 1.0));
  NvGeCircArc2d anArc;
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  double aParam3 = 0.0;
  Nova::Boolean aSuccess = Nova::kFalse;
  anArc.set (aLine1, aLine2, aLine3, aParam1, aParam2, aParam3, aSuccess);
  EXPECT_TRUE (aSuccess == Nova::kTrue);
  const double aRadius = 2.0 - std::sqrt (2.0);
  EXPECT_NEAR (anArc.radius(), aRadius, 1e-8);
  ExpectNear (anArc.center(), aRadius, aRadius);
  // Tangency points: (r, 0) on line1, (0, r) on line2, (1, 1) on the hypotenuse.
  EXPECT_NEAR (aParam1, aRadius, 1e-8);
  EXPECT_NEAR (aParam2, aRadius, 1e-8);
  // Touch (1, 1) sits sqrt (2) along line3 from its anchor (2, 0).
  EXPECT_NEAR (aParam3, std::sqrt (2.0), 1e-8);
  ExpectNear (anArc.startPoint(), aRadius, 0.0);
  ExpectNear (anArc.endPoint(), 0.0, aRadius);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, EvalPoint_FollowsAngleParameterization)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 2.0, 0.0, THE_PI / 2.0);
  const double aSqrt2 = std::sqrt (2.0);
  ExpectNear (anArc.evalPoint (THE_PI / 4.0), aSqrt2, aSqrt2);
  ExpectNear (anArc.evalPoint (0.0), 2.0, 0.0);
  ExpectNear (anArc.evalPoint (THE_PI / 2.0), 0.0, 2.0);
}

//=================================================================================================

TEST_F (NvGeCircArc2dTest, IsClosed_TrueOnlyForFullCircle)
{
  EXPECT_TRUE (NvGeCircArc2d().isClosed());
  const NvGeCircArc2d aQuarter (NvGePoint2d (0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);
  EXPECT_TRUE (aQuarter.isClosed() == Nova::kFalse);
}
