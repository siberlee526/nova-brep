// NvGeCurve2dTest.cpp - unit tests for the generic 2d curve base NvGeCurve2d.
//
// The tests drive NvGeCurve2d through its concrete subclasses NvGeLine2d and
// NvGeCircArc2d: a quarter circle of radius 2 centered at the origin and the
// horizontal line y = 2 through (1, 2) cover every method of the base class.

#include <NvException.h>
#include <gearc2d.h>
#include <geblok2d.h>
#include <gecurv2d.h>
#include <gelent2d.h>
#include <gegblge.h>
#include <geline2d.h>
#include <geintrvl.h>
#include <getol.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;

  // Tolerance for values produced by iterative algorithms (paramAtLength,
  // getNormalPoint, projections near a window boundary).
  constexpr double THE_PROJECT_TOL = 1e-6;

  // Tolerance for the sampled area quadrature.
  constexpr double THE_AREA_TOL = 1e-3;

  // The quarter test arc: center (0, 0), radius 2, from (2, 0) to (0, 2).
  NvGeCircArc2d MakeQuarterArc()
  {
    return NvGeCircArc2d (NvGePoint2d (0.0, 0.0), 2.0, 0.0, THE_PI / 2.0);
  }

  // The horizontal test line y = 2 through (1, 2).
  NvGeLine2d MakeTestLine()
  {
    return NvGeLine2d (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  }

  void ExpectPntNear (const NvGePoint2d& thePnt, double theX, double theY, double theTol)
  {
    EXPECT_NEAR (thePnt.x, theX, theTol);
    EXPECT_NEAR (thePnt.y, theY, theTol);
  }

  void ExpectVecNear (const NvGeVector2d& theVec, double theX, double theY, double theTol)
  {
    EXPECT_NEAR (theVec.x, theX, theTol);
    EXPECT_NEAR (theVec.y, theY, theTol);
  }
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeCurve2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCurve2dTest, GetInterval_ArcBoundedRange)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGeInterval anIntrvl;
  anArc.getInterval (anIntrvl);
  EXPECT_TRUE (anIntrvl.isBounded());
  EXPECT_NEAR (anIntrvl.lowerBound(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), THE_PI / 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, GetInterval_LineUnbounded_PointsFilledOnlyWhenFinite)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGeInterval anIntrvl;
  NvGePoint2d aStart (7.0, 7.0);
  NvGePoint2d anEnd (7.0, 7.0);
  aLine.getInterval (anIntrvl, aStart, anEnd);
  EXPECT_TRUE (anIntrvl.isUnBounded());
  ExpectPntNear (aStart, 7.0, 7.0, 0.0); // untouched: no finite bounds to evaluate
  ExpectPntNear (anEnd, 7.0, 7.0, 0.0);

  const NvGeCircArc2d anArc = MakeQuarterArc();
  anArc.getInterval (anIntrvl, aStart, anEnd);
  ExpectPntNear (aStart, 2.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (anEnd, 0.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, ReverseParam_ArcSwapsEndPoints)
{
  NvGeCircArc2d anArc = MakeQuarterArc();
  anArc.reverseParam();
  NvGePoint2d aStart;
  NvGePoint2d anEnd;
  ASSERT_TRUE (anArc.hasStartPoint (aStart));
  ASSERT_TRUE (anArc.hasEndPoint (anEnd));
  ExpectPntNear (aStart, 0.0, 2.0, THE_TEST_TOL); // the old end point
  ExpectPntNear (anEnd, 2.0, 0.0, THE_TEST_TOL);  // the old start point
  NvGeInterval anIntrvl;
  anArc.getInterval (anIntrvl);
  // OCCT Geom2d_TrimmedCurve::Reverse () maps the window (u1, u2) to
  // (ReversedParameter (u2), ReversedParameter (u1)); a circle maps u to
  // 2*PI - u, so the reversed quarter arc keeps the raw window [3*PI/2, 2*PI]
  // on a handedness-flipped basis while running old end -> old start.
  EXPECT_NEAR (anIntrvl.lowerBound(), 1.5 * THE_PI, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), 2.0 * THE_PI, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, ReverseParam_LineStaysUnbounded)
{
  NvGeLine2d aLine = MakeTestLine();
  aLine.reverseParam();
  NvGeInterval anIntrvl;
  aLine.getInterval (anIntrvl);
  EXPECT_TRUE (anIntrvl.isUnBounded());
  NvGePoint2d aPnt;
  EXPECT_FALSE (aLine.hasStartPoint (aPnt));
  EXPECT_FALSE (aLine.hasEndPoint (aPnt));
  ExpectPntNear (aLine.evalPoint (0.0), 1.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, SetInterval_RestrictsAndResetRestoresFullCircle)
{
  NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_TRUE (anArc.setInterval (NvGeInterval (0.1, 0.7)));
  NvGeInterval anIntrvl;
  anArc.getInterval (anIntrvl);
  EXPECT_NEAR (anIntrvl.lowerBound(), 0.1, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), 0.7, THE_TEST_TOL);

  anArc.setInterval();
  anArc.getInterval (anIntrvl);
  EXPECT_NEAR (anIntrvl.lowerBound(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), 2.0 * THE_PI, THE_TEST_TOL);
  EXPECT_TRUE (anArc.isClosed());
  ExpectPntNear (anArc.evalPoint (0.0), 2.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, SetInterval_OutsideNaturalBounds_FalseAndUnchanged)
{
  NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.setInterval (NvGeInterval (-0.5, 0.7)));
  NvGeInterval anIntrvl;
  anArc.getInterval (anIntrvl);
  EXPECT_NEAR (anIntrvl.lowerBound(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), THE_PI / 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, SetInterval_Degenerate_Throws)
{
  NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_THROW (anArc.setInterval (NvGeInterval (0.5, 0.5)), NvException);
}

TEST_F (NvGeCurve2dTest, DistanceTo_Point)
{
  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_NEAR (aLine.distanceTo (NvGePoint2d (0.0, 5.0)), 3.0, THE_TEST_TOL);
  // An unbounded line projects within its local window [-1, 1], so the
  // closest point to (3, 5) is the window end point (2, 2).
  EXPECT_NEAR (aLine.distanceTo (NvGePoint2d (3.0, 5.0)), std::sqrt (10.0), THE_TEST_TOL);

  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_NEAR (anArc.distanceTo (NvGePoint2d (0.0, 0.0)), 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, ClosestPointTo_Point)
{
  const NvGeLine2d aLine = MakeTestLine();
  ExpectPntNear (aLine.closestPointTo (NvGePoint2d (0.0, 5.0)), 0.0, 2.0, THE_TEST_TOL);

  const NvGeCircArc2d anArc = MakeQuarterArc();
  ExpectPntNear (anArc.closestPointTo (NvGePoint2d (1.0, 1.0)),
                 std::sqrt (2.0), std::sqrt (2.0), THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, DistanceTo_Curve_ParallelLines)
{
  const NvGeLine2d aLine1 = MakeTestLine();
  const NvGeLine2d aLine2 (NvGePoint2d (1.0, 5.0), NvGeVector2d (1.0, 0.0));
  EXPECT_NEAR (aLine1.distanceTo (aLine2), 3.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, ClosestPointTo_Curve_ParallelLines)
{
  const NvGeLine2d aLine1 = MakeTestLine();
  const NvGeLine2d aLine2 (NvGePoint2d (1.0, 5.0), NvGeVector2d (1.0, 0.0));
  NvGePoint2d aPntOnOther;
  const NvGePoint2d aPntOnThis = aLine1.closestPointTo (aLine2, aPntOnOther);
  EXPECT_NEAR (aPntOnThis.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPntOnOther.y, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPntOnThis.x, aPntOnOther.x, THE_TEST_TOL); // perpendicular feet share x
}

TEST_F (NvGeCurve2dTest, GetClosestPointTo_Point)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGePointOnCurve2d aPoc;
  aLine.getClosestPointTo (NvGePoint2d (0.0, 5.0), aPoc);
  ExpectPntNear (aPoc.point(), 0.0, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc.parameter(), -1.0, THE_TEST_TOL); // Value(u) = (1 + u, 2)
}

TEST_F (NvGeCurve2dTest, GetClosestPointTo_Curve)
{
  const NvGeLine2d aLine1 = MakeTestLine();
  const NvGeLine2d aLine2 (NvGePoint2d (1.0, 5.0), NvGeVector2d (1.0, 0.0));
  NvGePointOnCurve2d aPoc1;
  NvGePointOnCurve2d aPoc2;
  aLine1.getClosestPointTo (aLine2, aPoc1, aPoc2);
  EXPECT_NEAR (aPoc1.point().y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc2.point().y, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc1.point().x, aPoc2.point().x, THE_TEST_TOL);
  EXPECT_NEAR (aPoc1.parameter(), aPoc2.parameter(), THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, GetNormalPoint_FullCircle)
{
  const NvGeCircArc2d aCircle (NvGePoint2d (0.0, 0.0), 2.0);
  NvGePointOnCurve2d aPoc;
  EXPECT_TRUE (aCircle.getNormalPoint (NvGePoint2d (0.0, 5.0), aPoc));
  EXPECT_NEAR (aPoc.parameter(), THE_PI / 2.0, THE_PROJECT_TOL);
  ExpectPntNear (aPoc.point(), 0.0, 2.0, THE_PROJECT_TOL);
}

TEST_F (NvGeCurve2dTest, GetNormalPoint_NoNormalWithinRange_False)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGePointOnCurve2d aPoc;
  // The foot of the normal from (2, -3) sits at the parameter atan2 (-3, 2),
  // outside the arc range [0, PI/2]; the residual never vanishes in between.
  EXPECT_FALSE (anArc.getNormalPoint (NvGePoint2d (2.0, -3.0), aPoc));
}

TEST_F (NvGeCurve2dTest, IsOn_Point)
{
  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_TRUE (aLine.isOn (NvGePoint2d (0.0, 2.0)));
  EXPECT_FALSE (aLine.isOn (NvGePoint2d (0.0, 2.5)));

  double aParam = -7.0;
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_TRUE (anArc.isOn (NvGePoint2d (2.0, 0.0), aParam));
  EXPECT_NEAR (aParam, 0.0, THE_TEST_TOL);
  EXPECT_FALSE (anArc.isOn (NvGePoint2d (2.0, 0.1), aParam));
}

TEST_F (NvGeCurve2dTest, IsOn_Param)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_TRUE (anArc.isOn (0.5));
  EXPECT_FALSE (anArc.isOn (5.0));
  EXPECT_FALSE (anArc.isOn (-1.0));
}

TEST_F (NvGeCurve2dTest, ParamOf)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_NEAR (anArc.paramOf (NvGePoint2d (0.0, 2.0)), THE_PI / 2.0, THE_TEST_TOL);

  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_NEAR (aLine.paramOf (NvGePoint2d (1.0, 2.0)), 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, GetTrimmedOffset)
{
  // The offset normal is T ^ Z: for the line direction (1, 0) the +2 offset
  // moves the line y = 2 down to y = 0.
  const NvGeLine2d aLine = MakeTestLine();
  NvGeVoidPointerArray aLineList;
  aLine.getTrimmedOffset (2.0, aLineList);
  ASSERT_EQ (aLineList.length(), 1);
  NvGeCurve2d* aOffsetLine = static_cast<NvGeCurve2d*> (aLineList[0]);
  ASSERT_NE (aOffsetLine, nullptr);
  EXPECT_EQ (aOffsetLine->type(), NvGe::kOffsetCurve2d);
  ExpectPntNear (aOffsetLine->evalPoint (0.0), 1.0, 0.0, THE_TEST_TOL);
  delete aOffsetLine;

  // For the CCW quarter arc the +0.5 offset at parameter 0 points outwards,
  // growing the radius from 2 to 2.5.
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGeVoidPointerArray anArcList;
  anArc.getTrimmedOffset (0.5, anArcList);
  ASSERT_EQ (anArcList.length(), 1);
  NvGeCurve2d* aOffsetArc = static_cast<NvGeCurve2d*> (anArcList[0]);
  ASSERT_NE (aOffsetArc, nullptr);
  ExpectPntNear (aOffsetArc->evalPoint (0.0), 2.5, 0.0, THE_TEST_TOL);
  delete aOffsetArc;
}

TEST_F (NvGeCurve2dTest, IsClosed)
{
  const NvGeCircArc2d aCircle (NvGePoint2d (0.0, 0.0), 2.0);
  EXPECT_TRUE (aCircle.isClosed());
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isClosed());
  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_FALSE (aLine.isClosed());
}

TEST_F (NvGeCurve2dTest, IsPeriodic)
{
  double aPeriod = 0.0;
  const NvGeCircArc2d aCircle (NvGePoint2d (0.0, 0.0), 2.0);
  EXPECT_TRUE (aCircle.isPeriodic (aPeriod));
  EXPECT_NEAR (aPeriod, 2.0 * THE_PI, THE_TEST_TOL);

  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isPeriodic (aPeriod));

  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_FALSE (aLine.isPeriodic (aPeriod));
}

TEST_F (NvGeCurve2dTest, IsLinear)
{
  NvGeLine2d aLine;
  const NvGeLine2d aLineEnt = MakeTestLine();
  EXPECT_TRUE (aLineEnt.isLinear (aLine));
  ExpectPntNear (aLine.pointOnLine(), 1.0, 2.0, THE_TEST_TOL);
  ExpectVecNear (aLine.direction(), 1.0, 0.0, THE_TEST_TOL);

  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isLinear (aLine));
}

TEST_F (NvGeCurve2dTest, Length)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_NEAR (anArc.length (0.0, THE_PI / 2.0), THE_PI, THE_TEST_TOL);
  EXPECT_NEAR (anArc.length (THE_PI / 2.0, 0.0), THE_PI, THE_TEST_TOL); // swapped args
  EXPECT_NEAR (anArc.length (-10.0, 10.0), THE_PI, THE_TEST_TOL);       // clamped to the range

  const NvGeCircArc2d aCircle (NvGePoint2d (0.0, 0.0), 2.0);
  EXPECT_NEAR (aCircle.length (0.0, 2.0 * THE_PI), 4.0 * THE_PI, THE_TEST_TOL);

  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_NEAR (aLine.length (-1.0, 1.0), 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, ParamAtLength)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  EXPECT_NEAR (anArc.paramAtLength (0.0, THE_PI / 2.0), THE_PI / 4.0, THE_PROJECT_TOL);
  EXPECT_NEAR (anArc.paramAtLength (0.0, THE_PI), THE_PI / 2.0, THE_PROJECT_TOL); // reaches the end
  EXPECT_NEAR (anArc.paramAtLength (0.0, 0.0), 0.0, THE_TEST_TOL);                // <= tol: the datum
  EXPECT_NEAR (anArc.paramAtLength (THE_PI / 2.0, THE_PI / 2.0, Nova::kFalse),
               THE_PI / 4.0, THE_PROJECT_TOL);
}

TEST_F (NvGeCurve2dTest, Area)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  double aValue = 0.0;
  EXPECT_TRUE (anArc.area (0.0, THE_PI / 2.0, aValue));
  // Area between the quarter arc and its chord: r^2 / 2 * (theta - sin(theta)).
  EXPECT_NEAR (aValue, THE_PI - 2.0, THE_AREA_TOL);

  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_TRUE (aLine.area (0.0, 1.0, aValue));
  EXPECT_NEAR (aValue, 0.0, THE_TEST_TOL); // a straight enclosure is degenerate
}

TEST_F (NvGeCurve2dTest, IsDegenerate)
{
  NvGeCircArc2d anArc = MakeQuarterArc();
  NvGe::EntityId aType = NvGe::kEntity2d;
  EXPECT_FALSE (anArc.isDegenerate (aType));

  // Shrink the window below the point tolerance: the curve collapses to a point.
  EXPECT_TRUE (anArc.setInterval (NvGeInterval (0.5, 0.5 + 1e-9)));
  EXPECT_TRUE (anArc.isDegenerate (aType));
  EXPECT_EQ (aType, NvGe::kPointEnt2d);

  NvGeEntity2d* aConverted = nullptr;
  EXPECT_TRUE (anArc.isDegenerate (aConverted));
  ASSERT_NE (aConverted, nullptr);
  EXPECT_EQ (aConverted->type(), NvGe::kPosition2d);
  delete aConverted;

  NvGeCircArc2d anOtherArc = MakeQuarterArc();
  NvGeEntity2d* aNotConverted = &anOtherArc;
  EXPECT_FALSE (anOtherArc.isDegenerate (aNotConverted));
  EXPECT_EQ (aNotConverted, nullptr);
}

TEST_F (NvGeCurve2dTest, GetSplitCurves)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGeCurve2d* aPiece1 = nullptr;
  NvGeCurve2d* aPiece2 = nullptr;
  anArc.getSplitCurves (THE_PI / 4.0, aPiece1, aPiece2);
  ASSERT_NE (aPiece1, nullptr);
  ASSERT_NE (aPiece2, nullptr);
  EXPECT_EQ (aPiece1->type(), anArc.type());
  ExpectPntNear (aPiece1->evalPoint (0.0), 2.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (aPiece1->evalPoint (THE_PI / 4.0), std::sqrt (2.0), std::sqrt (2.0), THE_TEST_TOL);
  ExpectPntNear (aPiece2->evalPoint (THE_PI / 4.0), std::sqrt (2.0), std::sqrt (2.0), THE_TEST_TOL);
  ExpectPntNear (aPiece2->evalPoint (THE_PI / 2.0), 0.0, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPiece1->length (0.0, THE_PI / 4.0), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPiece2->length (THE_PI / 4.0, THE_PI / 2.0), THE_PI / 2.0, THE_TEST_TOL);
  delete aPiece1;
  delete aPiece2;

  NvGeLine2d aLine = MakeTestLine();
  EXPECT_THROW (anArc.getSplitCurves (0.0, aPiece1, aPiece2), NvException);
  EXPECT_THROW (anArc.getSplitCurves (THE_PI / 2.0, aPiece1, aPiece2), NvException);
  EXPECT_THROW (aLine.getSplitCurves (0.5, aPiece1, aPiece2), NvException); // unbounded curve
}

TEST_F (NvGeCurve2dTest, Explode_ReturnsFalse)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGeVoidPointerArray aList;
  NvGeIntArray aGroups;
  EXPECT_FALSE (anArc.explode (aList, aGroups));
}

TEST_F (NvGeCurve2dTest, GetLocalClosestPoints_Point)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGePointOnCurve2d aPoc;
  anArc.getLocalClosestPoints (NvGePoint2d (0.0, 3.0), aPoc);
  EXPECT_NEAR (aPoc.parameter(), THE_PI / 2.0, THE_PROJECT_TOL);
  ExpectPntNear (aPoc.point(), 0.0, 2.0, THE_PROJECT_TOL);

  // Within the neighborhood [0, 0.5] the closest foot is the window end 0.5.
  const NvGeInterval aNbhd (0.0, 0.5);
  anArc.getLocalClosestPoints (NvGePoint2d (0.0, 3.0), aPoc, &aNbhd);
  EXPECT_NEAR (aPoc.parameter(), 0.5, THE_PROJECT_TOL);
}

TEST_F (NvGeCurve2dTest, GetLocalClosestPoints_Curve)
{
  const NvGeLine2d aLine1 = MakeTestLine();
  const NvGeLine2d aLine2 (NvGePoint2d (1.0, 5.0), NvGeVector2d (1.0, 0.0));
  NvGePointOnCurve2d aPoc1;
  NvGePointOnCurve2d aPoc2;
  aLine1.getLocalClosestPoints (aLine2, aPoc1, aPoc2);
  EXPECT_NEAR (aPoc1.point().y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc2.point().y, 5.0, THE_TEST_TOL);

  const NvGeInterval aNbhd (0.5, 1.0);
  aLine1.getLocalClosestPoints (aLine2, aPoc1, aPoc2, &aNbhd);
  EXPECT_GE (aPoc1.parameter(), 0.5 - 1e-9);
  EXPECT_LE (aPoc1.parameter(), 1.0 + 1e-9);
}

TEST_F (NvGeCurve2dTest, BoundBlock_Arc)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();

  NvGePoint2d aMinPnt;
  NvGePoint2d aMaxPnt;
  anArc.boundBlock().getMinMaxPoints (aMinPnt, aMaxPnt);
  ExpectPntNear (aMinPnt, 0.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (aMaxPnt, 2.0, 2.0, THE_TEST_TOL);

  anArc.orthoBoundBlock().getMinMaxPoints (aMinPnt, aMaxPnt);
  ExpectPntNear (aMinPnt, 0.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (aMaxPnt, 2.0, 2.0, THE_TEST_TOL);

  const NvGeInterval aRange (0.0, THE_PI / 4.0);
  anArc.orthoBoundBlock (aRange).getMinMaxPoints (aMinPnt, aMaxPnt);
  ExpectPntNear (aMinPnt, std::sqrt (2.0), 0.0, THE_TEST_TOL);
  ExpectPntNear (aMaxPnt, 2.0, std::sqrt (2.0), THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, BoundBlock_Line)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGePoint2d aBase;
  NvGeVector2d aDir1;
  NvGeVector2d aDir2;
  aLine.boundBlock().get (aBase, aDir1, aDir2);
  ExpectPntNear (aBase, 1.0, 2.0, THE_TEST_TOL);
  ExpectVecNear (aDir1, 0.0, 0.0, THE_TEST_TOL); // a free line is unbounded
  ExpectVecNear (aDir2, 0.0, 0.0, THE_TEST_TOL);

  NvGeLine2d aSegment = MakeTestLine();
  EXPECT_TRUE (aSegment.setInterval (NvGeInterval (0.0, 5.0)));
  aSegment.boundBlock().get (aBase, aDir1, aDir2);
  ExpectPntNear (aBase, 1.0, 2.0, THE_TEST_TOL);
  ExpectVecNear (aDir1, 5.0, 0.0, THE_TEST_TOL); // exact oriented block along the segment
  ExpectVecNear (aDir2, 0.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, HasStartAndEndPoint)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGePoint2d aStart;
  NvGePoint2d anEnd;
  EXPECT_TRUE (anArc.hasStartPoint (aStart));
  EXPECT_TRUE (anArc.hasEndPoint (anEnd));
  ExpectPntNear (aStart, 2.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (anEnd, 0.0, 2.0, THE_TEST_TOL);

  const NvGeLine2d aLine = MakeTestLine();
  EXPECT_FALSE (aLine.hasStartPoint (aStart));
  EXPECT_FALSE (aLine.hasEndPoint (anEnd));
}

TEST_F (NvGeCurve2dTest, EvalPoint_Derivatives)
{
  const double aSqrt2 = std::sqrt (2.0);
  const NvGeCircArc2d anArc = MakeQuarterArc();
  ExpectPntNear (anArc.evalPoint (THE_PI / 4.0), aSqrt2, aSqrt2, THE_TEST_TOL);

  NvGeVector2dArray aDerivs;
  const NvGePoint2d aPnt = anArc.evalPoint (THE_PI / 4.0, 2, aDerivs);
  ExpectPntNear (aPnt, aSqrt2, aSqrt2, THE_TEST_TOL);
  ASSERT_EQ (aDerivs.length(), 2);
  ExpectVecNear (aDerivs[0], -aSqrt2, aSqrt2, THE_TEST_TOL);
  ExpectVecNear (aDerivs[1], -aSqrt2, -aSqrt2, THE_TEST_TOL);

  NvGeVector2dArray aFirst;
  anArc.evalPoint (0.0, 1, aFirst);
  ASSERT_EQ (aFirst.length(), 1);
  ExpectVecNear (aFirst[0], 0.0, 2.0, THE_TEST_TOL);

  NvGeVector2dArray aNone;
  anArc.evalPoint (0.0, 0, aNone);
  EXPECT_EQ (aNone.length(), 0);

  EXPECT_THROW (anArc.evalPoint (0.0, 4, aNone), NvException);
}

TEST_F (NvGeCurve2dTest, GetSamplePoints_Count)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGePoint2dArray aPoints;
  anArc.getSamplePoints (5, aPoints);
  ASSERT_EQ (aPoints.length(), 5);
  ExpectPntNear (aPoints[0], 2.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (aPoints[2], std::sqrt (2.0), std::sqrt (2.0), THE_TEST_TOL);
  ExpectPntNear (aPoints[4], 0.0, 2.0, THE_TEST_TOL);
  EXPECT_THROW (anArc.getSamplePoints (1, aPoints), NvException);
}

TEST_F (NvGeCurve2dTest, GetSamplePoints_Deflection)
{
  const NvGeCircArc2d anArc = MakeQuarterArc();
  NvGePoint2dArray aPoints;
  NvGeDoubleArray aParams;
  anArc.getSamplePoints (0.0, THE_PI / 2.0, 1e-4, aPoints, aParams);
  ASSERT_GE (aPoints.length(), 2);
  ASSERT_EQ (aPoints.length(), aParams.length());
  EXPECT_NEAR (aParams.first(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aParams.last(), THE_PI / 2.0, THE_TEST_TOL);
  for (int anIdx = 1; anIdx < aParams.length(); ++anIdx)
  {
    EXPECT_GE (aParams[anIdx], aParams[anIdx - 1] - 1e-12);
  }
  ExpectPntNear (aPoints.first(), 2.0, 0.0, THE_TEST_TOL);
  ExpectPntNear (aPoints.last(), 0.0, 2.0, THE_TEST_TOL);

  const NvGeLine2d aLine = MakeTestLine();
  aLine.getSamplePoints (-1.0, 1.0, 1e-4, aPoints, aParams);
  ASSERT_GE (aPoints.length(), 2);
  EXPECT_NEAR (aParams.first(), -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aParams.last(), 1.0, THE_TEST_TOL);
  ExpectPntNear (aPoints.first(), 0.0, 2.0, THE_TEST_TOL);
  ExpectPntNear (aPoints.last(), 2.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve2dTest, OperatorAssign_CopyOnWriteDetaches)
{
  NvGeCircArc2d anArcA = MakeQuarterArc();
  NvGeCircArc2d anArcB = MakeQuarterArc();
  NvGeCurve2d& aCrvA = anArcA;
  NvGeCurve2d& aCrvB = anArcB;
  aCrvA = aCrvB; // NvGeCurve2d::operator= shares the impl

  // Restricting the copy must detach it and leave the source untouched.
  EXPECT_TRUE (aCrvA.setInterval (NvGeInterval (0.1, 0.7)));
  NvGeInterval anIntrvlA;
  aCrvA.getInterval (anIntrvlA);
  EXPECT_NEAR (anIntrvlA.lowerBound(), 0.1, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvlA.upperBound(), 0.7, THE_TEST_TOL);

  NvGeInterval anIntrvlB;
  aCrvB.getInterval (anIntrvlB);
  EXPECT_NEAR (anIntrvlB.lowerBound(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvlB.upperBound(), THE_PI / 2.0, THE_TEST_TOL);
}
