// NvGePolyline2dTest.cpp - unit tests for NvGePolyline2d (OCCT-backed).
//
// NOTE: the EvalPoint test exercises NvGeCurve2d::evalPoint(); that lives in
// the base curve layer (gecurv2d.cpp) and pins the polyline parameterization
// contract: fit point i sits exactly at curve parameter i.

#include <NvException.h>
#include <gearc2d.h>
#include <geline2d.h>
#include <gekvec.h>
#include <gelnsg2d.h>
#include <gepnt2d.h>
#include <gept2dar.h>
#include <geplin2d.h>
#include <getol.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{

const double THE_TEST_TOL = 1e-9;

void ExpectNear (const NvGePoint2d& thePnt, double theX, double theY)
{
  EXPECT_NEAR (thePnt.x, theX, THE_TEST_TOL);
  EXPECT_NEAR (thePnt.y, theY, THE_TEST_TOL);
}

NvGePoint2dArray MakeThreePoints ()
{
  NvGePoint2dArray aPoints;
  aPoints.append (NvGePoint2d (0.0, 0.0));
  aPoints.append (NvGePoint2d (1.0, 3.0));
  aPoints.append (NvGePoint2d (4.0, 3.0));
  return aPoints;
}

}

class NvGePolyline2dTest : public testing::Test
{
protected:

  void SetUp () override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }

};

//=================================================================================================

TEST_F (NvGePolyline2dTest, DefaultConstructor_IsDegenerateLineAtOrigin)
{
  const NvGePolyline2d aPoly;
  EXPECT_EQ (aPoly.type(), NvGe::kPolyline2d);
  EXPECT_EQ (aPoly.numFitPoints(), 2);
  ExpectNear (aPoly.fitPointAt (0), 0.0, 0.0);
  ExpectNear (aPoly.fitPointAt (1), 0.0, 0.0);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, PointArrayConstructor_InterpolatesVertices)
{
  const NvGePolyline2d aPoly (MakeThreePoints());
  EXPECT_EQ (aPoly.numFitPoints(), 3);
  ExpectNear (aPoly.fitPointAt (0), 0.0, 0.0);
  ExpectNear (aPoly.fitPointAt (1), 1.0, 3.0);
  ExpectNear (aPoly.fitPointAt (2), 4.0, 3.0);
  // Clamped degree-1 knots: vertex i is interpolated at parameter i.
  EXPECT_EQ (aPoly.numKnots(), 5);
  EXPECT_NEAR (aPoly.knotAt (0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (2), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (3), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (4), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.endParam(), 2.0, THE_TEST_TOL);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, PointArrayConstructor_RejectsFewerThanTwoPoints)
{
  NvGePoint2dArray aPoints;
  aPoints.append (NvGePoint2d (0.0, 0.0));
  EXPECT_THROW ((NvGePolyline2d (aPoints)), NvException);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, KnotVectorConstructor_RoundTrips)
{
  const double aFlat[5] = {0.0, 0.0, 1.0, 2.0, 2.0};
  const NvGeKnotVector aKnots (5, aFlat);
  const NvGePolyline2d aPoly (aKnots, MakeThreePoints());
  EXPECT_EQ (aPoly.numFitPoints(), 3);
  EXPECT_EQ (aPoly.numKnots(), 5);
  EXPECT_NEAR (aPoly.knotAt (0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (2), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.knotAt (4), 2.0, THE_TEST_TOL);
  ExpectNear (aPoly.fitPointAt (2), 4.0, 3.0);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, KnotVectorConstructor_RejectsWrongKnotCount)
{
  const double aFlat[4] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aKnots (4, aFlat);
  EXPECT_THROW ((NvGePolyline2d (aKnots, MakeThreePoints())), NvException);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, SplineEntData_DescribesDegreeOneInterpolant)
{
  const NvGePolyline2d aPoly (MakeThreePoints());
  EXPECT_EQ (aPoly.degree(), 1);
  EXPECT_EQ (aPoly.order(), 2);
  EXPECT_TRUE (aPoly.isRational() == Nova::kFalse);
  EXPECT_TRUE (aPoly.hasFitData());
  EXPECT_EQ (aPoly.numControlPoints(), 3);
  // Continuity: -1 at clamped ends, 0 (C1) at interior knots.
  EXPECT_NEAR (aPoly.continuityAtKnot (0), -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.continuityAtKnot (2), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoly.continuityAtKnot (4), -1.0, THE_TEST_TOL);
  ExpectNear (aPoly.startPoint(), 0.0, 0.0);
  ExpectNear (aPoly.endPoint(), 4.0, 3.0);
  // Fit points coincide with control points for a degree-1 interpolant.
  ExpectNear (aPoly.controlPointAt (1), 1.0, 3.0);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, SetFitPointAt_ModifiesVertexWithoutTouchingCopies)
{
  const NvGePolyline2d aBase (MakeThreePoints());
  NvGePolyline2d aCopy (aBase);
  aCopy.setFitPointAt (1, NvGePoint2d (2.0, -1.0));
  ExpectNear (aCopy.fitPointAt (1), 2.0, -1.0);
  // The original is untouched (copy-on-write).
  ExpectNear (aBase.fitPointAt (1), 1.0, 3.0);
  ExpectNear (aBase.fitPointAt (0), 0.0, 0.0);
  ExpectNear (aBase.fitPointAt (2), 4.0, 3.0);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, ApproximationConstructor_FlattensLineSegment)
{
  const NvGeLineSeg2d aSeg (NvGePoint2d (0.0, 0.0), NvGePoint2d (4.0, 0.0));
  const NvGePolyline2d aPoly (aSeg, 0.1);
  EXPECT_EQ (aPoly.numFitPoints(), 2);
  ExpectNear (aPoly.fitPointAt (0), 0.0, 0.0);
  ExpectNear (aPoly.fitPointAt (1), 4.0, 0.0);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, ApproximationConstructor_SamplesCircleBySagitta)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  const NvGePolyline2d aPoly (anArc, 0.1);
  // Sagitta step 2*acos(1 - eps/r) ~= 0.902 rad -> 7 segments, 8 fit points.
  EXPECT_EQ (aPoly.numFitPoints(), 8);
  ExpectNear (aPoly.fitPointAt (0), 1.0, 0.0);
  // The polyline is closed: the last sample coincides with the first.
  EXPECT_NEAR (aPoly.fitPointAt (7).x, 1.0, 1e-9);
  EXPECT_NEAR (aPoly.fitPointAt (7).y, 0.0, 1e-9);
  for (int i = 0; i < aPoly.numFitPoints(); ++i)
  {
    const NvGePoint2d aPnt = aPoly.fitPointAt (i);
    EXPECT_NEAR (std::sqrt (aPnt.x * aPnt.x + aPnt.y * aPnt.y), 1.0, 1e-9);
  }
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, ApproximationConstructor_RejectsNonPositiveTolerance)
{
  const NvGeCircArc2d anArc (NvGePoint2d (0.0, 0.0), 1.0);
  EXPECT_THROW ((NvGePolyline2d (anArc, 0.0)), NvException);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, ApproximationConstructor_RejectsInfiniteLines)
{
  const NvGeLine2d aLine (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  EXPECT_THROW ((NvGePolyline2d (aLine, 0.1)), NvException);
}

//=================================================================================================

TEST_F (NvGePolyline2dTest, EvalPoint_AtKnots_ReturnsVertices)
{
  const NvGePolyline2d aPoly (MakeThreePoints());
  ExpectNear (aPoly.evalPoint (0.0), 0.0, 0.0);
  ExpectNear (aPoly.evalPoint (1.0), 1.0, 3.0);
  ExpectNear (aPoly.evalPoint (2.0), 4.0, 3.0);
}
