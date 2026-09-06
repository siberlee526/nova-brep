#include <NvException.h>
#include <gearc3d.h>
#include <geblok3d.h>
#include <gecurv3d.h>
#include <geline3d.h>
#include <geintrvl.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;
  constexpr double THE_SQRT2 = 1.41421356237309504880;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;

  // Quarter circle, radius 2, in the z = 0 plane: param 0 -> (2, 0, 0),
  // param pi/2 -> (0, 2, 0); the arc length is pi.
  NvGeCircArc3d MakeQuarterArc()
  {
    return NvGeCircArc3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                          NvGeVector3d (1.0, 0.0, 0.0), 2.0, 0.0, THE_PI / 2.0);
  }

  NvGeLine3d MakeXAxisLine()
  {
    return NvGeLine3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  }
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeCurve3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

//=================================================================================================
// Evaluate methods.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, EvalPoint_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();

  const NvGePoint3d aPnt = aLine.evalPoint (2.5);
  EXPECT_NEAR (aPnt.x, 2.5, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, EvalPoint_Arc_EndpointsAndMidpoint)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();

  NvGePoint3d aPnt = anArc.evalPoint (0.0);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);

  aPnt = anArc.evalPoint (THE_PI / 4.0);
  EXPECT_NEAR (aPnt.x, THE_SQRT2, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, THE_SQRT2, THE_TEST_TOL);

  aPnt = anArc.evalPoint (THE_PI / 2.0);
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, EvalPoint_Derivatives_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();

  NvGeVector3dArray aDerivs;
  const NvGePoint3d aPnt = aLine.evalPoint (2.0, 1, aDerivs);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  ASSERT_EQ (aDerivs.length(), 1);
  EXPECT_TRUE (aDerivs[0].isEqualTo (NvGeVector3d (1.0, 0.0, 0.0)));

  NvGeVector3dArray aNoDerivs;
  const NvGePoint3d aBase = aLine.evalPoint (0.0, 0, aNoDerivs);
  EXPECT_NEAR (aBase.x, 0.0, THE_TEST_TOL);
  EXPECT_EQ (aNoDerivs.length(), 0);
}

TEST_F (NvGeCurve3dTest, EvalPoint_Derivatives_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();

  NvGeVector3dArray aDerivs;
  const NvGePoint3d aPnt = anArc.evalPoint (THE_PI / 4.0, 3, aDerivs);
  EXPECT_NEAR (aPnt.x, THE_SQRT2, THE_TEST_TOL);
  ASSERT_EQ (aDerivs.length(), 3);
  // D1 = r * (-sin u, cos u, 0), D2 = r * (-cos u, -sin u, 0),
  // D3 = r * (sin u, -cos u, 0).
  EXPECT_TRUE (aDerivs[0].isEqualTo (NvGeVector3d (-THE_SQRT2, THE_SQRT2, 0.0)));
  EXPECT_TRUE (aDerivs[1].isEqualTo (NvGeVector3d (-THE_SQRT2, -THE_SQRT2, 0.0)));
  EXPECT_TRUE (aDerivs[2].isEqualTo (NvGeVector3d (THE_SQRT2, -THE_SQRT2, 0.0)));
}

TEST_F (NvGeCurve3dTest, EvalPoint_InvalidDerivativeOrder_Throws)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGeVector3dArray aDerivs;
  EXPECT_THROW (aLine.evalPoint (0.0, 4, aDerivs), NvException);
  EXPECT_THROW (aLine.evalPoint (0.0, -1, aDerivs), NvException);
}

//=================================================================================================
// Parametrization.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, ParamOf_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_NEAR (aLine.paramOf (NvGePoint3d (3.0, 0.0, 0.0)), 3.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, ParamOf_Arc_RoundTrip)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  const double aParams[] = {0.0, THE_PI / 8.0, THE_PI / 4.0, THE_PI / 2.0};
  for (const double aU : aParams)
  {
    const NvGePoint3d aPnt = anArc.evalPoint (aU);
    EXPECT_NEAR (anArc.paramOf (aPnt), aU, 1e-9);
  }
}

TEST_F (NvGeCurve3dTest, GetInterval_Line_Unbounded)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGeInterval anIntrvl;
  aLine.getInterval (anIntrvl);
  EXPECT_TRUE (anIntrvl.isUnBounded());
}

TEST_F (NvGeCurve3dTest, GetInterval_Arc_BoundedWithEndPoints)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();

  NvGeInterval anIntrvl;
  anArc.getInterval (anIntrvl);
  EXPECT_TRUE (anIntrvl.isBounded());
  EXPECT_NEAR (anIntrvl.lowerBound(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.upperBound(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntrvl.length(), THE_PI / 2.0, THE_TEST_TOL);

  NvGePoint3d aStart;
  NvGePoint3d anEnd;
  anArc.getInterval (anIntrvl, aStart, anEnd);
  EXPECT_NEAR (aStart.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aStart.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, SetInterval_RetrimsArc)
{
  NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_TRUE (anArc.setInterval (NvGeInterval (0.0, THE_PI / 4.0)));

  // Length of an eighth circle with radius 2 is pi / 2.
  EXPECT_NEAR (anArc.length (0.0, THE_PI / 4.0), THE_PI / 2.0, 1e-9);
  const NvGePoint3d anEnd = anArc.evalPoint (THE_PI / 4.0);
  EXPECT_NEAR (anEnd.x, THE_SQRT2, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.y, THE_SQRT2, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, SetInterval_OutsideBasisDomain_ReturnsFalse)
{
  NvGeCircArc3d anArc = MakeQuarterArc();
  // The basis is the full circle with domain [0, 2 pi]; beyond that the
  // re-trim is rejected.
  EXPECT_FALSE (anArc.setInterval (NvGeInterval (7.0, 8.0)));
}

TEST_F (NvGeCurve3dTest, RestoreNaturalInterval_DropsTrimming)
{
  NvGeCircArc3d anArc = MakeQuarterArc();
  anArc.setInterval (NvGeInterval (0.0, THE_PI / 4.0));

  anArc.setInterval();

  // The natural bounds are the full circle: closed, length 2 pi r.
  EXPECT_TRUE (anArc.isClosed());
  EXPECT_NEAR (anArc.length (0.0, 2.0 * THE_PI), 4.0 * THE_PI, 1e-9);
}

TEST_F (NvGeCurve3dTest, ReverseParam_Line_ReversesDirection)
{
  NvGeLine3d aLine = MakeXAxisLine();
  aLine.reverseParam();

  const NvGePoint3d aPnt = aLine.evalPoint (3.0);
  EXPECT_NEAR (aPnt.x, -3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, ReverseParam_Arc_PreservesPointSet)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGeCircArc3d aReversed (anArc);
  aReversed.reverseParam();

  const double aParams[] = {0.0, THE_PI / 8.0, THE_PI / 4.0, 3.0 * THE_PI / 8.0,
                            THE_PI / 2.0};
  for (const double aU : aParams)
  {
    EXPECT_TRUE (aReversed.isOn (anArc.evalPoint (aU)));
  }
}

//=================================================================================================
// Length based methods.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, Length_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_NEAR (aLine.length (1.0, 4.0), 3.0, 1e-9);
  // Argument order does not matter.
  EXPECT_NEAR (aLine.length (4.0, 1.0), 3.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, Length_Line_UnboundedDomain)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const double anInf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE (std::isinf (aLine.length (-anInf, anInf)));
}

TEST_F (NvGeCurve3dTest, Length_Arc_QuarterCircle)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_NEAR (anArc.length (0.0, THE_PI / 2.0), THE_PI, 1e-9);
}

TEST_F (NvGeCurve3dTest, ParamAtLength_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  // An arc length of pi (a quarter of the circle 2 pi r = 4 pi) from the
  // start lands at the end parameter.
  EXPECT_NEAR (anArc.paramAtLength (0.0, THE_PI), THE_PI / 2.0, 1e-9);
  // And backwards from the end.
  EXPECT_NEAR (anArc.paramAtLength (THE_PI / 2.0, THE_PI, Adesk::kFalse), 0.0,
               1e-9);
}

TEST_F (NvGeCurve3dTest, ParamAtLength_NegativeLength_Throws)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_THROW (aLine.paramAtLength (0.0, -1.0), NvException);
}

TEST_F (NvGeCurve3dTest, ParamAtLength_ExceedsCurveLength_Throws)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_THROW (anArc.paramAtLength (0.0, 100.0), NvException);
}

TEST_F (NvGeCurve3dTest, Area_QuarterCircleSegment)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  double aValue = 0.0;
  EXPECT_TRUE (anArc.area (0.0, THE_PI / 2.0, aValue));
  // The region between the quarter arc and its chord is a circular segment:
  // (r^2 / 2) * (theta - sin theta) = 2 * (pi/2 - 1).
  EXPECT_NEAR (aValue, 2.0 * (THE_PI / 2.0 - 1.0), 1e-3);
}

TEST_F (NvGeCurve3dTest, Area_InvalidRange_ReturnsFalse)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  double aValue = 0.0;
  EXPECT_FALSE (anArc.area (THE_PI / 2.0, 0.0, aValue));
}

//=================================================================================================
// Degeneracy.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, IsDegenerate_NonDegenerateCurves)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGe::EntityId aType = NvGe::kEntity3d;
  EXPECT_FALSE (aLine.isDegenerate (aType));
  EXPECT_EQ (aType, NvGe::kCurve3d);

  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isDegenerate (aType));
  EXPECT_EQ (aType, NvGe::kCurve3d);
}

//=================================================================================================
// Distance and closest points.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, DistanceTo_Point)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_NEAR (aLine.distanceTo (NvGePoint3d (3.0, 4.0, 0.0)), 4.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, DistanceTo_Curve_ParallelLines)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGeLine3d anOther (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aLine.distanceTo (anOther), 1.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, ClosestPointTo_Line_Point)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGePoint3d aPnt = aLine.closestPointTo (NvGePoint3d (2.0, 1.0, 0.0));
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, GetClosestPointTo_Point)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGePointOnCurve3d aPntOnCrv;
  aLine.getClosestPointTo (NvGePoint3d (2.0, 1.0, 0.0), aPntOnCrv);
  EXPECT_NEAR (aPntOnCrv.parameter(), 2.0, THE_TEST_TOL);
  const NvGePoint3d aPnt = aPntOnCrv.point();
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, ClosestPointTo_Curve_ParallelLines)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGeLine3d anOther (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  NvGePoint3d aPntOnOther;
  const NvGePoint3d aPntOnThis = aLine.closestPointTo (anOther, aPntOnOther);
  EXPECT_NEAR (aPntOnThis.distanceTo (aPntOnOther), 1.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, GetClosestPointTo_Curve_ParallelLines)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGeLine3d anOther (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  NvGePointOnCurve3d aPntOnThis;
  NvGePointOnCurve3d aPntOnOther;
  aLine.getClosestPointTo (anOther, aPntOnThis, aPntOnOther);
  EXPECT_NEAR (aPntOnThis.point().distanceTo (aPntOnOther.point()), 1.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, ProjClosestPointTo_Arc_AlongPlaneNormal)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  // Projecting (3, 3, -1) along z onto the curve plane keeps x and y, and the
  // nearest arc point to the projected (3, 3) is the unique 45 degree point
  // (an input projecting onto the center would tie at every arc point).
  const NvGePoint3d aPnt
    = anArc.projClosestPointTo (NvGePoint3d (3.0, 3.0, -1.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aPnt.x, THE_SQRT2, 1e-9);
  EXPECT_NEAR (aPnt.y, THE_SQRT2, 1e-9);
}

TEST_F (NvGeCurve3dTest, GetProjClosestPointTo_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePointOnCurve3d aPntOnCrv;
  // (3, 3, -1) projects along z onto (3, 3, 0), whose closest arc point is
  // the unique 45 degree point (an input projecting onto the center would
  // tie at every arc point).
  anArc.getProjClosestPointTo (NvGePoint3d (3.0, 3.0, -1.0), NvGeVector3d (0.0, 0.0, 1.0),
                               aPntOnCrv);
  EXPECT_NEAR (aPntOnCrv.parameter(), THE_PI / 4.0, 1e-9);
}

//=================================================================================================
// Normal point.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, GetNormalPoint_Arc_PointOnNormal)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  // (1, 1, 0) lies on the outward normal of the 45 degree point, so there the
  // connection is perpendicular to the tangent; it is the unique arc point
  // with that property for this input (the center would tie at every point).
  NvGePointOnCurve3d aPntOnCrv;
  EXPECT_TRUE (anArc.getNormalPoint (NvGePoint3d (1.0, 1.0, 0.0), aPntOnCrv));
  EXPECT_NEAR (aPntOnCrv.parameter(), THE_PI / 4.0, 1e-9);
  const NvGePoint3d aPnt = aPntOnCrv.point();
  EXPECT_NEAR (aPnt.x, THE_SQRT2, 1e-9);
  EXPECT_NEAR (aPnt.y, THE_SQRT2, 1e-9);
}

TEST_F (NvGeCurve3dTest, GetNormalPoint_Arc_NotPerpendicular_ReturnsFalse)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  // The closest point to (3, -1, 0) is the arc start (2, 0, 0); the
  // connection there is not perpendicular to the tangent (0, 1, 0).
  NvGePointOnCurve3d aPntOnCrv;
  EXPECT_FALSE (anArc.getNormalPoint (NvGePoint3d (3.0, -1.0, 0.0), aPntOnCrv));
}

//=================================================================================================
// Bounding blocks.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, BoundBlock_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  const NvGeBoundBlock3d aBlock = anArc.boundBlock();
  NvGePoint3d aMinPnt;
  NvGePoint3d aMaxPnt;
  aBlock.getMinMaxPoints (aMinPnt, aMaxPnt);
  EXPECT_NEAR (aMinPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMinPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMinPnt.z, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMaxPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aMaxPnt.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aMaxPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, BoundBlock_Arc_Range)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  const NvGeBoundBlock3d aBlock = anArc.boundBlock (NvGeInterval (THE_PI / 4.0, THE_PI / 2.0));
  NvGePoint3d aMinPnt;
  NvGePoint3d aMaxPnt;
  aBlock.getMinMaxPoints (aMinPnt, aMaxPnt);
  EXPECT_NEAR (aMinPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMinPnt.y, THE_SQRT2, THE_TEST_TOL);
  EXPECT_NEAR (aMaxPnt.x, THE_SQRT2, THE_TEST_TOL);
  EXPECT_NEAR (aMaxPnt.y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, OrthoBoundBlock_MatchesBoundBlock)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePoint3d aMinPnt;
  NvGePoint3d aMaxPnt;
  anArc.orthoBoundBlock (NvGeInterval (THE_PI / 4.0, THE_PI / 2.0))
    .getMinMaxPoints (aMinPnt, aMaxPnt);
  EXPECT_NEAR (aMinPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMinPnt.y, THE_SQRT2, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, BoundBlock_RangeMissingDomain_Throws)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_THROW (anArc.boundBlock (NvGeInterval (5.0, 6.0)), NvException);
}

//=================================================================================================
// Project methods.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, OrthoProject_Line_ReturnsNurbPolyline)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGePlane aPlane (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));

  NvGeEntity3d* aProjected = aLine.orthoProject (aPlane);
  ASSERT_NE (aProjected, nullptr);
  EXPECT_TRUE (aProjected->isKindOf (NvGe::kNurbCurve3d));

  // The line already lies in the plane, so the projected polyline is the
  // line segment itself.
  const NvGeCurve3d* aPolyline = (const NvGeCurve3d*) aProjected;
  EXPECT_NEAR (aPolyline->distanceTo (NvGePoint3d (0.5, 0.0, 0.0)), 0.0, 1e-9);
}

TEST_F (NvGeCurve3dTest, Project_ParallelDirection_Throws)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGePlane aPlane (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_THROW (aLine.project (aPlane, NvGeVector3d (1.0, 0.0, 0.0)), NvException);
}

//=================================================================================================
// Tests if point is on curve.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, IsOn_Line_PointVariants)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_TRUE (aLine.isOn (NvGePoint3d (4.0, 0.0, 0.0)));
  EXPECT_FALSE (aLine.isOn (NvGePoint3d (0.0, 0.001, 0.0)));

  // A loose tolerance accepts the point 1e-4 off the line.
  NvGeTol aLoose;
  aLoose.setEqualPoint (1e-3);
  double aParam = 0.0;
  EXPECT_TRUE (aLine.isOn (NvGePoint3d (0.002, 1e-4, 0.0), aParam, aLoose));
  EXPECT_FALSE (aLine.isOn (NvGePoint3d (5.0, 5.0, 0.0)));
}

TEST_F (NvGeCurve3dTest, IsOn_Arc_PointAndParam)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();

  double aParam = 0.0;
  EXPECT_TRUE (anArc.isOn (NvGePoint3d (THE_SQRT2, THE_SQRT2, 0.0), aParam));
  EXPECT_NEAR (aParam, THE_PI / 4.0, 1e-9);

  EXPECT_TRUE (anArc.isOn (THE_PI / 4.0));
  EXPECT_FALSE (anArc.isOn (2.0));
}

//=================================================================================================
// Offset.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, GetTrimmedOffset_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGeVoidPointerArray anOffsets;
  aLine.getTrimmedOffset (1.0, NvGeVector3d (0.0, 0.0, 1.0), anOffsets);
  ASSERT_EQ (anOffsets.length(), 1);

  const NvGeCurve3d* anOffsetCurve = (const NvGeCurve3d*) anOffsets[0];
  ASSERT_NE (anOffsetCurve, nullptr);
  EXPECT_TRUE (anOffsetCurve->isKindOf (NvGe::kOffsetCurve3d));
  // OCCT offsets along N = D1 ^ V: offsetting the x-axis by +1 with normal
  // +z yields the line y = -1 (same convention as the NvGeCurve2d tests).
  EXPECT_NEAR (anOffsetCurve->distanceTo (NvGePoint3d (3.0, -1.0, 0.0)), 0.0, 1e-9);
}

//=================================================================================================
// Geometric inquiry methods.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, IsClosed)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  EXPECT_FALSE (aLine.isClosed());

  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isClosed());

  // The full circle is closed.
  const NvGeCircArc3d aCircle (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                               2.0);
  EXPECT_TRUE (aCircle.isClosed());
}

TEST_F (NvGeCurve3dTest, IsPlanar_Line)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGePlane aPlane;
  EXPECT_TRUE (aLine.isPlanar (aPlane));
  EXPECT_NEAR (std::abs (aPlane.normal().z), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, IsLinear_Line_FastPath)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGeLine3d aRecovered;
  EXPECT_TRUE (aLine.isLinear (aRecovered));
  EXPECT_TRUE (aRecovered.direction().isEqualTo (NvGeVector3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aRecovered.pointOnLine().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
}

TEST_F (NvGeCurve3dTest, IsLinear_Arc_ReturnsFalse)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGeLine3d aLine;
  EXPECT_FALSE (anArc.isLinear (aLine));
}

TEST_F (NvGeCurve3dTest, IsCoplanarWith)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePlane aPlane;
  // Both lie in the z = 0 plane.
  EXPECT_TRUE (aLine.isCoplanarWith (anArc, aPlane));
  EXPECT_NEAR (std::abs (aPlane.normal().z), 1.0, THE_TEST_TOL);

  // A line in z = 1 running along y is skew to the x-axis.
  const NvGeLine3d aSkew (NvGePoint3d (0.0, 0.0, 1.0), NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_FALSE (aLine.isCoplanarWith (aSkew, aPlane));
}

TEST_F (NvGeCurve3dTest, IsPeriodic)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  double aPeriod = 0.0;
  EXPECT_FALSE (aLine.isPeriodic (aPeriod));

  // A trimmed arc is not periodic.
  const NvGeCircArc3d anArc = MakeQuarterArc();
  EXPECT_FALSE (anArc.isPeriodic (aPeriod));
}

//=================================================================================================
// Return start and end points.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, HasStartAndEndPoint_Line_Infinite)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGePoint3d aPnt;
  EXPECT_FALSE (aLine.hasStartPoint (aPnt));
  EXPECT_FALSE (aLine.hasEndPoint (aPnt));
}

TEST_F (NvGeCurve3dTest, HasStartAndEndPoint_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePoint3d aStart;
  NvGePoint3d anEnd;
  EXPECT_TRUE (anArc.hasStartPoint (aStart));
  EXPECT_TRUE (anArc.hasEndPoint (anEnd));
  EXPECT_NEAR (aStart.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aStart.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.y, 2.0, THE_TEST_TOL);
}

//=================================================================================================
// Modify methods.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, GetSplitCurves_Arc)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGeCurve3d* aPiece1 = nullptr;
  NvGeCurve3d* aPiece2 = nullptr;
  anArc.getSplitCurves (THE_PI / 4.0, aPiece1, aPiece2);
  ASSERT_NE (aPiece1, nullptr);
  ASSERT_NE (aPiece2, nullptr);

  EXPECT_NEAR (aPiece1->length (0.0, THE_PI / 4.0), THE_PI / 2.0, 1e-9);
  const NvGePoint3d aStart = aPiece1->evalPoint (0.0);
  EXPECT_NEAR (aStart.x, 2.0, THE_TEST_TOL);

  const NvGePoint3d anEnd = aPiece2->evalPoint (THE_PI / 2.0);
  EXPECT_NEAR (anEnd.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, GetSplitCurves_OutsideDomain_Throws)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGeCurve3d* aPiece1 = nullptr;
  NvGeCurve3d* aPiece2 = nullptr;
  EXPECT_THROW (anArc.getSplitCurves (2.0, aPiece1, aPiece2), NvException);
}

TEST_F (NvGeCurve3dTest, Explode_ReturnsFalse)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGeVoidPointerArray aCurves;
  NvGeIntArray anOwners;
  EXPECT_FALSE (aLine.explode (aCurves, anOwners));
}

//=================================================================================================
// Local closest points.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, GetLocalClosestPoints_Point_NeighborhoodRestricts)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  // Globally the closest point to (100, 1, 0) is (100, 0, 0); restricted to
  // the [0, 4] neighborhood it is the window end (4, 0, 0).
  const NvGeInterval aNbhd (0.0, 4.0);
  NvGePointOnCurve3d aPntOnCrv;
  aLine.getLocalClosestPoints (NvGePoint3d (100.0, 1.0, 0.0), aPntOnCrv, &aNbhd);
  EXPECT_NEAR (aPntOnCrv.parameter(), 4.0, THE_TEST_TOL);
  const NvGePoint3d aPnt = aPntOnCrv.point();
  EXPECT_NEAR (aPnt.x, 4.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, GetLocalClosestPoints_CurveCurve_ParallelLines)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  const NvGeLine3d anOther (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  const NvGeInterval aNbhd1 (0.0, 4.0);
  const NvGeInterval aNbhd2 (0.0, 4.0);
  NvGePointOnCurve3d aPntOnThis;
  NvGePointOnCurve3d aPntOnOther;
  aLine.getLocalClosestPoints (anOther, aPntOnThis, aPntOnOther, &aNbhd1, &aNbhd2);
  EXPECT_NEAR (aPntOnThis.point().distanceTo (aPntOnOther.point()), 1.0, 1e-9);
}

//=================================================================================================
// Polygonize curve to within a specified tolerance.
//
//=================================================================================================

TEST_F (NvGeCurve3dTest, GetSamplePoints_Arc_UniformCount)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePoint3dArray aPoints;
  NvGeDoubleArray aParams;
  anArc.getSamplePoints (9, aPoints, aParams);
  ASSERT_EQ (aPoints.length(), 9);
  ASSERT_EQ (aParams.length(), 9);

  EXPECT_NEAR (aPoints[0].x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoints[0].y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoints[8].x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoints[8].y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aParams[0], 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aParams[4], THE_PI / 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aParams[8], THE_PI / 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, GetSamplePoints_Arc_CountOnly)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePoint3dArray aPoints;
  anArc.getSamplePoints (5, aPoints);
  ASSERT_EQ (aPoints.length(), 5);
  EXPECT_NEAR (aPoints[0].x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoints[4].y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCurve3dTest, GetSamplePoints_Arc_Deflection)
{
  const NvGeCircArc3d anArc = MakeQuarterArc();
  NvGePoint3dArray aPoints;
  NvGeDoubleArray aParams;
  anArc.getSamplePoints (0.0, THE_PI / 2.0, 1e-4, aPoints, aParams);
  ASSERT_GE (aPoints.length(), 2);
  ASSERT_EQ (aPoints.length(), aParams.length());

  EXPECT_NEAR (aPoints[0].x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoints[0].y, 0.0, THE_TEST_TOL);
  for (int i = 0; i < aParams.length(); ++i)
  {
    EXPECT_GE (aParams[i], -THE_TEST_TOL);
    EXPECT_LE (aParams[i], THE_PI / 2.0 + THE_TEST_TOL);
  }
}

TEST_F (NvGeCurve3dTest, GetSamplePoints_TooFewSamples_Throws)
{
  const NvGeLine3d aLine = MakeXAxisLine();
  NvGePoint3dArray aPoints;
  EXPECT_THROW (aLine.getSamplePoints (1, aPoints), NvException);
}
