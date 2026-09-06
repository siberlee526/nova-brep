#include <NvException.h>
#include <gecurv2d.h>
#include <gegblge.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <geponc2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gtest/gtest.h>

namespace
{
  // Tolerance for exact planar values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGePointOnCurve2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }

  // Line through (1, 1) along the unit x axis: parameter t maps to (1+t, 1).
  static NvGeLine2d MakeTestLine()
  {
    return NvGeLine2d (NvGePoint2d (1.0, 1.0), NvGeVector2d (1.0, 0.0));
  }
};

TEST_F (NvGePointOnCurve2dTest, CurveParamConstructor_EvaluatesPointAtParameter)
{
  const NvGeLine2d aLine = MakeTestLine();
  const NvGePointOnCurve2d aPoc (aLine, 2.0);

  EXPECT_EQ (aPoc.type(), NvGe::kPointOnCurve2d);
  EXPECT_NEAR (aPoc.parameter(), 2.0, THE_TEST_TOL);

  const NvGePoint2d aPnt = aPoc.point();
  EXPECT_NEAR (aPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, PointConstructor_FromParamOf_BindsAtProjectedParameter)
{
  const NvGeLine2d aLine = MakeTestLine();
  // An unbounded line projects within its local window [-1, 1] (see
  // NvGeCurve2dTest), so the target stays inside that window: (1.5, 1.0)
  // lies at parameter 0.5 of the line through (1, 1) along (1, 0).
  const double aParam = aLine.paramOf (NvGePoint2d (1.5, 1.0));
  ASSERT_NEAR (aParam, 0.5, THE_TEST_TOL);

  const NvGePointOnCurve2d aPoc (aLine, aParam);
  EXPECT_NEAR (aPoc.parameter(), 0.5, THE_TEST_TOL);

  const NvGePoint2d aPnt = aPoc.point();
  EXPECT_NEAR (aPnt.x, 1.5, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, SetParameter_ReevaluatesStoredPoint)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGePointOnCurve2d aPoc (aLine, 2.0);

  aPoc.setParameter (0.5);
  EXPECT_NEAR (aPoc.parameter(), 0.5, THE_TEST_TOL);

  const NvGePoint2d aPnt = aPoc.point();
  EXPECT_NEAR (aPnt.x, 1.5, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, Deriv_FirstOrder_ReturnsLineDirection)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGePointOnCurve2d aPoc (aLine, 2.0);

  const NvGeVector2d aD1 = aPoc.deriv (1);
  EXPECT_NEAR (aD1.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aD1.y, 0.0, THE_TEST_TOL);

  // The parameter overload evaluates and adopts the given parameter.
  const NvGeVector2d aD1At = aPoc.deriv (1, 7.0);
  EXPECT_NEAR (aD1At.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aD1At.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc.parameter(), 7.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, Deriv_SecondOrder_LineHasZeroCurvatureDerivs)
{
  const NvGeLine2d aLine = MakeTestLine();
  const NvGePointOnCurve2d aPoc (aLine, 2.0);

  const NvGeVector2d aD2 = aPoc.deriv (2);
  EXPECT_NEAR (aD2.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aD2.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, Deriv_InvalidOrder_ThrowsNvException)
{
  const NvGeLine2d aLine = MakeTestLine();
  const NvGePointOnCurve2d aPoc (aLine, 2.0);

  EXPECT_THROW (aPoc.deriv (0), NvException);
  EXPECT_THROW (aPoc.deriv (4), NvException);
}

TEST_F (NvGePointOnCurve2dTest, SetCurve_OnDefaultPoint_BindsCurveKeepingParameter)
{
  NvGePointOnCurve2d aPoc;
  aPoc.setCurve (MakeTestLine());

  // Parameter is retained (0.0): the evaluated point is the line origin.
  const NvGePoint2d aPnt = aPoc.point();
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, CurveGetter_EvaluatesThroughBoundCurve)
{
  const NvGeLine2d aLine = MakeTestLine();
  const NvGePointOnCurve2d aPoc (aLine, 2.0);

  const NvGeCurve2d* aBound = aPoc.curve();
  ASSERT_TRUE (aBound != nullptr);
  EXPECT_TRUE (aBound->isKindOf (NvGe::kCurve2d));

  const NvGePoint2d aPnt = aBound->evalPoint (2.0);
  EXPECT_NEAR (aPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, UnboundPoint_QueriesThrow_AndSingularIsFalse)
{
  const NvGePointOnCurve2d aPoc;

  EXPECT_THROW (aPoc.point(), NvException);
  EXPECT_THROW (aPoc.deriv (1), NvException);
  EXPECT_THROW (aPoc.curve(), NvException);
  EXPECT_FALSE (aPoc.isSingular());
}

TEST_F (NvGePointOnCurve2dTest, Curvature_OnLine_IsZero)
{
  const NvGeLine2d aLine = MakeTestLine();
  NvGePointOnCurve2d aPoc (aLine, 2.0);

  double aValue = -1.0;
  EXPECT_TRUE (aPoc.curvature (aValue));
  EXPECT_NEAR (aValue, 0.0, THE_TEST_TOL);

  EXPECT_TRUE (aPoc.curvature (5.0, aValue));
  EXPECT_NEAR (aValue, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPoc.parameter(), 5.0, THE_TEST_TOL);
}

TEST_F (NvGePointOnCurve2dTest, CopyAndAssign_SetParameterOnCopy_KeepsOriginal)
{
  const NvGeLine2d aLine = MakeTestLine();
  const NvGePointOnCurve2d aPoc (aLine, 2.0);

  NvGePointOnCurve2d aCopy (aPoc);
  aCopy = aPoc;
  aCopy.setParameter (9.0);
  EXPECT_NEAR (aCopy.parameter(), 9.0, THE_TEST_TOL);

  // The original keeps its own parameter (fresh holder behind a detached
  // impl; the shared holder is never mutated).
  EXPECT_NEAR (aPoc.parameter(), 2.0, THE_TEST_TOL);
  const NvGePoint2d aPnt = aPoc.point();
  EXPECT_NEAR (aPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}
