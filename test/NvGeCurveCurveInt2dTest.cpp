#include <NvException.h>
#include <gecint2d.h>
#include <geline2d.h>
#include <gepnt2d.h>
#include <geintrvl.h>
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
class NvGeCurveCurveInt2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCurveCurveInt2dTest, CrossingLines_SingleTransversalPoint)
{
  // The x-axis and the vertical line x = 3 cross at (3, 0).
  const NvGeLine2d anXAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (3.0, -2.0), NvGeVector2d (0.0, 1.0));

  const NvGeCurveCurveInt2d anInt (anXAxis, aVertical);

  ASSERT_EQ (anInt.numIntPoints(), 1);

  const NvGePoint2d anIntPnt = anInt.intPoint (0);
  EXPECT_NEAR (anIntPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);

  // The intersection parameter is the distance along each unit line.
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  anInt.getIntParams (0, aParam1, aParam2);
  EXPECT_NEAR (aParam1, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, 2.0, THE_TEST_TOL);

  EXPECT_FALSE (anInt.isTangential (0));
  EXPECT_TRUE (anInt.isTransversal (0));

  // Two crossing lines share no overlapping portion.
  EXPECT_EQ (anInt.overlapCount(), 0);
}

TEST_F (NvGeCurveCurveInt2dTest, ParallelLines_NoIntersection)
{
  const NvGeLine2d anXAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aShifted (NvGePoint2d (0.0, 1.0), NvGeVector2d (1.0, 0.0));

  const NvGeCurveCurveInt2d anInt (anXAxis, aShifted);

  EXPECT_EQ (anInt.numIntPoints(), 0);
  EXPECT_EQ (anInt.overlapCount(), 0);
}

TEST_F (NvGeCurveCurveInt2dTest, RestrictedRange_ExcludingCrossing_NoIntersection)
{
  // The first line is restricted to x in [4, 6], which excludes the
  // crossing point at x = 3.
  const NvGeLine2d anXAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (3.0, -2.0), NvGeVector2d (0.0, 1.0));

  const NvGeCurveCurveInt2d anInt (anXAxis, aVertical, NvGeInterval (4.0, 6.0),
                                   NvGeInterval());

  EXPECT_EQ (anInt.numIntPoints(), 0);
}

TEST_F (NvGeCurveCurveInt2dTest, Set_RunsTheIntersectionAgain)
{
  const NvGeLine2d anXAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (3.0, -2.0), NvGeVector2d (0.0, 1.0));

  NvGeCurveCurveInt2d anInt;
  anInt.set (anXAxis, aVertical);

  ASSERT_EQ (anInt.numIntPoints(), 1);
  const NvGePoint2d anIntPnt = anInt.intPoint (0);
  EXPECT_NEAR (anIntPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCurveCurveInt2dTest, IntPoint_OutOfRange_Throws)
{
  const NvGeLine2d anXAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (3.0, -2.0), NvGeVector2d (0.0, 1.0));

  const NvGeCurveCurveInt2d anInt (anXAxis, aVertical);
  ASSERT_EQ (anInt.numIntPoints(), 1);

  EXPECT_THROW (anInt.intPoint (1), NvException);

  // Predicates never throw: an out-of-range index reads as false.
  EXPECT_FALSE (anInt.isTangential (1));
  EXPECT_FALSE (anInt.isTransversal (1));
}
