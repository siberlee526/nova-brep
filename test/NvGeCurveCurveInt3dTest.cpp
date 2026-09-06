#include <NvException.h>
#include <gecint3d.h>
#include <geline3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

namespace
{
  // Tolerance for exact spatial values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeCurveCurveInt3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCurveCurveInt3dTest, CrossingLines_SingleTransversalPoint)
{
  // The x-axis and the line x = 3 in the z = 0 plane cross at (3, 0, 0).
  const NvGeLine3d anXAxis (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aVertical (NvGePoint3d (3.0, -2.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));

  const NvGeCurveCurveInt3d anInt (anXAxis, aVertical);

  ASSERT_EQ (anInt.numIntPoints(), 1);

  const NvGePoint3d anIntPnt = anInt.intPoint (0);
  EXPECT_NEAR (anIntPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.z, 0.0, THE_TEST_TOL);

  // The intersection parameter is the distance along each unit line.
  double aParam1 = 0.0;
  double aParam2 = 0.0;
  anInt.getIntParams (0, aParam1, aParam2);
  EXPECT_NEAR (aParam1, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, 2.0, THE_TEST_TOL);

  EXPECT_FALSE (anInt.isTangential (0));
  EXPECT_TRUE (anInt.isTransversal (0));
}

TEST_F (NvGeCurveCurveInt3dTest, SkewLines_NoIntersection)
{
  // The shifted line is parallel to the y direction one unit above the
  // x-axis: the minimum distance (1.0) exceeds the equality tolerance.
  const NvGeLine3d anXAxis (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aSkew (NvGePoint3d (0.0, 0.0, 1.0), NvGeVector3d (0.0, 1.0, 0.0));

  const NvGeCurveCurveInt3d anInt (anXAxis, aSkew);

  EXPECT_EQ (anInt.numIntPoints(), 0);
}

TEST_F (NvGeCurveCurveInt3dTest, IntPoint_OutOfRange_Throws)
{
  const NvGeLine3d anXAxis (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aVertical (NvGePoint3d (3.0, -2.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));

  const NvGeCurveCurveInt3d anInt (anXAxis, aVertical);
  ASSERT_EQ (anInt.numIntPoints(), 1);

  EXPECT_THROW (anInt.intPoint (1), NvException);

  // Predicates never throw: an out-of-range index reads as false.
  EXPECT_FALSE (anInt.isTangential (1));
  EXPECT_FALSE (anInt.isTransversal (1));
}
