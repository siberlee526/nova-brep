#include <NvException.h>
#include <geent2d.h>
#include <gepnt2d.h>
#include <gepos2d.h>
#include <getol.h>

#include <gtest/gtest.h>

namespace
{
  // Tolerance for exact planar values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGePosition2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGePosition2dTest, DefaultConstructor_IsOrigin)
{
  const NvGePosition2d aPos;
  ASSERT_EQ (aPos.type(), NvGe::kPosition2d);

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, PointConstructor_StoresCoordinates)
{
  const NvGePosition2d aPos (NvGePoint2d (1.5, -2.5));

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x,  1.5, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, -2.5, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, XYConstructor_StoresCoordinates)
{
  const NvGePosition2d aPos (3.0, 4.0);

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 4.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, Set_PointVector_UpdatesCoordinates)
{
  NvGePosition2d aPos;
  aPos.set (NvGePoint2d (7.0, 8.0));

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x, 7.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 8.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, Set_XY_UpdatesCoordinates)
{
  NvGePosition2d aPos;
  aPos.set (-1.0, 2.0);

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y,  2.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, ConversionOperator_ReturnsPoint2d)
{
  const NvGePosition2d aPos (NvGePoint2d (5.0, 6.0));
  const NvGePoint2d aPnt = aPos;

  EXPECT_NEAR (aPnt.x, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 6.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, Type_IsPosition2d_AndKindOfPointEnt2d)
{
  const NvGePosition2d aPos;
  EXPECT_TRUE (aPos.isKindOf (NvGe::kPosition2d));
  EXPECT_TRUE (aPos.isKindOf (NvGe::kPointEnt2d));
  EXPECT_TRUE (aPos.isKindOf (NvGe::kEntity2d));
  EXPECT_FALSE (aPos.isKindOf (NvGe::kLine2d));
}

TEST_F (NvGePosition2dTest, PointEntBase_DispatchesPointQuery)
{
  const NvGePosition2d aPos (NvGePoint2d (2.0, 3.0));
  const NvGePointEnt2d& anEnt = aPos;

  const NvGePoint2d aPnt = anEnt.point2d();
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 3.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, TransformBy_InheritedBase_TranslatesStoredPoint)
{
  NvGePosition2d aPos (NvGePoint2d (1.0, 2.0));
  aPos.translateBy (NvGeVector2d (3.0, -1.0));

  const NvGePoint2d aPnt = aPos.point2d();
  EXPECT_NEAR (aPnt.x, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePosition2dTest, OperatorAssign_CopyOnWrite_KeepsOriginal)
{
  NvGePosition2d aPos (NvGePoint2d (1.0, 2.0));
  NvGePosition2d aCopy;
  aCopy = aPos;
  EXPECT_TRUE (aPos.isEqualTo (aCopy));

  // The copy must stay independent: setting it must not move the original.
  aCopy.set (NvGePoint2d (9.0, 9.0));

  const NvGePoint2d anOriginal = aPos.point2d();
  EXPECT_NEAR (anOriginal.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOriginal.y, 2.0, THE_TEST_TOL);
  EXPECT_FALSE (aPos.isEqualTo (aCopy));
}

TEST_F (NvGePosition2dTest, CopyConstructor_ProducesIndependentEqualPosition)
{
  const NvGePosition2d aPos (NvGePoint2d (1.0, 2.0));
  const NvGePosition2d aCopy (aPos);

  EXPECT_TRUE (aPos.isEqualTo (aCopy));

  NvGePosition2d aModified (aCopy);
  aModified.set (NvGePoint2d (0.0, 0.0));
  const NvGePoint2d anUntouched = aCopy.point2d();
  EXPECT_NEAR (anUntouched.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anUntouched.y, 2.0, THE_TEST_TOL);
}
