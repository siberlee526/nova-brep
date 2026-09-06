#include <NvException.h>
#include <gepnt3d.h>
#include <gepos3d.h>
#include <getol.h>

#include <gtest/gtest.h>

namespace
{
  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGePosition3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGePosition3dTest, DefaultConstructor_IsOrigin)
{
  const NvGePosition3d aPos;

  EXPECT_TRUE (aPos.isKindOf (NvGe::kPosition3d));
  EXPECT_TRUE (aPos.isKindOf (NvGe::kPointEnt3d));

  const NvGePoint3d aPnt = aPos.point3d();
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePosition3dTest, Constructors_CarryGivenCoordinates)
{
  const NvGePoint3d aPnt = NvGePosition3d (NvGePoint3d (1.0, 2.0, 3.0)).point3d();
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 3.0, THE_TEST_TOL);

  const NvGePoint3d anOther = NvGePosition3d (4.0, 5.0, 6.0).point3d();
  EXPECT_NEAR (anOther.x, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (anOther.y, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (anOther.z, 6.0, THE_TEST_TOL);
}

TEST_F (NvGePosition3dTest, SetOverloads_ReplaceCoordinates)
{
  NvGePosition3d aPos;
  aPos.set (NvGePoint3d (1.0, -2.0, 3.0));

  NvGePoint3d aPnt = aPos.point3d();
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, -2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 3.0, THE_TEST_TOL);

  aPos.set (4.0, 5.0, -6.0);
  aPnt = aPos.point3d();
  EXPECT_NEAR (aPnt.x, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, -6.0, THE_TEST_TOL);
}

TEST_F (NvGePosition3dTest, ConversionOperator_ReturnsPoint3d)
{
  const NvGePosition3d aPos (1.0, 2.0, 3.0);
  const NvGePoint3d aPnt = aPos;

  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 3.0, THE_TEST_TOL);
}

TEST_F (NvGePosition3dTest, CopyOnWrite_CopyThenModifyOriginal_CopyUnaffected)
{
  const NvGePosition3d aPos (1.0, 1.0, 1.0);
  NvGePosition3d aMutable (aPos); // shares the implementation with aPos

  aMutable.set (2.0, 2.0, 2.0);

  EXPECT_TRUE (aPos.point3d().isEqualTo (NvGePoint3d (1.0, 1.0, 1.0)));
  EXPECT_TRUE (aMutable.point3d().isEqualTo (NvGePoint3d (2.0, 2.0, 2.0)));
}

TEST_F (NvGePosition3dTest, CopyOnWrite_AssignThenModifyCopy_OriginalUnaffected)
{
  const NvGePosition3d aPos (1.0, 2.0, 3.0);
  NvGePosition3d aCopy;
  aCopy = aPos;

  aCopy.set (7.0, 8.0, 9.0);

  EXPECT_TRUE (aPos.point3d().isEqualTo (NvGePoint3d (1.0, 2.0, 3.0)));
  EXPECT_TRUE (aCopy.point3d().isEqualTo (NvGePoint3d (7.0, 8.0, 9.0)));
}

TEST_F (NvGePosition3dTest, IsEqualTo_ComparesCoordinates)
{
  const NvGePosition3d aPos (1.0, 2.0, 3.0);
  const NvGePosition3d aSame (1.0, 2.0, 3.0);
  const NvGePosition3d anOther (1.0, 2.0, 3.5);

  EXPECT_TRUE (aPos.isEqualTo (aSame));
  EXPECT_FALSE (aPos.isEqualTo (anOther));
}

TEST_F (NvGePosition3dTest, IsOn_PointAtStoredPosition_True)
{
  const NvGePosition3d aPos (1.0, 2.0, 3.0);

  EXPECT_TRUE (aPos.isOn (NvGePoint3d (1.0, 2.0, 3.0)));
  EXPECT_FALSE (aPos.isOn (NvGePoint3d (5.0, 5.0, 5.0)));
}
