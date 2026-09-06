#include <geblok2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gegbl.h>

#include <gtest/gtest.h>

class NvGeBoundBlock2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeBoundBlock2dTest, BoxConstructor_ContainsCornersAndInterior)
{
  const NvGeBoundBlock2d aBlock (NvGePoint2d (1.0, 2.0), NvGePoint2d (3.0, 5.0));
  EXPECT_TRUE (aBlock.contains (NvGePoint2d (1.0, 2.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint2d (3.0, 5.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint2d (2.0, 3.5)));
  EXPECT_FALSE (aBlock.contains (NvGePoint2d (0.9, 3.0)));
  EXPECT_FALSE (aBlock.contains (NvGePoint2d (2.0, 5.1)));
}

TEST_F (NvGeBoundBlock2dTest, GetMinMaxPoints_ReturnsAxisAlignedHull)
{
  const NvGeBoundBlock2d aBlock (NvGePoint2d (3.0, 5.0), NvGePoint2d (1.0, 2.0));
  NvGePoint2d aMin, aMax;
  aBlock.getMinMaxPoints (aMin, aMax);
  EXPECT_NEAR (aMin.x, 1.0, 1e-9);
  EXPECT_NEAR (aMin.y, 2.0, 1e-9);
  EXPECT_NEAR (aMax.x, 3.0, 1e-9);
  EXPECT_NEAR (aMax.y, 5.0, 1e-9);
}

TEST_F (NvGeBoundBlock2dTest, Extend_GrowsToContainPoint)
{
  NvGeBoundBlock2d aBlock (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0));
  aBlock.extend (NvGePoint2d (2.0, -1.0));
  EXPECT_TRUE (aBlock.contains (NvGePoint2d (2.0, -1.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint2d (0.5, 0.5))); // old content kept
}

TEST_F (NvGeBoundBlock2dTest, Swell_ExpandsByDistance)
{
  const NvGeBoundBlock2d aBox (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0));
  NvGeBoundBlock2d aSwell = aBox;
  aSwell.swell (0.5);
  EXPECT_TRUE (aSwell.contains (NvGePoint2d (-0.4, 0.5)));
  EXPECT_TRUE (aSwell.contains (NvGePoint2d (1.4, 1.4)));
  EXPECT_FALSE (aSwell.contains (NvGePoint2d (-0.6, 0.5)));
}

TEST_F (NvGeBoundBlock2dTest, IsBox_AndSetToBox)
{
  NvGeBoundBlock2d aSkew (NvGePoint2d (0.0, 0.0), NvGeVector2d (2.0, 1.0),
                          NvGeVector2d (-1.0, 2.0));
  EXPECT_FALSE (aSkew.isBox());
  aSkew.setToBox (true);
  EXPECT_TRUE (aSkew.isBox());
  NvGePoint2d aMin, aMax;
  aSkew.getMinMaxPoints (aMin, aMax);
  EXPECT_NEAR (aMin.x, -1.0, 1e-9);
  EXPECT_NEAR (aMax.x, 2.0, 1e-9);
}

TEST_F (NvGeBoundBlock2dTest, IsDisjoint_Boxes)
{
  const NvGeBoundBlock2d aBlockA (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0));
  const NvGeBoundBlock2d aBlockB (NvGePoint2d (2.0, 2.0), NvGePoint2d (3.0, 3.0));
  const NvGeBoundBlock2d aBlockC (NvGePoint2d (0.5, 0.5), NvGePoint2d (2.0, 2.0));
  EXPECT_TRUE (aBlockA.isDisjoint (aBlockB));
  EXPECT_FALSE (aBlockA.isDisjoint (aBlockC));
}

TEST_F (NvGeBoundBlock2dTest, CopySharesGeometry_IndependentModification)
{
  NvGeBoundBlock2d aBlock (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 1.0));
  NvGeBoundBlock2d aCopy (aBlock);
  aCopy.extend (NvGePoint2d (5.0, 5.0));
  EXPECT_TRUE (aCopy.contains (NvGePoint2d (5.0, 5.0)));
  EXPECT_FALSE (aBlock.contains (NvGePoint2d (5.0, 5.0))); // COW kept the original
}
