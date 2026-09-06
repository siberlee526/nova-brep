// NvGeBoundBlock3dTest.cpp - unit tests for NvGeBoundBlock3d.

#include <geblok3d.h>
#include <gepnt3d.h>
#include <gegbl.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

class NvGeBoundBlock3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeBoundBlock3dTest, BoxConstructor_ContainsCornersAndInterior)
{
  const NvGeBoundBlock3d aBlock (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (2.0, 0.0, 0.0),
                                 NvGeVector3d (0.0, 4.0, 0.0), NvGeVector3d (0.0, 0.0, 5.0));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (1.0, 2.0, 3.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (3.0, 6.0, 8.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (2.0, 4.0, 5.5)));
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (0.9, 4.0, 5.0)));
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (2.0, 6.1, 5.0)));
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (2.0, 4.0, 8.1)));
}

TEST_F (NvGeBoundBlock3dTest, GetMinMaxPoints_ReturnsAxisAlignedHull)
{
  const NvGeBoundBlock3d aBlock (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (2.0, 0.0, 0.0),
                                 NvGeVector3d (0.0, 4.0, 0.0), NvGeVector3d (0.0, 0.0, 6.0));
  NvGePoint3d aMin, aMax;
  aBlock.getMinMaxPoints (aMin, aMax);
  EXPECT_NEAR (aMin.x, 0.0, 1e-9);
  EXPECT_NEAR (aMin.y, 0.0, 1e-9);
  EXPECT_NEAR (aMin.z, 0.0, 1e-9);
  EXPECT_NEAR (aMax.x, 2.0, 1e-9);
  EXPECT_NEAR (aMax.y, 4.0, 1e-9);
  EXPECT_NEAR (aMax.z, 6.0, 1e-9);
}

TEST_F (NvGeBoundBlock3dTest, Get_ReturnsBaseAndDirections)
{
  const NvGeBoundBlock3d aBlock (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (2.0, 0.0, 0.0),
                                 NvGeVector3d (1.0, 2.0, 0.0), NvGeVector3d (0.0, 0.0, 3.0));
  NvGePoint3d aBase;
  NvGeVector3d aDir1, aDir2, aDir3;
  aBlock.get (aBase, aDir1, aDir2, aDir3);
  EXPECT_NEAR (aBase.x, 1.0, 1e-9);
  EXPECT_NEAR (aBase.y, 2.0, 1e-9);
  EXPECT_NEAR (aBase.z, 3.0, 1e-9);
  EXPECT_NEAR (aDir1.x, 2.0, 1e-9);
  EXPECT_NEAR (aDir2.y, 2.0, 1e-9);
  EXPECT_NEAR (aDir3.z, 3.0, 1e-9);
}

TEST_F (NvGeBoundBlock3dTest, Set_TwoPoints_MakesAxisAlignedBox)
{
  NvGeBoundBlock3d aBlock;
  aBlock.set (NvGePoint3d (3.0, 5.0, 7.0), NvGePoint3d (1.0, 2.0, 4.0));
  NvGePoint3d aMin, aMax;
  aBlock.getMinMaxPoints (aMin, aMax);
  EXPECT_NEAR (aMin.x, 1.0, 1e-9);
  EXPECT_NEAR (aMin.y, 2.0, 1e-9);
  EXPECT_NEAR (aMin.z, 4.0, 1e-9);
  EXPECT_NEAR (aMax.x, 3.0, 1e-9);
  EXPECT_NEAR (aMax.y, 5.0, 1e-9);
  EXPECT_NEAR (aMax.z, 7.0, 1e-9);
  EXPECT_TRUE (aBlock.isBox());
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (2.0, 3.5, 5.5)));
}

TEST_F (NvGeBoundBlock3dTest, Extend_GrowsToContainPoint)
{
  NvGeBoundBlock3d aBlock (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (2.0, 0.0, 0.0),
                           NvGeVector3d (1.0, 2.0, 0.0), NvGeVector3d (0.0, 0.0, 3.0));
  aBlock.extend (NvGePoint3d (4.0, 2.0, 0.0));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (4.0, 2.0, 0.0)));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (1.0, 1.0, 1.5))); // interior kept
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (2.0, 2.0, 3.0))); // old corner kept
}

TEST_F (NvGeBoundBlock3dTest, Swell_ExpandsByDistance)
{
  const NvGeBoundBlock3d aBox (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (2.0, 0.0, 0.0),
                               NvGeVector3d (0.0, 4.0, 0.0), NvGeVector3d (0.0, 0.0, 6.0));
  NvGeBoundBlock3d aSwell = aBox;
  aSwell.swell (0.5);
  EXPECT_TRUE (aSwell.contains (NvGePoint3d (-0.4, 2.0, 3.0)));
  EXPECT_TRUE (aSwell.contains (NvGePoint3d (1.0, -0.4, 6.4)));
  EXPECT_FALSE (aSwell.contains (NvGePoint3d (-0.6, 2.0, 3.0)));
}

TEST_F (NvGeBoundBlock3dTest, UnboundedDirection_ZeroVectorMakesInfiniteSlab)
{
  const NvGeBoundBlock3d aBlock (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 0.0),
                                 NvGeVector3d (0.0, 4.0, 0.0), NvGeVector3d (0.0, 0.0, 6.0));
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (1000.0, 2.0, 3.0)));  // free along x
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (-1000.0, 0.0, 6.0))); // free along x
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (1000.0, 4.1, 3.0))); // y still bounded
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (1000.0, 2.0, 6.1))); // z still bounded
}

TEST_F (NvGeBoundBlock3dTest, DefaultCtor_IsFullyUnbounded_ContainsEverything)
{
  const NvGeBoundBlock3d aBlock;
  EXPECT_TRUE (aBlock.contains (NvGePoint3d (1.0e6, -1.0e6, 1.0e6)));
}

TEST_F (NvGeBoundBlock3dTest, IsBox_AndSetToBox)
{
  NvGeBoundBlock3d aSkew (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (2.0, 1.0, 0.0),
                          NvGeVector3d (-1.0, 2.0, 0.0), NvGeVector3d (0.0, 0.0, 3.0));
  EXPECT_FALSE (aSkew.isBox());
  aSkew.setToBox (true);
  EXPECT_TRUE (aSkew.isBox());
  NvGePoint3d aMin, aMax;
  aSkew.getMinMaxPoints (aMin, aMax);
  EXPECT_NEAR (aMin.x, -1.0, 1e-9);
  EXPECT_NEAR (aMax.x, 2.0, 1e-9);
  EXPECT_NEAR (aMax.y, 3.0, 1e-9);
  EXPECT_NEAR (aMax.z, 3.0, 1e-9);
  aSkew.setToBox (false); // no-op on an already general representation
  EXPECT_TRUE (aSkew.isBox());
}

TEST_F (NvGeBoundBlock3dTest, IsDisjoint_Boxes)
{
  const NvGeBoundBlock3d aBlockA (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0),
                                  NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeBoundBlock3d aBlockB (NvGePoint3d (2.0, 2.0, 2.0), NvGeVector3d (1.0, 0.0, 0.0),
                                  NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeBoundBlock3d aBlockC (NvGePoint3d (0.5, 0.5, 0.5), NvGeVector3d (1.0, 0.0, 0.0),
                                  NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_TRUE (aBlockA.isDisjoint (aBlockB));
  EXPECT_FALSE (aBlockA.isDisjoint (aBlockC));
}

TEST_F (NvGeBoundBlock3dTest, CopySharesGeometry_IndependentModification)
{
  NvGeBoundBlock3d aBlock (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0),
                           NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  NvGeBoundBlock3d aCopy (aBlock);
  aCopy.extend (NvGePoint3d (5.0, 5.0, 5.0));
  EXPECT_TRUE (aCopy.contains (NvGePoint3d (5.0, 5.0, 5.0)));
  EXPECT_FALSE (aBlock.contains (NvGePoint3d (5.0, 5.0, 5.0))); // COW kept the original
}
