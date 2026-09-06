#include <NvException.h>
#include <geline3d.h>
#include <gemat3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeLine3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeLine3dTest, DefaultConstructor_IsXAxisThroughOrigin)
{
  const NvGeLine3d aLine;

  EXPECT_TRUE (aLine.isKindOf (NvGe::kLine3d));

  const NvGePoint3d aPnt = aLine.pointOnLine();
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine3dTest, StaticAxisLines_AlongCoordinateAxes)
{
  const NvGeVector3d aDirX = NvGeLine3d::kXAxis.direction();
  EXPECT_NEAR (aDirX.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirX.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirX.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d aDirY = NvGeLine3d::kYAxis.direction();
  EXPECT_NEAR (aDirY.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirY.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirY.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d aDirZ = NvGeLine3d::kZAxis.direction();
  EXPECT_NEAR (aDirZ.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirZ.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDirZ.z, 1.0, THE_TEST_TOL);

  EXPECT_TRUE (NvGeLine3d::kYAxis.pointOnLine().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (NvGeLine3d::kZAxis.pointOnLine().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
}

TEST_F (NvGeLine3dTest, SetPointAndVector_RedefinesCarrierLine)
{
  NvGeLine3d aLine;
  aLine.set (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (0.0, 0.0, 5.0));

  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (1.0, 2.0, 3.0)));
  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeLine3dTest, SetTwoPoints_DirectionFromFirstToSecond)
{
  NvGeLine3d aLine;
  aLine.set (NvGePoint3d (0.0, 0.0, 0.0), NvGePoint3d (3.0, 4.0, 0.0));

  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.6, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 0.8, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine3dTest, SetDegenerateInput_Throws)
{
  NvGeLine3d aLine;
  EXPECT_THROW (aLine.set (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 0.0)),
                NvException);
  EXPECT_THROW (aLine.set (NvGePoint3d (1.0, 1.0, 1.0), NvGePoint3d (1.0, 1.0, 1.0)),
                NvException);
}

TEST_F (NvGeLine3dTest, ConstructorDegenerateInput_Throws)
{
  EXPECT_THROW (NvGeLine3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 0.0)),
                NvException);
  EXPECT_THROW (NvGeLine3d (NvGePoint3d (2.0, 2.0, 2.0), NvGePoint3d (2.0, 2.0, 2.0)),
                NvException);
}

TEST_F (NvGeLine3dTest, PointOnLineAndDirection_ReturnNormalizedCarrierGeometry)
{
  const NvGeLine3d aLine (NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 3.0, 0.0));

  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (1.0, 1.0, 1.0)));
  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine3dTest, IntersectWith_CrossingLines_ReturnsIntersectionPoint)
{
  const NvGeLine3d aLine1 (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aLine2 (NvGePoint3d (2.0, 0.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));

  NvGePoint3d anIntPnt;
  EXPECT_TRUE (aLine1.intersectWith (aLine2, anIntPnt));
  EXPECT_TRUE (anIntPnt.isEqualTo (NvGePoint3d (2.0, 0.0, 0.0)));
}

TEST_F (NvGeLine3dTest, IntersectWith_ParallelLines_ReturnsFalse)
{
  const NvGeLine3d aLine1 (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aLine2 (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  NvGePoint3d anIntPnt;
  EXPECT_FALSE (aLine1.intersectWith (aLine2, anIntPnt));
}

TEST_F (NvGeLine3dTest, IntersectWith_SkewLines_ReturnsFalse)
{
  const NvGeLine3d aLine1 (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aLine2 (NvGePoint3d (0.0, 0.0, 1.0), NvGeVector3d (0.0, 1.0, 0.0));

  NvGePoint3d anIntPnt;
  EXPECT_FALSE (aLine1.intersectWith (aLine2, anIntPnt));
}

TEST_F (NvGeLine3dTest, IntersectWith_CoincidentLines_ReturnsFalse)
{
  const NvGeLine3d aLine1 (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGeLine3d aLine2 (NvGePoint3d (5.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  NvGePoint3d anIntPnt;
  EXPECT_FALSE (aLine1.intersectWith (aLine2, anIntPnt));
}

TEST_F (NvGeLine3dTest, IsColinearTo_ClassifiesSharedCarrierLine)
{
  const NvGeLine3d aLine (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  // Same carrier line, opposite orientation.
  const NvGeLine3d anOpposite (NvGePoint3d (2.0, 0.0, 0.0), NvGeVector3d (-1.0, 0.0, 0.0));
  EXPECT_TRUE (aLine.isColinearTo (anOpposite));

  // Parallel but offset.
  const NvGeLine3d anOffset (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_FALSE (aLine.isColinearTo (anOffset));

  // Perpendicular.
  const NvGeLine3d aPerp (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_FALSE (aLine.isColinearTo (aPerp));
}

TEST_F (NvGeLine3dTest, TransformBy_TranslateAndRotate_MovesCarrierLine)
{
  NvGeLine3d aLine; // origin, direction +X

  NvGeMatrix3d aRot;
  aRot.setToRotation (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0), NvGePoint3d (0.0, 0.0, 0.0));
  const NvGeMatrix3d anXfm = aRot * NvGeMatrix3d::translation (NvGeVector3d (1.0, 2.0, 3.0));

  aLine.transformBy (anXfm);

  // (0, 0, 0) -> (1, 2, 3) -> rotated by 90 deg about Z -> (-2, 1, 3).
  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (-2.0, 1.0, 3.0)));
  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine3dTest, CopyOnWrite_CopyThenModifyOriginal_CopyUnaffected)
{
  const NvGeLine3d aLine (NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeLine3d aCopy (aLine);

  NvGeLine3d aMutable (aLine);
  aMutable.set (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));

  // The copies keep the shared original geometry.
  EXPECT_TRUE (aCopy.pointOnLine().isEqualTo (NvGePoint3d (1.0, 1.0, 1.0)));
  EXPECT_TRUE (aCopy.direction().isEqualTo (NvGeVector3d (0.0, 0.0, 1.0)));
  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (1.0, 1.0, 1.0)));

  // The modified line carries the new geometry.
  EXPECT_TRUE (aMutable.pointOnLine().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aMutable.direction().isEqualTo (NvGeVector3d (0.0, 1.0, 0.0)));
}

TEST_F (NvGeLine3dTest, CopyOnWrite_AssignThenModifyCopy_OriginalUnaffected)
{
  const NvGeLine3d aLine (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (1.0, 0.0, 0.0));
  NvGeLine3d aCopy;
  aCopy = aLine;

  aCopy.set (NvGePoint3d (5.0, 5.0, 5.0), NvGeVector3d (0.0, 0.0, 1.0));

  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (1.0, 2.0, 3.0)));
  EXPECT_TRUE (aLine.direction().isEqualTo (NvGeVector3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aCopy.pointOnLine().isEqualTo (NvGePoint3d (5.0, 5.0, 5.0)));
}
