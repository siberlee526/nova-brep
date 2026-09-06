#include <NvException.h>
#include <gelent2d.h>
#include <geline2d.h>
#include <gelnsg2d.h>
#include <gemat2d.h>
#include <gepnt2d.h>
#include <geray2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact planar values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeLine2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeLine2dTest, DefaultConstructor_IsXAxisLine)
{
  const NvGeLine2d aLine;
  ASSERT_EQ (aLine.type(), NvGe::kLine2d);

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 0.0, THE_TEST_TOL);

  const NvGeVector2d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, KAxisConstants_ArePrincipalAxisLines)
{
  const NvGePoint2d anXOrigin = NvGeLine2d::kXAxis.pointOnLine();
  EXPECT_NEAR (anXOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anXOrigin.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeLine2d::kXAxis.direction().x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeLine2d::kXAxis.direction().y, 0.0, THE_TEST_TOL);

  const NvGePoint2d anYOrigin = NvGeLine2d::kYAxis.pointOnLine();
  EXPECT_NEAR (anYOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anYOrigin.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeLine2d::kYAxis.direction().x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeLine2d::kYAxis.direction().y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, PointVectorConstructor_BuildsNormalizedLine)
{
  const NvGeLine2d aLine (NvGePoint2d (2.0, 3.0), NvGeVector2d (0.0, 2.0));

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 3.0, THE_TEST_TOL);

  const NvGeVector2d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, PointVectorConstructor_ZeroVector_Throws)
{
  EXPECT_THROW (NvGeLine2d (NvGePoint2d (1.0, 1.0), NvGeVector2d (0.0, 0.0)), NvException);
}

TEST_F (NvGeLine2dTest, TwoPointConstructor_BuildsLineThroughPoints)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 1.0), NvGePoint2d (3.0, 3.0));

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 1.0, THE_TEST_TOL);

  const double anInvSqrt2 = std::sqrt (0.5);
  EXPECT_NEAR (aLine.direction().x, anInvSqrt2, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().y, anInvSqrt2, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, TwoPointConstructor_CoincidentPoints_Throws)
{
  EXPECT_THROW (NvGeLine2d (NvGePoint2d (2.0, 2.0), NvGePoint2d (2.0, 2.0)), NvException);
}

TEST_F (NvGeLine2dTest, Set_PointVector_ReplacesLineGeometry)
{
  NvGeLine2d aLine (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  aLine.set (NvGePoint2d (-1.0, 4.0), NvGeVector2d (2.0, 0.0));

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y,  4.0, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, Set_TwoPoints_ReplacesLineGeometry)
{
  NvGeLine2d aLine;
  aLine.set (NvGePoint2d (0.0, 0.0), NvGePoint2d (0.0, 5.0));

  EXPECT_NEAR (aLine.direction().x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().y, 1.0, THE_TEST_TOL);

  // The second definition point lies on the line.
  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, Set_DegenerateVector_Throws)
{
  NvGeLine2d aLine;
  EXPECT_THROW (aLine.set (NvGePoint2d (1.0, 1.0), NvGeVector2d (0.0, 0.0)), NvException);
}

TEST_F (NvGeLine2dTest, PointOnLine_AndDirection_AvailableThroughBase)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (3.0, 0.0));
  const NvGeLinearEnt2d& anEnt = aLine;

  const NvGePoint2d anOrigin = anEnt.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnt.direction().x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnt.direction().y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, IntersectWith_CrossingLines_ReturnsIntersectionPoint)
{
  const NvGeLine2d anXAxis;
  const NvGeLine2d aDiag (NvGePoint2d (0.0, 1.0), NvGeVector2d (1.0, -1.0));

  NvGePoint2d anIntPnt;
  EXPECT_TRUE (anXAxis.intersectWith (aDiag, anIntPnt));
  EXPECT_NEAR (anIntPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, IntersectWith_ParallelLines_ReturnsFalse)
{
  const NvGeLine2d anXAxis;
  const NvGeLine2d aShifted (NvGePoint2d (0.0, 1.0), NvGeVector2d (1.0, 0.0));

  NvGePoint2d anIntPnt;
  EXPECT_FALSE (anXAxis.intersectWith (aShifted, anIntPnt));
}

TEST_F (NvGeLine2dTest, IntersectWith_RayIntersectionBehindOrigin_ReturnsFalse)
{
  const NvGeRay2d aRay (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (-1.0, 0.0), NvGeVector2d (0.0, 1.0));

  NvGePoint2d anIntPnt;
  EXPECT_FALSE (aRay.intersectWith (aVertical, anIntPnt));
}

TEST_F (NvGeLine2dTest, IntersectWith_RayIntersectionAhead_ReturnsTrue)
{
  const NvGeRay2d aRay (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (2.0, 0.0), NvGeVector2d (0.0, 1.0));

  NvGePoint2d anIntPnt;
  EXPECT_TRUE (aRay.intersectWith (aVertical, anIntPnt));
  EXPECT_NEAR (anIntPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, IntersectWith_SegmentCrossingInside_ReturnsTrue)
{
  const NvGeLineSeg2d aSeg (NvGePoint2d (2.0, -1.0), NvGePoint2d (2.0, 1.0));
  const NvGeLine2d anXAxis;

  NvGePoint2d anIntPnt;
  EXPECT_TRUE (aSeg.intersectWith (anXAxis, anIntPnt));
  EXPECT_NEAR (anIntPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, IntersectWith_SegmentEndpointHit_ReturnsTrue)
{
  const NvGeLineSeg2d aSeg (NvGePoint2d (0.0, 0.0), NvGePoint2d (4.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (4.0, 1.0), NvGeVector2d (0.0, 1.0));

  NvGePoint2d anIntPnt;
  EXPECT_TRUE (aSeg.intersectWith (aVertical, anIntPnt));
  EXPECT_NEAR (anIntPnt.x, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (anIntPnt.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, IntersectWith_SegmentMissBeyondEndpoint_ReturnsFalse)
{
  const NvGeLineSeg2d aSeg (NvGePoint2d (0.0, 0.0), NvGePoint2d (4.0, 0.0));
  const NvGeLine2d aVertical (NvGePoint2d (4.001, 1.0), NvGeVector2d (0.0, 1.0));

  NvGePoint2d anIntPnt;
  EXPECT_FALSE (aSeg.intersectWith (aVertical, anIntPnt));
}

TEST_F (NvGeLine2dTest, IntersectWith_RayAndSegmentOnNegativeSide_ReturnsFalse)
{
  const NvGeRay2d aRay (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0));
  const NvGeLineSeg2d aSeg (NvGePoint2d (-2.0, -1.0), NvGePoint2d (-2.0, 1.0));

  NvGePoint2d anIntPnt;
  EXPECT_FALSE (aRay.intersectWith (aSeg, anIntPnt));
}

TEST_F (NvGeLine2dTest, IsColinearTo_SameLine_ReturnsTrue)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aSame (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  EXPECT_TRUE (aLine.isColinearTo (aSame));
}

TEST_F (NvGeLine2dTest, IsColinearTo_OffsetParallelLine_ReturnsFalse)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aShifted (NvGePoint2d (5.0, 3.0), NvGeVector2d (1.0, 0.0));
  EXPECT_FALSE (aLine.isColinearTo (aShifted));
}

TEST_F (NvGeLine2dTest, IsColinearTo_OppositeDirection_ReturnsTrue)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aReversed (NvGePoint2d (5.0, 2.0), NvGeVector2d (-1.0, 0.0));
  EXPECT_TRUE (aLine.isColinearTo (aReversed));
}

TEST_F (NvGeLine2dTest, IsParallelTo_And_IsPerpendicularTo_ClassifyDirections)
{
  const NvGeLine2d anXAxis;
  const NvGeLine2d aParallel (NvGePoint2d (7.0, 7.0), NvGeVector2d (-2.0, 0.0));
  const NvGeLine2d aPerp (NvGePoint2d (0.0, 0.0), NvGeVector2d (0.0, 3.0));

  EXPECT_TRUE  (anXAxis.isParallelTo (aParallel));
  EXPECT_FALSE (anXAxis.isPerpendicularTo (aParallel));
  EXPECT_TRUE  (anXAxis.isPerpendicularTo (aPerp));
  EXPECT_FALSE (anXAxis.isParallelTo (aPerp));
}

TEST_F (NvGeLine2dTest, GetPerpLine_PassesThroughPointPerpendicularly)
{
  const NvGeLine2d anXAxis;
  NvGeLine2d aPerp;
  anXAxis.getPerpLine (NvGePoint2d (3.0, 4.0), aPerp);

  const NvGePoint2d anOrigin = aPerp.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aPerp.direction().x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPerp.direction().y, 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aPerp.isPerpendicularTo (anXAxis));
}

TEST_F (NvGeLine2dTest, GetLine_ReturnsEquivalentLine)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 1.0));
  NvGeLine2d anOut;
  aLine.getLine (anOut);

  EXPECT_TRUE (aLine.isEqualTo (anOut));
}

TEST_F (NvGeLine2dTest, IsEqualTo_ViaBase_ComparesGeometry)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  const NvGeLine2d aSame (NvGePoint2d (1.0, 2.0), NvGeVector2d (2.0, 0.0));
  const NvGeLine2d aOther (NvGePoint2d (1.0, 3.0), NvGeVector2d (1.0, 0.0));

  EXPECT_TRUE  (aLine.isEqualTo (aSame));
  EXPECT_FALSE (aLine.isEqualTo (aOther));

  // The entity kind participates in the comparison: a ray with the same
  // supporting line is not equal to the line.
  const NvGeRay2d aRay (NvGePoint2d (1.0, 2.0), NvGeVector2d (1.0, 0.0));
  EXPECT_FALSE (aLine.isEqualTo (aRay));
}

TEST_F (NvGeLine2dTest, TransformBy_TranslateAndRotate_MapsLine)
{
  NvGeLine2d aLine (NvGePoint2d (1.0, 0.0), NvGeVector2d (1.0, 0.0));
  aLine.translateBy (NvGeVector2d (1.0, 1.0));
  aLine.rotateBy (0.5 * THE_PI);

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y,  2.0, THE_TEST_TOL);

  const NvGeVector2d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeLine2dTest, OperatorAssign_CopyOnWrite_KeepsOriginal)
{
  NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (0.0, 1.0));
  NvGeLine2d aCopy;
  aCopy = aLine;
  EXPECT_TRUE (aLine.isEqualTo (aCopy));

  // Modifying the copy must not touch the original geometry (the impl is
  // detached before the new line is installed).
  aCopy.set (NvGePoint2d (9.0, 9.0), NvGeVector2d (1.0, 0.0));

  const NvGePoint2d anOrigin = aLine.pointOnLine();
  EXPECT_NEAR (anOrigin.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aLine.direction().y, 1.0, THE_TEST_TOL);
  EXPECT_FALSE (aLine.isEqualTo (aCopy));
}

TEST_F (NvGeLine2dTest, CopyConstructor_ProducesIndependentEqualLine)
{
  const NvGeLine2d aLine (NvGePoint2d (1.0, 2.0), NvGeVector2d (0.0, 1.0));
  const NvGeLine2d aCopy (aLine);
  EXPECT_TRUE (aLine.isEqualTo (aCopy));
  EXPECT_EQ (aCopy.type(), NvGe::kLine2d);
}
