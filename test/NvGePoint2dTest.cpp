#include <NvException.h>
#include <gemat2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact planar values computed through cos/sin.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
// NvGePoint2d::mirror() is not exercised at runtime: NvGeLine2d and its
// pointOnLine()/direction() accessors are still unimplemented stubs, so a
// mirror test could not assert meaningful values until they land.
class NvGePoint2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGePoint2dTest, KOrigin_IsAtOrigin)
{
  EXPECT_NEAR (NvGePoint2d::kOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGePoint2d::kOrigin.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, OperatorMultiply_FullHomogeneous_TranslationIsApplied)
{
  // mat = T(2, 0) * R(90 deg): the point (1, 0) rotates to (0, 1) and then
  // moves to (2, 1); dropping the translation column would yield (0, 1).
  NvGeMatrix2d aMat = NvGeMatrix2d::translation (NvGeVector2d (2.0, 0.0));
  aMat.postMultBy (NvGeMatrix2d::rotation (THE_PI / 2.0));

  const NvGePoint2d aPnt = aMat * NvGePoint2d (1.0, 0.0);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, SetToProduct_MatchesMatrixPointOperator)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::rotation (THE_PI / 3.0, NvGePoint2d (1.0, 2.0));
  const NvGePoint2d aPnt (2.0, -1.0);
  const NvGePoint2d aExpected = aMat * aPnt;

  NvGePoint2d aResult;
  aResult.setToProduct (aMat, aPnt);
  EXPECT_NEAR (aResult.x, aExpected.x, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, aExpected.y, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, TransformBy_AppliesFullMatrix_TranslationIsApplied)
{
  NvGeMatrix2d aMat = NvGeMatrix2d::translation (NvGeVector2d (2.0, 0.0));
  aMat.postMultBy (NvGeMatrix2d::rotation (THE_PI / 2.0));

  NvGePoint2d aPnt (1.0, 0.0);
  aPnt.transformBy (aMat);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, TransformBy_IdentityMatrix_KeepsPointUnchanged)
{
  NvGePoint2d aPnt (1.5, -2.5);
  aPnt.transformBy (NvGeMatrix2d::kIdentity);
  EXPECT_NEAR (aPnt.x,  1.5, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, -2.5, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, RotateBy_AboutNonOriginCenter_KeepsCenterFixed)
{
  // Rotating (2, 0) by 90 degrees about (1, 0) moves it to (1, 1), while
  // the center itself stays in place.
  NvGePoint2d aPnt (2.0, 0.0);
  aPnt.rotateBy (THE_PI / 2.0, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);

  NvGePoint2d aCenter (1.0, 0.0);
  aCenter.rotateBy (THE_PI / 2.0, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, RotateBy_DefaultCenter_RotatesAboutOrigin)
{
  NvGePoint2d aPnt (1.0, 0.0);
  aPnt.rotateBy (THE_PI / 2.0);
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, ScaleBy_AboutNonOriginCenter_KeepsCenterFixed)
{
  NvGePoint2d aPnt (3.0, 0.0);
  aPnt.scaleBy (2.0, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aPnt.x, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);

  NvGePoint2d aOff (1.0, 5.0);
  aOff.scaleBy (2.0, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aOff.x,  1.0,  THE_TEST_TOL);
  EXPECT_NEAR (aOff.y, 10.0, THE_TEST_TOL);

  NvGePoint2d aCenter (1.0, 0.0);
  aCenter.scaleBy (2.0, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, SetToSum_ComputesPointPlusVector)
{
  NvGePoint2d aResult;
  aResult.setToSum (NvGePoint2d (1.0, 2.0), NvGeVector2d (3.0, -4.0));
  EXPECT_NEAR (aResult.x,  4.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, -2.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, DistanceTo_ReturnsEuclideanDistance)
{
  EXPECT_NEAR (NvGePoint2d (0.0, 0.0).distanceTo (NvGePoint2d (3.0, 4.0)), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGePoint2d (1.0, 2.0).distanceTo (NvGePoint2d (1.0, 2.0)), 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint2dTest, IsEqualTo_RespectsEqualPointTolerance)
{
  NvGeTol aStrict;
  aStrict.setEqualPoint (1e-12);

  // The default (pinned) tolerance of 1e-8 accepts a ~4.2e-9 distance.
  const NvGePoint2d aPnt (1.0, 2.0);
  const NvGePoint2d aNear (1.0 + 3e-9, 2.0 + 3e-9);
  EXPECT_TRUE  (aPnt.isEqualTo (aNear));
  EXPECT_FALSE (aPnt.isEqualTo (aNear, aStrict));
  EXPECT_FALSE (aPnt.isEqualTo (NvGePoint2d (2.0, 2.0)));

  // operator == / != forward to isEqualTo with the global tolerance.
  EXPECT_TRUE  ((aPnt == aNear));
  EXPECT_FALSE ((aPnt != aNear));
  EXPECT_TRUE  ((aPnt != NvGePoint2d (2.0, 2.0)));
}

TEST_F (NvGePoint2dTest, AsVectorAndDifference_ReturnTranslationsBetweenPoints)
{
  const NvGePoint2d aPnt0 (2.0, 3.0);
  const NvGePoint2d aPnt1 (5.0, 7.0);

  const NvGeVector2d aDiff = aPnt1 - aPnt0;
  EXPECT_NEAR (aDiff.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aDiff.y, 4.0, THE_TEST_TOL);

  const NvGeVector2d aVec = aPnt1.asVector();
  EXPECT_NEAR (aVec.x, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 7.0, THE_TEST_TOL);
}
