#include <NvException.h>
#include <gegblabb.h>
#include <gemat2d.h>
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
class NvGeVector2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeVector2dTest, StaticConstants_HoldAxisDirections)
{
  EXPECT_NEAR (NvGeVector2d::kIdentity.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d::kIdentity.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d::kXAxis.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d::kXAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d::kYAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d::kYAxis.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, RotateBy_RotatesCounterclockwise)
{
  NvGeVector2d aXAxis (1.0, 0.0);
  aXAxis.rotateBy (THE_PI / 2.0);
  EXPECT_NEAR (aXAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.y, 1.0, THE_TEST_TOL);

  NvGeVector2d aDiag (1.0, 1.0);
  aDiag.rotateBy (THE_PI / 2.0);
  EXPECT_NEAR (aDiag.x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDiag.y,  1.0, THE_TEST_TOL);

  // Negative angles rotate clockwise.
  NvGeVector2d aClock (1.0, 0.0);
  aClock.rotateBy (-THE_PI / 2.0);
  EXPECT_NEAR (aClock.x,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aClock.y, -1.0, THE_TEST_TOL);

  // Length is preserved.
  NvGeVector2d aVec (3.0, -4.0);
  aVec.rotateBy (1.234);
  EXPECT_NEAR (aVec.length(), 5.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Mirror_ReflectsAboutLineDirection)
{
  NvGeVector2d aVec (1.0, 1.0);
  aVec.mirror (NvGeVector2d (1.0, 0.0));
  EXPECT_NEAR (aVec.x,  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, -1.0, THE_TEST_TOL);

  // Reflection about the diagonal swaps the components.
  NvGeVector2d aSwapped (2.0, 0.0);
  aSwapped.mirror (NvGeVector2d (1.0, 1.0));
  EXPECT_NEAR (aSwapped.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aSwapped.y, 2.0, THE_TEST_TOL);

  // Mirroring about the vector itself is the identity; the line length
  // does not matter.
  NvGeVector2d aSame (3.0, 4.0);
  aSame.mirror (NvGeVector2d (6.0, 8.0));
  EXPECT_NEAR (aSame.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aSame.y, 4.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Mirror_ZeroLengthLine_Throws)
{
  NvGeVector2d aVec (1.0, 1.0);
  EXPECT_THROW (aVec.mirror (NvGeVector2d (0.0, 0.0)), NvException);
}

TEST_F (NvGeVector2dTest, Angle_ReturnsAtan2Argument)
{
  EXPECT_NEAR (NvGeVector2d (1.0, 1.0).angle(),  THE_PI / 4.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d (-1.0, 0.0).angle(), THE_PI,       THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d (0.0, -1.0).angle(), -THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector2d (0.0, 0.0).angle(),  0.0,          THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, AngleTo_MeasuresCCWAngleIn0To2Pi)
{
  const NvGeVector2d aXAxis (1.0, 0.0);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector2d (0.0, 1.0)),  THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector2d (-1.0, 0.0)), THE_PI,       THE_TEST_TOL);

  // Counterclockwise convention: from +y to +x is three quarter turns.
  const NvGeVector2d aYAxis (0.0, 1.0);
  EXPECT_NEAR (aYAxis.angleTo (NvGeVector2d (1.0, 0.0)), 3.0 * THE_PI / 2.0, THE_TEST_TOL);

  EXPECT_NEAR (aXAxis.angleTo (NvGeVector2d (2.0, 0.0)), 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Normal_ReturnsUnitVectorWithoutMutating)
{
  const NvGeVector2d aVec (3.0, 4.0);
  const NvGeVector2d aUnit = aVec.normal();
  EXPECT_NEAR (aUnit.x, 0.6, THE_TEST_TOL);
  EXPECT_NEAR (aUnit.y, 0.8, THE_TEST_TOL);
  EXPECT_NEAR (aVec.x, 3.0, THE_TEST_TOL); // original is untouched
}

TEST_F (NvGeVector2dTest, Normal_ZeroVector_ReturnsUnchangedCopy)
{
  const NvGeVector2d aZero (0.0, 0.0);
  const NvGeVector2d aResult = aZero.normal();
  EXPECT_NEAR (aResult.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Normalize_ScalesToUnitLength)
{
  NvGeVector2d aVec (3.0, 4.0);
  aVec.normalize();
  EXPECT_NEAR (aVec.x, 0.6, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 0.8, THE_TEST_TOL);
  EXPECT_NEAR (aVec.length(), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Normalize_ZeroVector_LeavesUnchanged)
{
  NvGeVector2d aVec (0.0, 0.0);
  aVec.normalize();
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, NormalizeWithFlag_ReportsK0ThisForZeroVector)
{
  NvGeError aFlag = kOk;

  NvGeVector2d aVec (0.0, 0.0);
  aVec.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0This);
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 0.0, THE_TEST_TOL);

  // A vector within the vector tolerance counts as zero as well.
  NvGeVector2d aTiny (1e-9, 0.0);
  aTiny.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0This);

  NvGeVector2d aGood (3.0, 4.0);
  aGood.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, kOk);
  EXPECT_NEAR (aGood.length(), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, Length_ReturnsEuclideanLength)
{
  const NvGeVector2d aVec (3.0, 4.0);
  EXPECT_NEAR (aVec.length(), 5.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, IsUnitLengthAndIsZeroLength_ClassifyWithinTolerance)
{
  EXPECT_TRUE  (NvGeVector2d (1.0, 0.0).isUnitLength());
  EXPECT_TRUE  (NvGeVector2d (0.999999999, 0.0).isUnitLength()); // 1e-9 deviation
  EXPECT_FALSE (NvGeVector2d (2.0, 0.0).isUnitLength());

  EXPECT_TRUE  (NvGeVector2d (0.0, 0.0).isZeroLength());
  EXPECT_TRUE  (NvGeVector2d (1e-9, 0.0).isZeroLength());
  EXPECT_FALSE (NvGeVector2d (1e-7, 0.0).isZeroLength());
}

TEST_F (NvGeVector2dTest, IsParallelTo_UsesCrossProductMagnitude)
{
  const NvGeVector2d aXAxis (1.0, 0.0);
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector2d (2.0, 0.0)));
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector2d (-3.0, 0.0))); // antiparallel counts
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector2d (5.0, 1e-9))); // within tolerance
  EXPECT_FALSE (aXAxis.isParallelTo (NvGeVector2d (0.0, 2.0)));
}

TEST_F (NvGeVector2dTest, IsParallelToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector2d (0.0, 0.0).isParallelTo (NvGeVector2d (1.0, 0.0),
                                                      NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector2d (1.0, 0.0).isParallelTo (NvGeVector2d (0.0, 0.0),
                                                      NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector2d (1.0, 0.0).isParallelTo (NvGeVector2d (2.0, 0.0),
                                                     NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector2dTest, IsCodirectionalTo_RequiresParallelAndSameDirection)
{
  const NvGeVector2d aXAxis (1.0, 0.0);
  EXPECT_TRUE  (aXAxis.isCodirectionalTo (NvGeVector2d (2.0, 0.0)));
  EXPECT_FALSE (aXAxis.isCodirectionalTo (NvGeVector2d (-2.0, 0.0)));
  EXPECT_FALSE (aXAxis.isCodirectionalTo (NvGeVector2d (0.0, 1.0)));
}

TEST_F (NvGeVector2dTest, IsCodirectionalToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector2d (0.0, 0.0).isCodirectionalTo (NvGeVector2d (1.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector2d (1.0, 0.0).isCodirectionalTo (NvGeVector2d (0.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector2d (1.0, 0.0).isCodirectionalTo (NvGeVector2d (3.0, 0.0),
                                                          NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector2dTest, IsPerpendicularTo_UsesDotProductMagnitude)
{
  const NvGeVector2d aXAxis (1.0, 0.0);
  EXPECT_TRUE  (aXAxis.isPerpendicularTo (NvGeVector2d (0.0, 3.0)));
  EXPECT_TRUE  (aXAxis.isPerpendicularTo (NvGeVector2d (1e-9, 2.0))); // dot within tolerance
  EXPECT_FALSE (aXAxis.isPerpendicularTo (NvGeVector2d (1.0, 1.0)));
}

TEST_F (NvGeVector2dTest, IsPerpendicularToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector2d (0.0, 0.0).isPerpendicularTo (NvGeVector2d (1.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector2d (1.0, 0.0).isPerpendicularTo (NvGeVector2d (0.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector2d (1.0, 0.0).isPerpendicularTo (NvGeVector2d (0.0, 2.0),
                                                          NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector2dTest, TransformBy_AppliesLinearPartOnly)
{
  // Composition of a rotation with a translation: the translation column
  // must not affect the vector.
  const NvGeMatrix2d aMat = NvGeMatrix2d::rotation (THE_PI / 2.0)
                          * NvGeMatrix2d::translation (NvGeVector2d (3.0, 4.0));

  NvGeVector2d aVec (2.0, 0.0);
  aVec.transformBy (aMat);
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 2.0, THE_TEST_TOL);

  // Scaling acts on the components.
  NvGeVector2d aScaled (3.0, 4.0);
  aScaled.transformBy (NvGeMatrix2d::scaling (2.0));
  EXPECT_NEAR (aScaled.x, 6.0, THE_TEST_TOL);
  EXPECT_NEAR (aScaled.y, 8.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, OperatorMultiplyMatrixVector_MatchesTransformBy)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::rotation (0.7)
                          * NvGeMatrix2d::translation (NvGeVector2d (1.0, 2.0));
  const NvGeVector2d aVec (2.0, -3.0);

  NvGeVector2d aTransformed = aVec;
  aTransformed.transformBy (aMat);

  const NvGeVector2d aProduct = aMat * aVec;
  EXPECT_NEAR (aProduct.x, aTransformed.x, THE_TEST_TOL);
  EXPECT_NEAR (aProduct.y, aTransformed.y, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, OperatorMultiplyScalar_FreeFunctionMatchesMember)
{
  const NvGeVector2d aVec (2.0, -3.0);
  const NvGeVector2d aFree = 2.0 * aVec;
  EXPECT_NEAR (aFree.x,  4.0, THE_TEST_TOL);
  EXPECT_NEAR (aFree.y, -6.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, MatrixConversion_ProducesTranslationMatrix)
{
  const NvGeMatrix2d aMat = NvGeVector2d (3.0, 4.0);
  EXPECT_NEAR (aMat (0, 0), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (0, 1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (0, 2), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 1), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 2), 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 2), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector2dTest, IsEqualTo_UsesEuclideanDistanceWithinTolerance)
{
  NvGeTol aStrict;
  aStrict.setEqualPoint (1e-8);
  aStrict.setEqualVector (1e-12);

  const NvGeVector2d aVec (1.0, 2.0);
  const NvGeVector2d aClose (1.0 + 5e-9, 2.0);

  EXPECT_TRUE  (aVec.isEqualTo (aClose));      // distance 5e-9 <= 1e-8
  EXPECT_FALSE (aVec.isEqualTo (aClose, aStrict));
  EXPECT_TRUE  (aVec == NvGeVector2d (1.0, 2.0));
  EXPECT_TRUE  (aVec != NvGeVector2d (1.0, 2.1));
}
