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

  // Applies the homogeneous matrix to a point in the column-vector
  // convention. Used instead of the (still unimplemented) mat * pnt
  // operator so that the test stays independent of other ge units.
  NvGePoint2d ApplyToPoint (const NvGeMatrix2d& theMat, const NvGePoint2d& thePnt)
  {
    return NvGePoint2d (theMat (0, 0) * thePnt.x + theMat (0, 1) * thePnt.y + theMat (0, 2),
                        theMat (1, 0) * thePnt.x + theMat (1, 1) * thePnt.y + theMat (1, 2));
  }
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeMatrix2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeMatrix2dTest, DefaultConstructor_IsIdentity)
{
  const NvGeMatrix2d aMat;
  EXPECT_NEAR (aMat (0, 0), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (0, 1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (0, 2), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 1), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 2), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 0), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 2), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, CopyConstructor_CopiesAllEntries)
{
  NvGeMatrix2d aSrc = NvGeMatrix2d::rotation (THE_PI / 3.0, NvGePoint2d (1.0, 2.0));
  aSrc.entry[2][0] = 7.0; // externally mangled entry must be copied as well

  const NvGeMatrix2d aCopy (aSrc);
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      EXPECT_NEAR (aCopy (aRow, aCol), aSrc (aRow, aCol), THE_TEST_TOL);
    }
  }

  aSrc.setToIdentity();
  EXPECT_NEAR (aCopy (2, 0), 7.0, THE_TEST_TOL); // deep copy: unaffected by the source
}

TEST_F (NvGeMatrix2dTest, KIdentity_EqualsIdentityMatrix)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::kIdentity;
  EXPECT_TRUE (aMat.isEqualTo (NvGeMatrix2d()));
}

TEST_F (NvGeMatrix2dTest, SetToIdentity_ResetsModifiedEntries)
{
  NvGeMatrix2d aMat = NvGeMatrix2d::translation (NvGeVector2d (5.0, -2.0));
  aMat.setToIdentity();
  EXPECT_TRUE (aMat.isEqualTo (NvGeMatrix2d::kIdentity));
}

TEST_F (NvGeMatrix2dTest, OperatorMultiply_AppliesRightMatrixFirst)
{
  const NvGeMatrix2d aRot    = NvGeMatrix2d::rotation (THE_PI / 2.0);
  const NvGeMatrix2d aTrans  = NvGeMatrix2d::translation (NvGeVector2d (2.0, 0.0));

  // (A * B) applied to the origin: B moves it to (2, 0), A rotates it to (0, 2).
  const NvGePoint2d aPntAB = ApplyToPoint (aRot * aTrans, NvGePoint2d::kOrigin);
  EXPECT_NEAR (aPntAB.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPntAB.y, 2.0, THE_TEST_TOL);

  // (B * A): the rotation leaves the origin fixed, then it is translated.
  const NvGePoint2d aPntBA = ApplyToPoint (aTrans * aRot, NvGePoint2d::kOrigin);
  EXPECT_NEAR (aPntBA.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPntBA.y, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, OperatorMultiplyAssign_MatchesExplicitProduct)
{
  NvGeMatrix2d aMat = NvGeMatrix2d::rotation (THE_PI / 6.0, NvGePoint2d (1.0, 1.0));
  const NvGeMatrix2d aRef = aMat * NvGeMatrix2d::scaling (2.0, NvGePoint2d (3.0, 3.0));

  aMat *= NvGeMatrix2d::scaling (2.0, NvGePoint2d (3.0, 3.0));
  EXPECT_TRUE (aMat.isEqualTo (aRef));
}

TEST_F (NvGeMatrix2dTest, PreAndPostMultBy_ComposeInCorrectOrder)
{
  const NvGeMatrix2d aRot = NvGeMatrix2d::rotation (THE_PI / 2.0);

  // preMultBy: this = rot * this, so the translation column gets rotated.
  NvGeMatrix2d aPre = NvGeMatrix2d::translation (NvGeVector2d (1.0, 1.0));
  aPre.preMultBy (aRot);
  EXPECT_NEAR (aPre.translation().x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPre.translation().y,  1.0, THE_TEST_TOL);

  // postMultBy: this = this * rot, the translation column is unchanged.
  NvGeMatrix2d aPost = NvGeMatrix2d::translation (NvGeVector2d (1.0, 1.0));
  aPost.postMultBy (aRot);
  EXPECT_NEAR (aPost.translation().x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPost.translation().y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetToProduct_MatchesChainedProduct)
{
  const NvGeMatrix2d aRot   = NvGeMatrix2d::rotation (THE_PI / 4.0, NvGePoint2d (-1.0, 2.0));
  const NvGeMatrix2d aScale = NvGeMatrix2d::scaling (3.0, NvGePoint2d (0.5, 0.5));
  const NvGeMatrix2d aRef   = aRot * aScale;

  NvGeMatrix2d aMat;
  aMat.setToProduct (aRot, aScale);
  EXPECT_TRUE (aMat.isEqualTo (aRef));
}

TEST_F (NvGeMatrix2dTest, Inverse_RoundTripsToIdentity)
{
  NvGeMatrix2d anOriginal = NvGeMatrix2d::rotation (THE_PI / 6.0, NvGePoint2d (1.0, 2.0));
  anOriginal *= NvGeMatrix2d::scaling (2.5, NvGePoint2d (3.0, -1.0));
  anOriginal *= NvGeMatrix2d::translation (NvGeVector2d (4.0, 5.0));

  EXPECT_TRUE ((anOriginal.inverse() * anOriginal).isEqualTo (NvGeMatrix2d::kIdentity));
  EXPECT_TRUE ((anOriginal * anOriginal.inverse()).isEqualTo (NvGeMatrix2d::kIdentity));

  NvGeMatrix2d aMat (anOriginal);
  aMat.invert();
  EXPECT_TRUE ((aMat * anOriginal).isEqualTo (NvGeMatrix2d::kIdentity));
}

TEST_F (NvGeMatrix2dTest, Invert_SingularMatrix_Throws)
{
  NvGeMatrix2d aMat;
  aMat.entry[0][0] = 0.0; aMat.entry[0][1] = 0.0;
  aMat.entry[1][0] = 0.0; aMat.entry[1][1] = 0.0;
  EXPECT_THROW (aMat.invert(), NvException);
  EXPECT_THROW (aMat.inverse(), NvException);
}

TEST_F (NvGeMatrix2dTest, IsSingular_DetectsDegenerateLinearPart)
{
  NvGeTol aTol;
  aTol.setEqualPoint (1e-8);

  EXPECT_FALSE (NvGeMatrix2d::kIdentity.isSingular (aTol));
  EXPECT_FALSE (NvGeMatrix2d::rotation (1.2345).isSingular (aTol));

  NvGeMatrix2d aMat;
  aMat.entry[0][0] = 2.0; aMat.entry[0][1] = 4.0;
  aMat.entry[1][0] = 1.0; aMat.entry[1][1] = 2.0; // second column is a multiple of the first
  EXPECT_TRUE (aMat.isSingular (aTol));
}

TEST_F (NvGeMatrix2dTest, Transpose_SwapsEntriesOfFullMatrix)
{
  // Rotation by 90 degrees about (1, 0): | 0 -1  1 |
  //                                     | 1  0 -1 |
  const NvGeMatrix2d aMat = NvGeMatrix2d::rotation (THE_PI / 2.0, NvGePoint2d (1.0, 0.0));
  NvGeMatrix2d aT = aMat.transpose();

  EXPECT_NEAR (aT (0, 0),  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (0, 1),  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (0, 2),  0.0, THE_TEST_TOL); // translation column moves to the bottom row
  EXPECT_NEAR (aT (1, 0), -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (1, 1),  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (1, 2),  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (2, 0),  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (2, 1), -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aT (2, 2),  1.0, THE_TEST_TOL);

  // Transposing twice restores the original matrix.
  aT.transposeIt();
  EXPECT_TRUE (aT.isEqualTo (aMat));
}

TEST_F (NvGeMatrix2dTest, IsEqualTo_UsesEntryWiseInfinityNorm)
{
  NvGeTol aLoose;
  aLoose.setEqualPoint (1e-8);
  NvGeTol aStrict;
  aStrict.setEqualPoint (1e-12);

  const NvGeMatrix2d a = NvGeMatrix2d::translation (NvGeVector2d (1.0, 2.0));
  NvGeMatrix2d b = NvGeMatrix2d::translation (NvGeVector2d (1.0, 2.0));
  b.entry[1][1] += 5e-9;

  EXPECT_TRUE  (a.isEqualTo (b, aLoose));
  EXPECT_FALSE (a.isEqualTo (b, aStrict));
  EXPECT_TRUE  ((a == b)); // operator == forwards to isEqualTo with the global tol
  EXPECT_TRUE  ((a != NvGeMatrix2d::kIdentity));
}

TEST_F (NvGeMatrix2dTest, IsUniScaledOrtho_ClassifiesLinearParts)
{
  EXPECT_TRUE (NvGeMatrix2d::rotation (0.7).isUniScaledOrtho());
  EXPECT_TRUE (NvGeMatrix2d::scaling (-2.0).isUniScaledOrtho()); // uniform scale, mirror allowed

  NvGeMatrix2d aNonUniform;
  aNonUniform.entry[0][0] = 2.0; aNonUniform.entry[1][1] = 3.0; // diag(2, 3)
  EXPECT_FALSE (aNonUniform.isUniScaledOrtho());

  NvGeMatrix2d aShear;
  aShear.entry[0][0] = 1.0; aShear.entry[0][1] = 1.0;
  aShear.entry[1][0] = 0.0; aShear.entry[1][1] = 1.0;
  EXPECT_FALSE (aShear.isUniScaledOrtho());

  NvGeMatrix2d aZero; // zero linear part is not an orthogonal basis
  aZero.entry[0][0] = 0.0; aZero.entry[1][1] = 0.0;
  EXPECT_FALSE (aZero.isUniScaledOrtho());
}

TEST_F (NvGeMatrix2dTest, IsScaledOrtho_ClassifiesLinearParts)
{
  EXPECT_TRUE (NvGeMatrix2d::rotation (0.7).isScaledOrtho());

  NvGeMatrix2d aNonUniform;
  aNonUniform.entry[0][0] = 2.0; aNonUniform.entry[1][1] = 3.0; // orthogonal columns, |2| != |3|
  EXPECT_TRUE  (aNonUniform.isScaledOrtho());
  EXPECT_FALSE (aNonUniform.isUniScaledOrtho());

  NvGeMatrix2d aShear;
  aShear.entry[0][0] = 1.0; aShear.entry[0][1] = 1.0;
  aShear.entry[1][0] = 0.0; aShear.entry[1][1] = 1.0;
  EXPECT_FALSE (aShear.isScaledOrtho());
}

TEST_F (NvGeMatrix2dTest, Scale_ReturnsFirstColumnNorm)
{
  NvGeMatrix2d aRot = NvGeMatrix2d::rotation (1.1);
  EXPECT_NEAR (aRot.scale(), 1.0, THE_TEST_TOL);

  NvGeMatrix2d aScale = NvGeMatrix2d::scaling (-3.0);
  EXPECT_NEAR (aScale.scale(), 3.0, THE_TEST_TOL); // norm is always positive
}

TEST_F (NvGeMatrix2dTest, Det_ReturnsLinearPartDeterminant)
{
  EXPECT_NEAR (NvGeMatrix2d::rotation (THE_PI / 2.0).det(),  1.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeMatrix2d::scaling (3.0).det(),            9.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeMatrix2d::scaling (-2.0).det(),           4.0, THE_TEST_TOL);

  NvGeMatrix2d aMirrorX; // reflection about the x-axis
  aMirrorX.entry[0][0] = 1.0; aMirrorX.entry[1][1] = -1.0;
  EXPECT_NEAR (aMirrorX.det(), -1.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetTranslation_ReplacesTranslationOnly)
{
  NvGeMatrix2d aMat = NvGeMatrix2d::rotation (0.9, NvGePoint2d (2.0, 2.0));
  aMat.setTranslation (NvGeVector2d (-4.0, 7.0));

  EXPECT_NEAR (aMat.translation().x, -4.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat.translation().y,  7.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat.det(), 1.0, THE_TEST_TOL); // linear part untouched
}

TEST_F (NvGeMatrix2dTest, Translation_ReturnsTranslationColumn)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::translation (NvGeVector2d (1.5, -2.5));
  EXPECT_NEAR (aMat.translation().x,  1.5, THE_TEST_TOL);
  EXPECT_NEAR (aMat.translation().y, -2.5, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, IsConformal_Rotation_ReturnsScaleAndAngle)
{
  double aScale = 0.0;
  double anAngle = 0.0;
  Nova::Boolean anIsMirror = true;
  NvGeVector2d aReflex;

  NvGeMatrix2d aRot = NvGeMatrix2d::rotation (THE_PI / 3.0);
  EXPECT_TRUE (aRot.isConformal (aScale, anAngle, anIsMirror, aReflex));
  EXPECT_NEAR (aScale, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anAngle, THE_PI / 3.0, THE_TEST_TOL);
  EXPECT_FALSE (anIsMirror);

  // Uniform scale composed with rotation stays conformal.
  NvGeMatrix2d aScaledRot = NvGeMatrix2d::rotation (THE_PI / 3.0);
  aScaledRot *= NvGeMatrix2d::scaling (2.0);
  EXPECT_TRUE (aScaledRot.isConformal (aScale, anAngle, anIsMirror, aReflex));
  EXPECT_NEAR (aScale, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anAngle, THE_PI / 3.0, THE_TEST_TOL);
  EXPECT_FALSE (anIsMirror);
}

TEST_F (NvGeMatrix2dTest, IsConformal_Mirror_ReturnsReflexAxis)
{
  double aScale = 0.0;
  double anAngle = 0.0;
  Nova::Boolean anIsMirror = false;
  NvGeVector2d aReflex;

  // scale * reflection about the x-axis: | 2  0 |
  //                                     | 0 -2 |
  NvGeMatrix2d aMirrorX;
  aMirrorX.entry[0][0] = 2.0; aMirrorX.entry[1][1] = -2.0;
  EXPECT_TRUE (aMirrorX.isConformal (aScale, anAngle, anIsMirror, aReflex));
  EXPECT_NEAR (aScale, 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anIsMirror);
  EXPECT_NEAR (anAngle, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aReflex.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aReflex.y, 0.0, THE_TEST_TOL);

  // Reflection about the line y = x, scaled by 3: | 0 3 |
  //                                             | 3 0 |
  NvGeMatrix2d aMirrorDiag;
  aMirrorDiag.entry[0][0] = 0.0; aMirrorDiag.entry[0][1] = 3.0;
  aMirrorDiag.entry[1][0] = 3.0; aMirrorDiag.entry[1][1] = 0.0;
  EXPECT_TRUE (aMirrorDiag.isConformal (aScale, anAngle, anIsMirror, aReflex));
  EXPECT_NEAR (aScale, 3.0, THE_TEST_TOL);
  EXPECT_TRUE (anIsMirror);
  EXPECT_NEAR (aReflex.x, std::cos (THE_PI / 4.0), THE_TEST_TOL);
  EXPECT_NEAR (aReflex.y, std::sin (THE_PI / 4.0), THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, IsConformal_NonConformal_ReturnsFalse)
{
  double aScale = 0.0;
  double anAngle = 0.0;
  Nova::Boolean anIsMirror = false;
  NvGeVector2d aReflex;

  NvGeMatrix2d aShear; // shear is not angle-preserving
  aShear.entry[0][0] = 1.0; aShear.entry[0][1] = 1.0;
  aShear.entry[1][0] = 0.0; aShear.entry[1][1] = 1.0;
  EXPECT_FALSE (aShear.isConformal (aScale, anAngle, anIsMirror, aReflex));

  NvGeMatrix2d aNonUniform;
  aNonUniform.entry[0][0] = 2.0; aNonUniform.entry[1][1] = 3.0;
  EXPECT_FALSE (aNonUniform.isConformal (aScale, anAngle, anIsMirror, aReflex));
}

TEST_F (NvGeMatrix2dTest, SetAndGetCoordSystem_RoundTripsArbitraryBasis)
{
  const NvGePoint2d  anOrigin (1.0, 2.0);
  const NvGeVector2d anE0 (2.0, 0.0);
  const NvGeVector2d anE1 (1.0, 3.0);

  NvGeMatrix2d aMat;
  aMat.setCoordSystem (anOrigin, anE0, anE1);

  // The matrix maps local coordinates to the world frame.
  const NvGePoint2d aLocal = ApplyToPoint (aMat, NvGePoint2d (1.0, 1.0));
  EXPECT_NEAR (aLocal.x, 4.0, THE_TEST_TOL); // 1 + 1*2 + 1*1
  EXPECT_NEAR (aLocal.y, 5.0, THE_TEST_TOL); // 2 + 1*0 + 1*3

  NvGePoint2d  anOriginOut;
  NvGeVector2d anE0Out;
  NvGeVector2d anE1Out;
  aMat.getCoordSystem (anOriginOut, anE0Out, anE1Out);
  EXPECT_NEAR (anOriginOut.x, anOrigin.x, THE_TEST_TOL);
  EXPECT_NEAR (anOriginOut.y, anOrigin.y, THE_TEST_TOL);
  EXPECT_NEAR (anE0Out.x, anE0.x, THE_TEST_TOL);
  EXPECT_NEAR (anE0Out.y, anE0.y, THE_TEST_TOL);
  EXPECT_NEAR (anE1Out.x, anE1.x, THE_TEST_TOL);
  EXPECT_NEAR (anE1Out.y, anE1.y, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetToRotation_KeepsCenterFixed)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::rotation (THE_PI / 2.0, NvGePoint2d (1.0, 0.0));

  const NvGePoint2d aCenter = ApplyToPoint (aMat, NvGePoint2d (1.0, 0.0));
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);

  const NvGePoint2d aPnt = ApplyToPoint (aMat, NvGePoint2d (2.0, 0.0));
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetToScaling_AboutCenter)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::scaling (2.0, NvGePoint2d (1.0, 0.0));

  const NvGePoint2d aPnt1 = ApplyToPoint (aMat, NvGePoint2d (3.0, 0.0));
  EXPECT_NEAR (aPnt1.x, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt1.y, 0.0, THE_TEST_TOL);

  const NvGePoint2d aPnt2 = ApplyToPoint (aMat, NvGePoint2d (1.0, 5.0));
  EXPECT_NEAR (aPnt2.x, 1.0,  THE_TEST_TOL);
  EXPECT_NEAR (aPnt2.y, 10.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetToMirroring_PointIsCenterOfSymmetry)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::mirroring (NvGePoint2d (1.0, 1.0));

  const NvGePoint2d aPnt = ApplyToPoint (aMat, NvGePoint2d (2.0, 3.0));
  EXPECT_NEAR (aPnt.x,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, -1.0, THE_TEST_TOL);
}

TEST_F (NvGeMatrix2dTest, SetToAlignCoordSys_MapsLocalCoordinatesBetweenFrames)
{
  const NvGeMatrix2d aMat = NvGeMatrix2d::alignCoordSys (
      NvGePoint2d (1.0, 1.0), NvGeVector2d (1.0, 0.0), NvGeVector2d (0.0, 1.0),
      NvGePoint2d (0.0, 0.0), NvGeVector2d (0.0, 1.0), NvGeVector2d (-1.0, 0.0));

  // The point (2, 2) has local coordinates (1, 1) in the "from" frame;
  // re-expressed in the "to" frame this is the world point (-1, 1).
  const NvGePoint2d aPnt = ApplyToPoint (aMat, NvGePoint2d (2.0, 2.0));
  EXPECT_NEAR (aPnt.x, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y,  1.0, THE_TEST_TOL);

  // Aligning a standard frame onto itself is the identity.
  const NvGeVector2d aX (1.0, 0.0);
  const NvGeVector2d aY (0.0, 1.0);
  const NvGeMatrix2d anIdentity = NvGeMatrix2d::alignCoordSys (
      NvGePoint2d::kOrigin, aX, aY, NvGePoint2d::kOrigin, aX, aY);
  EXPECT_TRUE (anIdentity.isEqualTo (NvGeMatrix2d::kIdentity));
}

TEST_F (NvGeMatrix2dTest, SetToAlignCoordSys_DegenerateSourceFrame_Throws)
{
  NvGeMatrix2d aMat;
  EXPECT_THROW (aMat.setToAlignCoordSys (
      NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0), NvGeVector2d (2.0, 0.0),
      NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0), NvGeVector2d (0.0, 1.0)),
      NvException);
}

TEST_F (NvGeMatrix2dTest, StaticFactories_MatchSetToVariants)
{
  const NvGeVector2d aVec (1.5, -0.5);
  NvGeMatrix2d aRef;

  EXPECT_TRUE (NvGeMatrix2d::translation (aVec).isEqualTo (aRef.setToTranslation (aVec)));
  EXPECT_TRUE (NvGeMatrix2d::rotation (THE_PI / 6.0, NvGePoint2d (2.0, 3.0))
                   .isEqualTo (aRef.setToRotation (THE_PI / 6.0, NvGePoint2d (2.0, 3.0))));
  EXPECT_TRUE (NvGeMatrix2d::scaling (3.0, NvGePoint2d (1.0, 1.0))
                   .isEqualTo (aRef.setToScaling (3.0, NvGePoint2d (1.0, 1.0))));
  EXPECT_TRUE (NvGeMatrix2d::mirroring (NvGePoint2d (2.0, -1.0))
                   .isEqualTo (aRef.setToMirroring (NvGePoint2d (2.0, -1.0))));
}
