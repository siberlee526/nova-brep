#include <NvException.h>
#include <gemat3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

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
// The planar operations of NvGePoint3d (mirror, project, orthoProject,
// convert2d, plane constructor/set) are not exercised at runtime: NvGePlane,
// NvGePlanarEnt and NvGeVector3d::crossProduct are still unimplemented
// stubs, so those tests could not assert meaningful values until they land.
class NvGePoint3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGePoint3dTest, KOrigin_IsAtOrigin)
{
  EXPECT_NEAR (NvGePoint3d::kOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGePoint3d::kOrigin.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGePoint3d::kOrigin.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, OperatorMultiply_FullHomogeneous_TranslationIsApplied)
{
  // mat = T(2, 0, 0) * R(90 deg, z-axis): the point (1, 0, 0) rotates to
  // (0, 1, 0) and then moves to (2, 1, 0); dropping the translation column
  // would yield (0, 1, 0).
  NvGeMatrix3d aMat = NvGeMatrix3d::translation (NvGeVector3d (2.0, 0.0, 0.0));
  aMat.postMultBy (NvGeMatrix3d::rotation (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0)));

  const NvGePoint3d aPnt = aMat * NvGePoint3d (1.0, 0.0, 0.0);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, SetToProduct_MatchesMatrixPointOperator)
{
  const NvGeMatrix3d aMat = NvGeMatrix3d::rotation (THE_PI / 3.0, NvGeVector3d (0.0, 0.0, 1.0),
                                                    NvGePoint3d (1.0, 2.0, 0.5));
  const NvGePoint3d aPnt (2.0, -1.0, 3.0);
  const NvGePoint3d aExpected = aMat * aPnt;

  NvGePoint3d aResult;
  aResult.setToProduct (aMat, aPnt);
  EXPECT_NEAR (aResult.x, aExpected.x, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, aExpected.y, THE_TEST_TOL);
  EXPECT_NEAR (aResult.z, aExpected.z, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, TransformBy_AppliesFullMatrix_TranslationIsApplied)
{
  NvGeMatrix3d aMat = NvGeMatrix3d::translation (NvGeVector3d (2.0, 0.0, 0.0));
  aMat.postMultBy (NvGeMatrix3d::rotation (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0)));

  NvGePoint3d aPnt (1.0, 0.0, 0.0);
  aPnt.transformBy (aMat);
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, TransformBy_IdentityMatrix_KeepsPointUnchanged)
{
  NvGePoint3d aPnt (1.5, -2.5, 0.75);
  aPnt.transformBy (NvGeMatrix3d::kIdentity);
  EXPECT_NEAR (aPnt.x,  1.50, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, -2.50, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z,  0.75, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, RotateBy_AboutNonOriginAxis_KeepsAxisPointFixed)
{
  // Rotating (2, 1, 0) by 90 degrees about the z-axis through (1, 1, 0)
  // moves it to (1, 2, 0), while the axis point itself stays in place.
  NvGePoint3d aPnt (2.0, 1.0, 0.0);
  aPnt.rotateBy (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0), NvGePoint3d (1.0, 1.0, 0.0));
  EXPECT_NEAR (aPnt.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);

  NvGePoint3d aCenter (1.0, 1.0, 0.0);
  aCenter.rotateBy (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0), NvGePoint3d (1.0, 1.0, 0.0));
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, RotateBy_DefaultCenter_RotatesAboutOriginAxis)
{
  NvGePoint3d aPnt (1.0, 0.0, 0.0);
  aPnt.rotateBy (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, RotateBy_DegenerateAxis_Throws)
{
  NvGePoint3d aPnt (1.0, 0.0, 0.0);
  EXPECT_THROW (aPnt.rotateBy (1.0, NvGeVector3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGePoint3dTest, ScaleBy_AboutNonOriginCenter_KeepsCenterFixed)
{
  NvGePoint3d aPnt (3.0, 0.0, 0.0);
  aPnt.scaleBy (2.0, NvGePoint3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aPnt.x, 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.z, 0.0, THE_TEST_TOL);

  NvGePoint3d aOff (1.0, 5.0, -2.0);
  aOff.scaleBy (2.0, NvGePoint3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aOff.x,  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aOff.y, 10.0, THE_TEST_TOL);
  EXPECT_NEAR (aOff.z, -4.0, THE_TEST_TOL);

  NvGePoint3d aCenter (1.0, 0.0, 0.0);
  aCenter.scaleBy (2.0, NvGePoint3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, SetToSum_ComputesPointPlusVector)
{
  NvGePoint3d aResult;
  aResult.setToSum (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (4.0, -5.0, 6.0));
  EXPECT_NEAR (aResult.x,  5.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, -3.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.z,  9.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, DistanceTo_ReturnsEuclideanDistance)
{
  EXPECT_NEAR (NvGePoint3d (0.0, 0.0, 0.0).distanceTo (NvGePoint3d (3.0, 4.0, 12.0)),
               13.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGePoint3d (1.0, 2.0, 3.0).distanceTo (NvGePoint3d (1.0, 2.0, 3.0)),
               0.0, THE_TEST_TOL);
}

TEST_F (NvGePoint3dTest, IsEqualTo_RespectsEqualPointTolerance)
{
  NvGeTol aStrict;
  aStrict.setEqualPoint (1e-12);

  // The default (pinned) tolerance of 1e-8 accepts a ~5.2e-9 distance.
  const NvGePoint3d aPnt (1.0, 2.0, 3.0);
  const NvGePoint3d aNear (1.0 + 3e-9, 2.0 + 3e-9, 3.0 + 3e-9);
  EXPECT_TRUE  (aPnt.isEqualTo (aNear));
  EXPECT_FALSE (aPnt.isEqualTo (aNear, aStrict));
  EXPECT_FALSE (aPnt.isEqualTo (NvGePoint3d (2.0, 2.0, 3.0)));

  // operator == / != forward to isEqualTo with the global tolerance.
  EXPECT_TRUE  ((aPnt == aNear));
  EXPECT_FALSE ((aPnt != aNear));
  EXPECT_TRUE  ((aPnt != NvGePoint3d (2.0, 2.0, 3.0)));
}

TEST_F (NvGePoint3dTest, AsVectorAndDifference_ReturnTranslationsBetweenPoints)
{
  const NvGePoint3d aPnt0 (2.0, 3.0, 4.0);
  const NvGePoint3d aPnt1 (5.0, 7.0, 13.0);

  const NvGeVector3d aDiff = aPnt1 - aPnt0;
  EXPECT_NEAR (aDiff.x,  3.0, THE_TEST_TOL);
  EXPECT_NEAR (aDiff.y,  4.0, THE_TEST_TOL);
  EXPECT_NEAR (aDiff.z,  9.0, THE_TEST_TOL);

  const NvGeVector3d aVec = aPnt1.asVector();
  EXPECT_NEAR (aVec.x,  5.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y,  7.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z, 13.0, THE_TEST_TOL);
}
