#include <NvException.h>
#include <gegblabb.h>
#include <gemat3d.h>
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
class NvGeVector3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeVector3dTest, StaticConstants_HoldAxisDirections)
{
  EXPECT_NEAR (NvGeVector3d::kIdentity.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kIdentity.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kIdentity.z, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kXAxis.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kXAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kXAxis.z, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kYAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kYAxis.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kYAxis.z, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kZAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kZAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (NvGeVector3d::kZAxis.z, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, RotateBy_RotatesAroundAxis)
{
  // Rodrigues rotation: +x turned by +90 degrees about +z gives +y.
  NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  aXAxis.rotateBy (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aXAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.z, 0.0, THE_TEST_TOL);

  // The axis length does not matter.
  NvGeVector3d aSame (1.0, 0.0, 0.0);
  aSame.rotateBy (THE_PI / 2.0, NvGeVector3d (0.0, 0.0, 7.0));
  EXPECT_NEAR (aSame.y, 1.0, THE_TEST_TOL);

  // +z rotated by pi about +y gives -z.
  NvGeVector3d aZAxis (0.0, 0.0, 1.0);
  aZAxis.rotateBy (THE_PI, NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_NEAR (aZAxis.x,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aZAxis.y,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aZAxis.z, -1.0, THE_TEST_TOL);

  // Length is preserved.
  NvGeVector3d aVec (3.0, -4.0, 0.0);
  aVec.rotateBy (1.234, NvGeVector3d (1.0, 1.0, 1.0));
  EXPECT_NEAR (aVec.length(), 5.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, RotateBy_ZeroLengthAxis_Throws)
{
  NvGeVector3d aVec (1.0, 0.0, 0.0);
  EXPECT_THROW (aVec.rotateBy (1.0, NvGeVector3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGeVector3dTest, Mirror_ReflectsInPlanePerpendicularToNormal)
{
  NvGeVector3d aVec (1.0, 2.0, 3.0);
  aVec.mirror (NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_NEAR (aVec.x,  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, -2.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z,  3.0, THE_TEST_TOL);

  // Reflection in the plane x + y = 0 swaps and negates the components.
  NvGeVector3d aSwapped (1.0, 0.0, 0.0);
  aSwapped.mirror (NvGeVector3d (1.0, 1.0, 0.0));
  EXPECT_NEAR (aSwapped.x,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aSwapped.y, -1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Mirror_ZeroLengthNormal_Throws)
{
  NvGeVector3d aVec (1.0, 0.0, 0.0);
  EXPECT_THROW (aVec.mirror (NvGeVector3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGeVector3dTest, AngleTo_ReturnsUnsignedAngleIn0ToPi)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (0.0, 1.0, 0.0)),  THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (0.0, 0.0, 5.0)),  THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (-3.0, 0.0, 0.0)), THE_PI,       THE_TEST_TOL);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (2.0, 0.0, 0.0)),  0.0,          THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, AngleToWithAxis_MeasuresCCWAngleAroundAxis)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  const NvGeVector3d aZAxis (0.0, 0.0, 1.0);
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (0.0, 1.0, 0.0), aZAxis),
               THE_PI / 2.0, THE_TEST_TOL);

  // Counterclockwise around +z: from +x to -y is three quarter turns.
  EXPECT_NEAR (aXAxis.angleTo (NvGeVector3d (0.0, -1.0, 0.0), aZAxis),
               3.0 * THE_PI / 2.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, AngleToWithAxis_ZeroLengthAxis_Throws)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_THROW (aXAxis.angleTo (NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 0.0)),
                NvException);
}

TEST_F (NvGeVector3dTest, PerpVector_ReturnsUnitPerpendicular)
{
  const NvGeVector3d aVec (1.0, 2.0, 3.0);
  const NvGeVector3d aPerp = aVec.perpVector();
  EXPECT_NEAR (aPerp.length(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPerp.dotProduct (aVec), 0.0, THE_TEST_TOL);

  // For an axis-aligned vector the least aligned axis x is crossed out.
  const NvGeVector3d aZAxis = NvGeVector3d (0.0, 0.0, 5.0).perpVector();
  EXPECT_NEAR (aZAxis.x,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aZAxis.y, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aZAxis.z,  0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Normal_ReturnsUnitVectorWithoutMutating)
{
  const NvGeVector3d aVec (3.0, 4.0, 0.0);
  const NvGeVector3d aUnit = aVec.normal();
  EXPECT_NEAR (aUnit.x, 0.6, THE_TEST_TOL);
  EXPECT_NEAR (aUnit.y, 0.8, THE_TEST_TOL);
  EXPECT_NEAR (aUnit.z, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.x, 3.0, THE_TEST_TOL); // original is untouched
}

TEST_F (NvGeVector3dTest, Normal_ZeroVector_ReturnsUnchangedCopy)
{
  const NvGeVector3d aZero (0.0, 0.0, 0.0);
  const NvGeVector3d aResult = aZero.normal();
  EXPECT_NEAR (aResult.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aResult.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Normalize_ScalesToUnitLength)
{
  NvGeVector3d aVec (1.0, 2.0, 2.0);
  aVec.normalize();
  EXPECT_NEAR (aVec.x, 1.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 2.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z, 2.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.length(), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Normalize_ZeroVector_LeavesUnchanged)
{
  NvGeVector3d aVec (0.0, 0.0, 0.0);
  aVec.normalize();
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, NormalizeWithFlag_ReportsK0ThisForZeroVector)
{
  NvGeError aFlag = kOk;

  NvGeVector3d aVec (0.0, 0.0, 0.0);
  aVec.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0This);
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z, 0.0, THE_TEST_TOL);

  // A vector within the vector tolerance counts as zero as well.
  NvGeVector3d aTiny (0.0, 1e-9, 0.0);
  aTiny.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0This);

  NvGeVector3d aGood (1.0, 2.0, 2.0);
  aGood.normalize (NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, kOk);
  EXPECT_NEAR (aGood.length(), 1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Length_ReturnsEuclideanLength)
{
  const NvGeVector3d aVec (1.0, 2.0, 2.0);
  EXPECT_NEAR (aVec.length(), 3.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, IsUnitLengthAndIsZeroLength_ClassifyWithinTolerance)
{
  EXPECT_TRUE  (NvGeVector3d (0.0, 0.0, 1.0).isUnitLength());
  EXPECT_TRUE  (NvGeVector3d (0.999999999, 0.0, 0.0).isUnitLength()); // 1e-9 deviation
  EXPECT_FALSE (NvGeVector3d (2.0, 0.0, 0.0).isUnitLength());

  EXPECT_TRUE  (NvGeVector3d (0.0, 0.0, 0.0).isZeroLength());
  EXPECT_TRUE  (NvGeVector3d (0.0, 0.0, 1e-9).isZeroLength());
  EXPECT_FALSE (NvGeVector3d (0.0, 0.0, 1e-7).isZeroLength());
}

TEST_F (NvGeVector3dTest, IsParallelTo_UsesCrossProductLength)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector3d (2.0, 0.0, 0.0)));
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector3d (-3.0, 0.0, 0.0))); // antiparallel counts
  EXPECT_TRUE  (aXAxis.isParallelTo (NvGeVector3d (5.0, 0.0, 1e-9))); // within tolerance
  EXPECT_FALSE (aXAxis.isParallelTo (NvGeVector3d (0.0, 2.0, 0.0)));
}

TEST_F (NvGeVector3dTest, IsParallelToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector3d (0.0, 0.0, 0.0).isParallelTo (NvGeVector3d (1.0, 0.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector3d (1.0, 0.0, 0.0).isParallelTo (NvGeVector3d (0.0, 0.0, 0.0),
                                                           NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector3d (1.0, 0.0, 0.0).isParallelTo (NvGeVector3d (2.0, 0.0, 0.0),
                                                          NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector3dTest, IsCodirectionalTo_RequiresParallelAndSameDirection)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_TRUE  (aXAxis.isCodirectionalTo (NvGeVector3d (2.0, 0.0, 0.0)));
  EXPECT_FALSE (aXAxis.isCodirectionalTo (NvGeVector3d (-2.0, 0.0, 0.0)));
  EXPECT_FALSE (aXAxis.isCodirectionalTo (NvGeVector3d (0.0, 1.0, 0.0)));
}

TEST_F (NvGeVector3dTest, IsCodirectionalToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector3d (0.0, 0.0, 0.0).isCodirectionalTo (NvGeVector3d (1.0, 0.0, 0.0),
                                                                NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector3d (1.0, 0.0, 0.0).isCodirectionalTo (NvGeVector3d (0.0, 0.0, 0.0),
                                                                NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector3d (1.0, 0.0, 0.0).isCodirectionalTo (NvGeVector3d (3.0, 0.0, 0.0),
                                                               NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector3dTest, IsPerpendicularTo_UsesDotProductMagnitude)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_TRUE  (aXAxis.isPerpendicularTo (NvGeVector3d (0.0, 3.0, 4.0)));
  EXPECT_TRUE  (aXAxis.isPerpendicularTo (NvGeVector3d (1e-9, 2.0, 0.0))); // dot within tolerance
  EXPECT_FALSE (aXAxis.isPerpendicularTo (NvGeVector3d (1.0, 1.0, 0.0)));
}

TEST_F (NvGeVector3dTest, IsPerpendicularToWithFlag_ReportsZeroVectors)
{
  NvGeError aFlag = kOk;

  EXPECT_FALSE (NvGeVector3d (0.0, 0.0, 0.0).isPerpendicularTo (NvGeVector3d (1.0, 0.0, 0.0),
                                                                NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0This);

  EXPECT_FALSE (NvGeVector3d (1.0, 0.0, 0.0).isPerpendicularTo (NvGeVector3d (0.0, 0.0, 0.0),
                                                                NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, k0Arg1);

  EXPECT_TRUE (NvGeVector3d (1.0, 0.0, 0.0).isPerpendicularTo (NvGeVector3d (0.0, 2.0, 2.0),
                                                               NvGeContext::gTol, aFlag));
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector3dTest, CrossProduct_ComputesRightHandedProduct)
{
  const NvGeVector3d aCross = NvGeVector3d (1.0, 0.0, 0.0)
                                  .crossProduct (NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_NEAR (aCross.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCross.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCross.z, 1.0, THE_TEST_TOL);

  // Anticommutativity and orthogonality to both operands.
  const NvGeVector3d aVec (1.0, 2.0, 3.0);
  const NvGeVector3d aOther (2.0, 1.0, 0.0);
  EXPECT_NEAR (aVec.crossProduct (aOther).x, -aOther.crossProduct (aVec).x, THE_TEST_TOL);
  EXPECT_NEAR (aVec.crossProduct (aOther).dotProduct (aVec),    0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.crossProduct (aOther).dotProduct (aOther),  0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, TransformBy_AppliesLinearPartOnly)
{
  // Composition of a rotation with a translation: the translation column
  // must not affect the vector.
  const NvGeMatrix3d aMat = NvGeMatrix3d::rotation (THE_PI / 2.0, NvGeVector3d::kZAxis)
                          * NvGeMatrix3d::translation (NvGeVector3d (3.0, 4.0, 5.0));

  NvGeVector3d aVec (2.0, 0.0, 0.0);
  aVec.transformBy (aMat);
  EXPECT_NEAR (aVec.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.z, 0.0, THE_TEST_TOL);

  // Scaling acts on the components.
  NvGeVector3d aScaled (1.0, 2.0, 3.0);
  aScaled.transformBy (NvGeMatrix3d::scaling (2.0));
  EXPECT_NEAR (aScaled.x, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aScaled.y, 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aScaled.z, 6.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, OperatorMultiplyMatrixVector_MatchesTransformBy)
{
  const NvGeMatrix3d aMat = NvGeMatrix3d::rotation (0.7, NvGeVector3d (1.0, 1.0, 1.0))
                          * NvGeMatrix3d::translation (NvGeVector3d (1.0, 2.0, 3.0));
  const NvGeVector3d aVec (2.0, -3.0, 4.0);

  NvGeVector3d aTransformed = aVec;
  aTransformed.transformBy (aMat);

  const NvGeVector3d aProduct = aMat * aVec;
  EXPECT_NEAR (aProduct.x, aTransformed.x, THE_TEST_TOL);
  EXPECT_NEAR (aProduct.y, aTransformed.y, THE_TEST_TOL);
  EXPECT_NEAR (aProduct.z, aTransformed.z, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, OperatorMultiplyScalar_FreeFunctionMatchesMember)
{
  const NvGeVector3d aVec (2.0, -3.0, 4.0);
  const NvGeVector3d aFree = 2.0 * aVec;
  EXPECT_NEAR (aFree.x,  4.0, THE_TEST_TOL);
  EXPECT_NEAR (aFree.y, -6.0, THE_TEST_TOL);
  EXPECT_NEAR (aFree.z,  8.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, RotateTo_ReturnsMatrixMappingThisOntoArgument)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);

  // Default axis (this x vec): the smallest rotation mapping +x onto +y.
  NvGeVector3d aRotated = aXAxis;
  aRotated.transformBy (aXAxis.rotateTo (NvGeVector3d (0.0, 1.0, 0.0)));
  EXPECT_NEAR (aRotated.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aRotated.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aRotated.z, 0.0, THE_TEST_TOL);

  // The same rotation with an explicit axis.
  aRotated = aXAxis;
  aRotated.transformBy (aXAxis.rotateTo (NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d::kZAxis));
  EXPECT_NEAR (aRotated.y, 1.0, THE_TEST_TOL);

  // Codirectional vectors need no rotation.
  aRotated = aXAxis;
  aRotated.transformBy (aXAxis.rotateTo (NvGeVector3d (2.0, 0.0, 0.0)));
  EXPECT_NEAR (aRotated.x, 1.0, THE_TEST_TOL);

  // Opposite vectors rotate by pi about a perpendicular axis.
  aRotated = aXAxis;
  aRotated.transformBy (aXAxis.rotateTo (NvGeVector3d (-2.0, 0.0, 0.0)));
  EXPECT_NEAR (aRotated.x, -1.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, RotateTo_ZeroLengthVector_Throws)
{
  const NvGeVector3d aXAxis (1.0, 0.0, 0.0);
  EXPECT_THROW (aXAxis.rotateTo (NvGeVector3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGeVector3dTest, Project_ProjectsObliquelyAlongDirection)
{
  // (1, 2, 3) projected onto the xy-plane along (0, 1, 1) slides to (1, -1, 0):
  // v - d * (v . n) / (d . n) = (1, 2, 3) - (0, 1, 1) * 3.
  const NvGeVector3d aProjected = NvGeVector3d (1.0, 2.0, 3.0)
                                      .project (NvGeVector3d (0.0, 0.0, 1.0),
                                                NvGeVector3d (0.0, 1.0, 1.0));
  EXPECT_NEAR (aProjected.x,  1.0, THE_TEST_TOL);
  EXPECT_NEAR (aProjected.y, -1.0, THE_TEST_TOL);
  EXPECT_NEAR (aProjected.z,  0.0, THE_TEST_TOL);
  EXPECT_NEAR (aProjected.dotProduct (NvGeVector3d (0.0, 0.0, 1.0)), 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, Project_InvalidInput_ThrowsOrSetsFlag)
{
  const NvGeVector3d aVec (1.0, 2.0, 3.0);
  const NvGeVector3d aZAxis (0.0, 0.0, 1.0);

  EXPECT_THROW (aVec.project (NvGeVector3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0)),
                NvException);
  EXPECT_THROW (aVec.project (aZAxis, NvGeVector3d (0.0, 0.0, 0.0)), NvException);

  NvGeError aFlag = kOk;
  aVec.project (NvGeVector3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0),
                NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0Arg1);

  aVec.project (aZAxis, NvGeVector3d (0.0, 0.0, 0.0), NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0Arg2);

  // A direction lying in the plane never meets it.
  aVec.project (aZAxis, NvGeVector3d (1.0, 0.0, 0.0), NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, kPerpendicularArg1Arg2);

  aVec.project (aZAxis, NvGeVector3d (0.0, 1.0, 1.0), NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector3dTest, OrthoProject_ProjectsPerpendicularly)
{
  const NvGeVector3d aProjected = NvGeVector3d (1.0, 2.0, 3.0)
                                      .orthoProject (NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aProjected.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aProjected.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aProjected.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeVector3dTest, OrthoProject_ZeroLengthNormal_ThrowsOrSetsFlag)
{
  const NvGeVector3d aVec (1.0, 2.0, 3.0);
  EXPECT_THROW (aVec.orthoProject (NvGeVector3d (0.0, 0.0, 0.0)), NvException);

  NvGeError aFlag = kOk;
  aVec.orthoProject (NvGeVector3d (0.0, 0.0, 0.0), NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, k0Arg1);

  aVec.orthoProject (NvGeVector3d (0.0, 0.0, 1.0), NvGeContext::gTol, aFlag);
  EXPECT_EQ (aFlag, kOk);
}

TEST_F (NvGeVector3dTest, IsEqualTo_UsesEuclideanDistanceWithinTolerance)
{
  NvGeTol aStrict;
  aStrict.setEqualPoint (1e-8);
  aStrict.setEqualVector (1e-12);

  const NvGeVector3d aVec (1.0, 2.0, 3.0);
  const NvGeVector3d aClose (1.0, 2.0, 3.0 + 5e-9);

  EXPECT_TRUE  (aVec.isEqualTo (aClose));      // distance 5e-9 <= 1e-8
  EXPECT_FALSE (aVec.isEqualTo (aClose, aStrict));
  EXPECT_TRUE  (aVec == NvGeVector3d (1.0, 2.0, 3.0));
  EXPECT_TRUE  (aVec != NvGeVector3d (1.0, 2.0, 3.1));
}

TEST_F (NvGeVector3dTest, LargestElement_ReturnsIndexOfLargestMagnitude)
{
  EXPECT_EQ (NvGeVector3d (1.0, 3.0, -4.5).largestElement(), 2u);
  EXPECT_EQ (NvGeVector3d (-5.0, 2.0, 1.0).largestElement(), 0u);
  EXPECT_EQ (NvGeVector3d (1.0, -2.0, 3.0).largestElement(), 2u);
  EXPECT_EQ (NvGeVector3d (2.0, 1.0, 0.5).largestElement(), 0u);
}

TEST_F (NvGeVector3dTest, MatrixConversion_ProducesTranslationMatrix)
{
  const NvGeMatrix3d aMat = NvGeVector3d (1.0, 2.0, 3.0);
  for (int aRow = 0; aRow < 3; ++aRow)
  {
    for (int aCol = 0; aCol < 3; ++aCol)
    {
      EXPECT_NEAR (aMat (aRow, aCol), aRow == aCol ? 1.0 : 0.0, THE_TEST_TOL);
    }
  }
  EXPECT_NEAR (aMat (0, 3), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (1, 3), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aMat (2, 3), 3.0, THE_TEST_TOL);
}
