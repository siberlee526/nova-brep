#include <NvException.h>
#include <geline3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <getorus.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;
  constexpr double THE_TWO_PI = 2.0 * THE_PI;

  // Tolerance for exact round-tripped radii, angles and coordinates.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeTorusTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeTorusTest, DefaultConstructor_IsDoughnutAtOrigin)
{
  const NvGeTorus aTorus;
  EXPECT_NEAR (aTorus.majorRadius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.minorRadius(), 1.0, THE_TEST_TOL);

  const NvGePoint3d aCenter = aTorus.center();
  EXPECT_NEAR (aCenter.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d anAxis = aTorus.axisOfSymmetry();
  EXPECT_NEAR (anAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.z, 1.0, THE_TEST_TOL);

  double aStartU = 0.0, anEndU = 0.0;
  aTorus.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, THE_TWO_PI, THE_TEST_TOL);

  double aStartV = 0.0, anEndV = 0.0;
  aTorus.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, THE_TWO_PI, THE_TEST_TOL);

  EXPECT_TRUE (aTorus.isOuterNormal());
}

TEST_F (NvGeTorusTest, RadiiConstructor_RoundTrip)
{
  const NvGeTorus aTorus (5.0, 1.5, NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_NEAR (aTorus.majorRadius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.minorRadius(), 1.5, THE_TEST_TOL);

  const NvGePoint3d aCenter = aTorus.center();
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 3.0, THE_TEST_TOL);

  const NvGeVector3d anAxis = aTorus.axisOfSymmetry();
  EXPECT_NEAR (anAxis.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.z, 0.0, THE_TEST_TOL);

  // Orthonormal frame: the reference axis must stay perpendicular to the
  // axis of symmetry.
  const NvGeVector3d aRef = aTorus.refAxis();
  EXPECT_NEAR (std::abs (aRef.x * anAxis.x + aRef.y * anAxis.y + aRef.z * anAxis.z),
               0.0, THE_TEST_TOL);
}

TEST_F (NvGeTorusTest, Classification_DoughnutAppleLemonPartition)
{
  const NvGeTorus aDoughnut (3.0, 1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_TRUE (aDoughnut.isDoughnut());
  EXPECT_TRUE (aDoughnut.isHollow());
  EXPECT_FALSE (aDoughnut.isApple());
  EXPECT_FALSE (aDoughnut.isLemon());
  EXPECT_FALSE (aDoughnut.isVortex());
  EXPECT_FALSE (aDoughnut.isDegenerate());

  const NvGeTorus anApple (1.0, 2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_TRUE (anApple.isApple());
  EXPECT_FALSE (anApple.isHollow());
  EXPECT_FALSE (anApple.isDoughnut());
  EXPECT_FALSE (anApple.isLemon());
  EXPECT_FALSE (anApple.isVortex());
  EXPECT_FALSE (anApple.isDegenerate());

  const NvGeTorus aLemon (2.0, 2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_TRUE (aLemon.isLemon());
  EXPECT_FALSE (aLemon.isHollow());
  EXPECT_FALSE (aLemon.isDoughnut());
  EXPECT_FALSE (aLemon.isApple());
  EXPECT_FALSE (aLemon.isVortex());
  EXPECT_FALSE (aLemon.isDegenerate());

  const NvGeTorus aDegenerate (2.0, 0.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_TRUE (aDegenerate.isDegenerate());
  EXPECT_FALSE (aDegenerate.isDoughnut());
  EXPECT_FALSE (aDegenerate.isApple());
  EXPECT_FALSE (aDegenerate.isLemon());
  EXPECT_FALSE (aDegenerate.isVortex());
}

TEST_F (NvGeTorusTest, FullConstructor_AngleRoundTrip)
{
  const NvGeTorus aTorus (4.0, 1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                          NvGeVector3d (1.0, 0.0, 0.0), 0.1, 1.0, 0.2, 2.0);
  double aStartU = 0.0, anEndU = 0.0;
  aTorus.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.1, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 1.0, THE_TEST_TOL);

  double aStartV = 0.0, anEndV = 0.0;
  aTorus.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, 0.2, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, 2.0, THE_TEST_TOL);

  const NvGeVector3d aRef = aTorus.refAxis();
  EXPECT_NEAR (aRef.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeTorusTest, SetRadii_RoundTrip_UpdateClassification)
{
  NvGeTorus aTorus (3.0, 1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));
  aTorus.setMajorRadius (5.5);
  EXPECT_NEAR (aTorus.majorRadius(), 5.5, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.minorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aTorus.isDoughnut());

  aTorus.setMinorRadius (7.0);
  EXPECT_NEAR (aTorus.minorRadius(), 7.0, THE_TEST_TOL);
  EXPECT_TRUE (aTorus.isApple());
  EXPECT_FALSE (aTorus.isDoughnut());
  EXPECT_FALSE (aTorus.isHollow());

  aTorus.setMajorRadius (7.0);
  EXPECT_TRUE (aTorus.isLemon());
}

TEST_F (NvGeTorusTest, SetAngles_RoundTrip_PreserveEachOther)
{
  NvGeTorus aTorus;
  aTorus.setAnglesInU (0.3, 1.3);
  double aStartU = 0.0, anEndU = 0.0;
  aTorus.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.3, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 1.3, THE_TEST_TOL);

  aTorus.setAnglesInV (0.5, 2.5);
  double aStartV = 0.0, anEndV = 0.0;
  aTorus.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, 2.5, THE_TEST_TOL);

  aTorus.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.3, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 1.3, THE_TEST_TOL);
}

TEST_F (NvGeTorusTest, SetRadii_Negative_Throws)
{
  NvGeTorus aTorus;
  EXPECT_THROW (aTorus.setMajorRadius (-1.0), NvException);
  EXPECT_THROW (aTorus.setMinorRadius (-1.0), NvException);
  EXPECT_THROW (NvGeTorus (2.0, -0.5, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0)),
                NvException);
}

TEST_F (NvGeTorusTest, SetAngles_Invalid_Throws)
{
  NvGeTorus aTorus;
  EXPECT_THROW (aTorus.setAnglesInU (1.0, 1.0), NvException);
  EXPECT_THROW (aTorus.setAnglesInU (2.0, 1.0), NvException);
  EXPECT_THROW (aTorus.setAnglesInV (1.0, 0.5), NvException);
}

TEST_F (NvGeTorusTest, Set_RadiiForm_RoundTrip)
{
  NvGeTorus aTorus;
  aTorus.set (4.0, 0.5, NvGePoint3d (1.0, -1.0, 2.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aTorus.majorRadius(), 4.0, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.minorRadius(), 0.5, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.center().x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aTorus.axisOfSymmetry().x, 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aTorus.isDoughnut());
}

TEST_F (NvGeTorusTest, Copy_IsIndependent)
{
  NvGeTorus aSrc (3.0, 1.0, NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeTorus aCopy (aSrc);
  EXPECT_NEAR (aCopy.majorRadius(), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.minorRadius(), 1.0, THE_TEST_TOL);

  // Copy-on-write: mutating the source must not touch the shared geometry.
  aSrc.setMinorRadius (5.0);
  EXPECT_NEAR (aSrc.minorRadius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.minorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aCopy.isDoughnut());
}

TEST_F (NvGeTorusTest, Assignment_IsIndependent)
{
  NvGeTorus aSrc (3.0, 1.0, NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  NvGeTorus aAssigned;
  aAssigned = aSrc;
  aSrc.setMinorRadius (6.0);
  EXPECT_NEAR (aAssigned.minorRadius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aAssigned.center().x, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeTorusTest, IntersectWith_LinearEntity_ReturnsFalse)
{
  const NvGeTorus aTorus;
  int anIntn = -1;
  NvGePoint3d aPnt1, aPnt2, aPnt3, aPnt4;
  EXPECT_FALSE (aTorus.intersectWith (NvGeLine3d(), anIntn, aPnt1, aPnt2, aPnt3, aPnt4));
  EXPECT_EQ (anIntn, 0);
}
