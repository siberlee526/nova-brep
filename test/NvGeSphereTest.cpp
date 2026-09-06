#include <NvException.h>
#include <geline3d.h>
#include <gepnt3d.h>
#include <gesphere.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;
  constexpr double THE_TWO_PI = 2.0 * THE_PI;
  constexpr double THE_HALF_PI = 0.5 * THE_PI;

  // Tolerance for exact round-tripped radii, angles and coordinates.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeSphereTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeSphereTest, DefaultConstructor_IsUnitSphereAtOrigin)
{
  const NvGeSphere aSphere;
  EXPECT_NEAR (aSphere.radius(), 1.0, THE_TEST_TOL);
  const NvGePoint3d aCenter = aSphere.center();
  EXPECT_NEAR (aCenter.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 0.0, THE_TEST_TOL);

  double aStartU = 0.0, anEndU = 0.0;
  aSphere.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, THE_TWO_PI, THE_TEST_TOL);

  double aStartV = 0.0, anEndV = 0.0;
  aSphere.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, -THE_HALF_PI, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, THE_HALF_PI, THE_TEST_TOL);

  const NvGeVector3d aNorth = aSphere.northAxis();
  EXPECT_NEAR (aNorth.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aNorth.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aNorth.z, 1.0, THE_TEST_TOL);

  EXPECT_TRUE (aSphere.isClosed());
  EXPECT_TRUE (aSphere.isOuterNormal());
}

TEST_F (NvGeSphereTest, RadiusCenterConstructor_RoundTrip)
{
  const NvGeSphere aSphere (2.5, NvGePoint3d (1.0, -2.0, 3.0));
  EXPECT_NEAR (aSphere.radius(), 2.5, THE_TEST_TOL);
  const NvGePoint3d aCenter = aSphere.center();
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, -2.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 3.0, THE_TEST_TOL);
  EXPECT_TRUE (aSphere.isClosed());
}

TEST_F (NvGeSphereTest, FramedConstructor_AngleRoundTrip)
{
  const NvGeSphere aSphere (3.0, NvGePoint3d (0.0, 0.0, 0.0),
                            NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                            0.25, 1.75, -THE_HALF_PI + 0.1, THE_HALF_PI - 0.2);
  double aStartU = 0.0, anEndU = 0.0;
  aSphere.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.25, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 1.75, THE_TEST_TOL);

  double aStartV = 0.0, anEndV = 0.0;
  aSphere.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, -THE_HALF_PI + 0.1, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, THE_HALF_PI - 0.2, THE_TEST_TOL);

  const NvGeVector3d aNorth = aSphere.northAxis();
  EXPECT_NEAR (aNorth.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aNorth.z, 0.0, THE_TEST_TOL);
  EXPECT_FALSE (aSphere.isClosed());
}

TEST_F (NvGeSphereTest, Poles_AlongNorthAxis)
{
  const NvGePoint3d aCenter (1.0, 2.0, 3.0);
  const NvGeSphere aSphere (4.0, aCenter, NvGeVector3d (0.0, 1.0, 0.0),
                            NvGeVector3d (1.0, 0.0, 0.0), 0.0, THE_TWO_PI,
                            -THE_HALF_PI, THE_HALF_PI);
  const NvGePoint3d aNorth = aSphere.northPole();
  EXPECT_NEAR (aNorth.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aNorth.y, 6.0, THE_TEST_TOL);
  EXPECT_NEAR (aNorth.z, 3.0, THE_TEST_TOL);

  const NvGePoint3d aSouth = aSphere.southPole();
  EXPECT_NEAR (aSouth.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aSouth.y, -2.0, THE_TEST_TOL);
  EXPECT_NEAR (aSouth.z, 3.0, THE_TEST_TOL);
}

TEST_F (NvGeSphereTest, SetRadius_RoundTrip_PreservesAngleRange)
{
  NvGeSphere aSphere (2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                      NvGeVector3d (1.0, 0.0, 0.0), 0.1, 1.1, -0.3, 0.4);
  aSphere.setRadius (5.5);
  EXPECT_NEAR (aSphere.radius(), 5.5, THE_TEST_TOL);

  double aStartU = 0.0, anEndU = 0.0;
  aSphere.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.1, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 1.1, THE_TEST_TOL);
}

TEST_F (NvGeSphereTest, SetAngles_RoundTrip)
{
  NvGeSphere aSphere;
  aSphere.setAnglesInU (0.5, 2.5);
  double aStartU = 0.0, anEndU = 0.0;
  aSphere.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 2.5, THE_TEST_TOL);
  EXPECT_FALSE (aSphere.isClosed());

  aSphere.setAnglesInV (-0.2, 0.3);
  double aStartV = 0.0, anEndV = 0.0;
  aSphere.getAnglesInV (aStartV, anEndV);
  EXPECT_NEAR (aStartV, -0.2, THE_TEST_TOL);
  EXPECT_NEAR (anEndV, 0.3, THE_TEST_TOL);

  // The u range survives the v change and vice versa.
  aSphere.getAnglesInU (aStartU, anEndU);
  EXPECT_NEAR (aStartU, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEndU, 2.5, THE_TEST_TOL);
}

TEST_F (NvGeSphereTest, SetRadiusCenter_ResetsToFullSphere)
{
  NvGeSphere aSphere (2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                      NvGeVector3d (1.0, 0.0, 0.0), 0.1, 1.1, -0.3, 0.4);
  aSphere.set (3.0, NvGePoint3d (1.0, 1.0, 1.0));
  EXPECT_NEAR (aSphere.radius(), 3.0, THE_TEST_TOL);
  const NvGePoint3d aCenter = aSphere.center();
  EXPECT_NEAR (aCenter.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aSphere.isClosed());
}

TEST_F (NvGeSphereTest, SetRadius_Negative_Throws)
{
  NvGeSphere aSphere;
  EXPECT_THROW (aSphere.setRadius (-1.0), NvException);
  EXPECT_THROW (NvGeSphere (-0.5, NvGePoint3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGeSphereTest, SetAngles_Invalid_Throws)
{
  NvGeSphere aSphere;
  EXPECT_THROW (aSphere.setAnglesInU (1.0, 1.0), NvException);
  EXPECT_THROW (aSphere.setAnglesInU (2.0, 1.0), NvException);
  EXPECT_THROW (aSphere.setAnglesInV (-THE_HALF_PI - 0.1, 0.0), NvException);
  EXPECT_THROW (aSphere.setAnglesInV (0.0, THE_HALF_PI + 0.1), NvException);
}

TEST_F (NvGeSphereTest, FramedConstructor_ParallelAxes_Throw)
{
  EXPECT_THROW (NvGeSphere (1.0, NvGePoint3d (0.0, 0.0, 0.0),
                            NvGeVector3d (0.0, 0.0, 1.0), NvGeVector3d (0.0, 0.0, 2.0),
                            0.0, THE_TWO_PI, -THE_HALF_PI, THE_HALF_PI),
                NvException);
}

TEST_F (NvGeSphereTest, Copy_IsIndependent)
{
  NvGeSphere aSrc (2.0, NvGePoint3d (1.0, 1.0, 1.0));
  const NvGeSphere aCopy (aSrc);
  EXPECT_NEAR (aCopy.radius(), 2.0, THE_TEST_TOL);

  // Copy-on-write: mutating the source must not touch the shared geometry.
  aSrc.setRadius (5.0);
  EXPECT_NEAR (aSrc.radius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.radius(), 2.0, THE_TEST_TOL);
}

TEST_F (NvGeSphereTest, Assignment_IsIndependent)
{
  NvGeSphere aSrc (2.0, NvGePoint3d (1.0, 1.0, 1.0));
  NvGeSphere aAssigned;
  aAssigned = aSrc;
  aSrc.setRadius (7.0);
  EXPECT_NEAR (aAssigned.radius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aAssigned.center().x, 1.0, THE_TEST_TOL);
}

TEST_F (NvGeSphereTest, IntersectWith_LinearEntity_ReturnsFalse)
{
  const NvGeSphere aSphere;
  int anIntn = -1;
  NvGePoint3d aPnt1, aPnt2;
  EXPECT_FALSE (aSphere.intersectWith (NvGeLine3d(), anIntn, aPnt1, aPnt2));
  EXPECT_EQ (anIntn, 0);
}
