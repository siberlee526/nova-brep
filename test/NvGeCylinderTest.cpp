#include <NvException.h>
#include <geintrvl.h>
#include <geline3d.h>
#include <gepnt3d.h>
#include <gecylndr.h>
#include <getol.h>
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
class NvGeCylinderTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCylinderTest, DefaultConstructor_IsUnitCylinderAtOrigin)
{
  const NvGeCylinder aCylinder;
  EXPECT_NEAR (aCylinder.radius(), 1.0, THE_TEST_TOL);
  const NvGePoint3d anOrigin = aCylinder.origin();
  EXPECT_NEAR (anOrigin.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d anAxis = aCylinder.axisOfSymmetry();
  EXPECT_NEAR (anAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.z, 1.0, THE_TEST_TOL);

  double aStart = 0.0, anEnd = 0.0;
  aCylinder.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, THE_TWO_PI, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCylinder.getHeight (aHeight);
  EXPECT_TRUE (aHeight.isUnBounded());

  EXPECT_TRUE (aCylinder.isClosed());
  EXPECT_TRUE (aCylinder.isOuterNormal());
}

TEST_F (NvGeCylinderTest, AxisConstructor_RoundTrip)
{
  const NvGeCylinder aCylinder (3.0, NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (0.0, 1.0, 0.0));
  EXPECT_NEAR (aCylinder.radius(), 3.0, THE_TEST_TOL);
  const NvGePoint3d anOrigin = aCylinder.origin();
  EXPECT_NEAR (anOrigin.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anOrigin.z, 3.0, THE_TEST_TOL);

  const NvGeVector3d anAxis = aCylinder.axisOfSymmetry();
  EXPECT_NEAR (anAxis.y, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.z, 0.0, THE_TEST_TOL);

  // Orthonormal frame: the reference axis must stay perpendicular to the
  // axis of symmetry.
  const NvGeVector3d aRef = aCylinder.refAxis();
  EXPECT_NEAR (std::abs (aRef.x * anAxis.x + aRef.y * anAxis.y + aRef.z * anAxis.z),
               0.0, THE_TEST_TOL);
  EXPECT_TRUE (aCylinder.isClosed());
}

TEST_F (NvGeCylinderTest, FullConstructor_RoundTrip)
{
  const NvGeCylinder aCylinder (2.0, NvGePoint3d (1.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                                NvGeVector3d (1.0, 0.0, 0.0), NvGeInterval (0.0, 10.0),
                                0.5, 2.0);
  EXPECT_NEAR (aCylinder.radius(), 2.0, THE_TEST_TOL);

  double aStart = 0.0, anEnd = 0.0;
  aCylinder.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCylinder.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 10.0, THE_TEST_TOL);

  const NvGeVector3d aRef = aCylinder.refAxis();
  EXPECT_NEAR (aRef.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.z, 0.0, THE_TEST_TOL);

  EXPECT_FALSE (aCylinder.isClosed());
}

TEST_F (NvGeCylinderTest, HeightAt_IsIdentityOnHeightParameter)
{
  const NvGeCylinder aCylinder;
  EXPECT_NEAR (aCylinder.heightAt (3.5), 3.5, THE_TEST_TOL);
  EXPECT_NEAR (aCylinder.heightAt (-2.25), -2.25, THE_TEST_TOL);
}

TEST_F (NvGeCylinderTest, SetRadius_RoundTrip_PreservesTrim)
{
  NvGeCylinder aCylinder (2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                          NvGeVector3d (1.0, 0.0, 0.0), NvGeInterval (0.0, 10.0), 0.5, 2.0);
  aCylinder.setRadius (4.0);
  EXPECT_NEAR (aCylinder.radius(), 4.0, THE_TEST_TOL);

  double aStart = 0.0, anEnd = 0.0;
  aCylinder.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCylinder.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 10.0, THE_TEST_TOL);
}

TEST_F (NvGeCylinderTest, SetAngles_RoundTrip)
{
  NvGeCylinder aCylinder;
  aCylinder.setAngles (0.25, 1.75);
  double aStart = 0.0, anEnd = 0.0;
  aCylinder.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.25, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 1.75, THE_TEST_TOL);
  EXPECT_FALSE (aCylinder.isClosed());
}

TEST_F (NvGeCylinderTest, SetHeight_RoundTrip_PreservesAngles)
{
  NvGeCylinder aCylinder;
  aCylinder.setAngles (0.5, 2.0);
  aCylinder.setHeight (NvGeInterval (-5.0, 5.0));

  NvGeInterval aHeight;
  aCylinder.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, -5.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 5.0, THE_TEST_TOL);
  EXPECT_TRUE (aHeight.isBounded());

  double aStart = 0.0, anEnd = 0.0;
  aCylinder.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCylinderTest, SetRadius_Negative_Throws)
{
  NvGeCylinder aCylinder;
  EXPECT_THROW (aCylinder.setRadius (-1.0), NvException);
  EXPECT_THROW (NvGeCylinder (-2.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0)),
                NvException);
}

TEST_F (NvGeCylinderTest, SetAngles_Invalid_Throws)
{
  NvGeCylinder aCylinder;
  EXPECT_THROW (aCylinder.setAngles (1.0, 1.0), NvException);
  EXPECT_THROW (aCylinder.setAngles (2.0, 1.0), NvException);
}

TEST_F (NvGeCylinderTest, SetHeight_Invalid_Throws)
{
  NvGeCylinder aCylinder;
  EXPECT_THROW (aCylinder.setHeight (NvGeInterval()), NvException); // unbounded
  EXPECT_THROW (aCylinder.setHeight (NvGeInterval (5.0, 5.0)), NvException);
}

TEST_F (NvGeCylinderTest, Set_AxisForm_RoundTrip)
{
  NvGeCylinder aCylinder;
  aCylinder.set (2.5, NvGePoint3d (1.0, -1.0, 2.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_NEAR (aCylinder.radius(), 2.5, THE_TEST_TOL);
  EXPECT_NEAR (aCylinder.axisOfSymmetry().x, 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aCylinder.isClosed());
}

TEST_F (NvGeCylinderTest, Copy_IsIndependent)
{
  NvGeCylinder aSrc (2.0, NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeCylinder aCopy (aSrc);
  EXPECT_NEAR (aCopy.radius(), 2.0, THE_TEST_TOL);

  // Copy-on-write: mutating the source must not touch the shared geometry.
  aSrc.setRadius (6.0);
  aSrc.setHeight (NvGeInterval (0.0, 1.0));
  EXPECT_NEAR (aSrc.radius(), 6.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.radius(), 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCopy.getHeight (aHeight);
  EXPECT_TRUE (aHeight.isUnBounded());
}

TEST_F (NvGeCylinderTest, Assignment_IsIndependent)
{
  NvGeCylinder aSrc (2.0, NvGePoint3d (1.0, 1.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  NvGeCylinder aAssigned;
  aAssigned = aSrc;
  aSrc.setRadius (7.0);
  EXPECT_NEAR (aAssigned.radius(), 2.0, THE_TEST_TOL);
}

TEST_F (NvGeCylinderTest, IntersectWith_LinearEntity_ReturnsFalse)
{
  const NvGeCylinder aCylinder;
  int anIntn = -1;
  NvGePoint3d aPnt1, aPnt2;
  EXPECT_FALSE (aCylinder.intersectWith (NvGeLine3d(), anIntn, aPnt1, aPnt2));
  EXPECT_EQ (anIntn, 0);
}
