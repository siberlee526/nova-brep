#include <NvException.h>
#include <gecone.h>
#include <geintrvl.h>
#include <geline3d.h>
#include <gepnt3d.h>
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
class NvGeConeTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeConeTest, DefaultConstructor_IsQuarterConeAlongZ)
{
  const NvGeCone aCone;
  EXPECT_NEAR (aCone.halfAngle(), THE_PI * 0.25, THE_TEST_TOL);
  EXPECT_NEAR (aCone.baseRadius(), 1.0, THE_TEST_TOL);

  const NvGePoint3d aCenter = aCone.baseCenter();
  EXPECT_NEAR (aCenter.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aCenter.z, 0.0, THE_TEST_TOL);

  const NvGeVector3d anAxis = aCone.axisOfSymmetry();
  EXPECT_NEAR (anAxis.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anAxis.z, 1.0, THE_TEST_TOL);

  // Apex one base radius above the base center for a PI/4 half angle.
  const NvGePoint3d anApex = aCone.apex();
  EXPECT_NEAR (anApex.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anApex.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anApex.z, 1.0, THE_TEST_TOL);

  double aCos = 0.0, aSin = 0.0;
  aCone.getHalfAngle (aCos, aSin);
  EXPECT_NEAR (aCos, std::sqrt (2.0) * 0.5, THE_TEST_TOL);
  EXPECT_NEAR (aSin, std::sqrt (2.0) * 0.5, THE_TEST_TOL);

  EXPECT_TRUE (aCone.isClosed());
  EXPECT_TRUE (aCone.isOuterNormal());

  NvGeInterval aHeight;
  aCone.getHeight (aHeight);
  EXPECT_TRUE (aHeight.isUnBounded());
}

TEST_F (NvGeConeTest, BaseConstructor_RoundTrip)
{
  const double aHalf = 0.3;
  const NvGeCone aCone (std::cos (aHalf), std::sin (aHalf), NvGePoint3d (1.0, 2.0, 3.0),
                        4.0, NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aCone.halfAngle(), aHalf, THE_TEST_TOL);
  EXPECT_NEAR (aCone.baseRadius(), 4.0, THE_TEST_TOL);

  // The axis of symmetry points from the base center toward the apex.
  const NvGePoint3d anApex = aCone.apex();
  EXPECT_NEAR (anApex.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anApex.y, 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anApex.z, 3.0 + 4.0 / std::tan (aHalf), THE_TEST_TOL);
  EXPECT_TRUE (aCone.isClosed());
}

TEST_F (NvGeConeTest, FullConstructor_RoundTrip)
{
  const double aHalf = 0.4;
  const NvGeCone aCone (std::cos (aHalf), std::sin (aHalf), NvGePoint3d (1.0, 1.0, 0.0),
                        3.0, NvGeVector3d (0.0, 0.0, 1.0), NvGeVector3d (1.0, 0.0, 0.0),
                        NvGeInterval (0.0, 5.0), 0.5, 2.0);
  EXPECT_NEAR (aCone.halfAngle(), aHalf, THE_TEST_TOL);
  EXPECT_NEAR (aCone.baseRadius(), 3.0, THE_TEST_TOL);

  double aStart = 0.0, anEnd = 0.0;
  aCone.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCone.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 5.0, THE_TEST_TOL);

  const NvGeVector3d aRef = aCone.refAxis();
  EXPECT_NEAR (aRef.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aRef.z, 0.0, THE_TEST_TOL);

  EXPECT_FALSE (aCone.isClosed());
}

TEST_F (NvGeConeTest, HeightAt_IsIdentityOnHeightParameter)
{
  const NvGeCone aCone;
  EXPECT_NEAR (aCone.heightAt (2.5), 2.5, THE_TEST_TOL);
  EXPECT_NEAR (aCone.heightAt (-1.0), -1.0, THE_TEST_TOL);
}

TEST_F (NvGeConeTest, SetBaseRadius_RoundTrip_PreservesTrim)
{
  const double aHalf = 0.35;
  NvGeCone aCone (std::cos (aHalf), std::sin (aHalf), NvGePoint3d (0.0, 0.0, 0.0),
                  2.0, NvGeVector3d (0.0, 0.0, 1.0), NvGeVector3d (1.0, 0.0, 0.0),
                  NvGeInterval (0.0, 4.0), 0.5, 2.0);
  aCone.setBaseRadius (6.0);
  EXPECT_NEAR (aCone.baseRadius(), 6.0, THE_TEST_TOL);

  double aStart = 0.0, anEnd = 0.0;
  aCone.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.5, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCone.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 4.0, THE_TEST_TOL);
}

TEST_F (NvGeConeTest, SetAngles_SetHeight_RoundTrip)
{
  const NvGeCone aSrc (std::cos (0.3), std::sin (0.3), NvGePoint3d (0.0, 0.0, 0.0),
                       2.0, NvGeVector3d (0.0, 0.0, 1.0));
  NvGeCone aCone;
  aCone.set (std::cos (0.3), std::sin (0.3), NvGePoint3d (0.0, 0.0, 0.0), 2.0,
             NvGeVector3d (0.0, 0.0, 1.0));
  EXPECT_NEAR (aCone.halfAngle(), aSrc.halfAngle(), THE_TEST_TOL);

  aCone.setAngles (0.25, 1.75);
  double aStart = 0.0, anEnd = 0.0;
  aCone.getAngles (aStart, anEnd);
  EXPECT_NEAR (aStart, 0.25, THE_TEST_TOL);
  EXPECT_NEAR (anEnd, 1.75, THE_TEST_TOL);
  EXPECT_FALSE (aCone.isClosed());

  aCone.setHeight (NvGeInterval (1.0, 7.0));
  NvGeInterval aHeight;
  aCone.getHeight (aHeight);
  double aLower = 0.0, anUpper = 0.0;
  aHeight.getBounds (aLower, anUpper);
  EXPECT_NEAR (aLower, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anUpper, 7.0, THE_TEST_TOL);
}

TEST_F (NvGeConeTest, SetBaseRadius_Negative_Throws)
{
  NvGeCone aCone;
  EXPECT_THROW (aCone.setBaseRadius (-1.0), NvException);
  EXPECT_THROW (NvGeCone (std::cos (0.3), std::sin (0.3), NvGePoint3d (0.0, 0.0, 0.0),
                          -2.0, NvGeVector3d (0.0, 0.0, 1.0)),
                NvException);
}

TEST_F (NvGeConeTest, InvalidHalfAngle_Throws)
{
  const NvGePoint3d anOrigin;
  const NvGeVector3d anAxis (0.0, 0.0, 1.0);
  // Sine/cosine outside the unit circle.
  EXPECT_THROW (NvGeCone (1.5, 0.5, anOrigin, 1.0, anAxis), NvException);
  EXPECT_THROW (NvGeCone (0.5, 1.5, anOrigin, 1.0, anAxis), NvException);
  // Half angle 0 (sine 0) and half angle PI/2 (cosine 0).
  EXPECT_THROW (NvGeCone (1.0, 0.0, anOrigin, 1.0, anAxis), NvException);
  EXPECT_THROW (NvGeCone (0.0, 1.0, anOrigin, 1.0, anAxis), NvException);
  // Half angle in ]PI/2, PI[ (negative cosine).
  EXPECT_THROW (NvGeCone (-0.5, 0.8660254037844386, anOrigin, 1.0, anAxis), NvException);
}

TEST_F (NvGeConeTest, SetAngles_SetHeight_Invalid_Throws)
{
  NvGeCone aCone;
  EXPECT_THROW (aCone.setAngles (1.0, 1.0), NvException);
  EXPECT_THROW (aCone.setHeight (NvGeInterval()), NvException); // unbounded
  EXPECT_THROW (aCone.setHeight (NvGeInterval (2.0, 2.0)), NvException);
}

TEST_F (NvGeConeTest, Copy_IsIndependent)
{
  NvGeCone aSrc (std::cos (0.3), std::sin (0.3), NvGePoint3d (1.0, 1.0, 1.0),
                 2.0, NvGeVector3d (0.0, 0.0, 1.0));
  const NvGeCone aCopy (aSrc);
  EXPECT_NEAR (aCopy.baseRadius(), 2.0, THE_TEST_TOL);

  // Copy-on-write: mutating the source must not touch the shared geometry.
  aSrc.setBaseRadius (5.0);
  aSrc.setHeight (NvGeInterval (0.0, 1.0));
  EXPECT_NEAR (aSrc.baseRadius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.baseRadius(), 2.0, THE_TEST_TOL);

  NvGeInterval aHeight;
  aCopy.getHeight (aHeight);
  EXPECT_TRUE (aHeight.isUnBounded());
}

TEST_F (NvGeConeTest, Assignment_IsIndependent)
{
  NvGeCone aSrc (std::cos (0.3), std::sin (0.3), NvGePoint3d (1.0, 1.0, 1.0),
                 2.0, NvGeVector3d (0.0, 0.0, 1.0));
  NvGeCone aAssigned;
  aAssigned = aSrc;
  aSrc.setBaseRadius (7.0);
  EXPECT_NEAR (aAssigned.baseRadius(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aAssigned.apex().z, 1.0 + 2.0 / std::tan (0.3), THE_TEST_TOL);
}

TEST_F (NvGeConeTest, IntersectWith_LinearEntity_ReturnsFalse)
{
  const NvGeCone aCone;
  int anIntn = -1;
  NvGePoint3d aPnt1, aPnt2;
  EXPECT_FALSE (aCone.intersectWith (NvGeLine3d(), anIntn, aPnt1, aPnt2));
  EXPECT_EQ (anIntn, 0);
}
