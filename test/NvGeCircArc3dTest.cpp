#include <NvException.h>
#include <gearc3d.h>
#include <geline3d.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <geplin3d.h>
#include <gesent3d.h>
#include <getol.h>
#include <gevec3d.h>
#include <gtest/gtest.h>
#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;
  constexpr double THE_TWO_PI = 6.28318530717958647692;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeCircArc3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCircArc3dTest, DefaultCtor_UnitCircleAboutOrigin)
{
  const NvGeCircArc3d anArc;

  EXPECT_TRUE (anArc.isKindOf (NvGe::kCircArc3d));

  EXPECT_TRUE (anArc.center().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  const NvGeVector3d aNormal = anArc.normal();
  EXPECT_NEAR (aNormal.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aNormal.y, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aNormal.z, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_TWO_PI, THE_TEST_TOL);

  EXPECT_TRUE (anArc.isInside (NvGePoint3d (0.5, 0.0, 0.0)));
  EXPECT_FALSE (anArc.isInside (NvGePoint3d (2.0, 0.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, FullCircleCtor_CoversTwoPi)
{
  const NvGeCircArc3d anArc (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (0.0, 0.0, 1.0), 5.0);

  EXPECT_TRUE (anArc.center().isEqualTo (NvGePoint3d (1.0, 2.0, 3.0)));
  EXPECT_NEAR (anArc.radius(), 5.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_TWO_PI, THE_TEST_TOL);

  const NvGePoint3d aRefPnt (6.0, 2.0, 3.0);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (aRefPnt));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (aRefPnt));
}

TEST_F (NvGeCircArc3dTest, SixArgCtor_QuarterArcFromAngles)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);

  EXPECT_TRUE (anArc.refVec().isEqualTo (NvGeVector3d (1.0, 0.0, 0.0)));
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_PI / 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (0.0, 1.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, SixArgCtor_NegativeSweep_WrapsAcrossZero)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, -THE_PI / 2.0);

  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), 1.5 * THE_PI, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (0.0, -1.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, ThreePointCtor_CounterclockwiseThroughMidPoint)
{
  const NvGeCircArc3d anArc (NvGePoint3d (2.0, 0.0, 0.0), NvGePoint3d (0.0, 2.0, 0.0),
                             NvGePoint3d (-2.0, 0.0, 0.0));

  EXPECT_TRUE (anArc.center().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_NEAR (anArc.radius(), 2.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (2.0, 0.0, 0.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (-2.0, 0.0, 0.0)));
  // The +Z normal pins the counterclockwise direction through the mid point.
  EXPECT_TRUE (anArc.normal().isEqualTo (NvGeVector3d (0.0, 0.0, 1.0)));
}

TEST_F (NvGeCircArc3dTest, CtorDegenerateInput_Throws)
{
  EXPECT_THROW (NvGeCircArc3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 0.0),
                NvException);
  EXPECT_THROW (NvGeCircArc3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 0.0), 1.0),
                NvException);
  EXPECT_THROW (NvGeCircArc3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                               NvGeVector3d (0.0, 0.0, 5.0), 1.0, 0.0, 1.0),
                NvException);
  EXPECT_THROW (NvGeCircArc3d (NvGePoint3d (0.0, 0.0, 0.0), NvGePoint3d (1.0, 0.0, 0.0),
                               NvGePoint3d (2.0, 0.0, 0.0)),
                NvException);
}

TEST_F (NvGeCircArc3dTest, SetThreePoints_ErrorLeavesObjectUnchanged)
{
  NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                       NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);

  NvGeError anError = NvGe::kOk;
  anArc.set (NvGePoint3d (2.0, 0.0, 0.0), NvGePoint3d (2.0, 0.0, 0.0),
             NvGePoint3d (-2.0, 0.0, 0.0), anError);
  EXPECT_TRUE (anError == NvGe::kEqualArg1Arg2);

  anArc.set (NvGePoint3d (0.0, 0.0, 0.0), NvGePoint3d (1.0, 0.0, 0.0),
             NvGePoint3d (2.0, 0.0, 0.0), anError);
  EXPECT_TRUE (anError == NvGe::kLinearlyDependentArg1Arg2Arg3);

  // Both failed calls left the arc untouched.
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, SetAngles_RetrimsAndWraps)
{
  NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                       NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);

  anArc.setAngles (7.0 * THE_PI / 4.0, THE_PI / 4.0);

  EXPECT_NEAR (anArc.startAng(), 7.0 * THE_PI / 4.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (std::sqrt (0.5), -std::sqrt (0.5), 0.0)));
  // A quarter of a turn across zero.
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (std::sqrt (0.5), std::sqrt (0.5), 0.0)));

  anArc.setAngles (0.0, THE_TWO_PI);
  EXPECT_NEAR (anArc.startAng(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anArc.endAng(), THE_TWO_PI, THE_TEST_TOL);
}

TEST_F (NvGeCircArc3dTest, SetCenterSetRadius_KeepAnglesAndShape)
{
  NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                       NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);

  anArc.setCenter (NvGePoint3d (5.0, 5.0, 5.0));
  EXPECT_TRUE (anArc.center().isEqualTo (NvGePoint3d (5.0, 5.0, 5.0)));
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (6.0, 5.0, 5.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (5.0, 6.0, 5.0)));

  anArc.setRadius (3.0);
  EXPECT_NEAR (anArc.radius(), 3.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (8.0, 5.0, 5.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (5.0, 8.0, 5.0)));

  EXPECT_THROW (anArc.setRadius (0.0), NvException);
  EXPECT_THROW (anArc.setRadius (-1.0), NvException);
}

TEST_F (NvGeCircArc3dTest, SetAxes_RedefinesPlaneAndReference)
{
  NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                       NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);

  anArc.setAxes (NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));

  EXPECT_TRUE (anArc.normal().isEqualTo (NvGeVector3d (0.0, 1.0, 0.0)));
  EXPECT_TRUE (anArc.refVec().isEqualTo (NvGeVector3d (0.0, 0.0, 1.0)));
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (anArc.startPoint().isEqualTo (NvGePoint3d (0.0, 0.0, 1.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));

  EXPECT_THROW (anArc.setAxes (NvGeVector3d (0.0, 0.0, 1.0), NvGeVector3d (0.0, 0.0, 5.0)),
                NvException);
  EXPECT_THROW (anArc.setAxes (NvGeVector3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0)),
                NvException);
}

TEST_F (NvGeCircArc3dTest, IsInside_StrictInteriorOfCarrierCircle)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  EXPECT_TRUE (anArc.isInside (NvGePoint3d (0.5, 0.0, 0.0)));
  // The in-plane radial distance decides: a point off the plane still
  // projects into the interior.
  EXPECT_TRUE (anArc.isInside (NvGePoint3d (0.0, 0.0, 0.5)));
  EXPECT_FALSE (anArc.isInside (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_FALSE (anArc.isInside (NvGePoint3d (2.0, 0.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, TangentLine_FromExternalPoint)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  NvGeLine3d aLine;
  EXPECT_TRUE (anArc.tangent (NvGePoint3d (3.0, 0.0, 0.0), aLine));

  EXPECT_TRUE (aLine.pointOnLine().isEqualTo (NvGePoint3d (3.0, 0.0, 0.0)));
  // The touch point (1/3, sqrt(8)/3, 0) lies counterclockwise of the radial.
  const NvGeVector3d aDir = aLine.direction();
  EXPECT_NEAR (aDir.x, -2.0 * std::sqrt (2.0) / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.y, 1.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aDir.z, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCircArc3dTest, TangentLine_ClassifiesInteriorAndCarrierPoints)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  NvGeLine3d aLine;
  NvGeError anError = NvGe::kOk;

  EXPECT_FALSE (anArc.tangent (NvGePoint3d (0.5, 0.0, 0.0), aLine, NvGeContext::gTol, anError));
  EXPECT_TRUE (anError == NvGe::kArg1InsideThis);

  EXPECT_FALSE (anArc.tangent (NvGePoint3d (1.0, 0.0, 0.0), aLine, NvGeContext::gTol, anError));
  EXPECT_TRUE (anError == NvGe::kArg1OnThis);

  EXPECT_TRUE (anArc.tangent (NvGePoint3d (3.0, 0.0, 0.0), aLine, NvGeContext::gTol, anError));
  EXPECT_TRUE (anError == NvGe::kOk);
}

TEST_F (NvGeCircArc3dTest, IntersectWithLine_SecantMissAndTangency)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  const NvGeLine3d aSecant (NvGePoint3d (0.0, 0.5, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  int anIntN = 0;
  NvGePoint3d aP1;
  NvGePoint3d aP2;
  EXPECT_TRUE (anArc.intersectWith (aSecant, anIntN, aP1, aP2));
  ASSERT_EQ (anIntN, 2);
  EXPECT_TRUE (aP1.isEqualTo (NvGePoint3d (-std::sqrt (0.75), 0.5, 0.0)));
  EXPECT_TRUE (aP2.isEqualTo (NvGePoint3d (std::sqrt (0.75), 0.5, 0.0)));

  const NvGeLine3d aMiss (NvGePoint3d (0.0, 2.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_FALSE (anArc.intersectWith (aMiss, anIntN, aP1, aP2));
  EXPECT_EQ (anIntN, 0);

  const NvGeLine3d aTangent (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  EXPECT_TRUE (anArc.intersectWith (aTangent, anIntN, aP1, aP2));
  EXPECT_EQ (anIntN, 1);
  EXPECT_TRUE (aP1.isEqualTo (NvGePoint3d (0.0, 1.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, IntersectWithArc_PairCoincidentAndDisjointCarriers)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  const NvGeCircArc3d anOffset (NvGePoint3d (1.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);
  int anIntN = 0;
  NvGePoint3d aP1;
  NvGePoint3d aP2;
  EXPECT_TRUE (anArc.intersectWith (anOffset, anIntN, aP1, aP2));
  ASSERT_EQ (anIntN, 2);
  EXPECT_TRUE (aP1.isEqualTo (NvGePoint3d (0.5, std::sqrt (0.75), 0.0)));
  EXPECT_TRUE (aP2.isEqualTo (NvGePoint3d (0.5, -std::sqrt (0.75), 0.0)));

  const NvGeCircArc3d aTwin (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);
  EXPECT_FALSE (anArc.intersectWith (aTwin, anIntN, aP1, aP2));

  const NvGeCircArc3d aFar (NvGePoint3d (5.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);
  EXPECT_FALSE (anArc.intersectWith (aFar, anIntN, aP1, aP2));
}

TEST_F (NvGeCircArc3dTest, ProjIntersectWith_MapsLineAlongProjectionDirection)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_TWO_PI);

  // The vertical line x = 0.5 projects along (0, 1, 1) onto the circle
  // plane as the in-plane line x = 0.5.
  const NvGeLine3d aLine (NvGePoint3d (0.5, 0.0, 1.0), NvGeVector3d (0.0, 0.0, 1.0));
  int aNumInt = 0;
  NvGePoint3d anArcP1;
  NvGePoint3d anArcP2;
  NvGePoint3d aLineP1;
  NvGePoint3d aLineP2;
  EXPECT_TRUE (anArc.projIntersectWith (aLine, NvGeVector3d (0.0, 1.0, 1.0), aNumInt,
                                        anArcP1, anArcP2, aLineP1, aLineP2));
  EXPECT_EQ (aNumInt, 2);
  EXPECT_TRUE (anArcP1.isEqualTo (NvGePoint3d (0.5, std::sqrt (0.75), 0.0)));
  EXPECT_TRUE (aLineP1.isEqualTo (NvGePoint3d (0.5, 0.0, -std::sqrt (0.75))));
  EXPECT_TRUE (anArcP2.isEqualTo (NvGePoint3d (0.5, -std::sqrt (0.75), 0.0)));
  EXPECT_TRUE (aLineP2.isEqualTo (NvGePoint3d (0.5, 0.0, std::sqrt (0.75))));
}

TEST_F (NvGeCircArc3dTest, GetPlane_ReturnsCarrierPlane)
{
  const NvGeCircArc3d anArc (NvGePoint3d (1.0, 2.0, 3.0), NvGeVector3d (0.0, 0.0, 1.0), 2.0);

  NvGePlane aPlane;
  EXPECT_NO_THROW (anArc.getPlane (aPlane));
}

TEST_F (NvGeCircArc3dTest, CopyOnWrite_CopyThenModifyOriginal_CopyUnaffected)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);
  const NvGeCircArc3d aCopy (anArc);

  NvGeCircArc3d aMutable (anArc);
  aMutable.setRadius (5.0);
  aMutable.setAngles (0.0, THE_PI);

  EXPECT_NEAR (aCopy.radius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aCopy.endPoint().isEqualTo (NvGePoint3d (0.0, 1.0, 0.0)));
  EXPECT_NEAR (anArc.radius(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMutable.radius(), 5.0, THE_TEST_TOL);
  // Radius 5 with angles [0, pi]: the end point is
  // center + 5 * (cos (pi), sin (pi), 0) = (-5, 0, 0).
  EXPECT_TRUE (aMutable.endPoint().isEqualTo (NvGePoint3d (-5.0, 0.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, CopyOnWrite_AssignThenModifyCopy_OriginalUnaffected)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 1.0, 0.0, THE_PI / 2.0);
  NvGeCircArc3d aCopy;
  aCopy = anArc;

  aCopy.setCenter (NvGePoint3d (4.0, 4.0, 0.0));

  EXPECT_TRUE (anArc.center().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (anArc.endPoint().isEqualTo (NvGePoint3d (0.0, 1.0, 0.0)));
  EXPECT_TRUE (aCopy.center().isEqualTo (NvGePoint3d (4.0, 4.0, 0.0)));
  EXPECT_TRUE (aCopy.endPoint().isEqualTo (NvGePoint3d (4.0, 5.0, 0.0)));
}

namespace
{

NvGeLine3d LineAlongX ()
{
  return NvGeLine3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
}

NvGeLine3d LineAlongY ()
{
  return NvGeLine3d (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));
}

}

TEST_F (NvGeCircArc3dTest, FilletTwoLines_TangencyNearestSeeds)
{
  const NvGeLine3d aLine1 = LineAlongX();
  const NvGeLine3d aLine2 = LineAlongY();

  NvGeCircArc3d aFillet;
  double aParam1 = 1.0;
  double aParam2 = 1.0;
  Nova::Boolean aSuccess = Nova::kFalse;
  aFillet.set (aLine1, aLine2, 1.0, aParam1, aParam2, aSuccess);

  EXPECT_TRUE (aSuccess);
  EXPECT_NEAR (aParam1, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aFillet.radius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aFillet.center().isEqualTo (NvGePoint3d (1.0, 1.0, 0.0)));
  EXPECT_TRUE (aFillet.startPoint().isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aFillet.endPoint().isEqualTo (NvGePoint3d (0.0, 1.0, 0.0)));
}

TEST_F (NvGeCircArc3dTest, FilletLineAndCircle_TangencyNearestSeeds)
{
  const NvGeLine3d aLine = LineAlongX();
  const NvGeCircArc3d aCircle (NvGePoint3d (2.0, 2.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0), 1.0);

  NvGeCircArc3d aFillet;
  double aParam1 = 5.0;
  double aParam2 = 0.0;
  Nova::Boolean aSuccess = Nova::kFalse;
  aFillet.set (aLine, aCircle, 1.0, aParam1, aParam2, aSuccess);

  EXPECT_TRUE (aSuccess);
  const double anExpectedParam1 = 2.0 + std::sqrt (3.0);
  EXPECT_NEAR (aParam1, anExpectedParam1, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, 11.0 * THE_PI / 6.0, THE_TEST_TOL);
  EXPECT_NEAR (aFillet.radius(), 1.0, THE_TEST_TOL);
  EXPECT_TRUE (aFillet.center().isEqualTo (NvGePoint3d (anExpectedParam1, 1.0, 0.0)));
  EXPECT_TRUE (aFillet.startPoint().isEqualTo (NvGePoint3d (anExpectedParam1, 0.0, 0.0)));
  EXPECT_TRUE (aFillet.endPoint().isEqualTo (NvGePoint3d (2.0 + std::sqrt (3.0) / 2.0, 1.5, 0.0)));
}

TEST_F (NvGeCircArc3dTest, FilletThreeLines_IncircleOfTriangle)
{
  // The carriers x = 0, y = 0 and x + y = 2 bound a right triangle whose
  // incircle has the center (2 - sqrt(2), 2 - sqrt(2)) and the radius
  // 2 - sqrt(2).
  const NvGeLine3d aLine1 = LineAlongX();
  const NvGeLine3d aLine2 = LineAlongY();
  const NvGeLine3d aLine3 (NvGePoint3d (2.0, 0.0, 0.0),
                           NvGeVector3d (-1.0, 1.0, 0.0));

  NvGeCircArc3d aFillet;
  double aParam1 = 1.0;
  double aParam2 = 1.0;
  double aParam3 = 1.0;
  Nova::Boolean aSuccess = Nova::kFalse;
  aFillet.set (aLine1, aLine2, aLine3, aParam1, aParam2, aParam3, aSuccess);

  EXPECT_TRUE (aSuccess);
  const double anInradius = 2.0 - std::sqrt (2.0);
  EXPECT_NEAR (aParam1, anInradius, THE_TEST_TOL);
  EXPECT_NEAR (aParam2, anInradius, THE_TEST_TOL);
  EXPECT_NEAR (aParam3, std::sqrt (2.0), THE_TEST_TOL);
  EXPECT_NEAR (aFillet.radius(), anInradius, THE_TEST_TOL);
  EXPECT_TRUE (aFillet.center().isEqualTo (NvGePoint3d (anInradius, anInradius, 0.0)));
  EXPECT_TRUE (aFillet.startPoint().isEqualTo (NvGePoint3d (anInradius, 0.0, 0.0)));
  EXPECT_TRUE (aFillet.endPoint().isEqualTo (NvGePoint3d (0.0, anInradius, 0.0)));
}

TEST_F (NvGeCircArc3dTest, Fillet_UnsupportedOrParallelCarriers_ReportFailure)
{
  const NvGeLine3d aLine1 = LineAlongX();
  const NvGeLine3d aParallel (NvGePoint3d (0.0, 1.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));
  const NvGePolyline3d aSpline;

  NvGeCircArc3d aFillet;
  double aParam1 = 1.0;
  double aParam2 = 1.0;
  Nova::Boolean aSuccess = Nova::kFalse;

  aFillet.set (aLine1, aParallel, 1.0, aParam1, aParam2, aSuccess);
  EXPECT_FALSE (aSuccess);

  aSuccess = Nova::kFalse;
  aFillet.set (aLine1, aSpline, 1.0, aParam1, aParam2, aSuccess);
  EXPECT_FALSE (aSuccess);
}
