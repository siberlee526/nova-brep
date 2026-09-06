#include <NvException.h>
#include <gecspl3d.h>
#include <gegbl.h>
#include <gekvec.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;

  void ExpectPointNear (const NvGePoint3d& theActual, double theX, double theY, double theZ, double theTol)
  {
    EXPECT_NEAR (theActual.x, theX, theTol);
    EXPECT_NEAR (theActual.y, theY, theTol);
    EXPECT_NEAR (theActual.z, theZ, theTol);
  }

  void ExpectVectorNear (const NvGeVector3d& theActual, double theX, double theY, double theZ, double theTol)
  {
    EXPECT_NEAR (theActual.x, theX, theTol);
    EXPECT_NEAR (theActual.y, theY, theTol);
    EXPECT_NEAR (theActual.z, theZ, theTol);
  }

  NvGePoint3dArray MakeOpenPoints()
  {
    NvGePoint3dArray aPnts;
    aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
    aPnts.append (NvGePoint3d (1.0, 0.0, 0.0));
    aPnts.append (NvGePoint3d (1.0, 2.0, 0.0));
    aPnts.append (NvGePoint3d (3.0, 2.0, 0.0));
    return aPnts;
  }

  NvGeKnotVector MakeUniformKnots (int theCount)
  {
    NvGeDoubleArray aValues;
    for (int i = 0; i < theCount; ++i)
    {
      aValues.append (i * 1.0);
    }
    return NvGeKnotVector (aValues);
  }
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeCubicSpline3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeCubicSpline3dTest, DefaultConstructor_IsDegenerateAtOrigin)
{
  const NvGeCubicSplineCurve3d aSpline;

  ExpectPointNear (aSpline.startPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.endPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
  EXPECT_EQ (aSpline.degree(), 1);
}

TEST_F (NvGeCubicSpline3dTest, ClampedConstructor_PassesThroughFitPoints)
{
  const NvGeCubicSplineCurve3d aSpline (MakeOpenPoints(),
                                        NvGeVector3d (1.0, 0.0, 0.0),
                                        NvGeVector3d (1.0, 0.0, 0.0));

  EXPECT_EQ (aSpline.numFitPoints(), 4);

  for (int i = 0; i < 4; ++i)
  {
    ExpectPointNear (aSpline.fitPointAt (i), i == 0 ? 0.0 : i == 1 ? 1.0 : i == 2 ? 1.0 : 3.0,
                     i < 2 ? 0.0 : 2.0, 0.0, THE_TEST_TOL);
  }

  ExpectVectorNear (aSpline.firstDerivAt (0), 1.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectVectorNear (aSpline.firstDerivAt (3), 1.0, 0.0, 0.0, THE_TEST_TOL);

  ExpectPointNear (aSpline.startPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.endPoint(), 3.0, 2.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCubicSpline3dTest, ClampedConstructor_TooFewPoints_Throws)
{
  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));

  EXPECT_THROW (NvGeCubicSplineCurve3d (aPnts, NvGeVector3d (1.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0)),
                NvException);
}

TEST_F (NvGeCubicSpline3dTest, HermiteConstructor_PassesThroughPointsWithDerivs)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (4);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 1.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 1.0, 0.0));
  aPnts.append (NvGePoint3d (3.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));

  const NvGeCubicSplineCurve3d aSpline (aKnots, aPnts, aDerivs);

  EXPECT_EQ (aSpline.numFitPoints(), 4);
  for (int i = 0; i < 4; ++i)
  {
    ExpectPointNear (aSpline.fitPointAt (i), aPnts[i].x, aPnts[i].y, aPnts[i].z, THE_TEST_TOL);
    ExpectVectorNear (aSpline.firstDerivAt (i), 1.0, 0.0, 0.0, THE_TEST_TOL);

    // One parameter per fit point: the curve meets the point at its own knot.
    ExpectPointNear (aSpline.evalPoint (i * 1.0), aPnts[i].x, aPnts[i].y, aPnts[i].z, 1e-6);
  }
}

TEST_F (NvGeCubicSpline3dTest, HermiteConstructor_CountMismatch_Throws)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (4);

  NvGePoint3dArray aPnts;
  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 4; ++i)
  {
    aPnts.append (NvGePoint3d (i * 1.0, 0.0, 0.0));
  }
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));

  EXPECT_THROW (NvGeCubicSplineCurve3d (aKnots, aPnts, aDerivs), NvException);
}

TEST_F (NvGeCubicSpline3dTest, PeriodicConstructor_ClosesCurve)
{
  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 2.0, 0.0));
  aPnts.append (NvGePoint3d (0.0, 2.0, 0.0));
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));

  const NvGeCubicSplineCurve3d aSpline (aPnts);

  EXPECT_EQ (aSpline.numFitPoints(), 5);
  ExpectPointNear (aSpline.fitPointAt (0), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.fitPointAt (4), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.startPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.endPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCubicSpline3dTest, PeriodicConstructor_FirstLastMismatch_Throws)
{
  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 2.0, 0.0));
  aPnts.append (NvGePoint3d (0.0, 2.0, 0.0));

  EXPECT_THROW (NvGeCubicSplineCurve3d {aPnts}, NvException);
}

TEST_F (NvGeCubicSpline3dTest, ApproximationConstructor_FollowsSource)
{
  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 0.1, 0.0));
  aPnts.append (NvGePoint3d (2.0, -0.1, 0.0));
  aPnts.append (NvGePoint3d (3.0, 0.0, 0.0));

  const NvGeCubicSplineCurve3d aSource (aPnts,
                                        NvGeVector3d (1.0, 0.0, 0.0),
                                        NvGeVector3d (1.0, 0.0, 0.0));

  const NvGeCubicSplineCurve3d anApprox (aSource, 1e-5);

  EXPECT_GE (anApprox.numFitPoints(), 2);
  ExpectPointNear (anApprox.startPoint(), 0.0, 0.0, 0.0, 1e-6);
  ExpectPointNear (anApprox.endPoint(), 3.0, 0.0, 0.0, 1e-6);

  // Both curves are chord-length parameterized over the same geometry, so a
  // mid-shape sample must land close to the source curve.
  const double aSourceMid = 0.5 * (aSource.startParam() + aSource.endParam());
  const double anApproxMid = 0.5 * (anApprox.startParam() + anApprox.endParam());
  const NvGePoint3d aSourcePnt = aSource.evalPoint (aSourceMid);
  const NvGePoint3d anApproxPnt = anApprox.evalPoint (anApproxMid);
  EXPECT_NEAR (anApproxPnt.x, aSourcePnt.x, 1e-2);
  EXPECT_NEAR (anApproxPnt.y, aSourcePnt.y, 1e-2);
  EXPECT_NEAR (anApproxPnt.z, aSourcePnt.z, 1e-2);
}

TEST_F (NvGeCubicSpline3dTest, SetFitPointAt_RebuildsThroughNewPoint)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (4);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 1.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 1.0, 0.0));
  aPnts.append (NvGePoint3d (3.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 4; ++i)
  {
    aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  }

  NvGeCubicSplineCurve3d aSpline (aKnots, aPnts, aDerivs);

  aSpline.setFitPointAt (1, NvGePoint3d (1.0, -1.0, 0.0));

  ExpectPointNear (aSpline.fitPointAt (1), 1.0, -1.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.evalPoint (1.0), 1.0, -1.0, 0.0, 1e-6);
  ExpectPointNear (aSpline.startPoint(), 0.0, 0.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCubicSpline3dTest, SetFirstDerivAt_ChangesTangentKeepsPoint)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (3);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 3; ++i)
  {
    aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  }

  NvGeCubicSplineCurve3d aSpline (aKnots, aPnts, aDerivs);

  aSpline.setFirstDerivAt (0, NvGeVector3d (0.0, 5.0, 0.0));

  ExpectVectorNear (aSpline.firstDerivAt (0), 0.0, 5.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSpline.evalPoint (0.0), 0.0, 0.0, 0.0, 1e-6);
}

TEST_F (NvGeCubicSpline3dTest, CopyOnWrite_CopyThenModifyOriginal_CopyUnaffected)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (3);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 3; ++i)
  {
    aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  }

  const NvGeCubicSplineCurve3d aSource (aKnots, aPnts, aDerivs);
  const NvGeCubicSplineCurve3d aCopy (aSource);

  NvGeCubicSplineCurve3d aMutable (aSource);
  aMutable.setFitPointAt (0, NvGePoint3d (0.0, 1.0, 0.0));

  ExpectPointNear (aCopy.fitPointAt (0), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aSource.fitPointAt (0), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aMutable.fitPointAt (0), 0.0, 1.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCubicSpline3dTest, OperatorAssign_IndependentAfterModify)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (3);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 3; ++i)
  {
    aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  }

  const NvGeCubicSplineCurve3d aSource (aKnots, aPnts, aDerivs);

  NvGeCubicSplineCurve3d aCopy;
  aCopy = aSource;

  aCopy.setFitPointAt (2, NvGePoint3d (2.0, 1.0, 0.0));

  ExpectPointNear (aSource.fitPointAt (2), 2.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aCopy.fitPointAt (2), 2.0, 1.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeCubicSpline3dTest, OutOfRangeAccess_Throws)
{
  const NvGeKnotVector aKnots = MakeUniformKnots (3);

  NvGePoint3dArray aPnts;
  aPnts.append (NvGePoint3d (0.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (1.0, 0.0, 0.0));
  aPnts.append (NvGePoint3d (2.0, 0.0, 0.0));

  NvGeVector3dArray aDerivs;
  for (int i = 0; i < 3; ++i)
  {
    aDerivs.append (NvGeVector3d (1.0, 0.0, 0.0));
  }

  const NvGeCubicSplineCurve3d aSpline (aKnots, aPnts, aDerivs);

  EXPECT_THROW (aSpline.fitPointAt (3), NvException);
  EXPECT_THROW (aSpline.firstDerivAt (-1), NvException);
}
