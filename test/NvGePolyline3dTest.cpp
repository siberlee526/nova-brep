#include <NvException.h>
#include <gearc3d.h>
#include <geline3d.h>
#include <gekvec.h>
#include <gepnt3d.h>
#include <gept3dar.h>
#include <geplin3d.h>
#include <gesent3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;

  NvGePoint3dArray MakePoints ()
  {
    NvGePoint3dArray aPoints;
    aPoints.append (NvGePoint3d (0.0, 0.0, 0.0));
    aPoints.append (NvGePoint3d (1.0, 0.0, 0.0));
    aPoints.append (NvGePoint3d (1.0, 1.0, 0.0));
    aPoints.append (NvGePoint3d (2.0, 1.0, 0.0));
    return aPoints;
  }
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGePolyline3dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGePolyline3dTest, PointsCtor_FitDataAndKnots)
{
  const NvGePolyline3d aPline (MakePoints());

  EXPECT_TRUE (aPline.isKindOf (NvGe::kPolyline3d));

  EXPECT_EQ (aPline.numFitPoints(), 4);
  EXPECT_TRUE (aPline.fitPointAt (1).isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.fitPointAt (2).isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.fitPointAt (3).isEqualTo (NvGePoint3d (1.0, 1.0, 0.0)));
  EXPECT_TRUE (aPline.fitPointAt (4).isEqualTo (NvGePoint3d (2.0, 1.0, 0.0)));

  EXPECT_EQ (aPline.degree(), 1);
  EXPECT_EQ (aPline.order(), 2);
  EXPECT_TRUE (aPline.hasFitData());
  EXPECT_EQ (aPline.numControlPoints(), 4);

  EXPECT_EQ (aPline.numKnots(), 4);
  EXPECT_NEAR (aPline.knotAt (1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (2), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (3), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (4), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.endParam(), 3.0, THE_TEST_TOL);
  EXPECT_TRUE (aPline.startPoint().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.endPoint().isEqualTo (NvGePoint3d (2.0, 1.0, 0.0)));

  // Degree 1 with multiplicities [2, 1, ..., 1, 2].
  EXPECT_EQ (aPline.continuityAtKnot (1), -1);
  EXPECT_EQ (aPline.continuityAtKnot (2), 0);
  EXPECT_EQ (aPline.continuityAtKnot (3), 0);
  EXPECT_EQ (aPline.continuityAtKnot (4), -1);
}

TEST_F (NvGePolyline3dTest, Accessors_IndexErrors_Throw)
{
  const NvGePolyline3d aPline (MakePoints());

  EXPECT_THROW (aPline.fitPointAt (0), NvException);
  EXPECT_THROW (aPline.fitPointAt (5), NvException);
  EXPECT_THROW (aPline.knotAt (0), NvException);
  EXPECT_THROW (aPline.knotAt (5), NvException);
  EXPECT_THROW (aPline.continuityAtKnot (0), NvException);
  EXPECT_THROW (aPline.continuityAtKnot (5), NvException);
}

TEST_F (NvGePolyline3dTest, PointsCtor_TooFewPoints_Throws)
{
  NvGePoint3dArray aSingle;
  aSingle.append (NvGePoint3d (0.0, 0.0, 0.0));

  // The temporary must be bound to a name: MSVC discards the constructor
  // call of an unused temporary of a dllimport class, so the exception
  // would never be raised.
  EXPECT_THROW (const NvGePolyline3d aPline (aSingle), NvException);
}

TEST_F (NvGePolyline3dTest, KnotCtor_GroupsFlatKnots)
{
  const double aRawKnots[] = { 0.0, 0.0, 1.0, 2.0, 3.0, 3.0 };
  const NvGeKnotVector aKnots (6, aRawKnots);
  const NvGePolyline3d aPline (aKnots, MakePoints());

  // Only adjacent equal raw knots group: the doubled ends clamp, while the
  // single interior knots 1 and 2 stay at multiplicity 1.
  ASSERT_EQ (aPline.numKnots(), 4);
  EXPECT_NEAR (aPline.knotAt (1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (2), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (3), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (4), 3.0, THE_TEST_TOL);

  // Multiplicities [2, 1, 1, 2]: continuity = degree - multiplicity, i.e.
  // clamped ends and single C0 breaks at the interior knots.
  EXPECT_EQ (aPline.continuityAtKnot (1), -1);
  EXPECT_EQ (aPline.continuityAtKnot (2), 0);
  EXPECT_EQ (aPline.continuityAtKnot (3), 0);
  EXPECT_EQ (aPline.continuityAtKnot (4), -1);

  EXPECT_NEAR (aPline.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.endParam(), 3.0, THE_TEST_TOL);
  EXPECT_TRUE (aPline.startPoint().isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.endPoint().isEqualTo (NvGePoint3d (2.0, 1.0, 0.0)));
}

TEST_F (NvGePolyline3dTest, KnotCtor_InvalidVectors_Throw)
{
  // An interior multiplicity of 2 exceeds the degree.
  const double anInteriorMult[] = { 0.0, 0.0, 1.0, 1.0, 2.0, 2.0 };
  EXPECT_THROW (NvGePolyline3d (NvGeKnotVector (6, anInteriorMult), MakePoints()),
                NvException);

  // The multiplicities do not add up to the control point count plus two.
  const double aWrongSum[] = { 0.0, 1.0, 2.0 };
  EXPECT_THROW (NvGePolyline3d (NvGeKnotVector (3, aWrongSum), MakePoints()),
                NvException);

  // The knot values must be non-descending.
  const double aDescending[] = { 0.0, 1.0, 0.0, 2.0, 3.0, 3.0 };
  EXPECT_THROW (NvGePolyline3d (NvGeKnotVector (6, aDescending), MakePoints()),
                NvException);
}

TEST_F (NvGePolyline3dTest, SetFitPointAt_ModifiesPrivateCopyOnly)
{
  const NvGePolyline3d aPline (MakePoints());
  const NvGePolyline3d aCopy (aPline);

  NvGePolyline3d aMutable (aPline);
  aMutable.setFitPointAt (2, NvGePoint3d (5.0, 5.0, 5.0));

  // fitPointAt is 1-based: the untouched originals hold the second fit
  // point of MakePoints(), i.e. (1, 0, 0).
  EXPECT_TRUE (aPline.fitPointAt (2).isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aCopy.fitPointAt (2).isEqualTo (NvGePoint3d (1.0, 0.0, 0.0)));
  EXPECT_TRUE (aMutable.fitPointAt (2).isEqualTo (NvGePoint3d (5.0, 5.0, 5.0)));

  EXPECT_THROW (aMutable.setFitPointAt (0, NvGePoint3d (0.0, 0.0, 0.0)), NvException);
  EXPECT_THROW (aMutable.setFitPointAt (5, NvGePoint3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGePolyline3dTest, SetKnotAt_MovesKnotGuardsOrderAndCopies)
{
  const NvGePolyline3d aPline (MakePoints());
  const NvGePolyline3d aCopy (aPline);

  NvGePolyline3d aMutable (aPline);
  aMutable.setKnotAt (2, 1.5);

  EXPECT_NEAR (aMutable.knotAt (2), 1.5, THE_TEST_TOL);
  EXPECT_NEAR (aMutable.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aMutable.endParam(), 3.0, THE_TEST_TOL);
  // The private copy still holds the original knots: knotAt is 1-based and
  // the second knot of MakePoints() is 1.0.
  EXPECT_NEAR (aCopy.knotAt (2), 1.0, THE_TEST_TOL);

  // Moving the knot past its neighbor is rejected and leaves the curve.
  EXPECT_THROW (aMutable.setKnotAt (2, 5.0), NvException);
  EXPECT_NEAR (aMutable.knotAt (2), 1.5, THE_TEST_TOL);

  EXPECT_THROW (aMutable.setKnotAt (0, 0.5), NvException);
  EXPECT_THROW (aMutable.setKnotAt (5, 0.5), NvException);
}

TEST_F (NvGePolyline3dTest, SetControlPointAt_MovesPole)
{
  NvGePolyline3d aPline (MakePoints());

  aPline.setControlPointAt (3, NvGePoint3d (0.0, 5.0, 0.0));

  EXPECT_TRUE (aPline.fitPointAt (3).isEqualTo (NvGePoint3d (0.0, 5.0, 0.0)));
  EXPECT_THROW (aPline.setControlPointAt (0, NvGePoint3d (0.0, 0.0, 0.0)), NvException);
  EXPECT_THROW (aPline.setControlPointAt (5, NvGePoint3d (0.0, 0.0, 0.0)), NvException);
}

TEST_F (NvGePolyline3dTest, DefaultCtor_DegenerateSegmentAtOrigin)
{
  const NvGePolyline3d aPline;

  EXPECT_TRUE (aPline.isKindOf (NvGe::kPolyline3d));
  EXPECT_EQ (aPline.numFitPoints(), 2);
  EXPECT_TRUE (aPline.fitPointAt (1).isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.fitPointAt (2).isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_EQ (aPline.degree(), 1);
  EXPECT_TRUE (aPline.hasFitData());
  EXPECT_EQ (aPline.numKnots(), 2);
  EXPECT_NEAR (aPline.knotAt (1), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.knotAt (2), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPline.endParam(), 1.0, THE_TEST_TOL);
}

TEST_F (NvGePolyline3dTest, ApproximateCircle_ChordHeightSampling)
{
  const NvGeCircArc3d anArc (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
                             NvGeVector3d (1.0, 0.0, 0.0), 2.0, 0.0, THE_PI / 2.0);

  const NvGePolyline3d aPline (anArc, 1e-3);

  EXPECT_TRUE (aPline.isKindOf (NvGe::kPolyline3d));
  EXPECT_TRUE (aPline.hasFitData());
  ASSERT_GE (aPline.numFitPoints(), 2);
  EXPECT_TRUE (aPline.fitPointAt (1).isEqualTo (NvGePoint3d (2.0, 0.0, 0.0)));

  // Every fit point is a sample of the arc, hence on the carrier circle.
  const int aNbCoarse = aPline.numFitPoints();
  for (int aFit = 1; aFit <= aNbCoarse; ++aFit)
  {
    const NvGePoint3d aPnt = aPline.fitPointAt (aFit);
    EXPECT_NEAR (std::sqrt (aPnt.x * aPnt.x + aPnt.y * aPnt.y + aPnt.z * aPnt.z),
                 2.0, THE_TEST_TOL);
  }
  EXPECT_TRUE (aPline.fitPointAt (aNbCoarse).isEqualTo (NvGePoint3d (0.0, 2.0, 0.0)));

  // A finer tolerance subdivides at least as much.
  const NvGePolyline3d aFiner (anArc, 1e-5);
  EXPECT_GE (aFiner.numFitPoints(), aNbCoarse);

  EXPECT_THROW (NvGePolyline3d (anArc, 0.0), NvException);
  EXPECT_THROW (NvGePolyline3d (anArc, -1e-3), NvException);
}

TEST_F (NvGePolyline3dTest, ApproximatePolyline_ReproducesFitPointsExactly)
{
  const NvGePolyline3d aSource (MakePoints());

  const NvGePolyline3d aCopy (aSource, 1e-4);

  ASSERT_EQ (aCopy.numFitPoints(), 4);
  for (int aFit = 1; aFit <= 4; ++aFit)
  {
    EXPECT_TRUE (aCopy.fitPointAt (aFit).isEqualTo (aSource.fitPointAt (aFit)));
  }
}

TEST_F (NvGePolyline3dTest, ApproximateSpline_ReproducesSegment)
{
  NvGePolyline3d aSpline;
  aSpline.setControlPointAt (1, NvGePoint3d (0.0, 0.0, 0.0));
  aSpline.setControlPointAt (2, NvGePoint3d (3.0, 0.0, 0.0));

  const NvGePolyline3d aPline (aSpline, 1e-4);

  ASSERT_EQ (aPline.numFitPoints(), 2);
  EXPECT_TRUE (aPline.fitPointAt (1).isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.fitPointAt (2).isEqualTo (NvGePoint3d (3.0, 0.0, 0.0)));
}

TEST_F (NvGePolyline3dTest, Approximate_UnsupportedSource_Throws)
{
  const NvGeLine3d aLine (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0));

  EXPECT_THROW (NvGePolyline3d (aLine, 1e-3), NvException);
}

TEST_F (NvGePolyline3dTest, CopyOnWrite_AssignThenModifyCopy_OriginalUnaffected)
{
  const NvGePolyline3d aPline (MakePoints());
  NvGePolyline3d aCopy;
  aCopy = aPline;

  aCopy.setFitPointAt (1, NvGePoint3d (9.0, 9.0, 9.0));

  EXPECT_TRUE (aPline.fitPointAt (1).isEqualTo (NvGePoint3d (0.0, 0.0, 0.0)));
  EXPECT_TRUE (aPline.numFitPoints() == 4);
  EXPECT_TRUE (aCopy.fitPointAt (1).isEqualTo (NvGePoint3d (9.0, 9.0, 9.0)));
}
