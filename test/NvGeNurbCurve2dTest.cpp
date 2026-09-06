#include <NvException.h>
#include <geell2d.h>
#include <gegbl.h>
#include <gelnsg2d.h>
#include <genurb2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <gtest/gtest.h>

#include <cmath>

namespace
{
  constexpr double THE_PI = 3.14159265358979323846;

  // Tolerance for exact analytic values.
  constexpr double THE_TEST_TOL = 1e-9;
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeNurbCurve2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeNurbCurve2dTest, DefaultConstructor_IsDegenerateAtOrigin)
{
  const NvGeNurbCurve2d aSpline;

  EXPECT_TRUE (aSpline.startPoint().isEqualTo (NvGePoint2d (0.0, 0.0)));
  EXPECT_TRUE (aSpline.endPoint().isEqualTo (NvGePoint2d (0.0, 0.0)));

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_EQ (aDegree, 1);
  EXPECT_FALSE (aRational != Adesk::kFalse);
}

TEST_F (NvGeNurbCurve2dTest, FitConstructor_PassesThroughFitPoints)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));

  const NvGeNurbCurve2d aSpline (aFitPnts,
                                 NvGeVector2d (1.0, 0.0), NvGeVector2d (1.0, 0.0),
                                 Adesk::kTrue, Adesk::kTrue);

  EXPECT_EQ (aSpline.numFitPoints(), 3);

  NvGePoint2d aPnt;
  EXPECT_TRUE (aSpline.getFitPointAt (0, aPnt));
  EXPECT_NEAR (aPnt.x, 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aPnt.y, 0.0, THE_TEST_TOL);
  EXPECT_TRUE (aSpline.getFitPointAt (2, aPnt));
  EXPECT_NEAR (aPnt.x, 2.0, THE_TEST_TOL);

  NvGeVector2d aStartTan;
  NvGeVector2d anEndTan;
  EXPECT_TRUE (aSpline.getFitTangents (aStartTan, anEndTan));
  EXPECT_NEAR (aStartTan.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anEndTan.x, 1.0, THE_TEST_TOL);

  NvGeTol aTol;
  EXPECT_TRUE (aSpline.getFitTolerance (aTol));
  EXPECT_NEAR (aTol.equalPoint(), 1e-8, THE_TEST_TOL);

  EXPECT_TRUE (aSpline.evalPoint (aSpline.startParam()).isEqualTo (NvGePoint2d (0.0, 0.0)));
  EXPECT_TRUE (aSpline.evalPoint (aSpline.endParam()).isEqualTo (NvGePoint2d (2.0, 0.0)));
}

TEST_F (NvGeNurbCurve2dTest, FitConstructor_TooFewPoints_Throws)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));

  EXPECT_THROW (NvGeNurbCurve2d {aFitPnts}, NvException);
}

TEST_F (NvGeNurbCurve2dTest, DefinitionConstructor_DefinitionRoundTrip)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 1.0, i % 2 == 0 ? 0.0 : 1.0));
  }

  const NvGeNurbCurve2d aSpline (3, aKnots, aPnts, Adesk::kFalse);

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector anOutKnots;
  NvGePoint2dArray anOutPnts;
  NvGeDoubleArray anOutWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, anOutKnots, anOutPnts, anOutWeights);
  EXPECT_EQ (aDegree, 3);
  EXPECT_FALSE (aRational != Adesk::kFalse);
  EXPECT_FALSE (aPeriodic != Adesk::kFalse);
  EXPECT_EQ (anOutKnots.length(), 9);       // flat knot vector
  EXPECT_EQ (anOutPnts.length(), 5);
  EXPECT_EQ (aSpline.numWeights(), 0);
}

TEST_F (NvGeNurbCurve2dTest, WeightsConstructor_IsRational)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 1.0, 0.0));
    aWeights.append (i == 2 ? 2.0 : 1.0);
  }

  const NvGeNurbCurve2d aSpline (3, aKnots, aPnts, aWeights, Adesk::kFalse);

  EXPECT_EQ (aSpline.numWeights(), 5);
  EXPECT_NEAR (aSpline.weightAt (2), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (aSpline.weightAt (0), 1.0, THE_TEST_TOL);
  EXPECT_THROW (aSpline.weightAt (5), NvException);
}

TEST_F (NvGeNurbCurve2dTest, LineSegConstructor_IsDegreeOneSegment)
{
  const NvGeNurbCurve2d aSpline (NvGeLineSeg2d (NvGePoint2d (0.0, 0.0), NvGePoint2d (4.0, 3.0)));

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_EQ (aDegree, 1);

  EXPECT_TRUE (aSpline.startPoint().isEqualTo (NvGePoint2d (0.0, 0.0)));
  EXPECT_TRUE (aSpline.endPoint().isEqualTo (NvGePoint2d (4.0, 3.0)));
}

TEST_F (NvGeNurbCurve2dTest, EllipseConstructor_IsRationalConic)
{
  NvGeEllipArc2d anEllipse;
  anEllipse.setCenter (NvGePoint2d (1.0, 1.0));
  anEllipse.setAxes (NvGeVector2d (1.0, 0.0), NvGeVector2d (0.0, 1.0));
  anEllipse.setMajorRadius (2.0);
  anEllipse.setMinorRadius (1.0);
  anEllipse.setAngles (0.0, THE_PI / 2.0);

  const NvGeNurbCurve2d aSpline (anEllipse);

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_EQ (aDegree, 2);
  EXPECT_TRUE (aRational != Adesk::kFalse);
  EXPECT_GT (aSpline.numWeights(), 0);

  // The spline parameterization reproduces the ellipse angles.
  const NvGePoint2d aStart = aSpline.evalPoint (0.0);
  EXPECT_NEAR (aStart.x, 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aStart.y, 1.0, THE_TEST_TOL);
  const NvGePoint2d anEnd = aSpline.evalPoint (THE_PI / 2.0);
  EXPECT_NEAR (anEnd.x, 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anEnd.y, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbCurve2dTest, SetFitPointAt_RebuildsFit)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));

  NvGeNurbCurve2d aSpline (aFitPnts);

  EXPECT_TRUE (aSpline.setFitPointAt (1, NvGePoint2d (1.0, -1.0)));

  NvGePoint2d aPnt;
  EXPECT_TRUE (aSpline.getFitPointAt (1, aPnt));
  EXPECT_NEAR (aPnt.y, -1.0, THE_TEST_TOL);
  EXPECT_TRUE (aSpline.evalPoint (aSpline.startParam()).isEqualTo (NvGePoint2d (0.0, 0.0)));
}

TEST_F (NvGeNurbCurve2dTest, AddAndDeleteFitPoint_ChangesFitCount)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));

  NvGeNurbCurve2d aSpline (aFitPnts);

  EXPECT_TRUE (aSpline.addFitPointAt (1, NvGePoint2d (0.5, 0.5)));
  EXPECT_EQ (aSpline.numFitPoints(), 4);

  EXPECT_TRUE (aSpline.deleteFitPointAt (1));
  EXPECT_EQ (aSpline.numFitPoints(), 3);
}

TEST_F (NvGeNurbCurve2dTest, PurgeAndBuildFitData_KeepsGeometry)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));

  NvGeNurbCurve2d aSpline (aFitPnts);
  const NvGePoint2d aStart = aSpline.startPoint();
  const NvGePoint2d anEnd = aSpline.endPoint();

  EXPECT_TRUE (aSpline.purgeFitData());
  EXPECT_EQ (aSpline.numFitPoints(), 0);

  EXPECT_TRUE (aSpline.buildFitData());
  EXPECT_GT (aSpline.numFitPoints(), 0);

  NvGePoint2d aPnt;
  EXPECT_TRUE (aSpline.getFitPointAt (0, aPnt));
  EXPECT_TRUE (aPnt.isEqualTo (aStart));

  EXPECT_TRUE (aSpline.purgeFitData());
  EXPECT_TRUE (aSpline.buildFitData (NvGe::kSqrtChord));
  EXPECT_TRUE (aSpline.endPoint().isEqualTo (anEnd));
}

TEST_F (NvGeNurbCurve2dTest, JoinWith_ExtendsInterval)
{
  const NvGeNurbCurve2d aLeft (NvGeLineSeg2d (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 0.0)));
  const NvGeNurbCurve2d aRight (NvGeLineSeg2d (NvGePoint2d (1.0, 0.0), NvGePoint2d (2.0, 0.0)));

  NvGeNurbCurve2d aJoined (aLeft);
  aJoined.joinWith (aRight);

  EXPECT_NEAR (aJoined.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aJoined.endParam(), 2.0, THE_TEST_TOL);
  EXPECT_TRUE (aJoined.endPoint().isEqualTo (NvGePoint2d (2.0, 0.0)));
  EXPECT_TRUE (aJoined.evalPoint (1.0).isEqualTo (NvGePoint2d (1.0, 0.0)));
}

TEST_F (NvGeNurbCurve2dTest, JoinWith_DetachedEnds_Throws)
{
  const NvGeNurbCurve2d aLeft (NvGeLineSeg2d (NvGePoint2d (0.0, 0.0), NvGePoint2d (1.0, 0.0)));
  const NvGeNurbCurve2d aRight (NvGeLineSeg2d (NvGePoint2d (5.0, 0.0), NvGePoint2d (6.0, 0.0)));

  NvGeNurbCurve2d aJoined (aLeft);
  EXPECT_THROW (aJoined.joinWith (aRight), NvException);
}

TEST_F (NvGeNurbCurve2dTest, HardTrimByParams_NarrowsInterval)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, 0.0));
  }

  NvGeNurbCurve2d aSpline (3, aKnots, aPnts, Adesk::kFalse);
  const NvGePoint2d aMidStart = aSpline.evalPoint (0.5);

  aSpline.hardTrimByParams (0.5, 1.5);

  EXPECT_NEAR (aSpline.startParam(), 0.5, THE_TEST_TOL);
  EXPECT_NEAR (aSpline.endParam(), 1.5, THE_TEST_TOL);
  EXPECT_TRUE (aSpline.startPoint().isEqualTo (aMidStart));
}

TEST_F (NvGeNurbCurve2dTest, MakeClosedThenOpen_RestoresPoleCount)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));

  NvGeNurbCurve2d aSpline (aFitPnts);

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  const int anOpenPoleCount = aPnts.length();

  aSpline.makeClosed();
  EXPECT_TRUE (aSpline.endPoint().isEqualTo (aSpline.startPoint()));

  aSpline.makeOpen();
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_EQ (aPnts.length(), anOpenPoleCount);
}

TEST_F (NvGeNurbCurve2dTest, MakePeriodicThenNonPeriodic_TogglesFlag)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));

  NvGeNurbCurve2d aSpline (aFitPnts);

  aSpline.makePeriodic();
  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_TRUE (aPeriodic != Adesk::kFalse);

  aSpline.makeNonPeriodic();
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_FALSE (aPeriodic != Adesk::kFalse);
}

TEST_F (NvGeNurbCurve2dTest, ElevateDegree_RaisesDegreeKeepsEnds)
{
  const NvGeNurbCurve2d aBase (NvGeLineSeg2d (NvGePoint2d (0.0, 0.0), NvGePoint2d (4.0, 3.0)));
  NvGeNurbCurve2d aSpline (aBase);

  aSpline.elevateDegree (2);

  int aDegree = 0;
  Adesk::Boolean aRational = Adesk::kFalse;
  Adesk::Boolean aPeriodic = Adesk::kFalse;
  NvGeKnotVector aKnots;
  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  aSpline.getDefinitionData (aDegree, aRational, aPeriodic, aKnots, aPnts, aWeights);
  EXPECT_EQ (aDegree, 3);
  EXPECT_TRUE (aSpline.startPoint().isEqualTo (NvGePoint2d (0.0, 0.0)));
  EXPECT_TRUE (aSpline.endPoint().isEqualTo (NvGePoint2d (4.0, 3.0)));
}

TEST_F (NvGeNurbCurve2dTest, AddKnot_PreservesShape)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, i % 2 == 0 ? 0.0 : 1.0));
  }

  NvGeNurbCurve2d aSpline (3, aKnots, aPnts, Adesk::kFalse);
  const NvGePoint2d aBefore1 = aSpline.evalPoint (0.3);
  const NvGePoint2d aBefore2 = aSpline.evalPoint (1.7);

  aSpline.addKnot (0.5);

  EXPECT_TRUE (aSpline.evalPoint (0.3).isEqualTo (aBefore1));
  EXPECT_TRUE (aSpline.evalPoint (1.7).isEqualTo (aBefore2));
}

TEST_F (NvGeNurbCurve2dTest, AddControlPointAt_RoundTrip)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, 0.0));
  }

  NvGeNurbCurve2d aSpline (3, aKnots, aPnts, Adesk::kFalse);
  ASSERT_EQ (aSpline.numControlPoints(), 5);

  // Knots strictly before 0.5: only knot 0 with multiplicity 4, so the new
  // pole lands at the 0-based index 4.
  EXPECT_TRUE (aSpline.addControlPointAt (0.5, NvGePoint2d (0.5, 1.0)));
  EXPECT_EQ (aSpline.numControlPoints(), 6);

  EXPECT_TRUE (aSpline.deleteControlPointAt (4));
  EXPECT_EQ (aSpline.numControlPoints(), 5);
}

TEST_F (NvGeNurbCurve2dTest, GetParamsOfC1Discontinuity_FindsC0Knot)
{
  // Degree 2 with an interior knot of multiplicity 2 (C0 joint) at 1.0.
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, 0.0));
  }

  const NvGeNurbCurve2d aSpline (2, aKnots, aPnts, Adesk::kFalse);

  NvGeDoubleArray aParams;
  EXPECT_TRUE (aSpline.getParamsOfC1Discontinuity (aParams));
  ASSERT_EQ (aParams.length(), 1);
  EXPECT_NEAR (aParams[0], 1.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbCurve2dTest, CopyOnWrite_CopyThenModifyOriginal_CopyUnaffected)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, 0.0));
    aWeights.append (1.0);
  }

  const NvGeNurbCurve2d aSource (3, aKnots, aPnts, aWeights, Adesk::kFalse);
  const NvGeNurbCurve2d aCopy (aSource);

  NvGeNurbCurve2d aMutable (aSource);
  aMutable.setWeightAt (0, 5.0);

  EXPECT_NEAR (aCopy.weightAt (0), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aSource.weightAt (0), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aMutable.weightAt (0), 5.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbCurve2dTest, OperatorAssign_IndependentAfterModify)
{
  NvGeDoubleArray aFlatKnots;
  const double aFlat[] = {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0};
  for (const double aKnot : aFlat)
  {
    aFlatKnots.append (aKnot);
  }
  const NvGeKnotVector aKnots (aFlatKnots);

  NvGePoint2dArray aPnts;
  NvGeDoubleArray aWeights;
  for (int i = 0; i < 5; ++i)
  {
    aPnts.append (NvGePoint2d (i * 0.5, 0.0));
    aWeights.append (1.0);
  }

  const NvGeNurbCurve2d aSource (3, aKnots, aPnts, aWeights, Adesk::kFalse);
  NvGeNurbCurve2d aCopy;
  aCopy = aSource;

  aCopy.setWeightAt (0, 5.0);

  EXPECT_NEAR (aSource.weightAt (0), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aCopy.weightAt (0), 5.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbCurve2dTest, EvalMode_TogglesOnFitSpline)
{
  NvGePoint2dArray aFitPnts;
  aFitPnts.append (NvGePoint2d (0.0, 0.0));
  aFitPnts.append (NvGePoint2d (1.0, 1.0));
  aFitPnts.append (NvGePoint2d (2.0, 0.0));

  NvGeNurbCurve2d aSpline (aFitPnts);
  EXPECT_FALSE (aSpline.evalMode() != Adesk::kFalse);

  aSpline.setEvalMode (Adesk::kTrue);
  EXPECT_TRUE (aSpline.evalMode() != Adesk::kFalse);
}
