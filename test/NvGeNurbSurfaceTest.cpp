#include <NvException.h>
#include <genurbsf.h>
#include <gegbl.h>
#include <gedblar.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <gept3dar.h>
#include <gekvec.h>
#include <getol.h>

#include <gtest/gtest.h>

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
}

// Pins the global tolerance: the ge layer defaults are not initialized yet,
// so tests must not depend on whatever NvGeTol() leaves in its array.
class NvGeNurbSurfaceTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeNurbSurfaceTest, DefaultConstructor_IsDegenerateAtOrigin)
{
  const NvGeNurbSurface aNurb;

  EXPECT_EQ (aNurb.degreeInU(), 1);
  EXPECT_EQ (aNurb.degreeInV(), 1);
  EXPECT_EQ (aNurb.numControlPointsInU(), 2);
  EXPECT_EQ (aNurb.numControlPointsInV(), 2);
  EXPECT_EQ (aNurb.numKnotsInU(), 4);
  EXPECT_EQ (aNurb.numKnotsInV(), 4);

  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.3, 0.7)), 0.0, 0.0, 0.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, BilinearConstructor_EvaluatesCorners)
{
  // Pole grid is U-major: controlPoints[u * numV + v].
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),   // (u=0, v=0)
    NvGePoint3d (0.0, 3.0, -1.0),  // (u=0, v=1)
    NvGePoint3d (2.0, 0.0, 1.0),   // (u=1, v=0)
    NvGePoint3d (2.0, 3.0, 2.0)    // (u=1, v=1)
  };
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  const NvGeNurbSurface aNurb (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                               aPoles, nullptr, aUKnots, aVKnots);

  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.0, 0.0)), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.0, 1.0)), 0.0, 3.0, -1.0, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (1.0, 0.0)), 2.0, 0.0, 1.0, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (1.0, 1.0)), 2.0, 3.0, 2.0, THE_TEST_TOL);

  // Bilinear center is the average of the four corners: z = (0 - 1 + 1 + 2) / 4.
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.5, 0.5)), 1.0, 1.5, 0.5, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, DefinitionRoundTrip_PreservesBilinearDefinition)
{
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),
    NvGePoint3d (0.0, 3.0, -1.0),
    NvGePoint3d (2.0, 0.0, 1.0),
    NvGePoint3d (2.0, 3.0, 2.0)
  };
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  const NvGeNurbSurface aNurb (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                               aPoles, nullptr, aUKnots, aVKnots);

  int aDegU = 0, aDegV = 0, aPropsU = 0, aPropsV = 0, aNumU = 0, aNumV = 0;
  NvGePoint3dArray anOutPoles;
  NvGeDoubleArray anOutWeights;
  NvGeKnotVector anOutUKnots;
  NvGeKnotVector anOutVKnots;
  aNurb.getDefinition (aDegU, aDegV, aPropsU, aPropsV, aNumU, aNumV,
                       anOutPoles, anOutWeights, anOutUKnots, anOutVKnots);

  EXPECT_EQ (aDegU, 1);
  EXPECT_EQ (aDegV, 1);
  EXPECT_EQ (aPropsU, NvGe::kOpen);
  EXPECT_EQ (aPropsV, NvGe::kOpen);
  EXPECT_EQ (aNumU, 2);
  EXPECT_EQ (aNumV, 2);

  // Flat poles come back U-major.
  ASSERT_EQ (anOutPoles.length(), 4);
  ExpectPointNear (anOutPoles[0], 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (anOutPoles[1], 0.0, 3.0, -1.0, THE_TEST_TOL);
  ExpectPointNear (anOutPoles[2], 2.0, 0.0, 1.0, THE_TEST_TOL);
  ExpectPointNear (anOutPoles[3], 2.0, 3.0, 2.0, THE_TEST_TOL);

  // Non-rational: the weights array stays empty.
  EXPECT_EQ (aNurb.getWeights (anOutWeights), Nova::kFalse);
  EXPECT_TRUE (anOutWeights.isEmpty());

  // Knot vectors come back flat (each knot repeated per multiplicity).
  ASSERT_EQ (anOutUKnots.length(), 4);
  EXPECT_NEAR (anOutUKnots[0], 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutUKnots[1], 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutUKnots[2], 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutUKnots[3], 1.0, THE_TEST_TOL);
  ASSERT_EQ (anOutVKnots.length(), 4);
  EXPECT_NEAR (anOutVKnots[0], 0.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutVKnots[3], 1.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, CopyCtorAndAssignment_IsolateCopiesOnSet)
{
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),
    NvGePoint3d (0.0, 3.0, -1.0),
    NvGePoint3d (2.0, 0.0, 1.0),
    NvGePoint3d (2.0, 3.0, 2.0)
  };
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  NvGeNurbSurface aSource (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                           aPoles, nullptr, aUKnots, aVKnots);
  const NvGeNurbSurface aCopy (aSource);
  NvGeNurbSurface anAssigned;
  anAssigned = aSource;

  // Rebuilding the source must detach its impl, leaving the copies alone.
  const NvGePoint3d aMoved[] =
  {
    NvGePoint3d (10.0, 0.0, 0.0),
    NvGePoint3d (10.0, 3.0, 0.0),
    NvGePoint3d (12.0, 0.0, 0.0),
    NvGePoint3d (12.0, 3.0, 0.0)
  };
  aSource.set (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2, aMoved, nullptr, aUKnots, aVKnots);

  ExpectPointNear (aSource.evalPoint (NvGePoint2d (1.0, 1.0)), 12.0, 3.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aCopy.evalPoint (NvGePoint2d (1.0, 1.0)), 2.0, 3.0, 2.0, THE_TEST_TOL);
  ExpectPointNear (anAssigned.evalPoint (NvGePoint2d (1.0, 1.0)), 2.0, 3.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, InvalidInput_Throws)
{
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),
    NvGePoint3d (0.0, 3.0, -1.0),
    NvGePoint3d (2.0, 0.0, 1.0),
    NvGePoint3d (2.0, 3.0, 2.0)
  };
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  // Degree zero.
  EXPECT_THROW (NvGeNurbSurface (0, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                                 aPoles, nullptr, aUKnots, aVKnots),
                NvException);

  // Single pole row.
  EXPECT_THROW (NvGeNurbSurface (1, 1, NvGe::kOpen, NvGe::kOpen, 1, 2,
                                 aPoles, nullptr, aUKnots, aVKnots),
                NvException);

  // A non-positive weight: the weights array carries no length, so a count
  // mismatch cannot be detected at the API boundary, but a zero weight must
  // always be rejected.
  const double aZeroWeights[] = {1.0, 2.0, 0.0, 3.0};
  EXPECT_THROW (NvGeNurbSurface (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                                 aPoles, aZeroWeights, aUKnots, aVKnots),
                NvException);

  // Under-multiplicated clamped ends: the pole count does not match the knots.
  const double aBadU[] = {0.0, 1.0};
  const NvGeKnotVector aBadUKnots (2, aBadU);
  EXPECT_THROW (NvGeNurbSurface (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                                 aPoles, nullptr, aBadUKnots, aVKnots),
                NvException);
}

TEST_F (NvGeNurbSurfaceTest, RationalWeights_RoundTrip)
{
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),
    NvGePoint3d (0.0, 3.0, -1.0),
    NvGePoint3d (2.0, 0.0, 1.0),
    NvGePoint3d (2.0, 3.0, 2.0)
  };
  const double aWeights[] = {1.0, 2.0, 2.0, 3.0};
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  const NvGeNurbSurface aNurb (1, 1, NvGe::kOpen, NvGe::kOpen, 2, 2,
                               aPoles, aWeights, aUKnots, aVKnots);

  EXPECT_EQ (aNurb.isRationalInU(), Nova::kTrue);
  EXPECT_EQ (aNurb.isRationalInV(), Nova::kTrue);

  NvGeDoubleArray anOutWeights;
  EXPECT_EQ (aNurb.getWeights (anOutWeights), Nova::kTrue);
  ASSERT_EQ (anOutWeights.length(), 4);
  EXPECT_NEAR (anOutWeights[0], 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutWeights[1], 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutWeights[2], 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anOutWeights[3], 3.0, THE_TEST_TOL);

  // Clamped corners stay on the control points whatever the weights are.
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.0, 1.0)), 0.0, 3.0, -1.0, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (1.0, 1.0)), 2.0, 3.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, RationalBitWithoutWeights_KeepsGeometry)
{
  // kRational without a weights array synthesizes unit weights: the surface
  // geometry must stay identical to the non-rational one.
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),
    NvGePoint3d (0.0, 3.0, -1.0),
    NvGePoint3d (2.0, 0.0, 1.0),
    NvGePoint3d (2.0, 3.0, 2.0)
  };
  const double aFlatU[] = {0.0, 0.0, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (4, aFlatU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  const NvGeNurbSurface aNurb (1, 1, NvGe::kRational, NvGe::kRational, 2, 2,
                               aPoles, nullptr, aUKnots, aVKnots);

  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.0, 0.0)), 0.0, 0.0, 0.0, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.5, 0.5)), 1.0, 1.5, 0.5, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (1.0, 1.0)), 2.0, 3.0, 2.0, THE_TEST_TOL);
}

TEST_F (NvGeNurbSurfaceTest, SingularityInU_FindsSeam)
{
  // Degree-1 grid with an interior U knot of full multiplicity: the surface
  // is only C0 across u = 0.5 (3 x 2 poles). Poles are the grid (u, v, u+v).
  const NvGePoint3d aPoles[] =
  {
    NvGePoint3d (0.0, 0.0, 0.0),   // (u=0, v=0)
    NvGePoint3d (0.0, 1.0, 1.0),   // (u=0, v=1)
    NvGePoint3d (1.0, 0.0, 1.0),   // (u=1, v=0)
    NvGePoint3d (1.0, 1.0, 2.0),   // (u=1, v=1)
    NvGePoint3d (2.0, 0.0, 2.0),   // (u=2, v=0)
    NvGePoint3d (2.0, 1.0, 3.0)    // (u=2, v=1)
  };
  const double aSeamU[] = {0.0, 0.0, 0.5, 1.0, 1.0};
  const double aFlatV[] = {0.0, 0.0, 1.0, 1.0};
  const NvGeKnotVector aUKnots (5, aSeamU);
  const NvGeKnotVector aVKnots (4, aFlatV);

  const NvGeNurbSurface aNurb (1, 1, NvGe::kOpen, NvGe::kOpen, 3, 2,
                               aPoles, nullptr, aUKnots, aVKnots);

  // The seam is the second distinct knot (0-based index 1); V has none.
  EXPECT_EQ (aNurb.singularityInU(), 1);
  EXPECT_EQ (aNurb.singularityInV(), -1);

  // The surface still interpolates its pole grid midway between knot 0 and
  // the seam: the midpoint of the first pole row pair. (At the seam u = 0.5
  // the middle pole row itself is exact.)
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.25, 0.0)), 0.5, 0.0, 0.5, THE_TEST_TOL);
  ExpectPointNear (aNurb.evalPoint (NvGePoint2d (0.25, 1.0)), 0.5, 1.0, 1.5, THE_TEST_TOL);
}
