#include <geclip2d.h>
#include <gepnt2d.h>
#include <gept2dar.h>
#include <gegbl.h>
#include <gegblge.h>

#include <gtest/gtest.h>

class NvGeClipBoundary2dTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeContext::gTol.setEqualPoint (1e-8);
    NvGeContext::gTol.setEqualVector (1e-8);
  }
};

TEST_F (NvGeClipBoundary2dTest, ClipPolygon_FullyInside)
{
  NvGeClipBoundary2d aClipper (NvGePoint2d (0.0, 0.0), NvGePoint2d (10.0, 10.0));

  NvGePoint2dArray aPoly;
  aPoly.append (NvGePoint2d (2.0, 2.0));
  aPoly.append (NvGePoint2d (4.0, 2.0));
  aPoly.append (NvGePoint2d (4.0, 4.0));
  aPoly.append (NvGePoint2d (2.0, 4.0));

  NvGePoint2dArray aClipped;
  NvGe::ClipCondition aCondition = NvGe::kInvalid;
  EXPECT_EQ (aClipper.clipPolygon (aPoly, aClipped, aCondition), NvGe::eOk);
  EXPECT_EQ (aCondition, NvGe::kAllSegmentsInside);
  EXPECT_EQ (aClipped.length(), 4);
}

TEST_F (NvGeClipBoundary2dTest, ClipPolygon_FullyOutside)
{
  NvGeClipBoundary2d aClipper (NvGePoint2d (0.0, 0.0), NvGePoint2d (10.0, 10.0));

  NvGePoint2dArray aPoly;
  aPoly.append (NvGePoint2d (20.0, 20.0));
  aPoly.append (NvGePoint2d (22.0, 20.0));
  aPoly.append (NvGePoint2d (22.0, 22.0));

  NvGePoint2dArray aClipped;
  NvGe::ClipCondition aCondition = NvGe::kInvalid;
  EXPECT_EQ (aClipper.clipPolygon (aPoly, aClipped, aCondition), NvGe::eOk);
  EXPECT_EQ (aCondition, NvGe::kAllSegmentsOutsideZeroWinds);
  EXPECT_EQ (aClipped.length(), 0);
}

TEST_F (NvGeClipBoundary2dTest, ClipPolygon_CrossingBoundaryIsCut)
{
  NvGeClipBoundary2d aClipper (NvGePoint2d (0.0, 0.0), NvGePoint2d (10.0, 10.0));

  // Square half outside the boundary: [5..15] x [5..15] -> clipped to
  // [5..10] x [5..10] with two cut corners.
  NvGePoint2dArray aPoly;
  aPoly.append (NvGePoint2d (5.0, 5.0));
  aPoly.append (NvGePoint2d (15.0, 5.0));
  aPoly.append (NvGePoint2d (15.0, 15.0));
  aPoly.append (NvGePoint2d (5.0, 15.0));

  NvGePoint2dArray aClipped;
  NvGe::ClipCondition aCondition = NvGe::kInvalid;
  EXPECT_EQ (aClipper.clipPolygon (aPoly, aClipped, aCondition), NvGe::eOk);
  EXPECT_EQ (aCondition, NvGe::kSegmentsIntersect);
  EXPECT_GE (aClipped.length(), 3);
  for (int anIdx = 0; anIdx < aClipped.length(); ++anIdx)
  {
    EXPECT_LE (aClipped[anIdx].x, 10.0 + 1e-9);
    EXPECT_LE (aClipped[anIdx].y, 10.0 + 1e-9);
  }
}

TEST_F (NvGeClipBoundary2dTest, ClipPolyline_SegmentCutAtBoundary)
{
  NvGeClipBoundary2d aClipper (NvGePoint2d (0.0, 0.0), NvGePoint2d (10.0, 10.0));

  NvGePoint2dArray aLine;
  aLine.append (NvGePoint2d (5.0, 5.0));
  aLine.append (NvGePoint2d (15.0, 5.0));

  NvGePoint2dArray aClipped;
  NvGe::ClipCondition aCondition = NvGe::kInvalid;
  EXPECT_EQ (aClipper.clipPolyline (aLine, aClipped, aCondition), NvGe::eOk);
  EXPECT_EQ (aCondition, NvGe::kSegmentsIntersect);
  ASSERT_GE (aClipped.length(), 2);
  EXPECT_NEAR (aClipped[0].x, 5.0, 1e-9);
  EXPECT_NEAR (aClipped[aClipped.length() - 1].x, 10.0, 1e-9);
  EXPECT_NEAR (aClipped[aClipped.length() - 1].y, 5.0, 1e-9);
}

TEST_F (NvGeClipBoundary2dTest, Set_InvalidPolygonReturnsError)
{
  NvGeClipBoundary2d aClipper;
  NvGePoint2dArray aDegenerate;
  aDegenerate.append (NvGePoint2d (0.0, 0.0));
  aDegenerate.append (NvGePoint2d (1.0, 0.0));
  EXPECT_EQ (aClipper.set (aDegenerate), NvGe::eInvalidClipBoundary);

  // Concave (non-convex) polygon is rejected as well.
  NvGePoint2dArray aConcave;
  aConcave.append (NvGePoint2d (0.0, 0.0));
  aConcave.append (NvGePoint2d (4.0, 0.0));
  aConcave.append (NvGePoint2d (1.0, 1.0));
  aConcave.append (NvGePoint2d (4.0, 4.0));
  aConcave.append (NvGePoint2d (0.0, 4.0));
  EXPECT_EQ (aClipper.set (aConcave), NvGe::eInvalidClipBoundary);
}
