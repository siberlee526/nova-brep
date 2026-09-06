#include <NvBoolean.h>
#include <NvException.h>
#include <NvPrim.h>

#include <gtest/gtest.h>

namespace
{
  // Two unit-tenth cubes: box1 = [0,10]^3, box2 = [5,15] x [0,10] x [0,10].
  // Overlap = 5 * 10 * 10 = 500.
  constexpr double THE_OVERLAP  = 500.0;
  constexpr double THE_TOLERANCE = 1e-9;
}

TEST(NvBooleanTest, Fuse_TwoOverlappingBoxes)
{
  const NvShape aBox1 = NvPrim::Box (10.0, 10.0, 10.0);
  const NvShape aBox2 = NvPrim::Box (gp_Pnt (5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

  const NvShape aFused = NvBoolean::Fuse (aBox1, aBox2);
  EXPECT_TRUE (aFused.IsValid());
  EXPECT_NEAR (aFused.Volume(), 2000.0 - THE_OVERLAP, THE_TOLERANCE);
}

TEST(NvBooleanTest, Cut_TwoOverlappingBoxes)
{
  const NvShape aBox1 = NvPrim::Box (10.0, 10.0, 10.0);
  const NvShape aBox2 = NvPrim::Box (gp_Pnt (5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

  const NvShape aCut = NvBoolean::Cut (aBox1, aBox2);
  EXPECT_TRUE (aCut.IsValid());
  EXPECT_NEAR (aCut.Volume(), 1000.0 - THE_OVERLAP, THE_TOLERANCE);
}

TEST(NvBooleanTest, Common_TwoOverlappingBoxes)
{
  const NvShape aBox1 = NvPrim::Box (10.0, 10.0, 10.0);
  const NvShape aBox2 = NvPrim::Box (gp_Pnt (5.0, 0.0, 0.0), 10.0, 10.0, 10.0);

  const NvShape aCommon = NvBoolean::Common (aBox1, aBox2);
  EXPECT_TRUE (aCommon.IsValid());
  EXPECT_NEAR (aCommon.Volume(), THE_OVERLAP, THE_TOLERANCE);
}

TEST(NvBooleanTest, NullInput_Throws)
{
  const NvShape aBox = NvPrim::Box (1.0, 1.0, 1.0);
  const NvShape aNull;
  EXPECT_THROW (NvBoolean::Fuse   (aBox,  aNull), NvException);
  EXPECT_THROW (NvBoolean::Cut    (aNull, aBox),  NvException);
  EXPECT_THROW (NvBoolean::Common (aNull, aNull), NvException);
}

TEST(NvBooleanTest, CutDisjointBoxes_KeepsOriginalVolume)
{
  const NvShape aBox1 = NvPrim::Box (1.0, 1.0, 1.0);
  const NvShape aBox2 = NvPrim::Box (gp_Pnt (100.0, 0.0, 0.0), 1.0, 1.0, 1.0);

  const NvShape aCut = NvBoolean::Cut (aBox1, aBox2);
  EXPECT_NEAR (aCut.Volume(), 1.0, THE_TOLERANCE);
}
