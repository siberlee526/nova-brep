#include <NvException.h>
#include <NvPrim.h>
#include <NvShape.h>

#include <gtest/gtest.h>

namespace
{
  // BRepBndLib bounding boxes are padded by sub-shape tolerances (~Precision::Confusion()).
  constexpr double THE_BND_TOLERANCE = 1e-6;
}

TEST(NvShapeTest, DefaultShape_IsNullAndInvalid)
{
  const NvShape aShape;
  EXPECT_TRUE  (aShape.IsNull());
  EXPECT_FALSE (aShape.IsValid());
  EXPECT_EQ    (aShape.CountSubShapes (TopAbs_FACE), 0);
}

TEST(NvShapeTest, QueriesOnNullShape_Throw)
{
  const NvShape aShape;
  EXPECT_THROW (aShape.Volume(),      NvException);
  EXPECT_THROW (aShape.Area(),        NvException);
  EXPECT_THROW (aShape.BoundingBox(), NvException);
}

TEST(NvShapeTest, Box_IsValid_AndCountsSubShapes)
{
  const NvShape aBox = NvPrim::Box (1.0, 2.0, 3.0);
  EXPECT_TRUE (aBox.IsValid());
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_SOLID), 1);
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_FACE),  6);
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_EDGE), 12);
}

TEST(NvShapeTest, Translated_MovesBoundingBox)
{
  const NvShape aBox = NvPrim::Box (1.0, 1.0, 1.0);
  const NvShape aMoved = aBox.Translated (gp_Vec (10.0, 20.0, 30.0));

  const Bnd_Box aBnd = aMoved.BoundingBox();
  double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
  aBnd.Get (aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
  EXPECT_NEAR (aXmin, 10.0, THE_BND_TOLERANCE);
  EXPECT_NEAR (aYmin, 20.0, THE_BND_TOLERANCE);
  EXPECT_NEAR (aZmin, 30.0, THE_BND_TOLERANCE);
  EXPECT_NEAR (aXmax, 11.0, THE_BND_TOLERANCE);

  // Original shape is untouched (value semantics).
  const Bnd_Box anOrigBnd = aBox.BoundingBox();
  anOrigBnd.Get (aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
  EXPECT_NEAR (aXmin, 0.0, THE_BND_TOLERANCE);
}

TEST(NvShapeTest, Rotated_PreservesVolume)
{
  const NvShape aBox = NvPrim::Box (1.0, 2.0, 3.0);
  const NvShape aRotated = aBox.Rotated (gp_Ax1 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)), M_PI / 4.0);
  EXPECT_NEAR (aRotated.Volume(), 6.0, 1e-9);
}

TEST(NvShapeTest, Scaled_ScalesVolumeByCube)
{
  const NvShape aBox = NvPrim::Box (1.0, 2.0, 3.0);
  const NvShape aScaled = aBox.Scaled (gp_Pnt (0.0, 0.0, 0.0), 2.0);
  EXPECT_NEAR (aScaled.Volume(), 48.0, 1e-9);
}

TEST(NvShapeTest, Scaled_ZeroFactor_Throws)
{
  const NvShape aBox = NvPrim::Box (1.0, 2.0, 3.0);
  EXPECT_THROW (aBox.Scaled (gp_Pnt (0.0, 0.0, 0.0), 0.0), NvException);
}

TEST(NvShapeTest, Mirrored_PreservesVolume)
{
  const NvShape aBox = NvPrim::Box (1.0, 2.0, 3.0);
  const NvShape aMirrored = aBox.Mirrored (gp_Ax2 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)));
  EXPECT_NEAR (aMirrored.Volume(), 6.0, 1e-9);
}

TEST(NvShapeTest, AreaOfBox_IsSurfaceArea)
{
  const NvShape aBox = NvPrim::Box (2.0, 3.0, 4.0);
  const double anExpected = 2.0 * (2.0 * 3.0 + 2.0 * 4.0 + 3.0 * 4.0);
  EXPECT_NEAR (aBox.Area(), anExpected, anExpected * 1e-9);
}
