#include <NovaException.h>
#include <NovaPrim.h>
#include <NovaShape.h>

#include <gtest/gtest.h>

namespace
{
  // BRepBndLib bounding boxes are padded by sub-shape tolerances (~Precision::Confusion()).
  constexpr double THE_BND_TOLERANCE = 1e-6;
}

TEST(NovaShapeTest, DefaultShape_IsNullAndInvalid)
{
  const NovaShape aShape;
  EXPECT_TRUE  (aShape.IsNull());
  EXPECT_FALSE (aShape.IsValid());
  EXPECT_EQ    (aShape.CountSubShapes (TopAbs_FACE), 0);
}

TEST(NovaShapeTest, QueriesOnNullShape_Throw)
{
  const NovaShape aShape;
  EXPECT_THROW (aShape.Volume(),      NovaException);
  EXPECT_THROW (aShape.Area(),        NovaException);
  EXPECT_THROW (aShape.BoundingBox(), NovaException);
}

TEST(NovaShapeTest, Box_IsValid_AndCountsSubShapes)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 2.0, 3.0);
  EXPECT_TRUE (aBox.IsValid());
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_SOLID), 1);
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_FACE),  6);
  EXPECT_EQ   (aBox.CountSubShapes (TopAbs_EDGE), 12);
}

TEST(NovaShapeTest, Translated_MovesBoundingBox)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 1.0, 1.0);
  const NovaShape aMoved = aBox.Translated (gp_Vec (10.0, 20.0, 30.0));

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

TEST(NovaShapeTest, Rotated_PreservesVolume)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 2.0, 3.0);
  const NovaShape aRotated = aBox.Rotated (gp_Ax1 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)), M_PI / 4.0);
  EXPECT_NEAR (aRotated.Volume(), 6.0, 1e-9);
}

TEST(NovaShapeTest, Scaled_ScalesVolumeByCube)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 2.0, 3.0);
  const NovaShape aScaled = aBox.Scaled (gp_Pnt (0.0, 0.0, 0.0), 2.0);
  EXPECT_NEAR (aScaled.Volume(), 48.0, 1e-9);
}

TEST(NovaShapeTest, Scaled_ZeroFactor_Throws)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 2.0, 3.0);
  EXPECT_THROW (aBox.Scaled (gp_Pnt (0.0, 0.0, 0.0), 0.0), NovaException);
}

TEST(NovaShapeTest, Mirrored_PreservesVolume)
{
  const NovaShape aBox = NovaPrim::Box (1.0, 2.0, 3.0);
  const NovaShape aMirrored = aBox.Mirrored (gp_Ax2 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)));
  EXPECT_NEAR (aMirrored.Volume(), 6.0, 1e-9);
}

TEST(NovaShapeTest, AreaOfBox_IsSurfaceArea)
{
  const NovaShape aBox = NovaPrim::Box (2.0, 3.0, 4.0);
  const double anExpected = 2.0 * (2.0 * 3.0 + 2.0 * 4.0 + 3.0 * 4.0);
  EXPECT_NEAR (aBox.Area(), anExpected, anExpected * 1e-9);
}
