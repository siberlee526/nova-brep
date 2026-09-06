#include <NvException.h>
#include <NvPrim.h>

#include <gtest/gtest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace
{
  // Relative tolerance: BRepGProp integrates curved faces numerically.
  constexpr double THE_TOLERANCE = 1e-5;
  // BRepBndLib bounding boxes are padded by sub-shape tolerances (~Precision::Confusion()).
  constexpr double THE_BND_TOLERANCE = 1e-6;
}

TEST(NvPrimTest, Box_AtOrigin_HasExactExtents)
{
  const NvShape aBox = NvPrim::Box (10.0, 20.0, 30.0);
  ASSERT_FALSE (aBox.IsNull());

  const Bnd_Box aBnd = aBox.BoundingBox();
  double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
  aBnd.Get (aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
  EXPECT_NEAR (aXmin, 0.0,  THE_BND_TOLERANCE);
  EXPECT_NEAR (aYmin, 0.0,  THE_BND_TOLERANCE);
  EXPECT_NEAR (aZmin, 0.0,  THE_BND_TOLERANCE);
  EXPECT_NEAR (aXmax, 10.0, THE_BND_TOLERANCE);
  EXPECT_NEAR (aYmax, 20.0, THE_BND_TOLERANCE);
  EXPECT_NEAR (aZmax, 30.0, THE_BND_TOLERANCE);
}

TEST(NvPrimTest, Box_AtPoint_HasExactVolume)
{
  const NvShape aBox = NvPrim::Box (gp_Pnt (1.0, 2.0, 3.0), 2.0, 3.0, 4.0);
  EXPECT_NEAR (aBox.Volume(), 24.0, 24.0 * 1e-9);
}

TEST(NvPrimTest, Cylinder_HasExactVolume)
{
  const NvShape aCyl = NvPrim::Cylinder (2.0, 5.0);
  const double anExpected = M_PI * 2.0 * 2.0 * 5.0;
  EXPECT_NEAR (aCyl.Volume(), anExpected, anExpected * THE_TOLERANCE);
}

TEST(NvPrimTest, Sphere_HasExactVolume)
{
  const NvShape aSphere = NvPrim::Sphere (3.0);
  const double anExpected = 4.0 / 3.0 * M_PI * 27.0;
  EXPECT_NEAR (aSphere.Volume(), anExpected, anExpected * THE_TOLERANCE);
}

TEST(NvPrimTest, Cone_HasExactVolume)
{
  const NvShape aCone = NvPrim::Cone (3.0, 1.0, 4.0);
  const double anExpected = M_PI * 4.0 * (9.0 + 3.0 + 1.0) / 3.0;
  EXPECT_NEAR (aCone.Volume(), anExpected, anExpected * THE_TOLERANCE);
}

TEST(NvPrimTest, Torus_HasExactVolume)
{
  const NvShape aTorus = NvPrim::Torus (5.0, 1.0);
  const double anExpected = 2.0 * M_PI * M_PI * 5.0 * 1.0 * 1.0;
  EXPECT_NEAR (aTorus.Volume(), anExpected, anExpected * THE_TOLERANCE);
}

TEST(NvPrimTest, Box_ZeroDimension_Throws)
{
  EXPECT_THROW (NvPrim::Box (0.0, 1.0, 1.0), NvException);
  EXPECT_THROW (NvPrim::Cylinder (-1.0, 1.0), NvException);
  EXPECT_THROW (NvPrim::Sphere (0.0), NvException);
  EXPECT_THROW (NvPrim::Cone (0.0, 0.0, 1.0), NvException);
  EXPECT_THROW (NvPrim::Torus (1.0, 0.0), NvException);
}
