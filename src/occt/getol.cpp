// getol.cpp - implementation of NvGeTol.

#include <getol.h>

//=================================================================================================

NvGeTol::NvGeTol()
{
  // ARX-compatible defaults: points and vectors compare equal within 1e-8.
  // The remaining slots mirror the two published tolerances so that a stale
  // read of an internal slot can never produce wild comparisons.
  mTolArr[0] = 1e-8;  // equalPoint
  mTolArr[1] = 1e-8;  // equalVector
  mTolArr[2] = 1e-8;
  mTolArr[3] = 1e-8;
  mTolArr[4] = 1e-8;
  mAbs = 0;
}
