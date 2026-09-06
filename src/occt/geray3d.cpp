// geray3d.cpp - implementation of NvGeRay3d.
//
// A ray is stored as a Geom_TrimmedCurve over a Geom_Line with the
// parameter range [0, max double): it starts at the origin point and
// extends infinitely along the unit direction. The curve parameter equals
// the distance from the origin point. Constructors validate the direction
// first so that the OCCT Standard_ConstructionError of a degenerate
// gp_Dir never escapes.

#include <geray3d.h>

#include <NvException.h>
#include <gepnt3d.h>
#include <gevec3d.h>

#include <geimpdata.h>

#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <gp.hxx>
#include <cmath>
#include <limits>

namespace
{

// Builds the trimmed carrier line of a ray through thePnt along theVec.
occ::handle<Geom_Curve> RayCurveOf (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  occ::handle<Geom_Line> aCarrier (new Geom_Line (gp_Pnt (thePnt.x, thePnt.y, thePnt.z),
                                                  gp_Dir (theVec.x, theVec.y, theVec.z)));
  return occ::handle<Geom_Curve> (new Geom_TrimmedCurve (aCarrier, 0.0,
                                                         std::numeric_limits<double>::max()));
}

} // namespace

//=================================================================================================

NvGeRay3d::NvGeRay3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay3d,
               RayCurveOf (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (1.0, 0.0, 0.0)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay3d::NvGeRay3d (const NvGeRay3d& theRay)
: NvGeLinearEnt3d (theRay)
{
}

//=================================================================================================

NvGeRay3d::NvGeRay3d (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeRay3d::NvGeRay3d(): the direction vector is degenerate");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay3d, RayCurveOf (thePnt, theVec));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay3d::NvGeRay3d (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  if (std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz) <= gp::Resolution())
  {
    throw NvException ("NvGeRay3d::NvGeRay3d(): the points are coincident");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay3d,
               RayCurveOf (thePnt1, NvGeVector3d (aDx, aDy, aDz)));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay3d& NvGeRay3d::set (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeRay3d::set(): the direction vector is degenerate");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (RayCurveOf (thePnt, theVec));
  return *this;
}

//=================================================================================================

NvGeRay3d& NvGeRay3d::set (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  if (std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz) <= gp::Resolution())
  {
    throw NvException ("NvGeRay3d::set(): the points are coincident");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (RayCurveOf (thePnt1, NvGeVector3d (aDx, aDy, aDz)));
  return *this;
}

//=================================================================================================

NvGeRay3d& NvGeRay3d::operator = (const NvGeRay3d& theRay)
{
  NvGeEntity3d::operator = (theRay);
  return *this;
}
