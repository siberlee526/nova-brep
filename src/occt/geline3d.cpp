// geline3d.cpp - implementation of NvGeLine3d.
//
// An infinite line is stored as a Geom_Line (origin point + unit
// direction); the curve parameter equals the distance from the origin
// point. Constructors validate the direction first so that the OCCT
// Standard_ConstructionError of a degenerate gp_Dir never escapes.

#include <geline3d.h>

#include <NvException.h>
#include <gepnt3d.h>
#include <gevec3d.h>

#include <geimpdata.h>

#include <Geom_Line.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <gp.hxx>
#include <cmath>

//=================================================================================================

NvGeLine3d::NvGeLine3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine3d,
               occ::handle<Geom_Curve> (new Geom_Line (gp_Pnt (0.0, 0.0, 0.0),
                                                       gp_Dir (1.0, 0.0, 0.0))));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLine3d::NvGeLine3d (const NvGeLine3d& theLine)
: NvGeLinearEnt3d (theLine)
{
}

//=================================================================================================

NvGeLine3d::NvGeLine3d (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLine3d::NvGeLine3d(): the direction vector is degenerate");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine3d,
               occ::handle<Geom_Curve> (new Geom_Line (gp_Pnt (thePnt.x, thePnt.y, thePnt.z),
                                                       gp_Dir (theVec.x, theVec.y, theVec.z))));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLine3d::NvGeLine3d (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  if (std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz) <= gp::Resolution())
  {
    throw NvException ("NvGeLine3d::NvGeLine3d(): the points are coincident");
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine3d,
               occ::handle<Geom_Curve> (new Geom_Line (gp_Pnt (thePnt1.x, thePnt1.y, thePnt1.z),
                                                       gp_Dir (aDx, aDy, aDz))));
  mpImpEnt->Ref();
}

// The coordinate axis lines; kXAxis is the default line, kYAxis and kZAxis
// pass the axis direction explicitly. Literal coordinates keep the static
// initializers independent of other translation units.
const NvGeLine3d NvGeLine3d::kXAxis;

const NvGeLine3d NvGeLine3d::kYAxis (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 1.0, 0.0));

const NvGeLine3d NvGeLine3d::kZAxis (NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));

//=================================================================================================

NvGeLine3d& NvGeLine3d::set (const NvGePoint3d& thePnt, const NvGeVector3d& theVec)
{
  const double aLen = std::sqrt (theVec.x * theVec.x + theVec.y * theVec.y + theVec.z * theVec.z);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGeLine3d::set(): the direction vector is degenerate");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (
    new Geom_Line (gp_Pnt (thePnt.x, thePnt.y, thePnt.z),
                   gp_Dir (theVec.x, theVec.y, theVec.z))));
  return *this;
}

//=================================================================================================

NvGeLine3d& NvGeLine3d::set (const NvGePoint3d& thePnt1, const NvGePoint3d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  const double aDz = thePnt2.z - thePnt1.z;
  if (std::sqrt (aDx * aDx + aDy * aDy + aDz * aDz) <= gp::Resolution())
  {
    throw NvException ("NvGeLine3d::set(): the points are coincident");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (
    new Geom_Line (gp_Pnt (thePnt1.x, thePnt1.y, thePnt1.z),
                   gp_Dir (aDx, aDy, aDz))));
  return *this;
}

//=================================================================================================

NvGeLine3d& NvGeLine3d::operator = (const NvGeLine3d& theLine)
{
  NvGeEntity3d::operator = (theLine);
  return *this;
}
