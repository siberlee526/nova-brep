// geray2d.cpp - implementation of NvGeRay2d based on OCCT Geom2d classes.
//
// A ray is stored as a Geom2d_TrimmedCurve wrapping a Geom2d_Line; the
// parameter range [0, +infinity) starts at the ray origin and the parameter
// measures the distance along the ray direction. As the end of the range
// the domain end of Geom2d_Line (Precision::Infinite()) is used - the
// numerical RealLast would be rejected by the trimmed-curve range check -
// and it is still recognized as infinite by Precision::IsPositiveInfinite.
// Modifiers follow the copy-on-write discipline of geimpdata.h.

#include <geray2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <string>

namespace
{

// Creates the ray geometry starting at thePnt along theVec. The zero-vector
// pre-check translates the gp_Dir2d construction failure into the project
// error type.
occ::handle<Geom2d_Curve> RayThrough (const NvGePoint2d& thePnt, const NvGeVector2d& theVec,
                                      const char* theMethod)
{
  if (std::sqrt (theVec.lengthSqrd()) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeRay2d::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  const occ::handle<Geom2d_Line> aLine (new Geom2d_Line (gp_Pnt2d (thePnt.x, thePnt.y),
                                                         gp_Dir2d (theVec.x, theVec.y)));
  return occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (aLine, 0.0, Precision::Infinite()));
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom2d_Curve>& theCurve)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theCurve);
}

}

//=================================================================================================

NvGeRay2d::NvGeRay2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay2d,
                                  RayThrough (NvGePoint2d (0.0, 0.0), NvGeVector2d (1.0, 0.0),
                                              "NvGeRay2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay2d::NvGeRay2d (const NvGeRay2d& theSrc)
: NvGeLinearEnt2d (theSrc)
{
}

//=================================================================================================

NvGeRay2d::NvGeRay2d (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay2d, RayThrough (thePnt, theVec, "NvGeRay2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay2d::NvGeRay2d (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kRay2d,
                                  RayThrough (thePnt1, thePnt2 - thePnt1, "NvGeRay2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeRay2d& NvGeRay2d::set (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  ReplaceGeometry (mpImpEnt, RayThrough (thePnt, theVec, "set"));
  return *this;
}

//=================================================================================================

NvGeRay2d& NvGeRay2d::set (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  ReplaceGeometry (mpImpEnt, RayThrough (thePnt1, thePnt2 - thePnt1, "set"));
  return *this;
}

//=================================================================================================

NvGeRay2d& NvGeRay2d::operator = (const NvGeRay2d& theRay)
{
  NvGeEntity2d::operator= (theRay);
  return *this;
}
