// geline2d.cpp - implementation of NvGeLine2d based on OCCT Geom2d_Line.
//
// An infinite line is stored as a Geom2d_Line defined by an origin point
// and a unit direction; the line parameter is the signed distance along
// the direction. Modifiers follow the copy-on-write discipline: the impl
// is detached when shared and a new Geom2d_Line replaces the old geometry,
// which is itself never mutated.

#include <geline2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <string>

namespace
{

// Creates the line geometry through thePnt along theVec. gp_Dir2d raises
// Standard_ConstructionError for a zero vector, which is translated into
// the project error type after an explicit length pre-check.
occ::handle<Geom2d_Line> LineThrough (const NvGePoint2d& thePnt, const NvGeVector2d& theVec,
                                      const char* theMethod)
{
  if (std::sqrt (theVec.lengthSqrd()) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeLine2d::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return occ::handle<Geom2d_Line> (new Geom2d_Line (gp_Pnt2d (thePnt.x, thePnt.y),
                                                    gp_Dir2d (theVec.x, theVec.y)));
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom2d_Line>& theLine)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theLine);
}

}

//=================================================================================================

NvGeLine2d::NvGeLine2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine2d,
                                  occ::handle<Geom2d_Line> (new Geom2d_Line (gp_Pnt2d (0.0, 0.0),
                                                                             gp_Dir2d (1.0, 0.0))));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLine2d::NvGeLine2d (const NvGeLine2d& theSrc)
: NvGeLinearEnt2d (theSrc)
{
}

// The default line is the x-axis line, so the constant is a
// default-constructed instance.
const NvGeLine2d NvGeLine2d::kXAxis;

const NvGeLine2d NvGeLine2d::kYAxis (NvGePoint2d (0.0, 0.0), NvGeVector2d (0.0, 1.0));

//=================================================================================================

NvGeLine2d::NvGeLine2d (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine2d, LineThrough (thePnt, theVec, "NvGeLine2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLine2d::NvGeLine2d (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kLine2d,
                                  LineThrough (thePnt1, thePnt2 - thePnt1, "NvGeLine2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeLine2d& NvGeLine2d::set (const NvGePoint2d& thePnt, const NvGeVector2d& theVec)
{
  ReplaceGeometry (mpImpEnt, LineThrough (thePnt, theVec, "set"));
  return *this;
}

//=================================================================================================

NvGeLine2d& NvGeLine2d::set (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  ReplaceGeometry (mpImpEnt, LineThrough (thePnt1, thePnt2 - thePnt1, "set"));
  return *this;
}

//=================================================================================================

NvGeLine2d& NvGeLine2d::operator = (const NvGeLine2d& theLine)
{
  NvGeEntity2d::operator= (theLine);
  return *this;
}
