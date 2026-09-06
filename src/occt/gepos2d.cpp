// gepos2d.cpp - implementation of NvGePosition2d.
//
// A position is a point entity whose impl geometry is an
// NvGePosition2dData holder (see geimpdata.h) carrying a gp_Pnt2d.
// transformBy and the comparison predicates are inherited from
// NvGeEntity2d, which dispatches centrally over the holder kinds; this
// file only builds and replaces the holder. The holder is never mutated:
// every set() installs a fresh holder, which keeps copies safe even when
// they share the impl through NvGeEntity2d::copy().

#include <gepos2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <gepnt2d.h>

#include <gp_Pnt2d.hxx>

namespace
{

// Detaches the impl when shared and installs a fresh holder carrying
// thePnt (copy-on-write; see the banner of geimpdata.h).
void SetPointData (NvGeImpEntity3d*& theImp, const gp_Pnt2d& thePnt)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  occ::handle<NvGePosition2dData> aData = new NvGePosition2dData();
  aData->Point = thePnt;
  theImp->SetGeom (aData);
}

}

//=================================================================================================

NvGePosition2d::NvGePosition2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition2d, occ::handle<NvGePosition2dData> (new NvGePosition2dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition2d::NvGePosition2d (const NvGePoint2d& thePnt)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePosition2dData> aData = new NvGePosition2dData();
  aData->Point = gp_Pnt2d (thePnt.x, thePnt.y);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition2d::NvGePosition2d (double theX, double theY)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePosition2dData> aData = new NvGePosition2dData();
  aData->Point = gp_Pnt2d (theX, theY);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition2d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition2d::NvGePosition2d (const NvGePosition2d& theSrc)
: NvGePointEnt2d (theSrc)
{
}

//=================================================================================================

NvGePosition2d& NvGePosition2d::set (const NvGePoint2d& thePnt)
{
  SetPointData (mpImpEnt, gp_Pnt2d (thePnt.x, thePnt.y));
  return *this;
}

//=================================================================================================

NvGePosition2d& NvGePosition2d::set (double theX, double theY)
{
  SetPointData (mpImpEnt, gp_Pnt2d (theX, theY));
  return *this;
}

//=================================================================================================

NvGePosition2d& NvGePosition2d::operator = (const NvGePosition2d& thePos)
{
  NvGePointEnt2d::operator= (thePos);
  return *this;
}
