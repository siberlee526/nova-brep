// gepos3d.cpp - implementation of NvGePosition3d.
//
// A position stores an NvGePosition3dData holder (a single gp_Pnt) in the
// impl geometry slot. set() installs a brand-new holder behind a
// copy-on-write detached impl, so the shared holder of other wrappers is
// never mutated.

#include <gepos3d.h>

#include <gepnt3d.h>

#include <geimpdata.h>

#include <gp_Pnt.hxx>

//=================================================================================================

NvGePosition3d::NvGePosition3d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePosition3dData> aData = new NvGePosition3dData();
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition3d::NvGePosition3d (const NvGePoint3d& thePnt)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePosition3dData> aData = new NvGePosition3dData();
  aData->Point = gp_Pnt (thePnt.x, thePnt.y, thePnt.z);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition3d::NvGePosition3d (double theX, double theY, double theZ)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGePosition3dData> aData = new NvGePosition3dData();
  aData->Point = gp_Pnt (theX, theY, theZ);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kPosition3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGePosition3d::NvGePosition3d (const NvGePosition3d& thePos)
: NvGePointEnt3d (thePos)
{
}

//=================================================================================================

NvGePosition3d& NvGePosition3d::set (const NvGePoint3d& thePnt)
{
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  occ::handle<NvGePosition3dData> aData = new NvGePosition3dData();
  aData->Point = gp_Pnt (thePnt.x, thePnt.y, thePnt.z);
  mpImpEnt->SetGeom (aData);
  return *this;
}

//=================================================================================================

NvGePosition3d& NvGePosition3d::set (double theX, double theY, double theZ)
{
  return set (NvGePoint3d (theX, theY, theZ));
}

//=================================================================================================

NvGePosition3d& NvGePosition3d::operator = (const NvGePosition3d& thePos)
{
  NvGePointEnt3d::operator = (thePos);
  return *this;
}
