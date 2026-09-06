// gepent3d.cpp - implementation of NvGePointEnt3d, the 3d point entity base.
//
// A point entity stores one of the small holders from geimpdata.h
// (NvGePosition3dData, NvGePointOnCurve3dData, NvGePointOnSurfaceData) in
// the impl geometry slot; point3d() dispatches over them. Points that
// reference a curve or a surface evaluate their coordinates through the
// shared geometry entity.

#include <gepent3d.h>

#include <NvException.h>
#include <gepnt3d.h>

#include <geimpdata.h>

#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt.hxx>

//=================================================================================================

NvGePoint3d NvGePointEnt3d::point3d () const
{
  if (mpImpEnt == nullptr)
  {
    throw NvException ("NvGePointEnt3d::point3d(): the entity has no implementation");
  }
  const occ::handle<Standard_Transient>& aGeom = mpImpEnt->Geom();

  if (const NvGePosition3dData* aPos = NvGeDataOf<NvGePosition3dData> (aGeom))
  {
    return NvGePoint3d (aPos->Point.X(), aPos->Point.Y(), aPos->Point.Z());
  }
  if (const NvGePointOnCurve3dData* anOnCurve = NvGeDataOf<NvGePointOnCurve3dData> (aGeom))
  {
    const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (anOnCurve->Curve.get());
    if (aCurve.IsNull())
    {
      throw NvException ("NvGePointEnt3d::point3d(): the referenced curve has no geometry");
    }
    const gp_Pnt aPnt = aCurve->Value (anOnCurve->Param);
    return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
  }
  if (const NvGePointOnSurfaceData* anOnSurface = NvGeDataOf<NvGePointOnSurfaceData> (aGeom))
  {
    const occ::handle<Geom_Surface> aSurface = NvGeSurfaceOf (anOnSurface->Surface.get());
    if (aSurface.IsNull())
    {
      throw NvException ("NvGePointEnt3d::point3d(): the referenced surface has no geometry");
    }
    const gp_Pnt aPnt = aSurface->Value (anOnSurface->U, anOnSurface->V);
    return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
  }
  throw NvException ("NvGePointEnt3d::point3d(): unsupported entity kind");
}

//=================================================================================================

NvGePointEnt3d::operator NvGePoint3d () const
{
  return point3d();
}

//=================================================================================================

NvGePointEnt3d& NvGePointEnt3d::operator = (const NvGePointEnt3d& thePnt)
{
  NvGeEntity3d::operator = (thePnt);
  return *this;
}

//=================================================================================================

NvGePointEnt3d::NvGePointEnt3d ()
: NvGeEntity3d ()
{
}

//=================================================================================================

NvGePointEnt3d::NvGePointEnt3d (const NvGePointEnt3d& thePnt)
: NvGeEntity3d (thePnt)
{
}
