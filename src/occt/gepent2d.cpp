// gepent2d.cpp - implementation of NvGePointEnt2d, the base of the 2d
// point entities (position, point-on-curve).
//
// The class is abstract: it only dispatches the point query centrally over
// the impl holder kinds (see geimpdata.h). A position reports its stored
// gp_Pnt2d directly; a point-on-curve evaluates the shared curve entity at
// the stored parameter through the NvGeCurve2dOf helper. Transformations
// and comparisons are inherited from NvGeEntity2d.

#include <gepent2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <gepnt2d.h>

#include <Geom2d_Curve.hxx>
#include <gp_Pnt2d.hxx>

//=================================================================================================

NvGePoint2d NvGePointEnt2d::point2d () const
{
  if (mpImpEnt == nullptr)
  {
    throw NvException ("NvGePointEnt2d::point2d(): the entity has no implementation");
  }
  const occ::handle<Standard_Transient>& aGeom = mpImpEnt->Geom();

  if (const NvGePosition2dData* aPos = NvGeDataOf<NvGePosition2dData> (aGeom))
  {
    return NvGePoint2d (aPos->Point.X(), aPos->Point.Y());
  }

  if (const NvGePointOnCurve2dData* anOnCurve = NvGeDataOf<NvGePointOnCurve2dData> (aGeom))
  {
    const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (anOnCurve->Curve.get());
    const gp_Pnt2d aPnt = aCurve->Value (anOnCurve->Param);
    return NvGePoint2d (aPnt.X(), aPnt.Y());
  }

  throw NvException ("NvGePointEnt2d::point2d(): unsupported point entity kind");
}

//=================================================================================================

NvGePointEnt2d::operator NvGePoint2d () const
{
  return point2d();
}

//=================================================================================================

NvGePointEnt2d& NvGePointEnt2d::operator = (const NvGePointEnt2d& thePnt)
{
  NvGeEntity2d::operator= (thePnt);
  return *this;
}

//=================================================================================================

NvGePointEnt2d::NvGePointEnt2d ()
: NvGeEntity2d()
{
}

//=================================================================================================

NvGePointEnt2d::NvGePointEnt2d (const NvGePointEnt2d& theSrc)
: NvGeEntity2d (theSrc)
{
}
