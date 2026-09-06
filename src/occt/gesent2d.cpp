// gesent2d.cpp - implementation of NvGeSplineEnt2d based on OCCT
// Geom2d_BSplineCurve.
//
// The wrapper exposes the OCCT B-spline data directly:
//   - numKnots is the FLAT knot count (the knot sequence with all
//     multiplicities expanded, the ARX convention), so knotAt/continuityAtKnot
//     index into that sequence,
//   - numControlPoints is the pole count (1-based OCCT poles are addressed
//     with 0-based wrapper indices),
//   - the continuity at a knot drops by one for every extra multiplicity
//     step: continuity = degree - multiplicity.
//
// knots() keeps no storage in the wrapper; a per-thread scratch knot vector
// is refreshed from the flat knot sequence on every call. Modifiers detach
// the impl when shared AND deep-copy the geometry before the in-place edit,
// so no other holder of the (shared) OCCT handle can observe the change
// (copy-on-write; see the banner of geimpdata.h).

#include <gesent2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <gekvec.h>
#include <gepnt2d.h>
#include <getol.h>

#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Geometry.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <gp_Pnt2d.hxx>

#include <string>
#include <vector>

namespace
{

// Returns the B-spline stored in theImp; throws when the geometry is not a
// B-spline.
occ::handle<Geom2d_BSplineCurve> SplineOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (theImp);
  const occ::handle<Geom2d_BSplineCurve> aSpline = occ::down_cast<Geom2d_BSplineCurve> (aCurve);
  if (aSpline.IsNull())
  {
    throw NvException (std::string ("NvGeSplineEnt2d::") + theMethod
                       + "(): the implementation geometry is not a B-spline");
  }
  return aSpline;
}

void CheckIndex (int theIdx, int theCount, const char* theMethod, const char* theWhat)
{
  if (theIdx < 0 || theIdx >= theCount)
  {
    throw NvException (std::string ("NvGeSplineEnt2d::") + theMethod + "(): " + theWhat
                       + " is out of range");
  }
}

// Maps a flat knot index to the 1-based OCCT distinct knot index.
int DistinctKnotOf (const occ::handle<Geom2d_BSplineCurve>& theSpline, int theFlatIdx)
{
  int aRest = theFlatIdx;
  for (int aKnot = 1; aKnot <= theSpline->NbKnots(); ++aKnot)
  {
    const int aMult = theSpline->Multiplicity (aKnot);
    if (aRest < aMult)
    {
      return aKnot;
    }
    aRest -= aMult;
  }
  throw NvException ("NvGeSplineEnt2d: inconsistent knot sequence");
}

// Detaches the impl when shared and deep-copies the B-spline geometry so an
// in-place edit cannot leak into other holders of the handle. Returns the
// editable copy installed on the impl.
occ::handle<Geom2d_BSplineCurve> EditableSplineOf (NvGeImpEntity3d*& theImp, const char* theMethod)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  const occ::handle<Geom2d_BSplineCurve> aCopy =
    occ::down_cast<Geom2d_BSplineCurve> (NvGeCurve2dOf (theImp)->Copy());
  if (aCopy.IsNull())
  {
    throw NvException (std::string ("NvGeSplineEnt2d::") + theMethod
                       + "(): the spline geometry cannot be copied");
  }
  theImp->SetGeom (aCopy);
  return aCopy;
}

}

//=================================================================================================

Adesk::Boolean NvGeSplineEnt2d::isRational () const
{
  return SplineOf (mpImpEnt, "isRational")->IsRational();
}

//=================================================================================================

int NvGeSplineEnt2d::degree () const
{
  return SplineOf (mpImpEnt, "degree")->Degree();
}

//=================================================================================================

int NvGeSplineEnt2d::order () const
{
  return SplineOf (mpImpEnt, "order")->Degree() + 1;
}

//=================================================================================================

int NvGeSplineEnt2d::numKnots () const
{
  // The flat knot sequence length (all multiplicities expanded).
  return SplineOf (mpImpEnt, "numKnots")->KnotSequence().Length();
}

//=================================================================================================

const NvGeKnotVector& NvGeSplineEnt2d::knots () const
{
  // The returned reference stays valid until the next knots() call on the
  // same thread.
  static thread_local NvGeKnotVector aScratch;
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "knots");
  const NCollection_Array1<double>& aFlat = aSpline->KnotSequence();
  std::vector<double> aKnots;
  aKnots.reserve (aFlat.Length());
  for (int i = aFlat.Lower(); i <= aFlat.Upper(); ++i)
  {
    aKnots.push_back (aFlat.Value (i));
  }
  aScratch = NvGeKnotVector (static_cast<int> (aKnots.size()), aKnots.data());
  return aScratch;
}

//=================================================================================================

int NvGeSplineEnt2d::numControlPoints () const
{
  return SplineOf (mpImpEnt, "numControlPoints")->NbPoles();
}

//=================================================================================================

int NvGeSplineEnt2d::continuityAtKnot (int theIdx, const NvGeTol&) const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "continuityAtKnot");
  CheckIndex (theIdx, aSpline->KnotSequence().Length(), "continuityAtKnot", "the knot index");
  int aRest = theIdx;
  for (int aKnot = 1; aKnot <= aSpline->NbKnots(); ++aKnot)
  {
    const int aMult = aSpline->Multiplicity (aKnot);
    if (aRest < aMult)
    {
      // The continuity drops by one for every extra multiplicity step.
      return aSpline->Degree() - aMult;
    }
    aRest -= aMult;
  }
  throw NvException ("NvGeSplineEnt2d::continuityAtKnot(): inconsistent knot sequence");
}

//=================================================================================================

double NvGeSplineEnt2d::startParam () const
{
  return SplineOf (mpImpEnt, "startParam")->FirstParameter();
}

//=================================================================================================

double NvGeSplineEnt2d::endParam () const
{
  return SplineOf (mpImpEnt, "endParam")->LastParameter();
}

//=================================================================================================

NvGePoint2d NvGeSplineEnt2d::startPoint () const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "startPoint");
  const gp_Pnt2d aPnt = aSpline->EvalD0 (aSpline->FirstParameter());
  return NvGePoint2d (aPnt.X(), aPnt.Y());
}

//=================================================================================================

NvGePoint2d NvGeSplineEnt2d::endPoint () const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "endPoint");
  const gp_Pnt2d aPnt = aSpline->EvalD0 (aSpline->LastParameter());
  return NvGePoint2d (aPnt.X(), aPnt.Y());
}

//=================================================================================================

Adesk::Boolean NvGeSplineEnt2d::hasFitData () const
{
  // In this representation the only spline entities with fit data are
  // degree-1 polylines, whose poles are the fit points.
  return SplineOf (mpImpEnt, "hasFitData")->Degree() == 1;
}

//=================================================================================================

double NvGeSplineEnt2d::knotAt (int theIdx) const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "knotAt");
  const NCollection_Array1<double>& aFlat = aSpline->KnotSequence();
  CheckIndex (theIdx, aFlat.Length(), "knotAt", "the knot index");
  return aFlat.Value (aFlat.Lower() + theIdx);
}

//=================================================================================================

NvGeSplineEnt2d& NvGeSplineEnt2d::setKnotAt (int theIdx, double theVal)
{
  {
    const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "setKnotAt");
    CheckIndex (theIdx, aSpline->KnotSequence().Length(), "setKnotAt", "the knot index");
  }
  const occ::handle<Geom2d_BSplineCurve> aSpline = EditableSplineOf (mpImpEnt, "setKnotAt");
  try
  {
    aSpline->SetKnot (DistinctKnotOf (aSpline, theIdx), theVal);
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGeSplineEnt2d::setKnotAt(): the knot value must keep the knot"
                       " sequence increasing; all copies of the knot move together");
  }
  return *this;
}

//=================================================================================================

NvGePoint2d NvGeSplineEnt2d::controlPointAt (int theIdx) const
{
  const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "controlPointAt");
  CheckIndex (theIdx, aSpline->NbPoles(), "controlPointAt", "the control point index");
  const gp_Pnt2d aPnt = aSpline->Pole (theIdx + 1);
  return NvGePoint2d (aPnt.X(), aPnt.Y());
}

//=================================================================================================

NvGeSplineEnt2d& NvGeSplineEnt2d::setControlPointAt (int theIdx, const NvGePoint2d& thePnt)
{
  {
    const occ::handle<Geom2d_BSplineCurve> aSpline = SplineOf (mpImpEnt, "setControlPointAt");
    CheckIndex (theIdx, aSpline->NbPoles(), "setControlPointAt", "the control point index");
  }
  occ::handle<Geom2d_BSplineCurve> aSpline = EditableSplineOf (mpImpEnt, "setControlPointAt");
  aSpline->SetPole (theIdx + 1, gp_Pnt2d (thePnt.x, thePnt.y));
  return *this;
}

//=================================================================================================

NvGeSplineEnt2d& NvGeSplineEnt2d::operator = (const NvGeSplineEnt2d& theSpline)
{
  NvGeEntity2d::operator= (theSpline);
  return *this;
}

//=================================================================================================

NvGeSplineEnt2d::NvGeSplineEnt2d ()
{
}

//=================================================================================================

NvGeSplineEnt2d::NvGeSplineEnt2d (const NvGeSplineEnt2d& theSrc)
: NvGeCurve2d (theSrc)
{
}
