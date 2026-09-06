// gesent3d.cpp - implementation of NvGeSplineEnt3d.
//
// The entity stores its geometry as a Geom_BSplineCurve. The accessors map
// onto the OCCT curve data (poles, knots, multiplicities, parametrization);
// the editing methods work on a private copy of the curve so that entities
// sharing an implementation stay unaffected (copy-on-write).

#include <gesent3d.h>
#include <Nova.h>
#include <NvException.h>
#include <gekvec.h>
#include <gepnt3d.h>
#include <getol.h>
#include <geimpdata.h>

#include <Geom_BSplineCurve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>

#include <vector>

namespace
{

// Returns the stored B-spline curve.
occ::handle<Geom_BSplineCurve> SplineOf (const occ::handle<Standard_Transient>& theGeom)
{
  if (theGeom.IsNull())
  {
    throw NvException ("NvGeSplineEnt3d: null curve data");
  }
  const occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (theGeom);
  if (aSpline.IsNull())
  {
    throw NvException ("NvGeSplineEnt3d: the entity does not store a B-spline curve");
  }
  return aSpline;
}

// Validates the 1-based index against the item count.
void CheckIndex (int theIdx, int theCount, const char* theMessage)
{
  if (theIdx < 1 || theIdx > theCount)
  {
    throw NvException (theMessage);
  }
}

// Degenerate placeholder curve of the default constructor: two coincident
// poles at the origin, a degree 1 curve over the knots 0 and 1.
occ::handle<Geom_Curve> DegenerateSplineCurve ()
{
  NCollection_Array1<gp_Pnt> aPoles (1, 2);
  aPoles.SetValue (1, gp_Pnt (0.0, 0.0, 0.0));
  aPoles.SetValue (2, gp_Pnt (0.0, 0.0, 0.0));
  NCollection_Array1<double> aKnots (1, 2);
  aKnots.SetValue (1, 0.0);
  aKnots.SetValue (2, 1.0);
  NCollection_Array1<int> aMults (1, 2);
  aMults.SetValue (1, 2);
  aMults.SetValue (2, 2);
  return occ::handle<Geom_Curve> (new Geom_BSplineCurve (aPoles, aKnots, aMults, 1));
}

}

// NvGeSplineEnt3d

//=======================================================================
// function : isRational
// purpose  : True when the stored curve carries weights.
//=======================================================================
Nova::Boolean NvGeSplineEnt3d::isRational () const
{
  return SplineOf (mpImpEnt->Geom())->IsRational();
}

//=======================================================================
// function : degree
// purpose  : Degree of the stored curve.
//=======================================================================
int NvGeSplineEnt3d::degree () const
{
  return SplineOf (mpImpEnt->Geom())->Degree();
}

//=======================================================================
// function : order
// purpose  : Degree plus one.
//=======================================================================
int NvGeSplineEnt3d::order () const
{
  return SplineOf (mpImpEnt->Geom())->Degree() + 1;
}

//=======================================================================
// function : numKnots
// purpose  : Number of distinct knots.
//=======================================================================
int NvGeSplineEnt3d::numKnots () const
{
  return SplineOf (mpImpEnt->Geom())->NbKnots();
}

//=======================================================================
// function : knots
// purpose  : The knot values materialized from the stored curve; the
//            returned reference stays valid until the next knots() call on
//            any spline entity of this thread.
//=======================================================================
const NvGeKnotVector& NvGeSplineEnt3d::knots () const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  static thread_local NvGeKnotVector aKnots;
  const int aNbKnots = aSpline->NbKnots();
  std::vector<double> aValues (aNbKnots);
  for (int aKnot = 1; aKnot <= aNbKnots; ++aKnot)
  {
    aValues[aKnot - 1] = aSpline->Knot (aKnot);
  }
  aKnots.set (aNbKnots, aValues.data());
  return aKnots;
}

//=======================================================================
// function : numControlPoints
// purpose  : Number of control points (poles).
//=======================================================================
int NvGeSplineEnt3d::numControlPoints () const
{
  return SplineOf (mpImpEnt->Geom())->NbPoles();
}

//=======================================================================
// function : continuityAtKnot
// purpose  : Degree minus the multiplicity at the 1-based knot index.
//=======================================================================
int NvGeSplineEnt3d::continuityAtKnot (int theIdx, const NvGeTol&) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbKnots(),
              "NvGeSplineEnt3d::continuityAtKnot(): the knot index is out of range");
  return aSpline->Degree() - aSpline->Multiplicity (theIdx);
}

//=======================================================================
// function : startParam
// purpose  : First parameter of the stored curve.
//=======================================================================
double NvGeSplineEnt3d::startParam () const
{
  return SplineOf (mpImpEnt->Geom())->FirstParameter();
}

//=======================================================================
// function : endParam
// purpose  : Last parameter of the stored curve.
//=======================================================================
double NvGeSplineEnt3d::endParam () const
{
  return SplineOf (mpImpEnt->Geom())->LastParameter();
}

//=======================================================================
// function : startPoint
// purpose  : Point at the start parameter.
//=======================================================================
NvGePoint3d NvGeSplineEnt3d::startPoint () const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  const gp_Pnt aPnt = aSpline->EvalD0 (aSpline->FirstParameter());
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : endPoint
// purpose  : Point at the end parameter.
//=======================================================================
NvGePoint3d NvGeSplineEnt3d::endPoint () const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  const gp_Pnt aPnt = aSpline->EvalD0 (aSpline->LastParameter());
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : hasFitData
// purpose  : True when the curve is stored in the fit-point form; the
//            wrapper maps that form onto the degree 1 B-spline whose poles
//            are the fit points, so the degree decides.
//=======================================================================
Nova::Boolean NvGeSplineEnt3d::hasFitData () const
{
  return SplineOf (mpImpEnt->Geom())->Degree() == 1;
}

//=======================================================================
// function : knotAt
// purpose  : The 1-based knot value.
//=======================================================================
double NvGeSplineEnt3d::knotAt (int theIdx) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbKnots(),
              "NvGeSplineEnt3d::knotAt(): the knot index is out of range");
  return aSpline->Knot (theIdx);
}

//=======================================================================
// function : setKnotAt
// purpose  : Moves the 1-based knot on a private copy of the curve; the
//            knot order must stay increasing.
//=======================================================================
NvGeSplineEnt3d& NvGeSplineEnt3d::setKnotAt (int theIdx, double theVal)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbKnots(),
              "NvGeSplineEnt3d::setKnotAt(): the knot index is out of range");
  const occ::handle<Geom_BSplineCurve> aCopy =
    occ::down_cast<Geom_BSplineCurve> (aSpline->Copy());
  try
  {
    aCopy->SetKnot (theIdx, theVal);
  }
  catch (const Standard_Failure&)
  {
    throw NvException (
      "NvGeSplineEnt3d::setKnotAt(): the knot value violates the knot order");
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (aCopy));
  return *this;
}

//=======================================================================
// function : controlPointAt
// purpose  : The 1-based control point (pole).
//=======================================================================
NvGePoint3d NvGeSplineEnt3d::controlPointAt (int theIdx) const
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbPoles(),
              "NvGeSplineEnt3d::controlPointAt(): the control point index is out of range");
  const gp_Pnt& aPole = aSpline->Pole (theIdx);
  return NvGePoint3d (aPole.X(), aPole.Y(), aPole.Z());
}

//=======================================================================
// function : setControlPointAt
// purpose  : Moves the 1-based control point on a private copy of the
//            curve.
//=======================================================================
NvGeSplineEnt3d& NvGeSplineEnt3d::setControlPointAt (int theIdx, const NvGePoint3d& thePnt)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbPoles(),
              "NvGeSplineEnt3d::setControlPointAt(): the control point index is out of range");
  const occ::handle<Geom_BSplineCurve> aCopy =
    occ::down_cast<Geom_BSplineCurve> (aSpline->Copy());
  aCopy->SetPole (theIdx, gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (occ::handle<Geom_Curve> (aCopy));
  return *this;
}

//=======================================================================
// function : operator =
// purpose  : Assignment; delegates the implementation sharing to the base.
//=======================================================================
NvGeSplineEnt3d& NvGeSplineEnt3d::operator = (const NvGeSplineEnt3d& theSpline)
{
  NvGeEntity3d::operator = (theSpline);
  return *this;
}

//=======================================================================
// function : NvGeSplineEnt3d
// purpose  : Degenerate placeholder: a degree 1 curve over two coincident
//            poles at the origin.
//=======================================================================
NvGeSplineEnt3d::NvGeSplineEnt3d ()
{
  const occ::handle<Geom_Curve> aCurve = DegenerateSplineCurve();
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kSplineEnt3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeSplineEnt3d
// purpose  : Copy constructor; shares the implementation of the source.
//=======================================================================
NvGeSplineEnt3d::NvGeSplineEnt3d (const NvGeSplineEnt3d& theSrc)
: NvGeCurve3d (theSrc)
{

}
