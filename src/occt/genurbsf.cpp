// genurbsf.cpp - implementation of NvGeNurbSurface based on OCCT
// Geom_BSplineSurface.
//
// The OCCT surface is stored 1:1 inside the entity impl (see geimpdata.h).
// Modifiers follow the copy-on-write discipline: they build a NEW surface
// object (never mutating the possibly shared one) and replace the impl
// geometry after detaching a shared impl.
//
// Knot convention: NvGeKnotVector is FLAT - every knot is repeated according
// to its multiplicity - exactly like Geom_BSplineSurface::UKnotSequence() /
// VKnotSequence(). Distinct knots and their multiplicities are recovered by
// grouping consecutive equal values within NvGeKnotVector::tolerance().
//
// Pole grid convention: the flat control point array is U-major, i.e.
// controlPoints[u * numControlPointsInV + v] is the pole of index (u, v).
// getControlPoints / getWeights produce the same layout.
//
// Properties: the ctor honors the kPeriodic and kRational bits of
// propsInU / propsInV (NvGe::NurbSurfaceProperties); the kOpen bit is
// implicit and the remaining bits are ignored on input. The surface is
// rational in a direction when the corresponding kRational bit is set or
// when weights are given; missing weights are synthesized as unit weights.
//
// Singular knots: singularityInU / singularityInV return the 0-based index
// into the distinct knot list of the first knot whose multiplicity reaches
// the degree (a C0-only seam), or -1 when no such knot exists. The clamping
// ends of a non-periodic surface are not considered.
//
// Boolean-returning methods report unavailable data with kFalse; methods
// returning a value throw NvException with the method name.

#include <genurbsf.h>
#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gedblar.h>
#include <gekvec.h>
#include <gepnt3d.h>
#include <gept3dar.h>
#include <getol.h>

#include <Geom_BSplineSurface.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <string>
#include <vector>

namespace
{

// The stored B-spline surface; throws when the impl holds something else.
occ::handle<Geom_BSplineSurface> SplineOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_BSplineSurface> aSpline =
    occ::down_cast<Geom_BSplineSurface> (NvGeSurfaceOf (theImp));
  if (aSpline.IsNull())
  {
    throw NvException ("NvGeNurbSurface: the entity does not hold a B-spline surface");
  }
  return aSpline;
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom_BSplineSurface>& theSpline)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theSpline);
}

// Distinct knots with multiplicities, grouped within theKnots.tolerance().
struct KnotStructure
{
  std::vector<double> Knots;
  std::vector<int>    Mults;
};

KnotStructure KnotsAndMults (const NvGeKnotVector& theKnots)
{
  KnotStructure aRes;
  const double aTol = theKnots.tolerance();
  for (int i = 0; i < theKnots.length(); ++i)
  {
    if (!aRes.Knots.empty() && std::abs (theKnots[i] - aRes.Knots.back()) <= aTol)
    {
      ++aRes.Mults.back();
    }
    else
    {
      aRes.Knots.push_back (theKnots[i]);
      aRes.Mults.push_back (1);
    }
  }
  return aRes;
}

// Flat knot vector (knots repeated per multiplicity), NvGeKnotVector form.
NvGeKnotVector FlatKnotVector (const NCollection_Array1<double>& theSequence)
{
  NvGeDoubleArray aFlat;
  for (int i = theSequence.Lower(); i <= theSequence.Upper(); ++i)
  {
    aFlat.append (theSequence.Value (i));
  }
  return NvGeKnotVector (aFlat);
}

// Complete B-spline surface definition used for rebuilds. The pole grid is
// U-major: Poles[u * numV + v].
struct SurfaceDefinition
{
  std::vector<gp_Pnt> Poles;
  std::vector<double> Weights;      // empty = non-rational
  std::vector<double> UKnots;       // distinct
  std::vector<int>    UMults;
  std::vector<double> VKnots;       // distinct
  std::vector<int>    VMults;
  int                 NumU = 2;
  int                 NumV = 2;
  int                 UDegree = 1;
  int                 VDegree = 1;
  bool                UPeriodic = false;
  bool                VPeriodic = false;
};

// Validates one direction (knot multiplicities and the pole count derived
// from the knot vector); throws NvException with theCtx on any
// inconsistency. thePoleCount is the number of poles in this direction.
void ValidateDirection (const std::vector<int>& theMults,
                        int                     theDegree,
                        bool                    thePeriodic,
                        int                     thePoleCount,
                        const std::string&      theCtx)
{
  for (size_t i = 0; i < theMults.size(); ++i)
  {
    const bool anEnd = (i == 0 || i == theMults.size() - 1) && !thePeriodic;
    const int aMaxMult = anEnd ? theDegree + 1 : theDegree;
    if (theMults[i] < 1 || theMults[i] > aMaxMult)
    {
      throw NvException (theCtx + "a knot multiplicity is out of range");
    }
  }
  if (thePeriodic && theMults.front() != theMults.back())
  {
    throw NvException (theCtx + "the first and last multiplicities must match"
                              " on a periodic surface");
  }
  long long aSum = 0;
  for (size_t i = 0; i < theMults.size(); ++i)
  {
    aSum += theMults[i];
  }
  const long long anExpected = thePeriodic
                              ? aSum - theMults.front()
                              : aSum - theDegree - 1;
  if (static_cast<long long> (thePoleCount) != anExpected)
  {
    throw NvException (theCtx + "the control point count does not match the"
                              " knot vector");
  }
}

// Validates theDef and builds a new B-spline surface from it; throws
// NvException with theMethod context on any inconsistency or on an OCCT
// rejection.
occ::handle<Geom_BSplineSurface> SplineFromDefinition (const SurfaceDefinition& theDef,
                                                       const char*              theMethod)
{
  const std::string aCtx = std::string ("NvGeNurbSurface::") + theMethod + "(): ";
  if (theDef.UDegree < 1 || theDef.UDegree > Geom_BSplineSurface::MaxDegree()
      || theDef.VDegree < 1 || theDef.VDegree > Geom_BSplineSurface::MaxDegree())
  {
    throw NvException (aCtx + "a degree is out of range");
  }
  if (theDef.UKnots.size() < 2 || theDef.UKnots.size() != theDef.UMults.size()
      || theDef.VKnots.size() < 2 || theDef.VKnots.size() != theDef.VMults.size())
  {
    throw NvException (aCtx + "a knot vector is degenerate");
  }
  ValidateDirection (theDef.UMults, theDef.UDegree, theDef.UPeriodic, theDef.NumU, aCtx);
  ValidateDirection (theDef.VMults, theDef.VDegree, theDef.VPeriodic, theDef.NumV, aCtx);
  if (!theDef.Weights.empty()
      && theDef.Weights.size() != static_cast<size_t> (theDef.NumU) * static_cast<size_t> (theDef.NumV))
  {
    throw NvException (aCtx + "the weight count does not match the control point count");
  }
  for (size_t i = 0; i < theDef.Weights.size(); ++i)
  {
    if (theDef.Weights[i] <= gp::Resolution())
    {
      throw NvException (aCtx + "a weight is not positive");
    }
  }
  NCollection_Array2<gp_Pnt> aPoles (1, theDef.NumU, 1, theDef.NumV);
  for (int u = 1; u <= theDef.NumU; ++u)
  {
    for (int v = 1; v <= theDef.NumV; ++v)
    {
      aPoles.SetValue (u, v, theDef.Poles[static_cast<size_t> ((u - 1) * theDef.NumV + (v - 1))]);
    }
  }
  NCollection_Array1<double> aUKnots (1, static_cast<int> (theDef.UKnots.size()));
  NCollection_Array1<int> aUMults (1, static_cast<int> (theDef.UMults.size()));
  for (size_t i = 0; i < theDef.UKnots.size(); ++i)
  {
    aUKnots.SetValue (static_cast<int> (i) + 1, theDef.UKnots[i]);
    aUMults.SetValue (static_cast<int> (i) + 1, theDef.UMults[i]);
  }
  NCollection_Array1<double> aVKnots (1, static_cast<int> (theDef.VKnots.size()));
  NCollection_Array1<int> aVMults (1, static_cast<int> (theDef.VMults.size()));
  for (size_t i = 0; i < theDef.VKnots.size(); ++i)
  {
    aVKnots.SetValue (static_cast<int> (i) + 1, theDef.VKnots[i]);
    aVMults.SetValue (static_cast<int> (i) + 1, theDef.VMults[i]);
  }
  try
  {
    if (theDef.Weights.empty())
    {
      return occ::handle<Geom_BSplineSurface> (
        new Geom_BSplineSurface (aPoles, aUKnots, aVKnots, aUMults, aVMults,
                                 theDef.UDegree, theDef.VDegree,
                                 theDef.UPeriodic, theDef.VPeriodic));
    }
    NCollection_Array2<double> aWeights (1, theDef.NumU, 1, theDef.NumV);
    for (int u = 1; u <= theDef.NumU; ++u)
    {
      for (int v = 1; v <= theDef.NumV; ++v)
      {
        aWeights.SetValue (u, v,
                           theDef.Weights[static_cast<size_t> ((u - 1) * theDef.NumV + (v - 1))]);
      }
    }
    return occ::handle<Geom_BSplineSurface> (
      new Geom_BSplineSurface (aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults,
                               theDef.UDegree, theDef.VDegree,
                               theDef.UPeriodic, theDef.VPeriodic));
  }
  catch (const Standard_Failure& aFailure)
  {
    throw NvException::FromFailure (aFailure);
  }
}

// Fills theDef from the public ctor / set() arguments; validates the pole
// grid and packs poles (U-major), weights and grouped knots. Throws
// NvException with the method context on any inconsistency.
void FillDefinition (SurfaceDefinition&    theDef,
                     int                   theDegreeU,
                     int                   theDegreeV,
                     int                   thePropsInU,
                     int                   thePropsInV,
                     int                   theNumU,
                     int                   theNumV,
                     const NvGePoint3d     theControlPoints[],
                     const double          theWeights[],
                     const NvGeKnotVector& theUKnots,
                     const NvGeKnotVector& theVKnots,
                     const char*           theMethod)
{
  const std::string aCtx = std::string ("NvGeNurbSurface::") + theMethod + "(): ";
  if (theNumU < 2 || theNumV < 2)
  {
    throw NvException (aCtx + "the control point grid must have at least two"
                              " control points in each direction");
  }
  if (theControlPoints == nullptr)
  {
    throw NvException (aCtx + "the control points are missing");
  }
  theDef.NumU = theNumU;
  theDef.NumV = theNumV;
  theDef.UDegree = theDegreeU;
  theDef.VDegree = theDegreeV;
  theDef.UPeriodic = (thePropsInU & NvGe::kPeriodic) != 0;
  theDef.VPeriodic = (thePropsInV & NvGe::kPeriodic) != 0;
  const size_t aPoleCount = static_cast<size_t> (theNumU) * static_cast<size_t> (theNumV);
  theDef.Poles.reserve (aPoleCount);
  for (int u = 0; u < theNumU; ++u)
  {
    for (int v = 0; v < theNumV; ++v)
    {
      const NvGePoint3d& aPnt = theControlPoints[u * theNumV + v];
      theDef.Poles.push_back (gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
    }
  }
  const bool aRational = (thePropsInU & NvGe::kRational) != 0
                         || (thePropsInV & NvGe::kRational) != 0
                         || theWeights != nullptr;
  if (aRational)
  {
    theDef.Weights.reserve (aPoleCount);
    for (int u = 0; u < theNumU; ++u)
    {
      for (int v = 0; v < theNumV; ++v)
      {
        theDef.Weights.push_back (theWeights != nullptr ? theWeights[u * theNumV + v] : 1.0);
      }
    }
  }
  const KnotStructure aU = KnotsAndMults (theUKnots);
  theDef.UKnots = aU.Knots;
  theDef.UMults = aU.Mults;
  const KnotStructure aV = KnotsAndMults (theVKnots);
  theDef.VKnots = aV.Knots;
  theDef.VMults = aV.Mults;
}

} // namespace

//=================================================================================================

NvGeNurbSurface::NvGeNurbSurface ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  // A valid degree-1 patch collapsed onto the origin (2 x 2 poles, knots 0..1).
  SurfaceDefinition aDef;
  aDef.Poles.assign (4, gp_Pnt (0.0, 0.0, 0.0));
  aDef.UKnots = {0.0, 1.0};
  aDef.UMults = {2, 2};
  aDef.VKnots = {0.0, 1.0};
  aDef.VMults = {2, 2};
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbSurface,
                                  SplineFromDefinition (aDef, "NvGeNurbSurface"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbSurface::NvGeNurbSurface (int theDegreeU, int theDegreeV,
                                  int thePropsInU, int thePropsInV,
                                  int theNumControlPointsInU, int theNumControlPointsInV,
                                  const NvGePoint3d theControlPoints[],
                                  const double theWeights[],
                                  const NvGeKnotVector& theUKnots,
                                  const NvGeKnotVector& theVKnots,
                                  const NvGeTol& theTol)
{
  (void) theTol;   // multiplicities are explicit in the knot vectors
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  SurfaceDefinition aDef;
  FillDefinition (aDef, theDegreeU, theDegreeV, thePropsInU, thePropsInV,
                  theNumControlPointsInU, theNumControlPointsInV,
                  theControlPoints, theWeights, theUKnots, theVKnots,
                  "NvGeNurbSurface");
  mpImpEnt = new NvGeImpEntity3d (NvGe::kNurbSurface,
                                  SplineFromDefinition (aDef, "NvGeNurbSurface"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeNurbSurface::NvGeNurbSurface (const NvGeNurbSurface& theNurb)
: NvGeSurface (theNurb)
{
}

//=================================================================================================

NvGeNurbSurface& NvGeNurbSurface::operator = (const NvGeNurbSurface& theNurb)
{
  NvGeSurface::operator= (theNurb);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeNurbSurface::isRationalInU () const
{
  return SplineOf (mpImpEnt)->IsURational() ? Adesk::kTrue : Adesk::kFalse;
}

//=================================================================================================

Adesk::Boolean NvGeNurbSurface::isPeriodicInU (double& thePeriod) const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  if (!aSpline->IsUPeriodic())
  {
    thePeriod = 0.0;
    return Adesk::kFalse;
  }
  const NCollection_Array1<double>& aKnots = aSpline->UKnots();
  thePeriod = aKnots.Value (aKnots.Upper()) - aKnots.Value (aKnots.Lower());
  return Adesk::kTrue;
}

//=================================================================================================

Adesk::Boolean NvGeNurbSurface::isRationalInV () const
{
  return SplineOf (mpImpEnt)->IsVRational() ? Adesk::kTrue : Adesk::kFalse;
}

//=================================================================================================

Adesk::Boolean NvGeNurbSurface::isPeriodicInV (double& thePeriod) const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  if (!aSpline->IsVPeriodic())
  {
    thePeriod = 0.0;
    return Adesk::kFalse;
  }
  const NCollection_Array1<double>& aKnots = aSpline->VKnots();
  thePeriod = aKnots.Value (aKnots.Upper()) - aKnots.Value (aKnots.Lower());
  return Adesk::kTrue;
}

//=================================================================================================

int NvGeNurbSurface::singularityInU () const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  const int aFirst = aSpline->IsUPeriodic() ? 1 : 2;
  const int aLast = aSpline->IsUPeriodic() ? aSpline->NbUKnots() : aSpline->NbUKnots() - 1;
  for (int i = aFirst; i <= aLast; ++i)
  {
    if (aSpline->UMultiplicity (i) >= aSpline->UDegree())
    {
      return i - 1;
    }
  }
  return -1;
}

//=================================================================================================

int NvGeNurbSurface::singularityInV () const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  const int aFirst = aSpline->IsVPeriodic() ? 1 : 2;
  const int aLast = aSpline->IsVPeriodic() ? aSpline->NbVKnots() : aSpline->NbVKnots() - 1;
  for (int i = aFirst; i <= aLast; ++i)
  {
    if (aSpline->VMultiplicity (i) >= aSpline->VDegree())
    {
      return i - 1;
    }
  }
  return -1;
}

//=================================================================================================

int NvGeNurbSurface::degreeInU () const
{
  return SplineOf (mpImpEnt)->UDegree();
}

//=================================================================================================

int NvGeNurbSurface::numControlPointsInU () const
{
  return SplineOf (mpImpEnt)->NbUPoles();
}

//=================================================================================================

int NvGeNurbSurface::degreeInV () const
{
  return SplineOf (mpImpEnt)->VDegree();
}

//=================================================================================================

int NvGeNurbSurface::numControlPointsInV () const
{
  return SplineOf (mpImpEnt)->NbVPoles();
}

//=================================================================================================

void NvGeNurbSurface::getControlPoints (NvGePoint3dArray& thePoints) const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  thePoints.removeAll();
  for (int u = 1; u <= aSpline->NbUPoles(); ++u)
  {
    for (int v = 1; v <= aSpline->NbVPoles(); ++v)
    {
      const gp_Pnt& aPnt = aSpline->Pole (u, v);
      thePoints.append (NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z()));
    }
  }
}

//=================================================================================================

Adesk::Boolean NvGeNurbSurface::getWeights (NvGeDoubleArray& theWeights) const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  theWeights.removeAll();
  if (!aSpline->IsURational() && !aSpline->IsVRational())
  {
    return Adesk::kFalse;
  }
  for (int u = 1; u <= aSpline->NbUPoles(); ++u)
  {
    for (int v = 1; v <= aSpline->NbVPoles(); ++v)
    {
      theWeights.append (aSpline->Weight (u, v));
    }
  }
  return Adesk::kTrue;
}

//=================================================================================================

int NvGeNurbSurface::numKnotsInU () const
{
  return SplineOf (mpImpEnt)->UKnotSequence().Length();
}

//=================================================================================================

void NvGeNurbSurface::getUKnots (NvGeKnotVector& theUKnots) const
{
  theUKnots = FlatKnotVector (SplineOf (mpImpEnt)->UKnotSequence());
}

//=================================================================================================

int NvGeNurbSurface::numKnotsInV () const
{
  return SplineOf (mpImpEnt)->VKnotSequence().Length();
}

//=================================================================================================

void NvGeNurbSurface::getVKnots (NvGeKnotVector& theVKnots) const
{
  theVKnots = FlatKnotVector (SplineOf (mpImpEnt)->VKnotSequence());
}

//=================================================================================================

void NvGeNurbSurface::getDefinition (int& theDegreeU, int& theDegreeV,
                                     int& thePropsInU, int& thePropsInV,
                                     int& theNumControlPointsInU,
                                     int& theNumControlPointsInV,
                                     NvGePoint3dArray& theControlPoints,
                                     NvGeDoubleArray& theWeights,
                                     NvGeKnotVector& theUKnots,
                                     NvGeKnotVector& theVKnots) const
{
  const occ::handle<Geom_BSplineSurface> aSpline = SplineOf (mpImpEnt);
  int aPropsU = NvGe::kOpen;
  if (aSpline->IsUClosed())
  {
    aPropsU |= NvGe::kClosed;
  }
  if (aSpline->IsUPeriodic())
  {
    aPropsU |= NvGe::kPeriodic;
  }
  if (aSpline->IsURational())
  {
    aPropsU |= NvGe::kRational;
  }
  thePropsInU = aPropsU;
  int aPropsV = NvGe::kOpen;
  if (aSpline->IsVClosed())
  {
    aPropsV |= NvGe::kClosed;
  }
  if (aSpline->IsVPeriodic())
  {
    aPropsV |= NvGe::kPeriodic;
  }
  if (aSpline->IsVRational())
  {
    aPropsV |= NvGe::kRational;
  }
  thePropsInV = aPropsV;
  theDegreeU = aSpline->UDegree();
  theDegreeV = aSpline->VDegree();
  theNumControlPointsInU = aSpline->NbUPoles();
  theNumControlPointsInV = aSpline->NbVPoles();
  getControlPoints (theControlPoints);
  getWeights (theWeights);
  getUKnots (theUKnots);
  getVKnots (theVKnots);
}

//=================================================================================================

NvGeNurbSurface& NvGeNurbSurface::set (int theDegreeU, int theDegreeV,
                                       int thePropsInU, int thePropsInV,
                                       int theNumControlPointsInU, int theNumControlPointsInV,
                                       const NvGePoint3d theControlPoints[],
                                       const double theWeights[],
                                       const NvGeKnotVector& theUKnots,
                                       const NvGeKnotVector& theVKnots,
                                       const NvGeTol& theTol)
{
  (void) theTol;   // multiplicities are explicit in the knot vectors
  SurfaceDefinition aDef;
  FillDefinition (aDef, theDegreeU, theDegreeV, thePropsInU, thePropsInV,
                  theNumControlPointsInU, theNumControlPointsInV,
                  theControlPoints, theWeights, theUKnots, theVKnots, "set");
  ReplaceGeometry (mpImpEnt, SplineFromDefinition (aDef, "set"));
  return *this;
}
