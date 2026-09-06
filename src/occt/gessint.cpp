// gessint.cpp - implementation of NvGeSurfSurfInt.
//
// The intersection entity is an empty NvGeEntity3d shell whose impl carries
// a file-local NvGeSurfSurfIntData holder: the input surface impls, the
// tolerance and the eager intersection results. The constructor and set()
// run GeomAPI_IntSS eagerly and fill the holder; every query only reads it
// and reports failures through the NvGeIntersectError channel (ARX
// semantics for this class), not through exceptions.
//
// GeomAPI_IntSS yields the intersection curves as 3d curves only, so every
// result has dimension 1; the point-result queries (intPoint,
// getIntPointParams) report kXXWrongDimensionAtIndex. The parametric curves
// are computed on demand from the 3d curve through GeomProjLib::Curve2d.
// The ssiType of each result is classified by comparing the surface normals
// at samples along the line (dot ~ +1 tangent, ~ -1 anti-tangent, else
// transverse); the ssiConfig quadruple classifies the neighborhood of each
// surface to the left / right of the line with respect to the other
// surface through offset sample projections.

#include <gessint.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <gecurv3d.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geimpdata.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <gesurf.h>
#include <getol.h>

#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomProjLib.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
//! Documented layout bridge used across the ge layer.
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Number of samples used to classify one intersection line.
constexpr int THE_NB_SAMPLES = 9;

//! Offset distance for the neighborhood projections, in model units.
constexpr double THE_NEIGHBORHOOD_STEP = 1e-4;

//! Per-result classification of one intersection component.
class NvGeSurfSurfIntResult
{
public:

  occ::handle<Geom_Curve> Line;                    // 3d intersection curve
  int                     Dim = 1;                 // always 1 (curve result)
  NvGe::ssiType           Type = NvGe::kSSITransverse;
  NvGe::ssiConfig         S1Left = NvGe::kSSIUnknown;
  NvGe::ssiConfig         S1Right = NvGe::kSSIUnknown;
  NvGe::ssiConfig         S2Left = NvGe::kSSIUnknown;
  NvGe::ssiConfig         S2Right = NvGe::kSSIUnknown;
};

//! Storage behind NvGeSurfSurfInt: inputs and eager results.
class NvGeSurfSurfIntData : public NvGeEntityData
{
public:

  occ::handle<NvGeImpEntity3d> Surface1;
  occ::handle<NvGeImpEntity3d> Surface2;
  NvGeTol                      Tol;
  bool                         Failed = false; // algorithm did not complete

  std::vector<NvGeSurfSurfIntResult> Results;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeSurfSurfIntData> aCopy = new NvGeSurfSurfIntData();
    aCopy->Surface1 = Surface1;
    aCopy->Surface2 = Surface2;
    aCopy->Tol = Tol;
    aCopy->Failed = Failed;
    aCopy->Results = Results; // handles are shared, immutable by convention
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeSurfSurfIntData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeSurfSurfIntData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeSurfSurfIntData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeSurfSurfIntData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeSurfSurfIntData> (theImp->Geom());
}

//! Validates the error channel and the query index; returns false when the
//! query must not read the holder (the error code is set).
bool CheckQuery (const NvGeSurfSurfIntData* theData, int theIntNum, NvGeIntersectError& theErr)
{
  if (theData->Failed)
  {
    theErr = NvGe::kXXUnknown;
    return false;
  }
  if (theIntNum < 0 || theIntNum >= static_cast<int> (theData->Results.size()))
  {
    theErr = NvGe::kXXIndexOutOfRange;
    return false;
  }
  theErr = NvGe::kXXOk;
  return true;
}

//! Maps an OCCT parametric bound to a plain double.
double PlainParam (const double theParam)
{
  if (Precision::IsPositiveInfinite (theParam))
  {
    return std::numeric_limits<double>::infinity();
  }
  if (Precision::IsNegativeInfinite (theParam))
  {
    return -std::numeric_limits<double>::infinity();
  }
  return theParam;
}

//! Finite sampling window of a line: huge-but-finite domains (OCCT stores
//! line domains as +/-Precision::Infinite()) are clamped to a local window.
void WindowOf (const occ::handle<Geom_Curve>& theCurve, double& theFirst, double& theLast)
{
  theFirst = PlainParam (theCurve->FirstParameter());
  theLast = PlainParam (theCurve->LastParameter());
  if (!std::isfinite (theFirst) || !std::isfinite (theLast) || theLast - theFirst > 1e3)
  {
    const double aMid = 0.5 * (std::isfinite (theFirst + theLast) ? theFirst + theLast : 0.0);
    theFirst = aMid - 1.0;
    theLast = aMid + 1.0;
  }
  if (!(theLast > theFirst))
  {
    theFirst = -1.0;
    theLast = 1.0;
  }
}

//! Unit surface normal at a point; false when the normal is undefined.
bool NormalOf (const occ::handle<Geom_Surface>& theSurface, const gp_Pnt& thePoint,
               const NvGeTol& theTol, gp_Pnt& theFootPoint, gp_Vec& theNormal)
{
  GeomAPI_ProjectPointOnSurf aProjector (thePoint, theSurface);
  if (aProjector.NbPoints() == 0)
  {
    return false;
  }
  double anU = 0.0;
  double aV = 0.0;
  aProjector.LowerDistanceParameters (anU, aV);
  GeomLProp_SLProps aProps (theSurface, anU, aV, 1, theTol.equalPoint());
  if (!aProps.IsNormalDefined())
  {
    return false;
  }
  theFootPoint = aProjector.NearestPoint();
  // The normal comes back as a gp_Dir; convert for gp_Vec arithmetic.
  theNormal = gp_Vec (aProps.Normal().XYZ());
  return true;
}

//! Signed distance of a point from the surface along its normal
//! (> 0 outside, < 0 inside); 0 when the side cannot be determined.
double SignedSideOf (const occ::handle<Geom_Surface>& theSurface, const gp_Pnt& thePoint,
                     const NvGeTol& theTol)
{
  gp_Pnt aFoot;
  gp_Vec aNormal;
  if (!NormalOf (theSurface, thePoint, theTol, aFoot, aNormal))
  {
    return 0.0;
  }
  gp_Vec aRel (thePoint.XYZ() - aFoot.XYZ());
  return aRel.Dot (aNormal);
}

//! Maps a signed neighborhood distance to its configuration.
NvGe::ssiConfig ConfigOf (double theSigned, double theTol)
{
  if (theSigned > theTol)
  {
    return NvGe::kSSIOut;
  }
  if (theSigned < -theTol)
  {
    return NvGe::kSSIIn;
  }
  return NvGe::kSSICoincident;
}

//! Classifies one intersection line: the ssiType from the normal dots along
//! the line and the left/right neighborhood configurations of both
//! surfaces, each with respect to the other surface.
NvGeSurfSurfIntResult ClassifyLine (const occ::handle<Geom_Curve>& theLine,
                                    const occ::handle<Geom_Surface>& theSurface1,
                                    const occ::handle<Geom_Surface>& theSurface2,
                                    const NvGeTol& theTol)
{
  NvGeSurfSurfIntResult aResult;
  aResult.Line = theLine;

  double aFirst = 0.0;
  double aLast = 0.0;
  WindowOf (theLine, aFirst, aLast);

  gp_Vec aTangent;
  gp_Vec aNormal1;
  gp_Vec aNormal2;
  gp_Pnt aMidPoint;
  bool aHasMidSample = false;

  int aNbValid = 0;
  int aNbPositive = 0;
  int aNbNegative = 0;
  for (int aSample = 0; aSample < THE_NB_SAMPLES; ++aSample)
  {
    const double aParam = aFirst + (aLast - aFirst) * aSample / (THE_NB_SAMPLES - 1);
    const gp_Pnt aPoint = theLine->Value (aParam);
    gp_Pnt aFoot1;
    gp_Pnt aFoot2;
    gp_Vec aNorm1;
    gp_Vec aNorm2;
    if (!NormalOf (theSurface1, aPoint, theTol, aFoot1, aNorm1)
      || !NormalOf (theSurface2, aPoint, theTol, aFoot2, aNorm2))
    {
      continue;
    }
    const double aDot = aNorm1.Dot (aNorm2);
    ++aNbValid;
    if (aDot >= 0.99)
    {
      ++aNbPositive;
    }
    else if (aDot <= -0.99)
    {
      ++aNbNegative;
    }
    if (aSample == THE_NB_SAMPLES / 2)
    {
      aMidPoint = aPoint;
      aTangent = theLine->DN (aParam, 1);
      aNormal1 = aNorm1;
      aNormal2 = aNorm2;
      aHasMidSample = true;
    }
  }

  // Normal comparison: identical normals along the line mean tangent
  // intersection, opposite normals mean anti-tangent, anything else
  // (including undecidable samples) stays transverse.
  if (aNbValid > 0 && aNbPositive == aNbValid)
  {
    aResult.Type = NvGe::kSSITangent;
  }
  else if (aNbValid > 0 && aNbNegative == aNbValid)
  {
    aResult.Type = NvGe::kSSIAntiTangent;
  }

  if (aHasMidSample && aTangent.Magnitude() > 1e-12)
  {
    aTangent.Divide (aTangent.Magnitude());
    // Walking along the line with the head along the surface normal, the
    // left side is N x T.
    const gp_Vec aLeft1 = aNormal1.Crossed (aTangent);
    const gp_Vec aLeft2 = aNormal2.Crossed (aTangent);
    aResult.S1Left = ConfigOf (SignedSideOf (theSurface2,
                                             aMidPoint.XYZ() + aLeft1.XYZ() * THE_NEIGHBORHOOD_STEP,
                                             theTol),
                               theTol.equalPoint());
    aResult.S1Right = ConfigOf (SignedSideOf (theSurface2,
                                              aMidPoint.XYZ() - aLeft1.XYZ() * THE_NEIGHBORHOOD_STEP,
                                              theTol),
                                theTol.equalPoint());
    aResult.S2Left = ConfigOf (SignedSideOf (theSurface1,
                                             aMidPoint.XYZ() + aLeft2.XYZ() * THE_NEIGHBORHOOD_STEP,
                                             theTol),
                               theTol.equalPoint());
    aResult.S2Right = ConfigOf (SignedSideOf (theSurface1,
                                              aMidPoint.XYZ() - aLeft2.XYZ() * THE_NEIGHBORHOOD_STEP,
                                              theTol),
                                theTol.equalPoint());
  }
  return aResult;
}

//! Runs the eager intersection computation and fills the holder.
void ComputeInto (NvGeSurfSurfIntData& theData)
{
  theData.Failed = false;
  theData.Results.clear();

  const occ::handle<Geom_Surface> aSurface1 = NvGeSurfaceOf (theData.Surface1.get());
  const occ::handle<Geom_Surface> aSurface2 = NvGeSurfaceOf (theData.Surface2.get());
  try
  {
    GeomAPI_IntSS anInter (aSurface1, aSurface2, theData.Tol.equalPoint());
    if (!anInter.IsDone())
    {
      theData.Failed = true;
      return;
    }
    const int aNbLines = anInter.NbLines();
    theData.Results.reserve (aNbLines);
    for (int anIdx = 1; anIdx <= aNbLines; ++anIdx)
    {
      theData.Results.push_back (ClassifyLine (anInter.Line (anIdx), aSurface1, aSurface2,
                                               theData.Tol));
    }
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//! Validates the inputs and installs them into the holder.
void PrepareData (NvGeSurfSurfIntData& theData, const NvGeSurface& theSurface1,
                  const NvGeSurface& theSurface2, const NvGeTol& theTol)
{
  // Validates that both wrappers really carry surface geometry.
  NvGeSurfaceOf (ImplOf (&theSurface1));
  NvGeSurfaceOf (ImplOf (&theSurface2));
  theData.Surface1 = ImplOf (&theSurface1);
  theData.Surface2 = ImplOf (&theSurface2);
  theData.Tol = theTol;
}

} // namespace

//=================================================================================================

NvGeSurfSurfInt::NvGeSurfSurfInt ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kSurfaceSurfaceInt,
                                  occ::handle<NvGeSurfSurfIntData> (new NvGeSurfSurfIntData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeSurfSurfInt::NvGeSurfSurfInt (const NvGeSurface& theSrf1, const NvGeSurface& theSrf2,
                                  const NvGeTol& theTol)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeSurfSurfIntData> aData = new NvGeSurfSurfIntData();
  PrepareData (*aData, theSrf1, theSrf2, theTol);
  ComputeInto (*aData);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kSurfaceSurfaceInt, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeSurfSurfInt::NvGeSurfSurfInt (const NvGeSurfSurfInt& theSource)
: NvGeEntity3d (theSource)
{
}

//=================================================================================================

NvGeTol NvGeSurfSurfInt::tolerance () const
{
  return DataOf (mpImpEnt)->Tol;
}

//=================================================================================================

int NvGeSurfSurfInt::numResults (NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, 0, theErr))
  {
    return 0;
  }
  return static_cast<int> (aData->Results.size());
}

//=================================================================================================

NvGeCurve3d* NvGeSurfSurfInt::intCurve (int theIntNum, Nova::Boolean theIsExternal,
                                        NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return nullptr;
  }
  const NvGeSurfSurfIntResult& anEntry = aData->Results[theIntNum];
  if (anEntry.Dim != 1)
  {
    theErr = NvGe::kXXWrongDimensionAtIndex;
    return nullptr;
  }
  // Documented layout cast: the base wrapper is laid out like every 3d
  // entity shell, the caller owns the returned curve object.
  const NvGe::EntityId anEntityType = theIsExternal ? NvGe::kExternalCurve3d : NvGe::kCurve3d;
  return (NvGeCurve3d*) newEntity3d (new NvGeImpEntity3d (anEntityType, anEntry.Line));
}

//=================================================================================================

NvGeCurve2d* NvGeSurfSurfInt::intParamCurve (int theIntNum, Nova::Boolean theIsExternal,
                                             Nova::Boolean theIsFirst,
                                             NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return nullptr;
  }
  const NvGeSurfSurfIntResult& anEntry = aData->Results[theIntNum];
  if (anEntry.Dim != 1)
  {
    theErr = NvGe::kXXWrongDimensionAtIndex;
    return nullptr;
  }
  const occ::handle<Geom_Surface> aSurface
    = NvGeSurfaceOf ((theIsFirst ? aData->Surface1 : aData->Surface2).get());
  occ::handle<Geom2d_Curve> aPcurve;
  try
  {
    aPcurve = GeomProjLib::Curve2d (anEntry.Line, aSurface);
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
  if (aPcurve.IsNull())
  {
    theErr = NvGe::kXXUnknown;
    return nullptr;
  }
  // Documented layout cast (see intCurve).
  const NvGe::EntityId anEntityType = theIsExternal ? NvGe::kExternalCurve2d : NvGe::kCurve2d;
  return (NvGeCurve2d*) newEntity2d (new NvGeImpEntity3d (anEntityType, aPcurve));
}

//=================================================================================================

NvGePoint3d NvGeSurfSurfInt::intPoint (int theIntNum, NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return NvGePoint3d();
  }
  // GeomAPI_IntSS reports curve results only, so no dimension-0 result
  // ever exists and the point query is invalid for every index.
  theErr = NvGe::kXXWrongDimensionAtIndex;
  return NvGePoint3d();
}

//=================================================================================================

void NvGeSurfSurfInt::getIntPointParams (int theIntNum, NvGePoint2d& theParam1,
                                         NvGePoint2d& theParam2,
                                         NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    theParam1 = NvGePoint2d();
    theParam2 = NvGePoint2d();
    return;
  }
  // Point results never exist (see intPoint).
  theErr = NvGe::kXXWrongDimensionAtIndex;
  theParam1 = NvGePoint2d();
  theParam2 = NvGePoint2d();
}

//=================================================================================================

void NvGeSurfSurfInt::getIntConfigs (int theIntNum, NvGe::ssiConfig& theSurf1Left,
                                     NvGe::ssiConfig& theSurf1Right, NvGe::ssiConfig& theSurf2Left,
                                     NvGe::ssiConfig& theSurf2Right, NvGe::ssiType& theIntType,
                                     int& theDim, NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return;
  }
  const NvGeSurfSurfIntResult& anEntry = aData->Results[theIntNum];
  theSurf1Left = anEntry.S1Left;
  theSurf1Right = anEntry.S1Right;
  theSurf2Left = anEntry.S2Left;
  theSurf2Right = anEntry.S2Right;
  theIntType = anEntry.Type;
  theDim = anEntry.Dim;
}

//=================================================================================================

int NvGeSurfSurfInt::getDimension (int theIntNum, NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return 0;
  }
  return aData->Results[theIntNum].Dim;
}

//=================================================================================================

NvGe::ssiType NvGeSurfSurfInt::getType (int theIntNum, NvGe::NvGeIntersectError& theErr) const
{
  const NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  if (!CheckQuery (aData, theIntNum, theErr))
  {
    return NvGe::kSSITransverse;
  }
  return aData->Results[theIntNum].Type;
}

//=================================================================================================

NvGeSurfSurfInt& NvGeSurfSurfInt::set (const NvGeSurface& theSrf1, const NvGeSurface& theSrf2,
                                       const NvGeTol& theTol)
{
  NvGeSurfSurfIntData* aData = DataOf (mpImpEnt);
  PrepareData (*aData, theSrf1, theSrf2, theTol);
  ComputeInto (*aData);
  return *this;
}

//=================================================================================================

NvGeSurfSurfInt& NvGeSurfSurfInt::operator = (const NvGeSurfSurfInt& theSource)
{
  NvGeEntity3d::operator= (theSource);
  return *this;
}
