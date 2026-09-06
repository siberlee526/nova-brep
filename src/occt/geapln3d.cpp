// geapln3d.cpp - implementation of NvGeAugPolyline3d.
//
// An augmented polyline is a polyline (see geplin3d.cpp: the fit points are
// the poles of a degree 1 Geom_BSplineCurve) plus a per-point D1 tangent
// bundle, an optional per-point D2 bundle and an approximation tolerance.
// The augmentation has no base-class storage, so it rides on a file-local
// Geom_BSplineCurve subclass (NvGeApln3dData); every base-class query keeps
// seeing a plain B-spline through its down-cast. Modifiers follow the
// copy-on-write discipline: they build a NEW curve object (never mutating
// the possibly shared one) and replace the impl geometry after detaching a
// shared impl.
//
// Indices are 1-based, matching NvGePolyline3d::fitPointAt.

#include <geapln3d.h>

#include <NvException.h>
#include <gegbl.h>
#include <geimpdata.h>

#include <Geom_BSplineCurve.hxx>
#include <Geom_Curve.hxx>
#include <NCollection_Array1.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

// Returns the stored B-spline curve.
occ::handle<Geom_BSplineCurve> SplineOf (const occ::handle<Standard_Transient>& theGeom)
{
  if (theGeom.IsNull())
  {
    throw NvException ("NvGeAugPolyline3d: null curve data");
  }
  const occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (theGeom);
  if (aSpline.IsNull())
  {
    throw NvException ("NvGeAugPolyline3d: the entity does not store a B-spline curve");
  }
  return aSpline;
}

// Side data riding on NvGeApln3dData: the tangent bundles and the
// approximation tolerance. Copied along with every private copy.
struct NvGeAplnData3d
{
  NvGeVector3dArray D1;         // tangent bundle at the fit points
  NvGeVector3dArray D2;         // optional second-derivative bundle
  double            ApproxTol = 0.0;
};

// B-spline storage: the OCCT definition plus the side data above.
class NvGeApln3dData : public Geom_BSplineCurve
{
public:

  NvGeApln3dData (const NCollection_Array1<gp_Pnt>& thePoles,
                  const NCollection_Array1<double>& theKnots,
                  const NCollection_Array1<int>& theMults,
                  const int theDegree,
                  const bool thePeriodic,
                  const NvGeAplnData3d& theData)
  : Geom_BSplineCurve (thePoles, theKnots, theMults, theDegree, thePeriodic),
    Data (theData)
  {
  }

  //! Definition copy carrying side data (used when attaching augmentation).
  NvGeApln3dData (const Geom_BSplineCurve& theSrc, const NvGeAplnData3d& theData)
  : Geom_BSplineCurve (theSrc),
    Data (theData)
  {
  }

  //! Full copy: definition plus side data (copy-on-write private copies).
  NvGeApln3dData (const NvGeApln3dData& theSrc)
  : Geom_BSplineCurve (theSrc),
    Data (theSrc.Data)
  {
  }

  NvGeAplnData3d Data;
};

// The side data behind theSpline; null when the curve carries none.
const NvGeApln3dData* FitDataOf (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  return dynamic_cast<const NvGeApln3dData*> (theSpline.get());
}

// Validates the 1-based index against the item count.
void CheckIndex (int theIdx, int theCount, const char* theMessage)
{
  if (theIdx < 1 || theIdx > theCount)
  {
    throw NvException (theMessage);
  }
}

// Fills the degree 1 curve arrays over the fit points: the knots 0 .. n - 1
// with the multiplicities [2, 1, ..., 1, 2]; requires at least two points.
void PolylineArraysOf (const std::vector<gp_Pnt>& thePoles,
                       NCollection_Array1<gp_Pnt>& thePoleArr,
                       NCollection_Array1<double>& theKnotArr,
                       NCollection_Array1<int>& theMultArr)
{
  const int aNbPoles = static_cast<int> (thePoles.size());
  if (aNbPoles < 2)
  {
    throw NvException ("NvGeAugPolyline3d: at least two points are required");
  }
  thePoleArr = NCollection_Array1<gp_Pnt> (1, aNbPoles);
  theKnotArr = NCollection_Array1<double> (1, aNbPoles);
  theMultArr = NCollection_Array1<int> (1, aNbPoles);
  for (int aPole = 1; aPole <= aNbPoles; ++aPole)
  {
    thePoleArr.SetValue (aPole, thePoles[aPole - 1]);
    theKnotArr.SetValue (aPole, static_cast<double> (aPole - 1));
    theMultArr.SetValue (aPole, (aPole == 1 || aPole == aNbPoles) ? 2 : 1);
  }
}

// Distance from thePnt to the segment [theSegA, theSegB].
double DistToSegment (const NvGePoint3d& thePnt, const NvGePoint3d& theSegA, const NvGePoint3d& theSegB)
{
  const double aDx = theSegB.x - theSegA.x;
  const double aDy = theSegB.y - theSegA.y;
  const double aDz = theSegB.z - theSegA.z;
  const double aLen2 = aDx * aDx + aDy * aDy + aDz * aDz;
  if (aLen2 <= 0.0)
  {
    return thePnt.distanceTo (theSegA);
  }
  double aT = ((thePnt.x - theSegA.x) * aDx + (thePnt.y - theSegA.y) * aDy + (thePnt.z - theSegA.z) * aDz) / aLen2;
  aT = std::max (0.0, std::min (1.0, aT));
  return thePnt.distanceTo (NvGePoint3d (theSegA.x + aT * aDx, theSegA.y + aT * aDy, theSegA.z + aT * aDz));
}

// Sample parameters over [theStart, theEnd] refining at midpoints whose
// chord deviation exceeds theEps (bounded to keep sampling predictable).
std::vector<double> AdaptiveParams (const NvGeCurve3d& theCurve,
                                    double theStart, double theEnd, double theEps)
{
  std::vector<double> aParams;
  constexpr int THE_SEEDS = 17;
  for (int i = 0; i < THE_SEEDS; ++i)
  {
    aParams.push_back (theStart + (theEnd - theStart) * i / static_cast<double> (THE_SEEDS - 1));
  }
  for (int aRound = 0; aRound < 7; ++aRound)
  {
    std::vector<double> aRefined;
    aRefined.reserve (aParams.size() * 2);
    bool aSplit = false;
    for (size_t i = 0; i + 1 < aParams.size(); ++i)
    {
      aRefined.push_back (aParams[i]);
      if (aRefined.size() + (aParams.size() - i) > 1024u)
      {
        continue;
      }
      const double aMid = 0.5 * (aParams[i] + aParams[i + 1]);
      const NvGePoint3d aP0 = theCurve.evalPoint (aParams[i]);
      const NvGePoint3d aPm = theCurve.evalPoint (aMid);
      const NvGePoint3d aP1 = theCurve.evalPoint (aParams[i + 1]);
      if (DistToSegment (aPm, aP0, aP1) > theEps)
      {
        aRefined.push_back (aMid);
        aSplit = true;
      }
    }
    aRefined.push_back (aParams.back());
    aParams.swap (aRefined);
    if (!aSplit)
    {
      break;
    }
  }
  return aParams;
}

// Detached full copy of theSpline (definition plus side data) for editing.
occ::handle<NvGeApln3dData> PrivateCopy (const occ::handle<Geom_BSplineCurve>& theSpline)
{
  const NvGeApln3dData* aSrc = FitDataOf (theSpline);
  if (aSrc != nullptr)
  {
    return occ::handle<NvGeApln3dData> (new NvGeApln3dData (*aSrc));
  }
  return occ::handle<NvGeApln3dData> (new NvGeApln3dData (*theSpline, NvGeAplnData3d()));
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom_BSplineCurve>& theSpline)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (occ::handle<Geom_Curve> (theSpline));
}

} // namespace

//=================================================================================================

NvGeAugPolyline3d::NvGeAugPolyline3d ()
{
  std::vector<gp_Pnt> aPoles (2, gp_Pnt (0.0, 0.0, 0.0));
  NCollection_Array1<gp_Pnt> aPoleArr;
  NCollection_Array1<double> aKnotArr;
  NCollection_Array1<int> aMultArr;
  PolylineArraysOf (aPoles, aPoleArr, aKnotArr, aMultArr);
  NvGeAplnData3d aData;
  aData.D1.append (NvGeVector3d ());
  aData.D1.append (NvGeVector3d ());
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kAugPolyline3d,
                                  occ::handle<Geom_Curve> (occ::handle<NvGeApln3dData> (new NvGeApln3dData (aPoleArr,
                                                                                                            aKnotArr,
                                                                                                            aMultArr,
                                                                                                            1,
                                                                                                            false,
                                                                                                            aData))));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeAugPolyline3d::NvGeAugPolyline3d (const NvGeAugPolyline3d& theSrc)
: NvGePolyline3d (theSrc)
{
}

//=================================================================================================

NvGeAugPolyline3d::NvGeAugPolyline3d (const NvGeKnotVector& theKnots,
                                      const NvGePoint3dArray& theCntrlPnts,
                                      const NvGeVector3dArray& theVecBundle)
: NvGePolyline3d (theKnots, theCntrlPnts)
{
  if (theVecBundle.length() != theCntrlPnts.length())
  {
    throw NvException ("NvGeAugPolyline3d::NvGeAugPolyline3d(): the vector count does not"
                       " match the control point count");
  }
  NvGeAplnData3d aData;
  aData.D1 = theVecBundle;
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  ReplaceGeometry (mpImpEnt, occ::handle<NvGeApln3dData> (new NvGeApln3dData (*aSpline, aData)));
}

//=================================================================================================

NvGeAugPolyline3d::NvGeAugPolyline3d (const NvGePoint3dArray& theCntrlPnts,
                                      const NvGeVector3dArray& theVecBundle)
: NvGePolyline3d (theCntrlPnts)
{
  if (theVecBundle.length() != theCntrlPnts.length())
  {
    throw NvException ("NvGeAugPolyline3d::NvGeAugPolyline3d(): the vector count does not"
                       " match the control point count");
  }
  NvGeAplnData3d aData;
  aData.D1 = theVecBundle;
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  ReplaceGeometry (mpImpEnt, occ::handle<NvGeApln3dData> (new NvGeApln3dData (*aSpline, aData)));
}

//=================================================================================================

NvGeAugPolyline3d::NvGeAugPolyline3d (const NvGeCurve3d& theCurve,
                                      double theFromParam, double theToParam,
                                      double theApprEps)
{
  if (!(theApprEps > 0.0))
  {
    throw NvException ("NvGeAugPolyline3d::NvGeAugPolyline3d(): the approximation tolerance"
                       " must be positive");
  }
  if (!(theToParam > theFromParam))
  {
    throw NvException ("NvGeAugPolyline3d::NvGeAugPolyline3d(): the end parameter must be"
                       " greater than the start parameter");
  }
  const std::vector<double> aParams = AdaptiveParams (theCurve, theFromParam, theToParam,
                                                      theApprEps);
  std::vector<gp_Pnt> aPoles;
  aPoles.reserve (aParams.size());
  NvGeAplnData3d aData;
  for (size_t i = 0; i < aParams.size(); ++i)
  {
    const NvGePoint3d aPnt = theCurve.evalPoint (aParams[i]);
    aPoles.push_back (gp_Pnt (aPnt.x, aPnt.y, aPnt.z));
    NvGeVector3dArray aDerivs;
    theCurve.evalPoint (aParams[i], 1, aDerivs);
    aData.D1.append (aDerivs.length() > 0 ? aDerivs[0] : NvGeVector3d ());
  }
  NCollection_Array1<gp_Pnt> aPoleArr;
  NCollection_Array1<double> aKnotArr;
  NCollection_Array1<int> aMultArr;
  PolylineArraysOf (aPoles, aPoleArr, aKnotArr, aMultArr);
  aData.ApproxTol = theApprEps;
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kAugPolyline3d,
                                  occ::handle<Geom_Curve> (occ::handle<NvGeApln3dData> (new NvGeApln3dData (aPoleArr,
                                                                                                            aKnotArr,
                                                                                                            aMultArr,
                                                                                                            1,
                                                                                                            false,
                                                                                                            aData))));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeAugPolyline3d& NvGeAugPolyline3d::operator = (const NvGeAugPolyline3d& theAplne)
{
  NvGePolyline3d::operator = (theAplne);
  return *this;
}

//=================================================================================================

NvGePoint3d NvGeAugPolyline3d::getPoint (int theIdx) const
{
  return fitPointAt (theIdx);
}

//=================================================================================================

NvGeAugPolyline3d& NvGeAugPolyline3d::setPoint (int theIdx, NvGePoint3d thePnt)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  CheckIndex (theIdx, aSpline->NbPoles(),
              "NvGeAugPolyline3d::setPoint(): the point index is out of range");
  // A full private copy keeps the augmentation side data alive.
  occ::handle<NvGeApln3dData> aData = PrivateCopy (aSpline);
  aData->SetPole (theIdx, gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

void NvGeAugPolyline3d::getPoints (NvGePoint3dArray& thePnts) const
{
  thePnts.removeAll();
  const int aCount = numFitPoints();
  for (int aPnt = 1; aPnt <= aCount; ++aPnt)
  {
    thePnts.append (fitPointAt (aPnt));
  }
}

//=================================================================================================

NvGeVector3d NvGeAugPolyline3d::getVector (int theIdx) const
{
  const NvGeApln3dData* aData = FitDataOf (SplineOf (mpImpEnt->Geom()));
  CheckIndex (theIdx, aData != nullptr ? aData->Data.D1.length() : 0,
              "NvGeAugPolyline3d::getVector(): the vector index is out of range");
  return aData->Data.D1[theIdx - 1];
}

//=================================================================================================

NvGeAugPolyline3d& NvGeAugPolyline3d::setVector (int theIdx, NvGeVector3d theVec)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  const NvGeApln3dData* aSrc = FitDataOf (aSpline);
  CheckIndex (theIdx, aSrc != nullptr ? aSrc->Data.D1.length() : 0,
              "NvGeAugPolyline3d::setVector(): the vector index is out of range");
  occ::handle<NvGeApln3dData> aData = PrivateCopy (aSpline);
  aData->Data.D1[theIdx - 1] = theVec;
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

void NvGeAugPolyline3d::getD1Vectors (NvGeVector3dArray& theTangents) const
{
  theTangents.removeAll();
  const NvGeApln3dData* aData = FitDataOf (SplineOf (mpImpEnt->Geom()));
  if (aData == nullptr)
  {
    return;
  }
  for (int i = 0; i < aData->Data.D1.length(); ++i)
  {
    theTangents.append (aData->Data.D1[i]);
  }
}

//=================================================================================================

NvGeVector3d NvGeAugPolyline3d::getD2Vector (int theIdx) const
{
  const NvGeApln3dData* aData = FitDataOf (SplineOf (mpImpEnt->Geom()));
  CheckIndex (theIdx, aData != nullptr ? aData->Data.D2.length() : 0,
              "NvGeAugPolyline3d::getD2Vector(): the vector index is out of range");
  return aData->Data.D2[theIdx - 1];
}

//=================================================================================================

NvGeAugPolyline3d& NvGeAugPolyline3d::setD2Vector (int theIdx, NvGeVector3d theVec)
{
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  const NvGeApln3dData* aSrc = FitDataOf (aSpline);
  CheckIndex (theIdx, aSrc != nullptr ? aSrc->Data.D2.length() : 0,
              "NvGeAugPolyline3d::setD2Vector(): the vector index is out of range");
  occ::handle<NvGeApln3dData> aData = PrivateCopy (aSpline);
  aData->Data.D2[theIdx - 1] = theVec;
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}

//=================================================================================================

void NvGeAugPolyline3d::getD2Vectors (NvGeVector3dArray& theD2Vectors) const
{
  theD2Vectors.removeAll();
  const NvGeApln3dData* aData = FitDataOf (SplineOf (mpImpEnt->Geom()));
  if (aData == nullptr)
  {
    return;
  }
  for (int i = 0; i < aData->Data.D2.length(); ++i)
  {
    theD2Vectors.append (aData->Data.D2[i]);
  }
}

//=================================================================================================

double NvGeAugPolyline3d::approxTol () const
{
  const NvGeApln3dData* aData = FitDataOf (SplineOf (mpImpEnt->Geom()));
  return aData != nullptr ? aData->Data.ApproxTol : 0.0;
}

//=================================================================================================

NvGeAugPolyline3d& NvGeAugPolyline3d::setApproxTol (double theApproxTol)
{
  if (!(theApproxTol > 0.0))
  {
    throw NvException ("NvGeAugPolyline3d::setApproxTol(): the approximation tolerance must"
                       " be positive");
  }
  const occ::handle<Geom_BSplineCurve> aSpline = SplineOf (mpImpEnt->Geom());
  occ::handle<NvGeApln3dData> aData = PrivateCopy (aSpline);
  aData->Data.ApproxTol = theApproxTol;
  ReplaceGeometry (mpImpEnt, aData);
  return *this;
}
