// gecbndry.cpp - implementation of NvGeCurveBoundary.
//
// A curve boundary pairs 3d contour curves with their 2d parameter images
// (curves in the surface parameter domain) plus per-curve orientation
// flags. It is a plain (non-entity) class with its own impl pointer,
// mirroring the ARX AcGeCurveBoundary contract.

#include <gecbndry.h>

#include <Nova.h>
#include <NvException.h>
#include <gecurv2d.h>
#include <geent3d.h>
#include <gepos3d.h>

#include <string>
#include <vector>

//=================================================================================================

//! Implementation storage behind NvGeCurveBoundary.
class NvGeImpCurveBoundary
{
public:
  std::vector<NvGeEntity3d*> Crv3d;       // owned when IsOwner
  std::vector<NvGeCurve2d*>  Crv2d;       // owned when IsOwner
  Adesk::Boolean*            Orientation3d = nullptr; // owning arrays
  Adesk::Boolean*            Orientation2d = nullptr;
  bool                       IsOwner = false;

  ~NvGeImpCurveBoundary()
  {
    Clear();
  }

  void Clear()
  {
    if (IsOwner)
    {
      for (NvGeEntity3d* aCurve : Crv3d)
      {
        delete aCurve;
      }
      for (NvGeCurve2d* aCurve : Crv2d)
      {
        delete aCurve;
      }
    }
    Crv3d.clear();
    Crv2d.clear();
    delete[] Orientation3d;
    Orientation3d = nullptr;
    delete[] Orientation2d;
    Orientation2d = nullptr;
    IsOwner = false;
  }

  //! Rebuilds the storage; clones the curve entities when theMakeCopy is
  //! set, otherwise shares the pointers.
  void SetFrom (int theNumElements,
                const NvGeEntity3d* const* theCrv3d,
                const NvGeCurve2d* const* theCrv2d,
                const Adesk::Boolean* theOrientation3d,
                const Adesk::Boolean* theOrientation2d,
                bool theMakeCopy)
  {
    Clear();
    IsOwner = theMakeCopy;
    Orientation3d = new Adesk::Boolean [theNumElements > 0 ? theNumElements : 1];
    Orientation2d = new Adesk::Boolean [theNumElements > 0 ? theNumElements : 1];
    for (int anIdx = 0; anIdx < theNumElements; ++anIdx)
    {
      NvGeEntity3d* aCurve3d = theMakeCopy && theCrv3d != nullptr && theCrv3d[anIdx] != nullptr
                             ? static_cast<NvGeEntity3d*> (theCrv3d[anIdx]->copy())
                             : const_cast<NvGeEntity3d*> (theCrv3d != nullptr ? theCrv3d[anIdx] : nullptr);
      NvGeCurve2d* aCurve2d = theMakeCopy && theCrv2d != nullptr && theCrv2d[anIdx] != nullptr
                             ? static_cast<NvGeCurve2d*> (theCrv2d[anIdx]->copy())
                             : const_cast<NvGeCurve2d*> (theCrv2d != nullptr ? theCrv2d[anIdx] : nullptr);
      Crv3d.push_back (aCurve3d);
      Crv2d.push_back (aCurve2d);
      Orientation3d[anIdx] = theOrientation3d != nullptr ? theOrientation3d[anIdx] : true;
      Orientation2d[anIdx] = theOrientation2d != nullptr ? theOrientation2d[anIdx] : true;
    }
  }

  NvGeImpCurveBoundary* Clone() const
  {
    NvGeImpCurveBoundary* aCopy = new NvGeImpCurveBoundary();
    const int aCount = static_cast<int> (Crv3d.size());
    if (aCount > 0)
    {
      std::vector<const NvGeEntity3d*> a3d (aCount);
      std::vector<const NvGeCurve2d*> a2d (aCount);
      for (int anIdx = 0; anIdx < aCount; ++anIdx)
      {
        a3d[anIdx] = Crv3d[anIdx];
        a2d[anIdx] = Crv2d[anIdx];
      }
      aCopy->SetFrom (aCount, a3d.data(), a2d.data(),
                      Orientation3d, Orientation2d, IsOwner);
    }
    return aCopy;
  }
};

//=================================================================================================

NvGeCurveBoundary::NvGeCurveBoundary()
: mpImpBnd (new NvGeImpCurveBoundary()),
  mDelBnd (1)
{
}

//=================================================================================================

NvGeCurveBoundary::NvGeCurveBoundary (int theNumberOfCurves,
                                      const NvGeEntity3d* const* theCrv3d,
                                      const NvGeCurve2d* const* theCrv2d,
                                      Adesk::Boolean* theOrientation3d,
                                      Adesk::Boolean* theOrientation2d,
                                      Adesk::Boolean theMakeCopy)
: mpImpBnd (new NvGeImpCurveBoundary()),
  mDelBnd (1)
{
  mpImpBnd->SetFrom (theNumberOfCurves, theCrv3d, theCrv2d,
                     theOrientation3d, theOrientation2d, theMakeCopy != false);
}

//=================================================================================================

NvGeCurveBoundary::NvGeCurveBoundary (const NvGeCurveBoundary& theSrc)
: mpImpBnd (theSrc.mpImpBnd->Clone()),
  mDelBnd (1)
{
}

//=================================================================================================

NvGeCurveBoundary::~NvGeCurveBoundary()
{
  if (mDelBnd && mpImpBnd != nullptr)
  {
    delete mpImpBnd;
  }
}

//=================================================================================================

NvGeCurveBoundary& NvGeCurveBoundary::operator = (const NvGeCurveBoundary& theSrc)
{
  if (this == &theSrc)
  {
    return *this;
  }
  if (mDelBnd && mpImpBnd != nullptr)
  {
    delete mpImpBnd;
  }
  mpImpBnd = theSrc.mpImpBnd->Clone();
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeCurveBoundary::isDegenerate() const
{
  return mpImpBnd->Crv3d.size() <= 1;
}

//=================================================================================================

Adesk::Boolean NvGeCurveBoundary::isDegenerate (NvGePosition3d& theDegenPoint,
                                                NvGeCurve2d** theParamCurve) const
{
  if (!isDegenerate() || mpImpBnd->Crv3d.empty())
  {
    return false;
  }
  // The single contour curve degenerates to a point when it is a position
  // entity; the parameter image curve is reported alongside.
  const NvGeEntity3d* aCurve = mpImpBnd->Crv3d[0];
  if (aCurve == nullptr || !aCurve->isKindOf (NvGe::kPosition3d))
  {
    return false;
  }
  const NvGePosition3d* aPos = static_cast<const NvGePosition3d*> (aCurve);
  theDegenPoint = *aPos;
  if (theParamCurve != nullptr)
  {
    *theParamCurve = mpImpBnd->Crv2d.empty() ? nullptr : mpImpBnd->Crv2d[0];
  }
  return true;
}

//=================================================================================================

int NvGeCurveBoundary::numElements() const
{
  return static_cast<int> (mpImpBnd->Crv3d.size());
}

//=================================================================================================

void NvGeCurveBoundary::getContour (int& theN,
                                    NvGeEntity3d*** theCrv3d,
                                    NvGeCurve2d*** theParamGeometry,
                                    Adesk::Boolean** theOrientation3d,
                                    Adesk::Boolean** theOrientation2d) const
{
  theN = numElements();
  *theCrv3d = theN > 0 ? const_cast<NvGeEntity3d**> (mpImpBnd->Crv3d.data()) : nullptr;
  *theParamGeometry = theN > 0 ? const_cast<NvGeCurve2d**> (mpImpBnd->Crv2d.data()) : nullptr;
  *theOrientation3d = theN > 0 ? mpImpBnd->Orientation3d : nullptr;
  *theOrientation2d = theN > 0 ? mpImpBnd->Orientation2d : nullptr;
}

//=================================================================================================

NvGeCurveBoundary& NvGeCurveBoundary::set (int theNumElements,
                                           const NvGeEntity3d* const* theCrv3d,
                                           const NvGeCurve2d* const* theCrv2d,
                                           Adesk::Boolean* theOrientation3d,
                                           Adesk::Boolean* theOrientation2d,
                                           Adesk::Boolean theMakeCopy)
{
  if (theNumElements < 0)
  {
    throw NvException ("NvGeCurveBoundary::set(): negative element count");
  }
  mpImpBnd->SetFrom (theNumElements, theCrv3d, theCrv2d,
                     theOrientation3d, theOrientation2d, theMakeCopy != false);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeCurveBoundary::isOwnerOfCurves() const
{
  return mpImpBnd->IsOwner;
}

//=================================================================================================

NvGeCurveBoundary& NvGeCurveBoundary::setToOwnCurves()
{
  // Take ownership: clone any borrowed curves into privately owned copies.
  if (!mpImpBnd->IsOwner)
  {
    const int aCount = numElements();
    std::vector<const NvGeEntity3d*> a3d (aCount);
    std::vector<const NvGeCurve2d*> a2d (aCount);
    for (int anIdx = 0; anIdx < aCount; ++anIdx)
    {
      a3d[anIdx] = mpImpBnd->Crv3d[anIdx];
      a2d[anIdx] = mpImpBnd->Crv2d[anIdx];
    }
    mpImpBnd->SetFrom (aCount, a3d.data(), a2d.data(),
                       mpImpBnd->Orientation3d, mpImpBnd->Orientation2d, true);
  }
  return *this;
}
