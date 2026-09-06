// gecomp3d.cpp - implementation of NvGeCompositeCurve3d.
//
// A composite curve concatenates a list of curve entities. The segment
// entity impls are stored in a file-local holder; the global parameter is
// the accumulated arc of per-segment parameter intervals (each segment
// spans its own [first, last] domain, mapped to a consecutive global
// window).
//
// KNOWN LIMITATION: the generic NvGeCurve3d operations (evalPoint and
// friends) operate on a single Geom_Curve and therefore throw for
// composite holders; use globalToLocalParam + the component curves to
// evaluate composite geometry.

#include <gecomp3d.h>

#include <NvException.h>
#include <gecurv3d.h>
#include <geimpdata.h>

#include <Geom_Curve.hxx>

#include <string>
#include <vector>

namespace
{

//! Storage: the shared impls of the component curves.
class NvGeComposite3dData : public NvGeEntityData
{
public:
  std::vector<occ::handle<NvGeImpEntity3d>> Segments;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeComposite3dData> aCopy = new NvGeComposite3dData();
    aCopy->Segments = Segments; // shared, immutable-by-convention
    return aCopy;
  }
};

//! Holder of this entity with the COW discipline applied.
NvGeComposite3dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned =
      NvGeDataOf<NvGeComposite3dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeComposite3dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeComposite3dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeComposite3dData> (theImp->Geom());
}

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

//! Collects the segment impls from a curve pointer list.
std::vector<occ::handle<NvGeImpEntity3d>>
SegmentsFromList (const NvGeVoidPointerArray& theCurveList, const char* theMethod)
{
  std::vector<occ::handle<NvGeImpEntity3d>> aSegments;
  aSegments.reserve (theCurveList.length());
  for (int anIdx = 0; anIdx < theCurveList.length(); ++anIdx)
  {
    const NvGeCurve3d* aCurve = static_cast<const NvGeCurve3d*> (theCurveList[anIdx]);
    if (aCurve == nullptr)
    {
      throw NvException (std::string ("NvGeCompositeCurve3d::") + theMethod
                         + "(): the curve list contains a null curve");
    }
    // Validate the entity really holds curve geometry, then share its impl.
    NvGeCurve3dOf (ImplOf (aCurve));
    aSegments.push_back (ImplOf (aCurve)->CloneShallow());
  }
  return aSegments;
}

//! Parameter span of a segment curve.
void SpanOf (const NvGeImpEntity3d* theSegment, double& theFirst, double& theLast)
{
  const occ::handle<Geom_Curve> aCurve = NvGeCurve3dOf (theSegment);
  theFirst = aCurve->FirstParameter();
  theLast = aCurve->LastParameter();
}

} // namespace

//=================================================================================================

NvGeCompositeCurve3d::NvGeCompositeCurve3d()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCompositeCrv3d,
                                  occ::handle<NvGeComposite3dData> (new NvGeComposite3dData()));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCompositeCurve3d::NvGeCompositeCurve3d (const NvGeVoidPointerArray& theCurveList)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeComposite3dData> aData = new NvGeComposite3dData();
  aData->Segments = SegmentsFromList (theCurveList, "NvGeCompositeCurve3d");
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCompositeCrv3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCompositeCurve3d::NvGeCompositeCurve3d (const NvGeVoidPointerArray& theCurveList,
                                            const NvGeIntArray& isOwnerOfCurves)
{
  // Ownership is irrelevant here: segments are stored as shared, immutable
  // impl clones, so the flags are accepted and ignored.
  (void)isOwnerOfCurves;
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  occ::handle<NvGeComposite3dData> aData = new NvGeComposite3dData();
  aData->Segments = SegmentsFromList (theCurveList, "NvGeCompositeCurve3d");
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCompositeCrv3d, aData);
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCompositeCurve3d::NvGeCompositeCurve3d (const NvGeCompositeCurve3d& theCompCurve)
: NvGeCurve3d (theCompCurve)
{
}

//=================================================================================================

void NvGeCompositeCurve3d::getCurveList (NvGeVoidPointerArray& theCurveList) const
{
  const NvGeComposite3dData* aData = DataOf (mpImpEnt);
  theCurveList.setLogicalLength (0);
  for (const occ::handle<NvGeImpEntity3d>& aSegment : aData->Segments)
  {
    // Borrowed wrappers around shared segment geometry; released when the
    // list is rebuilt (single-slot free-list keeps this leak-free for the
    // common rebuild-and-use pattern).
    theCurveList.append ((NvGeCurve3d*) newEntity3d (aSegment->CloneShallow()));
  }
}

//=================================================================================================

NvGeCompositeCurve3d& NvGeCompositeCurve3d::setCurveList (const NvGeVoidPointerArray& theCurveList)
{
  NvGeComposite3dData* aData = DataOf (mpImpEnt);
  aData->Segments = SegmentsFromList (theCurveList, "setCurveList");
  return *this;
}

//=================================================================================================

NvGeCompositeCurve3d& NvGeCompositeCurve3d::setCurveList (const NvGeVoidPointerArray& theCurveList,
                                                          const NvGeIntArray& isOwnerOfCurves)
{
  (void)isOwnerOfCurves; // ownership is absorbed by the shared-impl storage
  NvGeComposite3dData* aData = DataOf (mpImpEnt);
  aData->Segments = SegmentsFromList (theCurveList, "setCurveList");
  return *this;
}

//=================================================================================================

double NvGeCompositeCurve3d::globalToLocalParam (double theParam, int& theCrvNum) const
{
  const NvGeComposite3dData* aData = DataOf (mpImpEnt);
  double anOffset = 0.0;
  for (size_t aSeg = 0; aSeg < aData->Segments.size(); ++aSeg)
  {
    double aFirst = 0.0;
    double aLast = 0.0;
    SpanOf (aData->Segments[aSeg].get(), aFirst, aLast);
    const double aSpan = aLast - aFirst;
    if (theParam <= anOffset + aSpan || aSeg + 1 == aData->Segments.size())
    {
      theCrvNum = static_cast<int> (aSeg);
      return aFirst + (theParam - anOffset);
    }
    anOffset += aSpan;
  }
  theCrvNum = -1;
  return theParam;
}

//=================================================================================================

double NvGeCompositeCurve3d::localToGlobalParam (double theParam, int theCrvNum) const
{
  const NvGeComposite3dData* aData = DataOf (mpImpEnt);
  if (theCrvNum < 0 || theCrvNum >= static_cast<int> (aData->Segments.size()))
  {
    throw NvException ("NvGeCompositeCurve3d::localToGlobalParam(): segment index out of range");
  }
  double anOffset = 0.0;
  for (int aSeg = 0; aSeg < theCrvNum; ++aSeg)
  {
    double aFirst = 0.0;
    double aLast = 0.0;
    SpanOf (aData->Segments[aSeg].get(), aFirst, aLast);
    anOffset += aLast - aFirst;
  }
  double aFirst = 0.0;
  double aLast = 0.0;
  SpanOf (aData->Segments[theCrvNum].get(), aFirst, aLast);
  return anOffset + (theParam - aFirst);
}

//=================================================================================================

NvGeCompositeCurve3d& NvGeCompositeCurve3d::operator = (const NvGeCompositeCurve3d& theCompCurve)
{
  NvGeCurve3d::operator= (theCompCurve);
  return *this;
}
