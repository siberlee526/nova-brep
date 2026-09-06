// geent2d.cpp - implementation of NvGeEntity2d, the 2d pimpl entity root.
//
// The public hierarchy is non-virtual: every derived entity is an empty
// shell around an NvGeImpEntity3d (see geimpdata.h). This file implements
// the shared wrapper mechanics - reference counting, RTTI through entity
// ids, copy - plus the centralized dispatch of transformBy / isEqualTo /
// isOn over the impl geometry kinds.

#include <geent2d.h>

#include <Nova.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <geline2d.h>
#include <gemat2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <geimpdata.h>

#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Ellipse.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>

#include <cmath>

namespace
{

//! Immediate parent in the entity id hierarchy; returns theType itself for
//! roots (kEntity2d / kEntity3d).
NvGe::EntityId ParentEntityOf (NvGe::EntityId theType)
{
  switch (theType)
  {
    // ---- 2d chain ----
    case NvGe::kPointEnt2d:
    case NvGe::kCurve2d:
    case NvGe::kBoundBlock2d:
    case NvGe::kClipBoundary2d:
    case NvGe::kCurveCurveInt2d:
    case NvGe::kEnvelope2d:
      return NvGe::kEntity2d;
    case NvGe::kPosition2d:
    case NvGe::kPointOnCurve2d:
      return NvGe::kPointEnt2d;
    case NvGe::kLinearEnt2d:
    case NvGe::kConic2d:
    case NvGe::kSplineEnt2d:
    case NvGe::kCompositeCrv2d:
    case NvGe::kOffsetCurve2d:
    case NvGe::kTrimmedCrv2d:
    case NvGe::kTrimmedCurve2d:
    case NvGe::kExternalCurve2d:
      return NvGe::kCurve2d;
    case NvGe::kLine2d:
    case NvGe::kRay2d:
    case NvGe::kLineSeg2d:
      return NvGe::kLinearEnt2d;
    case NvGe::kCircArc2d:
    case NvGe::kEllipArc2d:
      return NvGe::kConic2d;
    case NvGe::kPolyline2d:
    case NvGe::kAugPolyline2d:
    case NvGe::kNurbCurve2d:
    case NvGe::kCubicSplineCurve2d:
    case NvGe::kDSpline2d:
      return NvGe::kSplineEnt2d;

    // ---- 3d chain ----
    case NvGe::kPointEnt3d:
    case NvGe::kCurve3d:
    case NvGe::kSurface:
    case NvGe::kBoundBlock3d:
    case NvGe::kCurveCurveInt3d:
    case NvGe::kCurveSurfaceInt:
    case NvGe::kSurfaceSurfaceInt:
    case NvGe::kFitData3d:
      return NvGe::kEntity3d;
    case NvGe::kPosition3d:
    case NvGe::kPointOnCurve3d:
    case NvGe::kPointOnSurface:
      return NvGe::kPointEnt3d;
    case NvGe::kLinearEnt3d:
    case NvGe::kConic3d:
    case NvGe::kSplineEnt3d:
    case NvGe::kCompositeCrv3d:
    case NvGe::kOffsetCurve3d:
    case NvGe::kTrimmedCurve3d:
    case NvGe::kExternalCurve3d:
    case NvGe::kPolynomCurve3d:
    case NvGe::kBezierCurve3d:
    case NvGe::kHelix:
      return NvGe::kCurve3d;
    case NvGe::kLine3d:
    case NvGe::kRay3d:
    case NvGe::kLineSeg3d:
      return NvGe::kLinearEnt3d;
    case NvGe::kCircArc3d:
    case NvGe::kEllipArc3d:
      return NvGe::kConic3d;
    case NvGe::kPolyline3d:
    case NvGe::kAugPolyline3d:
    case NvGe::kNurbCurve3d:
    case NvGe::kCubicSplineCurve3d:
    case NvGe::kDSpline3d:
      return NvGe::kSplineEnt3d;
    case NvGe::kPlanarEnt:
    case NvGe::kSphere:
    case NvGe::kCylinder:
    case NvGe::kTorus:
    case NvGe::kCone:
    case NvGe::kEllipCone:
    case NvGe::kEllipCylinder:
    case NvGe::kExternalSurface:
    case NvGe::kExternalBoundedSurface:
    case NvGe::kNurbSurface:
    case NvGe::kOffsetSurface:
    case NvGe::kCurveBoundedSurface:
      return NvGe::kSurface;
    case NvGe::kPlane:
    case NvGe::kBoundedPlane:
      return NvGe::kPlanarEnt;
    default:
      return theType;
  }
}

//! Builds a gp_Trsf carrying the 2d part of a conformal matrix, for the
//! Geom2d_Geometry::Transform() entry point which only accepts gp_Trsf.
gp_Trsf Trsf3dFromConformal2d (const NvGeMatrix2d& theMat)
{
  const gp_Trsf2d aTrsf2d = Trsf2dFromMatrix (theMat);
  gp_Trsf aTrsf;
  aTrsf.SetValues (aTrsf2d.Value (1, 1), aTrsf2d.Value (1, 2), 0.0, aTrsf2d.Value (1, 3),
                   aTrsf2d.Value (2, 1), aTrsf2d.Value (2, 2), 0.0, aTrsf2d.Value (2, 3),
                   0.0,                  0.0,                  1.0, 0.0);
  return aTrsf;
}

//! Transforms a 2d OCCT curve in place on a private copy. B-splines absorb
//! any affine matrix exactly (poles carry the geometry); analytic curves
//! require a conformal matrix.
occ::handle<Geom2d_Curve> TransformedCurve2d (const occ::handle<Geom2d_Curve>& theCurve,
                                              const NvGeMatrix2d& theMat)
{
  occ::handle<Geom2d_BSplineCurve> aSpline = occ::down_cast<Geom2d_BSplineCurve> (theCurve);
  if (!aSpline.IsNull())
  {
    // Poles are control data: a general affine map stays exact.
    occ::handle<Geom2d_BSplineCurve> aCopy = occ::down_cast<Geom2d_BSplineCurve> (aSpline->Copy());
    const int aNbPoles = aCopy->NbPoles();
    for (int aPole = 1; aPole <= aNbPoles; ++aPole)
    {
      aCopy->SetPole (aPole, ApplyMatrix2d (theMat, aCopy->Pole (aPole)));
    }
    return aCopy;
  }
  occ::handle<Geom2d_Curve> aCopy = occ::down_cast<Geom2d_Curve> (theCurve->Copy());
  aCopy->Transform (Trsf3dFromConformal2d (theMat));
  return aCopy;
}

//! Samples two 2d curves on a common parameter window and compares points.
bool Curves2dAreEqual (const occ::handle<Geom2d_Curve>& theC1,
                       const occ::handle<Geom2d_Curve>& theC2,
                       const NvGeTol& theTol)
{
  if (theC1->FirstParameter() != theC2->FirstParameter()
   || theC1->LastParameter()  != theC2->LastParameter())
  {
    return false;
  }
  double aFirst = theC1->FirstParameter();
  double aLast  = theC1->LastParameter();
  // Sampling window guard: huge-but-finite domains (OCCT stores line domains
  // as +/-Precision::Infinite()) amplify last-bit direction differences into
  // enormous point distances, so clamp the window to a local neighborhood.
  if (!std::isfinite (aFirst) || !std::isfinite (aLast) || aLast - aFirst > 1e3)
  {
    const double aMid = 0.5 * (std::isfinite (aFirst + aLast) ? aFirst + aLast : 0.0);
    aFirst = aMid - 1.0;
    aLast = aMid + 1.0;
  }
  if (!(aLast > aFirst))
  {
    return true; // both degenerate domains
  }
  const int THE_NB_SAMPLES = 17;
  for (int aSample = 0; aSample < THE_NB_SAMPLES; ++aSample)
  {
    const double aParam = aFirst + (aLast - aFirst) * aSample / (THE_NB_SAMPLES - 1);
    const gp_Pnt2d aP1 = theC1->Value (aParam);
    const gp_Pnt2d aP2 = theC2->Value (aParam);
    if (aP1.Distance (aP2) > theTol.equalPoint())
    {
      return false;
    }
  }
  return true;
}

//! Detaches the impl before mutation (copy-on-write).
void MakeUnique (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
}

} // namespace

//=================================================================================================

NvGeEntity2d::~NvGeEntity2d()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
}

//=================================================================================================

Adesk::Boolean NvGeEntity2d::isKindOf (NvGe::EntityId theEntType) const
{
  return NvGeIsKindOf (type(), theEntType);
}

//=================================================================================================

NvGe::EntityId NvGeEntity2d::type() const
{
  return mpImpEnt != nullptr ? mpImpEnt->Type() : NvGe::kEntity2d;
}

//=================================================================================================

NvGeEntity2d* NvGeEntity2d::copy() const
{
  // The returned base wrapper shares the immutable geometry; callers cast
  // it down to the concrete entity kind (ARX pattern).
  return newEntity2d (mpImpEnt != nullptr ? mpImpEnt->CloneShallow() : nullptr);
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::operator = (const NvGeEntity2d& theEntity)
{
  if (this == &theEntity)
  {
    return *this;
  }
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = theEntity.mpImpEnt;
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeEntity2d::operator == (const NvGeEntity2d& theEntity) const
{
  return isEqualTo (theEntity);
}

//=================================================================================================

Adesk::Boolean NvGeEntity2d::operator != (const NvGeEntity2d& theEntity) const
{
  return !isEqualTo (theEntity);
}

//=================================================================================================

Adesk::Boolean NvGeEntity2d::isEqualTo (const NvGeEntity2d& theEntity, const NvGeTol& theTol) const
{
  if (mpImpEnt == nullptr || theEntity.mpImpEnt == nullptr)
  {
    return mpImpEnt == theEntity.mpImpEnt;
  }
  if (mpImpEnt->Type() != theEntity.mpImpEnt->Type())
  {
    return false;
  }
  if (mpImpEnt->Geom() == theEntity.mpImpEnt->Geom())
  {
    return true; // same shared geometry
  }

  const occ::handle<Standard_Transient>& aGeom1 = mpImpEnt->Geom();
  const occ::handle<Standard_Transient>& aGeom2 = theEntity.mpImpEnt->Geom();

  occ::handle<Geom2d_Curve> aCurve1 = occ::down_cast<Geom2d_Curve> (aGeom1);
  occ::handle<Geom2d_Curve> aCurve2 = occ::down_cast<Geom2d_Curve> (aGeom2);
  if (!aCurve1.IsNull() && !aCurve2.IsNull())
  {
    return Curves2dAreEqual (aCurve1, aCurve2, theTol);
  }

  if (const NvGePosition2dData* aPos1 = NvGeDataOf<NvGePosition2dData> (aGeom1))
  {
    const NvGePosition2dData* aPos2 = NvGeDataOf<NvGePosition2dData> (aGeom2);
    return aPos2 != nullptr
        && aPos1->Point.Distance (aPos2->Point) <= theTol.equalPoint();
  }

  if (const NvGeBoundBlock2dData* aBlock1 = NvGeDataOf<NvGeBoundBlock2dData> (aGeom1))
  {
    const NvGeBoundBlock2dData* aBlock2 = NvGeDataOf<NvGeBoundBlock2dData> (aGeom2);
    return aBlock2 != nullptr
        && aBlock1->Point.Distance (aBlock2->Point) <= theTol.equalPoint()
        && std::abs (aBlock1->Vec1.Magnitude() - aBlock2->Vec1.Magnitude()) <= theTol.equalVector()
        && std::abs (aBlock1->Vec2.Magnitude() - aBlock2->Vec2.Magnitude()) <= theTol.equalVector();
  }

  if (const NvGeClipBoundary2dData* aClip1 = NvGeDataOf<NvGeClipBoundary2dData> (aGeom1))
  {
    const NvGeClipBoundary2dData* aClip2 = NvGeDataOf<NvGeClipBoundary2dData> (aGeom2);
    if (aClip2 == nullptr || aClip1->Points.size() != aClip2->Points.size())
    {
      return false;
    }
    for (size_t aPnt = 0; aPnt < aClip1->Points.size(); ++aPnt)
    {
      if (aClip1->Points[aPnt].Distance (aClip2->Points[aPnt]) > theTol.equalPoint())
      {
        return false;
      }
    }
    return true;
  }

  return false;
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::transformBy (const NvGeMatrix2d& theXfm)
{
  if (mpImpEnt == nullptr)
  {
    return *this;
  }
  MakeUnique (mpImpEnt);
  const occ::handle<Standard_Transient>& aGeom = mpImpEnt->Geom();
  if (aGeom.IsNull())
  {
    return *this;
  }

  if (occ::handle<Geom2d_Curve> aCurve = occ::down_cast<Geom2d_Curve> (aGeom))
  {
    mpImpEnt->SetGeom (TransformedCurve2d (aCurve, theXfm));
    return *this;
  }

  if (NvGePosition2dData* aPos = NvGeDataOf<NvGePosition2dData> (aGeom))
  {
    aPos->Point = ApplyMatrix2d (theXfm, aPos->Point);
    return *this;
  }

  if (NvGeBoundBlock2dData* aBlock = NvGeDataOf<NvGeBoundBlock2dData> (aGeom))
  {
    aBlock->Point = ApplyMatrix2d (theXfm, aBlock->Point);
    aBlock->Vec1 = gp_Vec2d (theXfm (0, 0) * aBlock->Vec1.X() + theXfm (0, 1) * aBlock->Vec1.Y(),
                             theXfm (1, 0) * aBlock->Vec1.X() + theXfm (1, 1) * aBlock->Vec1.Y());
    aBlock->Vec2 = gp_Vec2d (theXfm (0, 0) * aBlock->Vec2.X() + theXfm (0, 1) * aBlock->Vec2.Y(),
                             theXfm (1, 0) * aBlock->Vec2.X() + theXfm (1, 1) * aBlock->Vec2.Y());
    return *this;
  }

  if (NvGeClipBoundary2dData* aClip = NvGeDataOf<NvGeClipBoundary2dData> (aGeom))
  {
    for (gp_Pnt2d& aPnt : aClip->Points)
    {
      aPnt = ApplyMatrix2d (theXfm, aPnt);
    }
    return *this;
  }

  throw NvException ("NvGeEntity2d::transformBy(): unsupported entity kind");
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::translateBy (const NvGeVector2d& theTranslateVec)
{
  return transformBy (NvGeMatrix2d::translation (theTranslateVec));
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::rotateBy (double theAngle, const NvGePoint2d& theWrtPoint)
{
  return transformBy (NvGeMatrix2d::rotation (theAngle, theWrtPoint));
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::mirror (const NvGeLine2d& theLine)
{
  NvGeMatrix2d aMatrix;
  return transformBy (aMatrix.setToMirroring (theLine));
}

//=================================================================================================

NvGeEntity2d& NvGeEntity2d::scaleBy (double theScaleFactor, const NvGePoint2d& theWrtPoint)
{
  return transformBy (NvGeMatrix2d::scaling (theScaleFactor, theWrtPoint));
}

//=================================================================================================

Adesk::Boolean NvGeEntity2d::isOn (const NvGePoint2d& thePnt, const NvGeTol& theTol) const
{
  if (mpImpEnt == nullptr)
  {
    return false;
  }
  const occ::handle<Standard_Transient>& aGeom = mpImpEnt->Geom();

  if (occ::handle<Geom2d_Curve> aCurve = occ::down_cast<Geom2d_Curve> (aGeom))
  {
    Geom2dAPI_ProjectPointOnCurve aProjector (gp_Pnt2d (thePnt.x, thePnt.y), aCurve);
    return aProjector.NbPoints() > 0
        && aProjector.LowerDistance() <= theTol.equalPoint();
  }

  if (const NvGePosition2dData* aPos = NvGeDataOf<NvGePosition2dData> (aGeom))
  {
    return aPos->Point.Distance (gp_Pnt2d (thePnt.x, thePnt.y)) <= theTol.equalPoint();
  }

  if (const NvGeBoundBlock2dData* aBlock = NvGeDataOf<NvGeBoundBlock2dData> (aGeom))
  {
    // Containment in the parallelogram {Point + s*Vec1 + t*Vec2, s,t in [0,1]};
    // a zero vector disables the bound in that direction (ARX semantics).
    const double aPx = thePnt.x - aBlock->Point.X();
    const double aPy = thePnt.y - aBlock->Point.Y();
    const double aDet = aBlock->Vec1.X() * aBlock->Vec2.Y() - aBlock->Vec1.Y() * aBlock->Vec2.X();
    const double aTol = theTol.equalPoint();
    if (std::abs (aDet) > 1e-12)
    {
      const double aS = (aPx * aBlock->Vec2.Y() - aPy * aBlock->Vec2.X()) / aDet;
      const double aT = (aBlock->Vec1.X() * aPy - aBlock->Vec1.Y() * aPx) / aDet;
      const bool isUnbounded1 = aBlock->Vec1.Magnitude() <= aTol;
      const bool isUnbounded2 = aBlock->Vec2.Magnitude() <= aTol;
      return (isUnbounded1 || (aS >= -aTol && aS <= 1.0 + aTol))
          && (isUnbounded2 || (aT >= -aTol && aT <= 1.0 + aTol));
    }
    // Degenerate block: fall back to the segment/point test.
    return aBlock->Point.Distance (gp_Pnt2d (thePnt.x, thePnt.y)) <= aTol;
  }

  if (const NvGeClipBoundary2dData* aClip = NvGeDataOf<NvGeClipBoundary2dData> (aGeom))
  {
    // Winding-style containment: on-boundary points count as on.
    const size_t aNbPnts = aClip->Points.size();
    if (aNbPnts < 3)
    {
      return false;
    }
    const double aTol = theTol.equalPoint();
    // Boundary check first.
    for (size_t anIdx = 0; anIdx < aNbPnts; ++anIdx)
    {
      const gp_Pnt2d& aA = aClip->Points[anIdx];
      const gp_Pnt2d& aB = aClip->Points[(anIdx + 1) % aNbPnts];
      const gp_Vec2d anAB (aB.X() - aA.X(), aB.Y() - aA.Y());
      const gp_Vec2d anAP (thePnt.x - aA.X(), thePnt.y - aA.Y());
      const double aLen = anAB.Magnitude();
      if (aLen <= aTol)
      {
        if (anAP.Magnitude() <= aTol)
        {
          return true;
        }
        continue;
      }
      const double aCross = anAB.X() * anAP.Y() - anAB.Y() * anAP.X();
      if (std::abs (aCross) / aLen <= aTol && anAB.Dot (anAP) >= -aTol
          && anAB.Dot (anAP) <= aLen * aLen + aTol)
      {
        return true;
      }
    }
    // Even-odd rule for strict interior.
    bool isInside = false;
    for (size_t anIdx = 0, aPrev = aNbPnts - 1; anIdx < aNbPnts; aPrev = anIdx++)
    {
      const gp_Pnt2d& aA = aClip->Points[aPrev];
      const gp_Pnt2d& aB = aClip->Points[anIdx];
      if ((aA.Y() > thePnt.y) != (aB.Y() > thePnt.y))
      {
        const double aX = aA.X() + (thePnt.y - aA.Y()) / (aB.Y() - aA.Y()) * (aB.X() - aA.X());
        if (aX > thePnt.x)
        {
          isInside = !isInside;
        }
      }
    }
    return isInside;
  }

  return false;
}

//=================================================================================================

NvGeEntity2d::NvGeEntity2d()
: mpImpEnt (new NvGeImpEntity3d (NvGe::kEntity2d, occ::handle<Standard_Transient>())),
  mDelEnt (1)
{
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeEntity2d::NvGeEntity2d (const NvGeEntity2d& theSrc)
: mpImpEnt (theSrc.mpImpEnt),
  mDelEnt (theSrc.mDelEnt)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
}

//=================================================================================================

NvGeEntity2d::NvGeEntity2d (NvGeImpEntity3d& theImpEnt, int theDelEnt)
: mpImpEnt (&theImpEnt),
  mDelEnt (theDelEnt)
{
  if (mDelEnt)
  {
    theImpEnt.Ref();
  }
}

//=================================================================================================

NvGeEntity2d::NvGeEntity2d (NvGeImpEntity3d* theImpEnt)
: mpImpEnt (theImpEnt),
  mDelEnt (1)
{
  // Adopts a freshly created impl (reference count 0 so far).
  if (mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
}

//=================================================================================================

bool NvGeIsKindOf (NvGe::EntityId theType, NvGe::EntityId theParent)
{
  if (theType == theParent)
  {
    return true;
  }
  for (;;)
  {
    const NvGe::EntityId aParent = ParentEntityOf (theType);
    if (aParent == theType)
    {
      return false; // hierarchy root reached
    }
    if (aParent == theParent)
    {
      return true;
    }
    theType = aParent;
  }
}
