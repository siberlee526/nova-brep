// geent3d.cpp - implementation of NvGeEntity3d, the 3d pimpl entity root.
//
// Mirrors geent2d.cpp: wrapper mechanics (reference counting, RTTI through
// entity ids, copy) plus the centralized dispatch of transformBy / isEqualTo
// / isOn over the impl geometry kinds (Geom_Curve, Geom_Surface and the 3d
// data holders from geimpdata.h).

#include <geent3d.h>

#include <Nova.h>
#include <gegblabb.h>
#include <gegblge.h>
#include <gemat3d.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <geimpdata.h>

#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <cmath>

namespace
{

//! Transforms a 3d OCCT curve on a private copy. B-splines absorb any
//! affine matrix exactly through their poles; analytic curves need a
//! conformal matrix.
occ::handle<Geom_Curve> TransformedCurve3d (const occ::handle<Geom_Curve>& theCurve,
                                            const NvGeMatrix3d& theMat)
{
  occ::handle<Geom_BSplineCurve> aSpline = occ::down_cast<Geom_BSplineCurve> (theCurve);
  if (!aSpline.IsNull())
  {
    occ::handle<Geom_BSplineCurve> aCopy = occ::down_cast<Geom_BSplineCurve> (aSpline->Copy());
    const int aNbPoles = aCopy->NbPoles();
    for (int aPole = 1; aPole <= aNbPoles; ++aPole)
    {
      aCopy->SetPole (aPole, ApplyMatrix3d (theMat, aCopy->Pole (aPole)));
    }
    return aCopy;
  }
  occ::handle<Geom_Curve> aCopy = occ::down_cast<Geom_Curve> (theCurve->Copy());
  aCopy->Transform (TrsfFromMatrix (theMat));
  return aCopy;
}

//! Transforms a surface on a private copy; B-spline surfaces absorb any
//! affine matrix exactly, analytic surfaces need a conformal matrix.
occ::handle<Geom_Surface> TransformedSurface (const occ::handle<Geom_Surface>& theSurface,
                                              const NvGeMatrix3d& theMat)
{
  occ::handle<Geom_BSplineSurface> aSpline = occ::down_cast<Geom_BSplineSurface> (theSurface);
  if (!aSpline.IsNull())
  {
    occ::handle<Geom_BSplineSurface> aCopy = occ::down_cast<Geom_BSplineSurface> (aSpline->Copy());
    for (int anU = 1; anU <= aCopy->NbUPoles(); ++anU)
    {
      for (int aV = 1; aV <= aCopy->NbVPoles(); ++aV)
      {
        aCopy->SetPole (anU, aV, ApplyMatrix3d (theMat, aCopy->Pole (anU, aV)));
      }
    }
    return aCopy;
  }
  occ::handle<Geom_Surface> aCopy = occ::down_cast<Geom_Surface> (theSurface->Copy());
  aCopy->Transform (TrsfFromMatrix (theMat));
  return aCopy;
}

//! Samples two 3d curves on a common parameter window and compares points.
bool Curves3dAreEqual (const occ::handle<Geom_Curve>& theC1,
                       const occ::handle<Geom_Curve>& theC2,
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
    return true;
  }
  const int THE_NB_SAMPLES = 17;
  for (int aSample = 0; aSample < THE_NB_SAMPLES; ++aSample)
  {
    const double aParam = aFirst + (aLast - aFirst) * aSample / (THE_NB_SAMPLES - 1);
    if (theC1->Value (aParam).Distance (theC2->Value (aParam)) > theTol.equalPoint())
    {
      return false;
    }
  }
  return true;
}

//! Samples two surfaces on a common uv window and compares points.
bool SurfacesAreEqual (const occ::handle<Geom_Surface>& theS1,
                       const occ::handle<Geom_Surface>& theS2,
                       const NvGeTol& theTol)
{
  double anU1min = 0.0, anU1max = 0.0, aV1min = 0.0, aV1max = 0.0;
  double anU2min = 0.0, anU2max = 0.0, aV2min = 0.0, aV2max = 0.0;
  theS1->Bounds (anU1min, anU1max, aV1min, aV1max);
  theS2->Bounds (anU2min, anU2max, aV2min, aV2max);
  if (anU1min != anU2min || anU1max != anU2max || aV1min != aV2min || aV1max != aV2max)
  {
    return false;
  }
  // Sampling window guard (see Curves3dAreEqual): unbounded surfaces store
  // huge-but-finite uv domains that would amplify rounding noise.
  if (!std::isfinite (anU1min) || !std::isfinite (anU1max) || anU1max - anU1min > 1e3)
  {
    const double aMid = 0.5 * (std::isfinite (anU1min + anU1max) ? anU1min + anU1max : 0.0);
    anU1min = aMid - 1.0;
    anU1max = aMid + 1.0;
  }
  if (!std::isfinite (aV1min) || !std::isfinite (aV1max) || aV1max - aV1min > 1e3)
  {
    const double aMid = 0.5 * (std::isfinite (aV1min + aV1max) ? aV1min + aV1max : 0.0);
    aV1min = aMid - 1.0;
    aV1max = aMid + 1.0;
  }
  const int THE_NB_SAMPLES = 9;
  for (int anU = 0; anU < THE_NB_SAMPLES; ++anU)
  {
    const double aU = anU1min + (anU1max - anU1min) * anU / (THE_NB_SAMPLES - 1);
    for (int aVSample = 0; aVSample < THE_NB_SAMPLES; ++aVSample)
    {
      const double aV = aV1min + (aV1max - aV1min) * aVSample / (THE_NB_SAMPLES - 1);
      if (theS1->Value (aU, aV).Distance (theS2->Value (aU, aV)) > theTol.equalPoint())
      {
        return false;
      }
    }
  }
  return true;
}

//! Determinant of an n x n system built from up to three rows (n <= 3).
double DetOfSystem (const double theM[3][3], int theSize)
{
  if (theSize == 1)
  {
    return theM[0][0];
  }
  if (theSize == 2)
  {
    return theM[0][0] * theM[1][1] - theM[0][1] * theM[1][0];
  }
  return theM[0][0] * (theM[1][1] * theM[2][2] - theM[1][2] * theM[2][1])
       - theM[0][1] * (theM[1][0] * theM[2][2] - theM[1][2] * theM[2][0])
       + theM[0][2] * (theM[1][0] * theM[2][1] - theM[1][1] * theM[2][0]);
}

//! Containment in the parallelepiped {Point + s*V1 + t*V2 + r*V3,
//! coefficients in [0,1]}; a zero vector disables the bound in that
//! direction (ARX semantics).
bool IsInBlock3d (const NvGeBoundBlock3dData* theBlock, const gp_Pnt& thePnt, double theTol)
{
  const gp_Vec anAP (thePnt.XYZ() - theBlock->Point.XYZ());
  const gp_Vec aVecs[3] = { theBlock->Vec1, theBlock->Vec2, theBlock->Vec3 };
  const bool anUnb[3] = { aVecs[0].Magnitude() <= theTol,
                          aVecs[1].Magnitude() <= theTol,
                          aVecs[2].Magnitude() <= theTol };

  // Build the row system P = sum s_i * V_i over the bounded directions only.
  double aM[3][3] = { { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
  double aB[3] = { 0.0, 0.0, 0.0 };
  int aRow = 0;
  for (int aDir = 0; aDir < 3; ++aDir)
  {
    if (anUnb[aDir])
    {
      continue;
    }
    aM[aRow][0] = aVecs[aDir].X();
    aM[aRow][1] = aVecs[aDir].Y();
    aM[aRow][2] = aVecs[aDir].Z();
    aB[aRow] = (aRow == 0) ? anAP.X() : ((aRow == 1) ? anAP.Y() : anAP.Z());
    ++aRow;
  }
  if (aRow == 0)
  {
    return true; // fully unbounded block
  }
  const double aDet = DetOfSystem (aM, aRow);
  if (std::abs (aDet) <= 1e-12)
  {
    return false; // degenerate bounded block
  }
  int aCol = 0;
  for (int aDir = 0; aDir < 3; ++aDir)
  {
    if (anUnb[aDir])
    {
      continue; // free direction, no constraint on its coefficient
    }
    double aMCol[3][3];
    for (int aR = 0; aR < aRow; ++aR)
    {
      for (int aC = 0; aC < aRow; ++aC)
      {
        aMCol[aR][aC] = (aC == aCol) ? aB[aR] : aM[aR][aC];
      }
    }
    const double aCoef = DetOfSystem (aMCol, aRow) / aDet;
    if (aCoef < -theTol || aCoef > 1.0 + theTol)
    {
      return false;
    }
    ++aCol;
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

NvGeEntity3d::~NvGeEntity3d()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
}

//=================================================================================================

Nova::Boolean NvGeEntity3d::isKindOf (NvGe::EntityId theEntType) const
{
  return NvGeIsKindOf (type(), theEntType);
}

//=================================================================================================

NvGe::EntityId NvGeEntity3d::type() const
{
  return mpImpEnt != nullptr ? mpImpEnt->Type() : NvGe::kEntity3d;
}

//=================================================================================================

NvGeEntity3d* NvGeEntity3d::copy() const
{
  return newEntity3d (mpImpEnt != nullptr ? mpImpEnt->CloneShallow() : nullptr);
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::operator = (const NvGeEntity3d& theEntity)
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

Nova::Boolean NvGeEntity3d::operator == (const NvGeEntity3d& theEntity) const
{
  return isEqualTo (theEntity);
}

//=================================================================================================

Nova::Boolean NvGeEntity3d::operator != (const NvGeEntity3d& theEntity) const
{
  return !isEqualTo (theEntity);
}

//=================================================================================================

Nova::Boolean NvGeEntity3d::isEqualTo (const NvGeEntity3d& theEntity, const NvGeTol& theTol) const
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
    return true;
  }

  const occ::handle<Standard_Transient>& aGeom1 = mpImpEnt->Geom();
  const occ::handle<Standard_Transient>& aGeom2 = theEntity.mpImpEnt->Geom();

  occ::handle<Geom_Curve> aCurve1 = occ::down_cast<Geom_Curve> (aGeom1);
  occ::handle<Geom_Curve> aCurve2 = occ::down_cast<Geom_Curve> (aGeom2);
  if (!aCurve1.IsNull() && !aCurve2.IsNull())
  {
    return Curves3dAreEqual (aCurve1, aCurve2, theTol);
  }

  occ::handle<Geom_Surface> aSurf1 = occ::down_cast<Geom_Surface> (aGeom1);
  occ::handle<Geom_Surface> aSurf2 = occ::down_cast<Geom_Surface> (aGeom2);
  if (!aSurf1.IsNull() && !aSurf2.IsNull())
  {
    return SurfacesAreEqual (aSurf1, aSurf2, theTol);
  }

  if (const NvGePosition3dData* aPos1 = NvGeDataOf<NvGePosition3dData> (aGeom1))
  {
    const NvGePosition3dData* aPos2 = NvGeDataOf<NvGePosition3dData> (aGeom2);
    return aPos2 != nullptr
        && aPos1->Point.Distance (aPos2->Point) <= theTol.equalPoint();
  }

  if (const NvGeBoundBlock3dData* aBlock1 = NvGeDataOf<NvGeBoundBlock3dData> (aGeom1))
  {
    const NvGeBoundBlock3dData* aBlock2 = NvGeDataOf<NvGeBoundBlock3dData> (aGeom2);
    return aBlock2 != nullptr
        && aBlock1->Point.Distance (aBlock2->Point) <= theTol.equalPoint()
        && std::abs (aBlock1->Vec1.Magnitude() - aBlock2->Vec1.Magnitude()) <= theTol.equalVector()
        && std::abs (aBlock1->Vec2.Magnitude() - aBlock2->Vec2.Magnitude()) <= theTol.equalVector()
        && std::abs (aBlock1->Vec3.Magnitude() - aBlock2->Vec3.Magnitude()) <= theTol.equalVector();
  }

  return false;
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::transformBy (const NvGeMatrix3d& theXfm)
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

  if (occ::handle<Geom_Curve> aCurve = occ::down_cast<Geom_Curve> (aGeom))
  {
    mpImpEnt->SetGeom (TransformedCurve3d (aCurve, theXfm));
    return *this;
  }

  if (occ::handle<Geom_Surface> aSurface = occ::down_cast<Geom_Surface> (aGeom))
  {
    mpImpEnt->SetGeom (TransformedSurface (aSurface, theXfm));
    return *this;
  }

  if (NvGePosition3dData* aPos = NvGeDataOf<NvGePosition3dData> (aGeom))
  {
    aPos->Point = ApplyMatrix3d (theXfm, aPos->Point);
    return *this;
  }

  if (NvGeBoundBlock3dData* aBlock = NvGeDataOf<NvGeBoundBlock3dData> (aGeom))
  {
    aBlock->Point = ApplyMatrix3d (theXfm, aBlock->Point);
    aBlock->Vec1 = ApplyMatrix3d (theXfm, aBlock->Vec1);
    aBlock->Vec2 = ApplyMatrix3d (theXfm, aBlock->Vec2);
    aBlock->Vec3 = ApplyMatrix3d (theXfm, aBlock->Vec3);
    return *this;
  }

  throw NvException ("NvGeEntity3d::transformBy(): unsupported entity kind");
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::translateBy (const NvGeVector3d& theTranslateVec)
{
  return transformBy (NvGeMatrix3d::translation (theTranslateVec));
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::rotateBy (double theAngle, const NvGeVector3d& theAxis,
                                      const NvGePoint3d& theWrtPoint)
{
  return transformBy (NvGeMatrix3d::rotation (theAngle, theAxis, theWrtPoint));
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::mirror (const NvGePlane& thePlane)
{
  NvGeMatrix3d aMatrix;
  return transformBy (aMatrix.setToMirroring (thePlane));
}

//=================================================================================================

NvGeEntity3d& NvGeEntity3d::scaleBy (double theScaleFactor, const NvGePoint3d& theWrtPoint)
{
  return transformBy (NvGeMatrix3d::scaling (theScaleFactor, theWrtPoint));
}

//=================================================================================================

Nova::Boolean NvGeEntity3d::isOn (const NvGePoint3d& thePnt, const NvGeTol& theTol) const
{
  if (mpImpEnt == nullptr)
  {
    return false;
  }
  const occ::handle<Standard_Transient>& aGeom = mpImpEnt->Geom();
  const gp_Pnt aPnt (thePnt.x, thePnt.y, thePnt.z);

  if (occ::handle<Geom_Curve> aCurve = occ::down_cast<Geom_Curve> (aGeom))
  {
    GeomAPI_ProjectPointOnCurve aProjector (aPnt, aCurve);
    return aProjector.NbPoints() > 0
        && aProjector.LowerDistance() <= theTol.equalPoint();
  }

  if (occ::handle<Geom_Surface> aSurface = occ::down_cast<Geom_Surface> (aGeom))
  {
    GeomAPI_ProjectPointOnSurf aProjector (aPnt, aSurface);
    return aProjector.NbPoints() > 0
        && aProjector.LowerDistance() <= theTol.equalPoint();
  }

  if (const NvGePosition3dData* aPos = NvGeDataOf<NvGePosition3dData> (aGeom))
  {
    return aPos->Point.Distance (aPnt) <= theTol.equalPoint();
  }

  if (const NvGeBoundBlock3dData* aBlock = NvGeDataOf<NvGeBoundBlock3dData> (aGeom))
  {
    return IsInBlock3d (aBlock, aPnt, theTol.equalPoint());
  }

  return false;
}

//=================================================================================================

NvGeEntity3d::NvGeEntity3d()
: mpImpEnt (new NvGeImpEntity3d (NvGe::kEntity3d, occ::handle<Standard_Transient>())),
  mDelEnt (1)
{
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeEntity3d::NvGeEntity3d (const NvGeEntity3d& theSrc)
: mpImpEnt (theSrc.mpImpEnt),
  mDelEnt (theSrc.mDelEnt)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
}

//=================================================================================================

NvGeEntity3d::NvGeEntity3d (NvGeImpEntity3d& theImpEnt, int theDelEnt)
: mpImpEnt (&theImpEnt),
  mDelEnt (theDelEnt)
{
  if (mDelEnt)
  {
    theImpEnt.Ref();
  }
}

//=================================================================================================

NvGeEntity3d::NvGeEntity3d (NvGeImpEntity3d* theImpEnt)
: mpImpEnt (theImpEnt),
  mDelEnt (1)
{
  // Adopts a freshly created impl (reference count 0 so far).
  if (mpImpEnt != nullptr)
  {
    mpImpEnt->Ref();
  }
}
