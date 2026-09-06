// gesphere.cpp - implementation of NvGeSphere.
//
// The sphere stores a Geom_SphericalSurface in its impl, parameterized as
// P(u,v) = C + R*cos(v)*(cos(u)*U + sin(u)*V) + R*sin(v)*N with u in
// [0, 2*PI] and v in [-PI/2, PI/2]; the north axis is the main direction
// of the stored frame. A restricted angle range is represented by a
// Geom_RectangularTrimmedSurface over the analytic sphere (the same
// convention as NvGeBoundedPlane); the getters unwrap the patch.

#include <gesphere.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gelent3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>
#include <string>

namespace
{

//! Canonical parameter bounds of a full sphere.
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kHalfPi = 1.57079632679489661923;

//! Installs sphere geometry, replacing the base placeholder impl.
void MakeSphere (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  theImp = new NvGeImpEntity3d (NvGe::kSphere, theGeom);
  theImp->Ref();
}

//! Replaces the geometry with copy-on-write discipline.
void SetSphere (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theGeom);
}

//! Analytic sphere of this entity, unwrapping a trimmed patch.
occ::handle<Geom_SphericalSurface> SphereOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_SphericalSurface> aSphere =
    occ::down_cast<Geom_SphericalSurface> (theImp->Geom());
  if (aSphere.IsNull())
  {
    const occ::handle<Geom_RectangularTrimmedSurface> aPatch =
      occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
    if (!aPatch.IsNull())
    {
      aSphere = occ::down_cast<Geom_SphericalSurface> (aPatch->BasisSurface());
    }
  }
  if (aSphere.IsNull())
  {
    throw NvException ("NvGeSphere: the entity holds no sphere geometry");
  }
  return aSphere;
}

//! Parameter bounds of the stored geometry (patch bounds when trimmed).
void StoredBounds (const NvGeImpEntity3d* theImp,
                   double& theU1, double& theU2, double& theV1, double& theV2)
{
  NvGeSurfaceOf (theImp)->Bounds (theU1, theU2, theV1, theV2);
}

//! Patch of the basis surface between the given angle bounds; the full
//! canonical range keeps the plain analytic surface.
occ::handle<Geom_Surface> PatchOf (const occ::handle<Geom_SphericalSurface>& theBasis,
                                   double theU1, double theU2, double theV1, double theV2)
{
  if (theU1 <= 0.0 && theU2 >= kTwoPi && theV1 <= -kHalfPi && theV2 >= kHalfPi)
  {
    return occ::handle<Geom_Surface> (theBasis);
  }
  return occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
    theBasis, theU1, theU2, theV1, theV2));
}

//! Guards a radius: rejects negative and non-finite values.
void CheckRadius (const char* theMethod, double theRadius)
{
  if (!(theRadius >= 0.0))
  {
    throw NvException (std::string ("NvGeSphere::") + theMethod
                       + "(): the radius must be non-negative");
  }
}

//! Guards an angle interval against the OCCT trim constraints.
void CheckAngleRange (const char* theMethod, double theStart, double theEnd)
{
  if (!(theStart < theEnd))
  {
    throw NvException (std::string ("NvGeSphere::") + theMethod
                       + "(): the start angle must be less than the end angle");
  }
}

//! Guards a v angle interval; the sphere v direction is bounded by the poles.
void CheckVAngleRange (double theStart, double theEnd)
{
  if (theStart < -kHalfPi || theEnd > kHalfPi)
  {
    throw NvException ("NvGeSphere: the v angles must lie within [-PI/2, PI/2]");
  }
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeSphere::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

//! Builds the positioned analytic sphere from the defining frame.
occ::handle<Geom_SphericalSurface> BasisOf (double theRadius, const NvGePoint3d& theCenter,
                                            const NvGeVector3d& theNorthAxis,
                                            const NvGeVector3d& theRefAxis,
                                            const char* theMethod)
{
  const gp_Dir aNorth = DirOrThrow (theNorthAxis.x, theNorthAxis.y, theNorthAxis.z, theMethod);
  const gp_Dir aRef = DirOrThrow (theRefAxis.x, theRefAxis.y, theRefAxis.z, theMethod);
  if (1.0 - std::abs (aNorth.Dot (aRef)) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeSphere::") + theMethod
                       + "(): the north and reference axes must not be parallel");
  }
  return occ::handle<Geom_SphericalSurface> (new Geom_SphericalSurface (
    gp_Ax3 (gp_Pnt (theCenter.x, theCenter.y, theCenter.z), aNorth, aRef), theRadius));
}

//! Replaces the stored geometry with a patch of the current basis.
void SetPatch (NvGeImpEntity3d*& theImp, double theU1, double theU2, double theV1, double theV2)
{
  SetSphere (theImp, PatchOf (SphereOf (theImp), theU1, theU2, theV1, theV2));
}

} // namespace

//=================================================================================================

NvGeSphere::NvGeSphere()
{
  // Default: unit sphere centered at the world origin.
  MakeSphere (mpImpEnt, PatchOf (
    BasisOf (1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0),
             NvGeVector3d (1.0, 0.0, 0.0), "NvGeSphere"),
    0.0, kTwoPi, -kHalfPi, kHalfPi));
}

//=================================================================================================

NvGeSphere::NvGeSphere (const NvGeSphere& theSrc)
: NvGeSurface (theSrc)
{
}

//=================================================================================================

NvGeSphere::NvGeSphere (double theRadius, const NvGePoint3d& theCenter)
{
  CheckRadius ("NvGeSphere", theRadius);
  // A sphere defined only by radius and center carries the default frame
  // (+Z north, +X reference) and the full canonical angle range.
  MakeSphere (mpImpEnt, PatchOf (
    BasisOf (theRadius, theCenter, NvGeVector3d (0.0, 0.0, 1.0),
             NvGeVector3d (1.0, 0.0, 0.0), "NvGeSphere"),
    0.0, kTwoPi, -kHalfPi, kHalfPi));
}

//=================================================================================================

NvGeSphere::NvGeSphere (double theRadius, const NvGePoint3d& theCenter,
                        const NvGeVector3d& theNorthAxis, const NvGeVector3d& theRefAxis,
                        double theStartAngleU, double theEndAngleU,
                        double theStartAngleV, double theEndAngleV)
{
  CheckRadius ("NvGeSphere", theRadius);
  CheckAngleRange ("NvGeSphere", theStartAngleU, theEndAngleU);
  CheckAngleRange ("NvGeSphere", theStartAngleV, theEndAngleV);
  CheckVAngleRange (theStartAngleV, theEndAngleV);
  MakeSphere (mpImpEnt, PatchOf (
    BasisOf (theRadius, theCenter, theNorthAxis, theRefAxis, "NvGeSphere"),
    theStartAngleU, theEndAngleU, theStartAngleV, theEndAngleV));
}

//=================================================================================================

double NvGeSphere::radius() const
{
  return SphereOf (mpImpEnt)->Radius();
}

//=================================================================================================

NvGePoint3d NvGeSphere::center() const
{
  const gp_Pnt aCenter = SphereOf (mpImpEnt)->Location();
  return NvGePoint3d (aCenter.X(), aCenter.Y(), aCenter.Z());
}

//=================================================================================================

void NvGeSphere::getAnglesInU (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = anU1;
  theEnd = anU2;
}

//=================================================================================================

void NvGeSphere::getAnglesInV (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = aV1;
  theEnd = aV2;
}

//=================================================================================================

NvGeVector3d NvGeSphere::northAxis() const
{
  const gp_Dir aNorth = SphereOf (mpImpEnt)->Position().Direction();
  return NvGeVector3d (aNorth.X(), aNorth.Y(), aNorth.Z());
}

//=================================================================================================

NvGeVector3d NvGeSphere::refAxis() const
{
  const gp_Dir aRef = SphereOf (mpImpEnt)->Position().XDirection();
  return NvGeVector3d (aRef.X(), aRef.Y(), aRef.Z());
}

//=================================================================================================

NvGePoint3d NvGeSphere::northPole() const
{
  const occ::handle<Geom_SphericalSurface> aSphere = SphereOf (mpImpEnt);
  const gp_Pnt aCenter = aSphere->Location();
  const gp_Dir aNorth = aSphere->Position().Direction();
  const double aRadius = aSphere->Radius();
  return NvGePoint3d (aCenter.X() + aRadius * aNorth.X(),
                      aCenter.Y() + aRadius * aNorth.Y(),
                      aCenter.Z() + aRadius * aNorth.Z());
}

//=================================================================================================

NvGePoint3d NvGeSphere::southPole() const
{
  const occ::handle<Geom_SphericalSurface> aSphere = SphereOf (mpImpEnt);
  const gp_Pnt aCenter = aSphere->Location();
  const gp_Dir aNorth = aSphere->Position().Direction();
  const double aRadius = aSphere->Radius();
  return NvGePoint3d (aCenter.X() - aRadius * aNorth.X(),
                      aCenter.Y() - aRadius * aNorth.Y(),
                      aCenter.Z() - aRadius * aNorth.Z());
}

//=================================================================================================

Nova::Boolean NvGeSphere::isOuterNormal() const
{
  // OCCT builds elementary surfaces with the normal directed away from the
  // material side; for the sphere this is the outer normal.
  return true;
}

//=================================================================================================

Nova::Boolean NvGeSphere::isClosed (const NvGeTol& theTol) const
{
  // Closed means a full sphere: the u range covers a whole revolution and
  // the v range reaches both poles.
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  return anU1 <= theTol.equalPoint()
      && anU2 >= kTwoPi - theTol.equalPoint()
      && aV1 <= -kHalfPi + theTol.equalPoint()
      && aV2 >= kHalfPi - theTol.equalPoint();
}

//=================================================================================================

NvGeSphere& NvGeSphere::setRadius (double theRadius)
{
  CheckRadius ("setRadius", theRadius);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const occ::handle<Geom_SphericalSurface> aBasis = SphereOf (mpImpEnt);
  const occ::handle<Geom_SphericalSurface> aNew (new Geom_SphericalSurface (
    aBasis->Position(), theRadius));
  SetSphere (mpImpEnt, PatchOf (aNew, anU1, anU2, aV1, aV2));
  return *this;
}

//=================================================================================================

NvGeSphere& NvGeSphere::setAnglesInU (double theStart, double theEnd)
{
  CheckAngleRange ("setAnglesInU", theStart, theEnd);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, theStart, theEnd, aV1, aV2);
  return *this;
}

//=================================================================================================

NvGeSphere& NvGeSphere::setAnglesInV (double theStart, double theEnd)
{
  CheckAngleRange ("setAnglesInV", theStart, theEnd);
  CheckVAngleRange (theStart, theEnd);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, anU1, anU2, theStart, theEnd);
  return *this;
}

//=================================================================================================

NvGeSphere& NvGeSphere::set (double theRadius, const NvGePoint3d& theCenter)
{
  *this = NvGeSphere (theRadius, theCenter);
  return *this;
}

//=================================================================================================

NvGeSphere& NvGeSphere::set (double theRadius, const NvGePoint3d& theCenter,
                             const NvGeVector3d& theNorthAxis, const NvGeVector3d& theRefAxis,
                             double theStartAngleU, double theEndAngleU,
                             double theStartAngleV, double theEndAngleV)
{
  *this = NvGeSphere (theRadius, theCenter, theNorthAxis, theRefAxis,
                      theStartAngleU, theEndAngleU, theStartAngleV, theEndAngleV);
  return *this;
}

//=================================================================================================

NvGeSphere& NvGeSphere::operator = (const NvGeSphere& theSrc)
{
  NvGeSurface::operator= (theSrc);
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeSphere::intersectWith (const NvGeLinearEnt3d& theLinEnt, int& theIntn,
                                          NvGePoint3d& thePnt1, NvGePoint3d& thePnt2,
                                          const NvGeTol& theTol) const
{
  (void)theLinEnt;
  (void)thePnt1;
  (void)thePnt2;
  (void)theTol;
  theIntn = 0; // line/sphere intersection is not provided yet
  return false;
}
