// gecylndr.cpp - implementation of NvGeCylinder.
//
// The cylinder stores a Geom_CylindricalSurface in its impl, parameterized
// as P(u,v) = O + R*cos(u)*X + R*sin(u)*Y + v*N with u in [0, 2*PI] and v
// unbounded; the v parameter equals the signed axial height measured from
// the origin plane. A restricted angle range and/or height interval is
// represented by a Geom_RectangularTrimmedSurface over the analytic
// cylinder (the same convention as NvGeBoundedPlane); the getters unwrap
// the patch and map huge OCCT bounds back to infinities.

#include <gecylndr.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <geintrvl.h>
#include <gelent3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_CylindricalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>
#include <string>

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

//! Huge-bound threshold of NvGeSurface::getEnvelope; OCCT stores unbounded
//! domains as huge values, which are normalized to infinities.
bool IsHuge (double theValue)
{
  return std::abs (theValue) >= 1e100;
}

//! Installs cylinder geometry, replacing the base placeholder impl.
void MakeCylinder (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  theImp = new NvGeImpEntity3d (NvGe::kCylinder, theGeom);
  theImp->Ref();
}

//! Replaces the geometry with copy-on-write discipline.
void SetCylinder (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theGeom);
}

//! Analytic cylinder of this entity, unwrapping a trimmed patch.
occ::handle<Geom_CylindricalSurface> CylinderOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_CylindricalSurface> aCylinder =
    occ::down_cast<Geom_CylindricalSurface> (theImp->Geom());
  if (aCylinder.IsNull())
  {
    const occ::handle<Geom_RectangularTrimmedSurface> aPatch =
      occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
    if (!aPatch.IsNull())
    {
      aCylinder = occ::down_cast<Geom_CylindricalSurface> (aPatch->BasisSurface());
    }
  }
  if (aCylinder.IsNull())
  {
    throw NvException ("NvGeCylinder: the entity holds no cylinder geometry");
  }
  return aCylinder;
}

//! Parameter bounds of the stored geometry; huge OCCT bounds of unbounded
//! directions are normalized to infinities.
void StoredBounds (const NvGeImpEntity3d* theImp,
                   double& theU1, double& theU2, double& theV1, double& theV2)
{
  NvGeSurfaceOf (theImp)->Bounds (theU1, theU2, theV1, theV2);
  const double anInf = std::numeric_limits<double>::infinity();
  theU1 = IsHuge (theU1) ? -anInf : theU1;
  theU2 = IsHuge (theU2) ? anInf : theU2;
  theV1 = IsHuge (theV1) ? -anInf : theV1;
  theV2 = IsHuge (theV2) ? anInf : theV2;
}

//! Patch of the basis surface; an infinite bound keeps the corresponding
//! direction untrimmed, and no restriction keeps the plain surface.
occ::handle<Geom_Surface> PatchOf (const occ::handle<Geom_CylindricalSurface>& theBasis,
                                   double theU1, double theU2, double theV1, double theV2)
{
  const double anInf = std::numeric_limits<double>::infinity();
  const bool aTrimU = theU1 > -anInf && theU2 < anInf;
  const bool aTrimV = theV1 > -anInf && theV2 < anInf;
  if (aTrimU && aTrimV)
  {
    return occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
      theBasis, theU1, theU2, theV1, theV2));
  }
  if (aTrimU)
  {
    return occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
      occ::handle<Geom_Surface> (theBasis), theU1, theU2, true));
  }
  if (aTrimV)
  {
    return occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
      occ::handle<Geom_Surface> (theBasis), theV1, theV2, false));
  }
  return occ::handle<Geom_Surface> (theBasis);
}

//! Guards a radius: rejects negative and non-finite values.
void CheckRadius (const char* theMethod, double theRadius)
{
  if (!(theRadius >= 0.0))
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the radius must be non-negative");
  }
}

//! Guards an angle interval against the OCCT trim constraints.
void CheckAngleRange (const char* theMethod, double theStart, double theEnd)
{
  if (!(theStart < theEnd))
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the start angle must be less than the end angle");
  }
}

//! Guards a height interval: both bounds must be finite and ordered.
void CheckHeight (const char* theMethod, const NvGeInterval& theHeight)
{
  if (!theHeight.isBounded())
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the height interval must be bounded");
  }
  if (!(theHeight.lowerBound() < theHeight.upperBound()))
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the height interval must not be empty");
  }
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

//! Builds the positioned analytic cylinder from the defining frame.
occ::handle<Geom_CylindricalSurface> BasisOf (double theRadius, const NvGePoint3d& theOrigin,
                                              const NvGeVector3d& theAxisOfSymmetry,
                                              const NvGeVector3d& theRefAxis,
                                              const char* theMethod)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, theMethod);
  const gp_Dir aRef = DirOrThrow (theRefAxis.x, theRefAxis.y, theRefAxis.z, theMethod);
  if (1.0 - std::abs (anAxis.Dot (aRef)) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCylinder::") + theMethod
                       + "(): the axis of symmetry and the reference axis must not be parallel");
  }
  return occ::handle<Geom_CylindricalSurface> (new Geom_CylindricalSurface (
    gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), anAxis, aRef), theRadius));
}

//! Builds the analytic cylinder for a definition without reference axis;
//! the frame X direction is then seeded from the axis (gp_Ax3 two
//! direction convention), which never raises for a valid direction.
occ::handle<Geom_CylindricalSurface> BasisOfAxis (double theRadius, const NvGePoint3d& theOrigin,
                                                  const NvGeVector3d& theAxisOfSymmetry)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, "NvGeCylinder");
  return occ::handle<Geom_CylindricalSurface> (new Geom_CylindricalSurface (
    gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), anAxis), theRadius));
}

//! Replaces the stored geometry with a patch of the current basis.
void SetPatch (NvGeImpEntity3d*& theImp, double theU1, double theU2, double theV1, double theV2)
{
  SetCylinder (theImp, PatchOf (CylinderOf (theImp), theU1, theU2, theV1, theV2));
}

} // namespace

//=================================================================================================

NvGeCylinder::NvGeCylinder()
{
  // Default: unit cylinder at the world origin along +Z, full angle range
  // and unbounded height.
  MakeCylinder (mpImpEnt, PatchOf (
    BasisOfAxis (1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0)),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeCylinder::NvGeCylinder (const NvGeCylinder& theSrc)
: NvGeSurface (theSrc)
{
}

//=================================================================================================

NvGeCylinder::NvGeCylinder (double theRadius, const NvGePoint3d& theOrigin,
                            const NvGeVector3d& theAxisOfSymmetry)
{
  CheckRadius ("NvGeCylinder", theRadius);
  // A cylinder defined without a reference axis carries the canonical frame
  // seeded from the axis of symmetry and is unbounded.
  MakeCylinder (mpImpEnt, PatchOf (
    BasisOfAxis (theRadius, theOrigin, theAxisOfSymmetry),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeCylinder::NvGeCylinder (double theRadius, const NvGePoint3d& theOrigin,
                            const NvGeVector3d& theAxisOfSymmetry,
                            const NvGeVector3d& theRefAxis,
                            const NvGeInterval& theHeight,
                            double theStartAngle, double theEndAngle)
{
  CheckRadius ("NvGeCylinder", theRadius);
  CheckAngleRange ("NvGeCylinder", theStartAngle, theEndAngle);
  CheckHeight ("NvGeCylinder", theHeight);
  MakeCylinder (mpImpEnt, PatchOf (
    BasisOf (theRadius, theOrigin, theAxisOfSymmetry, theRefAxis, "NvGeCylinder"),
    theStartAngle, theEndAngle, theHeight.lowerBound(), theHeight.upperBound()));
}

//=================================================================================================

double NvGeCylinder::radius() const
{
  return CylinderOf (mpImpEnt)->Radius();
}

//=================================================================================================

NvGePoint3d NvGeCylinder::origin() const
{
  const gp_Pnt anOrigin = CylinderOf (mpImpEnt)->Location();
  return NvGePoint3d (anOrigin.X(), anOrigin.Y(), anOrigin.Z());
}

//=================================================================================================

void NvGeCylinder::getAngles (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = anU1;
  theEnd = anU2;
}

//=================================================================================================

void NvGeCylinder::getHeight (NvGeInterval& theHeight) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const double anInf = std::numeric_limits<double>::infinity();
  if (aV1 > -anInf && aV2 < anInf)
  {
    theHeight.set (aV1, aV2);
  }
  else
  {
    theHeight.set(); // unbounded height
  }
}

//=================================================================================================

double NvGeCylinder::heightAt (double theU) const
{
  // The ARX natural parameterization measures v as the signed axial height
  // from the origin plane, so the parameter-to-height map is the identity.
  return theU;
}

//=================================================================================================

NvGeVector3d NvGeCylinder::axisOfSymmetry() const
{
  const gp_Dir anAxis = CylinderOf (mpImpEnt)->Position().Direction();
  return NvGeVector3d (anAxis.X(), anAxis.Y(), anAxis.Z());
}

//=================================================================================================

NvGeVector3d NvGeCylinder::refAxis() const
{
  const gp_Dir aRef = CylinderOf (mpImpEnt)->Position().XDirection();
  return NvGeVector3d (aRef.X(), aRef.Y(), aRef.Z());
}

//=================================================================================================

Adesk::Boolean NvGeCylinder::isOuterNormal() const
{
  // OCCT builds elementary surfaces with the normal oriented towards the
  // outside region of the cylinder.
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCylinder::isClosed (const NvGeTol& theTol) const
{
  // Closed means a full revolution in u; a bounded height still leaves a
  // closed tube.
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  return anU1 <= theTol.equalPoint() && anU2 >= kTwoPi - theTol.equalPoint();
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::setRadius (double theRadius)
{
  CheckRadius ("setRadius", theRadius);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const occ::handle<Geom_CylindricalSurface> aBasis = CylinderOf (mpImpEnt);
  const occ::handle<Geom_CylindricalSurface> aNew (new Geom_CylindricalSurface (
    aBasis->Position(), theRadius));
  SetCylinder (mpImpEnt, PatchOf (aNew, anU1, anU2, aV1, aV2));
  return *this;
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::setAngles (double theStart, double theEnd)
{
  CheckAngleRange ("setAngles", theStart, theEnd);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, theStart, theEnd, aV1, aV2);
  return *this;
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::setHeight (const NvGeInterval& theHeight)
{
  CheckHeight ("setHeight", theHeight);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, anU1, anU2, theHeight.lowerBound(), theHeight.upperBound());
  return *this;
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::set (double theRadius, const NvGePoint3d& theOrigin,
                                 const NvGeVector3d& theAxisOfSym)
{
  *this = NvGeCylinder (theRadius, theOrigin, theAxisOfSym);
  return *this;
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::set (double theRadius, const NvGePoint3d& theOrigin,
                                 const NvGeVector3d& theAxisOfSym,
                                 const NvGeVector3d& theRefAxis,
                                 const NvGeInterval& theHeight,
                                 double theStartAngle, double theEndAngle)
{
  *this = NvGeCylinder (theRadius, theOrigin, theAxisOfSym, theRefAxis,
                        theHeight, theStartAngle, theEndAngle);
  return *this;
}

//=================================================================================================

NvGeCylinder& NvGeCylinder::operator = (const NvGeCylinder& theSrc)
{
  NvGeSurface::operator= (theSrc);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeCylinder::intersectWith (const NvGeLinearEnt3d& theLinEnt, int& theIntn,
                                            NvGePoint3d& thePnt1, NvGePoint3d& thePnt2,
                                            const NvGeTol& theTol) const
{
  (void)theLinEnt;
  (void)thePnt1;
  (void)thePnt2;
  (void)theTol;
  theIntn = 0; // line/cylinder intersection is not provided yet
  return false;
}
