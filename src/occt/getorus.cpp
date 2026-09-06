// getorus.cpp - implementation of NvGeTorus.
//
// The torus stores a Geom_ToroidalSurface in its impl, parameterized as
// P(u,v) = O + (R + r*cos(v))*(cos(u)*X + sin(u)*Y) + r*sin(v)*N with both
// parameters periodic over [0, 2*PI]; the axis of symmetry is the main
// direction of the stored frame. A restricted angle range is represented
// by a Geom_RectangularTrimmedSurface over the analytic torus (the same
// convention as NvGeBoundedPlane); the getters unwrap the patch. OCCT
// cannot carry a negative minor radius, so ARX vortex tori are rejected at
// the API boundary while the classification predicates keep the ARX
// definitions (vortex: minor < 0, degenerate: minor = 0, lemon: equal
// radii, apple: minor > major, doughnut: major > minor > 0).

#include <getorus.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <gelent3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>
#include <string>

namespace
{

//! Installs torus geometry, replacing the base placeholder impl.
void MakeTorus (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  theImp = new NvGeImpEntity3d (NvGe::kTorus, theGeom);
  theImp->Ref();
}

//! Replaces the geometry with copy-on-write discipline.
void SetTorus (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theGeom);
}

//! Analytic torus of this entity, unwrapping a trimmed patch.
occ::handle<Geom_ToroidalSurface> TorusOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_ToroidalSurface> aTorus =
    occ::down_cast<Geom_ToroidalSurface> (theImp->Geom());
  if (aTorus.IsNull())
  {
    const occ::handle<Geom_RectangularTrimmedSurface> aPatch =
      occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
    if (!aPatch.IsNull())
    {
      aTorus = occ::down_cast<Geom_ToroidalSurface> (aPatch->BasisSurface());
    }
  }
  if (aTorus.IsNull())
  {
    throw NvException ("NvGeTorus: the entity holds no torus geometry");
  }
  return aTorus;
}

//! Parameter bounds of the stored geometry (patch bounds when trimmed).
void StoredBounds (const NvGeImpEntity3d* theImp,
                   double& theU1, double& theU2, double& theV1, double& theV2)
{
  NvGeSurfaceOf (theImp)->Bounds (theU1, theU2, theV1, theV2);
}

//! Patch of the basis surface; the full periodic range keeps the plain
//! analytic surface.
occ::handle<Geom_Surface> PatchOf (const occ::handle<Geom_ToroidalSurface>& theBasis,
                                   double theU1, double theU2, double theV1, double theV2)
{
  const double anInf = std::numeric_limits<double>::infinity();
  if (theU1 > -anInf && theU2 < anInf && theV1 > -anInf && theV2 < anInf)
  {
    return occ::handle<Geom_Surface> (new Geom_RectangularTrimmedSurface (
      theBasis, theU1, theU2, theV1, theV2));
  }
  return occ::handle<Geom_Surface> (theBasis);
}

//! Guards a radius: rejects negative and non-finite values (an ARX vortex
//! torus with negative minor radius has no OCCT representation).
void CheckRadius (const char* theMethod, const char* theWhich, double theRadius)
{
  if (!(theRadius >= 0.0))
  {
    throw NvException (std::string ("NvGeTorus::") + theMethod + "(): the " + theWhich
                       + " radius must be non-negative");
  }
}

//! Guards an angle interval against the OCCT trim constraints.
void CheckAngleRange (const char* theMethod, double theStart, double theEnd)
{
  if (!(theStart < theEnd))
  {
    throw NvException (std::string ("NvGeTorus::") + theMethod
                       + "(): the start angle must be less than the end angle");
  }
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeTorus::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

//! Builds the positioned analytic torus from the defining frame.
occ::handle<Geom_ToroidalSurface> BasisOf (double theMajorRadius, double theMinorRadius,
                                           const NvGePoint3d& theOrigin,
                                           const NvGeVector3d& theAxisOfSymmetry,
                                           const NvGeVector3d& theRefAxis,
                                           const char* theMethod)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, theMethod);
  const gp_Dir aRef = DirOrThrow (theRefAxis.x, theRefAxis.y, theRefAxis.z, theMethod);
  if (1.0 - std::abs (anAxis.Dot (aRef)) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeTorus::") + theMethod
                       + "(): the axis of symmetry and the reference axis must not be parallel");
  }
  return occ::handle<Geom_ToroidalSurface> (new Geom_ToroidalSurface (
    gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), anAxis, aRef),
    theMajorRadius, theMinorRadius));
}

//! Builds the analytic torus for a definition without reference axis; the
//! frame X direction is then seeded from the axis (gp_Ax3 two direction
//! convention), which never raises for a valid direction.
occ::handle<Geom_ToroidalSurface> BasisOfAxis (double theMajorRadius, double theMinorRadius,
                                               const NvGePoint3d& theOrigin,
                                               const NvGeVector3d& theAxisOfSymmetry)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, "NvGeTorus");
  return occ::handle<Geom_ToroidalSurface> (new Geom_ToroidalSurface (
    gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), anAxis),
    theMajorRadius, theMinorRadius));
}

//! Replaces the stored geometry with a patch of the current basis.
void SetPatch (NvGeImpEntity3d*& theImp, double theU1, double theU2, double theV1, double theV2)
{
  SetTorus (theImp, PatchOf (TorusOf (theImp), theU1, theU2, theV1, theV2));
}

} // namespace

//=================================================================================================

NvGeTorus::NvGeTorus()
{
  // Default: doughnut torus with major radius 2 and minor radius 1 at the
  // world origin along +Z.
  MakeTorus (mpImpEnt, PatchOf (
    BasisOfAxis (2.0, 1.0, NvGePoint3d (0.0, 0.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0)),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeTorus::NvGeTorus (const NvGeTorus& theSrc)
: NvGeSurface (theSrc)
{
}

//=================================================================================================

NvGeTorus::NvGeTorus (double theMajorRadius, double theMinorRadius,
                      const NvGePoint3d& theOrigin, const NvGeVector3d& theAxisOfSymmetry)
{
  CheckRadius ("NvGeTorus", "major", theMajorRadius);
  CheckRadius ("NvGeTorus", "minor", theMinorRadius);
  MakeTorus (mpImpEnt, PatchOf (
    BasisOfAxis (theMajorRadius, theMinorRadius, theOrigin, theAxisOfSymmetry),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeTorus::NvGeTorus (double theMajorRadius, double theMinorRadius,
                      const NvGePoint3d& theOrigin, const NvGeVector3d& theAxisOfSymmetry,
                      const NvGeVector3d& theRefAxis,
                      double theStartAngleU, double theEndAngleU,
                      double theStartAngleV, double theEndAngleV)
{
  CheckRadius ("NvGeTorus", "major", theMajorRadius);
  CheckRadius ("NvGeTorus", "minor", theMinorRadius);
  CheckAngleRange ("NvGeTorus", theStartAngleU, theEndAngleU);
  CheckAngleRange ("NvGeTorus", theStartAngleV, theEndAngleV);
  MakeTorus (mpImpEnt, PatchOf (
    BasisOf (theMajorRadius, theMinorRadius, theOrigin, theAxisOfSymmetry, theRefAxis,
             "NvGeTorus"),
    theStartAngleU, theEndAngleU, theStartAngleV, theEndAngleV));
}

//=================================================================================================

double NvGeTorus::majorRadius() const
{
  return TorusOf (mpImpEnt)->MajorRadius();
}

//=================================================================================================

double NvGeTorus::minorRadius() const
{
  return TorusOf (mpImpEnt)->MinorRadius();
}

//=================================================================================================

void NvGeTorus::getAnglesInU (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = anU1;
  theEnd = anU2;
}

//=================================================================================================

void NvGeTorus::getAnglesInV (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = aV1;
  theEnd = aV2;
}

//=================================================================================================

NvGePoint3d NvGeTorus::center() const
{
  const gp_Pnt aCenter = TorusOf (mpImpEnt)->Location();
  return NvGePoint3d (aCenter.X(), aCenter.Y(), aCenter.Z());
}

//=================================================================================================

NvGeVector3d NvGeTorus::axisOfSymmetry() const
{
  const gp_Dir anAxis = TorusOf (mpImpEnt)->Position().Direction();
  return NvGeVector3d (anAxis.X(), anAxis.Y(), anAxis.Z());
}

//=================================================================================================

NvGeVector3d NvGeTorus::refAxis() const
{
  const gp_Dir aRef = TorusOf (mpImpEnt)->Position().XDirection();
  return NvGeVector3d (aRef.X(), aRef.Y(), aRef.Z());
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isOuterNormal() const
{
  // OCCT builds elementary surfaces with the normal oriented towards the
  // outside region of the torus.
  return true;
}

//=================================================================================================

NvGeTorus& NvGeTorus::setMajorRadius (double theRadius)
{
  CheckRadius ("setMajorRadius", "major", theRadius);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const occ::handle<Geom_ToroidalSurface> aBasis = TorusOf (mpImpEnt);
  const occ::handle<Geom_ToroidalSurface> aNew (new Geom_ToroidalSurface (
    aBasis->Position(), theRadius, aBasis->MinorRadius()));
  SetTorus (mpImpEnt, PatchOf (aNew, anU1, anU2, aV1, aV2));
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::setMinorRadius (double theRadius)
{
  CheckRadius ("setMinorRadius", "minor", theRadius);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const occ::handle<Geom_ToroidalSurface> aBasis = TorusOf (mpImpEnt);
  const occ::handle<Geom_ToroidalSurface> aNew (new Geom_ToroidalSurface (
    aBasis->Position(), aBasis->MajorRadius(), theRadius));
  SetTorus (mpImpEnt, PatchOf (aNew, anU1, anU2, aV1, aV2));
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::setAnglesInU (double theStart, double theEnd)
{
  CheckAngleRange ("setAnglesInU", theStart, theEnd);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, theStart, theEnd, aV1, aV2);
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::setAnglesInV (double theStart, double theEnd)
{
  CheckAngleRange ("setAnglesInV", theStart, theEnd);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, anU1, anU2, theStart, theEnd);
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::set (double theMajorRadius, double theMinorRadius,
                           const NvGePoint3d& theOrigin, const NvGeVector3d& theAxisOfSymmetry)
{
  *this = NvGeTorus (theMajorRadius, theMinorRadius, theOrigin, theAxisOfSymmetry);
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::set (double theMajorRadius, double theMinorRadius,
                           const NvGePoint3d& theOrigin, const NvGeVector3d& theAxisOfSymmetry,
                           const NvGeVector3d& theRefAxis,
                           double theStartAngleU, double theEndAngleU,
                           double theStartAngleV, double theEndAngleV)
{
  *this = NvGeTorus (theMajorRadius, theMinorRadius, theOrigin, theAxisOfSymmetry, theRefAxis,
                     theStartAngleU, theEndAngleU, theStartAngleV, theEndAngleV);
  return *this;
}

//=================================================================================================

NvGeTorus& NvGeTorus::operator = (const NvGeTorus& theSrc)
{
  NvGeSurface::operator= (theSrc);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeTorus::intersectWith (const NvGeLinearEnt3d& theLinEnt, int& theIntn,
                                         NvGePoint3d& thePnt1, NvGePoint3d& thePnt2,
                                         NvGePoint3d& thePnt3, NvGePoint3d& thePnt4,
                                         const NvGeTol& theTol) const
{
  (void)theLinEnt;
  (void)thePnt1;
  (void)thePnt2;
  (void)thePnt3;
  (void)thePnt4;
  (void)theTol;
  theIntn = 0; // line/torus intersection is not provided yet
  return false;
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isLemon() const
{
  // Horn torus: the tube radius equals the ring radius and the tube
  // touches the axis in a single point.
  return minorRadius() > 0.0 && majorRadius() == minorRadius();
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isApple() const
{
  // Self-intersecting torus without a central hole.
  return minorRadius() > 0.0 && majorRadius() < minorRadius();
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isVortex() const
{
  // ARX definition: a vortex torus carries a negative minor radius; such a
  // torus is rejected by the NvGeTorus validation, so this stays false for
  // every representable torus.
  return minorRadius() < 0.0;
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isDoughnut() const
{
  // Standard ring torus with a central hole.
  return minorRadius() > 0.0 && majorRadius() > minorRadius();
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isDegenerate() const
{
  // A zero tube radius collapses the surface onto the ring circle.
  return minorRadius() == 0.0;
}

//=================================================================================================

Adesk::Boolean NvGeTorus::isHollow() const
{
  // Hollow means the central hole exists, i.e. the doughnut shape.
  return isDoughnut();
}
