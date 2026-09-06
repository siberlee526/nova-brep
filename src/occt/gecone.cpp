// gecone.cpp - implementation of NvGeCone.
//
// The cone stores a Geom_ConicalSurface in its impl. OCCT parameterizes the
// cone as P(u,v) = O + (R + v*sin(Ang))*(cos(u)*X + sin(u)*Y) + v*cos(Ang)*N
// and places the apex on the negative side of the main direction when the
// semi-angle Ang is positive, while the ARX cone is given by its base
// circle and an axis of symmetry pointing from the base toward the apex.
// The stored semi-angle is therefore the NEGATED ARX half angle: the base
// plane maps onto the reference plane (v = 0), the main direction equals
// the ARX axis of symmetry, and the radius shrinks toward the apex, which
// lands at baseCenter + (baseRadius/tan(halfAngle))*axisOfSymmetry.
// A restricted angle range and/or height interval is represented by a
// Geom_RectangularTrimmedSurface over the analytic cone; since the OCCT v
// parameter measures slant distance, height intervals convert by cos(half).

#include <gecone.h>

#include <Nova.h>
#include <NvException.h>
#include <geimpdata.h>
#include <geintrvl.h>
#include <gelent3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_ConicalSurface.hxx>
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
constexpr double kHalfPi = 1.57079632679489661923;

//! Huge-bound threshold of NvGeSurface::getEnvelope; OCCT stores unbounded
//! domains as huge values, which are normalized to infinities.
bool IsHuge (double theValue)
{
  return std::abs (theValue) >= 1e100;
}

//! Installs cone geometry, replacing the base placeholder impl.
void MakeCone (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  theImp = new NvGeImpEntity3d (NvGe::kCone, theGeom);
  theImp->Ref();
}

//! Replaces the geometry with copy-on-write discipline.
void SetCone (NvGeImpEntity3d*& theImp, const occ::handle<Geom_Surface>& theGeom)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theGeom);
}

//! Analytic cone of this entity, unwrapping a trimmed patch.
occ::handle<Geom_ConicalSurface> ConeOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_ConicalSurface> aCone = occ::down_cast<Geom_ConicalSurface> (theImp->Geom());
  if (aCone.IsNull())
  {
    const occ::handle<Geom_RectangularTrimmedSurface> aPatch =
      occ::down_cast<Geom_RectangularTrimmedSurface> (theImp->Geom());
    if (!aPatch.IsNull())
    {
      aCone = occ::down_cast<Geom_ConicalSurface> (aPatch->BasisSurface());
    }
  }
  if (aCone.IsNull())
  {
    throw NvException ("NvGeCone: the entity holds no cone geometry");
  }
  return aCone;
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
occ::handle<Geom_Surface> PatchOf (const occ::handle<Geom_ConicalSurface>& theBasis,
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

//! Guards a base radius: rejects negative and non-finite values.
void CheckRadius (const char* theMethod, double theRadius)
{
  if (!(theRadius >= 0.0))
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the base radius must be non-negative");
  }
}

//! Guards the (cosine, sine) pair of the half angle and returns the angle
//! itself; a cone requires a half angle strictly inside ]0, PI/2[.
double CheckHalfAngle (const char* theMethod, double theCosineAngle, double theSineAngle)
{
  if (std::abs (theCosineAngle) > 1.0 || std::abs (theSineAngle) > 1.0)
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the half angle cosine/sine is out of range");
  }
  const double aHalfAngle = std::atan2 (theSineAngle, theCosineAngle);
  if (aHalfAngle <= gp::Resolution() || kHalfPi - aHalfAngle <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the half angle must lie inside ]0, PI/2[");
  }
  return aHalfAngle;
}

//! Guards an angle interval against the OCCT trim constraints.
void CheckAngleRange (const char* theMethod, double theStart, double theEnd)
{
  if (!(theStart < theEnd))
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the start angle must be less than the end angle");
  }
}

//! Guards a height interval: both bounds must be finite and ordered.
void CheckHeight (const char* theMethod, const NvGeInterval& theHeight)
{
  if (!theHeight.isBounded())
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the height interval must be bounded");
  }
  if (!(theHeight.lowerBound() < theHeight.upperBound()))
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the height interval must not be empty");
  }
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

//! Builds the positioned analytic cone; the ARX half angle is stored
//! negated as the OCCT semi-angle (see the file banner).
occ::handle<Geom_ConicalSurface> BasisOf (double theHalfAngle, const NvGePoint3d& theBaseCenter,
                                          double theBaseRadius,
                                          const NvGeVector3d& theAxisOfSymmetry,
                                          const NvGeVector3d& theRefAxis,
                                          const char* theMethod)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, theMethod);
  const gp_Dir aRef = DirOrThrow (theRefAxis.x, theRefAxis.y, theRefAxis.z, theMethod);
  if (1.0 - std::abs (anAxis.Dot (aRef)) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCone::") + theMethod
                       + "(): the axis of symmetry and the reference axis must not be parallel");
  }
  return occ::handle<Geom_ConicalSurface> (new Geom_ConicalSurface (
    gp_Ax3 (gp_Pnt (theBaseCenter.x, theBaseCenter.y, theBaseCenter.z), anAxis, aRef),
    -theHalfAngle, theBaseRadius));
}

//! Builds the analytic cone for a definition without reference axis; the
//! frame X direction is then seeded from the axis (gp_Ax3 two direction
//! convention), which never raises for a valid direction.
occ::handle<Geom_ConicalSurface> BasisOfAxis (double theHalfAngle, const NvGePoint3d& theBaseCenter,
                                              double theBaseRadius,
                                              const NvGeVector3d& theAxisOfSymmetry)
{
  const gp_Dir anAxis = DirOrThrow (theAxisOfSymmetry.x, theAxisOfSymmetry.y,
                                    theAxisOfSymmetry.z, "NvGeCone");
  return occ::handle<Geom_ConicalSurface> (new Geom_ConicalSurface (
    gp_Ax3 (gp_Pnt (theBaseCenter.x, theBaseCenter.y, theBaseCenter.z), anAxis),
    -theHalfAngle, theBaseRadius));
}

//! Replaces the stored geometry with a patch of the current basis.
void SetPatch (NvGeImpEntity3d*& theImp, double theU1, double theU2, double theV1, double theV2)
{
  SetCone (theImp, PatchOf (ConeOf (theImp), theU1, theU2, theV1, theV2));
}

} // namespace

//=================================================================================================

NvGeCone::NvGeCone()
{
  // Default: quarter cone (half angle PI/4) with unit base circle at the
  // world origin along +Z; the apex is then at (0, 0, 1).
  MakeCone (mpImpEnt, PatchOf (
    BasisOfAxis (kHalfPi * 0.5, NvGePoint3d (0.0, 0.0, 0.0), 1.0,
                 NvGeVector3d (0.0, 0.0, 1.0)),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeCone::NvGeCone (const NvGeCone& theSrc)
: NvGeSurface (theSrc)
{
}

//=================================================================================================

NvGeCone::NvGeCone (double theCosineAngle, double theSineAngle,
                    const NvGePoint3d& theBaseOrigin, double theBaseRadius,
                    const NvGeVector3d& theAxisOfSymmetry)
{
  const double aHalfAngle = CheckHalfAngle ("NvGeCone", theCosineAngle, theSineAngle);
  CheckRadius ("NvGeCone", theBaseRadius);
  MakeCone (mpImpEnt, PatchOf (
    BasisOfAxis (aHalfAngle, theBaseOrigin, theBaseRadius, theAxisOfSymmetry),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

//=================================================================================================

NvGeCone::NvGeCone (double theCosineAngle, double theSineAngle,
                    const NvGePoint3d& theBaseOrigin, double theBaseRadius,
                    const NvGeVector3d& theAxisOfSymmetry,
                    const NvGeVector3d& theRefAxis, const NvGeInterval& theHeight,
                    double theStartAngle, double theEndAngle)
{
  const double aHalfAngle = CheckHalfAngle ("NvGeCone", theCosineAngle, theSineAngle);
  CheckRadius ("NvGeCone", theBaseRadius);
  CheckAngleRange ("NvGeCone", theStartAngle, theEndAngle);
  CheckHeight ("NvGeCone", theHeight);
  // The OCCT v parameter measures slant distance: convert the axial height
  // bounds by cos(half angle).
  const double aCosHalf = std::cos (aHalfAngle);
  MakeCone (mpImpEnt, PatchOf (
    BasisOf (aHalfAngle, theBaseOrigin, theBaseRadius, theAxisOfSymmetry, theRefAxis,
             "NvGeCone"),
    theStartAngle, theEndAngle,
    theHeight.lowerBound() / aCosHalf, theHeight.upperBound() / aCosHalf));
}

//=================================================================================================

double NvGeCone::baseRadius() const
{
  return ConeOf (mpImpEnt)->RefRadius();
}

//=================================================================================================

NvGePoint3d NvGeCone::baseCenter() const
{
  const gp_Pnt aCenter = ConeOf (mpImpEnt)->Location();
  return NvGePoint3d (aCenter.X(), aCenter.Y(), aCenter.Z());
}

//=================================================================================================

void NvGeCone::getAngles (double& theStart, double& theEnd) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  theStart = anU1;
  theEnd = anU2;
}

//=================================================================================================

double NvGeCone::halfAngle() const
{
  // The ARX half angle is stored as the negated OCCT semi-angle.
  return -ConeOf (mpImpEnt)->SemiAngle();
}

//=================================================================================================

void NvGeCone::getHalfAngle (double& theCosineAngle, double& theSineAngle) const
{
  const double aHalfAngle = halfAngle();
  theCosineAngle = std::cos (aHalfAngle);
  theSineAngle = std::sin (aHalfAngle);
}

//=================================================================================================

void NvGeCone::getHeight (NvGeInterval& theRange) const
{
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const double anInf = std::numeric_limits<double>::infinity();
  if (aV1 > -anInf && aV2 < anInf)
  {
    // Axial height = slant parameter * cos(half angle).
    const double aCosHalf = std::cos (halfAngle());
    theRange.set (aV1 * aCosHalf, aV2 * aCosHalf);
  }
  else
  {
    theRange.set(); // unbounded height
  }
}

//=================================================================================================

double NvGeCone::heightAt (double theU) const
{
  // The ARX natural parameterization measures v as the signed axial height
  // from the base plane, so the parameter-to-height map is the identity.
  return theU;
}

//=================================================================================================

NvGeVector3d NvGeCone::axisOfSymmetry() const
{
  const gp_Dir anAxis = ConeOf (mpImpEnt)->Position().Direction();
  return NvGeVector3d (anAxis.X(), anAxis.Y(), anAxis.Z());
}

//=================================================================================================

NvGeVector3d NvGeCone::refAxis() const
{
  const gp_Dir aRef = ConeOf (mpImpEnt)->Position().XDirection();
  return NvGeVector3d (aRef.X(), aRef.Y(), aRef.Z());
}

//=================================================================================================

NvGePoint3d NvGeCone::apex() const
{
  const gp_Pnt anApex = ConeOf (mpImpEnt)->Apex();
  return NvGePoint3d (anApex.X(), anApex.Y(), anApex.Z());
}

//=================================================================================================

Adesk::Boolean NvGeCone::isClosed (const NvGeTol& theTol) const
{
  // Closed means a full revolution in u; a bounded height still leaves a
  // closed band.
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  return anU1 <= theTol.equalPoint() && anU2 >= kTwoPi - theTol.equalPoint();
}

//=================================================================================================

Adesk::Boolean NvGeCone::isOuterNormal() const
{
  // OCCT builds elementary surfaces with the normal oriented towards the
  // outside region of the cone.
  return true;
}

//=================================================================================================

NvGeCone& NvGeCone::setBaseRadius (double theRadius)
{
  CheckRadius ("setBaseRadius", theRadius);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const occ::handle<Geom_ConicalSurface> aBasis = ConeOf (mpImpEnt);
  const occ::handle<Geom_ConicalSurface> aNew (new Geom_ConicalSurface (
    aBasis->Position(), aBasis->SemiAngle(), theRadius));
  SetCone (mpImpEnt, PatchOf (aNew, anU1, anU2, aV1, aV2));
  return *this;
}

//=================================================================================================

NvGeCone& NvGeCone::setAngles (double theStartAngle, double theEndAngle)
{
  CheckAngleRange ("setAngles", theStartAngle, theEndAngle);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  SetPatch (mpImpEnt, theStartAngle, theEndAngle, aV1, aV2);
  return *this;
}

//=================================================================================================

NvGeCone& NvGeCone::setHeight (const NvGeInterval& theHeight)
{
  CheckHeight ("setHeight", theHeight);
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  StoredBounds (mpImpEnt, anU1, anU2, aV1, aV2);
  const double aCosHalf = std::cos (halfAngle());
  SetPatch (mpImpEnt, anU1, anU2,
            theHeight.lowerBound() / aCosHalf, theHeight.upperBound() / aCosHalf);
  return *this;
}

//=================================================================================================

NvGeCone& NvGeCone::set (double theCosineAngle, double theSineAngle,
                         const NvGePoint3d& theBaseCenter, double theBaseRadius,
                         const NvGeVector3d& theAxisOfSymmetry)
{
  *this = NvGeCone (theCosineAngle, theSineAngle, theBaseCenter, theBaseRadius,
                    theAxisOfSymmetry);
  return *this;
}

//=================================================================================================

NvGeCone& NvGeCone::set (double theCosineAngle, double theSineAngle,
                         const NvGePoint3d& theBaseCenter, double theBaseRadius,
                         const NvGeVector3d& theAxisOfSymmetry,
                         const NvGeVector3d& theRefAxis, const NvGeInterval& theHeight,
                         double theStartAngle, double theEndAngle)
{
  *this = NvGeCone (theCosineAngle, theSineAngle, theBaseCenter, theBaseRadius,
                    theAxisOfSymmetry, theRefAxis, theHeight, theStartAngle, theEndAngle);
  return *this;
}

//=================================================================================================

NvGeCone& NvGeCone::operator = (const NvGeCone& theSrc)
{
  NvGeSurface::operator= (theSrc);
  return *this;
}

//=================================================================================================

Adesk::Boolean NvGeCone::intersectWith (const NvGeLinearEnt3d& theLinEnt, int& theIntn,
                                        NvGePoint3d& thePnt1, NvGePoint3d& thePnt2,
                                        const NvGeTol& theTol) const
{
  (void)theLinEnt;
  (void)thePnt1;
  (void)thePnt2;
  (void)theTol;
  theIntn = 0; // line/cone intersection is not provided yet
  return false;
}
