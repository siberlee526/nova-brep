// geplane.cpp - implementation of NvGePlane.
//
// The plane stores a Geom_Plane in its impl. The local frame is built
// right-handed: u x v = normal; for a plane defined only by a normal the u
// axis is seeded from the least-aligned coordinate axis (the same strategy
// as BasisFromNormal in gemat3d.cpp).

#include <geplane.h>

#include <Nova.h>
#include <NvException.h>
#include <gebndpln.h>
#include <geimpdata.h>
#include <gelent3d.h>
#include <geline3d.h>
#include <gelnsg3d.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>

#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <limits>
#include <string>

namespace
{

//! Installs plane geometry, replacing the base placeholder impl.
void MakePlane (NvGeImpEntity3d*& theImp, const gp_Ax3& thePosition)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  theImp = new NvGeImpEntity3d (NvGe::kPlane,
      occ::handle<Geom_Surface> (new Geom_Plane (thePosition)));
  theImp->Ref();
}

//! Replaces the geometry with copy-on-write discipline.
void SetPlane (NvGeImpEntity3d*& theImp, const gp_Ax3& thePosition)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (occ::handle<Geom_Surface> (new Geom_Plane (thePosition)));
}

//! Builds an orthonormal (u, v) pair for a unit normal; u is seeded from
//! the coordinate axis least aligned with the normal.
void BasisForNormal (const gp_Dir& theNormal, gp_Dir& theU, gp_Dir& theV)
{
  const double aNx = theNormal.X();
  const double aNy = theNormal.Y();
  const double aNz = theNormal.Z();
  double anSx = 0.0, anSy = 0.0, anSz = 0.0;
  if (std::abs (aNx) <= std::abs (aNy) && std::abs (aNx) <= std::abs (aNz))
  {
    anSy = 1.0;
  }
  else if (std::abs (aNy) <= std::abs (aNz))
  {
    anSz = 1.0;
  }
  else
  {
    anSx = 1.0;
  }
  // u = seed x n (unit, since seed and n are orthogonal unit vectors).
  const double anUx = anSy * aNz - anSz * aNy;
  const double anUy = anSz * aNx - anSx * aNz;
  const double anUz = anSx * aNy - anSy * aNx;
  theU = gp_Dir (anUx, anUy, anUz);
  // v = n x u completes the right-handed frame.
  theV = gp_Dir (aNy * anUz - aNz * anUy,
                 aNz * anUx - aNx * anUz,
                 aNx * anUy - aNy * anUx);
}

//! Direction of a vector with a zero check that never lets gp_Dir raise.
gp_Dir DirOrThrow (double theX, double theY, double theZ, const char* theMethod)
{
  const double aLen = std::sqrt (theX * theX + theY * theY + theZ * theZ);
  if (aLen <= gp::Resolution())
  {
    throw NvException (std::string ("NvGePlane::") + theMethod
                       + "(): the direction vector is degenerate");
  }
  return gp_Dir (theX, theY, theZ);
}

//! Plane geometry of this entity.
occ::handle<Geom_Plane> PlaneOf (const NvGeImpEntity3d* theImp)
{
  occ::handle<Geom_Plane> aPlane = occ::down_cast<Geom_Plane> (theImp->Geom());
  if (aPlane.IsNull())
  {
    throw NvException ("NvGePlane: the entity holds no plane geometry");
  }
  return aPlane;
}

//! Impl pointer of an entity wrapper (mpImpEnt is the first member, pack 8).
NvGeImpEntity3d* ImplOf (const NvGeEntity3d* theWrapper)
{
  return *reinterpret_cast<NvGeImpEntity3d* const*> (theWrapper);
}

} // namespace

//=================================================================================================

NvGePlane::NvGePlane()
{
  // Default: the world XY plane.
  MakePlane (mpImpEnt, gp_Ax3 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0), gp_Dir (1.0, 0.0, 0.0)));
}

//=================================================================================================

NvGePlane::NvGePlane (const NvGePlane& theSrc)
: NvGePlanarEnt (theSrc)
{
}

// The default constructor builds the XY plane, so the constant is simply a
// default-constructed instance.
const NvGePlane NvGePlane::kXYPlane;

// NOTE: static initializers must not read other translation units' statics
// (initialization order is undefined); use self-contained literals and the
// frame-based constructor (no normal-basis derivation) to keep the static
// initialization minimal.
const NvGePlane NvGePlane::kYZPlane (NvGePoint3d (0.0, 0.0, 0.0),
                                     NvGeVector3d (0.0, 1.0, 0.0), NvGeVector3d (0.0, 0.0, 1.0));

const NvGePlane NvGePlane::kZXPlane (NvGePoint3d (0.0, 0.0, 0.0),
                                     NvGeVector3d (0.0, 0.0, 1.0), NvGeVector3d (1.0, 0.0, 0.0));

//=================================================================================================

NvGePlane::NvGePlane (const NvGePoint3d& theOrigin, const NvGeVector3d& theNormal)
{
  const gp_Dir aNormal = DirOrThrow (theNormal.x, theNormal.y, theNormal.z, "NvGePlane");
  gp_Dir anU, aV;
  BasisForNormal (aNormal, anU, aV);
  MakePlane (mpImpEnt, gp_Ax3 (gp_Pnt (theOrigin.x, theOrigin.y, theOrigin.z), aNormal, anU));
}

//=================================================================================================

NvGePlane::NvGePlane (const NvGePoint3d& thePntU, const NvGePoint3d& theOrg, const NvGePoint3d& thePntV)
{
  // Frame axes from the defining points; the plane normal completes them.
  const gp_Dir anU = DirOrThrow (thePntU.x - theOrg.x, thePntU.y - theOrg.y,
                                 thePntU.z - theOrg.z, "NvGePlane");
  const gp_Dir aV = DirOrThrow (thePntV.x - theOrg.x, thePntV.y - theOrg.y,
                                thePntV.z - theOrg.z, "NvGePlane");
  const gp_Dir aNormal (anU.Crossed (aV));
  MakePlane (mpImpEnt, gp_Ax3 (gp_Pnt (theOrg.x, theOrg.y, theOrg.z), aNormal, anU));
}

//=================================================================================================

NvGePlane::NvGePlane (const NvGePoint3d& theOrg, const NvGeVector3d& theUAxis,
                      const NvGeVector3d& theVAxis)
{
  const gp_Dir anU = DirOrThrow (theUAxis.x, theUAxis.y, theUAxis.z, "NvGePlane");
  const gp_Dir aV = DirOrThrow (theVAxis.x, theVAxis.y, theVAxis.z, "NvGePlane");
  const gp_Dir aNormal (anU.Crossed (aV));
  MakePlane (mpImpEnt, gp_Ax3 (gp_Pnt (theOrg.x, theOrg.y, theOrg.z), aNormal, anU));
}

//=================================================================================================

NvGePlane::NvGePlane (double theA, double theB, double theC, double theD)
{
  // Coefficients a*x + b*y + c*z + d = 0; the point closest to the origin
  // serves as the plane origin.
  const double aLen = std::sqrt (theA * theA + theB * theB + theC * theC);
  if (aLen <= gp::Resolution())
  {
    throw NvException ("NvGePlane::NvGePlane(): the normal coefficients are degenerate");
  }
  const gp_Dir aNormal (theA / aLen, theB / aLen, theC / aLen);
  const double aDist = theD / aLen; // normalized equation: n.x + aDist = 0
  const gp_Pnt aLocation (-aDist * aNormal.X(), -aDist * aNormal.Y(), -aDist * aNormal.Z());
  gp_Dir anU, aV;
  BasisForNormal (aNormal, anU, aV);
  MakePlane (mpImpEnt, gp_Ax3 (aLocation, aNormal, anU));
}

//=================================================================================================

double NvGePlane::signedDistanceTo (const NvGePoint3d& thePnt) const
{
  // Positive on the normal side: n . (p - o).
  const gp_Pln aPln = PlaneOf (mpImpEnt)->Pln();
  const gp_Dir aNormal = aPln.Axis().Direction();
  const gp_Pnt anOrigin = aPln.Location();
  return aNormal.X() * (thePnt.x - anOrigin.X())
       + aNormal.Y() * (thePnt.y - anOrigin.Y())
       + aNormal.Z() * (thePnt.z - anOrigin.Z());
}

//=================================================================================================

Adesk::Boolean NvGePlane::intersectWith (const NvGeLinearEnt3d& theLinEnt, NvGePoint3d& theResultPnt,
                                         const NvGeTol& theTol) const
{
  return NvGePlanarEnt::intersectWith (theLinEnt, theResultPnt, theTol);
}

//=================================================================================================

Adesk::Boolean NvGePlane::intersectWith (const NvGePlane& theOtherPln, NvGeLine3d& theResultLine,
                                         const NvGeTol& theTol) const
{
  const gp_Pln aPln1 = PlaneOf (mpImpEnt)->Pln();
  const gp_Pln aPln2 = PlaneOf (theOtherPln.mpImpEnt)->Pln();
  const gp_Dir aN1 = aPln1.Axis().Direction();
  const gp_Dir aN2 = aPln2.Axis().Direction();
  const gp_XYZ aCross = aN1.XYZ().Crossed (aN2.XYZ());
  if (aCross.Modulus() <= theTol.equalVector())
  {
    return false; // parallel planes
  }
  // A point on the intersection line from the normalized plane equations.
  double aA1 = 0.0, aB1 = 0.0, aC1 = 0.0, aD1 = 0.0;
  double aA2 = 0.0, aB2 = 0.0, aC2 = 0.0, aD2 = 0.0;
  aPln1.Coefficients (aA1, aB1, aC1, aD1);
  aPln2.Coefficients (aA2, aB2, aC2, aD2);
  const gp_XYZ aUnitCross = aCross.Normalized();
  const gp_XYZ aN2xC = aN2.XYZ().Crossed (aUnitCross);
  const gp_XYZ aCxN1 = aUnitCross.Crossed (aN1.XYZ());
  const double aK1 = -aD1;
  const double aK2 = -aD2;
  const double aDenom = aUnitCross.Dot (aUnitCross);
  const NvGePoint3d anOrigin ((aK1 * aN2xC.X() + aK2 * aCxN1.X()) / aDenom,
                              (aK1 * aN2xC.Y() + aK2 * aCxN1.Y()) / aDenom,
                              (aK1 * aN2xC.Z() + aK2 * aCxN1.Z()) / aDenom);
  theResultLine.set (anOrigin, NvGeVector3d (aUnitCross.X(), aUnitCross.Y(), aUnitCross.Z()));
  return true;
}

//=================================================================================================

Adesk::Boolean NvGePlane::intersectWith (const NvGeBoundedPlane& theBndPln,
                                         NvGeLineSeg3d& theResultLineSeg,
                                         const NvGeTol& theTol) const
{
  // Intersect with the bounded plane's carrier, then clip the line to the
  // bounded plane's parameter rectangle (stored in its trimmed surface).
  const occ::handle<Geom_RectangularTrimmedSurface> aTrimmed =
    occ::down_cast<Geom_RectangularTrimmedSurface> (ImplOf (&theBndPln)->Geom());
  if (aTrimmed.IsNull())
  {
    throw NvException ("NvGePlane::intersectWith(): the bounded plane holds no trimmed geometry");
  }
  NvGePlane aCarrier;
  aCarrier.set (theBndPln.pointOnPlane(), theBndPln.normal());
  NvGeLine3d aLine;
  if (!intersectWith (aCarrier, aLine, theTol))
  {
    return false;
  }
  // Clip the line parameter against the uv rectangle of the bounded plane.
  double anU1 = 0.0, anU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
  aTrimmed->Bounds (anU1, anU2, aV1, aV2);

  NvGePoint3d aFrameOrigin;
  NvGeVector3d anUAxis, aVAxis;
  aCarrier.getCoordSystem (aFrameOrigin, anUAxis, aVAxis);
  const NvGePoint3d aLineOrigin = aLine.pointOnLine();
  const NvGeVector3d aDir = aLine.direction();
  auto LocalUV = [&] (double theT, double& theU, double& theV)
  {
    const double aPx = aLineOrigin.x + theT * aDir.x - aFrameOrigin.x;
    const double aPy = aLineOrigin.y + theT * aDir.y - aFrameOrigin.y;
    const double aPz = aLineOrigin.z + theT * aDir.z - aFrameOrigin.z;
    theU = aPx * anUAxis.x + aPy * anUAxis.y + aPz * anUAxis.z;
    theV = aPx * aVAxis.x + aPy * aVAxis.y + aPz * aVAxis.z;
  };
  double aUat0 = 0.0, aVat0 = 0.0;
  double aUat1 = 0.0, aVat1 = 0.0;
  LocalUV (0.0, aUat0, aVat0);
  LocalUV (1.0, aUat1, aVat1);
  const double aDu = aUat1 - aUat0;
  const double aDv = aVat1 - aVat0;
  const double aTol = theTol.equalPoint();
  double aLow = -std::numeric_limits<double>::infinity();
  double anUp = std::numeric_limits<double>::infinity();
  auto ClipSide = [&aLow, &anUp, aTol] (double theStart, double theDelta, double theLo, double theHi)
  {
    if (std::abs (theDelta) <= 1e-300)
    {
      return theStart >= theLo - aTol && theStart <= theHi + aTol;
    }
    double aT1 = (theLo - theStart) / theDelta;
    double aT2 = (theHi - theStart) / theDelta;
    if (aT1 > aT2)
    {
      const double aTmp = aT1;
      aT1 = aT2;
      aT2 = aTmp;
    }
    aLow = aLow > aT1 ? aLow : aT1;
    anUp = anUp < aT2 ? anUp : aT2;
    return true;
  };
  if (!ClipSide (aUat0, aDu, anU1, anU2) || !ClipSide (aVat0, aDv, aV1, aV2) || !(anUp > aLow))
  {
    return false;
  }
  theResultLineSeg.set (NvGePoint3d (aLineOrigin.x + aLow * aDir.x, aLineOrigin.y + aLow * aDir.y,
                                     aLineOrigin.z + aLow * aDir.z),
                        NvGePoint3d (aLineOrigin.x + anUp * aDir.x, aLineOrigin.y + anUp * aDir.y,
                                     aLineOrigin.z + anUp * aDir.z));
  return true;
}

//=================================================================================================

NvGePlane& NvGePlane::set (const NvGePoint3d& thePnt, const NvGeVector3d& theNormal)
{
  const gp_Dir aNormal = DirOrThrow (theNormal.x, theNormal.y, theNormal.z, "set");
  gp_Dir anU, aV;
  BasisForNormal (aNormal, anU, aV);
  SetPlane (mpImpEnt, gp_Ax3 (gp_Pnt (thePnt.x, thePnt.y, thePnt.z), aNormal, anU));
  return *this;
}

//=================================================================================================

NvGePlane& NvGePlane::set (const NvGePoint3d& thePntU, const NvGePoint3d& theOrg,
                           const NvGePoint3d& thePntV)
{
  *this = NvGePlane (thePntU, theOrg, thePntV);
  return *this;
}

//=================================================================================================

NvGePlane& NvGePlane::set (double theA, double theB, double theC, double theD)
{
  *this = NvGePlane (theA, theB, theC, theD);
  return *this;
}

//=================================================================================================

NvGePlane& NvGePlane::set (const NvGePoint3d& theOrg, const NvGeVector3d& theUAxis,
                           const NvGeVector3d& theVAxis)
{
  *this = NvGePlane (theOrg, theUAxis, theVAxis);
  return *this;
}

//=================================================================================================

NvGePlane& NvGePlane::operator = (const NvGePlane& theSrc)
{
  NvGePlanarEnt::operator= (theSrc);
  return *this;
}
