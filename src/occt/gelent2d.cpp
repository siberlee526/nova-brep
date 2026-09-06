// gelent2d.cpp - implementation of NvGeLinearEnt2d, the base of the 2d
// linear entities (line, ray, segment).
//
// Every concrete linear entity stores a Geom2d_Line in its impl, either
// directly (NvGeLine2d) or wrapped into a Geom2d_TrimmedCurve whose
// parameter measures the signed distance along the line (NvGeRay2d,
// NvGeLineSeg2d). The algorithms below therefore work on (origin, unit
// direction) pairs plus the trimmed parameter bounds of each entity.

#include <gelent2d.h>

#include <NvException.h>
#include <geimpdata.h>
#include <geline2d.h>
#include <gelnsg2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <Standard_Handle.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace
{

// Returns the unbounded line behind a linear entity geometry; trimmed
// entities (rays, segments) are unwrapped to their basis line. Returns a
// null handle when the geometry is not a line at all.
occ::handle<Geom2d_Line> BasisLineOf (const occ::handle<Standard_Transient>& theGeom)
{
  const occ::handle<Geom2d_Curve> aCurve = occ::down_cast<Geom2d_Curve> (theGeom);
  if (aCurve.IsNull())
  {
    return {};
  }
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (aCurve);
  return occ::down_cast<Geom2d_Line> (aTrimmed.IsNull()
                                        ? occ::handle<Geom2d_Curve> (aCurve)
                                        : aTrimmed->BasisCurve());
}

// Line extraction that reports a missing or foreign impl geometry with the
// project error type (a linear entity impl always stores a line).
occ::handle<Geom2d_Line> LineOfOrThrow (const occ::handle<Standard_Transient>& theGeom,
                                        const char* theMethod)
{
  const occ::handle<Geom2d_Line> aLine = BasisLineOf (theGeom);
  if (aLine.IsNull())
  {
    throw NvException (std::string ("NvGeLinearEnt2d::") + theMethod
                       + "(): the entity has no line geometry");
  }
  return aLine;
}

// Parameter bounds of an entity along its own direction; the bounds of an
// unbounded line span (-infinity, +infinity). Rays and segments store the
// bounds explicitly in their trimmed curve.
void ParamBoundsOf (const NvGeImpEntity3d& theImp, double& theLower, double& theUpper)
{
  theLower = -std::numeric_limits<double>::infinity();
  theUpper =  std::numeric_limits<double>::infinity();
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (theImp.Geom());
  if (!aTrimmed.IsNull())
  {
    theLower = aTrimmed->FirstParameter();
    theUpper = aTrimmed->LastParameter();
  }
}

// Overlap intervals are limited to the Geom2d_Line domain so that the
// segment produced for the (possibly unbounded) common part remains
// representable.
constexpr double THE_PARAM_INF = Precision::Infinite();

}

//=================================================================================================

NvGePoint2d NvGeLinearEnt2d::pointOnLine () const
{
  const occ::handle<Geom2d_Line> aLine = LineOfOrThrow (mpImpEnt->Geom(), "pointOnLine");
  const gp_Pnt2d anOrigin = aLine->Lin2d().Location();
  return NvGePoint2d (anOrigin.X(), anOrigin.Y());
}

//=================================================================================================

NvGeVector2d NvGeLinearEnt2d::direction () const
{
  const occ::handle<Geom2d_Line> aLine = LineOfOrThrow (mpImpEnt->Geom(), "direction");
  const gp_Dir2d& aDir = aLine->Lin2d().Direction();
  return NvGeVector2d (aDir.X(), aDir.Y());
}

//=================================================================================================

void NvGeLinearEnt2d::getLine (NvGeLine2d& theLine) const
{
  theLine.set (pointOnLine(), direction());
}

//=================================================================================================

Adesk::Boolean NvGeLinearEnt2d::intersectWith (const NvGeLinearEnt2d& theLine, NvGePoint2d& theIntPnt,
                                               const NvGeTol& theTol) const
{
  const NvGeVector2d aDir1 = direction();
  const NvGeVector2d aDir2 = theLine.direction();
  const NvGePoint2d  anOrg1 = pointOnLine();
  const NvGePoint2d  anOrg2 = theLine.pointOnLine();

  const double aDenom = aDir1.x * aDir2.y - aDir1.y * aDir2.x;
  if (std::abs (aDenom) <= theTol.equalVector())
  {
    // Parallel (or colinear) entities have no single intersection point.
    return false;
  }

  // Solve org1 + t*dir1 = org2 + s*dir2 with Cramer's rule; both parameters
  // measure the signed distance along the respective unit direction.
  const double aDx = anOrg2.x - anOrg1.x;
  const double aDy = anOrg2.y - anOrg1.y;
  const double aT = (aDx * aDir2.y - aDy * aDir2.x) / aDenom;
  const double aS = (aDx * aDir1.y - aDy * aDir1.x) / aDenom;

  double aLow1 = 0.0;
  double anUp1 = 0.0;
  double aLow2 = 0.0;
  double anUp2 = 0.0;
  ParamBoundsOf (*mpImpEnt, aLow1, anUp1);
  ParamBoundsOf (*theLine.mpImpEnt, aLow2, anUp2);

  const double aTolPnt = theTol.equalPoint();
  if (aT < aLow1 - aTolPnt || aT > anUp1 + aTolPnt
   || aS < aLow2 - aTolPnt || aS > anUp2 + aTolPnt)
  {
    return false;
  }

  theIntPnt = NvGePoint2d (anOrg1.x + aT * aDir1.x, anOrg1.y + aT * aDir1.y);
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeLinearEnt2d::overlap (const NvGeLinearEnt2d& theLine,
                                         NvGeLinearEnt2d*& theOverlap,
                                         const NvGeTol& theTol) const
{
  theOverlap = nullptr;
  if (!isColinearTo (theLine, theTol))
  {
    return false;
  }

  const NvGeVector2d aDir1 = direction();
  const NvGePoint2d  anOrg1 = pointOnLine();
  const NvGeVector2d aDir2 = theLine.direction();
  const NvGePoint2d  anOrg2 = theLine.pointOnLine();

  double aLow1 = 0.0;
  double anUp1 = 0.0;
  double aLow2 = 0.0;
  double anUp2 = 0.0;
  ParamBoundsOf (*mpImpEnt, aLow1, anUp1);
  ParamBoundsOf (*theLine.mpImpEnt, aLow2, anUp2);

  // Map the other entity's parameter interval onto this line's parameter.
  // The directions are parallel, so the mapping is a shift plus a sign.
  const double anOffset = (anOrg2.x - anOrg1.x) * aDir1.x + (anOrg2.y - anOrg1.y) * aDir1.y;
  const double aSense = aDir1.x * aDir2.x + aDir1.y * aDir2.y; // +1 or -1
  double aStart = anOffset + aLow2 * aSense;
  double anEnd  = anOffset + anUp2 * aSense;
  if (aStart > anEnd)
  {
    std::swap (aStart, anEnd);
  }

  const double aLow = std::max (aLow1, aStart);
  const double anUp = std::min (anUp1, anEnd);
  if (anUp - aLow <= theTol.equalPoint())
  {
    return false; // empty or point-like overlap
  }

  const double aClampedLow = std::max (aLow, -THE_PARAM_INF);
  const double aClampedUp  = std::min (anUp,  THE_PARAM_INF);
  const NvGePoint2d aPnt1 (anOrg1.x + aClampedLow * aDir1.x, anOrg1.y + aClampedLow * aDir1.y);
  const NvGePoint2d aPnt2 (anOrg1.x + aClampedUp * aDir1.x,  anOrg1.y + aClampedUp * aDir1.y);
  theOverlap = GENEWLOC (NvGeLineSeg2d, this) (aPnt1, aPnt2);
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeLinearEnt2d::isParallelTo (const NvGeLinearEnt2d& theLine,
                                              const NvGeTol& theTol) const
{
  const NvGeVector2d aDir1 = direction();
  const NvGeVector2d aDir2 = theLine.direction();
  return std::abs (aDir1.x * aDir2.y - aDir1.y * aDir2.x) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGeLinearEnt2d::isPerpendicularTo (const NvGeLinearEnt2d& theLine,
                                                   const NvGeTol& theTol) const
{
  const NvGeVector2d aDir1 = direction();
  const NvGeVector2d aDir2 = theLine.direction();
  return std::abs (aDir1.x * aDir2.x + aDir1.y * aDir2.y) <= theTol.equalVector();
}

//=================================================================================================

Adesk::Boolean NvGeLinearEnt2d::isColinearTo (const NvGeLinearEnt2d& theLine,
                                              const NvGeTol& theTol) const
{
  if (!isParallelTo (theLine, theTol))
  {
    return false;
  }
  const NvGePoint2d  anOrg1 = pointOnLine();
  const NvGePoint2d  anOrg2 = theLine.pointOnLine();
  const NvGeVector2d aDir1 = direction();
  // Perpendicular distance from the other origin to this line; the
  // direction is unit, so the cross product magnitude is the distance.
  const double aCross = (anOrg2.x - anOrg1.x) * aDir1.y - (anOrg2.y - anOrg1.y) * aDir1.x;
  return std::abs (aCross) <= theTol.equalPoint();
}

//=================================================================================================

void NvGeLinearEnt2d::getPerpLine (const NvGePoint2d& thePnt, NvGeLine2d& thePerpLine) const
{
  const NvGeVector2d aDir = direction();
  thePerpLine.set (thePnt, NvGeVector2d (-aDir.y, aDir.x));
}

//=================================================================================================

NvGeLinearEnt2d& NvGeLinearEnt2d::operator = (const NvGeLinearEnt2d& theLine)
{
  NvGeEntity2d::operator= (theLine);
  return *this;
}

//=================================================================================================

NvGeLinearEnt2d::NvGeLinearEnt2d ()
: NvGeCurve2d()
{
}

//=================================================================================================

NvGeLinearEnt2d::NvGeLinearEnt2d (const NvGeLinearEnt2d& theSrc)
: NvGeCurve2d (theSrc)
{
}
