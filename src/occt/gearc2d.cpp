// gearc2d.cpp - implementation of NvGeCircArc2d based on OCCT Geom2d_Circle
// and Geom2d_TrimmedCurve.
//
// A full circle (360 degrees) is stored as a plain Geom2d_Circle; any
// smaller arc is stored as a Geom2d_TrimmedCurve over a full circle basis.
// All angles are measured counter-clockwise from the reference vector and
// are normalized into [0, 2*PI); the geometric sweep always runs from the
// start angle to the end angle, counter-clockwise or clockwise according to
// the isClockWise flag. Equal (mod 2*PI) input angles denote a full circle.
//
// Clockwise arcs store the negated angle range on the flipped basis axis
// (y = -perp (x)): keeping the Sense = true trim, ascending parameters
// traverse the arc clockwise from the start angle to the end angle, so the
// covered counter-clockwise angle span equals the requested arc. The
// direction is recovered from the handedness of the basis axis; the stored
// angles are recovered as 2*PI - first and 2*PI - last.
//
// Analytic conventions:
//   - intersections with a linear entity are computed against its carrier
//     line; segment and ray bounds are honored through the entity kind,
//   - tangent() draws the tangent at the argument itself when it lies on
//     the circle and at its radial projection when it lies outside,
//   - the fillet set() supports line/line, line/arc and arc/arc pairs via
//     offset-curve intersections (tangency may fall on the carrier line or
//     on the full circle of an arc; the arc nearest the input parameters
//     wins); the three-curve variant supports three carrier lines and
//     selects among the inscribed and escribed circles of the triangle.

#include <gearc2d.h>

#include <NvException.h>
#include <gegblabb.h>
#include <geimpdata.h>
#include <gelent2d.h>
#include <geline2d.h>
#include <gelnsg2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <Geom2d_Circle.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <gp.hxx>
#include <gp_Ax22d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

constexpr double THE_TWO_PI = 6.283185307179586476925286766559;

// Angular sweeps at or below this bound denote a full circle.
constexpr double THE_FULL_SWEEP_TOL = 1e-12;

// Normalizes an angle into [0, THE_TWO_PI).
double NormalizeAngle (double theAngle)
{
  double aResult = std::fmod (theAngle, THE_TWO_PI);
  if (aResult < 0.0)
  {
    aResult += THE_TWO_PI;
  }
  return aResult;
}

gp_Pnt2d AsPnt2d (const NvGePoint2d& thePnt)
{
  return gp_Pnt2d (thePnt.x, thePnt.y);
}

double PointDistance (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  return std::sqrt (aDx * aDx + aDy * aDy);
}

bool IsSamePoint (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2, double theTol)
{
  return PointDistance (thePnt1, thePnt2) <= theTol;
}

// Plain-data description of a circular arc, recovered from (or fed into)
// the implementation geometry. Angles are counter-clockwise from the
// reference vector; the sweep runs from the start angle to the end angle in
// the isClockWise direction.
struct ArcDef
{
  NvGePoint2d  Center;
  double       Radius = 0.0;
  double       StartAngle = 0.0;
  double       EndAngle = 0.0;
  NvGeVector2d RefVec = NvGeVector2d (1.0, 0.0);
  bool         IsClockWise = false;
  bool         IsFullCircle = false;
};

// Completes an arc description: normalizes the angles into [0, THE_TWO_PI)
// and collapses a vanishing sweep into the full-circle flag.
void NormalizeArcDef (ArcDef& theDef)
{
  theDef.StartAngle = NormalizeAngle (theDef.StartAngle);
  theDef.EndAngle = NormalizeAngle (theDef.EndAngle);
  if (theDef.IsFullCircle)
  {
    theDef.StartAngle = 0.0;
    theDef.EndAngle = THE_TWO_PI;
    return;
  }
  const double aSweep = theDef.IsClockWise
    ? NormalizeAngle (theDef.StartAngle - theDef.EndAngle)
    : NormalizeAngle (theDef.EndAngle - theDef.StartAngle);
  if (aSweep <= THE_FULL_SWEEP_TOL)
  {
    theDef.IsFullCircle = true;
    theDef.StartAngle = 0.0;
    theDef.EndAngle = THE_TWO_PI;
  }
}

// True when theAngle (counter-clockwise from the reference vector) lies on
// the arc sweep.
bool SweepContains (const ArcDef& theArc, double theAngle, double theTol)
{
  if (theArc.IsFullCircle)
  {
    return true;
  }
  const double aSweep = theArc.IsClockWise
    ? NormalizeAngle (theArc.StartAngle - theArc.EndAngle)
    : NormalizeAngle (theArc.EndAngle - theArc.StartAngle);
  const double aDelta = theArc.IsClockWise
    ? NormalizeAngle (theArc.StartAngle - theAngle)
    : NormalizeAngle (theAngle - theArc.StartAngle);
  return aDelta <= aSweep + theTol;
}

// Counter-clockwise angle of a radial vector relative to the arc frame.
double AngleInFrame (const ArcDef& theArc, const NvGeVector2d& theRadial)
{
  return std::atan2 (theArc.RefVec.x * theRadial.y - theArc.RefVec.y * theRadial.x,
                     theArc.RefVec.x * theRadial.x + theArc.RefVec.y * theRadial.y);
}

NvGePoint2d PointAtAngle (const ArcDef& theArc, double theAngle)
{
  const double aCos = std::cos (theAngle);
  const double aSin = std::sin (theAngle);
  return NvGePoint2d (theArc.Center.x + theArc.Radius * (aCos * theArc.RefVec.x - aSin * theArc.RefVec.y),
                      theArc.Center.y + theArc.Radius * (aSin * theArc.RefVec.x + aCos * theArc.RefVec.y));
}

// Builds the implementation geometry for theArc. Throws NvException for a
// non-positive radius, a degenerate reference vector or an OCCT-side
// construction failure (a vanishing sweep after normalization).
occ::handle<Geom2d_Curve> BuildGeometry (const ArcDef& theArc, const char* theMethod)
{
  if (theArc.Radius <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCircArc2d::") + theMethod
                       + "(): the radius must be positive");
  }
  if (std::sqrt (theArc.RefVec.lengthSqrd()) <= gp::Resolution())
  {
    throw NvException (std::string ("NvGeCircArc2d::") + theMethod
                       + "(): the reference vector is degenerate");
  }
  const gp_Dir2d aRefDir (theArc.RefVec.x, theArc.RefVec.y);
  // gp_Ax22d recomputes the y direction from the sense flag, which encodes
  // the parameterization direction of the full-circle basis.
  const gp_Ax22d anAxis (AsPnt2d (theArc.Center), aRefDir, !theArc.IsClockWise);
  const occ::handle<Geom2d_Circle> aCircle (new Geom2d_Circle (anAxis, theArc.Radius));
  if (theArc.IsFullCircle)
  {
    return occ::handle<Geom2d_Curve> (aCircle);
  }
  double aU1 = theArc.StartAngle;
  double aU2 = theArc.EndAngle;
  if (theArc.IsClockWise)
  {
    // The flipped basis axis encodes the clockwise direction; negating the
    // angle range keeps a Sense = true trim, so ascending parameters
    // traverse the arc clockwise from the start angle to the end angle
    // (see the banner).
    aU1 = THE_TWO_PI - theArc.StartAngle;
    aU2 = THE_TWO_PI - theArc.EndAngle;
  }
  try
  {
    return occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (aCircle, aU1, aU2, true, true));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (std::string ("NvGeCircArc2d::") + theMethod
                       + "(): the arc angles do not define a valid sweep");
  }
}

// Recovers the plain-data description of the arc stored in theImp. Throws
// NvException when the geometry is not a circle-based curve.
ArcDef ArcDefOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (theImp);
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (aCurve);
  occ::handle<Geom2d_Circle> aCircle = aTrimmed.IsNull()
    ? occ::down_cast<Geom2d_Circle> (aCurve)
    : occ::down_cast<Geom2d_Circle> (aTrimmed->BasisCurve());
  if (aCircle.IsNull())
  {
    throw NvException (std::string ("NvGeCircArc2d::") + theMethod
                       + "(): the implementation geometry is not a circle");
  }
  ArcDef aDef;
  const gp_Pnt2d aLoc = aCircle->Location();
  aDef.Center = NvGePoint2d (aLoc.X(), aLoc.Y());
  aDef.Radius = aCircle->Radius();
  const gp_Ax22d aPos = aCircle->Position();
  aDef.RefVec = NvGeVector2d (aPos.XDirection().X(), aPos.XDirection().Y());
  const bool isDirect = aPos.XDirection().Crossed (aPos.YDirection()) > 0.0;
  if (aTrimmed.IsNull())
  {
    aDef.IsFullCircle = true;
    aDef.StartAngle = 0.0;
    aDef.EndAngle = THE_TWO_PI;
  }
  else if (isDirect)
  {
    aDef.StartAngle = NormalizeAngle (aTrimmed->FirstParameter());
    aDef.EndAngle = NormalizeAngle (aTrimmed->LastParameter());
  }
  else
  {
    aDef.StartAngle = NormalizeAngle (THE_TWO_PI - aTrimmed->FirstParameter());
    aDef.EndAngle = NormalizeAngle (THE_TWO_PI - aTrimmed->LastParameter());
  }
  aDef.IsClockWise = !isDirect;
  return aDef;
}

// Replaces the impl geometry, detaching the impl first when it is shared
// (copy-on-write; see the banner of geimpdata.h).
void ReplaceGeometry (NvGeImpEntity3d*& theImp, const occ::handle<Geom2d_Curve>& theGeom)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
  }
  theImp->SetGeom (theGeom);
}

// Classifies degenerate three-point input; returns the matching error code
// or NvGe::kOk.
NvGe::ErrorCondition Arc3PointsError (const NvGePoint2d& theStart, const NvGePoint2d& thePoint,
                                      const NvGePoint2d& theEnd, const NvGeTol& theTol)
{
  if (IsSamePoint (theStart, thePoint, theTol.equalPoint()))
  {
    return NvGe::kEqualArg1Arg2;
  }
  if (IsSamePoint (theStart, theEnd, theTol.equalPoint()))
  {
    return NvGe::kEqualArg1Arg3;
  }
  if (IsSamePoint (thePoint, theEnd, theTol.equalPoint()))
  {
    return NvGe::kEqualArg2Arg3;
  }
  const NvGeVector2d aAB (thePoint.x - theStart.x, thePoint.y - theStart.y);
  const NvGeVector2d aAC (theEnd.x - theStart.x, theEnd.y - theStart.y);
  const double aDet = aAB.x * aAC.y - aAB.y * aAC.x;
  const double aLenAB = std::sqrt (aAB.lengthSqrd());
  const double aLenAC = std::sqrt (aAC.lengthSqrd());
  if (std::abs (aDet) <= theTol.equalVector() * std::max (aLenAB, aLenAC))
  {
    return NvGe::kLinearlyDependentArg1Arg2Arg3;
  }
  return NvGe::kOk;
}

// Computes the arc through three distinct, non-collinear points. The
// reference vector is the x axis; the direction follows the point order.
ArcDef ArcFrom3Points (const NvGePoint2d& theStart, const NvGePoint2d& thePoint,
                       const NvGePoint2d& theEnd)
{
  const NvGeVector2d aAB (thePoint.x - theStart.x, thePoint.y - theStart.y);
  const NvGeVector2d aAC (theEnd.x - theStart.x, theEnd.y - theStart.y);
  const double aDet = aAB.x * aAC.y - aAB.y * aAC.x;
  // Circumcenter from (O - S) . AB = |AB|^2 / 2 and (O - S) . AC = |AC|^2/2.
  const double aLenAB2 = aAB.lengthSqrd();
  const double aLenAC2 = aAC.lengthSqrd();
  const double aInvDet = 0.5 / aDet;
  const NvGePoint2d aCenter (theStart.x + (aLenAB2 * aAC.y - aLenAC2 * aAB.y) * aInvDet,
                             theStart.y + (aAB.x * aLenAC2 - aAC.x * aLenAB2) * aInvDet);
  const double anAngStart = std::atan2 (theStart.y - aCenter.y, theStart.x - aCenter.x);
  const double anAngMid = std::atan2 (thePoint.y - aCenter.y, thePoint.x - aCenter.x);
  const double anAngEnd = std::atan2 (theEnd.y - aCenter.y, theEnd.x - aCenter.x);
  ArcDef aDef;
  aDef.Center = aCenter;
  aDef.Radius = PointDistance (theStart, aCenter);
  aDef.StartAngle = NormalizeAngle (anAngStart);
  aDef.EndAngle = NormalizeAngle (anAngEnd);
  aDef.RefVec = NvGeVector2d (1.0, 0.0);
  aDef.IsClockWise = NormalizeAngle (anAngMid - anAngStart) >= NormalizeAngle (anAngEnd - anAngStart);
  aDef.IsFullCircle = false;
  return aDef;
}

// Computes the arc from two endpoints and a bulge value; throws for
// degenerate input. When theBulgeFlag is true theBulge is the signed
// sagitta of the arc, otherwise it is tan (sweep / 4). A positive bulge
// yields a counter-clockwise arc.
ArcDef ArcFromBulge (const NvGePoint2d& theStart, const NvGePoint2d& theEnd,
                     double theBulge, bool theBulgeFlag, const NvGeTol& theTol,
                     const char* theMethod)
{
  const std::string aContext = std::string ("NvGeCircArc2d::") + theMethod + "()";
  const NvGeVector2d aChord (theEnd.x - theStart.x, theEnd.y - theStart.y);
  const double aChordLen = std::sqrt (aChord.lengthSqrd());
  if (aChordLen <= theTol.equalPoint())
  {
    throw NvException (aContext + ": the endpoints are coincident");
  }
  const double anAbsBulge = std::abs (theBulge);
  if (anAbsBulge <= theTol.equalPoint())
  {
    throw NvException (aContext + ": a zero bulge defines a straight chord, not an arc");
  }
  const NvGeVector2d aUnit (aChord.x / aChordLen, aChord.y / aChordLen);
  // Points from the chord midpoint toward the center of a counter-clockwise
  // arc (the center lies to the left of the travel direction).
  const NvGeVector2d aCenterDir (-aUnit.y, aUnit.x);
  double aRadius = 0.0;
  double aCenterDist = 0.0;
  if (theBulgeFlag)
  {
    aRadius = (anAbsBulge * anAbsBulge + 0.25 * aChordLen * aChordLen) / (2.0 * anAbsBulge);
    aCenterDist = aRadius - anAbsBulge;
  }
  else
  {
    const double aSweep = 4.0 * std::atan (anAbsBulge);
    const double aHalfSine = std::sin (0.5 * aSweep);
    if (aHalfSine <= theTol.equalVector())
    {
      throw NvException (aContext + ": the bulge defines a nearly full arc"
                                  " with an unbounded radius");
    }
    aRadius = 0.5 * aChordLen / aHalfSine;
    aCenterDist = aRadius * std::cos (0.5 * aSweep);
  }
  const NvGePoint2d aMid (0.5 * (theStart.x + theEnd.x), 0.5 * (theStart.y + theEnd.y));
  const double aSide = (theBulge > 0.0) ? 1.0 : -1.0;
  ArcDef aDef;
  aDef.Center = NvGePoint2d (aMid.x + aSide * aCenterDist * aCenterDir.x,
                             aMid.y + aSide * aCenterDist * aCenterDir.y);
  aDef.Radius = aRadius;
  aDef.StartAngle = NormalizeAngle (std::atan2 (theStart.y - aDef.Center.y, theStart.x - aDef.Center.x));
  aDef.EndAngle = NormalizeAngle (std::atan2 (theEnd.y - aDef.Center.y, theEnd.x - aDef.Center.x));
  aDef.RefVec = NvGeVector2d (1.0, 0.0);
  aDef.IsClockWise = theBulge < 0.0;
  aDef.IsFullCircle = false;
  return aDef;
}

// Parameter bounds of a linear entity along its carrier line as signed
// distances from pointOnLine() along direction() (infinite for lines).
void LinearBoundsOf (const NvGeLinearEnt2d& theLine, double& theLower, double& theUpper)
{
  constexpr double anInf = std::numeric_limits<double>::max();
  theLower = -anInf;
  theUpper = anInf;
  if (theLine.type() == NvGe::kLineSeg2d)
  {
    const NvGeLineSeg2d& aSeg = static_cast<const NvGeLineSeg2d&> (theLine);
    const NvGePoint2d anAnchor = theLine.pointOnLine();
    const NvGeVector2d aDir = theLine.direction();
    const NvGeVector2d aRel1 (aSeg.startPoint().x - anAnchor.x, aSeg.startPoint().y - anAnchor.y);
    const NvGeVector2d aRel2 (aSeg.endPoint().x - anAnchor.x, aSeg.endPoint().y - anAnchor.y);
    const double aT1 = aRel1.x * aDir.x + aRel1.y * aDir.y;
    const double aT2 = aRel2.x * aDir.x + aRel2.y * aDir.y;
    theLower = std::min (aT1, aT2);
    theUpper = std::max (aT1, aT2);
  }
  else if (theLine.type() == NvGe::kRay2d)
  {
    theLower = 0.0;
  }
}

// Tangency carrier of a curve the fillet must touch: the carrier line of a
// linear entity or the full circle of a circular arc.
struct TangentSite
{
  bool         IsCircle = false;
  NvGePoint2d  Point;      // line anchor point or circle center
  NvGeVector2d Dir;        // unit carrier direction (lines)
  double       Radius = 0.0;
  NvGeVector2d RefVec = NvGeVector2d (1.0, 0.0);  // parameter origin (circles)
};

bool ExtractTangentSite (const NvGeCurve2d& theCurve, TangentSite& theSite)
{
  if (theCurve.isKindOf (NvGe::kLinearEnt2d))
  {
    const NvGeLinearEnt2d& aLine = static_cast<const NvGeLinearEnt2d&> (theCurve);
    theSite.IsCircle = false;
    theSite.Point = aLine.pointOnLine();
    theSite.Dir = aLine.direction();
    return true;
  }
  if (theCurve.isKindOf (NvGe::kCircArc2d))
  {
    const NvGeCircArc2d& anArc = static_cast<const NvGeCircArc2d&> (theCurve);
    theSite.IsCircle = true;
    theSite.Point = anArc.center();
    theSite.Radius = anArc.radius();
    theSite.RefVec = anArc.refVec();
    return true;
  }
  return false;
}

// A curve the fillet center must lie on: an offset of a tangency carrier
// by the fillet radius.
struct OffsetPrimitive
{
  bool         IsCircle = false;
  NvGePoint2d  Point;   // anchor point or circle center
  NvGeVector2d Dir;     // lines
  double       Radius = 0.0;  // circles
};

void CollectOffsetPrimitives (const TangentSite& theSite, double theFilletRadius,
                              std::vector<OffsetPrimitive>& theOut)
{
  if (!theSite.IsCircle)
  {
    const NvGeVector2d aPerp (-theSite.Dir.y, theSite.Dir.x);
    OffsetPrimitive aPrim;
    aPrim.IsCircle = false;
    aPrim.Dir = theSite.Dir;
    aPrim.Point = NvGePoint2d (theSite.Point.x + theFilletRadius * aPerp.x,
                               theSite.Point.y + theFilletRadius * aPerp.y);
    theOut.push_back (aPrim);
    aPrim.Point = NvGePoint2d (theSite.Point.x - theFilletRadius * aPerp.x,
                               theSite.Point.y - theFilletRadius * aPerp.y);
    theOut.push_back (aPrim);
    return;
  }
  OffsetPrimitive aPrim;
  aPrim.IsCircle = true;
  aPrim.Point = theSite.Point;
  aPrim.Radius = theSite.Radius + theFilletRadius;
  theOut.push_back (aPrim);
  const double anInner = theSite.Radius - theFilletRadius;
  if (std::abs (anInner) > THE_FULL_SWEEP_TOL)
  {
    aPrim.Radius = std::abs (anInner);
    theOut.push_back (aPrim);
  }
}

// Appends all intersection points of two offset primitives (candidate
// fillet centers). Parallel lines and nested circles yield no candidate.
void CollectCenters (const OffsetPrimitive& theA, const OffsetPrimitive& theB,
                     std::vector<NvGePoint2d>& theCenters)
{
  if (!theA.IsCircle && !theB.IsCircle)
  {
    const double aDen = theA.Dir.x * theB.Dir.y - theA.Dir.y * theB.Dir.x;
    if (std::abs (aDen) <= THE_FULL_SWEEP_TOL)
    {
      return;
    }
    const NvGeVector2d aRel (theB.Point.x - theA.Point.x, theB.Point.y - theA.Point.y);
    const double aT = (aRel.x * theB.Dir.y - aRel.y * theB.Dir.x) / aDen;
    theCenters.push_back (NvGePoint2d (theA.Point.x + aT * theA.Dir.x,
                                       theA.Point.y + aT * theA.Dir.y));
    return;
  }
  if (theA.IsCircle != theB.IsCircle)
  {
    const OffsetPrimitive& aLine = theA.IsCircle ? theB : theA;
    const OffsetPrimitive& aCirc = theA.IsCircle ? theA : theB;
    const NvGeVector2d aRel (aLine.Point.x - aCirc.Point.x, aLine.Point.y - aCirc.Point.y);
    const double aB = 2.0 * (aLine.Dir.x * aRel.x + aLine.Dir.y * aRel.y);
    const double aC = aRel.lengthSqrd() - aCirc.Radius * aCirc.Radius;
    const double aDisc = aB * aB - 4.0 * aC;
    if (aDisc < 0.0)
    {
      return;
    }
    const double aRoot = std::sqrt (aDisc);
    const double aParams[2] = { 0.5 * (-aB - aRoot), 0.5 * (-aB + aRoot) };
    const int aCount = (aRoot > THE_FULL_SWEEP_TOL) ? 2 : 1;
    for (int i = 0; i < aCount; ++i)
    {
      theCenters.push_back (NvGePoint2d (aLine.Point.x + aParams[i] * aLine.Dir.x,
                                         aLine.Point.y + aParams[i] * aLine.Dir.y));
    }
    return;
  }
  const NvGeVector2d aRel (theB.Point.x - theA.Point.x, theB.Point.y - theA.Point.y);
  const double aDist = std::sqrt (aRel.lengthSqrd());
  if (aDist <= THE_FULL_SWEEP_TOL)
  {
    return;
  }
  const NvGeVector2d aU (aRel.x / aDist, aRel.y / aDist);
  const double aAlong = (aDist * aDist + theA.Radius * theA.Radius - theB.Radius * theB.Radius)
                        / (2.0 * aDist);
  const double aH2 = theA.Radius * theA.Radius - aAlong * aAlong;
  if (aH2 < 0.0)
  {
    return;
  }
  const NvGeVector2d aPerp (-aU.y, aU.x);
  const NvGePoint2d aBase (theA.Point.x + aAlong * aU.x, theA.Point.y + aAlong * aU.y);
  const double aH = std::sqrt (aH2);
  theCenters.push_back (NvGePoint2d (aBase.x + aH * aPerp.x, aBase.y + aH * aPerp.y));
  if (aH > THE_FULL_SWEEP_TOL)
  {
    theCenters.push_back (NvGePoint2d (aBase.x - aH * aPerp.x, aBase.y - aH * aPerp.y));
  }
}

// Tangency point on the original carrier of theSite for a fillet of
// theFilletRadius centered at theCenter.
NvGePoint2d TangencyPointOf (const TangentSite& theSite, double theFilletRadius,
                             const NvGePoint2d& theCenter)
{
  const NvGeVector2d aRel (theCenter.x - theSite.Point.x, theCenter.y - theSite.Point.y);
  if (!theSite.IsCircle)
  {
    const double aT = aRel.x * theSite.Dir.x + aRel.y * theSite.Dir.y;
    return NvGePoint2d (theSite.Point.x + aT * theSite.Dir.x,
                        theSite.Point.y + aT * theSite.Dir.y);
  }
  const double aDist = std::sqrt (aRel.lengthSqrd());
  if (aDist <= gp::Resolution())
  {
    return theSite.Point;
  }
  double aSign = 1.0;
  if (theFilletRadius > theSite.Radius
   && std::abs (aDist - (theFilletRadius - theSite.Radius))
        <= 1e-6 * std::max (1.0, theFilletRadius))
  {
    aSign = -1.0;   // the fillet encloses the carrier circle
  }
  return NvGePoint2d (theSite.Point.x + aSign * theSite.Radius * aRel.x / aDist,
                      theSite.Point.y + aSign * theSite.Radius * aRel.y / aDist);
}

// Point at theParam on the carrier: a signed distance for lines, the
// counter-clockwise angle from the reference vector for circles.
NvGePoint2d PointAtParam (const TangentSite& theSite, double theParam)
{
  if (!theSite.IsCircle)
  {
    return NvGePoint2d (theSite.Point.x + theParam * theSite.Dir.x,
                        theSite.Point.y + theParam * theSite.Dir.y);
  }
  const double aCos = std::cos (theParam);
  const double aSin = std::sin (theParam);
  return NvGePoint2d (theSite.Point.x + theSite.Radius * (aCos * theSite.RefVec.x - aSin * theSite.RefVec.y),
                      theSite.Point.y + theSite.Radius * (aSin * theSite.RefVec.x + aCos * theSite.RefVec.y));
}

double ParamOfPoint (const TangentSite& theSite, const NvGePoint2d& thePnt)
{
  const NvGeVector2d aRel (thePnt.x - theSite.Point.x, thePnt.y - theSite.Point.y);
  if (!theSite.IsCircle)
  {
    return aRel.x * theSite.Dir.x + aRel.y * theSite.Dir.y;
  }
  return std::atan2 (theSite.RefVec.x * aRel.y - theSite.RefVec.y * aRel.x,
                     theSite.RefVec.x * aRel.x + theSite.RefVec.y * aRel.y);
}

// Fills theDef with the minor-sweep arc around theCenter through the two
// tangency points; returns false for a degenerate (near-coincident) pair.
bool MakeFilletDef (const NvGePoint2d& theCenter, double theRadius,
                    const NvGePoint2d& theT1, const NvGePoint2d& theT2,
                    double theTol, ArcDef& theDef)
{
  if (PointDistance (theT1, theT2) <= theTol)
  {
    return false;
  }
  const double anAng1 = std::atan2 (theT1.y - theCenter.y, theT1.x - theCenter.x);
  const double anAng2 = std::atan2 (theT2.y - theCenter.y, theT2.x - theCenter.x);
  const double aSweepCCW = NormalizeAngle (anAng2 - anAng1);
  const double aSweepCW = NormalizeAngle (anAng1 - anAng2);
  if (std::min (aSweepCCW, aSweepCW) <= THE_FULL_SWEEP_TOL)
  {
    return false;
  }
  theDef.Center = theCenter;
  theDef.Radius = theRadius;
  theDef.StartAngle = NormalizeAngle (anAng1);
  theDef.EndAngle = NormalizeAngle (anAng2);
  theDef.RefVec = NvGeVector2d (1.0, 0.0);
  theDef.IsClockWise = aSweepCW < aSweepCCW;
  theDef.IsFullCircle = false;
  return true;
}

}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  ArcDef aDef;
  aDef.Radius = 1.0;
  aDef.IsFullCircle = true;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc2d, BuildGeometry (aDef, "NvGeCircArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d (const NvGeCircArc2d& theSrc)
: NvGeCurve2d (theSrc)
{
}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d (const NvGePoint2d& theCent, double theRadius)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  ArcDef aDef;
  aDef.Center = theCent;
  aDef.Radius = theRadius;
  aDef.IsFullCircle = true;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc2d, BuildGeometry (aDef, "NvGeCircArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d (const NvGePoint2d& theCent, double theRadius,
                              double theStartAngle, double theEndAngle,
                              const NvGeVector2d& theRefVec, Adesk::Boolean theIsClockWise)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  ArcDef aDef;
  aDef.Center = theCent;
  aDef.Radius = theRadius;
  aDef.StartAngle = theStartAngle;
  aDef.EndAngle = theEndAngle;
  aDef.RefVec = theRefVec;
  aDef.IsClockWise = theIsClockWise == Adesk::kTrue;
  NormalizeArcDef (aDef);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc2d, BuildGeometry (aDef, "NvGeCircArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d (const NvGePoint2d& theStart, const NvGePoint2d& thePoint,
                              const NvGePoint2d& theEnd)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  if (Arc3PointsError (theStart, thePoint, theEnd, NvGeContext::gTol) != NvGe::kOk)
  {
    throw NvException ("NvGeCircArc2d::NvGeCircArc2d(): the three points are coincident"
                       " or collinear and do not define an arc");
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc2d,
                                  BuildGeometry (ArcFrom3Points (theStart, thePoint, theEnd),
                                                 "NvGeCircArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeCircArc2d::NvGeCircArc2d (const NvGePoint2d& theStart, const NvGePoint2d& theEnd,
                              double theBulge, Adesk::Boolean theBulgeFlag)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc2d,
                                  BuildGeometry (ArcFromBulge (theStart, theEnd, theBulge,
                                                               theBulgeFlag == Adesk::kTrue,
                                                               NvGeContext::gTol,
                                                               "NvGeCircArc2d"),
                                                 "NvGeCircArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::intersectWith (const NvGeLinearEnt2d& theLine, int& theIntn,
                                             NvGePoint2d& theP1, NvGePoint2d& theP2,
                                             const NvGeTol& theTol) const
{
  theIntn = 0;
  const ArcDef aDef = ArcDefOf (mpImpEnt, "intersectWith");
  const NvGePoint2d anAnchor = theLine.pointOnLine();
  const NvGeVector2d aDir = theLine.direction();
  const double aDirLen = std::sqrt (aDir.lengthSqrd());
  if (aDirLen <= gp::Resolution())
  {
    return false;
  }
  const NvGeVector2d aUnit (aDir.x / aDirLen, aDir.y / aDirLen);
  const NvGeVector2d aRel (anAnchor.x - aDef.Center.x, anAnchor.y - aDef.Center.y);
  // Solve |anchor + t * unit - center|^2 = R^2 (t is the distance along the
  // carrier line).
  const double aB = 2.0 * (aUnit.x * aRel.x + aUnit.y * aRel.y);
  const double aC = aRel.lengthSqrd() - aDef.Radius * aDef.Radius;
  const double aDisc = aB * aB - 4.0 * aC;
  if (aDisc < 0.0)
  {
    return false;
  }
  const double aRoot = std::sqrt (aDisc);
  const double aParams[2] = { 0.5 * (-aB - aRoot), 0.5 * (-aB + aRoot) };
  const int aRootCount = (aRoot > theTol.equalPoint()) ? 2 : 1;
  double aLower = 0.0;
  double anUpper = 0.0;
  LinearBoundsOf (theLine, aLower, anUpper);
  const double anAngleTol = theTol.equalPoint() / std::max (aDef.Radius, gp::Resolution());
  for (int i = 0; i < aRootCount; ++i)
  {
    if (aParams[i] < aLower - theTol.equalPoint() || aParams[i] > anUpper + theTol.equalPoint())
    {
      continue;
    }
    const NvGePoint2d aPnt (anAnchor.x + aParams[i] * aUnit.x,
                            anAnchor.y + aParams[i] * aUnit.y);
    const NvGeVector2d aRadial (aPnt.x - aDef.Center.x, aPnt.y - aDef.Center.y);
    if (!SweepContains (aDef, AngleInFrame (aDef, aRadial), anAngleTol))
    {
      continue;
    }
    if (theIntn == 0)
    {
      theP1 = aPnt;
      ++theIntn;
    }
    else if (PointDistance (theP1, aPnt) > theTol.equalPoint())
    {
      theP2 = aPnt;
      ++theIntn;
    }
  }
  return theIntn > 0;
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::intersectWith (const NvGeCircArc2d& theArc, int& theIntn,
                                             NvGePoint2d& theP1, NvGePoint2d& theP2,
                                             const NvGeTol& theTol) const
{
  theIntn = 0;
  const ArcDef aDef1 = ArcDefOf (mpImpEnt, "intersectWith");
  const ArcDef aDef2 = ArcDefOf (theArc.mpImpEnt, "intersectWith");
  const double aTol = theTol.equalPoint();
  const NvGeVector2d aRel (aDef2.Center.x - aDef1.Center.x, aDef2.Center.y - aDef1.Center.y);
  const double aDist = std::sqrt (aRel.lengthSqrd());
  if (aDist <= aTol)
  {
    // Concentric circles: either nested (no meeting) or coincident (an
    // infinite number of shared points, which cannot be enumerated).
    return false;
  }
  const double aRSum = aDef1.Radius + aDef2.Radius;
  const double aRDiff = std::abs (aDef1.Radius - aDef2.Radius);
  if (aDist > aRSum + aTol || aDist < aRDiff - aTol)
  {
    return false;
  }
  const NvGeVector2d aU (aRel.x / aDist, aRel.y / aDist);
  const double aAlong = (aDist * aDist + aDef1.Radius * aDef1.Radius - aDef2.Radius * aDef2.Radius)
                        / (2.0 * aDist);
  const double aH2 = aDef1.Radius * aDef1.Radius - aAlong * aAlong;
  NvGePoint2d aPoints[2];
  int aCount = 0;
  if (aH2 <= aTol * std::max (aDef1.Radius, 1.0))
  {
    aPoints[0] = NvGePoint2d (aDef1.Center.x + aAlong * aU.x, aDef1.Center.y + aAlong * aU.y);
    aCount = 1;
  }
  else
  {
    const double aH = std::sqrt (aH2);
    const NvGeVector2d aPerp (-aU.y, aU.x);
    const NvGePoint2d aBase (aDef1.Center.x + aAlong * aU.x, aDef1.Center.y + aAlong * aU.y);
    aPoints[0] = NvGePoint2d (aBase.x + aH * aPerp.x, aBase.y + aH * aPerp.y);
    aPoints[1] = NvGePoint2d (aBase.x - aH * aPerp.x, aBase.y - aH * aPerp.y);
    aCount = 2;
  }
  const double anAngleTol = aTol / std::max (std::max (aDef1.Radius, aDef2.Radius),
                                             gp::Resolution());
  for (int i = 0; i < aCount; ++i)
  {
    const NvGeVector2d aRadial1 (aPoints[i].x - aDef1.Center.x, aPoints[i].y - aDef1.Center.y);
    const NvGeVector2d aRadial2 (aPoints[i].x - aDef2.Center.x, aPoints[i].y - aDef2.Center.y);
    if (!SweepContains (aDef1, AngleInFrame (aDef1, aRadial1), anAngleTol)
     || !SweepContains (aDef2, AngleInFrame (aDef2, aRadial2), anAngleTol))
    {
      continue;
    }
    if (theIntn == 0)
    {
      theP1 = aPoints[i];
      ++theIntn;
    }
    else if (PointDistance (theP1, aPoints[i]) > aTol)
    {
      theP2 = aPoints[i];
      ++theIntn;
    }
  }
  return theIntn > 0;
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::tangent (const NvGePoint2d& thePnt, NvGeLine2d& theLine,
                                       const NvGeTol& theTol) const
{
  NvGeError anError = NvGe::kOk;
  return tangent (thePnt, theLine, theTol, anError);
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::tangent (const NvGePoint2d& thePnt, NvGeLine2d& theLine,
                                       const NvGeTol& theTol, NvGeError& theError) const
{
  const ArcDef aDef = ArcDefOf (mpImpEnt, "tangent");
  const NvGeVector2d aRadial (thePnt.x - aDef.Center.x, thePnt.y - aDef.Center.y);
  const double aDist = std::sqrt (aRadial.lengthSqrd());
  const NvGeVector2d aTangent (-aRadial.y, aRadial.x);
  if (std::abs (aDist - aDef.Radius) <= theTol.equalPoint())
  {
    theLine.set (thePnt, aTangent);
    theError = NvGe::kArg1OnThis;
    return true;
  }
  if (aDist < aDef.Radius - theTol.equalPoint())
  {
    theError = NvGe::kArg1InsideThis;
    return false;
  }
  // Outside: the tangent at the radial projection of the point.
  const double anInvDist = (aDist > gp::Resolution()) ? 1.0 / aDist : 0.0;
  const NvGePoint2d aTouch (aDef.Center.x + aDef.Radius * aRadial.x * anInvDist,
                            aDef.Center.y + aDef.Radius * aRadial.y * anInvDist);
  theLine.set (aTouch, aTangent);
  theError = NvGe::kArg1TooBig;
  return true;
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::isInside (const NvGePoint2d& thePnt, const NvGeTol& theTol) const
{
  const ArcDef aDef = ArcDefOf (mpImpEnt, "isInside");
  return PointDistance (thePnt, aDef.Center) < aDef.Radius - theTol.equalPoint();
}

//=================================================================================================

NvGePoint2d NvGeCircArc2d::center () const
{
  return ArcDefOf (mpImpEnt, "center").Center;
}

//=================================================================================================

double NvGeCircArc2d::radius () const
{
  return ArcDefOf (mpImpEnt, "radius").Radius;
}

//=================================================================================================

double NvGeCircArc2d::startAng () const
{
  return ArcDefOf (mpImpEnt, "startAng").StartAngle;
}

//=================================================================================================

double NvGeCircArc2d::endAng () const
{
  return ArcDefOf (mpImpEnt, "endAng").EndAngle;
}

//=================================================================================================

Adesk::Boolean NvGeCircArc2d::isClockWise () const
{
  return ArcDefOf (mpImpEnt, "isClockWise").IsClockWise;
}

//=================================================================================================

NvGeVector2d NvGeCircArc2d::refVec () const
{
  return ArcDefOf (mpImpEnt, "refVec").RefVec;
}

//=================================================================================================

NvGePoint2d NvGeCircArc2d::startPoint () const
{
  const ArcDef aDef = ArcDefOf (mpImpEnt, "startPoint");
  return PointAtAngle (aDef, aDef.StartAngle);
}

//=================================================================================================

NvGePoint2d NvGeCircArc2d::endPoint () const
{
  const ArcDef aDef = ArcDefOf (mpImpEnt, "endPoint");
  return PointAtAngle (aDef, aDef.EndAngle);
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::setCenter (const NvGePoint2d& theCent)
{
  ArcDef aDef = ArcDefOf (mpImpEnt, "setCenter");
  aDef.Center = theCent;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setCenter"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::setRadius (double theRadius)
{
  ArcDef aDef = ArcDefOf (mpImpEnt, "setRadius");
  aDef.Radius = theRadius;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setRadius"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::setAngles (double theStartAng, double theEndAng)
{
  ArcDef aDef = ArcDefOf (mpImpEnt, "setAngles");
  aDef.StartAngle = theStartAng;
  aDef.EndAngle = theEndAng;
  aDef.IsFullCircle = false;
  NormalizeArcDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setAngles"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::setToComplement ()
{
  // The complementary arc keeps the same start/end angles but runs in the
  // opposite direction, so flipping the flag swaps the covered sweep
  // between the two angle rays.
  ArcDef aDef = ArcDefOf (mpImpEnt, "setToComplement");
  aDef.IsClockWise = !aDef.IsClockWise;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setToComplement"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::setRefVec (const NvGeVector2d& theVec)
{
  ArcDef aDef = ArcDefOf (mpImpEnt, "setRefVec");
  aDef.RefVec = theVec;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setRefVec"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGePoint2d& theCent, double theRadius)
{
  ArcDef aDef;
  aDef.Center = theCent;
  aDef.Radius = theRadius;
  aDef.IsFullCircle = true;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGePoint2d& theCent, double theRadius,
                                   double theAng1, double theAng2,
                                   const NvGeVector2d& theRefVec, Adesk::Boolean theIsClockWise)
{
  ArcDef aDef;
  aDef.Center = theCent;
  aDef.Radius = theRadius;
  aDef.StartAngle = theAng1;
  aDef.EndAngle = theAng2;
  aDef.RefVec = theRefVec;
  aDef.IsClockWise = theIsClockWise == Adesk::kTrue;
  NormalizeArcDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGePoint2d& theStart, const NvGePoint2d& thePoint,
                                   const NvGePoint2d& theEnd)
{
  if (Arc3PointsError (theStart, thePoint, theEnd, NvGeContext::gTol) != NvGe::kOk)
  {
    throw NvException ("NvGeCircArc2d::set(): the three points are coincident"
                       " or collinear and do not define an arc");
  }
  ReplaceGeometry (mpImpEnt, BuildGeometry (ArcFrom3Points (theStart, thePoint, theEnd), "set"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGePoint2d& theStart, const NvGePoint2d& thePoint,
                                   const NvGePoint2d& theEnd, NvGeError& theError)
{
  theError = Arc3PointsError (theStart, thePoint, theEnd, NvGeContext::gTol);
  if (theError != NvGe::kOk)
  {
    return *this;   // the object is unchanged on error
  }
  ReplaceGeometry (mpImpEnt, BuildGeometry (ArcFrom3Points (theStart, thePoint, theEnd), "set"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGePoint2d& theStart, const NvGePoint2d& theEnd,
                                   double theBulge, Adesk::Boolean theBulgeFlag)
{
  ReplaceGeometry (mpImpEnt,
                   BuildGeometry (ArcFromBulge (theStart, theEnd, theBulge,
                                                theBulgeFlag == Adesk::kTrue,
                                                NvGeContext::gTol, "set"),
                                  "set"));
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGeCurve2d& theCurve1, const NvGeCurve2d& theCurve2,
                                   double theRadius, double& theParam1, double& theParam2,
                                   Adesk::Boolean& theSuccess)
{
  theSuccess = Adesk::kFalse;
  const NvGeTol& aTol = NvGeContext::gTol;
  if (theRadius <= aTol.equalPoint())
  {
    return *this;
  }
  TangentSite aSite1;
  TangentSite aSite2;
  if (!ExtractTangentSite (theCurve1, aSite1) || !ExtractTangentSite (theCurve2, aSite2))
  {
    return *this;
  }
  std::vector<OffsetPrimitive> anOffsets1;
  std::vector<OffsetPrimitive> anOffsets2;
  CollectOffsetPrimitives (aSite1, theRadius, anOffsets1);
  CollectOffsetPrimitives (aSite2, theRadius, anOffsets2);
  std::vector<NvGePoint2d> aCenters;
  for (std::size_t i = 0; i < anOffsets1.size(); ++i)
  {
    for (std::size_t j = 0; j < anOffsets2.size(); ++j)
    {
      CollectCenters (anOffsets1[i], anOffsets2[j], aCenters);
    }
  }
  const NvGePoint2d aHint1 = PointAtParam (aSite1, theParam1);
  const NvGePoint2d aHint2 = PointAtParam (aSite2, theParam2);
  bool hasBest = false;
  double aBestScore = 0.0;
  NvGePoint2d aBestCenter;
  NvGePoint2d aBestT1;
  NvGePoint2d aBestT2;
  for (std::size_t i = 0; i < aCenters.size(); ++i)
  {
    const NvGePoint2d aT1 = TangencyPointOf (aSite1, theRadius, aCenters[i]);
    const NvGePoint2d aT2 = TangencyPointOf (aSite2, theRadius, aCenters[i]);
    if (PointDistance (aT1, aT2) <= aTol.equalPoint())
    {
      continue;   // the fillet would be a full circle
    }
    const double aScore = PointDistance (aT1, aHint1) + PointDistance (aT2, aHint2);
    if (!hasBest || aScore < aBestScore)
    {
      hasBest = true;
      aBestScore = aScore;
      aBestCenter = aCenters[i];
      aBestT1 = aT1;
      aBestT2 = aT2;
    }
  }
  ArcDef aDef;
  if (!hasBest || !MakeFilletDef (aBestCenter, theRadius, aBestT1, aBestT2,
                                  aTol.equalPoint(), aDef))
  {
    return *this;
  }
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  theParam1 = ParamOfPoint (aSite1, aBestT1);
  theParam2 = ParamOfPoint (aSite2, aBestT2);
  theSuccess = Adesk::kTrue;
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::set (const NvGeCurve2d& theCurve1, const NvGeCurve2d& theCurve2,
                                   const NvGeCurve2d& theCurve3,
                                   double& theParam1, double& theParam2, double& theParam3,
                                   Adesk::Boolean& theSuccess)
{
  theSuccess = Adesk::kFalse;
  const NvGeTol& aTol = NvGeContext::gTol;
  // Only three carrier lines are supported: the candidate circles are the
  // inscribed and escribed circles of the triangle they form.
  TangentSite aSite1;
  TangentSite aSite2;
  TangentSite aSite3;
  if (!ExtractTangentSite (theCurve1, aSite1) || aSite1.IsCircle
   || !ExtractTangentSite (theCurve2, aSite2) || aSite2.IsCircle
   || !ExtractTangentSite (theCurve3, aSite3) || aSite3.IsCircle)
  {
    return *this;
  }
  OffsetPrimitive aLine1;
  aLine1.Point = aSite1.Point;
  aLine1.Dir = aSite1.Dir;
  OffsetPrimitive aLine2;
  aLine2.Point = aSite2.Point;
  aLine2.Dir = aSite2.Dir;
  OffsetPrimitive aLine3;
  aLine3.Point = aSite3.Point;
  aLine3.Dir = aSite3.Dir;
  std::vector<NvGePoint2d> aV12;
  std::vector<NvGePoint2d> aV23;
  std::vector<NvGePoint2d> aV31;
  CollectCenters (aLine1, aLine2, aV12);
  CollectCenters (aLine2, aLine3, aV23);
  CollectCenters (aLine3, aLine1, aV31);
  if (aV12.empty() || aV23.empty() || aV31.empty())
  {
    return *this;   // a pair of the carriers is (nearly) parallel
  }
  const NvGePoint2d aV1 = aV12[0];
  const NvGePoint2d aV2 = aV23[0];
  const NvGePoint2d aV3 = aV31[0];
  const double aSide1 = PointDistance (aV2, aV3);   // opposite aV1
  const double aSide2 = PointDistance (aV3, aV1);
  const double aSide3 = PointDistance (aV1, aV2);
  const double aDenoms[4] = { aSide1 + aSide2 + aSide3,
                              -aSide1 + aSide2 + aSide3,
                              aSide1 - aSide2 + aSide3,
                              aSide1 + aSide2 - aSide3 };
  const NvGePoint2d aHints[3] = { PointAtParam (aSite1, theParam1),
                                  PointAtParam (aSite2, theParam2),
                                  PointAtParam (aSite3, theParam3) };
  const TangentSite* aSites[3] = { &aSite1, &aSite2, &aSite3 };
  bool hasBest = false;
  double aBestScore = 0.0;
  NvGePoint2d aBestCenter;
  double aBestRadius = 0.0;
  NvGePoint2d aBestT[2];
  for (int aCandidate = 0; aCandidate < 4; ++aCandidate)
  {
    if (aDenoms[aCandidate] <= aTol.equalPoint())
    {
      continue;   // (nearly) degenerate triangle direction
    }
    const double aWeights[3] = { aSide1, aSide2, aSide3 };
    const double aSigns[3] = { aCandidate == 1 ? -1.0 : 1.0,
                               aCandidate == 2 ? -1.0 : 1.0,
                               aCandidate == 3 ? -1.0 : 1.0 };
    const NvGePoint2d aV[3] = { aV1, aV2, aV3 };
    double aCx = 0.0;
    double aCy = 0.0;
    for (int k = 0; k < 3; ++k)
    {
      aCx += aSigns[k] * aWeights[k] * aV[k].x;
      aCy += aSigns[k] * aWeights[k] * aV[k].y;
    }
    const NvGePoint2d aCenter (aCx / aDenoms[aCandidate], aCy / aDenoms[aCandidate]);
    // The radius is the (common) distance to the three carriers; a mismatch
    // marks a numerically degenerate escribed candidate.
    const NvGeVector2d aRels[3] = {
      NvGeVector2d (aCenter.x - aSites[0]->Point.x, aCenter.y - aSites[0]->Point.y),
      NvGeVector2d (aCenter.x - aSites[1]->Point.x, aCenter.y - aSites[1]->Point.y),
      NvGeVector2d (aCenter.x - aSites[2]->Point.x, aCenter.y - aSites[2]->Point.y)
    };
    const double aDists[3] = {
      std::abs (aSites[0]->Dir.x * aRels[0].y - aSites[0]->Dir.y * aRels[0].x),
      std::abs (aSites[1]->Dir.x * aRels[1].y - aSites[1]->Dir.y * aRels[1].x),
      std::abs (aSites[2]->Dir.x * aRels[2].y - aSites[2]->Dir.y * aRels[2].x)
    };
    if (aDists[0] <= aTol.equalPoint()
     || std::abs (aDists[1] - aDists[0]) > aTol.equalPoint() * (1.0 + aDists[0])
     || std::abs (aDists[2] - aDists[0]) > aTol.equalPoint() * (1.0 + aDists[0]))
    {
      continue;
    }
    const NvGePoint2d aTouches[3] = { TangencyPointOf (aSite1, aDists[0], aCenter),
                                      TangencyPointOf (aSite2, aDists[0], aCenter),
                                      TangencyPointOf (aSite3, aDists[0], aCenter) };
    if (PointDistance (aTouches[0], aTouches[1]) <= aTol.equalPoint())
    {
      continue;   // the fillet would be a full circle
    }
    const double aScore = PointDistance (aTouches[0], aHints[0])
                        + PointDistance (aTouches[1], aHints[1])
                        + PointDistance (aTouches[2], aHints[2]);
    if (!hasBest || aScore < aBestScore)
    {
      hasBest = true;
      aBestScore = aScore;
      aBestCenter = aCenter;
      aBestRadius = aDists[0];
      aBestT[0] = aTouches[0];
      aBestT[1] = aTouches[1];
    }
  }
  ArcDef aDef;
  if (!hasBest || !MakeFilletDef (aBestCenter, aBestRadius, aBestT[0], aBestT[1],
                                  aTol.equalPoint(), aDef))
  {
    return *this;
  }
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  theParam1 = ParamOfPoint (aSite1, aBestT[0]);
  theParam2 = ParamOfPoint (aSite2, aBestT[1]);
  theParam3 = ParamOfPoint (aSite3, TangencyPointOf (aSite3, aBestRadius, aBestCenter));
  theSuccess = Adesk::kTrue;
  return *this;
}

//=================================================================================================

NvGeCircArc2d& NvGeCircArc2d::operator = (const NvGeCircArc2d& theArc)
{
  NvGeEntity2d::operator= (theArc);
  return *this;
}
