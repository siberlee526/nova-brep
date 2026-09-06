// gearc3d.cpp - implementation of NvGeCircArc3d.
//
// The entity stores a full circle as a Geom_Circle and an arc as a
// Geom_TrimmedCurve over a Geom_Circle. The parameter of the carrier circle
// is the angle in radians counted counterclockwise from the reference
// direction of the circle placement. Stored angles are normalized so that
// startAng() lies in [0, 2pi) and the sweep lies in (0, 2pi]; a sweep equal
// to 2pi is stored as a full Geom_Circle.

#include <gearc3d.h>
#include <Nova.h>
#include <NvException.h>
#include <gecurv3d.h>
#include <gegblabb.h>
#include <gelent3d.h>
#include <geline3d.h>
#include <geplanar.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>
#include <geimpdata.h>

#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Mat.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp.hxx>

#include <cmath>

namespace
{

constexpr double THE_PI = 3.14159265358979323846;
constexpr double THE_TWO_PI = 6.28318530717958647692;

// Wraps the angle into [0, 2pi).
double Mod2Pi (double theAngle)
{
  double anAngle = std::fmod (theAngle, THE_TWO_PI);
  if (anAngle < 0.0)
  {
    anAngle += THE_TWO_PI;
  }
  return anAngle;
}

// Returns the carrier circle of the entity geometry, either as the stored
// curve itself or as the basis of a trimmed curve, and reports the trim
// state and its parameter range.
occ::handle<Geom_Circle> CircleOf (const occ::handle<Standard_Transient>& theGeom,
                                   bool& theIsTrimmed, double& theFirst, double& theLast)
{
  theIsTrimmed = false;
  theFirst = 0.0;
  theLast = THE_TWO_PI;
  if (theGeom.IsNull())
  {
    throw NvException ("NvGeCircArc3d: null curve data");
  }
  const occ::handle<Geom_Circle> aCircle = occ::down_cast<Geom_Circle> (theGeom);
  if (!aCircle.IsNull())
  {
    return aCircle;
  }
  const occ::handle<Geom_TrimmedCurve> aTrim = occ::down_cast<Geom_TrimmedCurve> (theGeom);
  if (aTrim.IsNull())
  {
    throw NvException ("NvGeCircArc3d: the entity does not store a circle");
  }
  const occ::handle<Geom_Circle> aBasis = occ::down_cast<Geom_Circle> (aTrim->BasisCurve());
  if (aBasis.IsNull())
  {
    throw NvException ("NvGeCircArc3d: the entity does not store a circle");
  }
  theIsTrimmed = true;
  theFirst = aTrim->FirstParameter();
  theLast = aTrim->LastParameter();
  return aBasis;
}

// Builds the stored geometry: a full Geom_Circle for a sweep equal to 2pi
// and a Geom_TrimmedCurve otherwise. The start angle is normalized into
// [0, 2pi).
occ::handle<Geom_Curve> CircleCurve (const gp_Ax2& thePos, double theRadius,
                                     double theStartAngle, double theEndAngle)
{
  double aSweep = std::fmod (theEndAngle - theStartAngle, THE_TWO_PI);
  if (aSweep <= 0.0)
  {
    aSweep += THE_TWO_PI;
  }
  const double aStart = Mod2Pi (theStartAngle);
  if (aSweep >= THE_TWO_PI)
  {
    return occ::handle<Geom_Curve> (new Geom_Circle (thePos, theRadius));
  }
  return occ::handle<Geom_Curve> (new Geom_TrimmedCurve (
    occ::handle<Geom_Curve> (new Geom_Circle (thePos, theRadius)),
    aStart, aStart + aSweep, true));
}

// Validates a direction argument and returns it as a gp_Dir.
gp_Dir DirOf (const NvGeVector3d& theVec, const char* theMessage)
{
  const gp_Vec aVec (theVec.x, theVec.y, theVec.z);
  if (aVec.Magnitude() <= gp::Resolution())
  {
    throw NvException (theMessage);
  }
  return gp_Dir (aVec);
}

// Angle of the point around the placement, in (-pi, pi].
double AngleOf (const gp_Pnt& thePnt, const gp_Ax2& thePos)
{
  const gp_Vec aRadial (thePos.Location(), thePnt);
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  return std::atan2 (aRadial.Dot (anYDir), aRadial.Dot (anXDir));
}

// Angle of the point around the placement normalized into [0, 2pi).
double PosAngle (const gp_Ax2& thePos, const gp_Pnt& thePnt)
{
  return Mod2Pi (AngleOf (thePnt, thePos));
}

// True when the point angle belongs to the stored arc span; a full circle
// accepts every angle.
bool AngleOnSpan (double theAngle, double theFirst, double theLast, double theTol)
{
  double aDelta = std::fmod (theAngle - theFirst, THE_TWO_PI);
  if (aDelta < -theTol)
  {
    aDelta += THE_TWO_PI;
  }
  return aDelta >= -theTol && aDelta <= (theLast - theFirst) + theTol;
}

// Point of the carrier circle at the angle.
gp_Pnt CirclePoint (const gp_Ax2& thePos, double theRadius, double theAngle)
{
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  const gp_Vec anOffset = anXDir.Multiplied (theRadius * std::cos (theAngle))
                         + anYDir.Multiplied (theRadius * std::sin (theAngle));
  return thePos.Location().Translated (anOffset);
}

// True when the point lies on the carrier circle within the in-plane radial
// tolerance and, for an arc, within its angular span.
bool PointOnArc (const gp_Ax2& thePos, double theRadius, bool theIsTrimmed,
                 double theFirst, double theLast, const gp_Pnt& thePnt, double theTol)
{
  const gp_Vec aRadial (thePos.Location(), thePnt);
  const gp_Vec aNormal (thePos.Direction().XYZ());
  const double anOutOfPlane = aRadial.Dot (aNormal);
  const double aRadialLen = std::sqrt (std::max (
    aRadial.SquareMagnitude() - anOutOfPlane * anOutOfPlane, 0.0));
  if (std::abs (aRadialLen - theRadius) > theTol)
  {
    return false;
  }
  if (!theIsTrimmed)
  {
    return true;
  }
  return AngleOnSpan (AngleOf (thePnt, thePos), theFirst, theLast, theTol);
}

// Intersections of the carrier circle with the in-plane line (thePnt,
// theDir); theDir must be unit. Reports at most two arc-span hits.
int CircleLineHits (const gp_Ax2& thePos, double theRadius, bool theIsTrimmed,
                    double theFirst, double theLast,
                    const gp_Pnt& thePnt, const gp_Vec& theDir,
                    double theTol, gp_Pnt theHits[2])
{
  const gp_Vec aW (thePnt, thePos.Location());
  const double aFoot = aW.Dot (theDir);
  const double aDist2 = aW.SquareMagnitude() - aFoot * aFoot;
  if (aDist2 > (theRadius + theTol) * (theRadius + theTol))
  {
    return 0;
  }
  const double aHalf = std::sqrt (std::max (theRadius * theRadius - aDist2, 0.0));
  int aCount = 0;
  for (int aSide = 0; aSide < 2; ++aSide)
  {
    if (aSide == 1 && aHalf <= theTol)
    {
      break;                                     // tangency: a single hit
    }
    const double anOffset = (aSide == 0) ? -aHalf : aHalf;
    const gp_Pnt aHit = thePnt.Translated (theDir.Multiplied (aFoot + anOffset));
    if (PointOnArc (thePos, theRadius, theIsTrimmed, theFirst, theLast, aHit, theTol))
    {
      theHits[aCount] = aHit;
      ++aCount;
    }
  }
  return aCount;
}

// Center of the circle through three points; throws on coincident or
// collinear points.
gp_Pnt CenterOf3Points (const gp_Pnt& theP1, const gp_Pnt& theP2, const gp_Pnt& theP3)
{
  const gp_Vec aD21 (theP1, theP2);
  const gp_Vec aD31 (theP1, theP3);
  if (aD21.Magnitude() <= gp::Resolution() || aD31.Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeCircArc3d: coincident points do not define a circle");
  }
  const gp_Vec aNormal = aD21.Crossed (aD31);
  if (aNormal.Magnitude() <= 1e-10 * aD21.Magnitude() * aD31.Magnitude())
  {
    throw NvException ("NvGeCircArc3d: collinear points do not define a circle");
  }
  // |C - P1|^2 = |C - P2|^2 and |C - P1|^2 = |C - P3|^2 give two linear
  // equations and the arc plane the third; the system determinant is |n|^2.
  gp_Mat aSys;
  aSys.SetRow (1, aD21.XYZ());
  aSys.SetRow (2, aD31.XYZ());
  aSys.SetRow (3, aNormal.XYZ());
  const gp_XYZ aRhs (0.5 * (theP2.XYZ().SquareModulus() - theP1.XYZ().SquareModulus()),
                     0.5 * (theP3.XYZ().SquareModulus() - theP1.XYZ().SquareModulus()),
                     aNormal.XYZ().Dot (theP1.XYZ()));
  const double aDet = aSys.Determinant();
  gp_Mat aColX (aSys);
  aColX.SetCol (1, aRhs);
  gp_Mat aColY (aSys);
  aColY.SetCol (2, aRhs);
  gp_Mat aColZ (aSys);
  aColZ.SetCol (3, aRhs);
  return gp_Pnt (aColX.Determinant() / aDet,
                 aColY.Determinant() / aDet,
                 aColZ.Determinant() / aDet);
}

// Placement, radius and CCW sweep of the arc running counterclockwise from
// theStart through theMid to theEnd; the start angle is 0 by construction.
void ArcDefOf3Points (const gp_Pnt& theStart, const gp_Pnt& theMid, const gp_Pnt& theEnd,
                      gp_Ax2& thePos, double& theRadius, double& theSweep)
{
  const gp_Pnt aCenter = CenterOf3Points (theStart, theMid, theEnd);
  const gp_Vec aNormal = gp_Vec (theStart, theMid).Crossed (gp_Vec (theStart, theEnd));
  const gp_Vec aRadStart (aCenter, theStart);
  theRadius = aRadStart.Magnitude();
  thePos = gp_Ax2 (aCenter, gp_Dir (aNormal), gp_Dir (aRadStart));
  const double anEnd = Mod2Pi (AngleOf (theEnd, thePos));
  const double aMid = Mod2Pi (AngleOf (theMid, thePos));
  theSweep = (aMid > anEnd) ? anEnd + THE_TWO_PI : anEnd;
}

// Carrier data of the curve entity kinds the fillet construction supports.
struct FilletLine
{
  gp_Pnt origin;
  gp_Vec direction;                              // unit
};

struct FilletCircle
{
  gp_Pnt center;
  gp_Ax2 frame;
  double radius;
};

// Carrier data of a linear entity through its public API.
bool FilletLineOf (const NvGeCurve3d& theCurve, FilletLine& theLine)
{
  if (!theCurve.isKindOf (NvGe::kLinearEnt3d))
  {
    return false;
  }
  const NvGeLinearEnt3d& aLinear = static_cast<const NvGeLinearEnt3d&> (theCurve);
  const NvGePoint3d anOrigin = aLinear.pointOnLine();
  const NvGeVector3d aDirection = aLinear.direction();
  theLine.origin = gp_Pnt (anOrigin.x, anOrigin.y, anOrigin.z);
  theLine.direction = gp_Vec (aDirection.x, aDirection.y, aDirection.z);
  return true;
}

// Carrier data of a circle entity through its public API.
bool FilletCircleOf (const NvGeCurve3d& theCurve, FilletCircle& theCircle)
{
  if (!theCurve.isKindOf (NvGe::kCircArc3d))
  {
    return false;
  }
  const NvGeCircArc3d& anArc = static_cast<const NvGeCircArc3d&> (theCurve);
  const NvGePoint3d aCenter = anArc.center();
  const gp_Pnt aCenterPnt (aCenter.x, aCenter.y, aCenter.z);
  theCircle.center = aCenterPnt;
  theCircle.frame = gp_Ax2 (aCenterPnt,
                            DirOf (anArc.normal(),
                                   "NvGeCircArc3d: a degenerate arc carrier"),
                            DirOf (anArc.refVec(),
                                   "NvGeCircArc3d: a degenerate arc carrier"));
  theCircle.radius = anArc.radius();
  return true;
}

// Intersection of two line carriers; false for parallel or skew lines.
bool LineLineHit (const FilletLine& theL1, const FilletLine& theL2, gp_Pnt& thePoint)
{
  const gp_Vec aD1 = theL1.direction;
  const gp_Vec aD2 = theL2.direction;
  const gp_Vec aDp (theL1.origin, theL2.origin);
  if (std::abs (aDp.Crossed (aD1).Dot (aD2)) > 1e-8 * (1.0 + aDp.Magnitude()))
  {
    return false;                                // skew carriers
  }
  const gp_Vec aNormal = aD1.Crossed (aD2);
  const double aNormal2 = aNormal.SquareMagnitude();
  if (aNormal2 <= gp::Resolution() * gp::Resolution())
  {
    return false;                                // parallel carriers
  }
  const double aT = aDp.Crossed (aD2).Dot (aNormal) / aNormal2;
  thePoint = theL1.origin.Translated (aD1.Multiplied (aT));
  return true;
}

// Carrier parameter of the point on a line carrier.
double LineParamOf (const FilletLine& theLine, const gp_Pnt& thePnt)
{
  return gp_Vec (theLine.origin, thePnt).Dot (theLine.direction);
}

// Projection of the point onto a line carrier, as a position vector.
gp_XYZ LineProjectionOf (const FilletLine& theLine, const gp_XYZ& thePoint)
{
  const gp_Vec aD = theLine.direction;
  return theLine.origin.XYZ()
       + aD.XYZ().Multiplied ((thePoint - theLine.origin.XYZ()).Dot (aD.XYZ()));
}

// Fillet of two line carriers: the center lies on one of the two angle
// bisectors at distance radius / sin(half angle) from the intersection; of
// the four candidates the one whose tangency points are nearest the seeds
// wins.
bool FilletTwoLines (const FilletLine& theL1, double theSeed1,
                     const FilletLine& theL2, double theSeed2,
                     double theRadius,
                     gp_Pnt& theCenter, gp_Pnt& theT1, gp_Pnt& theT2)
{
  gp_Pnt aCorner;
  if (!LineLineHit (theL1, theL2, aCorner))
  {
    return false;
  }
  const gp_Vec aD1 = theL1.direction;
  const gp_Vec aD2 = theL2.direction;
  const double aSinHalf = std::sqrt (std::max (0.5 * (1.0 - aD1.Dot (aD2)), 0.0));
  if (aSinHalf <= gp::Resolution())
  {
    return false;
  }
  const double aH = theRadius / aSinHalf;
  gp_Vec aBis[2] = { aD1.Added (aD2), aD1.Subtracted (aD2) };
  const gp_XYZ aSeed1 = theL1.origin.XYZ() + aD1.XYZ().Multiplied (theSeed1);
  const gp_XYZ aSeed2 = theL2.origin.XYZ() + aD2.XYZ().Multiplied (theSeed2);
  bool aFound = false;
  double aBest = 0.0;
  gp_XYZ aBestC, aBestT1, aBestT2;
  for (int aBisector = 0; aBisector < 2; ++aBisector)
  {
    const double aBisLen = aBis[aBisector].Magnitude();
    if (aBisLen <= gp::Resolution())
    {
      continue;
    }
    const gp_Vec aDir = aBis[aBisector].Multiplied (1.0 / aBisLen);
    for (int aSign = 0; aSign < 2; ++aSign)
    {
      const gp_XYZ aCenter = aCorner.XYZ() + aDir.XYZ().Multiplied (
        (aSign == 0) ? aH : -aH);
      const gp_XYZ aT1 = LineProjectionOf (theL1, aCenter);
      const gp_XYZ aT2 = LineProjectionOf (theL2, aCenter);
      const double aScore = (aT1 - aSeed1).SquareModulus()
                          + (aT2 - aSeed2).SquareModulus();
      if (!aFound || aScore < aBest)
      {
        aFound = true;
        aBest = aScore;
        aBestC = aCenter;
        aBestT1 = aT1;
        aBestT2 = aT2;
      }
    }
  }
  if (!aFound)
  {
    return false;
  }
  theCenter = gp_Pnt (aBestC);
  theT1 = gp_Pnt (aBestT1);
  theT2 = gp_Pnt (aBestT2);
  return true;
}

// Fillet of a line carrier and a circle carrier; both must lie in one
// plane. The center satisfies distance radius from the line and
// radius +/- carrier radius from the carrier center; of the candidates the
// one whose tangency points are nearest the seeds wins.
bool FilletLineAndCircle (const FilletLine& theLine, double theSeedLine,
                          const FilletCircle& theCircle, double theSeedCircle,
                          double theRadius,
                          gp_Pnt& theCenter, gp_Pnt& theT1, gp_Pnt& theT2)
{
  const gp_Vec aNormal (theCircle.frame.Direction().XYZ());
  const gp_Vec aD = theLine.direction;
  const gp_XYZ aP = theLine.origin.XYZ();
  const gp_XYZ aC = theCircle.center.XYZ();
  const double aR = theCircle.radius;
  if (std::abs (aD.Dot (aNormal)) > 1e-8)
  {
    return false;                                // the line crosses the circle plane
  }
  const gp_Vec anOffPlane (aP - aC);
  if (std::abs (anOffPlane.Dot (aNormal)) > 1e-8 * (1.0 + anOffPlane.Magnitude()))
  {
    return false;                                // the line lies off the circle plane
  }
  const gp_Vec aM = aNormal.Crossed (aD);        // in-plane unit normal of the line
  const gp_XYZ aSeedLine = aP + aD.XYZ().Multiplied (theSeedLine);
  const gp_XYZ aSeedCircle = CirclePoint (theCircle.frame, aR, theSeedCircle).XYZ();
  bool aFound = false;
  double aBest = 0.0;
  gp_XYZ aBestC, aBestT1, aBestT2;
  for (int aSide = 0; aSide < 2; ++aSide)
  {
    const double anOffset = (aSide == 0) ? theRadius : -theRadius;
    const gp_XYZ aBase = aP + aM.XYZ().Multiplied (anOffset);  // the offset line
    for (int aMode = 0; aMode < 2; ++aMode)
    {
      const double aTarget = (aMode == 0) ? aR + theRadius
                                          : std::abs (aR - theRadius);
      const gp_Vec aBaseC (aC, gp_Pnt (aBase));
      const double aLin = aBaseC.Dot (aD);
      const double aDisc = aLin * aLin - aBaseC.SquareMagnitude() + aTarget * aTarget;
      if (aDisc < 0.0)
      {
        continue;
      }
      const double aRoot = std::sqrt (aDisc);
      for (int aBranch = 0; aBranch < 2; ++aBranch)
      {
        const double aT = -aLin + ((aBranch == 0) ? aRoot : -aRoot);
        const gp_XYZ aCenter = aBase + aD.XYZ().Multiplied (aT);
        const gp_Vec aRadial (theCircle.center, gp_Pnt (aCenter));
        const double aRadialLen = aRadial.Magnitude();
        if (aRadialLen <= gp::Resolution())
        {
          continue;
        }
        const gp_XYZ aTl = aP + aD.XYZ().Multiplied (aT);      // on the line
        const double aSense = (aRadialLen >= aR) ? 1.0 : -1.0;
        const gp_XYZ aTc = aC
          + aRadial.XYZ().Multiplied (aR * aSense / aRadialLen);  // on the carrier
        const double aScore = (aTl - aSeedLine).SquareModulus()
                            + (aTc - aSeedCircle).SquareModulus();
        if (!aFound || aScore < aBest)
        {
          aFound = true;
          aBest = aScore;
          aBestC = aCenter;
          aBestT1 = aTl;
          aBestT2 = aTc;
        }
      }
    }
  }
  if (!aFound)
  {
    return false;
  }
  theCenter = gp_Pnt (aBestC);
  theT1 = gp_Pnt (aBestT1);
  theT2 = gp_Pnt (aBestT2);
  return true;
}

// Fillet of two circle carriers; both must lie in one plane. The center is
// an intersection of the offset carriers of radii R +/- radius; of the
// candidates the one whose tangency points are nearest the seeds wins.
bool FilletTwoCircles (const FilletCircle& theC1, double theSeed1,
                       const FilletCircle& theC2, double theSeed2,
                       double theRadius,
                       gp_Pnt& theCenter, gp_Pnt& theT1, gp_Pnt& theT2)
{
  const gp_Vec aN1 (theC1.frame.Direction().XYZ());
  const gp_XYZ aC1 = theC1.center.XYZ();
  const gp_XYZ aC2 = theC2.center.XYZ();
  const double aR1 = theC1.radius;
  const double aR2 = theC2.radius;
  const gp_Vec aD (theC1.center, theC2.center);
  const double aDist = aD.Magnitude();
  if (std::abs (aD.Dot (aN1)) > 1e-8 * (1.0 + aDist))
  {
    return false;                                // the carriers are not coplanar
  }
  if (aN1.Crossed (gp_Vec (theC2.frame.Direction().XYZ())).Magnitude() > 1e-8)
  {
    return false;                                // the carriers are not coplanar
  }
  const gp_XYZ aSeed1 = CirclePoint (theC1.frame, aR1, theSeed1).XYZ();
  const gp_XYZ aSeed2 = CirclePoint (theC2.frame, aR2, theSeed2).XYZ();
  bool aFound = false;
  double aBest = 0.0;
  gp_XYZ aBestC, aBestT1, aBestT2;
  for (int aMode1 = 0; aMode1 < 2; ++aMode1)
  {
    const double aTarget1 = (aMode1 == 0) ? aR1 + theRadius
                                          : std::abs (aR1 - theRadius);
    for (int aMode2 = 0; aMode2 < 2; ++aMode2)
    {
      const double aTarget2 = (aMode2 == 0) ? aR2 + theRadius
                                            : std::abs (aR2 - theRadius);
      if (aDist <= gp::Resolution())
      {
        continue;                                // concentric carriers
      }
      if (aDist > aTarget1 + aTarget2 || aDist < std::abs (aTarget1 - aTarget2))
      {
        continue;                                // the offset carriers miss
      }
      const gp_Vec aU = aD.Multiplied (1.0 / aDist);
      const gp_Vec aM = aN1.Crossed (aU);        // in-plane normal of the center line
      const double aA = (aDist * aDist + aTarget1 * aTarget1 - aTarget2 * aTarget2)
                      / (2.0 * aDist);
      const double aH = std::sqrt (std::max (aTarget1 * aTarget1 - aA * aA, 0.0));
      for (int aSign = 0; aSign < 2; ++aSign)
      {
        const gp_XYZ aCenter = aC1
          + aU.XYZ().Multiplied (aA)
          + aM.XYZ().Multiplied ((aSign == 0) ? aH : -aH);
        // a center outside a carrier touches its near side, inside its far side
        const gp_Vec aRad1 (theC1.center, gp_Pnt (aCenter));
        const gp_Vec aRad2 (theC2.center, gp_Pnt (aCenter));
        const double aLen1 = aRad1.Magnitude();
        const double aLen2 = aRad2.Magnitude();
        if (aLen1 <= gp::Resolution() || aLen2 <= gp::Resolution())
        {
          continue;
        }
        const gp_XYZ aTc1 = aC1 + aRad1.XYZ().Multiplied (
          aR1 * ((aLen1 >= aR1) ? 1.0 : -1.0) / aLen1);
        const gp_XYZ aTc2 = aC2 + aRad2.XYZ().Multiplied (
          aR2 * ((aLen2 >= aR2) ? 1.0 : -1.0) / aLen2);
        const double aScore = (aTc1 - aSeed1).SquareModulus()
                            + (aTc2 - aSeed2).SquareModulus();
        if (!aFound || aScore < aBest)
        {
          aFound = true;
          aBest = aScore;
          aBestC = aCenter;
          aBestT1 = aTc1;
          aBestT2 = aTc2;
        }
      }
    }
  }
  if (!aFound)
  {
    return false;
  }
  theCenter = gp_Pnt (aBestC);
  theT1 = gp_Pnt (aBestT1);
  theT2 = gp_Pnt (aBestT2);
  return true;
}

// Storage curve of the fillet arc: counterclockwise from theT1 (where its
// tangent equals theStartTangent) to theT2.
occ::handle<Geom_Curve> FilletArcCurve (const gp_Pnt& theCenter,
                                        const gp_Pnt& theT1, const gp_Pnt& theT2,
                                        double theRadius, const gp_Vec& theStartTangent)
{
  const gp_Vec aRadial (theCenter, theT1);
  const gp_Vec aNormal = aRadial.Crossed (theStartTangent);
  if (aNormal.Magnitude() <= gp::Resolution())
  {
    return nullptr;                              // tangency points coincide
  }
  const gp_Ax2 aPos (theCenter, gp_Dir (aNormal), gp_Dir (aRadial));
  const gp_Vec anEndRadial (theCenter, theT2);
  const gp_Vec anXDir (aPos.XDirection().XYZ());
  const gp_Vec anYDir (aPos.YDirection().XYZ());
  const double anEnd = std::atan2 (anEndRadial.Dot (anYDir),
                                   anEndRadial.Dot (anXDir));
  if (anEnd == 0.0)
  {
    return nullptr;                              // the tangency points coincide
  }
  return CircleCurve (aPos, theRadius, 0.0, (anEnd < 0.0) ? anEnd + THE_TWO_PI : anEnd);
}

}

// NvGeCircArc3d

//=======================================================================
// function : NvGeCircArc3d
// purpose  : Unit circle about the origin in the XY plane.
//=======================================================================
NvGeCircArc3d::NvGeCircArc3d ()
{
  const occ::handle<Geom_Curve> aCurve = CircleCurve (
    gp_Ax2 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)), 1.0, 0.0, THE_TWO_PI);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeCircArc3d
// purpose  : Copy constructor; shares the implementation of the source.
//=======================================================================
NvGeCircArc3d::NvGeCircArc3d (const NvGeCircArc3d& theSrc)
: NvGeCurve3d (theSrc)
{

}

//=======================================================================
// function : NvGeCircArc3d
// purpose  : Full circle about a center in the plane through the normal.
//=======================================================================
NvGeCircArc3d::NvGeCircArc3d (const NvGePoint3d& theCent, const NvGeVector3d& theNrm,
                              double theRadius)
{
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::NvGeCircArc3d(): the radius must be positive");
  }
  const gp_Pnt aCenter (theCent.x, theCent.y, theCent.z);
  const gp_Ax2 aPos (aCenter, DirOf (theNrm,
    "NvGeCircArc3d::NvGeCircArc3d(): the normal direction must be non-zero"));
  const occ::handle<Geom_Curve> aCurve = CircleCurve (aPos, theRadius, 0.0, THE_TWO_PI);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeCircArc3d
// purpose  : Arc from center, normal, reference direction and angles; the
//            sweep runs counterclockwise from the start to the end angle.
//=======================================================================
NvGeCircArc3d::NvGeCircArc3d (const NvGePoint3d& theCent, const NvGeVector3d& theNrm,
                              const NvGeVector3d& theRefVec, double theRadius,
                              double theStartAngle, double theEndAngle)
{
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::NvGeCircArc3d(): the radius must be positive");
  }
  const gp_Dir aNormal = DirOf (theNrm,
    "NvGeCircArc3d::NvGeCircArc3d(): the normal direction must be non-zero");
  const gp_Dir aRef = DirOf (theRefVec,
    "NvGeCircArc3d::NvGeCircArc3d(): the reference direction must be non-zero");
  if (aNormal.XYZ().Crossed (aRef.XYZ()).Modulus()
      <= 1e-10 * aNormal.XYZ().Modulus() * aRef.XYZ().Modulus())
  {
    throw NvException ("NvGeCircArc3d::NvGeCircArc3d(): "
                       "the reference direction must not be parallel to the normal");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z), aNormal, aRef);
  const occ::handle<Geom_Curve> aCurve = CircleCurve (aPos, theRadius,
                                                      theStartAngle, theEndAngle);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeCircArc3d
// purpose  : Arc through three points, counterclockwise from theStart
//            through theMid to theEnd.
//=======================================================================
NvGeCircArc3d::NvGeCircArc3d (const NvGePoint3d& theStart, const NvGePoint3d& theMid,
                              const NvGePoint3d& theEnd)
{
  gp_Ax2 aPos;
  double aRadius = 0.0;
  double aSweep = 0.0;
  ArcDefOf3Points (gp_Pnt (theStart.x, theStart.y, theStart.z),
                   gp_Pnt (theMid.x, theMid.y, theMid.z),
                   gp_Pnt (theEnd.x, theEnd.y, theEnd.z),
                   aPos, aRadius, aSweep);
  const occ::handle<Geom_Curve> aCurve = CircleCurve (aPos, aRadius, 0.0, aSweep);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kCircArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : closestPointToPlane
// purpose  : Point of the arc nearest the plane; the companion output is
//            its orthogonal projection onto the plane.
//=======================================================================
NvGePoint3d NvGeCircArc3d::closestPointToPlane (const NvGePlanarEnt& thePlaneEnt,
                                                NvGePoint3d& thePointOnPlane,
                                                const NvGeTol& theTol) const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = aCircle->Position();
  const gp_Vec aPlaneNormal = DirOf (thePlaneEnt.normal(),
    "NvGeCircArc3d::closestPointToPlane(): the plane entity is degenerate");
  const NvGePoint3d aPlanePoint = thePlaneEnt.pointOnPlane();
  const gp_Pnt& aCenter = aPos.Location();
  const gp_Vec aCenterToPlane = gp_Vec (gp_XYZ (aCenter.XYZ())
    - gp_XYZ (aPlanePoint.x, aPlanePoint.y, aPlanePoint.z));
  const double aRadius = aCircle->Radius();
  const gp_Vec aPlanesCross = gp_Vec (aPos.Direction().XYZ()).Crossed (aPlaneNormal);
  if (aPlanesCross.Magnitude() <= 1e-10)
  {
    // parallel planes: every arc point shares the distance; report the start
    const gp_Pnt aStartPnt = aCircle->EvalD0 (aFirst);
    const double aDist = aCenterToPlane.Dot (aPlaneNormal);
    thePointOnPlane = NvGePoint3d (aStartPnt.X() - aDist * aPlaneNormal.X(),
                                   aStartPnt.Y() - aDist * aPlaneNormal.Y(),
                                   aStartPnt.Z() - aDist * aPlaneNormal.Z());
    return NvGePoint3d (aStartPnt.X(), aStartPnt.Y(), aStartPnt.Z());
  }
  // in-plane signed distance of the carrier point at the angle a:
  //   f(a) = C0 + A cos a + B sin a
  const double aC0 = aCenterToPlane.Dot (aPlaneNormal);
  const double aA = aRadius * gp_Vec (aPos.XDirection().XYZ()).Dot (aPlaneNormal);
  const double aB = aRadius * gp_Vec (aPos.YDirection().XYZ()).Dot (aPlaneNormal);
  const double anAmp = std::sqrt (aA * aA + aB * aB);
  const double aPhase = std::atan2 (aB, aA);
  double aCandidates[4];
  int aNbCandidates = 0;
  if (std::abs (aC0) <= anAmp)
  {
    // the carrier crosses the plane: both zero angles are closest points
    const double aShift = std::acos (std::max (-1.0, std::min (1.0, -aC0 / anAmp)));
    aCandidates[aNbCandidates++] = aPhase + aShift;
    aCandidates[aNbCandidates++] = aPhase - aShift;
  }
  else
  {
    // the extremum of |f| nearest zero
    aCandidates[aNbCandidates++] = (aC0 > 0.0) ? aPhase : aPhase + THE_PI;
  }
  if (aTrimmed)
  {
    aCandidates[aNbCandidates++] = aFirst;
    aCandidates[aNbCandidates++] = aLast;
  }
  bool aFound = false;
  double aBestAngle = 0.0;
  double aBestDist = 0.0;
  for (int aCandidate = 0; aCandidate < aNbCandidates; ++aCandidate)
  {
    double anAngle = aCandidates[aCandidate];
    if (aTrimmed)
    {
      anAngle = Mod2Pi (anAngle - aFirst) + aFirst;
      if (anAngle > aLast + theTol.equalPoint())
      {
        continue;
      }
    }
    const double aDist = aC0 + aA * std::cos (anAngle) + aB * std::sin (anAngle);
    if (!aFound || std::abs (aDist) < std::abs (aBestDist))
    {
      aFound = true;
      aBestAngle = anAngle;
      aBestDist = aDist;
    }
  }
  const gp_Pnt aBestPnt = CirclePoint (aPos, aRadius, aBestAngle);
  thePointOnPlane = NvGePoint3d (aBestPnt.X() - aBestDist * aPlaneNormal.X(),
                                 aBestPnt.Y() - aBestDist * aPlaneNormal.Y(),
                                 aBestPnt.Z() - aBestDist * aPlaneNormal.Z());
  return NvGePoint3d (aBestPnt.X(), aBestPnt.Y(), aBestPnt.Z());
}

//=======================================================================
// function : intersectWith
// purpose  : Intersections with the carrier of a linear entity, restricted
//            to this arc span and the entity range.
//=======================================================================
Nova::Boolean NvGeCircArc3d::intersectWith (const NvGeLinearEnt3d& theLineEnt, int& theIntN,
                                             NvGePoint3d& theP1, NvGePoint3d& theP2,
                                             const NvGeTol& theTol) const
{
  theIntN = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const NvGePoint3d aLineOrigin = theLineEnt.pointOnLine();
  const NvGeVector3d aLineDirection = theLineEnt.direction();
  const gp_Pnt anOrigin (aLineOrigin.x, aLineOrigin.y, aLineOrigin.z);
  const gp_Vec aDir (aLineDirection.x, aLineDirection.y, aLineDirection.z);
  gp_Pnt aHits[2];
  const int aNbHits = CircleLineHits (aCircle->Position(), aCircle->Radius(), aTrimmed,
                                      aFirst, aLast, anOrigin, aDir,
                                      theTol.equalPoint(), aHits);
  int aAccepted = 0;
  for (int aHit = 0; aHit < aNbHits; ++aHit)
  {
    const NvGePoint3d aPnt (aHits[aHit].X(), aHits[aHit].Y(), aHits[aHit].Z());
    if (!theLineEnt.isOn (aPnt, theTol))
    {
      continue;
    }
    if (aAccepted == 0)
    {
      theP1 = aPnt;
    }
    else
    {
      theP2 = aPnt;
    }
    ++aAccepted;
  }
  theIntN = aAccepted;
  return aAccepted > 0;
}

//=======================================================================
// function : intersectWith
// purpose  : Intersections of two circle carriers restricted to both arc
//            spans; coplanar carriers use the planar construction, the
//            general case intersects along the line of the two planes.
//=======================================================================
Nova::Boolean NvGeCircArc3d::intersectWith (const NvGeCircArc3d& theArc, int& theIntN,
                                             NvGePoint3d& theP1, NvGePoint3d& theP2,
                                             const NvGeTol& theTol) const
{
  theIntN = 0;
  bool aTrim1;
  double aFirst1, aLast1;
  const occ::handle<Geom_Circle> aCircle1 = CircleOf (mpImpEnt->Geom(),
                                                      aTrim1, aFirst1, aLast1);
  const gp_Ax2& aPos1 = aCircle1->Position();
  const gp_Pnt& aC1 = aPos1.Location();
  const double aR1 = aCircle1->Radius();
  const NvGePoint3d anOtherCenter = theArc.center();
  const gp_Pnt aC2 (anOtherCenter.x, anOtherCenter.y, anOtherCenter.z);
  const gp_Dir aN2 = DirOf (theArc.normal(),
    "NvGeCircArc3d::intersectWith(): the other arc is degenerate");
  const gp_Ax2 aPos2 (aC2, aN2, DirOf (theArc.refVec(),
    "NvGeCircArc3d::intersectWith(): the other arc is degenerate"));
  const double aR2 = theArc.radius();
  const double aFirst2 = theArc.startAng();
  double aSweep2 = theArc.endAng() - aFirst2;
  if (aSweep2 <= 0.0)
  {
    aSweep2 += THE_TWO_PI;
  }
  const bool aTrim2 = aSweep2 < THE_TWO_PI;
  const double aLast2 = aFirst2 + aSweep2;
  const gp_Vec aDc (aC1, aC2);
  const double aDcLen = aDc.Magnitude();
  const gp_Vec aPlanesCross = gp_Vec (aPos1.Direction().XYZ()).Crossed (
    gp_Vec (aN2.XYZ()));
  gp_Pnt aHits[2];
  int aNbHits = 0;
  bool aNeedCheck1 = false;
  if (aPlanesCross.Magnitude() <= 1e-10)
  {
    // coplanar carriers: the classic planar construction in the arc plane
    if (aDcLen <= theTol.equalPoint() && std::abs (aR1 - aR2) <= theTol.equalPoint())
    {
      return false;                              // coincident carriers
    }
    if (aDcLen > aR1 + aR2 + theTol.equalPoint()
        || aDcLen < std::abs (aR1 - aR2) - theTol.equalPoint())
    {
      return false;
    }
    aNeedCheck1 = true;
    const gp_Vec aU = aDc.Multiplied (1.0 / aDcLen);
    const gp_Vec aM = gp_Vec (aPos1.Direction().XYZ()).Crossed (aU);
    const double aA = (aDcLen * aDcLen + aR1 * aR1 - aR2 * aR2) / (2.0 * aDcLen);
    const double aH = std::sqrt (std::max (aR1 * aR1 - aA * aA, 0.0));
    const gp_Pnt aBase = aC1.Translated (aU.Multiplied (aA));
    for (int aSide = 0; aSide < 2; ++aSide)
    {
      if (aSide == 1 && aH <= theTol.equalPoint())
      {
        break;                                   // tangency: a single hit
      }
      aHits[aNbHits] = aBase.Translated (aM.Multiplied ((aSide == 0) ? aH : -aH));
      ++aNbHits;
    }
  }
  else
  {
    // the line where the two carrier planes meet
    const gp_Vec aLineDir = aPlanesCross.Multiplied (1.0 / aPlanesCross.Magnitude());
    const gp_Vec anInPlane = gp_Vec (aPos1.Direction().XYZ()).Crossed (aLineDir);
    const double anOffset = aDc.Dot (gp_Vec (aN2.XYZ()))
                          / anInPlane.Dot (gp_Vec (aN2.XYZ()));
    const gp_Pnt aLinePoint = aC1.Translated (anInPlane.Multiplied (anOffset));
    aNbHits = CircleLineHits (aPos1, aR1, aTrim1, aFirst1, aLast1,
                              aLinePoint, aLineDir, theTol.equalPoint(), aHits);
  }
  int aAccepted = 0;
  for (int aHit = 0; aHit < aNbHits; ++aHit)
  {
    if (aNeedCheck1
        && !PointOnArc (aPos1, aR1, aTrim1, aFirst1, aLast1, aHits[aHit],
                        theTol.equalPoint()))
    {
      continue;
    }
    if (!PointOnArc (aPos2, aR2, aTrim2, aFirst2, aLast2, aHits[aHit],
                     theTol.equalPoint()))
    {
      continue;
    }
    const NvGePoint3d aPnt (aHits[aHit].X(), aHits[aHit].Y(), aHits[aHit].Z());
    if (aAccepted == 0)
    {
      theP1 = aPnt;
    }
    else
    {
      theP2 = aPnt;
    }
    ++aAccepted;
  }
  theIntN = aAccepted;
  return aAccepted > 0;
}

//=======================================================================
// function : intersectWith
// purpose  : Intersections with the plane, i.e. with the line where the
//            arc plane and the given plane meet.
//=======================================================================
Nova::Boolean NvGeCircArc3d::intersectWith (const NvGePlanarEnt& thePlaneEnt,
                                             int& theNumOfIntersect,
                                             NvGePoint3d& theP1, NvGePoint3d& theP2,
                                             const NvGeTol& theTol) const
{
  theNumOfIntersect = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = aCircle->Position();
  const gp_Vec aPlaneNormal = DirOf (thePlaneEnt.normal(),
    "NvGeCircArc3d::intersectWith(): the plane entity is degenerate");
  const NvGePoint3d aPlanePoint = thePlaneEnt.pointOnPlane();
  const gp_Pnt& aCenter = aPos.Location();
  const gp_Vec aPlanesCross = gp_Vec (aPos.Direction().XYZ()).Crossed (aPlaneNormal);
  if (aPlanesCross.Magnitude() <= 1e-10)
  {
    return false;                // parallel planes: the arc misses or lies in the plane
  }
  const gp_Vec aLineDir = aPlanesCross.Multiplied (1.0 / aPlanesCross.Magnitude());
  const gp_Vec anInPlane = gp_Vec (aPos.Direction().XYZ()).Crossed (aLineDir);
  const gp_Vec aCenterToPlane (aCenter, gp_Pnt (aPlanePoint.x, aPlanePoint.y,
                                                aPlanePoint.z));
  const double anOffset = aCenterToPlane.Dot (aPlaneNormal)
                        / anInPlane.Dot (aPlaneNormal);
  const double aRadius = aCircle->Radius();
  const double aHalf2 = aRadius * aRadius - anOffset * anOffset;
  if (aHalf2 < -(theTol.equalPoint() * theTol.equalPoint()))
  {
    return false;
  }
  const double aHalf = std::sqrt (std::max (aHalf2, 0.0));
  const gp_Pnt aLinePoint = aCenter.Translated (anInPlane.Multiplied (anOffset));
  int aCount = 0;
  for (int aSide = 0; aSide < 2; ++aSide)
  {
    if (aSide == 1 && aHalf <= theTol.equalPoint())
    {
      break;                                     // tangency: a single hit
    }
    const gp_Pnt aHit = aLinePoint.Translated (aLineDir.Multiplied (
      (aSide == 0) ? -aHalf : aHalf));
    if (!PointOnArc (aPos, aRadius, aTrimmed, aFirst, aLast, aHit, theTol.equalPoint()))
    {
      continue;
    }
    const NvGePoint3d aPnt (aHit.X(), aHit.Y(), aHit.Z());
    if (aCount == 0)
    {
      theP1 = aPnt;
    }
    else
    {
      theP2 = aPnt;
    }
    ++aCount;
  }
  theNumOfIntersect = aCount;
  return aCount > 0;
}

//=======================================================================
// function : projIntersectWith
// purpose  : Intersections of the arc with the line projected onto the arc
//            plane along theProjDir; reports the paired points on both.
//=======================================================================
Nova::Boolean NvGeCircArc3d::projIntersectWith (const NvGeLinearEnt3d& theLineEnt,
                                                 const NvGeVector3d& theProjDir,
                                                 int& theNumInt,
                                                 NvGePoint3d& thePntOnArc1,
                                                 NvGePoint3d& thePntOnArc2,
                                                 NvGePoint3d& thePntOnLine1,
                                                 NvGePoint3d& thePntOnLine2,
                                                 const NvGeTol& theTol) const
{
  theNumInt = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = aCircle->Position();
  const gp_Pnt& aCenter = aPos.Location();
  const gp_Vec aPlaneNormal (aPos.Direction().XYZ());
  const gp_Vec aProjRaw (theProjDir.x, theProjDir.y, theProjDir.z);
  if (aProjRaw.Magnitude() <= theTol.equalVector())
  {
    return false;
  }
  const gp_Vec aProj = aProjRaw.Multiplied (1.0 / aProjRaw.Magnitude());
  const double aProjDotN = aProj.Dot (aPlaneNormal);
  if (std::abs (aProjDotN) <= theTol.equalVector())
  {
    return false;                                // projection parallel to the arc plane
  }
  const NvGePoint3d aLineOrigin = theLineEnt.pointOnLine();
  const NvGeVector3d aLineDirection = theLineEnt.direction();
  const gp_Pnt anOrigin (aLineOrigin.x, aLineOrigin.y, aLineOrigin.z);
  const gp_Vec aDir (aLineDirection.x, aLineDirection.y, aLineDirection.z);
  // the line projected onto the arc plane along aProj
  const gp_Vec anOriginOff (anOrigin, aCenter);
  const gp_Pnt aProjOrigin = anOrigin.Translated (aProj.Multiplied (
    anOriginOff.Dot (aPlaneNormal) / aProjDotN));
  const gp_Vec aProjDirRaw = aDir.Subtracted (aProj.Multiplied (
    aDir.Dot (aPlaneNormal) / aProjDotN));
  int aAccepted = 0;
  if (aProjDirRaw.Magnitude() <= theTol.equalVector())
  {
    // the line is parallel to the projection direction: its projection is
    // the single point aProjOrigin
    if (!PointOnArc (aPos, aCircle->Radius(), aTrimmed, aFirst, aLast,
                     aProjOrigin, theTol.equalPoint()))
    {
      return false;
    }
    const gp_Pnt aLineHit = anOrigin.Translated (aDir.Multiplied (
      gp_Vec (anOrigin, aProjOrigin).Dot (aDir)));
    const NvGePoint3d aLinePnt (aLineHit.X(), aLineHit.Y(), aLineHit.Z());
    if (theLineEnt.isOn (aLinePnt, theTol))
    {
      thePntOnArc1 = NvGePoint3d (aProjOrigin.X(), aProjOrigin.Y(), aProjOrigin.Z());
      thePntOnLine1 = aLinePnt;
      aAccepted = 1;
    }
  }
  else
  {
    const gp_Vec aDirUnit = aProjDirRaw.Multiplied (1.0 / aProjDirRaw.Magnitude());
    gp_Pnt aHits[2];
    const int aNbHits = CircleLineHits (aPos, aCircle->Radius(), aTrimmed,
                                        aFirst, aLast, aProjOrigin, aDirUnit,
                                        theTol.equalPoint(), aHits);
    // map an arc point back to the original line:
    //   s = ((A - O) x proj) . (d x proj) / |d x proj|^2
    const gp_Vec aDirCrossProj = aDir.Crossed (aProj);
    const double aDenom = aDirCrossProj.SquareMagnitude();
    for (int aHit = 0; aHit < aNbHits; ++aHit)
    {
      const double aS = gp_Vec (anOrigin, aHits[aHit]).Crossed (aProj)
                      .Dot (aDirCrossProj) / aDenom;
      const gp_Pnt aLineHit = anOrigin.Translated (aDir.Multiplied (aS));
      const NvGePoint3d aLinePnt (aLineHit.X(), aLineHit.Y(), aLineHit.Z());
      if (!theLineEnt.isOn (aLinePnt, theTol))
      {
        continue;
      }
      const NvGePoint3d anArcPnt (aHits[aHit].X(), aHits[aHit].Y(), aHits[aHit].Z());
      if (aAccepted == 0)
      {
        thePntOnArc1 = anArcPnt;
        thePntOnLine1 = aLinePnt;
      }
      else
      {
        thePntOnArc2 = anArcPnt;
        thePntOnLine2 = aLinePnt;
      }
      ++aAccepted;
    }
  }
  theNumInt = aAccepted;
  return aAccepted > 0;
}

//=======================================================================
// function : tangent
// purpose  : Tangent line from the point to the carrier circle.
//=======================================================================
Nova::Boolean NvGeCircArc3d::tangent (const NvGePoint3d& thePnt, NvGeLine3d& theLine,
                                       const NvGeTol& theTol) const
{
  NvGeError anError = NvGe::kOk;
  return tangent (thePnt, theLine, theTol, anError);
}

//=======================================================================
// function : tangent
// purpose  : Tangent line from the point to the carrier circle; reports
//            kArg1InsideThis for an interior point and kArg1OnThis for a
//            point of the carrier. The touch point used is the one
//            counterclockwise of the radial direction.
//=======================================================================
Nova::Boolean NvGeCircArc3d::tangent (const NvGePoint3d& thePnt, NvGeLine3d& theLine,
                                       const NvGeTol& theTol, NvGeError& theError) const
{
  theError = NvGe::kOk;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Pnt aGiven (thePnt.x, thePnt.y, thePnt.z);
  const gp_Pnt& aCenter = aCircle->Position().Location();
  const gp_Vec aRadial (aCenter, aGiven);
  const double aDistance = aRadial.Magnitude();
  if (aDistance <= theTol.equalPoint()
      || aDistance < aCircle->Radius() - theTol.equalPoint())
  {
    theError = NvGe::kArg1InsideThis;
    return false;
  }
  if (aDistance <= aCircle->Radius() + theTol.equalPoint())
  {
    theError = NvGe::kArg1OnThis;
    return false;
  }
  // the touch direction is the radial unit rotated about the axis by the
  // angle with cos = radius / distance (Rodrigues)
  const gp_Vec aAxis (aCircle->Position().Direction().XYZ());
  const gp_Vec aUnitRadial = aRadial.Multiplied (1.0 / aDistance);
  const double aCos = aCircle->Radius() / aDistance;
  const double aSin = std::sqrt (std::max (1.0 - aCos * aCos, 0.0));
  const gp_Vec aTouchDir = aUnitRadial.Multiplied (aCos)
    .Added (aAxis.Crossed (aUnitRadial).Multiplied (aSin))
    .Added (aAxis.Multiplied (aAxis.Dot (aUnitRadial) * (1.0 - aCos)));
  const gp_Pnt aTouch = aCenter.Translated (aTouchDir.Multiplied (aCircle->Radius()));
  theLine = NvGeLine3d (thePnt,
                        NvGeVector3d (gp_Vec (aGiven, aTouch).X(),
                                      gp_Vec (aGiven, aTouch).Y(),
                                      gp_Vec (aGiven, aTouch).Z()));
  return true;
}

//=======================================================================
// function : getPlane
// purpose  : Plane of the carrier circle.
//=======================================================================
void NvGeCircArc3d::getPlane (NvGePlane& thePlane) const
{
  thePlane = NvGePlane (center(), normal());
}

//=======================================================================
// function : isInside
// purpose  : Strict interior test against the full carrier circle, using
//            the in-plane radial distance.
//=======================================================================
Nova::Boolean NvGeCircArc3d::isInside (const NvGePoint3d& thePnt,
                                        const NvGeTol& theTol) const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = aCircle->Position();
  const gp_Vec aRadial (aPos.Location(), gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  const gp_Vec aNormal (aPos.Direction().XYZ());
  const double anOutOfPlane = aRadial.Dot (aNormal);
  const double aRadialLen = std::sqrt (std::max (
    aRadial.SquareMagnitude() - anOutOfPlane * anOutOfPlane, 0.0));
  return aRadialLen < aCircle->Radius() - theTol.equalPoint();
}

//=======================================================================
// function : center
// purpose  : Center of the carrier circle.
//=======================================================================
NvGePoint3d NvGeCircArc3d::center () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Pnt& aCenter = aCircle->Position().Location();
  return NvGePoint3d (aCenter.X(), aCenter.Y(), aCenter.Z());
}

//=======================================================================
// function : normal
// purpose  : Normal of the arc plane.
//=======================================================================
NvGeVector3d NvGeCircArc3d::normal () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Dir& aNormal = aCircle->Position().Direction();
  return NvGeVector3d (aNormal.X(), aNormal.Y(), aNormal.Z());
}

//=======================================================================
// function : refVec
// purpose  : Reference direction; the zero angle starts here.
//=======================================================================
NvGeVector3d NvGeCircArc3d::refVec () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Dir& anXDir = aCircle->Position().XDirection();
  return NvGeVector3d (anXDir.X(), anXDir.Y(), anXDir.Z());
}

//=======================================================================
// function : radius
// purpose  : Radius of the carrier circle.
//=======================================================================
double NvGeCircArc3d::radius () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  return aCircle->Radius();
}

//=======================================================================
// function : startAng
// purpose  : Start angle; zero for a full circle.
//=======================================================================
double NvGeCircArc3d::startAng () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  return aFirst;
}

//=======================================================================
// function : endAng
// purpose  : End angle; 2pi for a full circle.
//=======================================================================
double NvGeCircArc3d::endAng () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  return aLast;
}

//=======================================================================
// function : startPoint
// purpose  : Point at the start angle.
//=======================================================================
NvGePoint3d NvGeCircArc3d::startPoint () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Pnt aPnt = aCircle->EvalD0 (aFirst);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : endPoint
// purpose  : Point at the end angle.
//=======================================================================
NvGePoint3d NvGeCircArc3d::endPoint () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Pnt aPnt = aCircle->EvalD0 (aLast);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : setCenter
// purpose  : Moves the carrier center, keeping the shape and the angles.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::setCenter (const NvGePoint3d& theCent)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  gp_Ax2 aPos = aCircle->Position();
  aPos.SetLocation (gp_Pnt (theCent.x, theCent.y, theCent.z));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aPos, aCircle->Radius(), aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setAxes
// purpose  : Redefines the plane normal and the reference direction,
//            keeping the center, the radius and the angles.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::setAxes (const NvGeVector3d& theNormal,
                                       const NvGeVector3d& theRefVec)
{
  const gp_Dir aNormal = DirOf (theNormal,
    "NvGeCircArc3d::setAxes(): the normal direction must be non-zero");
  const gp_Dir aRef = DirOf (theRefVec,
    "NvGeCircArc3d::setAxes(): the reference direction must be non-zero");
  if (aNormal.XYZ().Crossed (aRef.XYZ()).Modulus()
      <= 1e-10 * aNormal.XYZ().Modulus() * aRef.XYZ().Modulus())
  {
    throw NvException ("NvGeCircArc3d::setAxes(): "
                       "the reference direction must not be parallel to the normal");
  }
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  const gp_Ax2 aPos (aCircle->Position().Location(), aNormal, aRef);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aPos, aCircle->Radius(), aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setRadius
// purpose  : Rescales the carrier radius, keeping the shape and the angles.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::setRadius (double theRadius)
{
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::setRadius(): the radius must be positive");
  }
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aCircle->Position(), theRadius, aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setAngles
// purpose  : Retrimps the carrier; equal angles give the full circle and a
//            negative sweep is measured counterclockwise across zero.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::setAngles (double theStartAngle, double theEndAngle)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Circle> aCircle = CircleOf (mpImpEnt->Geom(),
                                                     aTrimmed, aFirst, aLast);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aCircle->Position(), aCircle->Radius(),
                                  theStartAngle, theEndAngle));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Full circle about a center in the plane through the normal.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGePoint3d& theCent,
                                   const NvGeVector3d& theNrm, double theRadius)
{
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::set(): the radius must be positive");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z),
                     DirOf (theNrm,
                            "NvGeCircArc3d::set(): the normal direction must be non-zero"));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aPos, theRadius, 0.0, THE_TWO_PI));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Arc from center, normal, reference direction and angles.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGePoint3d& theCent, const NvGeVector3d& theNrm,
                                   const NvGeVector3d& theRefVec, double theRadius,
                                   double theStartAngle, double theEndAngle)
{
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::set(): the radius must be positive");
  }
  const gp_Dir aNormal = DirOf (theNrm,
    "NvGeCircArc3d::set(): the normal direction must be non-zero");
  const gp_Dir aRef = DirOf (theRefVec,
    "NvGeCircArc3d::set(): the reference direction must be non-zero");
  if (aNormal.XYZ().Crossed (aRef.XYZ()).Modulus()
      <= 1e-10 * aNormal.XYZ().Modulus() * aRef.XYZ().Modulus())
  {
    throw NvException ("NvGeCircArc3d::set(): "
                       "the reference direction must not be parallel to the normal");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z), aNormal, aRef);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aPos, theRadius, theStartAngle, theEndAngle));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Arc through three points, counterclockwise from theStart
//            through theMid to theEnd.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGePoint3d& theStart, const NvGePoint3d& theMid,
                                   const NvGePoint3d& theEnd)
{
  gp_Ax2 aPos;
  double aRadius = 0.0;
  double aSweep = 0.0;
  ArcDefOf3Points (gp_Pnt (theStart.x, theStart.y, theStart.z),
                   gp_Pnt (theMid.x, theMid.y, theMid.z),
                   gp_Pnt (theEnd.x, theEnd.y, theEnd.z),
                   aPos, aRadius, aSweep);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (CircleCurve (aPos, aRadius, 0.0, aSweep));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Arc through three points with error reporting; on error the
//            object is unchanged.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGePoint3d& theStart, const NvGePoint3d& theMid,
                                   const NvGePoint3d& theEnd, NvGeError& theError)
{
  theError = NvGe::kOk;
  if (theStart.isEqualTo (theMid))
  {
    theError = NvGe::kEqualArg1Arg2;
    return *this;
  }
  if (theStart.isEqualTo (theEnd))
  {
    theError = NvGe::kEqualArg1Arg3;
    return *this;
  }
  if (theMid.isEqualTo (theEnd))
  {
    theError = NvGe::kEqualArg2Arg3;
    return *this;
  }
  const gp_Vec aD21 (gp_Pnt (theStart.x, theStart.y, theStart.z),
                     gp_Pnt (theMid.x, theMid.y, theMid.z));
  const gp_Vec aD31 (gp_Pnt (theStart.x, theStart.y, theStart.z),
                     gp_Pnt (theEnd.x, theEnd.y, theEnd.z));
  if (aD21.Crossed (aD31).Magnitude()
      <= 1e-10 * aD21.Magnitude() * aD31.Magnitude())
  {
    theError = NvGe::kLinearlyDependentArg1Arg2Arg3;
    return *this;
  }
  return set (theStart, theMid, theEnd);
}

//=======================================================================
// function : set
// purpose  : Fillet of the given radius between two curves, tangent to
//            each nearest the seed parameters. The tangency points on the
//            line carriers anchor the arc orientation. The carriers must
//            be linear entities or circles and, when a circle takes part,
//            lie in one plane; otherwise success is reported false.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGeCurve3d& theCurve1,
                                   const NvGeCurve3d& theCurve2,
                                   double theRadius, double& theParam1, double& theParam2,
                                   Nova::Boolean& theSuccess)
{
  theSuccess = false;
  if (!(theRadius > 0.0))
  {
    throw NvException ("NvGeCircArc3d::set(): the fillet radius must be positive");
  }
  FilletLine aLine1;
  FilletLine aLine2;
  FilletCircle aCircle1;
  FilletCircle aCircle2;
  const bool aLine1Ok = FilletLineOf (theCurve1, aLine1);
  const bool aLine2Ok = FilletLineOf (theCurve2, aLine2);
  const bool aCircle1Ok = !aLine1Ok && FilletCircleOf (theCurve1, aCircle1);
  const bool aCircle2Ok = !aLine2Ok && FilletCircleOf (theCurve2, aCircle2);
  gp_Pnt aCenter;
  gp_Pnt aT1;
  gp_Pnt aT2;
  gp_Vec aStartTangent;
  if (aLine1Ok && aLine2Ok)
  {
    if (!FilletTwoLines (aLine1, theParam1, aLine2, theParam2, theRadius,
                         aCenter, aT1, aT2))
    {
      return *this;
    }
    aStartTangent = aLine1.direction;
    theParam1 = LineParamOf (aLine1, aT1);
    theParam2 = LineParamOf (aLine2, aT2);
  }
  else if (aLine1Ok && aCircle2Ok)
  {
    if (!FilletLineAndCircle (aLine1, theParam1, aCircle2, theParam2, theRadius,
                              aCenter, aT1, aT2))
    {
      return *this;
    }
    aStartTangent = aLine1.direction;
    theParam1 = LineParamOf (aLine1, aT1);
    theParam2 = PosAngle (aCircle2.frame, aT2);
  }
  else if (aCircle1Ok && aLine2Ok)
  {
    if (!FilletLineAndCircle (aLine2, theParam2, aCircle1, theParam1, theRadius,
                              aCenter, aT1, aT2))
    {
      return *this;
    }
    aStartTangent = aLine2.direction;
    theParam1 = PosAngle (aCircle1.frame, aT2);
    theParam2 = LineParamOf (aLine2, aT1);
  }
  else if (aCircle1Ok && aCircle2Ok)
  {
    if (!FilletTwoCircles (aCircle1, theParam1, aCircle2, theParam2, theRadius,
                           aCenter, aT1, aT2))
    {
      return *this;
    }
    const gp_Vec aRadial (aCircle1.center, aT1);
    aStartTangent = gp_Vec (aCircle1.frame.Direction().XYZ())
      .Crossed (aRadial.Multiplied (1.0 / aRadial.Magnitude()));
    theParam1 = PosAngle (aCircle1.frame, aT1);
    theParam2 = PosAngle (aCircle2.frame, aT2);
  }
  else
  {
    return *this;                                // unsupported carrier kinds
  }
  const occ::handle<Geom_Curve> aCurve = FilletArcCurve (aCenter, aT1, aT2,
                                                         theRadius, aStartTangent);
  if (aCurve.IsNull())
  {
    return *this;
  }
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (aCurve);
  theSuccess = true;
  return *this;
}

//=======================================================================
// function : set
// purpose  : Circle tangent to three line carriers: the incenter or one of
//            the three excenters of the triangle they form; the candidate
//            whose tangency points are nearest the seed parameters wins.
//            The radius follows from the triangle.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::set (const NvGeCurve3d& theCurve1,
                                   const NvGeCurve3d& theCurve2,
                                   const NvGeCurve3d& theCurve3,
                                   double& theParam1, double& theParam2,
                                   double& theParam3,
                                   Nova::Boolean& theSuccess)
{
  theSuccess = false;
  FilletLine aLines[3];
  if (!FilletLineOf (theCurve1, aLines[0])
      || !FilletLineOf (theCurve2, aLines[1])
      || !FilletLineOf (theCurve3, aLines[2]))
  {
    return *this;                                // only three lines are supported
  }
  // triangle vertices: aVertA opposite aLines[0], etc.
  gp_Pnt aVertA;
  gp_Pnt aVertB;
  gp_Pnt aVertC;
  if (!LineLineHit (aLines[1], aLines[2], aVertA)
      || !LineLineHit (aLines[2], aLines[0], aVertB)
      || !LineLineHit (aLines[0], aLines[1], aVertC))
  {
    return *this;                                // two of the carriers are parallel
  }
  const gp_Vec anEdge1 (aVertA, aVertB);
  const gp_Vec anEdge2 (aVertA, aVertC);
  if (anEdge1.Crossed (anEdge2).Magnitude()
      <= 1e-10 * anEdge1.Magnitude() * anEdge2.Magnitude())
  {
    return *this;                                // degenerate triangle
  }
  // incenter and excenters by barycentric weights
  const double aLenA = aVertB.Distance (aVertC);
  const double aLenB = aVertC.Distance (aVertA);
  const double aLenC = aVertA.Distance (aVertB);
  const double aWeights[4][3] = {
    {  aLenA,  aLenB,  aLenC },
    { -aLenA,  aLenB,  aLenC },
    {  aLenA, -aLenB,  aLenC },
    {  aLenA,  aLenB, -aLenC }
  };
  const gp_XYZ aSeeds[3] = {
    aLines[0].origin.XYZ() + aLines[0].direction.XYZ().Multiplied (theParam1),
    aLines[1].origin.XYZ() + aLines[1].direction.XYZ().Multiplied (theParam2),
    aLines[2].origin.XYZ() + aLines[2].direction.XYZ().Multiplied (theParam3)
  };
  bool aFound = false;
  double aBest = 0.0;
  gp_Pnt aBestCenter;
  gp_Pnt aBestT[3];
  for (int aMode = 0; aMode < 4; ++aMode)
  {
    const double aDenom = aWeights[aMode][0] + aWeights[aMode][1] + aWeights[aMode][2];
    if (std::abs (aDenom) <= gp::Resolution())
    {
      continue;
    }
    const gp_XYZ aCenter = (aVertA.XYZ().Multiplied (aWeights[aMode][0])
                          + aVertB.XYZ().Multiplied (aWeights[aMode][1])
                          + aVertC.XYZ().Multiplied (aWeights[aMode][2]))
                         .Multiplied (1.0 / aDenom);
    gp_Pnt aT[3];
    for (int aLine = 0; aLine < 3; ++aLine)
    {
      aT[aLine] = gp_Pnt (LineProjectionOf (aLines[aLine], aCenter));
    }
    const double aScore = (aT[0].XYZ() - aSeeds[0]).SquareModulus()
                        + (aT[1].XYZ() - aSeeds[1]).SquareModulus()
                        + (aT[2].XYZ() - aSeeds[2]).SquareModulus();
    if (!aFound || aScore < aBest)
    {
      aFound = true;
      aBest = aScore;
      aBestCenter = gp_Pnt (aCenter);
      for (int aLine = 0; aLine < 3; ++aLine)
      {
        aBestT[aLine] = aT[aLine];
      }
    }
  }
  if (!aFound)
  {
    return *this;
  }
  const double aRadius = aBestT[0].Distance (aBestCenter);
  if (!(aRadius > 0.0))
  {
    return *this;
  }
  const occ::handle<Geom_Curve> aCurve = FilletArcCurve (aBestCenter,
                                                         aBestT[0], aBestT[1],
                                                         aRadius, aLines[0].direction);
  if (aCurve.IsNull())
  {
    return *this;
  }
  theParam1 = LineParamOf (aLines[0], aBestT[0]);
  theParam2 = LineParamOf (aLines[1], aBestT[1]);
  theParam3 = LineParamOf (aLines[2], aBestT[2]);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (aCurve);
  theSuccess = true;
  return *this;
}

//=======================================================================
// function : operator =
// purpose  : Assignment; delegates the implementation sharing to the base.
//=======================================================================
NvGeCircArc3d& NvGeCircArc3d::operator = (const NvGeCircArc3d& theArc)
{
  NvGeEntity3d::operator = (theArc);
  return *this;
}
