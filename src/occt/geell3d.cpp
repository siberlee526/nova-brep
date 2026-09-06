// geell3d.cpp - implementation of NvGeEllipArc3d.
//
// The entity stores a full ellipse as a Geom_Ellipse and an arc as a
// Geom_TrimmedCurve over a Geom_Ellipse. The parameter of the carrier
// ellipse is the angle in radians counted counterclockwise from the major
// axis of the ellipse placement. Stored angles are normalized so that
// startAng() lies in [0, 2pi) and the sweep lies in (0, 2pi]; a sweep equal
// to 2pi is stored as a full Geom_Ellipse.

#include <geell3d.h>
#include <Nova.h>
#include <NvException.h>
#include <gearc3d.h>
#include <gelent3d.h>
#include <geplanar.h>
#include <geplane.h>
#include <gepnt3d.h>
#include <getol.h>
#include <gevec3d.h>
#include <geimpdata.h>

#include <Geom_Ellipse.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp.hxx>

#include <cmath>

namespace
{

constexpr double THE_TWO_PI = 6.28318530717958647692;
constexpr double THE_PI = 3.14159265358979323846;

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

// Returns the carrier ellipse of the entity geometry, either as the stored
// curve itself or as the basis of a trimmed curve, and reports the trim
// state and its parameter range.
occ::handle<Geom_Ellipse> EllipseOf (const occ::handle<Standard_Transient>& theGeom,
                                     bool& theIsTrimmed, double& theFirst, double& theLast)
{
  theIsTrimmed = false;
  theFirst = 0.0;
  theLast = THE_TWO_PI;
  if (theGeom.IsNull())
  {
    throw NvException ("NvGeEllipArc3d: null curve data");
  }
  const occ::handle<Geom_Ellipse> anEllipse = occ::down_cast<Geom_Ellipse> (theGeom);
  if (!anEllipse.IsNull())
  {
    return anEllipse;
  }
  const occ::handle<Geom_TrimmedCurve> aTrim = occ::down_cast<Geom_TrimmedCurve> (theGeom);
  if (aTrim.IsNull())
  {
    throw NvException ("NvGeEllipArc3d: the entity does not store an ellipse");
  }
  const occ::handle<Geom_Ellipse> aBasis = occ::down_cast<Geom_Ellipse> (aTrim->BasisCurve());
  if (aBasis.IsNull())
  {
    throw NvException ("NvGeEllipArc3d: the entity does not store an ellipse");
  }
  theIsTrimmed = true;
  theFirst = aTrim->FirstParameter();
  theLast = aTrim->LastParameter();
  return aBasis;
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

// Validates the axis radii: major >= minor > 0.
void CheckRadii (double theMajor, double theMinor, const char* theMessage)
{
  if (!(theMinor > 0.0) || !(theMajor >= theMinor))
  {
    throw NvException (theMessage);
  }
}

// Builds the stored geometry: a full Geom_Ellipse for a sweep equal to 2pi
// and a Geom_TrimmedCurve otherwise. The start angle is normalized into
// [0, 2pi).
occ::handle<Geom_Curve> EllipseCurve (const gp_Ax2& thePos, double theMajor, double theMinor,
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
    return occ::handle<Geom_Curve> (new Geom_Ellipse (thePos, theMajor, theMinor));
  }
  return occ::handle<Geom_Curve> (new Geom_TrimmedCurve (
    occ::handle<Geom_Curve> (new Geom_Ellipse (thePos, theMajor, theMinor)),
    aStart, aStart + aSweep, true));
}

// Angle of the point around the placement, in (-pi, pi].
double AngleOf (const gp_Pnt& thePnt, const gp_Ax2& thePos)
{
  const gp_Vec aRadial (thePos.Location(), thePnt);
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  return std::atan2 (aRadial.Dot (anYDir), aRadial.Dot (anXDir));
}

// True when the point angle belongs to the stored arc span; a full ellipse
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

// Point of the carrier ellipse at the angle.
gp_Pnt EllipsePoint (const gp_Ax2& thePos, double theMajor, double theMinor, double theAngle)
{
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  const gp_Vec anOffset = anXDir.Multiplied (theMajor * std::cos (theAngle))
                         + anYDir.Multiplied (theMinor * std::sin (theAngle));
  return thePos.Location().Translated (anOffset);
}

// Normalized radius of the in-plane projection in the ellipse frame.
double NormalizedRadiusOf (const gp_Ax2& thePos, double theMajor, double theMinor,
                           const gp_Pnt& thePnt)
{
  const gp_Vec aRadial (thePos.Location(), thePnt);
  const gp_Vec aNormal (thePos.Direction().XYZ());
  const double anOutOfPlane = aRadial.Dot (aNormal);
  const gp_Vec anInPlane = aRadial.Subtracted (aNormal.Multiplied (anOutOfPlane));
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  return std::sqrt ((anInPlane.Dot (anXDir) * anInPlane.Dot (anXDir)) / (theMajor * theMajor)
                  + (anInPlane.Dot (anYDir) * anInPlane.Dot (anYDir)) / (theMinor * theMinor));
}

// True when the point lies on the carrier ellipse within the radial
// tolerance and, for an arc, within its angular span.
bool PointOnEllipse (const gp_Ax2& thePos, double theMajor, double theMinor,
                     bool theIsTrimmed, double theFirst, double theLast,
                     const gp_Pnt& thePnt, double theTol)
{
  const gp_Vec aRadial (thePos.Location(), thePnt);
  const gp_Vec aNormal (thePos.Direction().XYZ());
  const double anOutOfPlane = aRadial.Dot (aNormal);
  const gp_Vec anInPlane = aRadial.Subtracted (aNormal.Multiplied (anOutOfPlane));
  const double anInPlaneLen = anInPlane.Magnitude();
  const double aNormRad = NormalizedRadiusOf (thePos, theMajor, theMinor, thePnt);
  if (anInPlaneLen <= gp::Resolution() || aNormRad <= gp::Resolution())
  {
    return false;
  }
  const double aRadialDist = anInPlaneLen * std::abs (1.0 - 1.0 / aNormRad);
  if (aRadialDist > theTol)
  {
    return false;
  }
  if (!theIsTrimmed)
  {
    return true;
  }
  return AngleOnSpan (AngleOf (thePnt, thePos), theFirst, theLast, theTol);
}

// Intersections of the carrier ellipse with the in-plane line (thePnt,
// theDir); theDir must be unit. In the ellipse frame the line reads
// (x0 + s dx, y0 + s dy) against (x/a)^2 + (y/b)^2 = 1. Reports at most
// two arc-span hits.
int EllipseLineHits (const gp_Ax2& thePos, double theMajor, double theMinor,
                     bool theIsTrimmed, double theFirst, double theLast,
                     const gp_Pnt& thePnt, const gp_Vec& theDir,
                     double theTol, gp_Pnt theHits[2])
{
  const gp_Vec aW (thePos.Location(), thePnt);
  const gp_Vec anXDir (thePos.XDirection().XYZ());
  const gp_Vec anYDir (thePos.YDirection().XYZ());
  const double aX0 = aW.Dot (anXDir);
  const double aY0 = aW.Dot (anYDir);
  const double aDX = theDir.Dot (anXDir);
  const double aDY = theDir.Dot (anYDir);
  const double anA2 = theMajor * theMajor;
  const double aB2 = theMinor * theMinor;
  const double aQuad = (aDX * aDX) / anA2 + (aDY * aDY) / aB2;
  if (aQuad <= gp::Resolution())
  {
    return 0;
  }
  const double aLin = 2.0 * (aX0 * aDX / anA2 + aY0 * aDY / aB2);
  const double aConst = (aX0 * aX0) / anA2 + (aY0 * aY0) / aB2 - 1.0;
  const double aDisc = aLin * aLin - 4.0 * aQuad * aConst;
  if (aDisc < -theTol)
  {
    return 0;
  }
  const double aRoot = std::sqrt (std::max (aDisc, 0.0));
  int aCount = 0;
  for (int aBranch = 0; aBranch < 2; ++aBranch)
  {
    if (aBranch == 1 && aDisc <= theTol)
    {
      break;                                     // tangency: a single hit
    }
    const double aS = (-aLin + ((aBranch == 0) ? aRoot : -aRoot)) / (2.0 * aQuad);
    const gp_Pnt aHit = thePnt.Translated (theDir.Multiplied (aS));
    if (PointOnEllipse (thePos, theMajor, theMinor, theIsTrimmed, theFirst, theLast,
                        aHit, theTol))
    {
      theHits[aCount] = aHit;
      ++aCount;
    }
  }
  return aCount;
}

}

// NvGeEllipArc3d

//=======================================================================
// function : NvGeEllipArc3d
// purpose  : Unit circle about the origin in the XY plane.
//=======================================================================
NvGeEllipArc3d::NvGeEllipArc3d ()
{
  const occ::handle<Geom_Curve> aCurve = EllipseCurve (
    gp_Ax2 (gp_Pnt (0.0, 0.0, 0.0), gp_Dir (0.0, 0.0, 1.0)), 1.0, 1.0, 0.0, THE_TWO_PI);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeEllipArc3d
// purpose  : Copy constructor; shares the implementation of the source.
//=======================================================================
NvGeEllipArc3d::NvGeEllipArc3d (const NvGeEllipArc3d& theSrc)
: NvGeCurve3d (theSrc)
{

}

//=======================================================================
// function : NvGeEllipArc3d
// purpose  : Same carrier, axes and angles as the circle entity; the major
//            and the minor radius both equal the circle radius.
//=======================================================================
NvGeEllipArc3d::NvGeEllipArc3d (const NvGeCircArc3d& theArc)
{
  const gp_Ax2 aPos (gp_Pnt (theArc.center().x, theArc.center().y, theArc.center().z),
                     DirOf (theArc.normal(),
                            "NvGeEllipArc3d::NvGeEllipArc3d(): the source arc is degenerate"),
                     DirOf (theArc.refVec(),
                            "NvGeEllipArc3d::NvGeEllipArc3d(): the source arc is degenerate"));
  const double aRadius = theArc.radius();
  const occ::handle<Geom_Curve> aCurve = EllipseCurve (aPos, aRadius, aRadius,
                                                       theArc.startAng(), theArc.endAng());
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeEllipArc3d
// purpose  : Full ellipse about a center with the given axes and radii;
//            the plane normal is majorAxis x minorAxis.
//=======================================================================
NvGeEllipArc3d::NvGeEllipArc3d (const NvGePoint3d& theCent, const NvGeVector3d& theMajorAxis,
                                const NvGeVector3d& theMinorAxis, double theMajorRadius,
                                double theMinorRadius)
{
  CheckRadii (theMajorRadius, theMinorRadius,
              "NvGeEllipArc3d::NvGeEllipArc3d(): the radii must satisfy major >= minor > 0");
  const gp_Vec aMajor (theMajorAxis.x, theMajorAxis.y, theMajorAxis.z);
  const gp_Vec aMinor (theMinorAxis.x, theMinorAxis.y, theMinorAxis.z);
  if (aMajor.Magnitude() <= gp::Resolution() || aMinor.Magnitude() <= gp::Resolution()
      || aMajor.Crossed (aMinor).Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeEllipArc3d::NvGeEllipArc3d(): "
                       "the axes must be non-zero and non-parallel");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z),
                     gp_Dir (aMajor.Crossed (aMinor)), gp_Dir (aMajor));
  const occ::handle<Geom_Curve> aCurve = EllipseCurve (aPos, theMajorRadius,
                                                       theMinorRadius, 0.0, THE_TWO_PI);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : NvGeEllipArc3d
// purpose  : Arc of the ellipse; the sweep runs counterclockwise from
//            ang1 to ang2.
//=======================================================================
NvGeEllipArc3d::NvGeEllipArc3d (const NvGePoint3d& theCent, const NvGeVector3d& theMajorAxis,
                                const NvGeVector3d& theMinorAxis, double theMajorRadius,
                                double theMinorRadius, double theAng1, double theAng2)
{
  CheckRadii (theMajorRadius, theMinorRadius,
              "NvGeEllipArc3d::NvGeEllipArc3d(): the radii must satisfy major >= minor > 0");
  const gp_Vec aMajor (theMajorAxis.x, theMajorAxis.y, theMajorAxis.z);
  const gp_Vec aMinor (theMinorAxis.x, theMinorAxis.y, theMinorAxis.z);
  if (aMajor.Magnitude() <= gp::Resolution() || aMinor.Magnitude() <= gp::Resolution()
      || aMajor.Crossed (aMinor).Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeEllipArc3d::NvGeEllipArc3d(): "
                       "the axes must be non-zero and non-parallel");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z),
                     gp_Dir (aMajor.Crossed (aMinor)), gp_Dir (aMajor));
  const occ::handle<Geom_Curve> aCurve = EllipseCurve (aPos, theMajorRadius,
                                                       theMinorRadius, theAng1, theAng2);
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc3d, aCurve);
  mpImpEnt->Ref();
}

//=======================================================================
// function : closestPointToPlane
// purpose  : Point of the arc nearest the plane; the companion output is
//            its orthogonal projection onto the plane.
//=======================================================================
NvGePoint3d NvGeEllipArc3d::closestPointToPlane (const NvGePlanarEnt& thePlaneEnt,
                                                 NvGePoint3d& thePointOnPlane,
                                                 const NvGeTol& theTol) const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = anEllipse->Position();
  const gp_Vec aPlaneNormal = DirOf (thePlaneEnt.normal(),
    "NvGeEllipArc3d::closestPointToPlane(): the plane entity is degenerate");
  const NvGePoint3d aPlanePoint = thePlaneEnt.pointOnPlane();
  const gp_Pnt& aCenter = aPos.Location();
  const gp_Vec aCenterToPlane = gp_Vec (gp_XYZ (aCenter.XYZ())
    - gp_XYZ (aPlanePoint.x, aPlanePoint.y, aPlanePoint.z));
  const double aMajor = anEllipse->MajorRadius();
  const double aMinor = anEllipse->MinorRadius();
  const gp_Vec aPlanesCross = gp_Vec (aPos.Direction().XYZ()).Crossed (aPlaneNormal);
  if (aPlanesCross.Magnitude() <= 1e-10)
  {
    // parallel planes: every arc point shares the distance; report the start
    const gp_Pnt aStartPnt = anEllipse->EvalD0 (aFirst);
    const double aDist = aCenterToPlane.Dot (aPlaneNormal);
    thePointOnPlane = NvGePoint3d (aStartPnt.X() - aDist * aPlaneNormal.X(),
                                   aStartPnt.Y() - aDist * aPlaneNormal.Y(),
                                   aStartPnt.Z() - aDist * aPlaneNormal.Z());
    return NvGePoint3d (aStartPnt.X(), aStartPnt.Y(), aStartPnt.Z());
  }
  // in-plane signed distance of the carrier point at the angle a:
  //   f(a) = C0 + A cos a + B sin a
  const double aC0 = aCenterToPlane.Dot (aPlaneNormal);
  const double aA = aMajor * gp_Vec (aPos.XDirection().XYZ()).Dot (aPlaneNormal);
  const double aB = aMinor * gp_Vec (aPos.YDirection().XYZ()).Dot (aPlaneNormal);
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
  const gp_Pnt aBestPnt = EllipsePoint (aPos, aMajor, aMinor, aBestAngle);
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
Adesk::Boolean NvGeEllipArc3d::intersectWith (const NvGeLinearEnt3d& theLineEnt, int& theIntN,
                                              NvGePoint3d& theP1, NvGePoint3d& theP2,
                                              const NvGeTol& theTol) const
{
  theIntN = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const NvGePoint3d aLineOrigin = theLineEnt.pointOnLine();
  const NvGeVector3d aLineDirection = theLineEnt.direction();
  const gp_Pnt anOrigin (aLineOrigin.x, aLineOrigin.y, aLineOrigin.z);
  const gp_Vec aDir (aLineDirection.x, aLineDirection.y, aLineDirection.z);
  gp_Pnt aHits[2];
  const int aNbHits = EllipseLineHits (anEllipse->Position(), anEllipse->MajorRadius(),
                                       anEllipse->MinorRadius(), aTrimmed,
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
// purpose  : Intersections with the plane, i.e. with the line where the
//            ellipse plane and the given plane meet.
//=======================================================================
Adesk::Boolean NvGeEllipArc3d::intersectWith (const NvGePlanarEnt& thePlaneEnt,
                                              int& theNumOfIntersect,
                                              NvGePoint3d& theP1, NvGePoint3d& theP2,
                                              const NvGeTol& theTol) const
{
  theNumOfIntersect = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = anEllipse->Position();
  const gp_Vec aPlaneNormal = DirOf (thePlaneEnt.normal(),
    "NvGeEllipArc3d::intersectWith(): the plane entity is degenerate");
  const NvGePoint3d aPlanePoint = thePlaneEnt.pointOnPlane();
  const gp_Pnt& aCenter = aPos.Location();
  const gp_Vec aPlanesCross = gp_Vec (aPos.Direction().XYZ()).Crossed (aPlaneNormal);
  if (aPlanesCross.Magnitude() <= 1e-10)
  {
    return false;           // parallel planes: the arc misses or lies in the plane
  }
  const gp_Vec aLineDir = aPlanesCross.Multiplied (1.0 / aPlanesCross.Magnitude());
  const gp_Vec anInPlane = gp_Vec (aPos.Direction().XYZ()).Crossed (aLineDir);
  const gp_Vec aCenterToPlane (aCenter, gp_Pnt (aPlanePoint.x, aPlanePoint.y,
                                                aPlanePoint.z));
  const double anOffset = aCenterToPlane.Dot (aPlaneNormal)
                        / anInPlane.Dot (aPlaneNormal);
  const gp_Pnt aLinePoint = aCenter.Translated (anInPlane.Multiplied (anOffset));
  gp_Pnt aHits[2];
  const int aNbHits = EllipseLineHits (aPos, anEllipse->MajorRadius(),
                                       anEllipse->MinorRadius(), aTrimmed,
                                       aFirst, aLast, aLinePoint, aLineDir,
                                       theTol.equalPoint(), aHits);
  int aCount = 0;
  for (int aHit = 0; aHit < aNbHits; ++aHit)
  {
    const NvGePoint3d aPnt (aHits[aHit].X(), aHits[aHit].Y(), aHits[aHit].Z());
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
// purpose  : Intersections of the arc with the line projected onto the
//            ellipse plane along theProjDir; reports the paired points on
//            both.
//=======================================================================
Adesk::Boolean NvGeEllipArc3d::projIntersectWith (const NvGeLinearEnt3d& theLineEnt,
                                                  const NvGeVector3d& theProjDir,
                                                  int& theNumInt,
                                                  NvGePoint3d& thePntOnEllipse1,
                                                  NvGePoint3d& thePntOnEllipse2,
                                                  NvGePoint3d& thePntOnLine1,
                                                  NvGePoint3d& thePntOnLine2,
                                                  const NvGeTol& theTol) const
{
  theNumInt = 0;
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = anEllipse->Position();
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
    return false;               // projection parallel to the ellipse plane
  }
  const NvGePoint3d aLineOrigin = theLineEnt.pointOnLine();
  const NvGeVector3d aLineDirection = theLineEnt.direction();
  const gp_Pnt anOrigin (aLineOrigin.x, aLineOrigin.y, aLineOrigin.z);
  const gp_Vec aDir (aLineDirection.x, aLineDirection.y, aLineDirection.z);
  // the line projected onto the ellipse plane along aProj
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
    if (!PointOnEllipse (aPos, anEllipse->MajorRadius(), anEllipse->MinorRadius(),
                         aTrimmed, aFirst, aLast, aProjOrigin, theTol.equalPoint()))
    {
      return false;
    }
    const gp_Pnt aLineHit = anOrigin.Translated (aDir.Multiplied (
      gp_Vec (anOrigin, aProjOrigin).Dot (aDir)));
    const NvGePoint3d aLinePnt (aLineHit.X(), aLineHit.Y(), aLineHit.Z());
    if (theLineEnt.isOn (aLinePnt, theTol))
    {
      thePntOnEllipse1 = NvGePoint3d (aProjOrigin.X(), aProjOrigin.Y(), aProjOrigin.Z());
      thePntOnLine1 = aLinePnt;
      aAccepted = 1;
    }
  }
  else
  {
    const gp_Vec aDirUnit = aProjDirRaw.Multiplied (1.0 / aProjDirRaw.Magnitude());
    gp_Pnt aHits[2];
    const int aNbHits = EllipseLineHits (aPos, anEllipse->MajorRadius(),
                                         anEllipse->MinorRadius(), aTrimmed,
                                         aFirst, aLast, aProjOrigin, aDirUnit,
                                         theTol.equalPoint(), aHits);
    // map an ellipse point back to the original line:
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
      const NvGePoint3d anEllipsePnt (aHits[aHit].X(), aHits[aHit].Y(), aHits[aHit].Z());
      if (aAccepted == 0)
      {
        thePntOnEllipse1 = anEllipsePnt;
        thePntOnLine1 = aLinePnt;
      }
      else
      {
        thePntOnEllipse2 = anEllipsePnt;
        thePntOnLine2 = aLinePnt;
      }
      ++aAccepted;
    }
  }
  theNumInt = aAccepted;
  return aAccepted > 0;
}

//=======================================================================
// function : getPlane
// purpose  : Plane of the carrier ellipse.
//=======================================================================
void NvGeEllipArc3d::getPlane (NvGePlane& thePlane) const
{
  thePlane = NvGePlane (center(), normal());
}

//=======================================================================
// function : isCircular
// purpose  : True when the radii coincide within the point tolerance.
//=======================================================================
Adesk::Boolean NvGeEllipArc3d::isCircular (const NvGeTol& theTol) const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  return std::abs (anEllipse->MajorRadius() - anEllipse->MinorRadius())
         <= theTol.equalPoint();
}

//=======================================================================
// function : isInside
// purpose  : Strict interior test against the full carrier ellipse, using
//            the in-plane radial distance to the boundary.
//=======================================================================
Adesk::Boolean NvGeEllipArc3d::isInside (const NvGePoint3d& thePnt,
                                         const NvGeTol& theTol) const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Ax2& aPos = anEllipse->Position();
  const gp_Vec aRadial (aPos.Location(), gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  const gp_Vec aNormal (aPos.Direction().XYZ());
  const double anOutOfPlane = aRadial.Dot (aNormal);
  const gp_Vec anInPlane = aRadial.Subtracted (aNormal.Multiplied (anOutOfPlane));
  const double anInPlaneLen = anInPlane.Magnitude();
  if (anInPlaneLen <= gp::Resolution())
  {
    return true;                                 // the center is interior
  }
  const double aNormRad = NormalizedRadiusOf (aPos, anEllipse->MajorRadius(),
                                              anEllipse->MinorRadius(),
                                              gp_Pnt (thePnt.x, thePnt.y, thePnt.z));
  if (aNormRad >= 1.0)
  {
    return false;
  }
  const double aRadialDist = anInPlaneLen * std::abs (1.0 - 1.0 / aNormRad);
  return aRadialDist > theTol.equalPoint();
}

//=======================================================================
// function : center
// purpose  : Center of the carrier ellipse.
//=======================================================================
NvGePoint3d NvGeEllipArc3d::center () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Pnt& aCenter = anEllipse->Position().Location();
  return NvGePoint3d (aCenter.X(), aCenter.Y(), aCenter.Z());
}

//=======================================================================
// function : minorRadius
// purpose  : Minor radius of the carrier ellipse.
//=======================================================================
double NvGeEllipArc3d::minorRadius () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  return anEllipse->MinorRadius();
}

//=======================================================================
// function : majorRadius
// purpose  : Major radius of the carrier ellipse.
//=======================================================================
double NvGeEllipArc3d::majorRadius () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  return anEllipse->MajorRadius();
}

//=======================================================================
// function : minorAxis
// purpose  : Direction of the minor axis.
//=======================================================================
NvGeVector3d NvGeEllipArc3d::minorAxis () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Dir& anYDir = anEllipse->Position().YDirection();
  return NvGeVector3d (anYDir.X(), anYDir.Y(), anYDir.Z());
}

//=======================================================================
// function : majorAxis
// purpose  : Direction of the major axis; the zero angle starts here.
//=======================================================================
NvGeVector3d NvGeEllipArc3d::majorAxis () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Dir& anXDir = anEllipse->Position().XDirection();
  return NvGeVector3d (anXDir.X(), anXDir.Y(), anXDir.Z());
}

//=======================================================================
// function : normal
// purpose  : Normal of the ellipse plane.
//=======================================================================
NvGeVector3d NvGeEllipArc3d::normal () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Dir& aNormal = anEllipse->Position().Direction();
  return NvGeVector3d (aNormal.X(), aNormal.Y(), aNormal.Z());
}

//=======================================================================
// function : startAng
// purpose  : Start angle; zero for a full ellipse.
//=======================================================================
double NvGeEllipArc3d::startAng () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  return aFirst;
}

//=======================================================================
// function : endAng
// purpose  : End angle; 2pi for a full ellipse.
//=======================================================================
double NvGeEllipArc3d::endAng () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  return aLast;
}

//=======================================================================
// function : startPoint
// purpose  : Point at the start angle.
//=======================================================================
NvGePoint3d NvGeEllipArc3d::startPoint () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Pnt aPnt = anEllipse->EvalD0 (aFirst);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : endPoint
// purpose  : Point at the end angle.
//=======================================================================
NvGePoint3d NvGeEllipArc3d::endPoint () const
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Pnt aPnt = anEllipse->EvalD0 (aLast);
  return NvGePoint3d (aPnt.X(), aPnt.Y(), aPnt.Z());
}

//=======================================================================
// function : setCenter
// purpose  : Moves the carrier center, keeping the shape and the angles.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::setCenter (const NvGePoint3d& theCent)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  gp_Ax2 aPos = anEllipse->Position();
  aPos.SetLocation (gp_Pnt (theCent.x, theCent.y, theCent.z));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (aPos, anEllipse->MajorRadius(),
                                   anEllipse->MinorRadius(), aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setMinorRadius
// purpose  : Rescales the minor radius, keeping the rest of the definition.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::setMinorRadius (double theRad)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  CheckRadii (anEllipse->MajorRadius(), theRad,
              "NvGeEllipArc3d::setMinorRadius(): the radii must satisfy major >= minor > 0");
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (anEllipse->Position(), anEllipse->MajorRadius(),
                                   theRad, aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setMajorRadius
// purpose  : Rescales the major radius, keeping the rest of the definition.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::setMajorRadius (double theRad)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  CheckRadii (theRad, anEllipse->MinorRadius(),
              "NvGeEllipArc3d::setMajorRadius(): the radii must satisfy major >= minor > 0");
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (anEllipse->Position(), theRad,
                                   anEllipse->MinorRadius(), aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setAxes
// purpose  : Redefines the axes; the plane normal becomes
//            majorAxis x minorAxis, the center and the angles are kept.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::setAxes (const NvGeVector3d& theMajorAxis,
                                         const NvGeVector3d& theMinorAxis)
{
  const gp_Vec aMajor = gp_Vec (DirOf (theMajorAxis,
    "NvGeEllipArc3d::setAxes(): the major axis must be non-zero"));
  const gp_Vec aMinor = gp_Vec (DirOf (theMinorAxis,
    "NvGeEllipArc3d::setAxes(): the minor axis must be non-zero"));
  if (aMajor.Crossed (aMinor).Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeEllipArc3d::setAxes(): the axes must not be parallel");
  }
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  const gp_Ax2 aPos (anEllipse->Position().Location(),
                     gp_Dir (aMajor.Crossed (aMinor)), gp_Dir (aMajor));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (aPos, anEllipse->MajorRadius(),
                                   anEllipse->MinorRadius(), aFirst, aLast));
  return *this;
}

//=======================================================================
// function : setAngles
// purpose  : Retrimps the carrier; equal angles give the full ellipse and
//            a negative sweep is measured counterclockwise across zero.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::setAngles (double theStartAngle, double theEndAngle)
{
  bool aTrimmed;
  double aFirst, aLast;
  const occ::handle<Geom_Ellipse> anEllipse = EllipseOf (mpImpEnt->Geom(),
                                                         aTrimmed, aFirst, aLast);
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (anEllipse->Position(), anEllipse->MajorRadius(),
                                   anEllipse->MinorRadius(), theStartAngle, theEndAngle));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Full ellipse about a center with the given axes and radii.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::set (const NvGePoint3d& theCent,
                                     const NvGeVector3d& theMajorAxis,
                                     const NvGeVector3d& theMinorAxis,
                                     double theMajorRadius, double theMinorRadius)
{
  CheckRadii (theMajorRadius, theMinorRadius,
              "NvGeEllipArc3d::set(): the radii must satisfy major >= minor > 0");
  const gp_Vec aMajor = gp_Vec (DirOf (theMajorAxis,
    "NvGeEllipArc3d::set(): the major axis must be non-zero"));
  const gp_Vec aMinor = gp_Vec (DirOf (theMinorAxis,
    "NvGeEllipArc3d::set(): the minor axis must be non-zero"));
  if (aMajor.Crossed (aMinor).Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeEllipArc3d::set(): the axes must not be parallel");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z),
                     gp_Dir (aMajor.Crossed (aMinor)), gp_Dir (aMajor));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (aPos, theMajorRadius, theMinorRadius,
                                   0.0, THE_TWO_PI));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Arc of the ellipse; the sweep runs counterclockwise from
//            startAngle to endAngle.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::set (const NvGePoint3d& theCent,
                                     const NvGeVector3d& theMajorAxis,
                                     const NvGeVector3d& theMinorAxis,
                                     double theMajorRadius, double theMinorRadius,
                                     double theStartAngle, double theEndAngle)
{
  CheckRadii (theMajorRadius, theMinorRadius,
              "NvGeEllipArc3d::set(): the radii must satisfy major >= minor > 0");
  const gp_Vec aMajor = gp_Vec (DirOf (theMajorAxis,
    "NvGeEllipArc3d::set(): the major axis must be non-zero"));
  const gp_Vec aMinor = gp_Vec (DirOf (theMinorAxis,
    "NvGeEllipArc3d::set(): the minor axis must be non-zero"));
  if (aMajor.Crossed (aMinor).Magnitude() <= gp::Resolution())
  {
    throw NvException ("NvGeEllipArc3d::set(): the axes must not be parallel");
  }
  const gp_Ax2 aPos (gp_Pnt (theCent.x, theCent.y, theCent.z),
                     gp_Dir (aMajor.Crossed (aMinor)), gp_Dir (aMajor));
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (aPos, theMajorRadius, theMinorRadius,
                                   theStartAngle, theEndAngle));
  return *this;
}

//=======================================================================
// function : set
// purpose  : Same carrier, axes and angles as the circle entity.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::set (const NvGeCircArc3d& theArc)
{
  const gp_Ax2 aPos (gp_Pnt (theArc.center().x, theArc.center().y, theArc.center().z),
                     DirOf (theArc.normal(),
                            "NvGeEllipArc3d::set(): the source arc is degenerate"),
                     DirOf (theArc.refVec(),
                            "NvGeEllipArc3d::set(): the source arc is degenerate"));
  const double aRadius = theArc.radius();
  if (mpImpEnt->RefCount() > 1)
  {
    mpImpEnt->Unref();
    mpImpEnt = mpImpEnt->CloneShallow();
  }
  mpImpEnt->SetGeom (EllipseCurve (aPos, aRadius, aRadius,
                                   theArc.startAng(), theArc.endAng()));
  return *this;
}

//=======================================================================
// function : operator =
// purpose  : Assignment; delegates the implementation sharing to the base.
//=======================================================================
NvGeEllipArc3d& NvGeEllipArc3d::operator = (const NvGeEllipArc3d& theEll)
{
  NvGeEntity3d::operator = (theEll);
  return *this;
}
