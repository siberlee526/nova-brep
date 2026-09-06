// geell2d.cpp - implementation of NvGeEllipArc2d based on OCCT
// Geom2d_Ellipse and Geom2d_TrimmedCurve.
//
// A full ellipse is stored as a plain Geom2d_Ellipse; any smaller arc is
// stored as a Geom2d_TrimmedCurve over a full ellipse basis. Angles are
// the ellipse parameter angles: a point on the ellipse is
//   C + majorRadius * cos (u) * majorAxis + minorRadius * sin (u) * minorAxis,
// so positive angles run from the major axis toward the minor axis. The
// sweep direction follows the axes: cross (majorAxis, minorAxis) > 0 is
// counter-clockwise. Equal input angles (mod 2*PI) denote a full ellipse.
//
// Clockwise arcs keep the user's indirect frame (cross (majorAxis,
// minorAxis) < 0) and store the complementary range ascending from the end
// angle to the raised start angle with Sense = true, so the stored curve
// covers exactly the requested clockwise sweep; the user angles are
// recovered by swapping the trimmed first/last parameters. Note that
// gp_Ax22d (P, Vx, Vy) keeps the cross-sign of the input axes, which is
// what encodes the direction; non-orthogonal input axes are silently
// orthogonalized by OCCT.
//
// OCCT requires majorRadius >= minorRadius > 0; violations are rejected
// with NvException before reaching OCCT.

#include <geell2d.h>

#include <NvException.h>
#include <gearc2d.h>
#include <gegblabb.h>
#include <geimpdata.h>
#include <gelent2d.h>
#include <gelnsg2d.h>
#include <gepnt2d.h>
#include <getol.h>
#include <gevec2d.h>

#include <Geom2d_Curve.hxx>
#include <Geom2d_Ellipse.hxx>
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

// Angular sweeps at or below this bound denote a full ellipse.
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

double PointDistance (const NvGePoint2d& thePnt1, const NvGePoint2d& thePnt2)
{
  const double aDx = thePnt2.x - thePnt1.x;
  const double aDy = thePnt2.y - thePnt1.y;
  return std::sqrt (aDx * aDx + aDy * aDy);
}

// Plain-data description of an elliptical arc. The axes are unit vectors;
// the sweep runs from the start angle to the end angle in the direction
// selected by the axes cross-sign.
struct EllDef
{
  NvGePoint2d  Center;
  NvGeVector2d MajorAxis = NvGeVector2d (1.0, 0.0);
  NvGeVector2d MinorAxis = NvGeVector2d (0.0, 1.0);
  double       MajorRadius = 0.0;
  double       MinorRadius = 0.0;
  double       StartAngle = 0.0;
  double       EndAngle = 0.0;
  bool         IsClockWise = false;
  bool         IsFullEllipse = false;
};

// Derives the direction from the axes, normalizes the angles into
// [0, THE_TWO_PI) and collapses a vanishing sweep into the full flag.
void NormalizeEllDef (EllDef& theDef)
{
  theDef.IsClockWise = (theDef.MajorAxis.x * theDef.MinorAxis.y
                      - theDef.MajorAxis.y * theDef.MinorAxis.x) < 0.0;
  theDef.StartAngle = NormalizeAngle (theDef.StartAngle);
  theDef.EndAngle = NormalizeAngle (theDef.EndAngle);
  if (theDef.IsFullEllipse)
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
    theDef.IsFullEllipse = true;
    theDef.StartAngle = 0.0;
    theDef.EndAngle = THE_TWO_PI;
  }
}

// True when theAngle lies on the arc sweep.
bool SweepContains (const EllDef& theDef, double theAngle, double theTol)
{
  if (theDef.IsFullEllipse)
  {
    return true;
  }
  const double aSweep = theDef.IsClockWise
    ? NormalizeAngle (theDef.StartAngle - theDef.EndAngle)
    : NormalizeAngle (theDef.EndAngle - theDef.StartAngle);
  const double aDelta = theDef.IsClockWise
    ? NormalizeAngle (theDef.StartAngle - theAngle)
    : NormalizeAngle (theAngle - theDef.StartAngle);
  return aDelta <= aSweep + theTol;
}

// Ellipse parameter angle of a point given as the radial vector from the
// center.
double AngleInFrame (const EllDef& theDef, const NvGeVector2d& theRadial)
{
  const double aU = (theRadial.x * theDef.MajorAxis.x + theRadial.y * theDef.MajorAxis.y)
                    / theDef.MajorRadius;
  const double aV = (theRadial.x * theDef.MinorAxis.x + theRadial.y * theDef.MinorAxis.y)
                    / theDef.MinorRadius;
  return std::atan2 (aV, aU);
}

NvGePoint2d PointAtAngle (const EllDef& theDef, double theAngle)
{
  const double aCos = std::cos (theAngle);
  const double aSin = std::sin (theAngle);
  return NvGePoint2d (theDef.Center.x
                        + theDef.MajorRadius * aCos * theDef.MajorAxis.x
                        + theDef.MinorRadius * aSin * theDef.MinorAxis.x,
                      theDef.Center.y
                        + theDef.MajorRadius * aCos * theDef.MajorAxis.y
                        + theDef.MinorRadius * aSin * theDef.MinorAxis.y);
}

// Builds the implementation geometry for theDef. Throws NvException for
// invalid radii or axes and for OCCT-side construction failures.
occ::handle<Geom2d_Curve> BuildGeometry (const EllDef& theDef, const char* theMethod)
{
  const std::string aContext = std::string ("NvGeEllipArc2d::") + theMethod + "()";
  if (theDef.MajorRadius <= gp::Resolution() || theDef.MinorRadius <= gp::Resolution())
  {
    throw NvException (aContext + ": the radii must be positive");
  }
  if (theDef.MinorRadius > theDef.MajorRadius)
  {
    throw NvException (aContext + ": the major radius must not be smaller"
                                  " than the minor radius");
  }
  const double aMajorLen = std::sqrt (theDef.MajorAxis.lengthSqrd());
  const double aMinorLen = std::sqrt (theDef.MinorAxis.lengthSqrd());
  if (aMajorLen <= gp::Resolution() || aMinorLen <= gp::Resolution())
  {
    throw NvException (aContext + ": the axis vectors are degenerate");
  }
  const double aCross = theDef.MajorAxis.x * theDef.MinorAxis.y
                      - theDef.MajorAxis.y * theDef.MinorAxis.x;
  if (std::abs (aCross) <= gp::Resolution() * std::max (aMajorLen, aMinorLen))
  {
    throw NvException (aContext + ": the axis vectors are parallel");
  }
  const gp_Ax22d anAxis (gp_Pnt2d (theDef.Center.x, theDef.Center.y),
                         gp_Dir2d (theDef.MajorAxis.x, theDef.MajorAxis.y),
                         gp_Dir2d (theDef.MinorAxis.x, theDef.MinorAxis.y));
  const occ::handle<Geom2d_Ellipse> anEllipse (new Geom2d_Ellipse (anAxis,
                                                                   theDef.MajorRadius,
                                                                   theDef.MinorRadius));
  if (theDef.IsFullEllipse)
  {
    return occ::handle<Geom2d_Curve> (anEllipse);
  }
  double aU1 = theDef.StartAngle;
  double aU2 = theDef.EndAngle;
  if (theDef.IsClockWise)
  {
    // The user's indirect (flipped-minor) frame is kept, so the stored
    // ascending range runs over the requested clockwise sweep; the angles
    // are recovered by swapping the trimmed bounds (see EllDefOf and the
    // banner).
    aU1 = theDef.EndAngle;
    aU2 = theDef.StartAngle;
  }
  try
  {
    return occ::handle<Geom2d_Curve> (new Geom2d_TrimmedCurve (anEllipse, aU1, aU2, true, true));
  }
  catch (const Standard_Failure&)
  {
    throw NvException (aContext + ": the arc angles do not define a valid sweep");
  }
}

// Recovers the plain-data description of the elliptical arc stored in
// theImp. Throws NvException when the geometry is not an ellipse.
EllDef EllDefOf (const NvGeImpEntity3d* theImp, const char* theMethod)
{
  const occ::handle<Geom2d_Curve> aCurve = NvGeCurve2dOf (theImp);
  const occ::handle<Geom2d_TrimmedCurve> aTrimmed = occ::down_cast<Geom2d_TrimmedCurve> (aCurve);
  occ::handle<Geom2d_Ellipse> anEllipse = aTrimmed.IsNull()
    ? occ::down_cast<Geom2d_Ellipse> (aCurve)
    : occ::down_cast<Geom2d_Ellipse> (aTrimmed->BasisCurve());
  if (anEllipse.IsNull())
  {
    throw NvException (std::string ("NvGeEllipArc2d::") + theMethod
                       + "(): the implementation geometry is not an ellipse");
  }
  EllDef aDef;
  const gp_Ax22d aPos = anEllipse->Position();
  const gp_Pnt2d aLoc = aPos.Location();
  aDef.Center = NvGePoint2d (aLoc.X(), aLoc.Y());
  aDef.MajorAxis = NvGeVector2d (aPos.XDirection().X(), aPos.XDirection().Y());
  aDef.MinorAxis = NvGeVector2d (aPos.YDirection().X(), aPos.YDirection().Y());
  aDef.MajorRadius = anEllipse->MajorRadius();
  aDef.MinorRadius = anEllipse->MinorRadius();
  const bool isDirect = aPos.XDirection().Crossed (aPos.YDirection()) > 0.0;
  if (aTrimmed.IsNull())
  {
    aDef.IsFullEllipse = true;
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
    // Clockwise arc: the indirect (flipped-minor) user frame is kept and
    // the stored range ascends from the end angle to the raised start
    // angle (see the banner).
    aDef.StartAngle = NormalizeAngle (aTrimmed->LastParameter());
    aDef.EndAngle = NormalizeAngle (aTrimmed->FirstParameter());
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

}

//=================================================================================================

NvGeEllipArc2d::NvGeEllipArc2d ()
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  EllDef aDef;
  aDef.MajorRadius = 1.0;
  aDef.MinorRadius = 1.0;
  aDef.IsFullEllipse = true;
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc2d, BuildGeometry (aDef, "NvGeEllipArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeEllipArc2d::NvGeEllipArc2d (const NvGeEllipArc2d& theSrc)
: NvGeCurve2d (theSrc)
{
}

//=================================================================================================

NvGeEllipArc2d::NvGeEllipArc2d (const NvGeCircArc2d& theArc)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  const bool isClockWise = theArc.isClockWise() == Adesk::kTrue;
  const NvGeVector2d aRef = theArc.refVec();
  EllDef aDef;
  aDef.Center = theArc.center();
  aDef.MajorAxis = aRef;
  // The minor axis encodes the direction through its cross-sign with the
  // major axis.
  aDef.MinorAxis = isClockWise ? NvGeVector2d (aRef.y, -aRef.x) : NvGeVector2d (-aRef.y, aRef.x);
  aDef.MajorRadius = theArc.radius();
  aDef.MinorRadius = theArc.radius();
  aDef.StartAngle = theArc.startAng();
  aDef.EndAngle = theArc.endAng();
  NormalizeEllDef (aDef);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc2d, BuildGeometry (aDef, "NvGeEllipArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeEllipArc2d::NvGeEllipArc2d (const NvGePoint2d& theCent, const NvGeVector2d& theMajorAxis,
                                const NvGeVector2d& theMinorAxis, double theMajorRadius,
                                double theMinorRadius)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  EllDef aDef;
  aDef.Center = theCent;
  aDef.MajorAxis = theMajorAxis;
  aDef.MinorAxis = theMinorAxis;
  aDef.MajorRadius = theMajorRadius;
  aDef.MinorRadius = theMinorRadius;
  NormalizeEllDef (aDef);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc2d, BuildGeometry (aDef, "NvGeEllipArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

NvGeEllipArc2d::NvGeEllipArc2d (const NvGePoint2d& theCent, const NvGeVector2d& theMajorAxis,
                                const NvGeVector2d& theMinorAxis, double theMajorRadius,
                                double theMinorRadius, double theStartAngle, double theEndAngle)
{
  if (mDelEnt && mpImpEnt != nullptr)
  {
    mpImpEnt->Unref();
  }
  EllDef aDef;
  aDef.Center = theCent;
  aDef.MajorAxis = theMajorAxis;
  aDef.MinorAxis = theMinorAxis;
  aDef.MajorRadius = theMajorRadius;
  aDef.MinorRadius = theMinorRadius;
  aDef.StartAngle = theStartAngle;
  aDef.EndAngle = theEndAngle;
  NormalizeEllDef (aDef);
  mpImpEnt = new NvGeImpEntity3d (NvGe::kEllipArc2d, BuildGeometry (aDef, "NvGeEllipArc2d"));
  mpImpEnt->Ref();
}

//=================================================================================================

Adesk::Boolean NvGeEllipArc2d::intersectWith (const NvGeLinearEnt2d& theLine, int& theIntn,
                                              NvGePoint2d& theP1, NvGePoint2d& theP2,
                                              const NvGeTol& theTol) const
{
  theIntn = 0;
  const EllDef aDef = EllDefOf (mpImpEnt, "intersectWith");
  const NvGePoint2d anAnchor = theLine.pointOnLine();
  const NvGeVector2d aDir = theLine.direction();
  const double aDirLen = std::sqrt (aDir.lengthSqrd());
  if (aDirLen <= gp::Resolution())
  {
    return false;
  }
  const NvGeVector2d aUnit (aDir.x / aDirLen, aDir.y / aDirLen);
  // Solve the conic quadratic in the line parameter: with u/v the frame
  // coordinates of a point, u^2 / a^2 + v^2 / b^2 = 1.
  const double anInvA2 = 1.0 / (aDef.MajorRadius * aDef.MajorRadius);
  const double anInvB2 = 1.0 / (aDef.MinorRadius * aDef.MinorRadius);
  const double aDu = aUnit.x * aDef.MajorAxis.x + aUnit.y * aDef.MajorAxis.y;
  const double aDv = aUnit.x * aDef.MinorAxis.x + aUnit.y * aDef.MinorAxis.y;
  const double aWu = (anAnchor.x - aDef.Center.x) * aDef.MajorAxis.x
                   + (anAnchor.y - aDef.Center.y) * aDef.MajorAxis.y;
  const double aWv = (anAnchor.x - aDef.Center.x) * aDef.MinorAxis.x
                   + (anAnchor.y - aDef.Center.y) * aDef.MinorAxis.y;
  const double anA = aDu * aDu * anInvA2 + aDv * aDv * anInvB2;
  const double aB = 2.0 * (aWu * aDu * anInvA2 + aWv * aDv * anInvB2);
  const double aC = aWu * aWu * anInvA2 + aWv * aWv * anInvB2 - 1.0;
  const double aDisc = aB * aB - 4.0 * anA * aC;
  if (aDisc < 0.0 || anA <= gp::Resolution())
  {
    return false;
  }
  const double aRoot = std::sqrt (aDisc);
  const double aParams[2] = { 0.5 * (-aB - aRoot) / anA, 0.5 * (-aB + aRoot) / anA };
  const int aRootCount = (aRoot > theTol.equalPoint() * std::max (anA, 1.0)) ? 2 : 1;
  double aLower = 0.0;
  double anUpper = 0.0;
  LinearBoundsOf (theLine, aLower, anUpper);
  const double anAngleTol = theTol.equalPoint()
    / std::max (std::min (aDef.MajorRadius, aDef.MinorRadius), gp::Resolution());
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

Adesk::Boolean NvGeEllipArc2d::isCircular (const NvGeTol& theTol) const
{
  const EllDef aDef = EllDefOf (mpImpEnt, "isCircular");
  return std::abs (aDef.MajorRadius - aDef.MinorRadius) <= theTol.equalPoint();
}

//=================================================================================================

Adesk::Boolean NvGeEllipArc2d::isInside (const NvGePoint2d& thePnt, const NvGeTol& theTol) const
{
  const EllDef aDef = EllDefOf (mpImpEnt, "isInside");
  const NvGeVector2d aRel (thePnt.x - aDef.Center.x, thePnt.y - aDef.Center.y);
  const double aU = (aRel.x * aDef.MajorAxis.x + aRel.y * aDef.MajorAxis.y) / aDef.MajorRadius;
  const double aV = (aRel.x * aDef.MinorAxis.x + aRel.y * aDef.MinorAxis.y) / aDef.MinorRadius;
  const double aValue = aU * aU + aV * aV;
  // The margin term converts the tolerance distance into the units of the
  // normalized quadratic form, which grows fastest at the minor axis.
  return 1.0 - aValue > 2.0 * theTol.equalPoint() / aDef.MinorRadius;
}

//=================================================================================================

NvGePoint2d NvGeEllipArc2d::center () const
{
  return EllDefOf (mpImpEnt, "center").Center;
}

//=================================================================================================

double NvGeEllipArc2d::minorRadius () const
{
  return EllDefOf (mpImpEnt, "minorRadius").MinorRadius;
}

//=================================================================================================

double NvGeEllipArc2d::majorRadius () const
{
  return EllDefOf (mpImpEnt, "majorRadius").MajorRadius;
}

//=================================================================================================

NvGeVector2d NvGeEllipArc2d::minorAxis () const
{
  return EllDefOf (mpImpEnt, "minorAxis").MinorAxis;
}

//=================================================================================================

NvGeVector2d NvGeEllipArc2d::majorAxis () const
{
  return EllDefOf (mpImpEnt, "majorAxis").MajorAxis;
}

//=================================================================================================

double NvGeEllipArc2d::startAng () const
{
  return EllDefOf (mpImpEnt, "startAng").StartAngle;
}

//=================================================================================================

double NvGeEllipArc2d::endAng () const
{
  return EllDefOf (mpImpEnt, "endAng").EndAngle;
}

//=================================================================================================

NvGePoint2d NvGeEllipArc2d::startPoint () const
{
  const EllDef aDef = EllDefOf (mpImpEnt, "startPoint");
  return PointAtAngle (aDef, aDef.StartAngle);
}

//=================================================================================================

NvGePoint2d NvGeEllipArc2d::endPoint () const
{
  const EllDef aDef = EllDefOf (mpImpEnt, "endPoint");
  return PointAtAngle (aDef, aDef.EndAngle);
}

//=================================================================================================

Adesk::Boolean NvGeEllipArc2d::isClockWise () const
{
  return EllDefOf (mpImpEnt, "isClockWise").IsClockWise;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::setCenter (const NvGePoint2d& theCent)
{
  EllDef aDef = EllDefOf (mpImpEnt, "setCenter");
  aDef.Center = theCent;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setCenter"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::setMinorRadius (double theRad)
{
  EllDef aDef = EllDefOf (mpImpEnt, "setMinorRadius");
  if (theRad <= gp::Resolution() || theRad > aDef.MajorRadius)
  {
    throw NvException ("NvGeEllipArc2d::setMinorRadius(): the minor radius must be positive"
                       " and must not exceed the major radius");
  }
  aDef.MinorRadius = theRad;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setMinorRadius"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::setMajorRadius (double theRad)
{
  EllDef aDef = EllDefOf (mpImpEnt, "setMajorRadius");
  if (theRad <= gp::Resolution() || theRad < aDef.MinorRadius)
  {
    throw NvException ("NvGeEllipArc2d::setMajorRadius(): the major radius must be positive"
                       " and must not be smaller than the minor radius");
  }
  aDef.MajorRadius = theRad;
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setMajorRadius"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::setAxes (const NvGeVector2d& theMajorAxis,
                                         const NvGeVector2d& theMinorAxis)
{
  EllDef aDef = EllDefOf (mpImpEnt, "setAxes");
  aDef.MajorAxis = theMajorAxis;
  aDef.MinorAxis = theMinorAxis;
  // NormalizeEllDef re-derives the direction from the new axes.
  NormalizeEllDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setAxes"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::setAngles (double theStartAngle, double theEndAngle)
{
  EllDef aDef = EllDefOf (mpImpEnt, "setAngles");
  aDef.StartAngle = theStartAngle;
  aDef.EndAngle = theEndAngle;
  aDef.IsFullEllipse = false;
  NormalizeEllDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "setAngles"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::set (const NvGePoint2d& theCent,
                                     const NvGeVector2d& theMajorAxis,
                                     const NvGeVector2d& theMinorAxis,
                                     double theMajorRadius, double theMinorRadius)
{
  EllDef aDef;
  aDef.Center = theCent;
  aDef.MajorAxis = theMajorAxis;
  aDef.MinorAxis = theMinorAxis;
  aDef.MajorRadius = theMajorRadius;
  aDef.MinorRadius = theMinorRadius;
  NormalizeEllDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::set (const NvGePoint2d& theCent,
                                     const NvGeVector2d& theMajorAxis,
                                     const NvGeVector2d& theMinorAxis,
                                     double theMajorRadius, double theMinorRadius,
                                     double theStartAngle, double theEndAngle)
{
  EllDef aDef;
  aDef.Center = theCent;
  aDef.MajorAxis = theMajorAxis;
  aDef.MinorAxis = theMinorAxis;
  aDef.MajorRadius = theMajorRadius;
  aDef.MinorRadius = theMinorRadius;
  aDef.StartAngle = theStartAngle;
  aDef.EndAngle = theEndAngle;
  NormalizeEllDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::set (const NvGeCircArc2d& theArc)
{
  const bool isClockWise = theArc.isClockWise() == Adesk::kTrue;
  const NvGeVector2d aRef = theArc.refVec();
  EllDef aDef;
  aDef.Center = theArc.center();
  aDef.MajorAxis = aRef;
  aDef.MinorAxis = isClockWise ? NvGeVector2d (aRef.y, -aRef.x) : NvGeVector2d (-aRef.y, aRef.x);
  aDef.MajorRadius = theArc.radius();
  aDef.MinorRadius = theArc.radius();
  aDef.StartAngle = theArc.startAng();
  aDef.EndAngle = theArc.endAng();
  NormalizeEllDef (aDef);
  ReplaceGeometry (mpImpEnt, BuildGeometry (aDef, "set"));
  return *this;
}

//=================================================================================================

NvGeEllipArc2d& NvGeEllipArc2d::operator = (const NvGeEllipArc2d& theEll)
{
  NvGeEntity2d::operator= (theEll);
  return *this;
}
