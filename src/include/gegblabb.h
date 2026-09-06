#ifndef NV_GEGBLABB_H
#define NV_GEGBLABB_H

#include "gegbl.h"

const int eGood = NvGe::eGood;
const int eBad = NvGe::eBad;

typedef NvGe::EntityId EntityId;

const NvGe::EntityId kEntity2d = NvGe::kEntity2d;
const NvGe::EntityId kEntity3d = NvGe::kEntity3d;
const NvGe::EntityId kPointEnt2d = NvGe::kPointEnt2d;
const NvGe::EntityId kPointEnt3d = NvGe::kPointEnt3d;
const NvGe::EntityId kPosition2d = NvGe::kPosition2d;
const NvGe::EntityId kPosition3d = NvGe::kPosition3d;
const NvGe::EntityId kPointOnCurve2d = NvGe::kPointOnCurve2d;
const NvGe::EntityId kPointOnCurve3d = NvGe::kPointOnCurve3d;
const NvGe::EntityId kBoundedPlane = NvGe::kBoundedPlane;
const NvGe::EntityId kCircArc2d = NvGe::kCircArc2d;
const NvGe::EntityId kCircArc3d = NvGe::kCircArc3d;
const NvGe::EntityId kConic2d = NvGe::kConic2d;
const NvGe::EntityId kConic3d = NvGe::kConic3d;
const NvGe::EntityId kCurve2d = NvGe::kCurve2d;
const NvGe::EntityId kCurve3d = NvGe::kCurve3d;
const NvGe::EntityId kEllipArc2d = NvGe::kEllipArc2d;
const NvGe::EntityId kEllipArc3d = NvGe::kEllipArc3d;
const NvGe::EntityId kHelix = NvGe::kHelix;
const NvGe::EntityId kLine2d = NvGe::kLine2d;
const NvGe::EntityId kLine3d = NvGe::kLine3d;
const NvGe::EntityId kLinearEnt2d = NvGe::kLinearEnt2d;
const NvGe::EntityId kLinearEnt3d = NvGe::kLinearEnt3d;
const NvGe::EntityId kLineSeg2d = NvGe::kLineSeg2d;
const NvGe::EntityId kLineSeg3d = NvGe::kLineSeg3d;
const NvGe::EntityId kPlanarEnt = NvGe::kPlanarEnt;
const NvGe::EntityId kExternalCurve3d = NvGe::kExternalCurve3d;
const NvGe::EntityId kExternalCurve2d = NvGe::kExternalCurve2d;
const NvGe::EntityId kPlane = NvGe::kPlane;
const NvGe::EntityId kRay2d = NvGe::kRay2d;
const NvGe::EntityId kRay3d = NvGe::kRay3d;
const NvGe::EntityId kSurface = NvGe::kSurface;
const NvGe::EntityId kSphere = NvGe::kSphere;
const NvGe::EntityId kCone = NvGe::kCone;
const NvGe::EntityId kTorus = NvGe::kTorus;
const NvGe::EntityId kCylinder = NvGe::kCylinder;
const NvGe::EntityId kSplineEnt2d = NvGe::kSplineEnt2d;
const NvGe::EntityId kSurfaceCurve2dTo3d = NvGe::kSurfaceCurve2dTo3d;
const NvGe::EntityId kSurfaceCurve3dTo2d = NvGe::kSurfaceCurve3dTo2d;

const NvGe::EntityId kPolyline2d = NvGe::kPolyline2d;
const NvGe::EntityId kAugPolyline2d = NvGe::kAugPolyline2d;
const NvGe::EntityId kNurbCurve2d = NvGe::kNurbCurve2d;
const NvGe::EntityId kDSpline2d = NvGe::kDSpline2d;
const NvGe::EntityId kCubicSplineCurve2d = NvGe::kCubicSplineCurve2d;
const NvGe::EntityId kSplineEnt3d = NvGe::kSplineEnt3d;
const NvGe::EntityId kPolyline3d = NvGe::kPolyline3d;
const NvGe::EntityId kAugPolyline3d = NvGe::kAugPolyline3d;
const NvGe::EntityId kNurbCurve3d = NvGe::kNurbCurve3d;
const NvGe::EntityId kDSpline3d = NvGe::kDSpline3d;
const NvGe::EntityId kCubicSplineCurve3d = NvGe::kCubicSplineCurve3d;
const NvGe::EntityId kTrimmedCrv2d = NvGe::kTrimmedCrv2d;
const NvGe::EntityId kCompositeCrv2d = NvGe::kCompositeCrv2d;
const NvGe::EntityId kCompositeCrv3d = NvGe::kCompositeCrv3d;
const NvGe::EntityId kEnvelope2d = NvGe::kEnvelope2d;

const NvGe::EntityId kExternalSurface = NvGe::kExternalSurface;
const NvGe::EntityId kNurbSurface = NvGe::kNurbSurface;
const NvGe::EntityId kOffsetSurface = NvGe::kOffsetSurface;
const NvGe::EntityId kTrimmedSurface = NvGe::kTrimmedSurface;
const NvGe::EntityId kCurveBoundedSurface = NvGe::kCurveBoundedSurface;
const NvGe::EntityId kPointOnSurface = NvGe::kPointOnSurface;
const NvGe::EntityId kExternalBoundedSurface = NvGe::kExternalBoundedSurface;
const NvGe::EntityId kCurveCurveInt2d = NvGe::kCurveCurveInt2d;
const NvGe::EntityId kCurveCurveInt3d = NvGe::kCurveCurveInt3d;
const NvGe::EntityId kBoundBlock2d = NvGe::kBoundBlock2d;
const NvGe::EntityId kBoundBlock3d = NvGe::kBoundBlock3d;
const NvGe::EntityId kOffsetCurve2d = NvGe::kOffsetCurve2d;
const NvGe::EntityId kOffsetCurve3d = NvGe::kOffsetCurve3d;
const NvGe::EntityId kPolynomCurve3d = NvGe::kPolynomCurve3d;
const NvGe::EntityId kBezierCurve3d = NvGe::kBezierCurve3d;
const NvGe::EntityId kObject = NvGe::kObject;
const NvGe::EntityId kFitData3d = NvGe::kFitData3d;
const NvGe::EntityId kHatch = NvGe::kHatch;
const NvGe::EntityId kTrimmedCurve2d = NvGe::kTrimmedCurve2d;
const NvGe::EntityId kTrimmedCurve3d = NvGe::kTrimmedCurve3d;
const NvGe::EntityId kCurveSampleData = NvGe::kCurveSampleData;

const NvGe::EntityId kEllipCone = NvGe::kEllipCone;
const NvGe::EntityId kEllipCylinder = NvGe::kEllipCylinder;
const NvGe::EntityId kIntervalBoundBlock = NvGe::kIntervalBoundBlock;
const NvGe::EntityId kClipBoundary2d = NvGe::kClipBoundary2d;
const NvGe::EntityId kExternalObject = NvGe::kExternalObject;
const NvGe::EntityId kCurveSurfaceInt = NvGe::kCurveSurfaceInt;
const NvGe::EntityId kSurfaceSurfaceInt = NvGe::kSurfaceSurfaceInt;


typedef NvGe::ExternalEntityKind ExternalEntityKind;

const NvGe::ExternalEntityKind kAcisEntity = NvGe::kAcisEntity;
const NvGe::ExternalEntityKind kExternalEntityUndefined
                                = NvGe::kExternalEntityUndefined;

typedef NvGe::NurbSurfaceProperties NurbSurfaceProperties;

const NvGe::NurbSurfaceProperties kOpen = NvGe::kOpen;
const NvGe::NurbSurfaceProperties kClosed = NvGe::kClosed;
const NvGe::NurbSurfaceProperties kPeriodic = NvGe::kPeriodic;
const NvGe::NurbSurfaceProperties kRational = NvGe::kRational;
const NvGe::NurbSurfaceProperties kNoPoles= NvGe::kNoPoles;
const NvGe::NurbSurfaceProperties kPoleAtMin = NvGe::kPoleAtMin;
const NvGe::NurbSurfaceProperties kPoleAtMax = NvGe::kPoleAtMax;
const NvGe::NurbSurfaceProperties kPoleAtBoth = NvGe::kPoleAtBoth;

typedef NvGe::PointContainment PointContainment;

const NvGe::PointContainment kInside = NvGe::kInside;
const NvGe::PointContainment kOutside = NvGe::kOutside;
const NvGe::PointContainment kOnBoundary = NvGe::kOnBoundary;

typedef NvGe::NvGeXConfig NvGeXConfig;

const NvGe::NvGeXConfig kNotDefined = NvGe::kNotDefined;
const NvGe::NvGeXConfig kUnknown = NvGe::kUnknown;
const NvGe::NvGeXConfig kLeftRight = NvGe::kLeftRight;
const NvGe::NvGeXConfig kRightLeft = NvGe::kRightLeft;
const NvGe::NvGeXConfig kLeftLeft = NvGe::kLeftLeft;
const NvGe::NvGeXConfig kRightRight = NvGe::kRightRight;
const NvGe::NvGeXConfig kPointLeft = NvGe::kPointLeft;
const NvGe::NvGeXConfig kPointRight = NvGe::kPointRight;
const NvGe::NvGeXConfig kLeftOverlap = NvGe::kLeftOverlap;
const NvGe::NvGeXConfig kOverlapLeft = NvGe::kOverlapLeft;
const NvGe::NvGeXConfig kRightOverlap = NvGe::kRightOverlap;
const NvGe::NvGeXConfig kOverlapRight = NvGe::kOverlapRight;
const NvGe::NvGeXConfig kOverlapStart = NvGe::kOverlapStart;
const NvGe::NvGeXConfig kOverlapEnd = NvGe::kOverlapEnd;
const NvGe::NvGeXConfig kOverlapOverlap = NvGe::kOverlapOverlap;

typedef NvGe::ErrorCondition  NvGeError;
 
const NvGe::ErrorCondition	kOk = NvGe::kOk;
const NvGe::ErrorCondition	k0This = NvGe::k0This;
const NvGe::ErrorCondition	k0Arg1 = NvGe::k0Arg1;
const NvGe::ErrorCondition	k0Arg2 = NvGe::k0Arg2;
const NvGe::ErrorCondition	kPerpendicularArg1Arg2 = NvGe::kPerpendicularArg1Arg2;     
const NvGe::ErrorCondition	kEqualArg1Arg2 = NvGe::kEqualArg1Arg2;
const NvGe::ErrorCondition	kEqualArg1Arg3 = NvGe::kEqualArg1Arg3;
const NvGe::ErrorCondition	kEqualArg2Arg3 = NvGe::kEqualArg2Arg3;
const NvGe::ErrorCondition	kLinearlyDependentArg1Arg2Arg3 = NvGe::kLinearlyDependentArg1Arg2Arg3;
const NvGe::ErrorCondition	kArg1TooBig = NvGe::kArg1TooBig;
const NvGe::ErrorCondition	kArg1OnThis = NvGe::kArg1OnThis;
const NvGe::ErrorCondition	kArg1InsideThis = NvGe::kArg1InsideThis;

typedef NvGe::NvGeIntersectError NvGeIntersectError;

const NvGe::NvGeIntersectError kXXOk = NvGe::kXXOk;
const NvGe::NvGeIntersectError kXXIndexOutOfRange = NvGe::kXXIndexOutOfRange;
const NvGe::NvGeIntersectError kXXWrongDimensionAtIndex = NvGe::kXXWrongDimensionAtIndex;
const NvGe::NvGeIntersectError kXXUnknown = NvGe::kXXUnknown;

typedef NvGe::KnotParameterization KnotParameterization;

const NvGe::KnotParameterization kChord = NvGe::kChord;
const NvGe::KnotParameterization kSqrtChord = NvGe::kSqrtChord;
const NvGe::KnotParameterization kUniform = NvGe::kUniform;
const NvGe::KnotParameterization kCustomParameterization = NvGe::kCustomParameterization;
const NvGe::KnotParameterization kNotDefinedKnotParam = NvGe::kNotDefinedKnotParam;

#endif
