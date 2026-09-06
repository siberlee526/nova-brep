// #ifndef NV_GEFILEIO_H
// #define NV_GEFILEIO_H     

// #include "acadstrc.h"
// #include "gegbl.h"
// #include "gedll.h"
// #include "gept2dar.h"
// #include "gevc2dar.h"
// #include "gept3dar.h"
// #include "gevc3dar.h"
// #include "gedblar.h"
// #include "geintarr.h"
// #pragma pack (push, 8)
     
// class NvGeEntity2d;
// class NvGeEntity3d;
// class NvGePoint2d;
// class NvGeVector2d;
// class NvGeMatrix2d;
// class NvGeScale2d;
// class NvGePoint3d;
// class NvGeVector3d;
// class NvGeMatrix3d;
// class NvGeScale3d;
// class NvGeTol;
// class NvGeInterval;
// class NvGeKnotVector;
// class NvGeCurveBoundary;
// class NvGePosition2d;
// class NvGePointOnCurve2d;
// class NvGeLine2d;
// class NvGeLineSeg2d;
// class NvGeRay2d;
// class NvGeCircArc2d;
// class NvGeEllipArc2d;
// class NvGeExternalCurve2d;
// class NvGeCubicSplineCurve2d;
// class NvGeCompositeCurve2d;
// class NvGeOffsetCurve2d;
// class NvGeNurbCurve2d;
// class NvGePolyline2d;
// class NvGePosition3d;
// class NvGePointOnCurve3d;
// class NvGePointOnSurface;
// class NvGeLine3d;
// class NvGeRay3d;
// class NvGeLineSeg3d;
// class NvGePlane;
// class NvGeBoundedPlane;
// class NvGeBoundBlock2d;
// class NvGeBoundBlock3d;
// class NvGeCircArc3d;
// class NvGeEllipArc3d;
// class NvGeCubicSplineCurve3d;
// class NvGeCompositeCurve3d;
// class NvGeOffsetCurve3d;
// class NvGeNurbCurve3d;
// class NvGePolyline3d;
// class NvGeAugPolyline3d;
// class NvGeExternalCurve3d;
// class NvGeSurface;
// class NvGeCone;
// class NvGeCylinder;
// class NvGeTorus;
// class NvGeExternalSurface;
// class NvGeOffsetSurface;
// class NvGeNurbSurface;
// class NvGeExternalBoundedSurface;
// class NvGeSphere;
// class NvGeCurveCurveInt2d;
// class NvGeCurveCurveInt3d;
// class NvGeEllipCone;
     
// class NvGeFiler;
// class NvGeLibVersion;
     
     
// class 
// NvGeFileIO
// {
// public:
     
//  // Write out to file
//  //
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePoint2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeVector2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeMatrix2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeScale2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePoint2dArray&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeVector2dArray&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePoint3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeVector3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeMatrix3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeScale3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePoint3dArray&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeVector3dArray&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeTol&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeInterval&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeKnotVector&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeDoubleArray&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler* filer, const NvGeIntArray& ent,
//                                 const NvGeLibVersion& version);
//  GX_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCurveBoundary&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePosition2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePointOnCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeLine2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeLineSeg2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeRay2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCircArc2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeEllipArc2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeExternalCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCubicSplineCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCompositeCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeOffsetCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeNurbCurve2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePolyline2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePosition3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePointOnCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePointOnSurface&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeLine3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeRay3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeLineSeg3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePlane&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeBoundedPlane&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCircArc3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeEllipArc3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCubicSplineCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCompositeCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeOffsetCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeNurbCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGePolyline3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeAugPolyline3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeExternalCurve3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCone&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCylinder&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeTorus&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeExternalSurface&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeOffsetSurface&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeNurbSurface&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeExternalBoundedSurface&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeSphere&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeBoundBlock2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeBoundBlock3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCurveCurveInt2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeCurveCurveInt3d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeLibVersion&);

//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus outFields(NvGeFiler*, const NvGeEllipCone&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
     
//  // Read in from file
//  //
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePoint2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeVector2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeMatrix2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeScale2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePoint2dArray&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeVector2dArray&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePoint3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeVector3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeMatrix3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeScale3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePoint3dArray&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeVector3dArray&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeTol&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeInterval&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeKnotVector&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeDoubleArray&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static
//  Acad::ErrorStatus inFields(NvGeFiler* filer, NvGeIntArray& ent,
//                                 const NvGeLibVersion& version);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCurveBoundary&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePosition2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePointOnCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeLine2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeLineSeg2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeRay2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCircArc2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeEllipArc2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeExternalCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCubicSplineCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCompositeCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeOffsetCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeNurbCurve2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePolyline2d&,
//                              const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePosition3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePointOnCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePointOnSurface&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeLine3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeRay3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeLineSeg3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePlane&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeBoundedPlane&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCircArc3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeEllipArc3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCubicSplineCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCompositeCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeOffsetCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeNurbCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGePolyline3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeAugPolyline3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeExternalCurve3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCone&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCylinder&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeTorus&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeExternalSurface&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeOffsetSurface&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeNurbSurface&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeExternalBoundedSurface&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);
//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeSphere&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeBoundBlock2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeBoundBlock3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCurveCurveInt2d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeCurveCurveInt3d&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  GX_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeEllipCone&,
//                             const NvGeLibVersion& = NvGe::gLibVersion);

//  // There is no version on a version object.
//  //
//  GE_DLLEXPIMPORT static 
//  Acad::ErrorStatus inFields(NvGeFiler*, NvGeLibVersion&);
     
// private:
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeBoolean(NvGeFiler*, Adesk::Boolean,
//                                 const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readBoolean(NvGeFiler*, Adesk::Boolean*,
//                                const NvGeLibVersion&);
//     static
//     Acad::ErrorStatus writeBool(NvGeFiler*, bool,
//                                 const NvGeLibVersion&);
//     static
//     Acad::ErrorStatus readBool(NvGeFiler*, bool*,
//                                const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeLong(NvGeFiler*, Adesk::Int32,
//                              const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readLong(NvGeFiler*, Adesk::Int32*,
//                             const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeDouble(NvGeFiler*, double,
//                              const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readDouble(NvGeFiler*, double*,
//                               const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writePoint2d(NvGeFiler*, const NvGePoint2d&,
//                                 const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readPoint2d(NvGeFiler*, NvGePoint2d*,
//                                const NvGeLibVersion&);
//     static
//     Acad::ErrorStatus writeVector2d(NvGeFiler*, const NvGeVector2d&,
//                                  const NvGeLibVersion&);
//     static
//     Acad::ErrorStatus readVector2d(NvGeFiler*, NvGeVector2d*,
//                                 const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writePoint3d(NvGeFiler*, const NvGePoint3d&,
//                                 const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readPoint3d(NvGeFiler*, NvGePoint3d*,
//                                const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeVector3d(NvGeFiler*, const NvGeVector3d&,
//                                  const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readVector3d(NvGeFiler*, NvGeVector3d*,
//                                 const NvGeLibVersion&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeAcGeSurface(NvGeFiler*, const NvGeSurface&,
//                                     const NvGeLibVersion& version);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readAcGeSurface(NvGeFiler*, NvGeSurface&,
//                                    const NvGeLibVersion& version);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeAcGeEntity2d(NvGeFiler* filer,
// 	            const NvGeEntity2d& ent, const NvGeLibVersion& version);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readAcGeEntity2d(NvGeFiler* filer, NvGeEntity2d*& ent,
// 	            NvGe::EntityId id, const NvGeLibVersion& version);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus writeAcGeEntity3d(NvGeFiler* filer,
// 	            const NvGeEntity3d& ent, const NvGeLibVersion& version);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus readAcGeEntity3d(NvGeFiler* filer, NvGeEntity3d*& ent,
// 	            NvGe::EntityId id, const NvGeLibVersion& version);

//     static
//     Acad::ErrorStatus writeBytes(NvGeFiler* filer, const void* buf,
//                                  Adesk::UInt32 len, const NvGeLibVersion& version);
//     static
//     Acad::ErrorStatus readBytes(NvGeFiler* filer, void* buf,
//                                 Adesk::UInt32 len, const NvGeLibVersion& version);

// 	friend class NvGeEllipArcParamOffset;

// };
     
// #pragma pack (pop)
// #endif
