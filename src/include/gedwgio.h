// #ifndef NV_GEDWGIO_H
// #define NV_GEDWGIO_H

// #include "gefileio.h"
// #include "gelibver.h"
// #pragma pack (push, 8)

// class AcDbDwgFiler;

// class
// NvGeDwgIO
// {
// public:

//     // Write out to file
//     //
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePoint2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeVector2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeMatrix2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeScale2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePoint2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeVector2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePoint3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeVector3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeMatrix3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeScale3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePoint3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeVector3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeTol&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeInterval&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeKnotVector&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeDoubleArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeIntArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCurveBoundary&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePosition2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePointOnCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeLine2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeLineSeg2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeRay2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCircArc2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeEllipArc2d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeExternalCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCubicSplineCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeNurbCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCompositeCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeOffsetCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePolyline2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePosition3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePointOnCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePointOnSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeLine3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeRay3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeLineSeg3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeBoundedPlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCircArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeEllipArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCubicSplineCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeNurbCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCompositeCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeOffsetCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGePolyline3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeAugPolyline3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeExternalCurve3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCone&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCylinder&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeTorus&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeExternalSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeOffsetSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeNurbSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*,const NvGeExternalBoundedSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeSphere&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeBoundBlock2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeBoundBlock3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCurveCurveInt2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeCurveCurveInt3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDwgFiler*, const NvGeEllipCone&);

//     // Read in from file
//     //
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePoint2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeVector2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeMatrix2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeScale2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePoint2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeVector2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePoint3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeVector3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeMatrix3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeScale3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePoint3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeVector3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeTol&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeInterval&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeKnotVector&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeDoubleArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeIntArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCurveBoundary&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePosition2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePointOnCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeLine2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeLineSeg2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeRay2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCircArc2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeEllipArc2d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeExternalCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCubicSplineCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeNurbCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCompositeCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeOffsetCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePolyline2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePosition3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePointOnCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePointOnSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeLine3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeRay3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeLineSeg3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeBoundedPlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCircArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeEllipArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCubicSplineCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCompositeCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeOffsetCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeNurbCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGePolyline3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeAugPolyline3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeExternalCurve3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCone&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCylinder&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeTorus&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeExternalSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeOffsetSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeNurbSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeExternalBoundedSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeSphere&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeBoundBlock2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeBoundBlock3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCurveCurveInt2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeCurveCurveInt3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDwgFiler*, NvGeEllipCone&);

//     GE_DLLDATAEXIMP static const NvGeLibVersion  NvGeDwgIOVersion;
// };


// #pragma pack (pop)
// #endif
