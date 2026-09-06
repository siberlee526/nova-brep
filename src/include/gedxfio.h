// #ifndef NV_GEDXFIO_H
// #define NV_GEDXFIO_H


// #include "gefileio.h"
// #include "gelibver.h"
// #pragma pack (push, 8)

// class AcDbDxfFiler;

// class
// NvGeDxfIO
// {
// public:

//     // Write to file
//     //
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePoint2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeVector2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeMatrix2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeScale2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePoint2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeVector2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePoint3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeVector3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeMatrix3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeScale3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePoint3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeVector3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeTol&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeInterval&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeKnotVector&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeDoubleArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeIntArray&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCurveBoundary&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePosition2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePointOnCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeLine2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeLineSeg2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeRay2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCircArc2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeEllipArc2d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeExternalCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCubicSplineCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeNurbCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCompositeCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeOffsetCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePolyline2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePosition3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePointOnCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePointOnSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeLine3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeRay3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeLineSeg3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeBoundedPlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCircArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeEllipArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCubicSplineCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeNurbCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCompositeCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeOffsetCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGePolyline3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeAugPolyline3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeExternalCurve3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCone&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCylinder&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeTorus&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeExternalSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeOffsetSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeNurbSurface&);
//     GE_DLLEXPIMPORT static
//    Acad::ErrorStatus outFields(AcDbDxfFiler*,const NvGeExternalBoundedSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeSphere&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeBoundBlock2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeBoundBlock3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCurveCurveInt2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeCurveCurveInt3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus outFields(AcDbDxfFiler*, const NvGeEllipCone&);

//     // Read from file
//     //
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePoint2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeVector2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeMatrix2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeScale2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePoint2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeVector2dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePoint3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeVector3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeMatrix3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeScale3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePoint3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeVector3dArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeTol&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeInterval&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeKnotVector&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeDoubleArray&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeIntArray&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCurveBoundary&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePosition2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePointOnCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeLine2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeLineSeg2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeRay2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCircArc2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeEllipArc2d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeExternalCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCubicSplineCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeNurbCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCompositeCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeOffsetCurve2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePolyline2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePosition3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePointOnCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePointOnSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeLine3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeRay3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeLineSeg3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeBoundedPlane&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCircArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeEllipArc3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCubicSplineCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeNurbCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCompositeCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeOffsetCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGePolyline3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeAugPolyline3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeExternalCurve3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCone&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCylinder&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeTorus&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeExternalSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeOffsetSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeNurbSurface&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeExternalBoundedSurface&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeSphere&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeBoundBlock2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeBoundBlock3d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCurveCurveInt2d&);
//     GE_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeCurveCurveInt3d&);
//     GX_DLLEXPIMPORT static
//     Acad::ErrorStatus inFields(AcDbDxfFiler*, NvGeEllipCone&);

//     GE_DLLDATAEXIMP static const NvGeLibVersion  NvGeDxfIOVersion;
// };


// #pragma pack (pop)
// #endif
