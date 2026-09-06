#ifndef NV_GEAPLN3D_H
#define NV_GEAPLN3D_H

#include "gecurv3d.h"
#include "gekvec.h"
#include "gept3dar.h"
#include "gevc3dar.h"
#include "gevec3d.h"
#include "gepnt3d.h"
#include "geplin3d.h"

#pragma pack (push, 8)

class 

NvGeAugPolyline3d : public NvGePolyline3d
{
public:
    GE_DLLEXPIMPORT NvGeAugPolyline3d();
    GE_DLLEXPIMPORT NvGeAugPolyline3d(const NvGeAugPolyline3d& apline);
    GE_DLLEXPIMPORT NvGeAugPolyline3d(const NvGeKnotVector& knots,
                      const NvGePoint3dArray& cntrlPnts,
                      const NvGeVector3dArray& vecBundle);
    GE_DLLEXPIMPORT NvGeAugPolyline3d(const NvGePoint3dArray& cntrlPnts,
                      const NvGeVector3dArray& vecBundle);

    // Approximation constructor
    //
    GE_DLLEXPIMPORT NvGeAugPolyline3d(const NvGeCurve3d& curve,
                      double fromParam, double toParam, 
		              double apprEps);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeAugPolyline3d& operator = (const NvGeAugPolyline3d& apline);

    // Points
    //
    GE_DLLEXPIMPORT NvGePoint3d           getPoint(int idx) const;
    GE_DLLEXPIMPORT NvGeAugPolyline3d&    setPoint(int idx, NvGePoint3d pnt);
    GE_DLLEXPIMPORT void                  getPoints(NvGePoint3dArray& pnts) const;

    // Tangent bundle 
    //
    GE_DLLEXPIMPORT NvGeVector3d          getVector(int idx) const;
    GE_DLLEXPIMPORT NvGeAugPolyline3d&    setVector(int idx, NvGeVector3d pnt);
    GE_DLLEXPIMPORT void                  getD1Vectors(NvGeVector3dArray& tangents) const;

    // D2 Tangent bundle 
    //
    GE_DLLEXPIMPORT NvGeVector3d          getD2Vector(int idx) const;
    GE_DLLEXPIMPORT NvGeAugPolyline3d&    setD2Vector(int idx, NvGeVector3d pnt);
    GE_DLLEXPIMPORT void                  getD2Vectors(NvGeVector3dArray& d2Vectors) const;

	// Approximation tolerance
	//
	GE_DLLEXPIMPORT double                approxTol      () const;
    GE_DLLEXPIMPORT NvGeAugPolyline3d&    setApproxTol   (double approxTol);

};

#pragma pack (pop)
#endif
