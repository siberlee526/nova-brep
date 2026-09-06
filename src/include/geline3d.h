#ifndef NV_GELINE3D_H
#define NV_GELINE3D_H

#include "gelent3d.h"
#pragma pack (push, 8)

class NvGeLine2d;

class
NvGeLine3d : public NvGeLinearEnt3d
{
public:
    GE_DLLEXPIMPORT NvGeLine3d();
    GE_DLLEXPIMPORT NvGeLine3d(const NvGeLine3d& line);
    GE_DLLEXPIMPORT NvGeLine3d(const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeLine3d(const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);

    // The x-axis, y-axis, and z-axis lines.
    //
    GE_DLLDATAEXIMP static const NvGeLine3d kXAxis;
    GE_DLLDATAEXIMP static const NvGeLine3d kYAxis;
    GE_DLLDATAEXIMP static const NvGeLine3d kZAxis;

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeLine3d& set(const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeLine3d& set(const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLine3d& operator = (const NvGeLine3d& line);
};

#pragma pack (pop)
#endif
