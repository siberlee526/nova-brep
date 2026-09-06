#ifndef NV_GELINE2D_H
#define NV_GELINE2D_H

#include "gelent2d.h"
#pragma pack (push, 8)

class
NvGeLine2d : public NvGeLinearEnt2d
{
public:
    GE_DLLEXPIMPORT NvGeLine2d();
    GE_DLLEXPIMPORT NvGeLine2d(const NvGeLine2d& line);
    GE_DLLEXPIMPORT NvGeLine2d(const NvGePoint2d& pnt, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeLine2d(const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);

    // The x-axis and y-axis lines.
    //
    GE_DLLDATAEXIMP static const NvGeLine2d kXAxis;
    GE_DLLDATAEXIMP static const NvGeLine2d kYAxis;

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeLine2d& set (const NvGePoint2d& pnt, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeLine2d& set (const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeLine2d& operator = (const NvGeLine2d& line);
};

#pragma pack (pop)
#endif
