#ifndef NV_GERAY2D_H
#define NV_GERAY2D_H

#include "gelent2d.h"
#pragma pack (push, 8)

class 

NvGeRay2d : public NvGeLinearEnt2d
{
public:
    GE_DLLEXPIMPORT NvGeRay2d();
    GE_DLLEXPIMPORT NvGeRay2d(const NvGeRay2d& line);
    GE_DLLEXPIMPORT NvGeRay2d(const NvGePoint2d& pnt, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeRay2d(const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeRay2d&     set         (const NvGePoint2d& pnt, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeRay2d&     set         (const NvGePoint2d& pnt1, const NvGePoint2d& pnt2);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeRay2d&     operator =  (const NvGeRay2d& line);
};

#pragma pack (pop)
#endif
