#ifndef NV_GERAY3D_H
#define NV_GERAY3D_H

#include "gelent3d.h"
#pragma pack (push, 8)

class NvGeRay2d;

class

NvGeRay3d : public NvGeLinearEnt3d
{
public:
    GE_DLLEXPIMPORT NvGeRay3d();
    GE_DLLEXPIMPORT NvGeRay3d(const NvGeRay3d& line);
    GE_DLLEXPIMPORT NvGeRay3d(const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeRay3d(const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);

    // Set methods.
    //
    GE_DLLEXPIMPORT NvGeRay3d&     set         (const NvGePoint3d& pnt, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeRay3d&     set         (const NvGePoint3d& pnt1, const NvGePoint3d& pnt2);

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeRay3d&     operator =  (const NvGeRay3d& line);
};

#pragma pack (pop)
#endif
