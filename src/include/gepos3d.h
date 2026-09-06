#ifndef NV_GEPOS3D_H
#define NV_GEPOS3D_H

#include "Nova.h"
#include "gepent3d.h"
#include "gepnt3d.h"
#pragma pack (push, 8)

class

NvGePosition3d : public NvGePointEnt3d
{
public:
    GE_DLLEXPIMPORT NvGePosition3d ();
    GE_DLLEXPIMPORT NvGePosition3d (const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT NvGePosition3d (double x, double y, double z);
    GE_DLLEXPIMPORT NvGePosition3d (const NvGePosition3d& pos);

    // Set point coordinates.
    //
    GE_DLLEXPIMPORT NvGePosition3d&  set        (const NvGePoint3d&);
    GE_DLLEXPIMPORT NvGePosition3d&  set        (double x, double y, double z );

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePosition3d& operator =  (const NvGePosition3d& pos);
};

#pragma pack (pop)
#endif
