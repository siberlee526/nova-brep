#ifndef NV_GEPOS2D_H
#define NV_GEPOS2D_H

#include "Nova.h"
#include "gepent2d.h"
#pragma pack (push, 8)

class

NvGePosition2d : public NvGePointEnt2d
{
public:
    GE_DLLEXPIMPORT NvGePosition2d ();
    GE_DLLEXPIMPORT NvGePosition2d (const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT NvGePosition2d (double x, double y);
    GE_DLLEXPIMPORT NvGePosition2d (const NvGePosition2d& pos);

    // Set point coordinates.
    //
    GE_DLLEXPIMPORT NvGePosition2d&  set        (const NvGePoint2d&);
    GE_DLLEXPIMPORT NvGePosition2d&  set        (double x, double y );

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePosition2d& operator =  (const NvGePosition2d& pos);
};

#pragma pack (pop)
#endif
