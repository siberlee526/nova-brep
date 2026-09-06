#ifndef NV_GEPENT3D_H
#define NV_GEPENT3D_H

#include "Nova.h"
#include "geent3d.h"
#pragma pack (push, 8)

class

NvGePointEnt3d : public NvGeEntity3d
{
public:
    // Return point coordinates.
    //
    GE_DLLEXPIMPORT NvGePoint3d     point3d     () const;

    // Conversion operator to convert to NvGePoint3d.
    //
    GE_DLLEXPIMPORT operator        NvGePoint3d () const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePointEnt3d& operator =  (const NvGePointEnt3d& pnt);

protected:
    GE_DLLEXPIMPORT NvGePointEnt3d ();
    GE_DLLEXPIMPORT NvGePointEnt3d (const NvGePointEnt3d&);
};

#pragma pack (pop)
#endif
