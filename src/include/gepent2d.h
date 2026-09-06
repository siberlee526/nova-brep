#ifndef NV_GEPENT2D_H
#define NV_GEPENT2D_H

#include "Nova.h"
#include "geent2d.h"
#include "gepnt2d.h"
#pragma pack (push, 8)

class 

NvGePointEnt2d : public NvGeEntity2d
{
public:

    // Return point coordinates.
    //
    GE_DLLEXPIMPORT NvGePoint2d     point2d     () const;

    // Conversion operator to convert to NvGePoint2d.
    //
    GE_DLLEXPIMPORT operator        NvGePoint2d () const;
    
    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGePointEnt2d& operator =  (const NvGePointEnt2d& pnt);

protected:

    // Private constructors so that no object of this class can be instantiated.
    GE_DLLEXPIMPORT NvGePointEnt2d ();
    GE_DLLEXPIMPORT NvGePointEnt2d (const NvGePointEnt2d&);
};

#pragma pack (pop)
#endif
