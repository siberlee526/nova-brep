#ifndef NV_GETOL_H
#define NV_GETOL_H

#ifndef unix
#include <stdlib.h>
#endif
#include "gedll.h"
#include "gedblar.h"
#pragma pack (push, 8)

class

NvGeTol {
public:
    GE_DLLEXPIMPORT NvGeTol();

    // Inquiry functions.
    //
    GE_DLLEXPIMPORT double  equalPoint          () const;
    GE_DLLEXPIMPORT double  equalVector         () const;

    // Set functions.
    //
    GE_DLLEXPIMPORT void    setEqualPoint       ( double val );
    GE_DLLEXPIMPORT void    setEqualVector      ( double val );

private:
    double				mTolArr[5];
    int                 mAbs;

    friend class NvGeTolA;
};

// Inlines for NvGeTol
//

inline void NvGeTol::setEqualVector( double val )
    { mTolArr[1] = val; }

inline double NvGeTol::equalVector() const
    { return mTolArr[1]; }

inline void NvGeTol::setEqualPoint( double val )
    { mTolArr[0] = val; }

inline double NvGeTol::equalPoint() const
    { return mTolArr[0]; }

#pragma pack (pop)
#endif
