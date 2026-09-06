#ifndef NV_GECBNDRY_H
#define NV_GECBNDRY_H

#include "gegbl.h"
#pragma pack (push, 8)

class NvGeCurve2d;
class NvGeEntity3d;
class NvGePosition3d;
class NvGeImpCurveBoundary;

class 
GX_DLLEXPIMPORT
NvGeCurveBoundary
{
public:
    NvGeCurveBoundary();
    NvGeCurveBoundary(int numberOfCurves, const NvGeEntity3d *const * crv3d,
                      const NvGeCurve2d *const * crv2d,
                      Nova::Boolean* orientation3d,
                      Nova::Boolean* orientation2d,
                      Nova::Boolean makeCopy = Nova::kTrue);
    NvGeCurveBoundary(const NvGeCurveBoundary&);

    ~NvGeCurveBoundary();

    // Assignment.
    //
    NvGeCurveBoundary& operator =   (const NvGeCurveBoundary& src);

    // Query the data.
    //
    Nova::Boolean     isDegenerate () const;
    Nova::Boolean     isDegenerate (NvGePosition3d& degenPoint, NvGeCurve2d** paramCurve) const;
    int                numElements  () const;
    void               getContour   (int& n, NvGeEntity3d*** crv3d,
                                     NvGeCurve2d*** paramGeometry,
                                     Nova::Boolean** orientation3d,
                                     Nova::Boolean** orientation2d) const;

    NvGeCurveBoundary& set (int numElements, const NvGeEntity3d *const * crv3d,
                            const NvGeCurve2d *const * crv2d,
                            Nova::Boolean* orientation3d,
                            Nova::Boolean* orientation2d,
                            Nova::Boolean makeCopy = Nova::kTrue);

    // Curve ownership.
    //
    Nova::Boolean     isOwnerOfCurves() const;
    NvGeCurveBoundary& setToOwnCurves ();

protected:
    friend class NvGeImpCurveBoundary;

    NvGeImpCurveBoundary    *mpImpBnd;
    int                     mDelBnd;
};

#pragma pack (pop)
#endif
