#ifndef NV_GEENT2D_H
#define NV_GEENT2D_H

#include "gegbl.h"
#include "gepnt2d.h"
#include "geintrvl.h"
#include "gegblnew.h"
#pragma pack (push, 8)

class

NvGeEntity2d
{
public:
    GE_DLLEXPIMPORT ~NvGeEntity2d();

    // Run time type information.
    //
    GE_DLLEXPIMPORT Adesk::Boolean   isKindOf    (NvGe::EntityId entType) const;
    GE_DLLEXPIMPORT NvGe::EntityId   type        () const;

    // Make a copy of the entity.
    //
    GE_DLLEXPIMPORT NvGeEntity2d*    copy        () const;
    GE_DLLEXPIMPORT NvGeEntity2d&    operator =  (const NvGeEntity2d& entity);

    // Equivalence
    //
    GE_DLLEXPIMPORT Adesk::Boolean   operator == (const NvGeEntity2d& entity) const;
    GE_DLLEXPIMPORT Adesk::Boolean   operator != (const NvGeEntity2d& entity) const;
    GE_DLLEXPIMPORT Adesk::Boolean   isEqualTo   (const NvGeEntity2d& entity,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
                                            
    // Matrix multiplication
    //
    GE_DLLEXPIMPORT NvGeEntity2d&    transformBy (const NvGeMatrix2d& xfm);
    GE_DLLEXPIMPORT NvGeEntity2d&    translateBy (const NvGeVector2d& translateVec);
    GE_DLLEXPIMPORT NvGeEntity2d&    rotateBy    (double angle, const NvGePoint2d& wrtPoint
                                  = NvGePoint2d::kOrigin);      
    GE_DLLEXPIMPORT NvGeEntity2d&    mirror      (const NvGeLine2d& line);
    GE_DLLEXPIMPORT NvGeEntity2d&    scaleBy     (double scaleFactor,
                                  const NvGePoint2d& wrtPoint
                                  = NvGePoint2d::kOrigin);
    // Point containment
    //
    GE_DLLEXPIMPORT Adesk::Boolean   isOn        (const NvGePoint2d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
protected:
    friend class NvGeEntity3d;
    friend class NvGeImpEntity3d;
    class NvGeImpEntity3d* mpImpEnt;
    int mDelEnt;
    GE_DLLEXPIMPORT NvGeEntity2d ();
    GE_DLLEXPIMPORT NvGeEntity2d (const NvGeEntity2d&);
    GE_DLLEXPIMPORT NvGeEntity2d (NvGeImpEntity3d&, int);
    GE_DLLEXPIMPORT NvGeEntity2d (NvGeImpEntity3d*);
    GE_DLLEXPIMPORT NvGeEntity2d*    newEntity2d (NvGeImpEntity3d*) const;
};


inline NvGeEntity2d*   
NvGeEntity2d::newEntity2d (NvGeImpEntity3d *impEnt ) const
{
    return GENEWLOC( NvGeEntity2d, this) ( impEnt );
}

#pragma pack (pop)
#endif
