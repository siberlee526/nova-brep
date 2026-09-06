#ifndef NV_GEENT3D_H
#define NV_GEENT3D_H

#include "gegbl.h"
#include "gepnt3d.h"
#include "geent2d.h"
#include "geintrvl.h"
#include "gegblnew.h"
#pragma pack (push, 8)

class

NvGeEntity3d
{
public:
    GE_DLLEXPIMPORT ~NvGeEntity3d();

    // Run time type information.
    //
    GE_DLLEXPIMPORT Nova::Boolean   isKindOf    (NvGe::EntityId entType) const;
    GE_DLLEXPIMPORT NvGe::EntityId   type        () const;

    // Make a copy of the entity.
    //
    GE_DLLEXPIMPORT NvGeEntity3d*    copy        () const;
    GE_DLLEXPIMPORT NvGeEntity3d&    operator =  (const NvGeEntity3d& entity);

    // Equivalence
    //
    GE_DLLEXPIMPORT Nova::Boolean   operator == (const NvGeEntity3d& entity) const;
    GE_DLLEXPIMPORT Nova::Boolean   operator != (const NvGeEntity3d& entity) const;
    GE_DLLEXPIMPORT Nova::Boolean   isEqualTo   (const NvGeEntity3d& ent,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
    // Matrix multiplication
    //
    GE_DLLEXPIMPORT NvGeEntity3d&    transformBy (const NvGeMatrix3d& xfm);
    GE_DLLEXPIMPORT NvGeEntity3d&    translateBy (const NvGeVector3d& translateVec);
    GE_DLLEXPIMPORT NvGeEntity3d&    rotateBy    (double angle, const NvGeVector3d& vec,
                                  const NvGePoint3d& wrtPoint = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT NvGeEntity3d&    mirror      (const NvGePlane& plane);
    GE_DLLEXPIMPORT NvGeEntity3d&    scaleBy     (double scaleFactor,
                                  const NvGePoint3d& wrtPoint
                                  = NvGePoint3d::kOrigin);
    // Point containment
    //
    GE_DLLEXPIMPORT Nova::Boolean   isOn        (const NvGePoint3d& pnt,
                                  const NvGeTol& tol = NvGeContext::gTol) const;
protected:
    friend class NvGeImpEntity3d;
    class NvGeImpEntity3d  *mpImpEnt;
    int              mDelEnt;
    GE_DLLEXPIMPORT NvGeEntity3d ();
    GE_DLLEXPIMPORT NvGeEntity3d (const NvGeEntity3d&);
    GE_DLLEXPIMPORT NvGeEntity3d (NvGeImpEntity3d&, int);
    GE_DLLEXPIMPORT NvGeEntity3d (NvGeImpEntity3d*);
    GE_DLLEXPIMPORT NvGeEntity2d* newEntity2d (NvGeImpEntity3d*) const;
    GE_DLLEXPIMPORT NvGeEntity2d* newEntity2d (NvGeImpEntity3d&, int) const;
    GE_DLLEXPIMPORT NvGeEntity3d* newEntity3d (NvGeImpEntity3d*) const;
    GE_DLLEXPIMPORT NvGeEntity3d* newEntity3d (NvGeImpEntity3d&, int) const;
};

inline NvGeEntity2d*
NvGeEntity3d::newEntity2d (NvGeImpEntity3d *impEnt ) const
{
    return GENEWLOC( NvGeEntity2d, this) ( impEnt );
}

inline NvGeEntity3d*
NvGeEntity3d::newEntity3d (NvGeImpEntity3d *impEnt ) const
{
    return GENEWLOC( NvGeEntity3d, this) ( impEnt );
}

inline NvGeEntity3d*
NvGeEntity3d::newEntity3d(NvGeImpEntity3d& impEnt, int dummy) const
{
    return GENEWLOC( NvGeEntity3d, this)(impEnt, dummy);
}

inline NvGeEntity2d*
NvGeEntity3d::newEntity2d(NvGeImpEntity3d& impEnt, int dummy) const
{
    return GENEWLOC( NvGeEntity2d, this)(impEnt, dummy);
}

#pragma pack (pop)
#endif
