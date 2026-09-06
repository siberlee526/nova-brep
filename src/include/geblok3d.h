#ifndef NV_GEBLOCK3D_H
#define NV_GEBLOCK3D_H

#include "geent3d.h"
#pragma pack (push, 8)
class NvGePoint3d;
class NvGeVector3d;

class 

NvGeBoundBlock3d : public NvGeEntity3d
{
public:
                    
	GE_DLLEXPIMPORT NvGeBoundBlock3d ();
	GE_DLLEXPIMPORT NvGeBoundBlock3d (const NvGePoint3d& base, const NvGeVector3d& dir1,
					  const NvGeVector3d& dir2, const NvGeVector3d& dir3);
	GE_DLLEXPIMPORT NvGeBoundBlock3d (const NvGeBoundBlock3d& block);
    
	// Access methods.
    //    
    GE_DLLEXPIMPORT void              getMinMaxPoints  (NvGePoint3d& point1,
								        NvGePoint3d& point2) const;
    GE_DLLEXPIMPORT void              get              (NvGePoint3d& base,
								        NvGeVector3d& dir1,
								        NvGeVector3d& dir2,
								        NvGeVector3d& dir3) const;
	// Set methods.
    //    
    GE_DLLEXPIMPORT NvGeBoundBlock3d& set              (const NvGePoint3d& point1,
								        const NvGePoint3d& point2);
    GE_DLLEXPIMPORT NvGeBoundBlock3d& set              (const NvGePoint3d& base,
								        const NvGeVector3d& dir1,
								        const NvGeVector3d& dir2,
								        const NvGeVector3d& dir3);
    // Expand to contain point.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock3d& extend           (const NvGePoint3d& point);
   
	// Expand by a specified distance.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock3d& swell            (double distance);

    // Containment and intersection tests
    //
    GE_DLLEXPIMPORT Nova::Boolean    contains         (const NvGePoint3d& point) const;
    GE_DLLEXPIMPORT Nova::Boolean    isDisjoint       (const NvGeBoundBlock3d& block) const;

    // Assignment opearator
    //
    GE_DLLEXPIMPORT NvGeBoundBlock3d& operator =       (const NvGeBoundBlock3d& block);

    GE_DLLEXPIMPORT Nova::Boolean     isBox    () const;
    GE_DLLEXPIMPORT NvGeBoundBlock3d&  setToBox (Nova::Boolean);
};


#pragma pack (pop)
#endif
