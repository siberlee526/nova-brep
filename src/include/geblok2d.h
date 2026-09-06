#ifndef NV_GEBLOCK2D_H
#define NV_GEBLOCK2D_H

#include "geent2d.h"
#pragma pack (push, 8)
class NvGePoint2d;
class NvGeVector2d;

class 

NvGeBoundBlock2d : public NvGeEntity2d
{
public:
                    
	GE_DLLEXPIMPORT NvGeBoundBlock2d ();
	GE_DLLEXPIMPORT NvGeBoundBlock2d (const NvGePoint2d& point1, const NvGePoint2d& point2);
	GE_DLLEXPIMPORT NvGeBoundBlock2d (const NvGePoint2d& base,
                      const NvGeVector2d& dir1, const NvGeVector2d& dir2);
	GE_DLLEXPIMPORT NvGeBoundBlock2d (const NvGeBoundBlock2d& block);
    
	// Access methods.
    //    
    GE_DLLEXPIMPORT void              getMinMaxPoints  (NvGePoint2d& point1,
								        NvGePoint2d& point2) const;
    GE_DLLEXPIMPORT void              get              (NvGePoint2d& base,
								        NvGeVector2d& dir1,
								        NvGeVector2d& dir2) const;
    
	// Set methods.
    //    
    GE_DLLEXPIMPORT NvGeBoundBlock2d& set         (const NvGePoint2d& point1,
                                   const NvGePoint2d& point2);
    GE_DLLEXPIMPORT NvGeBoundBlock2d& set         (const NvGePoint2d& base,
                                   const NvGeVector2d& dir1,
                                   const NvGeVector2d& dir2);
    // Expand to contain point.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock2d& extend      (const NvGePoint2d& point);
   
	// Expand by a specified distance.
    //
    GE_DLLEXPIMPORT NvGeBoundBlock2d& swell       (double distance);

    // Containment and intersection tests
    //
    GE_DLLEXPIMPORT Nova::Boolean    contains    (const NvGePoint2d& point) const;
    GE_DLLEXPIMPORT Nova::Boolean    isDisjoint  (const NvGeBoundBlock2d& block)
                                                 const;
    // Assignment operator
    //
    GE_DLLEXPIMPORT NvGeBoundBlock2d& operator =  (const NvGeBoundBlock2d& block);

	GE_DLLEXPIMPORT Nova::Boolean    isBox     () const;
	GE_DLLEXPIMPORT NvGeBoundBlock2d& setToBox  (Nova::Boolean);
};


#pragma pack (pop)
#endif
