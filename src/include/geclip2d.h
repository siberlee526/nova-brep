#ifndef NV_GECLIP2D_H
#define NV_GECLIP2D_H

#include "Nova.h"
#include "assert.h"
#include "geent2d.h"
#include "gepnt2d.h"
#include "gept2dar.h"
#include "gegbl.h"
#include "geintarr.h"
#pragma pack (push, 8)

class NvGeImpClipBoundary2d;

class

NvGeClipBoundary2d : public NvGeEntity2d
{
public:

    GE_DLLEXPIMPORT NvGeClipBoundary2d();

    // Initialize for ortho-aligned rectangular clip boundary.
    //
    GE_DLLEXPIMPORT NvGeClipBoundary2d(const NvGePoint2d& cornerA, const NvGePoint2d& cornerB);

    // Initialize for convex polyline/polygon clip boundary (1 or more edges).  
    //
    GE_DLLEXPIMPORT NvGeClipBoundary2d(const NvGePoint2dArray& clipBoundary);
   
	// Copy constructor.
	//
	GE_DLLEXPIMPORT NvGeClipBoundary2d (const NvGeClipBoundary2d& src);

    // Notes on set() methods:
    // 1) set() need be called only once after the creation of the clipper.  
    //    The clipper can then be reused by calling
    //    clipPolygon or clipPolyline repeatedly.

    // Initialize for ortho-aligned rectangular clip boundary.
    //
	GE_DLLEXPIMPORT NvGe::ClipError		set(const NvGePoint2d& cornerA, const NvGePoint2d& cornerB);

    // Initialize for convex polyline/polygon clip boundary (1 or more edges).  
    // 1) Counterclockwise orientation is
    //    assumed if there is only one non-colinear clipping edge in the polygon.
    // 2) If more than one non-colinear edge is supplied, the orientation of the edges
    //    is computed automagically.
    //
    GE_DLLEXPIMPORT NvGe::ClipError		set(const NvGePoint2dArray& clipBoundary);

    // Optional clipped segment source information (below) is interpreted as follows:
    // Let srcInx(n) := (*pClippedSegmentSourceLabel).at(n).
    //
    // Case 1: If srcInx(n) > 0, then segment number "n", connecting
    //      output vertices "n-1" and "n", is a piece of source segment "srcInx(n)", connecting
    //      input vertices "srcInx(n)-1" and "srcInx(n)".
    // Case 2: If srcInx(n) < 0, then it is a piece of clip boundary segment "srcInx(n)", connecting
    //      clip boundary vertices "srcInx(n)-1" and "srcInx(n)".
	//
	// Notes: 
    // 1)   srcInx(0), which doesn't correspond to a segment, should be ignored.
    // 2)   Degenerate clip boundary edges and polyline/polygon segments are skipped, so the
    //      srcInx() values aren't necessarily sequential.
    //
    
    // Clip a closed polygon, creating a second closed polygon.
    //
    GE_DLLEXPIMPORT NvGe::ClipError		clipPolygon(const NvGePoint2dArray& rawVertices, 
                                    NvGePoint2dArray& clippedVertices,
                                    NvGe::ClipCondition& clipCondition,
                                    NvGeIntArray* pClippedSegmentSourceLabel = 0) const;

    // Clip a polyline, creating a second polyline.
    //
    GE_DLLEXPIMPORT NvGe::ClipError		clipPolyline(const NvGePoint2dArray& rawVertices, 
                                     NvGePoint2dArray& clippedVertices,
                                     NvGe::ClipCondition& clipCondition,
                                     NvGeIntArray* pClippedSegmentSourceLabel = 0) const;

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeClipBoundary2d& operator = (const NvGeClipBoundary2d& crvInt);
};


#pragma pack (pop)
#endif
