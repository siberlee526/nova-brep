// geclip2d.cpp - implementation of NvGeClipBoundary2d.
//
// A convex clip boundary stored as a closed polygon (NvGeClipBoundary2dData
// holder from geimpdata.h). Clipping uses the Sutherland-Hodgman algorithm
// against the convex boundary; the clip condition classifies the relation
// of the input to the boundary.

#include <geclip2d.h>

#include <gegblge.h>
#include <geimpdata.h>
#include <geintarr.h>
#include <gepnt2d.h>
#include <gept2dar.h>

#include <cmath>
#include <vector>

namespace
{

//! Holder of this boundary, with the COW discipline applied.
NvGeClipBoundary2dData* DataOf (NvGeImpEntity3d*& theImp)
{
  if (theImp->RefCount() > 1)
  {
    theImp->Unref();
    theImp = theImp->CloneShallow();
    occ::handle<NvGeEntityData> aCloned = NvGeDataOf<NvGeClipBoundary2dData> (theImp->Geom())->Clone();
    theImp->SetGeom (aCloned);
  }
  return NvGeDataOf<NvGeClipBoundary2dData> (theImp->Geom());
}

//! Read-only holder access.
const NvGeClipBoundary2dData* DataOf (const NvGeImpEntity3d* theImp)
{
  return NvGeDataOf<NvGeClipBoundary2dData> (theImp->Geom());
}

//! Installs a fresh holder, replacing the base placeholder impl.
void MakeClip (NvGeImpEntity3d*& theImp, const std::vector<gp_Pnt2d>& thePoints)
{
  if (theImp != nullptr)
  {
    theImp->Unref(); // release the placeholder adopted by the base constructor
  }
  occ::handle<NvGeClipBoundary2dData> aData = new NvGeClipBoundary2dData();
  aData->Points = thePoints;
  theImp = new NvGeImpEntity3d (NvGe::kClipBoundary2d, aData);
  theImp->Ref();
}

//! Signed area of a polygon (positive when counterclockwise).
double SignedArea (const std::vector<gp_Pnt2d>& thePoints)
{
  double anArea = 0.0;
  const size_t aCount = thePoints.size();
  for (size_t anIdx = 0; anIdx < aCount; ++anIdx)
  {
    const gp_Pnt2d& aA = thePoints[anIdx];
    const gp_Pnt2d& aB = thePoints[(anIdx + 1) % aCount];
    anArea += aA.X() * aB.Y() - aB.X() * aA.Y();
  }
  return 0.5 * anArea;
}

//! Validates and normalizes a convex polygon; returns false on bad input.
bool NormalizePolygon (std::vector<gp_Pnt2d>& thePoints)
{
  if (thePoints.size() < 3)
  {
    return false;
  }
  // Drop repeated closing point.
  const gp_Pnt2d& aFirst = thePoints.front();
  const gp_Pnt2d& aLast = thePoints.back();
  if (aFirst.Distance (aLast) <= 1e-12)
  {
    thePoints.pop_back();
  }
  if (thePoints.size() < 3)
  {
    return false;
  }
  // Convexity check via consistent cross-product signs (degenerate edges
  // with zero cross are tolerated).
  const size_t aCount = thePoints.size();
  int aPosCount = 0;
  int aNegCount = 0;
  for (size_t anIdx = 0; anIdx < aCount; ++anIdx)
  {
    const gp_Pnt2d& aA = thePoints[anIdx];
    const gp_Pnt2d& aB = thePoints[(anIdx + 1) % aCount];
    const gp_Pnt2d& aC = thePoints[(anIdx + 2) % aCount];
    const double anUx = aB.X() - aA.X();
    const double anUy = aB.Y() - aA.Y();
    const double aVx = aC.X() - aB.X();
    const double aVy = aC.Y() - aB.Y();
    const double aCross = anUx * aVy - anUy * aVx;
    if (aCross > 1e-12)
    {
      ++aPosCount;
    }
    else if (aCross < -1e-12)
    {
      ++aNegCount;
    }
  }
  return aPosCount == 0 || aNegCount == 0;
}

//! Inside test against one convex boundary edge (interior on the left of
//! each directed edge for a counterclockwise boundary).
bool IsInsideEdge (const gp_Pnt2d& thePnt, const gp_Pnt2d& theA, const gp_Pnt2d& theB, bool theCCW)
{
  const double aCross = (theB.X() - theA.X()) * (thePnt.Y() - theA.Y())
                      - (theB.Y() - theA.Y()) * (thePnt.X() - theA.X());
  return theCCW ? aCross >= -1e-12 : aCross <= 1e-12;
}

//! Intersection of segment [theA, theB] with the line through (theE1, theE2).
gp_Pnt2d SegmentLineIntersection (const gp_Pnt2d& theA, const gp_Pnt2d& theB,
                                  const gp_Pnt2d& theE1, const gp_Pnt2d& theE2)
{
  const double aDx = theB.X() - theA.X();
  const double aDy = theB.Y() - theA.Y();
  const double aEx = theE2.X() - theE1.X();
  const double aEy = theE2.Y() - theE1.Y();
  const double aDenom = aDx * aEy - aDy * aEx;
  if (std::abs (aDenom) <= 1e-300)
  {
    return theB; // parallel: keep the moving vertex
  }
  const double aT = ((theE1.X() - theA.X()) * aEy - (theE1.Y() - theA.Y()) * aEx) / aDenom;
  return gp_Pnt2d (theA.X() + aT * aDx, theA.Y() + aT * aDy);
}

} // namespace

//=================================================================================================

NvGeClipBoundary2d::NvGeClipBoundary2d()
{
  // Default: unit box boundary (valid state until set() is called).
  std::vector<gp_Pnt2d> aBox;
  aBox.push_back (gp_Pnt2d (0.0, 0.0));
  aBox.push_back (gp_Pnt2d (1.0, 0.0));
  aBox.push_back (gp_Pnt2d (1.0, 1.0));
  aBox.push_back (gp_Pnt2d (0.0, 1.0));
  MakeClip (mpImpEnt, aBox);
}

//=================================================================================================

NvGeClipBoundary2d::NvGeClipBoundary2d (const NvGePoint2d& theCornerA, const NvGePoint2d& theCornerB)
{
  const double aMinX = theCornerA.x < theCornerB.x ? theCornerA.x : theCornerB.x;
  const double aMinY = theCornerA.y < theCornerB.y ? theCornerA.y : theCornerB.y;
  const double aMaxX = theCornerA.x < theCornerB.x ? theCornerB.x : theCornerA.x;
  const double aMaxY = theCornerA.y < theCornerB.y ? theCornerB.y : theCornerA.y;
  std::vector<gp_Pnt2d> aBox;
  aBox.push_back (gp_Pnt2d (aMinX, aMinY));
  aBox.push_back (gp_Pnt2d (aMaxX, aMinY));
  aBox.push_back (gp_Pnt2d (aMaxX, aMaxY));
  aBox.push_back (gp_Pnt2d (aMinX, aMaxY));
  MakeClip (mpImpEnt, aBox);
}

//=================================================================================================

NvGeClipBoundary2d::NvGeClipBoundary2d (const NvGePoint2dArray& theClipBoundary)
{
  std::vector<gp_Pnt2d> aPoints (theClipBoundary.length());
  for (int anIdx = 0; anIdx < theClipBoundary.length(); ++anIdx)
  {
    aPoints[anIdx] = gp_Pnt2d (theClipBoundary[anIdx].x, theClipBoundary[anIdx].y);
  }
  if (!NormalizePolygon (aPoints))
  {
    throw NvException ("NvGeClipBoundary2d: the clip boundary polygon is invalid"
                       " (needs >= 3 non-repeated vertices forming a convex loop)");
  }
  MakeClip (mpImpEnt, aPoints);
}

//=================================================================================================

NvGeClipBoundary2d::NvGeClipBoundary2d (const NvGeClipBoundary2d& theSrc)
: NvGeEntity2d (theSrc)
{
}

//=================================================================================================

NvGe::ClipError NvGeClipBoundary2d::set (const NvGePoint2d& theCornerA, const NvGePoint2d& theCornerB)
{
  const double aMinX = theCornerA.x < theCornerB.x ? theCornerA.x : theCornerB.x;
  const double aMinY = theCornerA.y < theCornerB.y ? theCornerA.y : theCornerB.y;
  const double aMaxX = theCornerA.x < theCornerB.x ? theCornerB.x : theCornerA.x;
  const double aMaxY = theCornerA.y < theCornerB.y ? theCornerB.y : theCornerA.y;
  if (aMaxX - aMinX <= 1e-300 || aMaxY - aMinY <= 1e-300)
  {
    return NvGe::eInvalidClipBoundary;
  }
  std::vector<gp_Pnt2d> aBox;
  aBox.push_back (gp_Pnt2d (aMinX, aMinY));
  aBox.push_back (gp_Pnt2d (aMaxX, aMinY));
  aBox.push_back (gp_Pnt2d (aMaxX, aMaxY));
  aBox.push_back (gp_Pnt2d (aMinX, aMaxY));
  NvGeClipBoundary2dData* aData = DataOf (mpImpEnt);
  aData->Points = aBox;
  return NvGe::eOk;
}

//=================================================================================================

NvGe::ClipError NvGeClipBoundary2d::set (const NvGePoint2dArray& theClipBoundary)
{
  std::vector<gp_Pnt2d> aPoints (theClipBoundary.length());
  for (int anIdx = 0; anIdx < theClipBoundary.length(); ++anIdx)
  {
    aPoints[anIdx] = gp_Pnt2d (theClipBoundary[anIdx].x, theClipBoundary[anIdx].y);
  }
  if (!NormalizePolygon (aPoints))
  {
    return NvGe::eInvalidClipBoundary;
  }
  NvGeClipBoundary2dData* aData = DataOf (mpImpEnt);
  aData->Points = aPoints;
  return NvGe::eOk;
}

//=================================================================================================

NvGe::ClipError NvGeClipBoundary2d::clipPolygon (const NvGePoint2dArray& theRawVertices,
                                                 NvGePoint2dArray& theClippedVertices,
                                                 NvGe::ClipCondition& theClipCondition,
                                                 NvGeIntArray* pClippedSegmentSourceLabel) const
{
  (void)pClippedSegmentSourceLabel; // source labels are not tracked (approximation)
  const NvGeClipBoundary2dData* aData = DataOf (mpImpEnt);
  if (aData->Points.size() < 3 || theRawVertices.length() < 3)
  {
    theClipCondition = NvGe::kInvalid;
    return NvGe::eInvalidClipBoundary;
  }
  const bool aCCW = SignedArea (aData->Points) > 0.0;

  // Sutherland-Hodgman: clip the closed polygon against every boundary edge.
  std::vector<gp_Pnt2d> anInput;
  anInput.reserve (theRawVertices.length());
  for (int anIdx = 0; anIdx < theRawVertices.length(); ++anIdx)
  {
    anInput.push_back (gp_Pnt2d (theRawVertices[anIdx].x, theRawVertices[anIdx].y));
  }
  const size_t aNbEdges = aData->Points.size();
  for (size_t anEdge = 0; anEdge < aNbEdges && !anInput.empty(); ++anEdge)
  {
    const gp_Pnt2d& anE1 = aData->Points[anEdge];
    const gp_Pnt2d& anE2 = aData->Points[(anEdge + 1) % aNbEdges];
    std::vector<gp_Pnt2d> anOutput;
    anOutput.reserve (anInput.size() + 1);
    const size_t aCount = anInput.size();
    for (size_t anIdx = 0; anIdx < aCount; ++anIdx)
    {
      const gp_Pnt2d& aCur = anInput[anIdx];
      const gp_Pnt2d& aPrev = anInput[(anIdx + aCount - 1) % aCount];
      const bool aCurIn = IsInsideEdge (aCur, anE1, anE2, aCCW);
      const bool aPrevIn = IsInsideEdge (aPrev, anE1, anE2, aCCW);
      if (aCurIn)
      {
        if (!aPrevIn)
        {
          anOutput.push_back (SegmentLineIntersection (aPrev, aCur, anE1, anE2));
        }
        anOutput.push_back (aCur);
      }
      else if (aPrevIn)
      {
        anOutput.push_back (SegmentLineIntersection (aPrev, aCur, anE1, anE2));
      }
    }
    anInput.swap (anOutput);
  }

  theClippedVertices.setLogicalLength (0);
  for (const gp_Pnt2d& aPnt : anInput)
  {
    theClippedVertices.append (NvGePoint2d (aPnt.X(), aPnt.Y()));
  }

  // Condition classification.
  if (anInput.size() < 3)
  {
    theClipCondition = NvGe::kAllSegmentsOutsideZeroWinds;
  }
  else if (anInput.size() == static_cast<size_t> (theRawVertices.length()))
  {
    bool aAllSame = true;
    for (int anIdx = 0; anIdx < theRawVertices.length(); ++anIdx)
    {
      if (std::abs (anInput[anIdx].X() - theRawVertices[anIdx].x) > 1e-9
       || std::abs (anInput[anIdx].Y() - theRawVertices[anIdx].y) > 1e-9)
      {
        aAllSame = false;
        break;
      }
    }
    theClipCondition = aAllSame ? NvGe::kAllSegmentsInside : NvGe::kSegmentsIntersect;
  }
  else
  {
    theClipCondition = NvGe::kSegmentsIntersect;
  }
  return NvGe::eOk;
}

//=================================================================================================

NvGe::ClipError NvGeClipBoundary2d::clipPolyline (const NvGePoint2dArray& theRawVertices,
                                                  NvGePoint2dArray& theClippedVertices,
                                                  NvGe::ClipCondition& theClipCondition,
                                                  NvGeIntArray* pClippedSegmentSourceLabel) const
{
  (void)pClippedSegmentSourceLabel; // source labels are not tracked (approximation)
  const NvGeClipBoundary2dData* aData = DataOf (mpImpEnt);
  if (aData->Points.size() < 3 || theRawVertices.length() < 2)
  {
    theClipCondition = NvGe::kInvalid;
    return NvGe::eInvalidClipBoundary;
  }
  const bool aCCW = SignedArea (aData->Points) > 0.0;

  // Clip segment by segment, splitting the output whenever the polyline
  // leaves the boundary (represented as a single polyline for simplicity;
  // re-entering pieces are appended consecutively).
  theClippedVertices.setLogicalLength (0);
  bool anAllInside = true;
  bool anAnyInside = false;
  for (int anIdx = 1; anIdx < theRawVertices.length(); ++anIdx)
  {
    gp_Pnt2d anA (theRawVertices[anIdx - 1].x, theRawVertices[anIdx - 1].y);
    gp_Pnt2d aB (theRawVertices[anIdx].x, theRawVertices[anIdx].y);
    const size_t aNbEdges = aData->Points.size();
    std::vector<gp_Pnt2d> aSegment;
    aSegment.push_back (anA);
    aSegment.push_back (aB);
    for (size_t anEdge = 0; anEdge < aNbEdges && aSegment.size() >= 2; ++anEdge)
    {
      const gp_Pnt2d& anE1 = aData->Points[anEdge];
      const gp_Pnt2d& anE2 = aData->Points[(anEdge + 1) % aNbEdges];
      std::vector<gp_Pnt2d> anOutput;
      const size_t aCount = aSegment.size();
      for (size_t aVert = 0; aVert < aCount; ++aVert)
      {
        // Open polyline: no wrap-around; the first vertex is emitted only
        // when it is inside, later vertices follow the in/out transitions.
        const gp_Pnt2d& aCur = aSegment[aVert];
        const bool aCurIn = IsInsideEdge (aCur, anE1, anE2, aCCW);
        if (aVert == 0)
        {
          if (aCurIn)
          {
            anOutput.push_back (aCur);
          }
          continue;
        }
        const gp_Pnt2d& aPrev = aSegment[aVert - 1];
        const bool aPrevIn = IsInsideEdge (aPrev, anE1, anE2, aCCW);
        if (aCurIn)
        {
          if (!aPrevIn)
          {
            anOutput.push_back (SegmentLineIntersection (aPrev, aCur, anE1, anE2));
          }
          anOutput.push_back (aCur);
        }
        else if (aPrevIn)
        {
          anOutput.push_back (SegmentLineIntersection (aPrev, aCur, anE1, anE2));
        }
      }
      aSegment.swap (anOutput);
    }
    if (aSegment.empty())
    {
      anAllInside = false;
      continue;
    }
    anAnyInside = true;
    // The segment counts as untouched only when its endpoints survive
    // unchanged; a moved or inserted vertex means real clipping.
    if (static_cast<int> (aSegment.size()) != 2
     || aSegment.front().Distance (anA) > 1e-9
     || aSegment.back().Distance (aB) > 1e-9)
    {
      anAllInside = false;
    }
    // Avoid duplicating the shared vertex between consecutive segments.
    const bool aIsEmpty = theClippedVertices.length() == 0;
    const size_t aStart = (aIsEmpty || aSegment.size() != 2) ? 0 : 1;
    for (size_t aVert = aStart; aVert < aSegment.size(); ++aVert)
    {
      theClippedVertices.append (NvGePoint2d (aSegment[aVert].X(), aSegment[aVert].Y()));
    }
  }

  if (!anAnyInside)
  {
    theClipCondition = NvGe::kAllSegmentsOutsideZeroWinds;
  }
  else if (anAllInside)
  {
    theClipCondition = NvGe::kAllSegmentsInside;
  }
  else
  {
    theClipCondition = NvGe::kSegmentsIntersect;
  }
  return NvGe::eOk;
}

//=================================================================================================

NvGeClipBoundary2d& NvGeClipBoundary2d::operator = (const NvGeClipBoundary2d& theSrc)
{
  NvGeEntity2d::operator= (theSrc);
  return *this;
}
