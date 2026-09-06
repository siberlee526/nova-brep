#include <NvShape.h>

#include <NvException.h>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <cmath>

//=================================================================================================

bool NvShape::IsValid() const
{
  if (myShape.IsNull())
  {
    return false;
  }
  BRepCheck_Analyzer anAnalyzer (myShape);
  return anAnalyzer.IsValid();
}

//=================================================================================================

Bnd_Box NvShape::BoundingBox() const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::BoundingBox(): the shape is null");
  }

  // Reset the gap so that the box returns exact extents of the geometry.
  Bnd_Box aBox;
  aBox.SetGap (0.0);
  BRepBndLib::Add (myShape, aBox);
  return aBox;
}

//=================================================================================================

double NvShape::Volume() const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Volume(): the shape is null");
  }
  GProp_GProps aProperties;
  BRepGProp::VolumeProperties (myShape, aProperties);
  // Mass properties are signed with respect to shape orientation (e.g. a mirrored
  // solid has negative mass); the physical volume is orientation-independent.
  return std::abs (aProperties.Mass());
}

//=================================================================================================

double NvShape::Area() const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Area(): the shape is null");
  }
  GProp_GProps aProperties;
  BRepGProp::SurfaceProperties (myShape, aProperties);
  return aProperties.Mass();
}

//=================================================================================================

int NvShape::CountSubShapes (TopAbs_ShapeEnum theType) const
{
  if (myShape.IsNull())
  {
    return 0;
  }

  // MapShapes counts each shared sub-shape once, unlike TopExp_Explorer
  // which yields every occurrence within the topology tree.
  TopTools_IndexedMapOfShape aMap;
  TopExp::MapShapes (myShape, theType, aMap);
  return aMap.Extent();
}

//=================================================================================================

NvShape NvShape::Transformed (const gp_Trsf& theTrsf) const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Transformed(): the shape is null");
  }

  try
  {
    // Rigid transformations (identity, translation, rotation, mirrors) are applied
    // through the shape location, which is cheap and never mutates shared geometry.
    // Scale and compound transformations modify geometry, so request an explicit copy
    // to preserve value semantics of shared OCCT shapes.
    const gp_TrsfForm aForm = theTrsf.Form();
    const bool aNeedsCopy = (aForm == gp_Scale) || (aForm == gp_CompoundTrsf);
    if (!aNeedsCopy)
    {
      return NvShape (myShape.Moved (TopLoc_Location (theTrsf)));
    }

    BRepBuilderAPI_Transform aTransform (myShape, theTrsf, true);
    if (!aTransform.IsDone())
    {
      throw NvException ("NvShape::Transformed(): BRepBuilderAPI_Transform failed");
    }
    return NvShape (aTransform.Shape());
  }
  catch (const Standard_Failure& theFailure)
  {
    throw NvException::FromFailure (theFailure);
  }
}

//=================================================================================================

NvShape NvShape::Translated (const gp_Vec& theVector) const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Translated(): the shape is null");
  }

  gp_Trsf aTrsf;
  aTrsf.SetTranslation (theVector);
  return NvShape (myShape.Moved (TopLoc_Location (aTrsf)));
}

//=================================================================================================

NvShape NvShape::Rotated (const gp_Ax1& theAxis, double theAngleRadians) const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Rotated(): the shape is null");
  }

  gp_Trsf aTrsf;
  aTrsf.SetRotation (theAxis, theAngleRadians);
  return NvShape (myShape.Moved (TopLoc_Location (aTrsf)));
}

//=================================================================================================

NvShape NvShape::Scaled (const gp_Pnt& thePoint, double theFactor) const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Scaled(): the shape is null");
  }
  if (std::abs (theFactor) < Precision::Confusion())
  {
    throw NvException ("NvShape::Scaled(): the scale factor is zero");
  }

  gp_Trsf aTrsf;
  aTrsf.SetScale (thePoint, theFactor);
  return Transformed (aTrsf);
}

//=================================================================================================

NvShape NvShape::Mirrored (const gp_Ax2& thePlane) const
{
  if (myShape.IsNull())
  {
    throw NvException ("NvShape::Mirrored(): the shape is null");
  }

  gp_Trsf aTrsf;
  aTrsf.SetMirror (thePlane);
  return NvShape (myShape.Moved (TopLoc_Location (aTrsf)));
}
