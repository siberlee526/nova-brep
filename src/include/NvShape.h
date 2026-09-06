#ifndef NvShape_HeaderFile
#define NvShape_HeaderFile

#include <NvMacros.h>

#include <Bnd_Box.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>

//! Value-semantic wrapper over TopoDS_Shape providing higher-level queries
//! and transformations with built-in error checking.
//!
//! Copying a NvShape is cheap (OCCT shapes are reference-counted internally).
//! All non-const semantics follow OCCT conventions: transformations return a new
//! shape and never silently mutate shared geometry. Methods that require a
//! non-null shape throw NvException when called on a null shape.
class NV_EXPORT NvShape
{
public:

  //! Default constructor; creates a null shape.
  NvShape() = default;

  //! Wraps an OCCT shape (no deep copy; the underlying TShape is shared).
  NvShape (const TopoDS_Shape& theShape)
  : myShape (theShape) {}

  //! Default copy/move semantics (cheap; OCCT shape data is shared).
  NvShape (const NvShape& theOther) = default;
  NvShape (NvShape&& theOther) = default;
  NvShape& operator= (const NvShape& theOther) = default;
  NvShape& operator= (NvShape&& theOther) = default;

  ~NvShape() = default;

  //! Returns true if the shape is null (was never initialized).
  bool IsNull() const { return myShape.IsNull(); }

  //! Returns true if the shape is a valid B-Rep (BRepCheck_Analyzer).
  //! A null shape is reported as invalid.
  bool IsValid() const;

  //! Access the underlying OCCT shape (escape hatch for advanced use).
  const TopoDS_Shape& Shape() const { return myShape; }

  //! Computes a tight bounding box of the shape.
  //! @note the box is padded by sub-shape tolerances (as computed by BRepBndLib).
  //! @return the bounding box of the shape
  //! @throws NvException if the shape is null
  Bnd_Box BoundingBox() const;

  //! Computes the volume of the shape (mass properties).
  //! @return absolute (orientation-independent) volume in model units cubed
  //! @throws NvException if the shape is null
  double Volume() const;

  //! Computes the total surface area of the shape.
  //! @return area in model units squared
  //! @throws NvException if the shape is null
  double Area() const;

  //! Counts unique sub-shapes of the given type (faces, edges, vertices, ...);
  //! shared sub-shapes are counted once, independently of the number of occurrences.
  //! @param[in] theType sub-shape type to count, e.g. TopAbs_FACE
  //! @return number of unique sub-shapes of the given type; 0 for a null shape
  int CountSubShapes (TopAbs_ShapeEnum theType) const;

  //! Applies an arbitrary transformation, returning a new shape.
  //! Rigid transformations reuse the shape location (cheap and non-mutating);
  //! scaling transformations are applied on a copy, so shared geometry is safe.
  //! @param[in] theTrsf transformation to apply
  //! @return the transformed shape
  //! @throws NvException if the shape is null or the transformation fails
  NvShape Transformed (const gp_Trsf& theTrsf) const;

  //! Returns a copy translated by the given vector (cheap; uses location).
  NvShape Translated (const gp_Vec& theVector) const;

  //! Returns a copy rotated around the given axis by theAngleRadians.
  NvShape Rotated (const gp_Ax1& theAxis, double theAngleRadians) const;

  //! Returns a copy scaled relative to the given point.
  //! @throws NvException if the factor is (almost) zero
  NvShape Scaled (const gp_Pnt& thePoint, double theFactor) const;

  //! Returns a copy mirrored with respect to the given plane (gp_Ax2 Z axis is the normal).
  NvShape Mirrored (const gp_Ax2& thePlane) const;

private:

  TopoDS_Shape myShape;
};

#endif // NvShape_HeaderFile
