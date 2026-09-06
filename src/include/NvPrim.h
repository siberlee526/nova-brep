#ifndef NvPrim_HeaderFile
#define NvPrim_HeaderFile

#include <NvShape.h>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

//! Higher-level factory for parametric primitives, wrapping BRepPrimAPI builders.
//!
//! Every builder checks IsDone() of the underlying OCCT algorithm and throws
//! NvException on failure, so a returned NvShape is always well-formed.
class NV_EXPORT NvPrim
{
public:

  //! Creates a box with dimensions theDx x theDy x theDz with a corner at origin.
  NvPrim() = delete;

  //! Creates a box with dimensions theDx x theDy x theDz and a corner at the origin.
  static NvShape Box (double theDx, double theDy, double theDz);

  //! Creates a box with dimensions theDx x theDy x theDz and a corner at thePoint.
  static NvShape Box (const gp_Pnt& thePoint, double theDx, double theDy, double theDz);

  //! Creates a cylinder of the given radius and height with base at the origin (OZ axis).
  static NvShape Cylinder (double theRadius, double theHeight);

  //! Creates a cylinder of the given radius and height along the axis system theAxis.
  static NvShape Cylinder (const gp_Ax2& theAxis, double theRadius, double theHeight);

  //! Creates a full sphere of the given radius centered at the origin.
  static NvShape Sphere (double theRadius);

  //! Creates a full sphere of the given radius centered at thePoint.
  static NvShape Sphere (const gp_Pnt& thePoint, double theRadius);

  //! Creates a cone with the given radii and height, base at the origin (OZ axis).
  //! A radius of 0 produces an apex; both radii equal 0 is invalid.
  static NvShape Cone (double theRadius1, double theRadius2, double theHeight);

  //! Creates a cone with the given radii and height along the axis system theAxis.
  static NvShape Cone (const gp_Ax2& theAxis, double theRadius1, double theRadius2, double theHeight);

  //! Creates a torus with the given major/minor radii in the XOY plane of the origin.
  static NvShape Torus (double theMajorRadius, double theMinorRadius);

  //! Creates a torus with the given major/minor radii in the XOY plane of theAxis.
  static NvShape Torus (const gp_Ax2& theAxis, double theMajorRadius, double theMinorRadius);
};

#endif // NvPrim_HeaderFile
