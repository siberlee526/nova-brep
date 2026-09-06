// geimpdata.h - internal implementation backbone of the ge entity layer.
//
// PRIVATE to src/occt: never installed, never included from public headers.
// Every NvGeEntity2d / NvGeEntity3d public class is a non-virtual pimpl
// shell whose only data is the mpImpEnt pointer declared in geent2d.h /
// geent3d.h. All geometry lives here, inside NvGeImpEntity3d:
//
//   - 2d curves   : occ::handle<Geom2d_Curve>  (the exact OCCT curve;
//                   bounded arcs/segments are stored as Geom2d_TrimmedCurve,
//                   rational/periodic nurbs as Geom2d_BSplineCurve, offsets
//                   as Geom2d_OffsetCurve)
//   - 3d curves   : occ::handle<Geom_Curve>    (same conventions)
//   - surfaces    : occ::handle<Geom_Surface>  (analytic + BSpline + offset
//                   + rectangular-trimmed variants)
//   - other kinds : one of the small NvGe*Data holder classes below.
//
// COPY-ON-WRITE DISCIPLINE (critical): impl objects are refcounted and the
// underlying OCCT geometry handles are SHARED between copies. Modifier
// methods of the public classes must therefore NEVER call mutating Set*()
// methods on a possibly shared OCCT object. Instead they build a NEW OCCT
// object and replace the impl geometry, after cloning the impl when its
// reference count is greater than one:
//
//   if (mpImpEnt->RefCount() > 1)
//   {
//     mpImpEnt->Unref();
//     mpImpEnt = mpImpEnt->CloneShallow();
//   }
//   mpImpEnt->SetGeom (theNewHandle);
//
// Freshly created, uniquely-owned handles may of course be built in place.

#ifndef NV_GeImpData_HeaderFile
#define NV_GeImpData_HeaderFile

#include <NvException.h>
#include <gegbl.h>
#include <geintrvl.h>
#include <gemat2d.h>
#include <gemat3d.h>
#include <gepnt2d.h>
#include <gepnt3d.h>
#include <gevec2d.h>
#include <gevec3d.h>

#include <Standard_Handle.hxx>
#include <Standard_Transient.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Trsf2d.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Vec.hxx>

#include <limits>
#include <vector>

class NvGeCurve2d;
class NvGeCurve3d;
class NvGeSurface;

// =====================================================================
// Interval implementation
// =====================================================================

//! Storage behind NvGeInterval: one or two bounds, either of which may be
//! infinite (an unbounded direction is encoded by an infinite bound).
class NvGeImpInterval
{
public:
  double LowerBound;
  double UpperBound;
  double Tolerance;

  //! Creates an unbounded interval.
  NvGeImpInterval()
  : LowerBound (-std::numeric_limits<double>::infinity()),
    UpperBound (std::numeric_limits<double>::infinity()),
    Tolerance (1e-12)
  {
  }
};

// =====================================================================
// Entity implementation
// =====================================================================

//! Reference-counted implementation object behind every NvGeEntity2d / 3d.
//! Derives Standard_Transient so that occ::handle can share ownership with
//! the manually-refcounted public wrappers (both paths use the same
//! counter; deletion goes through the virtual Delete()).
//! myGeom holds the OCCT geometry of the entity or one of the holders below.
class NvGeImpEntity3d : public Standard_Transient
{
public:
  //! Fresh impl starts with a zero reference count, exactly like any
  //! Standard_Transient object created into a handle.
  NvGeImpEntity3d (NvGe::EntityId theType, const occ::handle<Standard_Transient>& theGeom)
  : myType (theType),
    myGeom (theGeom)
  {
  }

  //! Manual reference counted ownership (wrapper side).
  void Ref()
  {
    IncrementRefCounter();
  }

  void Unref()
  {
    if (DecrementRefCounter() == 0)
    {
      Delete();
    }
  }

  int RefCount() const
  {
    return GetRefCount();
  }

  NvGe::EntityId Type() const
  {
    return myType;
  }

  void SetType (NvGe::EntityId theType)
  {
    myType = theType;
  }

  const occ::handle<Standard_Transient>& Geom() const
  {
    return myGeom;
  }

  //! Replaces the geometry. Only legal on a uniquely owned impl (see the
  //! copy-on-write discipline in the file banner) or to install geometry on
  //! a freshly created impl.
  void SetGeom (const occ::handle<Standard_Transient>& theGeom)
  {
    myGeom = theGeom;
  }

  //! New impl sharing the same (immutable-by-convention) geometry handle.
  NvGeImpEntity3d* CloneShallow() const
  {
    return new NvGeImpEntity3d (myType, myGeom);
  }

private:
  NvGe::EntityId                  myType;
  occ::handle<Standard_Transient> myGeom;
};

// =====================================================================
// Holder classes for entity kinds without a direct OCCT counterpart
// =====================================================================

//! Base of all custom data holders; deep-copyable through Clone().
class NvGeEntityData : public Standard_Transient
{
public:
  //! Creates a deep copy holding no shared mutable state.
  virtual occ::handle<NvGeEntityData> Clone() const = 0;
};

//! NvGePosition2d storage.
class NvGePosition2dData : public NvGeEntityData
{
public:
  gp_Pnt2d Point;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGePosition2dData> aCopy = new NvGePosition2dData();
    aCopy->Point = Point;
    return aCopy;
  }
};

//! NvGePosition3d storage.
class NvGePosition3dData : public NvGeEntityData
{
public:
  gp_Pnt Point;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGePosition3dData> aCopy = new NvGePosition3dData();
    aCopy->Point = Point;
    return aCopy;
  }
};

//! NvGePointOnCurve2d storage: shares the curve entity impl and stores the
//! curve parameter of the point.
class NvGeCurve2d;

class NvGePointOnCurve2dData : public NvGeEntityData
{
public:
  occ::handle<NvGeImpEntity3d> Curve; // shared, immutable-by-convention
  double Param = 0.0;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGePointOnCurve2dData> aCopy = new NvGePointOnCurve2dData();
    aCopy->Curve = Curve;
    aCopy->Param = Param;
    return aCopy;
  }
};

//! NvGePointOnCurve3d storage.
class NvGePointOnCurve3dData : public NvGeEntityData
{
public:
  occ::handle<NvGeImpEntity3d> Curve;
  double Param = 0.0;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGePointOnCurve3dData> aCopy = new NvGePointOnCurve3dData();
    aCopy->Curve = Curve;
    aCopy->Param = Param;
    return aCopy;
  }
};

//! NvGePointOnSurface storage.
class NvGePointOnSurfaceData : public NvGeEntityData
{
public:
  occ::handle<NvGeImpEntity3d> Surface;
  double U = 0.0;
  double V = 0.0;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGePointOnSurfaceData> aCopy = new NvGePointOnSurfaceData();
    aCopy->Surface = Surface;
    aCopy->U = U;
    aCopy->V = V;
    return aCopy;
  }
};

//! NvGeBoundBlock2d storage: a parallelogram spanned by two vectors from a
//! base point (ARX semantics; degenerate vectors mean an unbounded side).
class NvGeBoundBlock2dData : public NvGeEntityData
{
public:
  gp_Pnt2d Point;
  gp_Vec2d Vec1;
  gp_Vec2d Vec2;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeBoundBlock2dData> aCopy = new NvGeBoundBlock2dData();
    aCopy->Point = Point;
    aCopy->Vec1 = Vec1;
    aCopy->Vec2 = Vec2;
    return aCopy;
  }
};

//! NvGeBoundBlock3d storage: a parallelepiped spanned by three vectors.
class NvGeBoundBlock3dData : public NvGeEntityData
{
public:
  gp_Pnt  Point;
  gp_Vec  Vec1;
  gp_Vec  Vec2;
  gp_Vec  Vec3;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeBoundBlock3dData> aCopy = new NvGeBoundBlock3dData();
    aCopy->Point = Point;
    aCopy->Vec1 = Vec1;
    aCopy->Vec2 = Vec2;
    aCopy->Vec3 = Vec3;
    return aCopy;
  }
};

//! NvGeClipBoundary2d storage: a closed polygon (first point not repeated).
class NvGeClipBoundary2dData : public NvGeEntityData
{
public:
  std::vector<gp_Pnt2d> Points;

  occ::handle<NvGeEntityData> Clone() const override
  {
    occ::handle<NvGeClipBoundary2dData> aCopy = new NvGeClipBoundary2dData();
    aCopy->Points = Points;
    return aCopy;
  }
};

// =====================================================================
// Matrix conversion helpers
// =====================================================================

//! Converts a conformal 2d matrix (rotation + uniform scale + optional
//! mirror) into a gp_Trsf2d. Throws NvException for non-conformal input:
//! analytic OCCT curves cannot represent shear or non-uniform scaling.
inline gp_Trsf2d Trsf2dFromMatrix (const NvGeMatrix2d& theMat)
{
  double aScale = 0.0;
  double anAngle = 0.0;
  Nova::Boolean isMirror = false;
  NvGeVector2d aReflex;
  if (!theMat.isConformal (aScale, anAngle, isMirror, aReflex))
  {
    throw NvException ("NvGe: the matrix is not conformal; analytic entities"
                       " cannot absorb shear or non-uniform scaling");
  }
  gp_Trsf2d aTrsf;
  if (isMirror)
  {
    // Reflection about the line through the origin along aReflex, then
    // rotation by anAngle (zero in the current isConformal decomposition)
    // and uniform scaling.
    aTrsf.SetMirror (gp_Ax2d (gp_Pnt2d (0.0, 0.0), gp_Dir2d (aReflex.x, aReflex.y)));
    gp_Trsf2d aRot;
    aRot.SetRotation (gp_Pnt2d (0.0, 0.0), anAngle);
    aTrsf.Multiply (aRot);
    aTrsf.SetScaleFactor (aScale);
    aTrsf.SetTranslationPart (gp_Vec2d (theMat (0, 2), theMat (1, 2)));
    return aTrsf;
  }
  aTrsf.SetRotation (gp_Pnt2d (0.0, 0.0), anAngle);
  aTrsf.SetScaleFactor (aScale);
  aTrsf.SetTranslationPart (gp_Vec2d (theMat (0, 2), theMat (1, 2)));
  return aTrsf;
}

//! Converts a conformal 3d matrix into a gp_Trsf; throws NvException
//! otherwise (same rationale as Trsf2dFromMatrix).
inline gp_Trsf TrsfFromMatrix (const NvGeMatrix3d& theMat)
{
  gp_Trsf aTrsf;
  // gp_Trsf::SetValues validates similarity; non-conformal matrices raise
  // Standard_ConstructionError, translated to the project error type.
  try
  {
    aTrsf.SetValues (theMat (0, 0), theMat (0, 1), theMat (0, 2), theMat (0, 3),
                     theMat (1, 0), theMat (1, 1), theMat (1, 2), theMat (1, 3),
                     theMat (2, 0), theMat (2, 1), theMat (2, 2), theMat (2, 3));
  }
  catch (const Standard_Failure&)
  {
    throw NvException ("NvGe: the matrix is not conformal; analytic entities"
                       " cannot absorb shear or non-uniform scaling");
  }
  return aTrsf;
}

//! Applies a general 2d matrix to a 2d point (column-vector convention).
inline gp_Pnt2d ApplyMatrix2d (const NvGeMatrix2d& theMat, const gp_Pnt2d& thePnt)
{
  return gp_Pnt2d (theMat (0, 0) * thePnt.X() + theMat (0, 1) * thePnt.Y() + theMat (0, 2),
                   theMat (1, 0) * thePnt.X() + theMat (1, 1) * thePnt.Y() + theMat (1, 2));
}

//! Applies a general 3d matrix to a point (column-vector convention).
inline gp_Pnt ApplyMatrix3d (const NvGeMatrix3d& theMat, const gp_Pnt& thePnt)
{
  return gp_Pnt (theMat (0, 0) * thePnt.X() + theMat (0, 1) * thePnt.Y() + theMat (0, 2) * thePnt.Z() + theMat (0, 3),
                 theMat (1, 0) * thePnt.X() + theMat (1, 1) * thePnt.Y() + theMat (1, 2) * thePnt.Z() + theMat (1, 3),
                 theMat (2, 0) * thePnt.X() + theMat (2, 1) * thePnt.Y() + theMat (2, 2) * thePnt.Z() + theMat (2, 3));
}

//! Applies a general 3d matrix to a vector (linear part only).
inline gp_Vec ApplyMatrix3d (const NvGeMatrix3d& theMat, const gp_Vec& theVec)
{
  return gp_Vec (theMat (0, 0) * theVec.X() + theMat (0, 1) * theVec.Y() + theMat (0, 2) * theVec.Z(),
                 theMat (1, 0) * theVec.X() + theMat (1, 1) * theVec.Y() + theMat (1, 2) * theVec.Z(),
                 theMat (2, 0) * theVec.X() + theMat (2, 1) * theVec.Y() + theMat (2, 2) * theVec.Z());
}

// =====================================================================
// Shared helpers
// =====================================================================

//! Returns kTrue when theType equals theParent or derives from it through
//! the entity id hierarchy. Defined in geent2d.cpp.
bool NvGeIsKindOf (NvGe::EntityId theType, NvGe::EntityId theParent);

//! Extracts the Geom2d_Curve stored in a 2d entity impl; throws when the
//! impl holds something else.
inline occ::handle<Geom2d_Curve> NvGeCurve2dOf (const NvGeImpEntity3d* theImp)
{
  if (theImp == nullptr)
  {
    throw NvException ("NvGe: the entity has no implementation object");
  }
  occ::handle<Geom2d_Curve> aCurve = occ::down_cast<Geom2d_Curve> (theImp->Geom());
  if (aCurve.IsNull())
  {
    throw NvException ("NvGe: the entity holds no 2d curve geometry");
  }
  return aCurve;
}

//! Extracts the Geom_Curve stored in a 3d entity impl; throws otherwise.
inline occ::handle<Geom_Curve> NvGeCurve3dOf (const NvGeImpEntity3d* theImp)
{
  if (theImp == nullptr)
  {
    throw NvException ("NvGe: the entity has no implementation object");
  }
  occ::handle<Geom_Curve> aCurve = occ::down_cast<Geom_Curve> (theImp->Geom());
  if (aCurve.IsNull())
  {
    throw NvException ("NvGe: the entity holds no 3d curve geometry");
  }
  return aCurve;
}

//! Extracts the Geom_Surface stored in an entity impl; throws otherwise.
inline occ::handle<Geom_Surface> NvGeSurfaceOf (const NvGeImpEntity3d* theImp)
{
  if (theImp == nullptr)
  {
    throw NvException ("NvGe: the entity has no implementation object");
  }
  occ::handle<Geom_Surface> aSurface = occ::down_cast<Geom_Surface> (theImp->Geom());
  if (aSurface.IsNull())
  {
    throw NvException ("NvGe: the entity holds no surface geometry");
  }
  return aSurface;
}

//! Downcasts entity geometry to a custom holder; returns nullptr when the
//! geometry is of another kind. The returned pointer is non-owning: the
//! impl keeps the holder alive.
template <typename TheData>
TheData* NvGeDataOf (const occ::handle<Standard_Transient>& theGeom)
{
  return dynamic_cast<TheData*> (theGeom.get());
}

#endif // NV_GeImpData_HeaderFile
