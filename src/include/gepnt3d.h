#ifndef NV_GEPNT3D_H
#define NV_GEPNT3D_H

#include "gevec3d.h"
#pragma pack (push, 8)

class NvGeMatrix3d;
class NvGeLinearEnt3d;
class NvGePlane;
class NvGePlanarEnt;
class NvGeVector3d;
class NvGePoint2d;

class
NvGePoint3d
{
public:
    GE_DLLEXPIMPORT NvGePoint3d();
    GE_DLLEXPIMPORT NvGePoint3d(const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT NvGePoint3d(double x, double y, double z);
    GE_DLLEXPIMPORT NvGePoint3d(const NvGePlanarEnt& pln, const NvGePoint2d& pnt2d);

    // The origin, or (0, 0, 0).
    //
    GE_DLLDATAEXIMP static const   NvGePoint3d    kOrigin;

    // Matrix multiplication.
    //
    friend GE_DLLEXPIMPORT
    NvGePoint3d    operator *  (const NvGeMatrix3d& mat, const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT NvGePoint3d&   setToProduct(const NvGeMatrix3d& mat, const NvGePoint3d& pnt);

    GE_DLLEXPIMPORT NvGePoint3d&   transformBy (const NvGeMatrix3d& leftSide);
    GE_DLLEXPIMPORT NvGePoint3d&   rotateBy    (double angle, const NvGeVector3d& vec,
                                const NvGePoint3d& wrtPoint = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT NvGePoint3d&   mirror      (const NvGePlane& pln);
    GE_DLLEXPIMPORT NvGePoint3d&   scaleBy     (double scaleFactor, const NvGePoint3d&
                                wrtPoint = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT NvGePoint2d    convert2d   (const NvGePlanarEnt& pln) const;

    // Scale multiplication.
    //
    GE_DLLEXPIMPORT NvGePoint3d    operator *  (double scl) const;
    GE_DLLEXPIMPORT friend
    NvGePoint3d    operator *  (double scl, const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT NvGePoint3d&   operator *= (double scl);
    GE_DLLEXPIMPORT NvGePoint3d    operator /  (double scl) const;
    GE_DLLEXPIMPORT NvGePoint3d&   operator /= (double scl);

    // Translation by a vector.
    //
    GE_DLLEXPIMPORT NvGePoint3d    operator +  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGePoint3d&   operator += (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGePoint3d    operator -  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGePoint3d&   operator -= (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGePoint3d&   setToSum    (const NvGePoint3d& pnt, const NvGeVector3d& vec);

    // Get the vector between two points.
    //
    GE_DLLEXPIMPORT NvGeVector3d   operator -  (const NvGePoint3d& pnt) const;
    GE_DLLEXPIMPORT NvGeVector3d   asVector    () const;

    // Distance to other geometric objects.
    //
    GE_DLLEXPIMPORT double         distanceTo       (const NvGePoint3d& pnt) const;

    // Projection on plane
    //
    GE_DLLEXPIMPORT NvGePoint3d    project       (const NvGePlane& pln, const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGePoint3d    orthoProject  (const NvGePlane& pln) const;

    // Tests for equivalence using the Euclidean norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGePoint3d& pnt) const;
    GE_DLLEXPIMPORT bool operator != (const NvGePoint3d& pnt) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGePoint3d& pnt,
                      const NvGeTol& tol = NvGeContext::gTol) const;

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int idx);
    GE_DLLEXPIMPORT NvGePoint3d&   set         (double x, double y, double z);
    GE_DLLEXPIMPORT NvGePoint3d&   set         (const NvGePlanarEnt& pln, const NvGePoint2d& pnt);

    // The co-ordinates of the point.
    //
    double         x, y, z;
};

// Creates a point at the origin.
//
inline
NvGePoint3d::NvGePoint3d() : x(0.0), y(0.0), z(0.0)
{
}

// Creates a point with the same values as `src'.
//
inline
NvGePoint3d::NvGePoint3d(const NvGePoint3d& src)
{
    constexpr size_t sizeOfDouble = sizeof(double);
    memcpy_s(&x, sizeOfDouble, &(src.x), sizeOfDouble);
    memcpy_s(&y, sizeOfDouble, &(src.y), sizeOfDouble);
    memcpy_s(&z, sizeOfDouble, &(src.z), sizeOfDouble);
}

// Creates a point intialized to ( xx, yy, zz ).
//
inline
NvGePoint3d::NvGePoint3d(double xx, double yy, double zz) : x(xx), y(yy), z(zz)
{
}

inline bool
NvGePoint3d::operator == (const NvGePoint3d& p) const
{
    return this->isEqualTo(p);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGePoint3d::operator != (const NvGePoint3d& p) const
{
    return !this->isEqualTo(p);
}

// Returns a point such that each of the coordinates of this point
// have been multiplied by val.
//
inline NvGePoint3d
NvGePoint3d::operator * (double val) const
{
    return NvGePoint3d(x*val, y*val, z*val);
}

// Returns a point such that each of the coordinates of this point
// have been multiplied by val.
//
inline NvGePoint3d
operator * (double val, const NvGePoint3d& p)
{
    return NvGePoint3d(p.x*val, p.y*val, p.z*val);
}

// This is equivalent to the statement `p = p * val;'
// Each coordinate of this point is multiplied by val.
//
inline NvGePoint3d&
NvGePoint3d::operator *= (double val)
{
    x *= val;
    y *= val;
    z *= val;
    return *this;
}

// Returns a point such that each of the coordinates of this point
// have been divided by val.
//
inline NvGePoint3d
NvGePoint3d::operator / (double val) const
{
    return NvGePoint3d (x/val, y/val, z/val);
}

// This is equivalent to the statement `p = p / val;'
// Each coordinate of this point is divided by val.
//
inline NvGePoint3d&
NvGePoint3d::operator /= (double val)
{
    x /= val;
    y /= val;
    z /= val;
    return *this;
}

// Returns a point that is equivalent to the result of translating
// this point by the vector `v'.  (It yields the same result as if
// the vector had been cast to a translation matrix and then
// multiplied with the point.)
//
inline NvGePoint3d
NvGePoint3d::operator + (const NvGeVector3d& v) const
{
    return NvGePoint3d (x + v.x, y + v.y, z + v.z);
}

// This is equivalent to the statement `p = p + v;'
//
inline NvGePoint3d&
NvGePoint3d::operator += (const NvGeVector3d& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

// This is equivalent to the statement `p + (-v);'
//
inline NvGePoint3d
NvGePoint3d::operator - (const NvGeVector3d& v) const
{
    return NvGePoint3d (x - v.x, y - v.y, z - v.z);
}

// This is equivalent to the statement `p = p - v;'
//
inline NvGePoint3d&
NvGePoint3d::operator -= (const NvGeVector3d& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

// This operator returns a vector such that if `v = p1 - p0',
// then, `v' is equivalent to the translation that takes
// `p0' to `p1'.  (This point is `p1').
//
inline NvGeVector3d
NvGePoint3d::operator - (const NvGePoint3d& p) const
{
    return NvGeVector3d (x - p.x, y - p.y, z - p.z);
}

// This operator returns the vector that would have resulted
// from the operation `p1 - NvGePoint3d::kOrigin', which is
// a common operation to perform.
//
inline NvGeVector3d
NvGePoint3d::asVector() const
{
    return NvGeVector3d(x, y, z);
}

// Sets the point to ( xx, yy, zz ).
//
inline NvGePoint3d&
NvGePoint3d::set(double xx, double yy, double zz)
{
    x = xx;
    y = yy;
    z = zz;
    return *this;
}

// Indexes the point as if it were an array.  `x' is index `0',
// `y' is index `1', `z' is index `2'.
//
inline double
NvGePoint3d::operator [] (unsigned int i) const
{
    return *(&x+i);
}

inline double&
NvGePoint3d::operator [] (unsigned int i)
{
    return *(&x+i);
}

#define ADSK_ACGEPOINT3D_DEFINED
#include "nvarrayhelper.h"

#pragma pack (pop)
#endif
