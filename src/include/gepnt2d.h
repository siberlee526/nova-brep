#ifndef NV_GEPNT2D_H
#define NV_GEPNT2D_H

#include "gevec2d.h"
#pragma pack (push, 8)

class NvGeMatrix2d;
class NvGeVector2d;
class NvGeLinearEnt2d;
class NvGeLine2d;

class
NvGePoint2d
{
public:
    GE_DLLEXPIMPORT NvGePoint2d();
    GE_DLLEXPIMPORT NvGePoint2d(const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT NvGePoint2d(double x, double y);

    // The origin, or (0, 0).
    //
    GE_DLLDATAEXIMP static const   NvGePoint2d kOrigin;

    // Matrix multiplication.
    //
    friend GE_DLLEXPIMPORT
    NvGePoint2d    operator *  (const NvGeMatrix2d& mat, const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT NvGePoint2d&   setToProduct(const NvGeMatrix2d& mat, const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT NvGePoint2d&   transformBy (const NvGeMatrix2d& leftSide);
    GE_DLLEXPIMPORT NvGePoint2d&   rotateBy    (double angle, const NvGePoint2d& wrtPoint
                                = NvGePoint2d::kOrigin);
    GE_DLLEXPIMPORT NvGePoint2d&   mirror      (const NvGeLine2d& line);
    GE_DLLEXPIMPORT NvGePoint2d&   scaleBy     (double scaleFactor, const NvGePoint2d& wrtPoint
                                = NvGePoint2d::kOrigin);

    // Scale multiplication.
    //
    GE_DLLEXPIMPORT NvGePoint2d    operator *  (double) const;
    GE_DLLEXPIMPORT friend
    NvGePoint2d    operator *  (double, const NvGePoint2d& scl);
    GE_DLLEXPIMPORT NvGePoint2d&   operator *= (double scl);
    GE_DLLEXPIMPORT NvGePoint2d    operator /  (double scl) const;
    GE_DLLEXPIMPORT NvGePoint2d&   operator /= (double scl);

    // Translation by a vector.
    //
    GE_DLLEXPIMPORT NvGePoint2d    operator +  (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT NvGePoint2d&   operator += (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGePoint2d    operator -  (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT NvGePoint2d&   operator -= (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGePoint2d&   setToSum    (const NvGePoint2d& pnt, const NvGeVector2d& vec);

    // Get vector between two points.
    //
    GE_DLLEXPIMPORT NvGeVector2d   operator -  (const NvGePoint2d& pnt) const;
    GE_DLLEXPIMPORT NvGeVector2d   asVector    () const;

    // Distance to other geometric objects.
    //
    GE_DLLEXPIMPORT double         distanceTo  (const NvGePoint2d& pnt) const;

    // Tests for equivalence using the Euclidean norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGePoint2d& pnt) const;
    GE_DLLEXPIMPORT bool operator != (const NvGePoint2d& pnt) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGePoint2d& pnt,
                      const NvGeTol& tol = NvGeContext::gTol) const;

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int idx);
    GE_DLLEXPIMPORT NvGePoint2d&   set         (double x, double y);

    // The co-ordinates of the point.
    //
    double         x, y;
};

// Creates a point at the origin.
//
inline
NvGePoint2d::NvGePoint2d() : x(0.0), y(0.0)
{
}

inline
NvGePoint2d::NvGePoint2d(const NvGePoint2d& src) : x(src.x), y(src.y)
{
}

// Creates a point intialized to ( xx, yy ).
//
inline
NvGePoint2d::NvGePoint2d(double xx, double yy) : x(xx), y(yy)
{
}

inline bool
NvGePoint2d::operator == (const NvGePoint2d& p) const
{
    return this->isEqualTo(p);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGePoint2d::operator != (const NvGePoint2d& p) const
{
    return !this->isEqualTo(p);
}

// Returns a point such that each of the coordinates of this point
// have been multiplied by val.
//
inline NvGePoint2d
NvGePoint2d::operator * (double val) const
{
    return NvGePoint2d(x*val, y*val);
}

// Returns a point such that each of the coordinates of this point
// have been multiplied by val.
//
inline NvGePoint2d
operator * (double val, const NvGePoint2d& p)
{
    return NvGePoint2d(p.x*val, p.y*val);
}

// This is equivalent to the statement `p = p * val;'
// Each coordinate of this point is multiplied by val.
//
inline NvGePoint2d&
NvGePoint2d::operator *= (double val)
{
    x *= val;
    y *= val;
    return *this;
}

// Returns a point such that each of the coordinates of this point
// have been divided by val.
//
inline NvGePoint2d
NvGePoint2d::operator / (double val) const
{
    return NvGePoint2d (x/val, y/val);
}

// This is equivalent to the statement `p = p / val;'
// Each coordinate of this point is divided by val.
//
inline NvGePoint2d&
NvGePoint2d::operator /= (double val)
{
    x /= val;
    y /= val;
    return *this;
}

// Returns a point that is equivalent to the result of translating
// this point by the vector `v'.  (It yields the same result as if
// the vector had been cast to a translation matrix and then
// multiplied with the point.)
//
inline NvGePoint2d
NvGePoint2d::operator + (const NvGeVector2d& v) const
{
    return NvGePoint2d(x + v.x, y + v.y);
}

// This is equivalent to the statement `p = p + v;'
//
inline NvGePoint2d&
NvGePoint2d::operator += (const NvGeVector2d& v)
{
    x += v.x;
    y += v.y;
    return *this;
}

// This is equivalent to the statement `p + (-v);'
//
inline NvGePoint2d
NvGePoint2d::operator - (const NvGeVector2d& v) const
{
    return NvGePoint2d(x - v.x, y - v.y);
}

// This is equivalent to the statement `p = p - v;'
//
inline NvGePoint2d&
NvGePoint2d::operator -= (const NvGeVector2d& v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

// This operator returns a vector such that if `v = p1 - p0',
// then, `v' is equivalent to the translation that takes
// `p0' to `p1'.  (This point is `p1').
//
inline NvGeVector2d
NvGePoint2d::operator - (const NvGePoint2d& p) const
{
    return NvGeVector2d(x - p.x, y - p.y);
}

// This operator returns the vector that would have resulted
// from the operation `p1 - NvGePoint2d::kOrigin', which is
// a common operation to perform.
//
inline NvGeVector2d
NvGePoint2d::asVector() const
{
    return NvGeVector2d(x, y);
}

// Sets the point to ( xx, yy ).
//
inline NvGePoint2d&
NvGePoint2d::set(double xx, double yy)
{
    x = xx;
    y = yy;
    return *this;
}

//     Indexes the point as if it were an array.  `x' is index `0',
//     `y' is index `1'.
//
inline double
NvGePoint2d::operator [] (unsigned int i) const
{
    return *(&x+i);
}

inline double&
NvGePoint2d::operator [] (unsigned int i)
{
    return *(&x+i);
}

#define NOVA_NVEPOINT2D_DEFINED
#include "nvarrayhelper.h"

#pragma pack (pop)
#endif
