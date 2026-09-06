#ifndef NV_GEVEC2D_H
#define NV_GEVEC2D_H

#include "Nova.h"
#include "gegbl.h"
#include "gegblabb.h"
#pragma pack (push, 8)

class NvGeMatrix2d;

class 
NvGeVector2d
{
public:
    GE_DLLEXPIMPORT NvGeVector2d();
    GE_DLLEXPIMPORT NvGeVector2d(const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d(double x, double y);

    // The additive identity, x-axis, and y-axis.
    //
    GE_DLLDATAEXIMP static const   NvGeVector2d kIdentity;
    GE_DLLDATAEXIMP static const   NvGeVector2d kXAxis;
    GE_DLLDATAEXIMP static const   NvGeVector2d kYAxis;

    // Matrix multiplication.
    //
    friend GE_DLLEXPIMPORT
    NvGeVector2d   operator *  (const NvGeMatrix2d& mat, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d&  setToProduct(const NvGeMatrix2d& mat, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d&  transformBy (const NvGeMatrix2d& leftSide);
    GE_DLLEXPIMPORT NvGeVector2d&  rotateBy    (double angle);
    GE_DLLEXPIMPORT NvGeVector2d&  mirror      (const NvGeVector2d& line );


    // Scale multiplication.
    //
    GE_DLLEXPIMPORT NvGeVector2d   operator *  (double scl) const;
    friend GE_DLLEXPIMPORT
    NvGeVector2d   operator *  (double scl, const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d&  operator *= (double scl);
    GE_DLLEXPIMPORT NvGeVector2d&  setToProduct(const NvGeVector2d& vec, double scl);
    GE_DLLEXPIMPORT NvGeVector2d   operator /  (double scl) const;
    GE_DLLEXPIMPORT NvGeVector2d&  operator /= (double scl);

    // Addition and subtraction of vectors.
    //
    GE_DLLEXPIMPORT NvGeVector2d   operator +  (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT NvGeVector2d&  operator += (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d   operator -  (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT NvGeVector2d&  operator -= (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d&  setToSum    (const NvGeVector2d& vec1, const NvGeVector2d& vec2);
    GE_DLLEXPIMPORT NvGeVector2d   operator -  () const;
    GE_DLLEXPIMPORT NvGeVector2d&  negate      ();

    // Perpendicular vector
    //
    GE_DLLEXPIMPORT NvGeVector2d   perpVector  () const;

    // Angle argument.
    //
    GE_DLLEXPIMPORT double         angle       () const;
    GE_DLLEXPIMPORT double         angleTo     (const NvGeVector2d& vec) const;

    // Vector length operations.
    //
    GE_DLLEXPIMPORT NvGeVector2d   normal      (const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGeVector2d&  normalize   (const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeVector2d&  normalize   (const NvGeTol& tol, NvGeError& flag);
        // Possible errors:  k0This.  Returns object unchanged on error. 
    GE_DLLEXPIMPORT double         length      () const;
    GE_DLLEXPIMPORT double         lengthSqrd  () const;
    GE_DLLEXPIMPORT Nova::Boolean isUnitLength(const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isZeroLength(const NvGeTol& tol = NvGeContext::gTol) const;

    // Direction tests.
    //
    GE_DLLEXPIMPORT Nova::Boolean isParallelTo(const NvGeVector2d& vec,
                                const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isParallelTo(const NvGeVector2d& vec,
                                const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors:  k0This, k0Arg1. 
        // Returns kFalse on error.
    GE_DLLEXPIMPORT Nova::Boolean isCodirectionalTo(const NvGeVector2d& vec,
                                const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isCodirectionalTo(const NvGeVector2d& vec,
                        const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors:  k0This, k0Arg1. 
        // Returns kFalse on error.
    GE_DLLEXPIMPORT Nova::Boolean isPerpendicularTo(const NvGeVector2d& vec,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isPerpendicularTo(const NvGeVector2d& vec,
                        const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors:  k0This, k0Arg1. 
        // Returns kFalse on error.

    // Dot product.
    //
    GE_DLLEXPIMPORT double         dotProduct  (const NvGeVector2d& vec) const;

    // Tests for equivalence using the Euclidean norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT bool operator != (const NvGeVector2d& vec) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGeVector2d& vec,
                      const NvGeTol& tol = NvGeContext::gTol) const;

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int i) ;
    GE_DLLEXPIMPORT NvGeVector2d&  set         (double x, double y);

    // Convert to/from matrix form.
    //
    GE_DLLEXPIMPORT operator       NvGeMatrix2d() const;

    // Co-ordinates.
    //
    double         x, y;
};

// Creates the identity translation vector.
//
inline
NvGeVector2d::NvGeVector2d() : x(0.0), y(0.0)
{
}

inline
NvGeVector2d::NvGeVector2d(const NvGeVector2d& src) : x(src.x), y(src.y)
{
}

// Creates a vector intialized to ( xx, yy ).
//
inline
NvGeVector2d::NvGeVector2d(double xx, double yy) : x(xx), y(yy)
{
}

inline bool
NvGeVector2d::operator == (const NvGeVector2d& v) const
{
    return this->isEqualTo(v);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeVector2d::operator != (const NvGeVector2d& v) const
{
    return !this->isEqualTo(v);
}

// This operator returns a vector that is the scalar product of
// `s' and this vector.
//
inline NvGeVector2d
NvGeVector2d::operator * (double s) const
{
    return NvGeVector2d (x * s, y * s);
}

// This is equivalent to the statement `v = v * s'.
//
inline NvGeVector2d&
NvGeVector2d::operator *= (double s)
{
    x *= s;
    y *= s;
    return *this;
}

inline NvGeVector2d&
NvGeVector2d::setToProduct(const NvGeVector2d& v, double s)
{
    x = s * v.x;
    y = s * v.y;
    return *this;
}

// Returns a vector such that each of the coordinates of this vector
// have been divided by val.
//
inline NvGeVector2d
NvGeVector2d::operator / (double val) const
{
    return NvGeVector2d (x/val, y/val);
}

// This is equivalent to the statement `v = v / val;'
// Each coordinate of this vector is divided by val.
//
inline NvGeVector2d&
NvGeVector2d::operator /= (double val)
{
    x /= val;
    y /= val;
    return *this;
}

// Returns a vector that is formed from adding the components of
// this vector with `v'.
//
inline NvGeVector2d
NvGeVector2d::operator + (const NvGeVector2d& v) const
{
    return NvGeVector2d (x + v.x, y + v.y);
}

// This is equivalent to the statement `thisVec = thisVec + v;'
//
inline NvGeVector2d&
NvGeVector2d::operator += (const NvGeVector2d& v)
{
    x += v.x;
    y += v.y;
    return *this;
}

// Using this operator is equivalent to using `thisVec + (-v);'
//
inline NvGeVector2d
NvGeVector2d::operator - (const NvGeVector2d& v) const
{
    return NvGeVector2d (x - v.x, y - v.y);
}

// This is equivalent to the statement `thisVec = thisVec - v;'
//
inline NvGeVector2d&
NvGeVector2d::operator -= (const NvGeVector2d& v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

inline NvGeVector2d&
NvGeVector2d::setToSum(const NvGeVector2d& v1, const NvGeVector2d& v2)
{
    x = v1.x + v2.x;
    y = v1.y + v2.y;
    return *this;
}

// Returns a vector that is formed by negating each of the components
// of this vector.
//
inline NvGeVector2d
NvGeVector2d::operator - () const
{
    return NvGeVector2d (-x, -y);
}

// `v.negate()' is equivalent to the statement `v = -v;'
//
inline NvGeVector2d&
NvGeVector2d::negate()
{
    x = -x;
    y = -y;
    return *this;
}

// Returns a vector orthogonal to this vector.
//
inline NvGeVector2d
NvGeVector2d::perpVector() const
{
    return NvGeVector2d (-y, x);
}

// Returns the square of the Euclidean length of this vector.
//
inline double
NvGeVector2d::lengthSqrd() const
{
    return x*x + y*y;
}

// Returns the dot product of this vector and `v'.
//
inline double
NvGeVector2d::dotProduct(const NvGeVector2d& v) const
{
    return x * v.x + y * v.y;
}

// Sets the vector to ( xx, yy ).
//
inline NvGeVector2d&
NvGeVector2d::set(double xx, double yy)
{
    x = xx;
    y = yy;
    return *this;
}

// Indexes the vector as if it were an array.  `x' is index `0',
// `y' is index `1'.
//
inline double
NvGeVector2d::operator [] (unsigned int i) const
{
    return *(&x+i);
}

inline double&
NvGeVector2d::operator [] (unsigned int i)
{
    return *(&x+i);
}

#define NOVA_NVGEVECTOR2D_DEFINED
#include "nvarrayhelper.h"

#pragma pack (pop)
#endif
