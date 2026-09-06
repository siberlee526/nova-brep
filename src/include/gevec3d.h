#ifndef NV_GEVEC3D_H
#define NV_GEVEC3D_H

#include "Nova.h"
#include "gegbl.h"
#include "gegblabb.h"
#pragma pack (push, 8)

class NvGeMatrix3d;
class NvGeVector2d;
class NvGePlane;
class NvGePlanarEnt;

class 
NvGeVector3d
{
public:
    GE_DLLEXPIMPORT NvGeVector3d();
    GE_DLLEXPIMPORT NvGeVector3d(const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d(double x, double y, double z);
    GE_DLLEXPIMPORT NvGeVector3d(const NvGePlanarEnt&, const NvGeVector2d&);

    // The additive identity, x-axis, y-axis, and z-axis.
    //
    GE_DLLDATAEXIMP static const   NvGeVector3d kIdentity;
    GE_DLLDATAEXIMP static const   NvGeVector3d kXAxis;
    GE_DLLDATAEXIMP static const   NvGeVector3d kYAxis;
    GE_DLLDATAEXIMP static const   NvGeVector3d kZAxis;

    // Multiplication with matrices.
    //
    friend GE_DLLEXPIMPORT
    NvGeVector3d   operator *  (const NvGeMatrix3d& mat, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d&  setToProduct(const NvGeMatrix3d& mat, const NvGeVector3d& vec);

    GE_DLLEXPIMPORT NvGeVector3d&  transformBy (const NvGeMatrix3d& leftSide);
    GE_DLLEXPIMPORT NvGeVector3d&  rotateBy    (double ang , const NvGeVector3d& axis );
    GE_DLLEXPIMPORT NvGeVector3d&  mirror      (const NvGeVector3d& normalToPlane);
    GE_DLLEXPIMPORT NvGeVector2d   convert2d   (const NvGePlanarEnt& pln) const;

    // Multiplication by scalar.
    //
    GE_DLLEXPIMPORT NvGeVector3d   operator *  (double scl) const;
    friend GE_DLLEXPIMPORT
    NvGeVector3d   operator *  (double scl, const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d&  operator *= (double scl);
    GE_DLLEXPIMPORT NvGeVector3d&  setToProduct(const NvGeVector3d& vec, double scl);
    GE_DLLEXPIMPORT NvGeVector3d   operator /  (double scl) const;
    GE_DLLEXPIMPORT NvGeVector3d&  operator /= (double scl);

    // Addition and subtraction of vectors.
    //
    GE_DLLEXPIMPORT NvGeVector3d   operator +  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGeVector3d&  operator += (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d   operator -  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGeVector3d&  operator -= (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d&  setToSum    (const NvGeVector3d& vec1, const NvGeVector3d& vec2);
    GE_DLLEXPIMPORT NvGeVector3d   operator -  () const;
    GE_DLLEXPIMPORT NvGeVector3d&  negate      ();

    // Perpendicular vector
    //
    GE_DLLEXPIMPORT NvGeVector3d   perpVector  () const;

    // Angle argument.
    //
    GE_DLLEXPIMPORT double         angleTo     (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT double         angleTo     (const NvGeVector3d& vec,
                                const NvGeVector3d& refVec) const;
    GE_DLLEXPIMPORT double         angleOnPlane(const NvGePlanarEnt& pln) const;

    // Vector length operations.
    //
    GE_DLLEXPIMPORT NvGeVector3d   normal      (const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT NvGeVector3d&  normalize   (const NvGeTol& tol = NvGeContext::gTol);
    GE_DLLEXPIMPORT NvGeVector3d&  normalize   (const NvGeTol& tol, NvGeError& flag);
        // Possible errors:  k0This.  Returns object unchanged on error. 
    GE_DLLEXPIMPORT double         length      () const;
    GE_DLLEXPIMPORT double         lengthSqrd  () const;
    GE_DLLEXPIMPORT Nova::Boolean isUnitLength(const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isZeroLength(const NvGeTol& tol = NvGeContext::gTol) const;

    // Direction tests.
    //
    GE_DLLEXPIMPORT Nova::Boolean isParallelTo(const NvGeVector3d& vec,
                                const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isParallelTo(const NvGeVector3d& vec,
                                const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors:  k0This, k0Arg1. 
        // Returns kFalse on error.
    GE_DLLEXPIMPORT Nova::Boolean isCodirectionalTo(const NvGeVector3d& vec,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isCodirectionalTo(const NvGeVector3d& vec,
                                     const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors: k0Arg1, k0Arg2, kPerpendicularArg1Arg2. 
        // Returns copy of unchanged object on error.
    GE_DLLEXPIMPORT Nova::Boolean isPerpendicularTo(const NvGeVector3d& vec,
                                     const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Nova::Boolean isPerpendicularTo(const NvGeVector3d& vec,
                                     const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors: k0Arg1, k0Arg2, kPerpendicularArg1Arg2. 
        // Returns copy of unchanged object on error.

    // Dot and Cross product.
    //
    GE_DLLEXPIMPORT double         dotProduct  (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT NvGeVector3d   crossProduct(const NvGeVector3d& vec) const;

    // Calculate rotation.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d   rotateTo    (const NvGeVector3d& vec, const NvGeVector3d& axis
                                = NvGeVector3d::kIdentity) const;

    // Projection on plane
    //
    GE_DLLEXPIMPORT NvGeVector3d   project      (const NvGeVector3d& planeNormal,
                                 const NvGeVector3d& projectDirection) const;
    GE_DLLEXPIMPORT NvGeVector3d   project      (const NvGeVector3d& planeNormal,
                                 const NvGeVector3d& projectDirection,  
                                 const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors: k0Arg1, k0Arg2, kPerpendicularArg1Arg2. 
        // Returns copy of unchanged object on error.
    GE_DLLEXPIMPORT NvGeVector3d   orthoProject (const NvGeVector3d& planeNormal) const;
    GE_DLLEXPIMPORT NvGeVector3d   orthoProject (const NvGeVector3d& planeNormal,
                                 const NvGeTol& tol, NvGeError& flag) const;
        // Possible errors:  k0Arg1.
        // Returns a copy of unchanged object on error. 

    // Tests for equivalence using the Euclidean norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT bool operator != (const NvGeVector3d& vec) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGeVector3d& vec,
                      const NvGeTol& tol = NvGeContext::gTol) const;

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int i);
    GE_DLLEXPIMPORT unsigned int   largestElement() const;
    GE_DLLEXPIMPORT NvGeVector3d&  set         (double x, double y, double z);
    GE_DLLEXPIMPORT NvGeVector3d&  set         (const NvGePlanarEnt& pln, const NvGeVector2d& vec);

    // Convert to the matrix of translation.
    //
    GE_DLLEXPIMPORT operator       NvGeMatrix3d() const;

    // Co-ordinates.
    //
    double         x, y, z;
};

// Creates the identity vector.
//
inline
NvGeVector3d::NvGeVector3d() : x(0.0), y(0.0), z(0.0)
{
}

inline
NvGeVector3d::NvGeVector3d(const NvGeVector3d& src)
{
    constexpr size_t sizeOfDouble = sizeof(double);
    memcpy_s(&x, sizeOfDouble, &(src.x), sizeOfDouble);
    memcpy_s(&y, sizeOfDouble, &(src.y), sizeOfDouble);
    memcpy_s(&z, sizeOfDouble, &(src.z), sizeOfDouble);
}

// Creates a vector intialized to ( xx, yy, zz ).
//
inline
NvGeVector3d::NvGeVector3d(double xx, double yy, double zz) : x(xx),y(yy),z(zz)
{
}

inline bool
NvGeVector3d::operator == (const NvGeVector3d& v) const
{
    return this->isEqualTo(v);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeVector3d::operator != (const NvGeVector3d& v) const
{
    return !this->isEqualTo(v);
}

// Returns a vector that is formed from adding the components of
// this vector with `v'.
//
inline NvGeVector3d
NvGeVector3d::operator + (const NvGeVector3d& v) const
{
    return NvGeVector3d (x + v.x, y + v.y, z + v.z);
}

// This is equivalent to the statement `thisVec = thisVec + v;'
//
inline NvGeVector3d&
NvGeVector3d::operator += (const NvGeVector3d& v)
{
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

// Using this operator is equivalent to using `thisVec + (-v);'
//
inline NvGeVector3d
NvGeVector3d::operator - (const NvGeVector3d& v) const
{
    return NvGeVector3d (x - v.x, y - v.y, z - v.z);
}

// This is equivalent to the statement `thisVec = thisVec - v;'
//
inline NvGeVector3d&
NvGeVector3d::operator -= (const NvGeVector3d& v)
{
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

inline NvGeVector3d&
NvGeVector3d::setToSum(const NvGeVector3d& v1, const NvGeVector3d& v2)
{
    x = v1.x + v2.x;
    y = v1.y + v2.y;
    z = v1.z + v2.z;
    return *this;
}

// Returns a vector that is formed by negating each of the components
// of this vector.
//
inline NvGeVector3d
NvGeVector3d::operator - () const
{
    return NvGeVector3d (-x, -y, -z);
}

// `v.negate()' is equivalent to the statement `v = -v;'
//
inline NvGeVector3d&
NvGeVector3d::negate()
{
    x = -x;
    y = -y;
    z = -z;
    return *this;
}

// This operator returns a vector that is the scalar product of
// `s' and this vector.
//
inline NvGeVector3d
NvGeVector3d::operator * (double s) const
{
    return NvGeVector3d (x * s, y * s, z * s);
}

// This is equivalent to the statement `v = v * s'.
//
inline NvGeVector3d&
NvGeVector3d::operator *= (double s)
{
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

inline NvGeVector3d&
NvGeVector3d::setToProduct(const NvGeVector3d& v, double s)
{
    x = s * v.x;
    y = s * v.y;
    z = s * v.z;
    return *this;
}

// Returns a vector such that each of the coordinates of this vector
// have been divided by val.
//
inline NvGeVector3d
NvGeVector3d::operator / (double val) const
{
    return NvGeVector3d (x/val, y/val, z/val);
}

// This is equivalent to the statement `v = v / val;'
// Each coordinate of this vector is divided by val.
//
inline NvGeVector3d&
NvGeVector3d::operator /= (double val)
{
    x /= val;
    y /= val;
    z /= val;
    return *this;
}

// Returns the square of the Euclidean length of this vector.
//
inline double
NvGeVector3d::lengthSqrd() const
{
    return x*x + y*y + z*z;
}

// Returns the dot product of this vector and `v'.
//
inline double
NvGeVector3d::dotProduct(const NvGeVector3d& v) const
{
    return x * v.x + y * v.y + z * v.z;
}

// Sets the vector to ( xx, yy, zz ).
//
inline NvGeVector3d&
NvGeVector3d::set(double xx, double yy, double zz)
{
    x = xx;
    y = yy;
    z = zz;
    return *this;
}

// Indexes the vector as if it were an array.  `x' is index `0',
// `y' is index `1' and `z' is index `2'.
//
inline double
NvGeVector3d::operator [] (unsigned int i) const
{
    return *(&x+i);
}

inline double& NvGeVector3d::operator [] (unsigned int i)
{
    return *(&x+i);
}
#define NOVA_NVGEVECTOR3D_DEFINED
#include "nvarrayhelper.h"

#pragma pack (pop)
#endif
