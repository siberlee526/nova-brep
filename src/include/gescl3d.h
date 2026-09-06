#ifndef NV_GESCL3D_H
#define NV_GESCL3D_H

#include "Nova.h"
#include "gegbl.h"
#pragma pack (push, 8)

class NvGeMatrix3d;

class 
NvGeScale3d
{
public:
    GE_DLLEXPIMPORT NvGeScale3d();
    GE_DLLEXPIMPORT NvGeScale3d(const NvGeScale3d& src);
    GE_DLLEXPIMPORT NvGeScale3d(double factor);
    GE_DLLEXPIMPORT NvGeScale3d(double xFact, double yFact, double zFact);

    // The identity scaling operation.
    //
    GE_DLLDATAEXIMP static const   NvGeScale3d kIdentity;

    // Multiplication.
    //
    GE_DLLEXPIMPORT NvGeScale3d    operator *  (const NvGeScale3d& sclVec) const;
    GE_DLLEXPIMPORT NvGeScale3d&   operator *= (const NvGeScale3d& scl);
    GE_DLLEXPIMPORT NvGeScale3d&   preMultBy   (const NvGeScale3d& leftSide);
    GE_DLLEXPIMPORT NvGeScale3d&   postMultBy  (const NvGeScale3d& rightSide);
    GE_DLLEXPIMPORT NvGeScale3d&   setToProduct(const NvGeScale3d& sclVec1, const NvGeScale3d& sclVec2);
    GE_DLLEXPIMPORT NvGeScale3d    operator *  (double s) const;
    GE_DLLEXPIMPORT NvGeScale3d&   operator *= (double s);
    GE_DLLEXPIMPORT NvGeScale3d&   setToProduct(const NvGeScale3d& sclVec, double s);
    friend GE_DLLEXPIMPORT
    NvGeScale3d    operator *  (double, const NvGeScale3d& scl);

    // Multiplicative inverse.
    //
    GE_DLLEXPIMPORT NvGeScale3d    inverse        () const;
    GE_DLLEXPIMPORT NvGeScale3d&   invert         ();

    GE_DLLEXPIMPORT Adesk::Boolean isProportional(const NvGeTol& tol = NvGeContext::gTol) const;

    // Tests for equivalence using the infinity norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGeScale3d& sclVec) const;
    GE_DLLEXPIMPORT bool operator != (const NvGeScale3d& sclVec) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGeScale3d& scaleVec,
                      const NvGeTol& tol = NvGeContext::gTol) const;

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int i);
    GE_DLLEXPIMPORT NvGeScale3d&   set         (double sc0, double sc1, double sc2);

    // Conversion to/from matrix form.
    //
    GE_DLLEXPIMPORT operator       NvGeMatrix3d   () const;
    GE_DLLEXPIMPORT void getMatrix(NvGeMatrix3d& mat) const;
    GE_DLLEXPIMPORT NvGeScale3d&   extractScale   ( const NvGeMatrix3d& mat );
    GE_DLLEXPIMPORT NvGeScale3d&   removeScale    ( NvGeMatrix3d& mat );

    // The scale components in x, y and z.
    //
    double         sx, sy, sz;
};

inline bool
NvGeScale3d::operator == (const NvGeScale3d& s) const
{
    return this->isEqualTo(s);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeScale3d::operator != (const NvGeScale3d& s) const
{
    return !(this->isEqualTo(s));
}

// Indexes the scale vector as if it were an array.  `sx' is index `0',
// `sy' is index `1' and `sz' is index `2'.
//
inline double
NvGeScale3d::operator [] (unsigned int i) const
{
    return *(&sx+i);
}

inline double&
NvGeScale3d::operator [] (unsigned int i)
{
    return *(&sx+i);
}

#pragma pack (pop)
#endif
