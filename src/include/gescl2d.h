#ifndef NV_GESCL2D_H
#define NV_GESCL2D_H

#include "Nova.h"
#include "gegbl.h"
#pragma pack (push, 8)

class NvGeMatrix2d;
class NvGeScale3d;

class 
NvGeScale2d
{
public:
    GE_DLLEXPIMPORT NvGeScale2d();
    GE_DLLEXPIMPORT NvGeScale2d(const NvGeScale2d& src);
    GE_DLLEXPIMPORT NvGeScale2d(double factor);
    GE_DLLEXPIMPORT NvGeScale2d(double xFactor, double yFactor);

    // The identity scaling operation.
    //
    GE_DLLDATAEXIMP static const   NvGeScale2d kIdentity;

    // Multiplication.
    //
    GE_DLLEXPIMPORT NvGeScale2d    operator *  (const NvGeScale2d& sclVec) const;
    GE_DLLEXPIMPORT NvGeScale2d&   operator *= (const NvGeScale2d& scl);
    GE_DLLEXPIMPORT NvGeScale2d&   preMultBy   (const NvGeScale2d& leftSide);
    GE_DLLEXPIMPORT NvGeScale2d&   postMultBy  (const NvGeScale2d& rightSide);
    GE_DLLEXPIMPORT NvGeScale2d&   setToProduct(const NvGeScale2d& sclVec1, const NvGeScale2d& sclVec2);
    GE_DLLEXPIMPORT NvGeScale2d    operator *  (double s) const;
    GE_DLLEXPIMPORT NvGeScale2d&   operator *= (double s);
    GE_DLLEXPIMPORT NvGeScale2d&   setToProduct(const NvGeScale2d& sclVec, double s);
    friend GE_DLLEXPIMPORT
    NvGeScale2d    operator *  (double, const NvGeScale2d& scl);

    // Multiplicative inverse.
    //
    GE_DLLEXPIMPORT NvGeScale2d    inverse        () const;
    GE_DLLEXPIMPORT NvGeScale2d&   invert         ();

    GE_DLLEXPIMPORT Nova::Boolean isProportional(const NvGeTol& tol = NvGeContext::gTol) const;

    // Tests for equivalence using the infinity norm.
    //
    GE_DLLEXPIMPORT bool operator == (const NvGeScale2d& sclVec) const;
    GE_DLLEXPIMPORT bool operator != (const NvGeScale2d& sclVec) const;
    GE_DLLEXPIMPORT bool isEqualTo   (const NvGeScale2d& scaleVec,
                      const NvGeTol& tol = NvGeContext::gTol) const;

     // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator [] (unsigned int i) const;
    GE_DLLEXPIMPORT double&        operator [] (unsigned int i);
    GE_DLLEXPIMPORT NvGeScale2d&   set         (double sc0, double sc1);

    // Conversion to/from matrix form.
    //
    GE_DLLEXPIMPORT operator       NvGeMatrix2d   () const;
    GE_DLLEXPIMPORT void           getMatrix      (NvGeMatrix2d& mat) const;
    GE_DLLEXPIMPORT NvGeScale2d&   extractScale   ( const NvGeMatrix2d& mat );
    GE_DLLEXPIMPORT NvGeScale2d&   removeScale    ( NvGeMatrix2d& mat );

    // Cast up to 3d scale.
    //
    GE_DLLEXPIMPORT operator       NvGeScale3d    () const;

    // The scale components in x and y.
    //
    double         sx, sy;
};

inline double
NvGeScale2d::operator [] (unsigned int i) const
{
    return *(&sx+i);
}

inline double&
NvGeScale2d::operator [] (unsigned int i)
{
    return *(&sx+i);
}

inline bool
NvGeScale2d::operator == (const NvGeScale2d& s) const
{
    return this->isEqualTo(s);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeScale2d::operator != (const NvGeScale2d& s) const
{
    return !this->isEqualTo(s);
}

#pragma pack (pop)
#endif
