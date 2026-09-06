#ifndef NV_GEMAT2D_H
#define NV_GEMAT2D_H

#include "gegbl.h"
#include "gepnt2d.h"
#pragma pack (push, 8)

class NvGeVector2d;
class NvGeLine2d;
class NvGeTol;

class 
NvGeMatrix2d
{
public:
    GE_DLLEXPIMPORT NvGeMatrix2d();
    GE_DLLEXPIMPORT NvGeMatrix2d(const NvGeMatrix2d& src);

    // The multiplicative identity.
    //
    GE_DLLDATAEXIMP static const   NvGeMatrix2d   kIdentity;

    // Reset matrix.
    //
    GE_DLLEXPIMPORT NvGeMatrix2d&  setToIdentity();

    // Multiplication.
    //
    GE_DLLEXPIMPORT NvGeMatrix2d   operator *   (const NvGeMatrix2d& mat) const;
    GE_DLLEXPIMPORT NvGeMatrix2d&  operator *=  (const NvGeMatrix2d& mat);
    GE_DLLEXPIMPORT NvGeMatrix2d&  preMultBy    (const NvGeMatrix2d& leftSide);
    GE_DLLEXPIMPORT NvGeMatrix2d&  postMultBy   (const NvGeMatrix2d& rightSide);
    GE_DLLEXPIMPORT NvGeMatrix2d&  setToProduct (const NvGeMatrix2d& mat1, const NvGeMatrix2d& mat2);

    // Multiplicative inverse.
    //
    GE_DLLEXPIMPORT NvGeMatrix2d&  invert       ();
    GE_DLLEXPIMPORT NvGeMatrix2d   inverse      () const;

    // Test if it is a singular matrix. A singular matrix is not invertable.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isSingular   (const NvGeTol& tol = NvGeContext::gTol) const;

    // Matrix transposition.
    //
    GE_DLLEXPIMPORT NvGeMatrix2d&  transposeIt  ();
    GE_DLLEXPIMPORT NvGeMatrix2d   transpose    () const;

    // Tests for equivalence using the infinity norm.
    //
    GE_DLLEXPIMPORT bool operator ==  (const NvGeMatrix2d& mat) const;
    GE_DLLEXPIMPORT bool operator !=  (const NvGeMatrix2d& mat) const;
    GE_DLLEXPIMPORT bool isEqualTo    (const NvGeMatrix2d& mat,
                       const NvGeTol& tol = NvGeContext::gTol) const;

    // Test scaling effects of matrix
    //
    GE_DLLEXPIMPORT Adesk::Boolean isUniScaledOrtho(const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isScaledOrtho(const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT double		   scale(void);

    // Determinant
    //
    GE_DLLEXPIMPORT double         det          () const;

    // Set/retrieve translation.
    //
    GE_DLLEXPIMPORT NvGeMatrix2d&  setTranslation(const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeVector2d   translation  () const;

    // Retrieve scaling, rotation, mirror components
    //
    GE_DLLEXPIMPORT Adesk::Boolean isConformal (double& scale, double& angle,
                                Adesk::Boolean& isMirror, NvGeVector2d& reflex) const;


    // Set/get coordinate system
    //
    GE_DLLEXPIMPORT NvGeMatrix2d&  setCoordSystem(const NvGePoint2d& origin,
                                 const NvGeVector2d& e0,
                                 const NvGeVector2d& e1);
    GE_DLLEXPIMPORT void           getCoordSystem(NvGePoint2d& origin,
                                 NvGeVector2d& e0,
                                 NvGeVector2d& e1) const;

    // Set the matrix to be a specified transformation
    //
    GE_DLLEXPIMPORT NvGeMatrix2d& setToTranslation(const NvGeVector2d& vec);
    GE_DLLEXPIMPORT NvGeMatrix2d& setToRotation (double angle,
                                 const NvGePoint2d& center
				 = NvGePoint2d::kOrigin);
    GE_DLLEXPIMPORT NvGeMatrix2d& setToScaling  (double scaleAll,
                                 const NvGePoint2d& center
				 = NvGePoint2d::kOrigin);
    GE_DLLEXPIMPORT NvGeMatrix2d& setToMirroring(const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT NvGeMatrix2d& setToMirroring(const NvGeLine2d& line);
    GE_DLLEXPIMPORT NvGeMatrix2d& setToAlignCoordSys(const NvGePoint2d&  fromOrigin,
                                 const NvGeVector2d& fromE0,
                                 const NvGeVector2d& fromE1,
                                 const NvGePoint2d&  toOrigin,
                                 const NvGeVector2d& toE0,
                                 const NvGeVector2d& toE1);

    // Functions that make a 2d transformation matrix using various approaches
    //
    GE_DLLEXPIMPORT static
    NvGeMatrix2d translation    (const NvGeVector2d& vec);
    GE_DLLEXPIMPORT static
    NvGeMatrix2d rotation       (double angle, const NvGePoint2d& center
				 = NvGePoint2d::kOrigin);
    GE_DLLEXPIMPORT static
    NvGeMatrix2d scaling        (double scaleAll, const NvGePoint2d& center
				 = NvGePoint2d::kOrigin);
    GE_DLLEXPIMPORT static
    NvGeMatrix2d mirroring      (const NvGePoint2d& pnt);
    GE_DLLEXPIMPORT static
    NvGeMatrix2d mirroring      (const NvGeLine2d& line);
    GE_DLLEXPIMPORT static
    NvGeMatrix2d alignCoordSys  (const NvGePoint2d& fromOrigin,
                                 const NvGeVector2d& fromE0,
                                 const NvGeVector2d& fromE1,
                                 const NvGePoint2d&  toOrigin,
                                 const NvGeVector2d& toE0,
                                 const NvGeVector2d& toE1);

    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator ()  (unsigned int, unsigned int) const;
    GE_DLLEXPIMPORT double&        operator ()  (unsigned int, unsigned int);

    // The components of the matrix.
    //
    double         entry[3][3]; // [row][column]
};

inline bool
NvGeMatrix2d::operator == (const NvGeMatrix2d& otherMatrix) const
{
    return this->isEqualTo(otherMatrix);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeMatrix2d::operator != (const NvGeMatrix2d& otherMatrix) const
{
    return !this->isEqualTo(otherMatrix);
}

// Return a reference to the element in position [row][column]
// of the `entry' array.
//
inline double
NvGeMatrix2d::operator () (
    unsigned int row,
    unsigned int column) const
{
    return entry[row][column];
}

inline double&
NvGeMatrix2d::operator () (
    unsigned int row,
    unsigned int column)
{
    return entry[row][column];
}

#pragma pack (pop)
#endif
