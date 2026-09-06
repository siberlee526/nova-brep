#ifndef NV_GEMAT3D_H
#define NV_GEMAT3D_H

#include "gegbl.h"
#include "gemat2d.h"
#include "gepnt3d.h"
#pragma pack (push, 8)

class NvGeLine3d;
class NvGeVector3d;
class NvGePlane;
class NvGeTol;

class 
NvGeMatrix3d
{
public:
    GE_DLLEXPIMPORT NvGeMatrix3d();
    GE_DLLEXPIMPORT NvGeMatrix3d(const NvGeMatrix3d& src);

    // The multiplicative identity.
    //
    GE_DLLDATAEXIMP static const   NvGeMatrix3d    kIdentity;

    // Reset matrix.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToIdentity();

    // Multiplication.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d   operator *      (const NvGeMatrix3d& mat) const;
    GE_DLLEXPIMPORT NvGeMatrix3d&  operator *=     (const NvGeMatrix3d& mat);
    GE_DLLEXPIMPORT NvGeMatrix3d&  preMultBy       (const NvGeMatrix3d& leftSide);
    GE_DLLEXPIMPORT NvGeMatrix3d&  postMultBy      (const NvGeMatrix3d& rightSide);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToProduct    (const NvGeMatrix3d& mat1, const NvGeMatrix3d& mat2);

    // Multiplicative inverse.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  invert          ();
    GE_DLLEXPIMPORT NvGeMatrix3d   inverse         () const;
    GE_DLLEXPIMPORT NvGeMatrix3d   inverse         (const NvGeTol& tol) const;
    // Test if it is a singular matrix.  A singular matrix is not invertable.
    //
    GE_DLLEXPIMPORT Adesk::Boolean isSingular      (const NvGeTol& tol = NvGeContext::gTol) const;

    // Matrix transposition.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  transposeIt     ();
    GE_DLLEXPIMPORT NvGeMatrix3d   transpose       () const;

    // Tests for equivalence using the infinity norm.
    //
    GE_DLLEXPIMPORT bool operator ==     (const NvGeMatrix3d& mat) const;
    GE_DLLEXPIMPORT bool operator !=     (const NvGeMatrix3d& mat) const;
    GE_DLLEXPIMPORT bool isEqualTo       (const NvGeMatrix3d& mat, const NvGeTol& tol
                                    = NvGeContext::gTol) const;

    // Test scaling effects of matrix
    //
    GE_DLLEXPIMPORT Adesk::Boolean isUniScaledOrtho(const NvGeTol& tol = NvGeContext::gTol) const;
    GE_DLLEXPIMPORT Adesk::Boolean isScaledOrtho   (const NvGeTol& tol = NvGeContext::gTol) const;

    // Determinant
    //
    GE_DLLEXPIMPORT double         det             () const;

    // Set/retrieve translation.
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  setTranslation  (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeVector3d   translation     () const;

    // Set/get coordinate system
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  setCoordSystem  (const NvGePoint3d& origin,
                                    const NvGeVector3d& xAxis,
                                    const NvGeVector3d& yAxis,
                                    const NvGeVector3d& zAxis);
    GE_DLLEXPIMPORT void           getCoordSystem  (NvGePoint3d& origin,
                                    NvGeVector3d& xAxis,
                                    NvGeVector3d& yAxis,
                                    NvGeVector3d& zAxis) const;

    // Set the matrix to be a specified transformation
    //
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToTranslation(const NvGeVector3d& vec);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToRotation   (double angle, const NvGeVector3d& axis,
                                    const NvGePoint3d& center
                                    = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToScaling    (double scaleAll, const NvGePoint3d& center
                                    = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToMirroring  (const NvGePlane& pln);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToMirroring  (const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToMirroring  (const NvGeLine3d& line);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToProjection (const NvGePlane& projectionPlane,
                                    const NvGeVector3d& projectDir);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToAlignCoordSys(const NvGePoint3d& fromOrigin,
                                    const NvGeVector3d& fromXAxis,
                                    const NvGeVector3d& fromYAxis,
                                    const NvGeVector3d& fromZAxis,
                                    const NvGePoint3d& toOrigin,
                                    const NvGeVector3d& toXAxis,
                                    const NvGeVector3d& toYAxis,
                                    const NvGeVector3d& toZAxis);

    GE_DLLEXPIMPORT NvGeMatrix3d&  setToWorldToPlane(const NvGeVector3d& normal);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToWorldToPlane(const NvGePlane& plane);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToPlaneToWorld(const NvGeVector3d& normal);
    GE_DLLEXPIMPORT NvGeMatrix3d&  setToPlaneToWorld(const NvGePlane& plane);

    // Similar to above, but creates matrix on the stack.
    //
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   translation     (const NvGeVector3d& vec);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   rotation        (double angle, const NvGeVector3d& axis,
                                    const NvGePoint3d& center
                                    = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   scaling         (double scaleAll, const NvGePoint3d& center
                                    = NvGePoint3d::kOrigin);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   mirroring       (const NvGePlane& pln);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   mirroring       (const NvGePoint3d& pnt);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   mirroring       (const NvGeLine3d& line);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   projection      (const NvGePlane& projectionPlane,
                                    const NvGeVector3d& projectDir);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   alignCoordSys   (const NvGePoint3d&  fromOrigin,
                                    const NvGeVector3d& fromXAxis,
                                    const NvGeVector3d& fromYAxis,
                                    const NvGeVector3d& fromZAxis,
                                    const NvGePoint3d&  toOrigin,
                                    const NvGeVector3d& toXAxis,
                                    const NvGeVector3d& toYAxis,
                                    const NvGeVector3d& toZAxis);

    GE_DLLEXPIMPORT static
    NvGeMatrix3d   worldToPlane    (const NvGeVector3d& normal);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   worldToPlane    (const NvGePlane&);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   planeToWorld    (const NvGeVector3d& normal);
    GE_DLLEXPIMPORT static
    NvGeMatrix3d   planeToWorld    (const NvGePlane&);

    // Get the length of the MAXIMUM column of the
    // 3x3 portion of the matrix.
    //
    GE_DLLEXPIMPORT double scale(void) const;

    GE_DLLEXPIMPORT double          norm            () const;

    GE_DLLEXPIMPORT NvGeMatrix2d convertToLocal     (NvGeVector3d& normal, double& elev) const;


    // For convenient access to the data.
    //
    GE_DLLEXPIMPORT double         operator ()     (unsigned int, unsigned int) const;
    GE_DLLEXPIMPORT double&        operator ()     (unsigned int, unsigned int);

    // The components of the matrix.
    //
    double         entry[4][4];    // [row][column]

    GE_DLLEXPIMPORT Adesk::Boolean  inverse(NvGeMatrix3d& inv, double tol) const;

private:
    void           pivot           (int, NvGeMatrix3d&);
    int            pivotIndex(int) const;
    void           swapRows        (int, int, NvGeMatrix3d&);
};

inline bool
NvGeMatrix3d::operator == (const NvGeMatrix3d& otherMatrix) const
{
    return this->isEqualTo(otherMatrix);
}

// This operator is the logical negation of the `==' operator.
//
inline bool
NvGeMatrix3d::operator != (const NvGeMatrix3d& otherMatrix) const
{
    return !this->isEqualTo(otherMatrix);
}

// Return the element in position [row][column] of the `entry' array.
//
inline double
NvGeMatrix3d::operator () (
    unsigned int row,
    unsigned int column) const
{
    return entry[row][column];
}

inline double&
NvGeMatrix3d::operator () (
    unsigned int row,
    unsigned int column)
{
    return entry[row][column];
}

#pragma pack (pop)
#endif
