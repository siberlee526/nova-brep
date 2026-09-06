#ifndef NV_GEKVEC_H
#define NV_GEKVEC_H

#include "gegbl.h"
#include "gedblar.h"
#pragma pack (push, 8)

class NvGeInterval;

class
NvGeKnotVector
{
public:
    // By default it is 1.0e-9. It could be changed by API user.
    GE_DLLDATAEXIMP static double globalKnotTolerance;
    GE_DLLEXPIMPORT NvGeKnotVector(double eps = globalKnotTolerance);
    GE_DLLEXPIMPORT NvGeKnotVector(int size, int growSize, double eps = globalKnotTolerance);
    GE_DLLEXPIMPORT NvGeKnotVector(int size, const double [], double eps = globalKnotTolerance);
	// Elevates multiplicity of each DISTINCT knot by plusMult;
	// Contract: plusMul >= 0;
    GE_DLLEXPIMPORT NvGeKnotVector(int plusMult, const NvGeKnotVector& src);
    GE_DLLEXPIMPORT NvGeKnotVector(const NvGeKnotVector& src);
    GE_DLLEXPIMPORT NvGeKnotVector(const NvGeDoubleArray& src, double eps = globalKnotTolerance);

    GE_DLLEXPIMPORT ~NvGeKnotVector();

    // Copy operator.
    //
    GE_DLLEXPIMPORT NvGeKnotVector&     operator =  (const NvGeKnotVector& src);
    GE_DLLEXPIMPORT NvGeKnotVector&     operator =  (const NvGeDoubleArray& src);


    // Indexing into the knot vector.
    //
    GE_DLLEXPIMPORT double&             operator [] (int);
    GE_DLLEXPIMPORT const double        operator [] (int) const;

    // Equality test
    //
    GE_DLLEXPIMPORT Adesk::Boolean      isEqualTo (const NvGeKnotVector& other) const;

    // Inquiry functions
    //
    GE_DLLEXPIMPORT double              startParam         () const;
    GE_DLLEXPIMPORT double              endParam           () const;
    GE_DLLEXPIMPORT int                 multiplicityAt     (int i) const;
    GE_DLLEXPIMPORT int                 multiplicityAt     (double param) const;
    GE_DLLEXPIMPORT int                 numIntervals       () const;

    // Evaluate funtions
    //
    GE_DLLEXPIMPORT int                 getInterval        (int ord, double par,
                                            NvGeInterval& interval ) const;
    GE_DLLEXPIMPORT void                getDistinctKnots   (NvGeDoubleArray& knots) const;
    GE_DLLEXPIMPORT Adesk::Boolean      contains           (double param) const;
    GE_DLLEXPIMPORT Adesk::Boolean      isOn               (double knot) const;

    // Edit function
    //
    GE_DLLEXPIMPORT NvGeKnotVector&     reverse            ();
    GE_DLLEXPIMPORT NvGeKnotVector&     removeAt           (int);
    GE_DLLEXPIMPORT NvGeKnotVector&     removeSubVector    (int startIndex, int endIndex);

    GE_DLLEXPIMPORT NvGeKnotVector&     insertAt           (int indx, double u,
                                            int multiplicity = 1);
    GE_DLLEXPIMPORT NvGeKnotVector&     insert             (double u);
    GE_DLLEXPIMPORT int                 append             (double val);
    GE_DLLEXPIMPORT NvGeKnotVector&     append             (NvGeKnotVector& tail,
                                            double knotRatio = 0.);
    GE_DLLEXPIMPORT int                 split              (double par,
                                            NvGeKnotVector* pKnot1,
                                            int multLast,
                                            NvGeKnotVector* pKnot2,
                                            int multFirst ) const;


    GE_DLLEXPIMPORT NvGeKnotVector&     setRange           (double lower, double upper);

    GE_DLLEXPIMPORT double              tolerance          () const;
    GE_DLLEXPIMPORT NvGeKnotVector&     setTolerance       (double tol);

    // Array length.
    //
    GE_DLLEXPIMPORT int                 length             () const; // Logical length.
    GE_DLLEXPIMPORT Adesk::Boolean      isEmpty            () const;
    GE_DLLEXPIMPORT int                 logicalLength      () const;
    GE_DLLEXPIMPORT NvGeKnotVector&     setLogicalLength   (int);
    GE_DLLEXPIMPORT int                 physicalLength     () const;
    GE_DLLEXPIMPORT NvGeKnotVector&     setPhysicalLength  (int);

    // Automatic resizing.
    //
    GE_DLLEXPIMPORT int                 growLength  () const;
    GE_DLLEXPIMPORT NvGeKnotVector&     setGrowLength(int);

    // Treat as simple array of double.
    //
    GE_DLLEXPIMPORT const double*       asArrayPtr  () const;
    GE_DLLEXPIMPORT double*             asArrayPtr  ();

    GE_DLLEXPIMPORT NvGeKnotVector&     set (int size, const double [], double eps = globalKnotTolerance);

protected:
    NvGeDoubleArray    mData;
    double             mTolerance;

    Adesk::Boolean     isValid (int) const;
};

// Inline methods.
//
inline double
NvGeKnotVector::tolerance() const
{ return mTolerance;}

inline NvGeKnotVector&
NvGeKnotVector::setTolerance(double eps)
{ mTolerance = eps;	return *this;}

inline Adesk::Boolean
NvGeKnotVector::isValid(int i) const
{ return i >= 0 && i < mData.logicalLength(); }

inline double&
NvGeKnotVector::operator [] (int i)
{ assert(isValid(i)); return mData[i]; }

inline const double
NvGeKnotVector::operator [] (int i) const
{ assert(isValid(i)); return mData[i]; }

inline const double*
NvGeKnotVector::asArrayPtr() const
{ return mData.asArrayPtr(); }

inline double*
NvGeKnotVector::asArrayPtr()
{ return mData.asArrayPtr(); }

#pragma pack (pop)
#endif
