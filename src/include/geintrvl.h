#ifndef NV_GEINTRVL_H
#define NV_GEINTRVL_H

#include "gegbl.h"
#pragma pack (push, 8)

class

NvGeInterval
{
public:
    GE_DLLEXPIMPORT NvGeInterval(double tol = 1.e-12);
    GE_DLLEXPIMPORT NvGeInterval(const NvGeInterval& src);
    GE_DLLEXPIMPORT NvGeInterval(double lower, double upper, double tol = 1.e-12);
    GE_DLLEXPIMPORT NvGeInterval(Nova::Boolean boundedBelow, double bound,
                 double tol = 1.e-12);
    GE_DLLEXPIMPORT ~NvGeInterval();

    // Assignment operator.
    //
    GE_DLLEXPIMPORT NvGeInterval&  operator =       (const NvGeInterval& otherInterval);

    // Get/set methods.
    //
    GE_DLLEXPIMPORT double         lowerBound       () const;
    GE_DLLEXPIMPORT double         upperBound       () const;
    GE_DLLEXPIMPORT double         element          () const;
    GE_DLLEXPIMPORT void           getBounds        (double& lower, double& upper) const;
    GE_DLLEXPIMPORT double         length           () const;
    GE_DLLEXPIMPORT double         tolerance        () const;

    GE_DLLEXPIMPORT NvGeInterval&  set              (double lower, double upper);
    GE_DLLEXPIMPORT NvGeInterval&  set              (Nova::Boolean boundedBelow, double bound);
    GE_DLLEXPIMPORT NvGeInterval&  set              ();
    GE_DLLEXPIMPORT NvGeInterval&  setUpper         (double upper);
    GE_DLLEXPIMPORT NvGeInterval&  setLower         (double lower);
    GE_DLLEXPIMPORT NvGeInterval&  setTolerance     (double tol);

    // Interval editing.
    //
    GE_DLLEXPIMPORT void           getMerge         (const NvGeInterval& otherInterval, NvGeInterval& result) const;
    GE_DLLEXPIMPORT int            subtract         (const NvGeInterval& otherInterval,
                                     NvGeInterval& lInterval,
                                     NvGeInterval& rInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean intersectWith    (const NvGeInterval& otherInterval, NvGeInterval& result) const;

    // Interval characterization.
    //
    GE_DLLEXPIMPORT Nova::Boolean isBounded        () const;
    GE_DLLEXPIMPORT Nova::Boolean isBoundedAbove   () const;
    GE_DLLEXPIMPORT Nova::Boolean isBoundedBelow   () const;
    GE_DLLEXPIMPORT Nova::Boolean isUnBounded      () const;
    GE_DLLEXPIMPORT Nova::Boolean isSingleton      () const;

    // Relation to other intervals.
    //
    GE_DLLEXPIMPORT Nova::Boolean isDisjoint       (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean contains         (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean contains         (double val) const;

    // Continuity
    //
    GE_DLLEXPIMPORT Nova::Boolean isContinuousAtUpper (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean isOverlapAtUpper    (const NvGeInterval& otherInterval,
                                        NvGeInterval& overlap) const;
    // Equality
    //
    GE_DLLEXPIMPORT Nova::Boolean operator ==      (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean operator !=      (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean isEqualAtUpper   (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean isEqualAtUpper   (double value) const;
    GE_DLLEXPIMPORT Nova::Boolean isEqualAtLower   (const NvGeInterval& otherInterval) const;
    GE_DLLEXPIMPORT Nova::Boolean isEqualAtLower   (double value) const;

    // To be used with periodic curves
    //
    GE_DLLEXPIMPORT Nova::Boolean isPeriodicallyOn (double period, double& val);

    // Comparisons.
    //
    friend
    GE_DLLEXPIMPORT
    Nova::Boolean operator >       (double val, const NvGeInterval& intrvl);
    GE_DLLEXPIMPORT Nova::Boolean operator >       (double val) const;
    GE_DLLEXPIMPORT Nova::Boolean operator >       (const NvGeInterval& otherInterval) const;
    friend
    GE_DLLEXPIMPORT
    Nova::Boolean operator >=      (double val, const NvGeInterval& intrvl);
    GE_DLLEXPIMPORT Nova::Boolean operator >=      (double val) const;
    GE_DLLEXPIMPORT Nova::Boolean operator >=      (const NvGeInterval& otherInterval) const;
    friend
    GE_DLLEXPIMPORT
    Nova::Boolean operator <       (double val, const NvGeInterval& intrvl);
    GE_DLLEXPIMPORT Nova::Boolean operator <       (double val) const;
    GE_DLLEXPIMPORT Nova::Boolean operator <       (const NvGeInterval& otherInterval) const;
    friend
    GE_DLLEXPIMPORT
    Nova::Boolean operator <=      (double val, const NvGeInterval& intrvl);
    GE_DLLEXPIMPORT Nova::Boolean operator <=      (double val) const;
    GE_DLLEXPIMPORT Nova::Boolean operator <=      (const NvGeInterval& otherInterval) const;

protected:
    friend class NvGeImpInterval;

    class NvGeImpInterval  *mpImpInt;

    // Construct object from its corresponding implementation object.
    GE_DLLEXPIMPORT NvGeInterval (NvGeImpInterval&, int);

private:
    int              mDelInt;
};

#pragma pack (pop)
#endif
