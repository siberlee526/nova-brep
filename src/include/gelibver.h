#ifndef NV_GELIBVERSION_H
#define NV_GELIBVERSION_H

#include "Nova.h"
#include "gedll.h"
#pragma pack (push, 8)

// Current image version.
//
#define IMAGE_MAJOR_VER          2
#define IMAGE_MINOR_VER          0
#define IMAGE_CORRECTIVE_VER     0
#define IMAGE_INTERNAL_VER       0

class 
NvGeLibVersion {
public:
    // Constructors.
    //
    GE_DLLEXPIMPORT NvGeLibVersion();
    GE_DLLEXPIMPORT NvGeLibVersion(const NvGeLibVersion&);
    GE_DLLEXPIMPORT NvGeLibVersion(Nova::UInt8 major, Nova::UInt8 minor,
                   Nova::UInt8 corrective,
                   Nova::UInt8 internal_version);

    // Inquiry functions.
    //
    GE_DLLEXPIMPORT Nova::UInt8     majorVersion        () const;
    GE_DLLEXPIMPORT Nova::UInt8     minorVersion        () const;
    GE_DLLEXPIMPORT Nova::UInt8     correctiveVersion   () const;
    GE_DLLEXPIMPORT Nova::UInt8     schemaVersion       () const;

    // Set functions.
    //
    GE_DLLEXPIMPORT NvGeLibVersion&     setMajorVersion        (Nova::UInt8 val);
    GE_DLLEXPIMPORT NvGeLibVersion&     setMinorVersion        (Nova::UInt8 val);
    GE_DLLEXPIMPORT NvGeLibVersion&     setCorrectiveVersion   (Nova::UInt8 val);
    GE_DLLEXPIMPORT NvGeLibVersion&     setSchemaVersion       (Nova::UInt8 val);

    // Comparisons
    //
    GE_DLLEXPIMPORT Nova::Boolean operator ==      (const NvGeLibVersion&) const;
    GE_DLLEXPIMPORT Nova::Boolean operator !=      (const NvGeLibVersion&) const;
    GE_DLLEXPIMPORT Nova::Boolean operator <       (const NvGeLibVersion&) const;
    GE_DLLEXPIMPORT Nova::Boolean operator <=      (const NvGeLibVersion&) const;
    GE_DLLEXPIMPORT Nova::Boolean operator >       (const NvGeLibVersion&) const;
    GE_DLLEXPIMPORT Nova::Boolean operator >=      (const NvGeLibVersion&) const;

    // Relased version objects.
    // gelib release 0 --- End of 1995.
    //
    GE_DLLDATAEXIMP static const NvGeLibVersion kRelease0_95;

    // gelib r14 release.
    //
    GE_DLLDATAEXIMP static const NvGeLibVersion kReleaseSed;

    // gelib r15 release.
    //
    GE_DLLDATAEXIMP static const NvGeLibVersion kReleaseTah;

private:
    Nova::UInt8   mVersion[10];
};

#pragma pack (pop)
#endif
