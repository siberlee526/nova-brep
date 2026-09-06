// gelibver.cpp - implementation of NvGeLibVersion.
//
// The version is packed into ten bytes: [0] major, [1] minor,
// [2] corrective, [3] schema; the remaining bytes are reserved padding
// (kept zero). The image version macros from gelibver.h define the
// default-constructed value.

#include <gelibver.h>

#include <Nova.h>

namespace
{

//! Packs the four published version components into the byte array.
void PackVersion (Nova::UInt8 theVersion[10],
                  Nova::UInt8 theMajor, Nova::UInt8 theMinor,
                  Nova::UInt8 theCorrective, Nova::UInt8 theSchema)
{
  theVersion[0] = theMajor;
  theVersion[1] = theMinor;
  theVersion[2] = theCorrective;
  theVersion[3] = theSchema;
  for (int aSlot = 4; aSlot < 10; ++aSlot)
  {
    theVersion[aSlot] = 0;
  }
}

//! Three-way comparison of two packed versions.
int CompareVersions (const Nova::UInt8 theLeft[10], const Nova::UInt8 theRight[10])
{
  for (int aSlot = 0; aSlot < 10; ++aSlot)
  {
    if (theLeft[aSlot] < theRight[aSlot])
    {
      return -1;
    }
    if (theLeft[aSlot] > theRight[aSlot])
    {
      return 1;
    }
  }
  return 0;
}

} // namespace

//=================================================================================================

NvGeLibVersion::NvGeLibVersion()
{
  PackVersion (mVersion, IMAGE_MAJOR_VER, IMAGE_MINOR_VER,
               IMAGE_CORRECTIVE_VER, IMAGE_INTERNAL_VER);
}

//=================================================================================================

NvGeLibVersion::NvGeLibVersion (const NvGeLibVersion& theSrc)
{
  for (int aSlot = 0; aSlot < 10; ++aSlot)
  {
    mVersion[aSlot] = theSrc.mVersion[aSlot];
  }
}

//=================================================================================================

NvGeLibVersion::NvGeLibVersion (Nova::UInt8 theMajor, Nova::UInt8 theMinor,
                                Nova::UInt8 theCorrective, Nova::UInt8 theInternalVersion)
{
  PackVersion (mVersion, theMajor, theMinor, theCorrective, theInternalVersion);
}

//=================================================================================================

Nova::UInt8 NvGeLibVersion::majorVersion() const
{
  return mVersion[0];
}

//=================================================================================================

Nova::UInt8 NvGeLibVersion::minorVersion() const
{
  return mVersion[1];
}

//=================================================================================================

Nova::UInt8 NvGeLibVersion::correctiveVersion() const
{
  return mVersion[2];
}

//=================================================================================================

Nova::UInt8 NvGeLibVersion::schemaVersion() const
{
  return mVersion[3];
}

//=================================================================================================

NvGeLibVersion& NvGeLibVersion::setMajorVersion (Nova::UInt8 theVal)
{
  mVersion[0] = theVal;
  return *this;
}

//=================================================================================================

NvGeLibVersion& NvGeLibVersion::setMinorVersion (Nova::UInt8 theVal)
{
  mVersion[1] = theVal;
  return *this;
}

//=================================================================================================

NvGeLibVersion& NvGeLibVersion::setCorrectiveVersion (Nova::UInt8 theVal)
{
  mVersion[2] = theVal;
  return *this;
}

//=================================================================================================

NvGeLibVersion& NvGeLibVersion::setSchemaVersion (Nova::UInt8 theVal)
{
  mVersion[3] = theVal;
  return *this;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator == (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) == 0;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator != (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) != 0;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator < (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) < 0;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator <= (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) <= 0;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator > (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) > 0;
}

//=================================================================================================

Nova::Boolean NvGeLibVersion::operator >= (const NvGeLibVersion& theOther) const
{
  return CompareVersions (mVersion, theOther.mVersion) >= 0;
}

//=================================================================================================

// Historical release markers of the ARX gelib lineage.
const NvGeLibVersion NvGeLibVersion::kRelease0_95 (0, 95, 0, 0);

const NvGeLibVersion NvGeLibVersion::kReleaseSed (14, 0, 0, 0);

const NvGeLibVersion NvGeLibVersion::kReleaseTah (15, 0, 0, 0);
