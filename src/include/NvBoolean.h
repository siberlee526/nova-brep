#ifndef NvBoolean_HeaderFile
#define NvBoolean_HeaderFile

#include <NvShape.h>

//! Higher-level boolean operations, wrapping BRepAlgoAPI algorithms.
//!
//! Every operation verifies IsDone()/HasErrors() of the underlying OCCT
//! algorithm and translates failures (including raised Standard_Failure)
//! into NvException, so results are always safe to use.
class NV_EXPORT NvBoolean
{
public:

  NvBoolean() = delete;

  //! Unites two shapes (BRepAlgoAPI_Fuse).
  //! @throws NvException on null input or failed operation
  static NvShape Fuse (const NvShape& theShape1, const NvShape& theShape2);

  //! Subtracts theShape2 from theShape1 (BRepAlgoAPI_Cut).
  //! @throws NvException on null input or failed operation
  static NvShape Cut (const NvShape& theShape1, const NvShape& theShape2);

  //! Computes the common part of two shapes (BRepAlgoAPI_Common).
  //! @throws NvException on null input or failed operation
  static NvShape Common (const NvShape& theShape1, const NvShape& theShape2);
};

#endif // NvBoolean_HeaderFile
