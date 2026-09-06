#include <NvBoolean.h>

#include <NvException.h>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>

namespace
{
  //! Validates inputs and runs a boolean algorithm, translating failures into NvException.
  template <typename theAlgo_t>
  NvShape performBoolean (const NvShape& theShape1,
                             const NvShape& theShape2,
                             const char*       theName)
  {
    if (theShape1.IsNull() || theShape2.IsNull())
    {
      throw NvException (std::string (theName) + ": input shape is null");
    }

    try
    {
      theAlgo_t anAlgo (theShape1.Shape(), theShape2.Shape());
      anAlgo.Build();
      if (!anAlgo.IsDone() || anAlgo.HasErrors())
      {
        throw NvException (std::string (theName) + ": boolean operation failed");
      }
      return NvShape (anAlgo.Shape());
    }
    catch (const Standard_Failure& theFailure)
    {
      throw NvException::FromFailure (theFailure);
    }
  }
}

//=================================================================================================

NvShape NvBoolean::Fuse (const NvShape& theShape1, const NvShape& theShape2)
{
  return performBoolean<BRepAlgoAPI_Fuse> (theShape1, theShape2, "NvBoolean::Fuse()");
}

//=================================================================================================

NvShape NvBoolean::Cut (const NvShape& theShape1, const NvShape& theShape2)
{
  return performBoolean<BRepAlgoAPI_Cut> (theShape1, theShape2, "NvBoolean::Cut()");
}

//=================================================================================================

NvShape NvBoolean::Common (const NvShape& theShape1, const NvShape& theShape2)
{
  return performBoolean<BRepAlgoAPI_Common> (theShape1, theShape2, "NvBoolean::Common()");
}
