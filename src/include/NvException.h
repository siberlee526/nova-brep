#ifndef NvException_HeaderFile
#define NvException_HeaderFile

#include <NvMacros.h>

#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>

#include <stdexcept>
#include <string>

//! Error type thrown by NvBREP APIs.
//!
//! NvBREP wraps OCCT algorithms and reports every failure (invalid input,
//! failed algorithm, raised Standard_Failure) as NvException, so that
//! consumers do not need to handle OCCT exception types directly.
class NV_EXPORT NvException : public std::runtime_error
{
public:

  //! Constructs the exception with a plain message.
  explicit NvException (const char* theMessage)
  : std::runtime_error (theMessage) {}

  //! Constructs the exception with a plain message.
  explicit NvException (const std::string& theMessage)
  : std::runtime_error (theMessage.c_str()) {}

  //! Constructs the exception from an OCCT string.
  explicit NvException (const TCollection_AsciiString& theMessage)
  : std::runtime_error (theMessage.ToCString()) {}

  //! Creates an exception describing the given OCCT failure
  //! (exception type name followed by its message).
  static NvException FromFailure (const Standard_Failure& theFailure);
};

#endif // NvException_HeaderFile
