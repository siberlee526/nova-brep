#include <NvException.h>

#include <Standard_Failure.hxx>

#include <string>

//=================================================================================================

NvException NvException::FromFailure (const Standard_Failure& theFailure)
{
  // Compose "<ExceptionType>: <Message>" so that the OCCT exception kind is visible in logs.
  // ExceptionType() and what() never return null strings.
  const std::string aMessage = std::string (theFailure.ExceptionType()) + ": " + theFailure.what();
  return NvException (aMessage);
}
