#include <NvPrim.h>

#include <NvException.h>

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>

namespace
{
  //! Checks a dimension-like argument to be positive.
  void checkPositive (double theValue, const char* theName)
  {
    if (theValue < Precision::Confusion())
    {
      throw NvException (std::string ("NvPrim: ") + theName + " must be positive");
    }
  }
}

//=================================================================================================

NvShape NvPrim::Box (double theDx, double theDy, double theDz)
{
  checkPositive (theDx, "theDx");
  checkPositive (theDy, "theDy");
  checkPositive (theDz, "theDz");

  BRepPrimAPI_MakeBox aBuilder (theDx, theDy, theDz);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Box(): BRepPrimAPI_MakeBox failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Box (const gp_Pnt& thePoint, double theDx, double theDy, double theDz)
{
  checkPositive (theDx, "theDx");
  checkPositive (theDy, "theDy");
  checkPositive (theDz, "theDz");

  BRepPrimAPI_MakeBox aBuilder (thePoint, theDx, theDy, theDz);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Box(): BRepPrimAPI_MakeBox failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Cylinder (double theRadius, double theHeight)
{
  checkPositive (theRadius, "theRadius");
  checkPositive (theHeight, "theHeight");

  BRepPrimAPI_MakeCylinder aBuilder (theRadius, theHeight);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Cylinder(): BRepPrimAPI_MakeCylinder failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Cylinder (const gp_Ax2& theAxis, double theRadius, double theHeight)
{
  checkPositive (theRadius, "theRadius");
  checkPositive (theHeight, "theHeight");

  BRepPrimAPI_MakeCylinder aBuilder (theAxis, theRadius, theHeight);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Cylinder(): BRepPrimAPI_MakeCylinder failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Sphere (double theRadius)
{
  checkPositive (theRadius, "theRadius");

  BRepPrimAPI_MakeSphere aBuilder (theRadius);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Sphere(): BRepPrimAPI_MakeSphere failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Sphere (const gp_Pnt& thePoint, double theRadius)
{
  checkPositive (theRadius, "theRadius");

  BRepPrimAPI_MakeSphere aBuilder (thePoint, theRadius);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Sphere(): BRepPrimAPI_MakeSphere failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Cone (double theRadius1, double theRadius2, double theHeight)
{
  checkPositive (theHeight, "theHeight");
  if (theRadius1 < Precision::Confusion() && theRadius2 < Precision::Confusion())
  {
    throw NvException ("NvPrim::Cone(): at least one radius must be positive");
  }

  BRepPrimAPI_MakeCone aBuilder (theRadius1, theRadius2, theHeight);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Cone(): BRepPrimAPI_MakeCone failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Cone (const gp_Ax2& theAxis, double theRadius1, double theRadius2, double theHeight)
{
  checkPositive (theHeight, "theHeight");
  if (theRadius1 < Precision::Confusion() && theRadius2 < Precision::Confusion())
  {
    throw NvException ("NvPrim::Cone(): at least one radius must be positive");
  }

  BRepPrimAPI_MakeCone aBuilder (theAxis, theRadius1, theRadius2, theHeight);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Cone(): BRepPrimAPI_MakeCone failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Torus (double theMajorRadius, double theMinorRadius)
{
  checkPositive (theMajorRadius, "theMajorRadius");
  checkPositive (theMinorRadius, "theMinorRadius");

  BRepPrimAPI_MakeTorus aBuilder (theMajorRadius, theMinorRadius);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Torus(): BRepPrimAPI_MakeTorus failed");
  }
  return NvShape (aBuilder.Shape());
}

//=================================================================================================

NvShape NvPrim::Torus (const gp_Ax2& theAxis, double theMajorRadius, double theMinorRadius)
{
  checkPositive (theMajorRadius, "theMajorRadius");
  checkPositive (theMinorRadius, "theMinorRadius");

  BRepPrimAPI_MakeTorus aBuilder (theAxis, theMajorRadius, theMinorRadius);
  aBuilder.Build();
  if (!aBuilder.IsDone())
  {
    throw NvException ("NvPrim::Torus(): BRepPrimAPI_MakeTorus failed");
  }
  return NvShape (aBuilder.Shape());
}
