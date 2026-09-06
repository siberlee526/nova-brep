// #ifndef NV_GEFILER_H
// #define NV_GEFILER_H   

// #include "NvAChar.h"
// #include "gegbl.h"
// #include "acdb.h"
// #include "acadstrc.h"
// #pragma pack (push, 8)
     
     
// class AcDbDwgFiler;
// class NvGePoint2d;
// class NvGePoint3d;
// class NvGeVector2d;
// class NvGeVector3d;
// class NvString;

// class 
//  ADESK_NO_VTABLE
// NvGeFiler
// {
// protected:
//     GE_DLLEXPIMPORT NvGeFiler();
// public:
//     // Read/write functions.
//     //
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readBoolean(Adesk::Boolean*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeBoolean(Adesk::Boolean) = 0;

//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readBool(bool*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeBool(bool) = 0;

//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readSignedByte(char *) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeSignedByte(char) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readString(NvString &) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeString(const NvString &) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readShort(short*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeShort(short) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readLong(Adesk::Int32*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeLong(Adesk::Int32) = 0;
     
//     // Unicode: this is assumed to be a binary value, not a text character!
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readUChar(unsigned char*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeUChar(unsigned char) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readUShort(unsigned short*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeUShort(unsigned short) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readULong(Adesk::UInt32*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeULong(Adesk::UInt32) = 0;
          
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readDouble(double*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeDouble(double) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readPoint2d(NvGePoint2d*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writePoint2d(const NvGePoint2d&) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readPoint3d(NvGePoint3d*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writePoint3d(const NvGePoint3d&) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readVector2d(NvGeVector2d*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeVector2d(const NvGeVector2d&) = 0;
     
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readVector3d(NvGeVector3d*) = 0; 
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeVector3d(const NvGeVector3d&) = 0;
     
//     // This method must be implemented in order to read/write 
//     // external curves and surfaces (and nurb surfaces). 
//     GE_DLLEXPIMPORT virtual
//     AcDbDwgFiler*          dwgFiler();

//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      readBytes(void*, Adesk::UInt32) = 0;
//     GE_DLLEXPIMPORT virtual
//     Acad::ErrorStatus      writeBytes(const void*, Adesk::UInt32) = 0;
// };

// inline
// AcDbDwgFiler* NvGeFiler::dwgFiler()
// {
//     return NULL;
// }
     
     
// #pragma pack (pop)
// #endif
