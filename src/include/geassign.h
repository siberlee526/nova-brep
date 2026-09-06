#ifndef NV_GEASSIGN_H
#define NV_GEASSIGN_H

#pragma pack (push, 8)
class NvGePoint2d;
class NvGeVector2d;
class NvGePoint3d;
class NvGeVector3d;

inline NvGePoint2d&
asPnt2d(const double* pnt)
{
    return *((NvGePoint2d*)pnt);
}

inline NvGeVector2d&
asVec2d(const double* vec)
{
    return *((NvGeVector2d*)vec);
}
//clang 4.0 has a false positive where NvGePoint2d is treated as alignment of 4. silencing the compiler here
//error: cast from 'const NvGeVector3d *' to 'double *' increases required alignment from 4 to 8 [-Werror,-Wcast-align]
#ifdef  __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-align"
#endif
inline double*
asDblArray(const NvGePoint2d& pnt)
{
    return (double*)&pnt;
}

inline double*
asDblArray(const NvGeVector2d& vec)
{
    return (double*)&vec;
}
#ifdef  __clang__
#pragma clang diagnostic pop
#endif

inline NvGePoint3d&
asPnt3d(const double* pnt)
{
    return *((NvGePoint3d*)pnt);
}

inline NvGeVector3d&
asVec3d(const double* vec)
{
    return *((NvGeVector3d*)vec);
}
#ifdef  __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wcast-align"
#endif
inline double*
asDblArray(const NvGePoint3d& pnt)
{
    return (double*)&pnt;
}

inline double*
asDblArray(const NvGeVector3d& vec)
{
    return (double*)&vec;
}
#ifdef  __clang__
#pragma clang diagnostic pop
#endif

#pragma pack (pop)
#endif
