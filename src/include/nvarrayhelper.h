#if defined(ADSK_ACGEVECTOR2D_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACGEVECTOR2D_DEFINED
template<>
struct AcArrayItemCopierSelector<AcGeVector2d, false>
{
    typedef AcArrayMemCopyReallocator<AcGeVector2d> allocator;
};
#endif

#if defined(ADSK_ACGEVECTOR3D_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACGEVECTOR3D_DEFINED
template<>
struct AcArrayItemCopierSelector<AcGeVector3d, false>
{
    typedef AcArrayMemCopyReallocator<AcGeVector3d> allocator;
};
#endif

#if defined(ADSK_ACGEPOINT3D_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACGEPOINT3D_DEFINED
template<>
struct AcArrayItemCopierSelector<AcGePoint3d, false>
{
    typedef AcArrayMemCopyReallocator<AcGePoint3d> allocator;
};
#endif

#if defined(ADSK_ACGEPOINT2D_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACGEPOINT2D_DEFINED
template<>
struct AcArrayItemCopierSelector<AcGePoint2d, false>
{
    typedef AcArrayMemCopyReallocator<AcGePoint2d> allocator;
};
#endif

#if defined(ADSK_ACDBOBJECTID_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACDBOBJECTID_DEFINED
template<>
struct AcArrayItemCopierSelector<AcDbObjectId, false>
{
    typedef AcArrayMemCopyReallocator<AcDbObjectId> allocator;
};
#endif

#if defined(ADSK_ACCMENTITYCOLOR_DEFINED) && defined(AC_ACARRAY_H)
#undef ADSK_ACCMENTITYCOLOR_DEFINED
template<>
struct AcArrayItemCopierSelector<AcCmEntityColor, false>
{
    typedef AcArrayMemCopyReallocator<AcCmEntityColor> allocator;
};
#endif


