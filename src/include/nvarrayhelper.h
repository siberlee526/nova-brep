#if defined(NOVA_NVGEVECTOR2D_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVGEVECTOR2D_DEFINED
template<>
struct NvArrayItemCopierSelector<NvGeVector2d, false>
{
    typedef NvArrayMemCopyReallocator<NvGeVector2d> allocator;
};
#endif

#if defined(NOVA_NVGEVECTOR3D_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVGEVECTOR3D_DEFINED
template<>
struct NvArrayItemCopierSelector<NvGeVector3d, false>
{
    typedef NvArrayMemCopyReallocator<NvGeVector3d> allocator;
};
#endif

#if defined(NOVA_NVGEPOINT3D_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVGEPOINT3D_DEFINED
template<>
struct NvArrayItemCopierSelector<NvGePoint3d, false>
{
    typedef NvArrayMemCopyReallocator<NvGePoint3d> allocator;
};
#endif

#if defined(NOVA_NVEPOINT2D_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVEPOINT2D_DEFINED
template<>
struct NvArrayItemCopierSelector<NvGePoint2d, false>
{
    typedef NvArrayMemCopyReallocator<NvGePoint2d> allocator;
};
#endif

#if defined(NOVA_NVDBOBJECTID_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVDBOBJECTID_DEFINED
template<>
struct NvArrayItemCopierSelector<AcDbObjectId, false>
{
    typedef NvArrayMemCopyReallocator<AcDbObjectId> allocator;
};
#endif

#if defined(NOVA_NVCMENTITYCOLOR_DEFINED) && defined(NV_NVARRAY_H)
#undef NOVA_NVCMENTITYCOLOR_DEFINED
template<>
struct NvArrayItemCopierSelector<AcCmEntityColor, false>
{
    typedef NvArrayMemCopyReallocator<AcCmEntityColor> allocator;
};
#endif


