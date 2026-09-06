# NvBREP library
# - public headers live in src/include (installed as <include>/Nv*.h);
# - implementation sources live in src/occt (the OCCT algorithm encapsulation layer).

set (NV_HEADERS
  include/Nova.h
  include/Nv.h
  include/NvAChar.h
  include/NvBoolean.h
  include/NvException.h
  include/NvHeapOpers.h
  include/NvMacros.h
  include/NvPrim.h
  include/NvShape.h
  include/NvString.h
  include/NvVersion.h
  include/c11_Annex_K.h
  include/charset.h
  include/def.h
  include/geapln3d.h
  include/gearc2d.h
  include/gearc3d.h
  include/geassign.h
  include/geblok2d.h
  include/geblok3d.h
  include/gebndpln.h
  include/gecbndry.h
  include/gecint2d.h
  include/gecint3d.h
  include/geclip2d.h
  include/gecomp2d.h
  include/gecomp3d.h
  include/gecone.h
  include/gecsint.h
  include/gecspl2d.h
  include/gecspl3d.h
  include/gecurv2d.h
  include/gecurv3d.h
  include/gecylndr.h
  include/gedblar.h
  include/gedll.h
  include/gedwgio.h
  include/gedxfio.h
  include/geell2d.h
  include/geell3d.h
  include/geent2d.h
  include/geent3d.h
  include/geextc2d.h
  include/geextc3d.h
  include/geextsf.h
  include/gefileio.h
  include/gefiler.h
  include/gegbl.h
  include/gegblabb.h
  include/gegblge.h
  include/gegblnew.h
  include/geintarr.h
  include/geintrvl.h
  include/gekvec.h
  include/gelent2d.h
  include/gelent3d.h
  include/gelibver.h
  include/geline2d.h
  include/geline3d.h
  include/gelnsg2d.h
  include/gelnsg3d.h
  include/gemat2d.h
  include/gemat3d.h
  include/genurb2d.h
  include/genurb3d.h
  include/genurbsf.h
  include/geoffc2d.h
  include/geoffc3d.h
  include/geoffsf.h
  include/gepent2d.h
  include/gepent3d.h
  include/geplanar.h
  include/geplane.h
  include/geplin2d.h
  include/geplin3d.h
  include/gepnt2d.h
  include/gepnt3d.h
  include/geponc2d.h
  include/geponc3d.h
  include/geponsrf.h
  include/gepos2d.h
  include/gepos3d.h
  include/gept2dar.h
  include/gept3dar.h
  include/geray2d.h
  include/geray3d.h
  include/gescl2d.h
  include/gescl3d.h
  include/gesent2d.h
  include/gesent3d.h
  include/gesphere.h
  include/gessint.h
  include/gesurf.h
  include/getol.h
  include/getorus.h
  include/gevc2dar.h
  include/gevc3dar.h
  include/gevec2d.h
  include/gevec3d.h
  include/gevptar.h
  include/gexbndsf.h
  include/heap.h
  include/nvarray.h
  include/nvarrayhelper.h
  include/nvbasedefs.h
  include/nvheapmanager.h
)

set (NV_SOURCES
  occt/NvBoolean.cpp
  occt/NvException.cpp
  occt/NvPrim.cpp
  occt/NvShape.cpp
  occt/NvString.cpp
  occt/geapln3d.cpp
  occt/gearc2d.cpp
  occt/gearc3d.cpp
  occt/geblok2d.cpp
  occt/geblok3d.cpp
  occt/gebndpln.cpp
  occt/gecbndry.cpp
  occt/gecint2d.cpp
  occt/gecint3d.cpp
  occt/geclip2d.cpp
  occt/gecomp2d.cpp
  occt/gecomp3d.cpp
  occt/gecone.cpp
  occt/gecsint.cpp
  occt/gecspl2d.cpp
  occt/gecspl3d.cpp
  occt/gecurv2d.cpp
  occt/gecurv3d.cpp
  occt/gecylndr.cpp
  occt/geell2d.cpp
  occt/geell3d.cpp
  occt/geent2d.cpp
  occt/geent3d.cpp
  occt/geextc2d.cpp
  occt/geextc3d.cpp
  occt/geextsf.cpp
  occt/gegbl.cpp
  occt/gegblge.cpp
  occt/geintrvl.cpp
  occt/gekvec.cpp
  occt/gelent2d.cpp
  occt/gelent3d.cpp
  occt/gelibver.cpp
  occt/geline2d.cpp
  occt/geline3d.cpp
  occt/gelnsg2d.cpp
  occt/gelnsg3d.cpp
  occt/gemat2d.cpp
  occt/gemat3d.cpp
  occt/genurb2d.cpp
  occt/genurb3d.cpp
  occt/genurbsf.cpp
  occt/geoffc2d.cpp
  occt/geoffc3d.cpp
  occt/geoffsf.cpp
  occt/gepent2d.cpp
  occt/gepent3d.cpp
  occt/geplanar.cpp
  occt/geplane.cpp
  occt/geplin2d.cpp
  occt/geplin3d.cpp
  occt/gepnt2d.cpp
  occt/gepnt3d.cpp
  occt/geponc2d.cpp
  occt/geponc3d.cpp
  occt/geponsrf.cpp
  occt/gepos2d.cpp
  occt/gepos3d.cpp
  occt/geray2d.cpp
  occt/geray3d.cpp
  occt/gescl2d.cpp
  occt/gescl3d.cpp
  occt/gesent2d.cpp
  occt/gesent3d.cpp
  occt/gesphere.cpp
  occt/gessint.cpp
  occt/gesurf.cpp
  occt/getol.cpp
  occt/getorus.cpp
  occt/gevec2d.cpp
  occt/gevec3d.cpp
  occt/gexbndsf.cpp
  occt/heap.cpp
  occt/nvheapmanager.cpp
)

add_library (NvBREP ${NV_SOURCES} ${NV_HEADERS})
add_library (NvBREP::NvBREP ALIAS NvBREP)

if (WIN32)
  target_compile_definitions (NvBREP PRIVATE
  NVBREP_DLL_EXPORTS
  NVGE_INTERNAL)     # ge library compiled from inside: dllexport instead of dllimport
  target_compile_definitions (NvBREP PRIVATE ACGX_INTERNAL)  # GX_-prefixed exports (gegx classes)
  target_compile_definitions (NvBREP PRIVATE NVBASE_API)     # NvString/NvBase exports
endif()

target_include_directories (NvBREP
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  # private implementation backbone of the ge layer (geimpdata.h); never installed
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/occt>)

# OCCT toolkits used directly by NvBREP:
# - TKernel / TKMath  : gp, Bnd, math primitives (FoundationClasses)
# - TKBRep            : TopoDS, TopExp (ModelingData)
# - TKTopAlgo         : BRepBuilderAPI, BRepCheck, BRepGProp, BRepBndLib (ModelingAlgorithms)
# - TKPrim            : BRepPrimAPI (ModelingAlgorithms)
# - TKBO              : BRepAlgoAPI (ModelingAlgorithms)
target_link_libraries (NvBREP PUBLIC TKernel TKMath TKBRep TKTopAlgo TKPrim TKBO)

if (BUILD_USE_PCH AND NOT CMAKE_DISABLE_PRECOMPILE_HEADERS)
  target_precompile_headers (NvBREP PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/Nv.h>")
endif()

set_target_properties (NvBREP PROPERTIES
  FOLDER    "Nv"
  VERSION   ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

install (TARGETS NvBREP
         EXPORT NvBREPTargets
         RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
         LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
         ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
         INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

# public headers + configured version header
install (DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
         DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
         FILES_MATCHING PATTERN "*.h")

install (FILES "${CMAKE_BINARY_DIR}/include/NvVersion.h"
         DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
