# NvBREP unit tests (Google Test, mirrors OCCT GTest conventions)

set (NV_TEST_SOURCES
  NvBooleanTest.cpp
  NvGeBoundBlock2dTest.cpp
  NvGeBoundBlock3dTest.cpp
  NvGeCircArc2dTest.cpp
  NvGeCircArc3dTest.cpp
  NvGeClipBoundary2dTest.cpp
  NvGeConeTest.cpp
  NvGeCubicSpline3dTest.cpp
  NvGeCurve2dTest.cpp
  NvGeCurve3dTest.cpp
  NvGeCurveCurveInt2dTest.cpp
  NvGeCurveCurveInt3dTest.cpp
  NvGeCylinderTest.cpp
  NvGeEllipArc2dTest.cpp
  NvGeKnotVectorTest.cpp
  NvGeLine2dTest.cpp
  NvGeLine3dTest.cpp
  NvGeMatrix2dTest.cpp
  NvGeNurbCurve2dTest.cpp
  NvGeNurbSurfaceTest.cpp
  NvGePoint2dTest.cpp
  NvGePoint3dTest.cpp
  NvGePointOnCurve2dTest.cpp
  NvGePolyline2dTest.cpp
  NvGePolyline3dTest.cpp
  NvGePosition2dTest.cpp
  NvGePosition3dTest.cpp
  NvGeSphereTest.cpp
  NvGeTorusTest.cpp
  NvGeVector2dTest.cpp
  NvGeVector3dTest.cpp
  NvPrimTest.cpp
  NvShapeTest.cpp
  NvStringTest.cpp)

add_executable (NvBREPTest ${NV_TEST_SOURCES})

target_link_libraries (NvBREPTest PRIVATE NvBREP GTest::gtest_main)

set_target_properties (NvBREPTest PROPERTIES FOLDER "Testing")

# Windows: deploy dependent runtime DLLs next to the test executable,
# so that both GTest discovery (ctest) and test execution find them without any
# environment setup.
# - $<TARGET_RUNTIME_DLLS> covers the linked OCCT toolkits (TK*.dll) and vcpkg DLLs;
# - OCCT toolkits have transitive non-CMake dependencies (e.g. TKernel needs TBB),
#   which are taken from the OCCT install bin directory. Following the OCCT
#   convention, debug binaries live in the sibling "bind" directory and use
#   _debug-suffixed names for TBB.
if (WIN32)
  set (NV_OCCT_RUNTIME_DIR "${OpenCASCADE_BINARY_DIR}")
  if (CMAKE_BUILD_TYPE STREQUAL "Debug" AND EXISTS "${OpenCASCADE_BINARY_DIR}d")
    set (NV_OCCT_RUNTIME_DIR "${OpenCASCADE_BINARY_DIR}d")
  endif()

  set (NV_OCCT_TRANSITIVE_DLLS)
  foreach (aDll tbb12 tbbmalloc tbbmalloc_proxy)
    if (CMAKE_BUILD_TYPE STREQUAL "Debug" AND EXISTS "${NV_OCCT_RUNTIME_DIR}/${aDll}_debug.dll")
      list (APPEND NV_OCCT_TRANSITIVE_DLLS "${NV_OCCT_RUNTIME_DIR}/${aDll}_debug.dll")
    elseif (EXISTS "${NV_OCCT_RUNTIME_DIR}/${aDll}.dll")
      list (APPEND NV_OCCT_TRANSITIVE_DLLS "${NV_OCCT_RUNTIME_DIR}/${aDll}.dll")
    endif()
  endforeach()

  add_custom_command (TARGET NvBREPTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:NvBREPTest>
            $<TARGET_FILE_DIR:NvBREPTest>
    COMMAND_EXPAND_LISTS)
  if (NV_OCCT_TRANSITIVE_DLLS)
    add_custom_command (TARGET NvBREPTest POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${NV_OCCT_TRANSITIVE_DLLS}
              $<TARGET_FILE_DIR:NvBREPTest>)
  endif()
endif()

include (GoogleTest)

# PRE_TEST discovery registers tests at run time, which keeps the list in sync
# with the actual binary (important for multi-configuration generators).
gtest_discover_tests (NvBREPTest
  TEST_PREFIX    "Nv::"
  DISCOVERY_MODE PRE_TEST
  PROPERTIES
    ENVIRONMENT "PATH=$<TARGET_FILE_DIR:NvBREPTest>;${OpenCASCADE_BINARY_DIR};$ENV{PATH}")
