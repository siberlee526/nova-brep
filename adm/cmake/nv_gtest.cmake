##
# Google Test resolution for NvBREP (mirrors OCCT approach):
#  - vcpkg manifest (BUILD_USE_VCPKG=ON) provides GTest via the vcpkg toolchain;
#  - otherwise an installed GTest is used when available;
#  - as a last resort Google Test is fetched via FetchContent.
##

macro (NV_RESOLVE_GTEST)
  set (NV_GTEST_FOUND OFF)

  if (BUILD_USE_VCPKG)
    find_package (GTest CONFIG REQUIRED)
    set (NV_GTEST_FOUND ON)
  else()
    find_package (GTest CONFIG QUIET)
    if (GTest_FOUND)
      set (NV_GTEST_FOUND ON)
      message (STATUS "NvBREP uses installed Google Test.")
    else()
      include (FetchContent)
      FetchContent_Declare (googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0)
      set (INSTALL_GTEST OFF CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable (googletest)
      set (NV_GTEST_FOUND ON)
      message (STATUS "NvBREP uses Google Test fetched via FetchContent.")
    endif()
  endif()
endmacro()
