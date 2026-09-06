# Define NvBREP root directory as cache variable - works for both standalone and subproject builds
set (NV_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Root directory of NvBREP project" FORCE)

# vcpkg bootstrap (mirrors OCCT BUILD_USE_VCPKG logic; must happen before PROJECT()
# so that the vcpkg toolchain installs manifest dependencies at configure time)
if (NOT DEFINED BUILD_USE_VCPKG)
  set (BUILD_USE_VCPKG OFF CACHE BOOL "Use vcpkg for 3rd-party dependencies (Google Test).")
  if (CMAKE_TOOLCHAIN_FILE MATCHES "vcpkg.cmake")
    set (BUILD_USE_VCPKG ON)
  endif()
endif()

if (BUILD_USE_VCPKG)
  # The VCPKG_ROOT environment variable is authoritative and overrides any cached or
  # IDE-injected value (e.g. the vcpkg instance embedded in Visual Studio, whose git
  # history does not necessarily contain the manifest baseline).
  if (DEFINED ENV{VCPKG_ROOT} AND EXISTS "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set (VCPKG_ROOT "$ENV{VCPKG_ROOT}" CACHE PATH "Path to vcpkg root directory." FORCE)
  endif()

  if (NOT "${CMAKE_TOOLCHAIN_FILE}" STREQUAL "" AND EXISTS "${CMAKE_TOOLCHAIN_FILE}")
    if (NOT "${CMAKE_TOOLCHAIN_FILE}" MATCHES "vcpkg.cmake")
      set (VCPKG_CHAINLOAD_TOOLCHAIN_FILE ${CMAKE_TOOLCHAIN_FILE})
      set (CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
    endif()
  elseif (EXISTS "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set (CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
  else()
    message (FATAL_ERROR "BUILD_USE_VCPKG is ON but VCPKG_ROOT is not defined or invalid. "
                         "Set VCPKG_ROOT to the vcpkg root directory, or pass -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake.")
  endif()
endif()

cmake_minimum_required (VERSION 3.21 FATAL_ERROR)

project (NvBREP VERSION 0.1.0 LANGUAGES CXX)

set (CMAKE_MODULE_PATH "${NV_ROOT_DIR}/adm/cmake")

# installation layout (needed by src/ and test/ subdirectories)
include (GNUInstallDirs)

# Solution folder property (mirrors OCCT)
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

set (CMAKE_CONFIGURATION_TYPES Release Debug RelWithDebInfo CACHE INTERNAL "" FORCE)

# set using C++ standard (mirrors OCCT BUILD_CPP_STANDARD)
set (BUILD_CPP_STANDARD "C++17" CACHE STRING "Select using C++ standard.")
set_property (CACHE BUILD_CPP_STANDARD PROPERTY STRINGS "C++17" "C++20" "C++23" "C++26")

# Set desired C++ standard
if ("${BUILD_CPP_STANDARD}" STREQUAL "C++17")
  set (CMAKE_CXX_STANDARD 17)
elseif ("${BUILD_CPP_STANDARD}" STREQUAL "C++20")
  set (CMAKE_CXX_STANDARD 20)
elseif ("${BUILD_CPP_STANDARD}" STREQUAL "C++23")
  set (CMAKE_CXX_STANDARD 23)
elseif ("${BUILD_CPP_STANDARD}" STREQUAL "C++26")
  set (CMAKE_CXX_STANDARD 26)
else()
  message (WARNING "C++ standard is not set or invalid. Set to C++17.")
  set (CMAKE_CXX_STANDARD 17)
endif()

set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# set type of NvBREP library (mirrors OCCT BUILD_LIBRARY_TYPE)
if (NOT BUILD_LIBRARY_TYPE)
  set (BUILD_LIBRARY_TYPE "Shared" CACHE STRING "Type of NvBREP library (Shared or Static)." FORCE)
  set_property (CACHE BUILD_LIBRARY_TYPE PROPERTY STRINGS Shared Static)
endif()

if ("${BUILD_LIBRARY_TYPE}" STREQUAL "Shared")
  set (BUILD_SHARED_LIBS ON)
else()
  unset (BUILD_SHARED_LIBS)
endif()

# set profile for C++ compiler and linker (mirrors OCCT BUILD_OPT_PROFILE)
set (BUILD_OPT_PROFILE "Default" CACHE STRING "Select profile for compiler and linker.")
set_property (CACHE BUILD_OPT_PROFILE PROPERTY STRINGS "Default" "Production")

# option to enable or disable use of precompiled headers
set (BUILD_USE_PCH OFF CACHE BOOL "Use precompiled headers.")
if (CMAKE_VERSION VERSION_LESS "3.16")
  set (BUILD_USE_PCH OFF)
endif()

# option to build unit tests (mirrors OCCT BUILD_GTEST)
set (BUILD_GTEST ON CACHE BOOL "Enable building unit tests using GTest.")

# a single-configuration generator like Ninja defines CMAKE_BUILD_TYPE variable;
# check this variable and set it if it's required
if (DEFINED CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# Unicode build on Windows (mirrors OCCT)
if (WIN32)
  add_definitions (-DUNICODE -D_UNICODE)
endif()

# compiler flags and definitions (ported from OCCT adm/cmake/occt_defs_flags.cmake)
include (nv_defs_flags)

# output directories (bin/ and lib/ at the top of the build tree).
# Note: multi-configuration generators (VS, Xcode) append a per-configuration
# subdirectory automatically, so all targets stay colocated in bin/<config>/.
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# locate OCCT installation
# - OpenCASCADE_DIR cache variable has the highest priority (also set by CMakePresets);
# - OpenCASCADE_DIR environment variable is honored next;
# - OCCT_ROOT environment variable is used next ($OCCT_ROOT/install/cmake);
# - the default fallback expects OCCT as a sibling checkout (../OCCT relative to
#   the NvBREP project root).
if (NOT DEFINED OpenCASCADE_DIR)
  if (DEFINED ENV{OpenCASCADE_DIR})
    set (OpenCASCADE_DIR "$ENV{OpenCASCADE_DIR}" CACHE PATH "The directory containing OpenCASCADEConfig.cmake.")
  elseif (DEFINED ENV{OCCT_ROOT} AND EXISTS "$ENV{OCCT_ROOT}/install/cmake")
    set (OpenCASCADE_DIR "$ENV{OCCT_ROOT}/install/cmake" CACHE PATH "The directory containing OpenCASCADEConfig.cmake.")
  elseif (EXISTS "${NV_ROOT_DIR}/../OCCT/install/cmake")
    set (OpenCASCADE_DIR "${NV_ROOT_DIR}/../OCCT/install/cmake" CACHE PATH "The directory containing OpenCASCADEConfig.cmake.")
  endif()
endif()

find_package (OpenCASCADE REQUIRED COMPONENTS FoundationClasses ModelingData ModelingAlgorithms)

message (STATUS "NvBREP ${PROJECT_VERSION} uses OpenCASCADE ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION} "
                "from ${OpenCASCADE_INSTALL_PREFIX}")

# configured version header
configure_file ("${NV_ROOT_DIR}/adm/templates/NvVersion.h.in" "${CMAKE_BINARY_DIR}/include/NvVersion.h" @ONLY)

# library
add_subdirectory (src)

# unit tests
if (BUILD_GTEST)
  include (nv_gtest)
  NV_RESOLVE_GTEST()
  if (NV_GTEST_FOUND)
    enable_testing()
    add_subdirectory (test)
  else()
    message (STATUS "Google Test is not available - unit tests are skipped.")
  endif()
endif()

# installation and export
include (CMakePackageConfigHelpers)

set (NV_INSTALL_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/NvBREP")

install (EXPORT NvBREPTargets
         FILE NvBREPTargets.cmake
         NAMESPACE NvBREP::
         DESTINATION "${NV_INSTALL_CMAKEDIR}")

configure_package_config_file (
  "${NV_ROOT_DIR}/adm/templates/NvBREPConfig.cmake.in"
  "${CMAKE_BINARY_DIR}/NvBREPConfig.cmake"
  INSTALL_DESTINATION "${NV_INSTALL_CMAKEDIR}")

write_basic_package_version_file (
  "${CMAKE_BINARY_DIR}/NvBREPConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install (FILES "${CMAKE_BINARY_DIR}/NvBREPConfig.cmake"
               "${CMAKE_BINARY_DIR}/NvBREPConfigVersion.cmake"
         DESTINATION "${NV_INSTALL_CMAKEDIR}")
