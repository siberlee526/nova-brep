##
# Compiler flags and definitions for NvBREP.
# Ported from OCCT adm/cmake/occt_defs_flags.cmake (sibling ../OCCT checkout)
# so that NvBREP is compiled with the same floating-point model,
# exception model, warning level and optimization profile as OCCT itself.
##

if(NV_FLAGS_ALREADY_INCLUDED)
  return()
endif()
set(NV_FLAGS_ALREADY_INCLUDED 1)

# force option /fp:precise for Visual Studio projects.
#
# Note that while this option is default for MSVC compiler, Visual Studio
# project can be switched later to use Intel Compiler (ICC).
# Enforcing /fp:precise ensures that in such case ICC will use correct
# option instead of its default /fp:fast which is harmful for CAD algorithms.
if (MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /fp:precise")

  # suppress C26812 on VS2019/C++20 (prefer 'enum class' over 'enum')
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd26812")
  # increase number of sections in object files (needed for heavy template code
  # in unit tests built without optimizations, e.g. Debug configuration)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
  # suppress warning on using portable non-secure functions in favor of non-portable secure ones
  add_definitions (-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE)
endif()

# enable asynchronous (structured) exceptions for MSVC, as expected by OCCT
string (REGEX MATCH "EHsc" ISFLAG_EHSC "${CMAKE_CXX_FLAGS}")
if (ISFLAG_EHSC)
  string (REGEX REPLACE "EHsc" "EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
elseif (MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHa")
endif()

# enable parallel compilation on MSVC 9 and above
if (MSVC AND (MSVC_VERSION GREATER 1400))
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

# increase compiler warnings level (/W4 for MSVC, -Wall -Wextra for GCC/Clang)
if (MSVC)
  if (CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string (REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
elseif (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "[Cc][Ll][Aa][Nn][Gg]"))
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -Wextra")

  # OCCT algorithms throw C++ exceptions; always keep them enabled
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fexceptions")

  if (NOT CMAKE_CXX_SIMULATE_ID STREQUAL "MSVC")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fPIC")
  endif()

  if (CMAKE_CXX_COMPILER_ID MATCHES "[Cc][Ll][Aa][Nn][Gg]")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshorten-64-to-32")
  endif()

  if (APPLE)
    # Suppress elaborated-enum-base warnings from Apple system headers
    # when using newer Clang versions (LLVM 18+)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-elaborated-enum-base")
    set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wno-elaborated-enum-base")
  endif()
endif()

# optimization profile (mirrors OCCT BUILD_OPT_PROFILE)
if ("${BUILD_OPT_PROFILE}" STREQUAL "Production")
  if (MSVC)
    # string pooling (GF), function-level linking (Gy)
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GF /Gy")
    set (CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   /GF /Gy")

    # Favor fast code (Ot), omit frame pointers (Oy)
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /Oy")
    set (CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   /Ot /Oy")

    # Whole Program Optimisation (GL), enable intrinsic functions (Oi)
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL /Oi")
    set (CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   /GL /Oi")

    # Link-Time Code Generation (LTCG) is required for Whole Program Optimisation (GL)
    set (CMAKE_EXE_LINKER_FLAGS_RELEASE    "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    set (CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG")
    set (CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} /LTCG")
    set (CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} /LTCG")
  elseif (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR (CMAKE_CXX_COMPILER_ID MATCHES "[Cc][Ll][Aa][Nn][Gg]"))
    # /Ot (favor speed over size) is similar to -O3; /Oy to -fomit-frame-pointer;
    # /GL (whole program optimization) is similar to -flto (Link Time Optimization);
    # /Gy (function-level linking) is similar to -ffunction-sections.
    set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -flto")
    set (CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -O3 -fomit-frame-pointer -flto")

    # Apply function sections only on non-macOS platforms
    if (NOT APPLE)
      set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ffunction-sections")
      set (CMAKE_C_FLAGS_RELEASE   "${CMAKE_C_FLAGS_RELEASE}   -ffunction-sections")
    endif()

    set (CMAKE_EXE_LINKER_FLAGS_RELEASE    "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")
    set (CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -flto")
    set (CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS_RELEASE} -flto")
    set (CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} -flto")

    # Garbage-collect unused sections on Linux
    if (NOT WIN32 AND NOT APPLE)
      set (CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -Wl,--gc-sections")
      set (CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} -Wl,--gc-sections")
      set (CMAKE_EXE_LINKER_FLAGS_RELEASE    "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wl,--gc-sections")
    endif()
  endif()
endif()

if (MINGW)
  add_definitions (-D_WIN32_WINNT=0x0601)
  # workaround bugs in mingw with vtable export
  set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--export-all-symbols")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wattributes")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wattributes")
endif()

# prevent Windows.h from redefining std::min/std::max
if (WIN32)
  add_definitions (-DNOMINMAX)
endif()
