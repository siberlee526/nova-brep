# Nova BREP

NvBREP is a cross-platform C++ library that encapsulates [Open CASCADE Technology (OCCT)](https://dev.opencascade.org)
algorithms and exposes them through higher-level, value-semantic abstractions:

- **`NvShape`** â€?a null-safe, value-semantic wrapper over `TopoDS_Shape` with mass properties
  (volume, area), exact bounding box, sub-shape counting, and non-mutating transformations
  (translate / rotate / scale / mirror).
- **`NvPrim`** â€?a factory for parametric primitives (box, cylinder, sphere, cone, torus) that
  validates inputs and checks `IsDone()` so returned shapes are always well-formed.
- **`NvBoolean`** â€?fuse / cut / common operations with full error checking; every failure
  (invalid input or a raised `Standard_Failure`) is translated into a single `NvException` type.

The build system follows the OCCT project conventions (CMake + Ninja, `BUILD_CPP_STANDARD`,
`BUILD_OPT_PROFILE`, `BUILD_USE_PCH`, `BUILD_GTEST`, vcpkg integration for third-party
dependencies, `Package_Class` naming, `//====` method separators, GTest-based unit tests).

## Prerequisites

- **CMake 3.21+**
- **Ninja** (for the Ninja presets) â€?on Windows available in a Visual Studio developer prompt
  or standalone on `PATH`
- **Compiler**: MSVC 2022 (cl.exe), GCC, or Clang with C++17 support
- **OCCT 8.1+** built and installed as a sibling checkout of this repository, e.g.
  `../OCCT/install` (Release) and `../OCCT/install-debug` (Debug). The CMake package must
  be available (`<install>/cmake/OpenCASCADEConfig.cmake`).
- **vcpkg** (optional, Windows presets): `VCPKG_ROOT` environment variable pointing to a vcpkg
  checkout â€?used to install Google Test (`gtest` dependency in `vcpkg.json`).

## Building

### Windows (Ninja, recommended)

From a *VS x64 developer prompt* (or any environment with `cl.exe` and `ninja` on `PATH`):

```bat
cmake --preset win-release
cmake --build --preset win-release
ctest --preset win-release
```

Presets:

| Preset       | Generator     | Build type | OCCT   | Notes                              |
|--------------|---------------|------------|--------|------------------------------------|
| `win-release`| Ninja         | Release    | Release| Production profile, PCH, vcpkg     |
| `win-debug`  | Ninja         | Debug      | Debug  | links the Debug OCCT build         |
| `win-msvc`   | VS 17 2022    | multi-config | Release | IDE-oriented                    |
| `release` / `debug` | Ninja   | Release/Debug | â€?   | portable; pass `-DOpenCASCADE_DIR=...` |

OCCT is located through (in priority order): the `OpenCASCADE_DIR` cache variable (the Windows
presets set it to the sibling `../OCCT` checkout), the `OpenCASCADE_DIR` environment variable,
the `OCCT_ROOT` environment variable (`$OCCT_ROOT/install/cmake`), or the default sibling
checkout `../OCCT` relative to the project root.

### Linux / macOS

```sh
cmake --preset release -DOpenCASCADE_DIR=/opt/occt/lib/cmake/opencascade
cmake --build --preset release
ctest --preset release
```

Without vcpkg, Google Test is resolved from the system (or fetched via FetchContent when network
is available). Disable tests with `-DBUILD_GTEST=OFF`.

### Installing

```bat
cmake --build --preset win-release --target install
```

Installs headers to `install/include`, the library to `install/lib`, and the CMake package to
`install/lib/cmake/NvBREP`, consumable by downstream projects:

```cmake
find_package(NvBREP 0.1 REQUIRED)
target_link_libraries(app PRIVATE NvBREP::NvBREP)
```

## Project layout

```
adm/cmake/      build modules (compiler flags ported from OCCT, GTest resolution)
adm/templates/  CMake package templates and configured version header
src/include/    public API headers (Nv.h umbrella, Nv*.h â€?pure PascalCase, no underscores)
src/occt/       implementation: the OCCT algorithm encapsulation layer (.cpp)
test/           GTest unit tests (run by CTest)
script/         environment setup scripts (env.bat / env.sh)
docs/           documentation
content/        static resources / sample data
```

## Code conventions

The mandatory C++ coding standard for this project lives in
[`.opencode/rules/cpp-rules.md`](.opencode/rules/cpp-rules.md) (injected into every opencode
session via `opencode.json`). In short:

- `Package_Class` naming (`NvShape`), `theParam` / `aLocal` / `myField` naming inside code
- Native C++ types (`double`, `bool`, `int`) instead of deprecated `Standard_*` typedefs
- `//!` Doxygen comments in headers; `//====...====` (99-column) separators in sources
- `occ::handle<>` for reference-counted OCCT objects; `IsDone()` checks after algorithms
- `TopoDS::Face(...)`-style safe casts; `TopExp_Explorer(shape, type)` constructor loops

Format code with the bundled `.clang-format` (same as OCCT).
