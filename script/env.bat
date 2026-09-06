@echo off
:: NvBREP development environment (Windows).
:: Usage: script\env.bat [OCCT root]
:: Adds the NvBREP build output and the OCCT runtime libraries to PATH.
:: The default OCCT location is the sibling checkout ..\..\OCCT
:: (relative to this script, i.e. ../OCCT relative to the project root).

rem Normalize paths (resolve relative ..-segments) for reliable PATH entries
if "%NV_ROOT%"=="" for %%i in ("%~dp0..") do set "NV_ROOT=%%~fi"

if not "%~1"=="" (
  set "OCCT_ROOT=%~1"
) else (
  if "%OCCT_ROOT%"=="" for %%i in ("%~dp0..\..\OCCT") do set "OCCT_ROOT=%%~fi"
)

set "PATH=%NV_ROOT%\build\bin;%OCCT_ROOT%\install\win64\vc14\bin;%PATH%"

echo NvBREP root : %NV_ROOT%
echo OCCT root     : %OCCT_ROOT%
echo NvBREP bin  : %NV_ROOT%\build\bin
echo OCCT bin      : %OCCT_ROOT%\install\win64\vc14\bin
