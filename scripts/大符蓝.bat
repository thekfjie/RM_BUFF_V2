@echo off
setlocal
cd /d "%~dp0.."
set "PATH=D:\develop\opencv\build\x64\vc16\bin;D:\develop\onnxruntime\lib;%PATH%"
set "ROOT=E:\RM\rm_vision"
set "PARAM=%ROOT%\examples\example_for_prediction\6_dark_blue_big\parameter.yaml"
set "EXE=%~dp0..\build\vs2022-release\Release\predict_example_main.exe"

if not exist "%EXE%" call "%~dp0build_release.bat"
if errorlevel 1 goto :fail
if not exist "%EXE%" goto :fail
if not exist "%PARAM%" goto :missing_param

"%EXE%" --python-root "%ROOT%" --parameter "%PARAM%" --mode big --color blue
set "EXITCODE=%ERRORLEVEL%"
if "%EXITCODE%"=="0" exit /b 0

echo.
echo Program exited with code %EXITCODE%.
pause
exit /b %EXITCODE%

:missing_param
echo.
echo Missing parameter file:
echo %PARAM%
pause
exit /b 1

:fail
echo.
echo Build failed or executable is missing.
pause
exit /b 1
